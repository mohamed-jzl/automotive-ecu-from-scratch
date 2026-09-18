/**
 * @file    can_driver.c
 * @brief   bxCAN implementation - see can_driver.h for the protocol background.
 */

#include "can_driver.h"

#include "ecu_config.h"
#include "pin_config.h"
#include "stm32f4xx_hal.h"

#include <string.h>

static CAN_HandleTypeDef s_can;
static bool              s_initialised      = false;
static uint32_t          s_tx_failure_count = 0U;

/**
 * @brief Configure the acceptance filter to receive every standard identifier.
 *
 * bxCAN filters in hardware before a frame ever reaches software. Each filter
 * bank holds an identifier and a mask; a frame is accepted when
 *
 *      (received_id & mask) == (filter_id & mask)
 *
 * Setting the mask to zero makes every bit a "don't care", so all frames pass.
 * A production ECU would set one bank per message it actually consumes,
 * because hardware filtering is free while software filtering costs CPU time
 * on every single frame present on the bus.
 */
static bool can_configure_filter(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000U;
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = 0x0000U;   /* mask 0 = accept everything */
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = CAN_FILTER_ENABLE;

    /* The 28 filter banks are shared between CAN1 and CAN2. This value splits
     * them: banks 0-13 belong to CAN1, banks 14-27 to CAN2. We only use CAN1,
     * but the field must still be set to a legal value. */
    filter.SlaveStartFilterBank = 14U;

    return (HAL_CAN_ConfigFilter(&s_can, &filter) == HAL_OK);
}

bool CanDriver_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    /* PB8 = CAN1_RX, PB9 = CAN1_TX, both on alternate function 9. */
    gpio.Pin       = PIN_CAN_RX_PIN | PIN_CAN_TX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;               /* keeps RX at a defined recessive
                                                 * level if the transceiver is
                                                 * unpowered or disconnected      */
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH; /* sharp edges preserve the sample
                                                 * point accuracy at 500 kbit/s   */
    gpio.Alternate = PIN_CAN_AF;
    HAL_GPIO_Init(PIN_CAN_RX_PORT, &gpio);

    s_can.Instance = PIN_CAN_INSTANCE;

    /* Bit timing: 42 MHz APB1 / (6 * 14 tq) = 500 kbit/s, sample point 85.7%.
     * See ECU_CAN_* in ecu_config.h for the full derivation. */
    s_can.Init.Prescaler     = ECU_CAN_PRESCALER;
    s_can.Init.SyncJumpWidth = CAN_SJW_1TQ;
    s_can.Init.TimeSeg1      = CAN_BS1_11TQ;
    s_can.Init.TimeSeg2      = CAN_BS2_2TQ;
    s_can.Init.Mode          = CAN_MODE_NORMAL;

    /* Automatic bus-off recovery: after the mandatory 128 occurrences of 11
     * consecutive recessive bits, the controller rejoins the bus by itself.
     * Suitable for a body ECU, whose messages are not safety-critical. A
     * safety function would instead stay off the bus and require a deliberate,
     * diagnosable recovery so that a hardware fault cannot be masked by an
     * endless reconnect loop. */
    s_can.Init.AutoBusOff = ENABLE;

    /* Retransmit until acknowledged. Standard for periodic status messages.
     * Disabling this gives "single shot" behaviour, used for time-critical
     * data where a stale retransmission would be worse than a lost frame. */
    s_can.Init.AutoRetransmission = ENABLE;

    /* Wake automatically on bus activity after entering sleep. */
    s_can.Init.AutoWakeUp = ENABLE;

    /* Keep the FIFO's oldest frame and drop new ones when full, so a burst
     * cannot silently discard the message we already started processing. */
    s_can.Init.ReceiveFifoLocked = DISABLE;

    /* Transmit in the order the frames were queued rather than by identifier
     * priority. Makes the transmit sequence match the code's intent, which is
     * much easier to correlate against a bus trace during debugging. */
    s_can.Init.TransmitFifoPriority = ENABLE;

    /* Time-triggered mode is for TTCAN scheduling; not used here. */
    s_can.Init.TimeTriggeredMode = DISABLE;

    if (HAL_CAN_Init(&s_can) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    if (!can_configure_filter())
    {
        s_initialised = false;
        return false;
    }

    /* Leaves initialisation mode and connects the controller to the bus. */
    if (HAL_CAN_Start(&s_can) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    s_tx_failure_count = 0U;
    s_initialised      = true;
    return true;
}

bool CanDriver_Transmit(const CanFrame_t *frame)
{
    if ((!s_initialised) || (frame == NULL) || (frame->dlc > CAN_FRAME_MAX_DLC))
    {
        return false;
    }

    /* No free mailbox means the bus is not draining our frames. Fail fast so
     * the caller can raise a diagnostic rather than stalling the task. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&s_can) == 0U)
    {
        s_tx_failure_count++;
        return false;
    }

    CAN_TxHeaderTypeDef header = {0};
    uint32_t            mailbox = 0U;

    header.StdId              = frame->id;
    header.ExtId              = 0U;
    header.IDE                = CAN_ID_STD;     /* 11-bit standard identifier */
    header.RTR                = CAN_RTR_DATA;   /* data frame, not a remote request */

    /* The STM32F4 HAL takes the data length as a plain byte count, 0..8 -
     * not a symbolic CAN_DLC_x constant. (Some other STM32 families and the
     * CAN-FD peripherals do use symbolic codes, because CAN-FD's DLC values
     * above 8 are non-linear: 9 means 12 bytes, 10 means 16, and so on.) */
    header.DLC                = frame->dlc;
    header.TransmitGlobalTime = DISABLE;

    /* The HAL signature takes a non-const payload pointer even though it only
     * reads it. The cast is confined to this line. */
    if (HAL_CAN_AddTxMessage(&s_can, &header, (uint8_t *)frame->data, &mailbox) != HAL_OK)
    {
        s_tx_failure_count++;
        return false;
    }

    return true;
}

bool CanDriver_Receive(CanFrame_t *frame)
{
    if ((!s_initialised) || (frame == NULL))
    {
        return false;
    }

    if (HAL_CAN_GetRxFifoFillLevel(&s_can, CAN_RX_FIFO0) == 0U)
    {
        return false;   /* nothing waiting - not an error */
    }

    CAN_RxHeaderTypeDef header = {0};
    uint8_t             payload[CAN_FRAME_MAX_DLC] = {0};

    if (HAL_CAN_GetRxMessage(&s_can, CAN_RX_FIFO0, &header, payload) != HAL_OK)
    {
        return false;
    }

    /* Extended (29-bit) identifiers are outside this project's message set.
     * Discarding them here keeps every layer above free of the distinction. */
    if (header.IDE != CAN_ID_STD)
    {
        return false;
    }

    frame->id  = header.StdId;
    frame->dlc = (uint8_t)header.DLC;

    if (frame->dlc > CAN_FRAME_MAX_DLC)
    {
        frame->dlc = CAN_FRAME_MAX_DLC;
    }

    (void)memcpy(frame->data, payload, frame->dlc);

    /* Zero the unused tail so a short frame never exposes stale bytes from a
     * previous, longer message. */
    if (frame->dlc < CAN_FRAME_MAX_DLC)
    {
        (void)memset(&frame->data[frame->dlc], 0, CAN_FRAME_MAX_DLC - frame->dlc);
    }

    return true;
}

bool CanDriver_IsBusOff(void)
{
    if (!s_initialised)
    {
        return false;
    }

    return ((HAL_CAN_GetError(&s_can) & HAL_CAN_ERROR_BOF) != 0U);
}

uint32_t CanDriver_GetTxFailureCount(void)
{
    return s_tx_failure_count;
}
