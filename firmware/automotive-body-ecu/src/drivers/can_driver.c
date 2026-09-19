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

/* =========================================================================
 * Receive queue: interrupt (producer) -> main loop (consumer)
 *
 * The bxCAN hardware FIFO holds only 3 frames. A diagnostic tester sending a
 * multi-frame request can deliver them faster than a 5 ms task polls, so
 * reception is interrupt-driven: the ISR moves every frame into this larger
 * software queue immediately, and the main loop drains it at its own pace.
 *
 * This is a single-producer / single-consumer ring buffer, and it needs NO
 * lock, because each index has exactly one writer:
 *
 *     s_rx_head  written only by the ISR       (where the next frame goes)
 *     s_rx_tail  written only by the main loop (where the next read comes from)
 *
 *          tail              head
 *           v                 v
 *     [ .. | F1 | F2 | F3 |   |   | .. ]     empty when head == tail
 *                                           full  when head + 1 == tail
 *
 * Both indices are `volatile` because each is changed by code the compiler
 * cannot see from the other context. Without it, the main loop could keep
 * s_rx_head in a register and never notice a new frame.
 *
 * `volatile` is not enough on its own, though. The compiler may still move an
 * ordinary write (copying the frame into the slot) AFTER the volatile write
 * that publishes it (advancing head). __DMB() is a memory barrier with a
 * compiler "memory" clobber: nothing is reordered across it, so the frame is
 * always complete before head says it exists.
 * ========================================================================= */

#define RX_QUEUE_MASK   (ECU_CAN_RX_QUEUE_SIZE - 1U)

#if ((ECU_CAN_RX_QUEUE_SIZE & RX_QUEUE_MASK) != 0U)
#error "ECU_CAN_RX_QUEUE_SIZE must be a power of two"
#endif

static CanFrame_t        s_rx_queue[ECU_CAN_RX_QUEUE_SIZE];
static volatile uint32_t s_rx_head           = 0U;
static volatile uint32_t s_rx_tail           = 0U;
static volatile uint32_t s_rx_overflow_count = 0U;

/* Priority of the CAN receive interrupt. Lower number = more urgent on
 * Cortex-M. Set below SysTick's so the millisecond tick is never delayed by
 * a burst of CAN traffic. */
#define CAN_RX_IRQ_PRIORITY  5U

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

    s_rx_head           = 0U;
    s_rx_tail           = 0U;
    s_rx_overflow_count = 0U;

    /* Ask the controller to raise an interrupt whenever FIFO 0 holds a frame,
     * then enable that interrupt line in the NVIC (the Cortex-M interrupt
     * controller). Both steps are needed: the peripheral decides WHEN to
     * signal, the NVIC decides WHETHER the CPU listens. */
    if (HAL_CAN_ActivateNotification(&s_can, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, CAN_RX_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    s_tx_failure_count = 0U;
    s_initialised      = true;
    return true;
}

/* ========================================================================= */
/* Interrupt context                                                         */
/* ========================================================================= */

/**
 * @brief CAN1 FIFO 0 interrupt vector.
 *
 * The name must match the vector table entry in startup_stm32f446retx.s
 * exactly. There it is declared "weak", pointing at a default handler that
 * loops forever; defining a function with the same name here replaces it.
 * A typo in this name compiles without complaint and hangs the ECU on the
 * first received frame - which is why it is worth knowing.
 */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&s_can);
}

/**
 * @brief Called by the HAL from the interrupt above, once per event.
 *
 * Runs in interrupt context: it must be short, must never block, and must
 * never call anything that is not safe to interrupt the main loop with.
 * Here it only copies frames into the queue.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             payload[CAN_FRAME_MAX_DLC];

    /* Empty the whole hardware FIFO in one interrupt rather than taking one
     * interrupt per frame. */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, payload) != HAL_OK)
        {
            return;
        }

        /* Extended (29-bit) identifiers are not part of this project's
         * interfaces; filtering them here keeps every upper layer simpler. */
        if (header.IDE != CAN_ID_STD)
        {
            continue;
        }

        const uint32_t head = s_rx_head;
        const uint32_t next = (head + 1U) & RX_QUEUE_MASK;

        if (next == s_rx_tail)
        {
            /* Queue full: drop the NEWEST frame and count it. Overwriting the
             * oldest instead would corrupt a frame the main loop may be
             * reading right now. */
            s_rx_overflow_count++;
            continue;
        }

        CanFrame_t *slot = &s_rx_queue[head];
        const uint8_t dlc = (header.DLC > CAN_FRAME_MAX_DLC) ? CAN_FRAME_MAX_DLC
                                                             : (uint8_t)header.DLC;
        slot->id  = header.StdId;
        slot->dlc = dlc;
        (void)memset(slot->data, 0, CAN_FRAME_MAX_DLC);
        (void)memcpy(slot->data, payload, dlc);

        __DMB();            /* frame fully written BEFORE it is published */
        s_rx_head = next;
    }
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

    const uint32_t tail = s_rx_tail;

    if (tail == s_rx_head)
    {
        return false;   /* queue empty - not an error */
    }

    *frame = s_rx_queue[tail];

    __DMB();            /* finish reading the slot BEFORE handing it back */
    s_rx_tail = (tail + 1U) & RX_QUEUE_MASK;

    return true;
}

uint32_t CanDriver_GetRxOverflowCount(void)
{
    return s_rx_overflow_count;
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
