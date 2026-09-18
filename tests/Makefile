# =============================================================================
#  Unit test build for the Automotive Body Control ECU
#
#  These tests run on the HOST (your PC), not on the microcontroller.
#
#  That is possible because the modules under test are pure: no STM32 HAL, no
#  register access, no hardware. They are ordinary C that happens to be
#  deployed to an ECU, so an ordinary C compiler can build and run them.
#
#  Deliberately NOT tested here: gpio_driver, adc_driver.c, pwm_driver,
#  uart_driver, can_driver, watchdog_driver. Those exist to touch registers,
#  and testing them without hardware would mean mocking the entire HAL - a
#  large amount of work that would mostly prove the mocks match the mocks.
#  They are verified on the bench instead, and the test plan says so.
#
#  Usage:
#      make            build and run every suite
#      make build      compile only
#      make clean      remove build artefacts
#      make <suite>    run one suite, e.g. make test_can_signals
#
#  Requires: gcc (or clang) and make.
#    Linux/macOS  - already present, or `sudo apt install build-essential`
#    Windows      - MSYS2 (`pacman -S mingw-w64-x86_64-gcc make`), or WSL,
#                   or simply let the GitHub Actions CI run them on push.
# =============================================================================

CC      ?= gcc
BUILD   := build

# Source tree being tested
FW      := ../firmware/automotive-body-ecu/src

INCLUDES := \
	-Iframework \
	-I$(FW)/app \
	-I$(FW)/services \
	-I$(FW)/drivers \
	-I$(FW)/config

# -Wall -Wextra          catch the common mistakes
# -Werror                a warning in test code is a defect like any other
# -std=c11 -pedantic     no compiler-specific extensions in portable modules
# -g -O0                 debuggable, and no optimisation to confuse a stack trace
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -g -O0 $(INCLUDES)

# Shared framework object, linked into every suite
FRAMEWORK := framework/unity_min.c

# -----------------------------------------------------------------------------
#  Each suite is its own executable, as Ceedling does. One suite's setUp and
#  tearDown cannot collide with another's, and a crash in one cannot stop the
#  others from reporting.
# -----------------------------------------------------------------------------

SUITES := test_vehicle_state_machine \
          test_fault_manager \
          test_can_signals \
          test_adc_conversion

# Production sources each suite needs. The test links the REAL module - never
# a copy - so the tests can only pass if the shipped code is correct.
SRC_test_vehicle_state_machine := $(FW)/app/vehicle_state_machine.c
SRC_test_fault_manager         := $(FW)/services/fault_manager.c
SRC_test_can_signals           := $(FW)/services/can_signals.c
SRC_test_adc_conversion        :=
# (the ADC conversion under test is a pure inline function in the header,
#  so no .c file is required)

EXES := $(addprefix $(BUILD)/,$(SUITES))

.PHONY: all build clean test $(SUITES)

all: test

build: $(EXES)

$(BUILD):
	@mkdir -p $(BUILD)

# Static pattern rule: build each suite from its test file, its production
# source, and the framework.
$(BUILD)/%: %.c $(FRAMEWORK) | $(BUILD)
	@echo "  CC    $@"
	@$(CC) $(CFLAGS) -o $@ $< $(SRC_$*) $(FRAMEWORK)

test: $(EXES)
	@echo ""
	@echo "########################################################"
	@echo "#  Running unit test suites"
	@echo "########################################################"
	@failed=0; \
	for exe in $(EXES); do \
		./$$exe || failed=1; \
	done; \
	echo ""; \
	if [ $$failed -eq 0 ]; then \
		echo "  >>> ALL SUITES PASSED"; \
	else \
		echo "  >>> ONE OR MORE SUITES FAILED"; \
	fi; \
	exit $$failed

# Run a single suite by name, e.g. `make test_can_signals`
$(SUITES): %: $(BUILD)/%
	@./$<

clean:
	@rm -rf $(BUILD)
	@echo "  clean"
