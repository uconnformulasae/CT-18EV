# Host tests for the HAL-independent firmware modules.
#
#   make          build and run everything
#   make coverage line and branch coverage (needs clang and the LLVM tools)
#   make clean    remove the binaries
#
# These run on the build machine, not the target, and are deliberately kept
# out of the STM32CubeIDE build.

CC      ?= cc
CFLAGS  ?= -O1 -g -Wall -Wextra -I../Core/Inc
LDLIBS   = -lm

TESTS = lc_test can_tx_test rtd_test adc_test regen_test regen_off_test \
        control_test version_test bms_limits_test

.PHONY: all test coverage clean
all: test

test: $(TESTS)
	@for t in $(TESTS); do \
		echo "== $$t"; \
		./$$t || exit 1; \
	done

# launch_control has no HAL dependency, so it needs no shim.
lc_test: lc_test.c ../Core/Src/launch_control.c ../Core/Inc/launch_control.h
	$(CC) $(CFLAGS) -o $@ lc_test.c $(LDLIBS)

# adc reaches the HAL for the converters; ADC_HOST lets the test act as the DMA.
adc_test: adc_test.c ../Core/Src/adc.c ../Core/Inc/adc.h
	$(CC) $(CFLAGS) -DADC_HOST -o $@ adc_test.c $(LDLIBS)

# rtd takes elapsed time from the caller, so it needs no shim.
rtd_test: rtd_test.c ../Core/Src/rtd.c ../Core/Inc/rtd.h
	$(CC) $(CFLAGS) -o $@ rtd_test.c $(LDLIBS)

# can_tx reaches the HAL for the mailboxes; CAN_TX_HOST swaps in a fake one.
can_tx_test: can_tx_test.c ../Core/Src/can_tx.c ../Core/Inc/can_tx.h
	$(CC) $(CFLAGS) -DCAN_TX_HOST -o $@ can_tx_test.c $(LDLIBS)

# regen_test also runs bms_limits, whose critical sections BMS_LIMITS_HOST stubs.
regen_test: regen_test.c ../Core/Src/regen.c ../Core/Inc/regen.h ../Core/Inc/config.h ../Core/Src/bms_limits.c ../Core/Inc/bms_limits.h ../Core/Inc/car_limits.h
	$(CC) $(CFLAGS) -DBMS_LIMITS_HOST -o $@ regen_test.c $(LDLIBS)

# Same module with REGEN_ENABLE 0, the passthrough build.
regen_off_test: regen_off_test.c ../Core/Src/regen.c ../Core/Inc/regen.h ../Core/Inc/config.h ../Core/Inc/bms_limits.h ../Core/Inc/car_limits.h
	$(CC) $(CFLAGS) -o $@ regen_off_test.c $(LDLIBS)

# control takes inputs and elapsed time from the caller, so it needs no shim.
control_test: control_test.c ../Core/Src/control.c ../Core/Inc/control.h ../Core/Inc/config.h ../Core/Inc/bms_limits.h ../Core/Inc/car_limits.h
	$(CC) $(CFLAGS) -o $@ control_test.c $(LDLIBS)

# version has no HAL dependency; uses build_version.h when version.sh has run.
version_test: version_test.c ../Core/Src/version.c ../Core/Inc/version.h
	$(CC) $(CFLAGS) -o $@ version_test.c $(LDLIBS)

# bms_limits reaches the HAL for its critical sections; BMS_LIMITS_HOST swaps in stubs.
bms_limits_test: bms_limits_test.c ../Core/Src/bms_limits.c ../Core/Inc/bms_limits.h ../Core/Inc/car_limits.h ../Core/Inc/config.h
	$(CC) $(CFLAGS) -DBMS_LIMITS_HOST -o $@ bms_limits_test.c $(LDLIBS)

LLVM_COV      ?= $(shell xcrun -f llvm-cov 2>/dev/null || echo llvm-cov)
LLVM_PROFDATA ?= $(shell xcrun -f llvm-profdata 2>/dev/null || echo llvm-profdata)
COV_SRCS = $(addprefix ../Core/Src/,adc.c bms_limits.c can_tx.c control.c launch_control.c regen.c rtd.c \
           version.c)

# Rebuilds instrumented, reports, then cleans so plain builds aren't left instrumented.
coverage:
	$(MAKE) clean
	$(MAKE) $(TESTS) CC=clang CFLAGS="$(CFLAGS) -fprofile-instr-generate -fcoverage-mapping"
	rm -rf cov && mkdir cov
	for t in $(TESTS); do LLVM_PROFILE_FILE=cov/$$t.profraw ./$$t > /dev/null || exit 1; done
	$(LLVM_PROFDATA) merge -o cov/all.profdata cov/*.profraw
	$(LLVM_COV) report -show-branch-summary -instr-profile cov/all.profdata $(firstword $(TESTS)) \
		$(addprefix -object ,$(wordlist 2,$(words $(TESTS)),$(TESTS))) $(COV_SRCS)
	$(MAKE) clean

clean:
	rm -rf $(TESTS) *.dSYM cov
