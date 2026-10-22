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

TESTS = soc_kf_test lc_test can_tx_test rtd_test adc_test regen_test regen_off_test \
        soc_derate_test soc_derate_endurance_test control_test version_test inv_tune_test

# Built so they stay compiling, not run (need a log to replay)
TOOLS = soc_kf_replay

.PHONY: all test coverage clean
all: test

test: $(TESTS) $(TOOLS)
	@for t in $(TESTS); do \
		echo "== $$t"; \
		./$$t || exit 1; \
	done

# soc_kf reaches the HAL for its critical sections; SOC_KF_HOST swaps in stubs.
soc_kf_test: soc_kf_test.c ../Core/Src/soc_kf.c ../Core/Inc/soc_kf.h ../Core/Inc/soc_kf_tables.h
	$(CC) $(CFLAGS) -DSOC_KF_HOST -o $@ soc_kf_test.c $(LDLIBS)

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

regen_test: regen_test.c ../Core/Src/regen.c ../Core/Inc/regen.h ../Core/Inc/config.h ../Core/Src/soc_kf.c ../Core/Inc/soc_kf.h
	$(CC) $(CFLAGS) -DSOC_KF_HOST -o $@ regen_test.c $(LDLIBS)

# Same module with REGEN_ENABLE 0, the passthrough build.
regen_off_test: regen_off_test.c ../Core/Src/regen.c ../Core/Inc/regen.h ../Core/Inc/config.h ../Core/Src/soc_kf.c ../Core/Inc/soc_kf.h
	$(CC) $(CFLAGS) -DSOC_KF_HOST -o $@ regen_off_test.c $(LDLIBS)

soc_derate_test: soc_derate_test.c ../Core/Src/soc_derate.c ../Core/Inc/soc_derate.h ../Core/Inc/soc_derate_tunes.h ../Core/Inc/config.h ../Core/Src/soc_kf.c ../Core/Inc/soc_kf.h ../Core/Inc/soc_kf_tables.h
	$(CC) $(CFLAGS) -DSOC_KF_HOST -o $@ soc_derate_test.c $(LDLIBS)

# Same tests on the endurance tune, so its temp curve is covered whichever is active.
soc_derate_endurance_test: soc_derate_test.c ../Core/Src/soc_derate.c ../Core/Inc/soc_derate.h ../Core/Inc/soc_derate_tunes.h ../Core/Inc/config.h ../Core/Src/soc_kf.c ../Core/Inc/soc_kf.h ../Core/Inc/soc_kf_tables.h
	$(CC) $(CFLAGS) -DSOC_KF_HOST -DSOC_DERATE_TUNE=endurance -o $@ soc_derate_test.c $(LDLIBS)

# control takes inputs and elapsed time from the caller, so it needs no shim.
control_test: control_test.c ../Core/Src/control.c ../Core/Inc/control.h ../Core/Inc/config.h
	$(CC) $(CFLAGS) -o $@ control_test.c $(LDLIBS)

# version has no HAL dependency; uses build_version.h when version.sh has run.
version_test: version_test.c ../Core/Src/version.c ../Core/Inc/version.h
	$(CC) $(CFLAGS) -o $@ version_test.c $(LDLIBS)

# inv_tune takes ticks and frames from the caller, so it needs no shim.
inv_tune_test: inv_tune_test.c ../Core/Src/inv_tune.c ../Core/Inc/inv_tune.h ../Core/Inc/inv_tune_params.h
	$(CC) $(CFLAGS) -o $@ inv_tune_test.c $(LDLIBS)

soc_kf_replay: soc_kf_replay.c ../Core/Src/soc_kf.c ../Core/Inc/soc_kf.h ../Core/Inc/soc_kf_tables.h
	$(CC) $(CFLAGS) -DSOC_KF_HOST -o $@ soc_kf_replay.c $(LDLIBS)

LLVM_COV      ?= $(shell xcrun -f llvm-cov 2>/dev/null || echo llvm-cov)
LLVM_PROFDATA ?= $(shell xcrun -f llvm-profdata 2>/dev/null || echo llvm-profdata)
COV_SRCS = $(addprefix ../Core/Src/,adc.c can_tx.c control.c launch_control.c regen.c rtd.c \
           soc_derate.c soc_kf.c version.c inv_tune.c)

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
	rm -rf $(TESTS) $(TOOLS) *.dSYM cov
