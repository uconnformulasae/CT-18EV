#!/bin/sh
# Format the hand-written firmware sources. --check exits non-zero if anything
# is unformatted, for CI.
#
# Runs as a pre-build step (see prebuildStep in .cproject). clang-format -i
# leaves an already-formatted file's mtime alone, so this does not trigger
# rebuilds. If clang-format is missing the build still proceeds; only --check
# treats that as an error.
#
# main.c is excluded: most of it is CubeMX-generated and is rewritten in ST's
# style on every regeneration. Its USER CODE regions are kept to this style by
# hand.

set -e
cd "$(dirname "$0")/.."

FILES="
Core/Inc/config.h
Core/Inc/pinout.h
Core/Inc/adc.h
Core/Inc/can_tx.h
Core/Inc/rtd.h
Core/Inc/launch_control.h
Core/Inc/regen.h
Core/Inc/soc_kf.h
Core/Inc/soc_kf_tables.h
Core/Src/adc.c
Core/Src/can_tx.c
Core/Src/rtd.c
Core/Src/launch_control.c
Core/Src/regen.c
Core/Src/soc_kf.c
tools/adc_test.c
tools/can_tx_test.c
tools/lc_test.c
tools/rtd_test.c
tools/soc_kf_test.c
"

if command -v clang-format >/dev/null 2>&1; then
    CF="clang-format"
elif command -v xcrun >/dev/null 2>&1 && xcrun -f clang-format >/dev/null 2>&1; then
    CF="xcrun clang-format"
else
    if [ "$1" = "--check" ]; then
        echo "format.sh: clang-format not found" >&2
        exit 1
    fi
    echo "format.sh: clang-format not found, skipping" >&2
    exit 0
fi

if [ "$1" = "--check" ]; then
    $CF --dry-run --Werror $FILES
else
    $CF -i $FILES
fi
