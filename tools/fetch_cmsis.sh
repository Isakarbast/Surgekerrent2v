#!/usr/bin/env bash
# Fetch the CMSIS core + STM32G4 device headers needed by the firmware build.
# Only headers are used (no ST HAL): the HAL in src/hal/stm32g4 is register-level.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p third_party/cmsis

if [ ! -d third_party/cmsis/core ]; then
    git clone --depth 1 --filter=blob:none --sparse https://github.com/ARM-software/CMSIS_5.git /tmp/sk2v_cmsis5
    (cd /tmp/sk2v_cmsis5 && git sparse-checkout set CMSIS/Core/Include)
    mkdir -p third_party/cmsis/core
    cp -r /tmp/sk2v_cmsis5/CMSIS/Core/Include third_party/cmsis/core/
    rm -rf /tmp/sk2v_cmsis5
fi

if [ ! -d third_party/cmsis/device_g4 ]; then
    git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_g4.git third_party/cmsis/device_g4
fi

echo "CMSIS ready in third_party/cmsis"
