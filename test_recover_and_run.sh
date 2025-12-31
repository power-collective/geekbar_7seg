#!/bin/bash
# Recover board and test shellcode

echo "=== Recovering board ==="
cd ..
./venv/bin/python utils/recover_halted_device.py
sleep 1

./venv/bin/python utils/reset_mcu_new.py
sleep 1

echo ""
echo "=== Running shellcode test ==="
cd gpio-shellcode-generator

# Generate fresh shellcode
./venv/bin/python -c "
from gpio_blink_generator import generate_gpio_shellcode
shellcode = generate_gpio_shellcode('PA4', 'PA5', num_blinks=3, save_output='./build/test_final')
print(f'Generated {len(shellcode)} bytes')
"

# Inject and run
../venv/bin/python -c "
from gpio_blink_generator import inject_and_run_blink
from pyrsp_py32f003 import PY32F003

with open('./build/test_final/shellcode.bin', 'rb') as f:
    shellcode = f.read()

print(f'Loaded {len(shellcode)} bytes')

with PY32F003(verbose=True) as mcu:
    inject_and_run_blink(mcu, shellcode)
    print('✓ Shellcode running - watch LEDs!')
"

echo ""
echo "=== Test complete ==="
echo "Expected: PA4 blinks 3x, PA5 goes HIGH, MCU resets after 16 sec"
