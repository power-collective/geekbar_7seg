#!/usr/bin/env python3
"""
pyrsp_py32f003 - Interactive debugger class for PY32F003/F030 MCUs
Consolidates common operations for GPIO, Flash, and debugging via Black Magic Probe

This class eliminates ~800+ lines of redundant code across multiple scripts.
"""

import time
import glob as _glob
import re
import atexit
import weakref
from pyrsp.rsp import CortexM3, RSP

# Global registry of active connections for cleanup on interpreter exit
_active_connections = weakref.WeakSet()


class Pin:
    """
    Represents a single GPIO pin with convenient methods

    This provides an elegant API for GPIO control:
        mcu.PA0.high()      # Set PA0 high
        mcu.PA0.low()       # Set PA0 low
        mcu.PA0.toggle()    # Toggle PA0
        value = mcu.PA0.read()  # Read PA0 state

    You can also use it as output or input:
        mcu.PA0.output()    # Configure as output
        mcu.PA0.input()     # Configure as input
    """

    def __init__(self, mcu, port, pin):
        """
        Initialize Pin object

        Args:
            mcu: PY32F003 instance
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)
        """
        self.mcu = mcu
        self.port = port
        self.pin = pin
        self.name = f"P{port}{pin}"

    def high(self):
        """Set pin HIGH"""
        self.mcu.gpio_set_high(self.port, self.pin)
        return self

    def low(self):
        """Set pin LOW"""
        self.mcu.gpio_set_low(self.port, self.pin)
        return self

    def toggle(self):
        """
        Toggle pin state

        Returns:
            int: New pin state (0 or 1)
        """
        return self.mcu.gpio_toggle(self.port, self.pin)

    def read(self):
        """
        Read pin output state

        Returns:
            int: Pin state (0 or 1)
        """
        return self.mcu.gpio_read_output(self.port, self.pin)

    def read_input(self):
        """
        Read pin input state (from IDR)

        Returns:
            int: Pin state (0 or 1)
        """
        return self.mcu.gpio_read_input(self.port, self.pin)

    def output(self):
        """Configure pin as output"""
        self.mcu.gpio_set_output(self.port, self.pin)
        return self

    def input(self):
        """Configure pin as input"""
        self.mcu.gpio_set_input(self.port, self.pin)
        return self

    def __repr__(self):
        return f"Pin({self.name})"

    def __str__(self):
        return self.name


class PY32F003:
    """
    Interactive debugger for PY32F003/F030 MCUs via Black Magic Probe

    Features:
    - Auto-detect device via glob pattern
    - Context manager support (use with 'with' statement)
    - GPIO control (read/write pins, configure as input/output)
    - Flash operations (unlock, erase, program, read)
    - CPU control (halt, resume, reset)
    - Watchdog handling
    - Option byte management

    Example usage:
        with PY32F003() as mcu:
            mcu.halt()
            mcu.gpio_set_output('A', 1)
            mcu.gpio_set_high('A', 1)
            value = mcu.read_u32(mcu.GPIOA_ODR)
            print(f"GPIOA_ODR: 0x{value:04X}")
    """

    # ========================================================================
    # Register Addresses - parsed from py32f0xx.h
    # ========================================================================

    # Flash memory
    FLASH_BASE = 0x08000000

    # SRAM
    SRAM_BASE = 0x20000000

    # GPIO bases
    GPIOA_BASE = 0x50000000
    GPIOB_BASE = 0x50000400
    GPIOF_BASE = 0x50001400

    # GPIO register offsets
    MODER_OFFSET = 0x00    # Mode register
    OTYPER_OFFSET = 0x04   # Output type register
    OSPEEDR_OFFSET = 0x08  # Output speed register
    PUPDR_OFFSET = 0x0C    # Pull-up/pull-down register
    IDR_OFFSET = 0x10      # Input data register
    ODR_OFFSET = 0x14      # Output data register
    BSRR_OFFSET = 0x18     # Bit set/reset register
    BRR_OFFSET = 0x28      # Bit reset register

    # Computed GPIO register addresses
    @property
    def GPIOA_MODER(self): return self.GPIOA_BASE + self.MODER_OFFSET
    @property
    def GPIOA_IDR(self): return self.GPIOA_BASE + self.IDR_OFFSET
    @property
    def GPIOA_ODR(self): return self.GPIOA_BASE + self.ODR_OFFSET

    @property
    def GPIOB_MODER(self): return self.GPIOB_BASE + self.MODER_OFFSET
    @property
    def GPIOB_IDR(self): return self.GPIOB_BASE + self.IDR_OFFSET
    @property
    def GPIOB_ODR(self): return self.GPIOB_BASE + self.ODR_OFFSET

    @property
    def GPIOF_MODER(self): return self.GPIOF_BASE + self.MODER_OFFSET
    @property
    def GPIOF_IDR(self): return self.GPIOF_BASE + self.IDR_OFFSET
    @property
    def GPIOF_ODR(self): return self.GPIOF_BASE + self.ODR_OFFSET

    # Flash registers
    FLASH_ACR = 0x40022000
    FLASH_KEYR = 0x40022008
    FLASH_OPTKEYR = 0x4002200C
    FLASH_SR = 0x40022010
    FLASH_CR = 0x40022014
    FLASH_OPTR = 0x40022020
    FLASH_SDKR = 0x40022024
    FLASH_WRPR = 0x4002202C
    FLASH_TRIGGER = 0x40022080

    # Flash timing registers (for 24MHz operation)
    FLASH_TS0 = 0x40022100
    FLASH_TS1 = 0x40022104
    FLASH_TS2P = 0x40022108
    FLASH_TPS3 = 0x4002210C
    FLASH_TS3 = 0x40022110
    FLASH_PERTPE = 0x40022114
    FLASH_SMERTPE = 0x40022118
    FLASH_PRGTPE = 0x4002211C
    FLASH_PRETPE = 0x40022120

    # Flash keys
    FLASH_KEY1 = 0x45670123
    FLASH_KEY2 = 0xCDEF89AB
    OPTKEY1 = 0x08192A3B
    OPTKEY2 = 0x4C5D6E7F

    # Flash control bits
    FLASH_CR_LOCK = (1 << 31)
    FLASH_CR_OPTLOCK = (1 << 30)
    FLASH_CR_OBL_LAUNCH = (1 << 27)
    FLASH_CR_EOPIE = (1 << 24)
    FLASH_CR_PGSTRT = (1 << 19)
    FLASH_CR_OPTSTRT = (1 << 17)
    FLASH_CR_SER = (1 << 11)
    FLASH_CR_OPTER = (1 << 5)
    FLASH_CR_OPTPG = (1 << 4)
    FLASH_CR_PER = (1 << 1)
    FLASH_CR_PG = (1 << 0)

    # Flash status bits
    FLASH_SR_BSY = (1 << 16)
    FLASH_SR_OPTVERR = (1 << 15)
    FLASH_SR_WRPERR = (1 << 4)
    FLASH_SR_EOP = (1 << 0)

    # RCC registers
    RCC_BASE = 0x40021000
    RCC_CR = 0x40021000
    RCC_ICSCR = 0x40021004
    RCC_IOPENR = RCC_BASE + 0x34
    RCC_APBENR2 = RCC_BASE + 0x40

    # RCC bits
    RCC_CR_HSIRDY = (1 << 10)
    RCC_ICSCR_HSI_FS_Mask = 0x0000E000
    RCC_ICSCR_HSI_FS_24MHz = 0x00008000
    RCC_ICSCR_HSI_TRIM_Mask = 0x00001FFF

    # DBGMCU
    DBGMCU_BASE = 0x40015800
    DBGMCU_CR = 0x40015804
    DBGMCU_APBFZ1 = 0x40015808
    DBGMCU_APBFZ2 = 0x4001580C

    # Watchdog
    IWDG_BASE = 0x40003000
    IWDG_KR = 0x40003000
    IWDG_PR = 0x40003004
    IWDG_RLR = 0x40003008

    # ADC (Analog to Digital Converter)
    ADC_BASE = 0x40012400
    ADC_ISR = 0x40012400      # Interrupt and status register
    ADC_IER = 0x40012404      # Interrupt enable register
    ADC_CR = 0x40012408       # Control register
    ADC_CFGR1 = 0x4001240C    # Configuration register 1
    ADC_CFGR2 = 0x40012410    # Configuration register 2
    ADC_SMPR = 0x40012414     # Sampling time register
    ADC_TR = 0x40012420       # Analog watchdog threshold register
    ADC_CHSELR = 0x40012428   # Channel selection register
    ADC_DR = 0x40012440       # Data register
    ADC_CCSR = 0x40012444     # Calibration config and status register
    ADC_CCR = 0x40012708      # Common configuration register

    # ADC Control register (CR) bits
    ADC_CR_ADEN = (1 << 0)     # ADC enable
    ADC_CR_ADSTART = (1 << 2)  # ADC start conversion
    ADC_CR_ADSTP = (1 << 4)    # ADC stop conversion
    ADC_CR_ADCAL = (1 << 31)   # ADC calibration

    # ADC Configuration register 1 (CFGR1) bits
    ADC_CFGR1_RES_12BIT = (0b00 << 3)  # 12-bit resolution (default)
    ADC_CFGR1_RES_10BIT = (0b01 << 3)  # 10-bit resolution
    ADC_CFGR1_RES_8BIT = (0b10 << 3)   # 8-bit resolution
    ADC_CFGR1_RES_6BIT = (0b11 << 3)   # 6-bit resolution
    ADC_CFGR1_RES_MASK = (0b11 << 3)   # Resolution mask
    ADC_CFGR1_CONT = (1 << 13)         # Continuous conversion mode

    # ADC Status register (ISR) bits
    # Note: PY32F003 does NOT have ADRDY bit (bit 0 is reserved)
    ADC_ISR_EOSMP = (1 << 1)   # End of sampling flag
    ADC_ISR_EOC = (1 << 2)     # End of conversion
    ADC_ISR_EOSEQ = (1 << 3)   # End of sequence
    ADC_ISR_OVR = (1 << 4)     # Overrun
    ADC_ISR_AWD = (1 << 7)     # Analog watchdog flag

    # ADC channels (pin mapping)
    # CH0-CH7: PA0-PA7
    # CH8-CH9: PB0-PB1
    # CH10: Internal temperature sensor
    # CH11: Internal voltage reference (VREFINT)
    # CH12: PA13 (SWDIO)
    # CH13: PA14 (SWCLK)
    ADC_CHANNEL_0 = (1 << 0)   # PA0
    ADC_CHANNEL_1 = (1 << 1)   # PA1
    ADC_CHANNEL_2 = (1 << 2)   # PA2
    ADC_CHANNEL_3 = (1 << 3)   # PA3
    ADC_CHANNEL_4 = (1 << 4)   # PA4
    ADC_CHANNEL_5 = (1 << 5)   # PA5
    ADC_CHANNEL_6 = (1 << 6)   # PA6
    ADC_CHANNEL_7 = (1 << 7)   # PA7
    ADC_CHANNEL_8 = (1 << 8)   # PB0
    ADC_CHANNEL_9 = (1 << 9)   # PB1
    ADC_CHANNEL_TEMPSENSOR = (1 << 10)  # Internal temperature sensor
    ADC_CHANNEL_VREFINT = (1 << 11)     # Internal voltage reference
    ADC_CHANNEL_12 = (1 << 12) # PA13 (SWDIO)
    ADC_CHANNEL_13 = (1 << 13) # PA14 (SWCLK)

    # ARM Cortex-M System Control Block
    AIRCR = 0xE000ED0C
    AIRCR_VECTKEY = 0x05FA0000
    AIRCR_SYSRESETREQ = 0x04

    # Factory calibration addresses
    HSI_TRIM_24MHZ_ADDR = 0x1FFF0F10
    FLASH_CAL_BASE1 = 0x1FFF0F1C + (4 * 0x14)
    FLASH_CAL_BASE2 = 0x1FFF0F20 + (4 * 0x14)
    FLASH_CAL_BASE3 = 0x1FFF0F24 + (4 * 0x14)
    FLASH_CAL_BASE4 = 0x1FFF0F28 + (4 * 0x14)
    FLASH_CAL_BASE5 = 0x1FFF0F2C + (4 * 0x14)

    # Option bytes
    OB_BASE = 0x1FFF0E80

    # ========================================================================
    # Initialization
    # ========================================================================

    def __init__(self, device=None, verbose=False):
        """
        Initialize connection to PY32F003/F030 MCU via Black Magic Probe

        Args:
            device: Device path, glob pattern, or None for auto-detect
                   Examples: '/dev/cu.usbmodem101', '/dev/cu.usbmodem*', None
            verbose: Enable verbose output from pyrsp
        """
        self.verbose = verbose
        self.device = self._detect_device(device)
        self.rsp = self._quick_connect()
        self._closed = False

        # Register this connection for automatic cleanup on interpreter exit
        _active_connections.add(self)

    def _detect_device(self, device):
        """
        Auto-detect Black Magic Probe device

        Args:
            device: Device path, glob pattern, or None

        Returns:
            str: Resolved device path
        """
        if device is None:
            # Auto-detect using common patterns
            patterns = [
                '/dev/cu.usbmodem*',     # macOS
                '/dev/ttyACM*',          # Linux
                '/dev/ttyBmpGdb',        # Linux with udev rule
            ]
            for pattern in patterns:
                devices = _glob.glob(pattern)
                if devices:
                    device = devices[0]
                    if self.verbose:
                        print(f"Auto-detected device: {device}")
                    return device
            raise RuntimeError(
                "No Black Magic Probe found. Please specify device path.\n"
                "Common paths: /dev/cu.usbmodem* (macOS), /dev/ttyACM* (Linux)"
            )
        elif '*' in device or '?' in device:
            # User provided glob pattern
            devices = _glob.glob(device)
            if not devices:
                raise RuntimeError(f"No device found matching pattern: {device}")
            device = devices[0]
            if self.verbose:
                print(f"Found device: {device}")

        return device

    def _quick_connect(self):
        """
        Connect to device with RSP.fetchOK monkeypatch

        This monkeypatch allows halt responses (T packets) to be accepted
        """
        original_fetchOK = RSP.fetchOK

        def patched_fetchOK(self, data, ok=b'OK'):
            res = self.fetch(data)
            if ok.startswith(b'T') and res.startswith(ok):
                return
            if res != ok:
                raise ValueError(res)

        RSP.fetchOK = patched_fetchOK

        try:
            if self.verbose:
                print(f"Connecting to Black Magic Probe at {self.device}...")
            rsp = CortexM3(self.device, elffile=None, verbose=self.verbose)
            if self.verbose:
                print("Connected!")
            return rsp
        finally:
            RSP.fetchOK = original_fetchOK

    # ========================================================================
    # Context Manager Support
    # ========================================================================

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - close connection"""
        self.close()
        return False

    def close(self):
        """
        Close connection to device

        IMPORTANT: This method attempts to resume the CPU and clear debug halt
        state before disconnecting. This prevents the device from being left in
        an unrecoverable halted state.
        """
        if self._closed:
            return  # Already closed, don't run cleanup twice

        self._closed = True


        try:
            # Clear DBG_STOP bit in DBGMCU_CR to prevent persistent halt state
            # This ensures the device can run normally after reset
            dbgmcu_cr = self.read_u32(self.DBGMCU_CR)
            if dbgmcu_cr & (1 << 1):  # DBG_STOP bit set
                dbgmcu_cr &= ~(1 << 1)  # Clear DBG_STOP
                self.write_u32(self.DBGMCU_CR, dbgmcu_cr)
                if self.verbose:
                    print("Cleared DBG_STOP bit")
        except Exception as e:
            if self.verbose:
                print(f"Warning: Could not clear DBG_STOP: {e}")

        try:
            # Try to resume CPU before disconnecting
            # This prevents leaving the device in a halted state
            if self.verbose:
                print("Resuming CPU before disconnect...")
            self.resume()
            time.sleep(0.05)  # Brief delay to ensure resume takes effect
        except Exception as e:
            if self.verbose:
                print(f"Warning: Could not resume CPU: {e}")

        try:
            # Close serial port
            self.rsp.port.close(self.rsp)
        except:
            pass

    # ========================================================================
    # Memory Access
    # ========================================================================

    def read_u32(self, address):
        """
        Read 32-bit value from memory

        Args:
            address: Memory address to read

        Returns:
            int: 32-bit value
        """
        data = self.rsp.dump(4, address)
        return int.from_bytes(data, byteorder='little')

    def read_u16(self, address):
        """
        Read 16-bit value from memory

        Args:
            address: Memory address to read

        Returns:
            int: 16-bit value
        """
        data = self.rsp.dump(2, address)
        return int.from_bytes(data, byteorder='little')

    def read_u8(self, address):
        """
        Read 8-bit value from memory

        Args:
            address: Memory address to read

        Returns:
            int: 8-bit value
        """
        data = self.rsp.dump(1, address)
        return int.from_bytes(data, byteorder='little')

    def write_u32(self, address, value):
        """
        Write 32-bit value to memory

        Args:
            address: Memory address to write
            value: 32-bit value to write
        """
        self.rsp.store(value.to_bytes(4, byteorder='little'), address)

    def write_u16(self, address, value):
        """
        Write 16-bit value to memory

        Args:
            address: Memory address to write
            value: 16-bit value to write
        """
        self.rsp.store(value.to_bytes(2, byteorder='little'), address)

    def write_u8(self, address, value):
        """
        Write 8-bit value to memory

        Args:
            address: Memory address to write
            value: 8-bit value to write
        """
        self.rsp.store(value.to_bytes(1, byteorder='little'), address)

    def dump_memory(self, address, size):
        """
        Read block of memory

        Args:
            address: Starting memory address
            size: Number of bytes to read

        Returns:
            bytes: Memory contents
        """
        return self.rsp.dump(size, address)

    # ========================================================================
    # CPU Control
    # ========================================================================

    def halt(self):
        """
        Halt CPU execution
        """
        self.rsp.port.write(b'\x03')  # Send Ctrl+C
        time.sleep(0.05)
        try:
            self.rsp.readpkt(timeout=0.5)
        except:
            pass

    def resume(self):
        """
        Resume CPU execution
        """
        self.rsp.send(b'c')

    def read_pc(self):
        """
        Read current Program Counter (PC) value

        Returns:
            int: Current PC address
        """
        self.rsp.refresh_regs()
        return int(self.rsp.regs['pc'], 16)

    def read_register(self, reg_name):
        """
        Read any CPU register value

        Args:
            reg_name: Register name (e.g., 'r0', 'r1', 'sp', 'lr', 'pc')

        Returns:
            int: Register value
        """
        self.rsp.refresh_regs()
        return int(self.rsp.regs[reg_name], 16)

    def write_register(self, reg_name, value):
        """
        Write value to CPU register

        Args:
            reg_name: Register name (e.g., 'r0', 'r1', 'sp', 'lr', 'pc')
            value: Value to write (int)
        """
        self.rsp.set_reg(reg_name, value)

    def dump_registers(self):
        """
        Print all CPU registers
        """
        self.rsp.dump_regs()

    def read_current_instruction(self, disassemble=True):
        """
        Read instruction at current PC and optionally disassemble

        Args:
            disassemble: If True, attempt to disassemble instruction

        Returns:
            dict: {'pc': int, 'bytes': bytes, 'disasm': str (if available)}
        """
        pc = self.read_pc()

        # Read first 2 bytes (minimum instruction size in Thumb)
        instr_bytes = self.dump_memory(pc, 2)
        first_half = int.from_bytes(instr_bytes, 'little')

        # Check if it's a 32-bit Thumb instruction
        # 32-bit instructions have first halfword with bits [15:11] = 0b11101, 0b11110, or 0b11111
        is_32bit = ((first_half & 0xE000) == 0xE000) and ((first_half & 0x1800) != 0x0000)

        if is_32bit:
            # Read additional 2 bytes
            instr_bytes = self.dump_memory(pc, 4)

        result = {
            'pc': pc,
            'bytes': instr_bytes,
            'size': len(instr_bytes),
        }

        if disassemble:
            try:
                # Try using capstone for disassembly
                import capstone
                md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
                md.detail = True

                for insn in md.disasm(instr_bytes, pc):
                    result['disasm'] = f"{insn.mnemonic} {insn.op_str}"
                    result['mnemonic'] = insn.mnemonic
                    result['operands'] = insn.op_str
                    break
            except ImportError:
                # Capstone not available - decode common instructions manually
                if len(instr_bytes) == 2:
                    instr = int.from_bytes(instr_bytes, 'little')

                    # Common 16-bit Thumb instructions
                    if instr == 0xBF30:
                        result['disasm'] = 'wfi  (wait for interrupt)'
                    elif instr == 0xBF20:
                        result['disasm'] = 'wfe  (wait for event)'
                    elif instr == 0xBF40:
                        result['disasm'] = 'sev  (set event)'
                    elif instr == 0xBF00:
                        result['disasm'] = 'nop'
                    elif (instr & 0xFE00) == 0xE000:
                        # Unconditional branch
                        offset = (instr & 0x7FF) << 1
                        if offset & 0x800:
                            offset |= 0xFFFFF000  # Sign extend
                        target = (pc + 4 + offset) & 0xFFFFFFFF
                        result['disasm'] = f'b    0x{target:08x}'
                    elif (instr & 0xFF00) == 0xBD00:
                        # Pop with PC
                        result['disasm'] = f'pop  {{...}} (return)'
                    else:
                        result['disasm'] = f'<0x{instr:04x}> (install capstone for full disassembly)'
                else:
                    instr = int.from_bytes(instr_bytes, 'little')
                    result['disasm'] = f'<0x{instr:08x}> (install capstone for full disassembly)'

        return result

    def show_current_instruction(self):
        """
        Print current instruction at PC with disassembly
        """
        info = self.read_current_instruction(disassemble=True)

        hex_bytes = ' '.join(f'{b:02x}' for b in info['bytes'])
        print(f"PC: 0x{info['pc']:08X}")
        print(f"Bytes: {hex_bytes}")

        if 'disasm' in info:
            print(f"Instruction: {info['disasm']}")

        return info

    def reset(self, halt_immediately=False):
        """
        Reset MCU via GDB remote protocol

        Args:
            halt_immediately: If True, halt CPU immediately after reset
        """
        self.rsp.port.write(b'$R#52')
        time.sleep(0.05)
        try:
            self.rsp.readpkt(timeout=0.3)
        except:
            pass

        if halt_immediately:
            time.sleep(0.01)
            self.halt()

    def reset_via_aircr(self):
        """
        Reset MCU via ARM AIRCR register (system reset)
        """
        self.write_u32(self.AIRCR, self.AIRCR_VECTKEY | self.AIRCR_SYSRESETREQ)

    # ========================================================================
    # GPIO Operations
    # ========================================================================

    def _get_gpio_base(self, port):
        """
        Get GPIO base address from port letter

        Args:
            port: Port letter ('A', 'B', or 'F')

        Returns:
            int: GPIO base address
        """
        port = port.upper()
        bases = {
            'A': self.GPIOA_BASE,
            'B': self.GPIOB_BASE,
            'F': self.GPIOF_BASE,
        }
        if port not in bases:
            raise ValueError(f"Invalid port '{port}'. Must be A, B, or F")
        return bases[port]

    def gpio_set_mode(self, port, pin, mode):
        """
        Set GPIO pin mode

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)
            mode: Mode value (0=input, 1=output, 2=alternate, 3=analog)
        """
        base = self._get_gpio_base(port)
        moder_addr = base + self.MODER_OFFSET

        moder = self.read_u32(moder_addr)
        moder &= ~(0x3 << (pin * 2))  # Clear mode bits
        moder |= (mode & 0x3) << (pin * 2)  # Set new mode
        self.write_u32(moder_addr, moder)

    def gpio_set_output(self, port, pin):
        """
        Configure GPIO pin as output

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)
        """
        self.gpio_set_mode(port, pin, 1)

    def gpio_set_input(self, port, pin):
        """
        Configure GPIO pin as input

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)
        """
        self.gpio_set_mode(port, pin, 0)

    def gpio_set_high(self, port, pin):
        """
        Set GPIO pin output HIGH

        Uses BSRR register for atomic bit-set operation (more efficient and safer
        than read-modify-write on ODR).

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)
        """
        base = self._get_gpio_base(port)
        bsrr_addr = base + self.BSRR_OFFSET

        # BSRR bits [15:0] set corresponding ODR bits
        self.write_u32(bsrr_addr, 1 << pin)

    def gpio_set_low(self, port, pin):
        """
        Set GPIO pin output LOW

        Uses BSRR register for atomic bit-reset operation (more efficient and safer
        than read-modify-write on ODR).

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)
        """
        base = self._get_gpio_base(port)
        bsrr_addr = base + self.BSRR_OFFSET

        # BSRR bits [31:16] reset corresponding ODR bits
        # Writing 1 to bit (16+pin) clears ODR bit (pin)
        self.write_u32(bsrr_addr, 1 << (16 + pin))

    def gpio_toggle(self, port, pin):
        """
        Toggle GPIO pin output state

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)

        Returns:
            int: New pin state (0 or 1)
        """
        current = self.gpio_read_output(port, pin)
        if current:
            self.gpio_set_low(port, pin)
            return 0
        else:
            self.gpio_set_high(port, pin)
            return 1

    def gpio_read_input(self, port, pin):
        """
        Read GPIO pin input state

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)

        Returns:
            int: Pin state (0 or 1)
        """
        base = self._get_gpio_base(port)
        idr_addr = base + self.IDR_OFFSET

        idr = self.read_u32(idr_addr)
        return (idr >> pin) & 1

    def gpio_read_output(self, port, pin):
        """
        Read GPIO pin output state

        Args:
            port: Port letter ('A', 'B', or 'F')
            pin: Pin number (0-15)

        Returns:
            int: Pin state (0 or 1)
        """
        base = self._get_gpio_base(port)
        odr_addr = base + self.ODR_OFFSET

        odr = self.read_u32(odr_addr)
        return (odr >> pin) & 1

    def gpio_write_port(self, port, value):
        """
        Write entire GPIO port ODR register

        Args:
            port: Port letter ('A', 'B', or 'F')
            value: 16-bit value to write
        """
        base = self._get_gpio_base(port)
        odr_addr = base + self.ODR_OFFSET
        self.write_u32(odr_addr, value & 0xFFFF)

    def gpio_read_port(self, port, use_odr=False):
        """
        Read entire GPIO port register

        Args:
            port: Port letter ('A', 'B', or 'F')
            use_odr: If True, read ODR (output), else read IDR (input)

        Returns:
            int: 16-bit port value
        """
        base = self._get_gpio_base(port)
        reg_addr = base + (self.ODR_OFFSET if use_odr else self.IDR_OFFSET)
        return self.read_u32(reg_addr) & 0xFFFF

    # ========================================================================
    # Watchdog
    # ========================================================================

    def pet_watchdog(self):
        """
        Pet the independent watchdog (IWDG) to prevent reset
        Silently ignores errors if watchdog is not enabled
        """
        try:
            self.write_u32(self.IWDG_KR, 0xAAAA)
        except:
            pass

    def extend_watchdog(self):
        """
        Configure watchdog for maximum timeout (~26 seconds)
        """
        try:
            self.write_u32(self.IWDG_KR, 0x5555)  # Unlock
            time.sleep(0.01)
            self.write_u32(self.IWDG_PR, 0x7)     # Max prescaler
            time.sleep(0.01)
            self.write_u32(self.IWDG_RLR, 0xFFF)  # Max reload
            time.sleep(0.01)
            self.write_u32(self.IWDG_KR, 0xAAAA)  # Reload
            if self.verbose:
                print("Watchdog timeout extended to ~26 seconds")
        except Exception as e:
            if self.verbose:
                print(f"Warning: Watchdog config failed: {e}")

    # ========================================================================
    # RCC / Clock Configuration
    # ========================================================================

    def enable_gpio_clocks(self):
        """
        Enable all GPIO peripheral clocks (GPIOA, GPIOB, GPIOF)
        """
        iopenr = self.read_u32(self.RCC_IOPENR)
        iopenr |= (1 << 0) | (1 << 1) | (1 << 5)  # GPIOA, GPIOB, GPIOF
        self.write_u32(self.RCC_IOPENR, iopenr)

    def enable_adc_clock(self):
        """
        Enable ADC peripheral clock
        Required for ADC access during debug halt
        """
        apbenr2 = self.read_u32(self.RCC_APBENR2)
        apbenr2 |= (1 << 20)  # ADCEN (bit 20)
        self.write_u32(self.RCC_APBENR2, apbenr2)
        if self.verbose:
            print("ADC peripheral clock enabled")

    def configure_hsi_24mhz(self):
        """
        Configure HSI clock to 24MHz using factory calibration
        Required for proper flash programming timing
        """
        if self.verbose:
            print("Configuring HSI to 24MHz...")

        # Read factory trim for 24MHz
        hsi_trim_24mhz = self.read_u32(self.HSI_TRIM_24MHZ_ADDR)

        # Read current ICSCR
        icscr = self.read_u32(self.RCC_ICSCR)

        # Set HSI_FS to 24MHz
        icscr = (icscr & ~self.RCC_ICSCR_HSI_FS_Mask) | self.RCC_ICSCR_HSI_FS_24MHz

        # Set HSI_TRIM from factory calibration
        icscr = (icscr & ~self.RCC_ICSCR_HSI_TRIM_Mask) | (hsi_trim_24mhz & self.RCC_ICSCR_HSI_TRIM_Mask)

        # Write new ICSCR value
        self.write_u32(self.RCC_ICSCR, icscr)

        # Wait for HSI ready
        timeout = 1000
        for _ in range(timeout):
            cr = self.read_u32(self.RCC_CR)
            if cr & self.RCC_CR_HSIRDY:
                if self.verbose:
                    print("HSI 24MHz ready")
                return True
            time.sleep(0.001)

        raise RuntimeError("Timeout waiting for HSI ready")

    # ========================================================================
    # DBGMCU Configuration
    # ========================================================================

    def configure_debug_freeze(self):
        """
        Configure DBGMCU to freeze watchdog and timers during debug halt

        This writes to the correct DBGMCU_APBFZ1 and DBGMCU_APBFZ2 registers
        (not DBGMCU_CR which only controls debug enable, not peripheral freeze)
        """
        if self.verbose:
            print("Configuring DBGMCU to freeze peripherals during debug halt...")

        try:
            # Read current values
            apbfz1 = self.read_u32(self.DBGMCU_APBFZ1)
            apbfz2 = self.read_u32(self.DBGMCU_APBFZ2)

            if self.verbose:
                print(f"Current DBGMCU_APBFZ1 = 0x{apbfz1:08x}")
                print(f"Current DBGMCU_APBFZ2 = 0x{apbfz2:08x}")

            # Configure APBFZ1 (APB1 peripherals)
            # Bit 12: DBG_IWDG_STOP - Freeze IWDG during halt
            # Bit 11: DBG_WWDG_STOP - Freeze WWDG during halt
            # Bit 10: DBG_RTC_STOP - Freeze RTC during halt
            # Bit 1: DBG_TIM3_STOP - Freeze TIM3 during halt
            apbfz1_new = apbfz1 | (1 << 12) | (1 << 11) | (1 << 10) | (1 << 1)
            self.write_u32(self.DBGMCU_APBFZ1, apbfz1_new)

            # Configure APBFZ2 (APB2 peripherals)
            # Bit 18: DBG_TIM17_STOP - Freeze TIM17 during halt
            # Bit 17: DBG_TIM16_STOP - Freeze TIM16 during halt
            # Bit 15: DBG_TIM14_STOP - Freeze TIM14 during halt
            # Bit 11: DBG_TIM1_STOP - Freeze TIM1 during halt
            apbfz2_new = apbfz2 | (1 << 18) | (1 << 17) | (1 << 15) | (1 << 11)
            self.write_u32(self.DBGMCU_APBFZ2, apbfz2_new)

            # Verify
            apbfz1_verify = self.read_u32(self.DBGMCU_APBFZ1)
            apbfz2_verify = self.read_u32(self.DBGMCU_APBFZ2)

            if self.verbose:
                print(f"New DBGMCU_APBFZ1 = 0x{apbfz1_verify:08x}")
                print(f"New DBGMCU_APBFZ2 = 0x{apbfz2_verify:08x}")

                if apbfz1_verify & (1 << 12):
                    print("✓ IWDG will freeze during halt")
                if apbfz1_verify & (1 << 11):
                    print("✓ WWDG will freeze during halt")
                if apbfz2_verify & ((1 << 11) | (1 << 15) | (1 << 17) | (1 << 18)):
                    print("✓ Timers will freeze during halt")

        except Exception as e:
            if self.verbose:
                print(f"Warning: DBGMCU access failed: {e}")

    # ========================================================================
    # Flash Operations
    # ========================================================================

    def flash_wait_not_busy(self, timeout=5.0):
        """
        Wait for flash operation to complete

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            int: Flash status register value
        """
        start = time.time()
        while time.time() - start < timeout:
            sr = self.read_u32(self.FLASH_SR)
            if not (sr & self.FLASH_SR_BSY):
                # Check for errors
                if sr & self.FLASH_SR_WRPERR:
                    raise RuntimeError(f"Flash write protection error! SR=0x{sr:08X}")
                return sr
            # Pet watchdog while waiting
            self.pet_watchdog()
            time.sleep(0.01)
        raise RuntimeError("Timeout waiting for Flash operation")

    def flash_clear_flags(self):
        """Clear flash status flags"""
        sr = self.read_u32(self.FLASH_SR)
        if sr & (self.FLASH_SR_EOP | self.FLASH_SR_WRPERR):
            self.write_u32(self.FLASH_SR, sr)

    def flash_unlock(self):
        """
        Unlock flash for programming
        """
        if self.verbose:
            print("Unlocking Flash...")

        self.write_u32(self.FLASH_KEYR, self.FLASH_KEY1)
        self.write_u32(self.FLASH_KEYR, self.FLASH_KEY2)

        cr = self.read_u32(self.FLASH_CR)
        if cr & self.FLASH_CR_LOCK:
            raise RuntimeError("Failed to unlock Flash")

        if self.verbose:
            print("Flash unlocked")

    def flash_unlock_option_bytes(self):
        """
        Unlock option bytes for modification
        """
        if self.verbose:
            print("Unlocking Option Bytes...")

        self.write_u32(self.FLASH_OPTKEYR, self.OPTKEY1)
        self.write_u32(self.FLASH_OPTKEYR, self.OPTKEY2)

        cr = self.read_u32(self.FLASH_CR)
        if cr & self.FLASH_CR_OPTLOCK:
            raise RuntimeError("Failed to unlock Option Bytes")

        if self.verbose:
            print("Option Bytes unlocked")

    def flash_configure_timing_24mhz(self):
        """
        Configure flash timing registers from factory calibration for 24MHz
        Must be called after configure_hsi_24mhz()
        """
        if self.verbose:
            print("Reading factory calibration values (24MHz)...")

        # Read factory calibration data
        cal1 = self.read_u32(self.FLASH_CAL_BASE1)
        cal2 = self.read_u32(self.FLASH_CAL_BASE2)
        cal3 = self.read_u32(self.FLASH_CAL_BASE3)
        cal4 = self.read_u32(self.FLASH_CAL_BASE4)
        cal5 = self.read_u32(self.FLASH_CAL_BASE5)

        if self.verbose:
            print(f"Factory cal: 0x{cal1:08X}, 0x{cal2:08X}, 0x{cal3:08X}, 0x{cal4:08X}, 0x{cal5:08X}")

        # Write timing parameters (exactly as FlashPrg.c does)
        self.write_u32(self.FLASH_TS0, (cal1 >> 0) & 0x000000FF)
        self.write_u32(self.FLASH_TS3, (cal1 >> 8) & 0x000000FF)
        self.write_u32(self.FLASH_TS1, (cal1 >> 16) & 0x000001FF)
        self.write_u32(self.FLASH_TS2P, (cal2 >> 0) & 0x000000FF)
        self.write_u32(self.FLASH_TPS3, (cal2 >> 16) & 0x000007FF)
        self.write_u32(self.FLASH_PERTPE, (cal3 >> 0) & 0x0001FFFF)
        self.write_u32(self.FLASH_SMERTPE, (cal4 >> 0) & 0x0001FFFF)
        self.write_u32(self.FLASH_PRGTPE, (cal5 >> 0) & 0x0000FFFF)
        self.write_u32(self.FLASH_PRETPE, (cal5 >> 16) & 0x0000FFFF)

        if self.verbose:
            print("Flash timing configured from factory calibration")

    def flash_erase_page(self, address):
        """
        Erase a flash sector (4KB on PY32F030)

        Note: PY32F030 uses sector erase (SER), not page erase (PER).
        Each sector is 4KB (4 pages of 1KB each).

        Args:
            address: Address within the sector to erase
        """
        # Clear EOP if set
        sr = self.read_u32(self.FLASH_SR)
        if sr & self.FLASH_SR_EOP:
            self.write_u32(self.FLASH_SR, self.FLASH_SR_EOP)

        # Set sector erase bit + EOPIE
        cr = self.read_u32(self.FLASH_CR)
        cr |= self.FLASH_CR_SER | self.FLASH_CR_EOPIE
        self.write_u32(self.FLASH_CR, cr)

        # Write to the sector address - triggers erase
        # Align to 4KB sector boundary
        sector_addr = address & ~(4096 - 1)
        self.write_u32(sector_addr, 0xFFFFFFFF)

        # Wait for completion
        self.flash_wait_not_busy()

        # Clear SER and EOPIE bits
        cr = self.read_u32(self.FLASH_CR)
        cr &= ~(self.FLASH_CR_SER | self.FLASH_CR_EOPIE)
        self.write_u32(self.FLASH_CR, cr)

    def flash_program_page_128(self, address, data):
        """
        Program a 128-byte flash page using correct 32-word sequence
        Matches ARM FlashPrg.c implementation exactly

        Args:
            address: Flash address to program (must be 128-byte aligned)
            data: Exactly 128 bytes to program
        """
        if len(data) != 128:
            raise ValueError(f"Data must be exactly 128 bytes, got {len(data)}")

        # Clear EOP flag if set
        sr = self.read_u32(self.FLASH_SR)
        if sr & self.FLASH_SR_EOP:
            self.write_u32(self.FLASH_SR, self.FLASH_SR_EOP)

        # Set PG and EOPIE bits
        cr = self.read_u32(self.FLASH_CR)
        cr |= self.FLASH_CR_PG | self.FLASH_CR_EOPIE
        self.write_u32(self.FLASH_CR, cr)

        # Write first 31 words (124 bytes)
        for i in range(31):
            offset = i * 4
            word = int.from_bytes(data[offset:offset+4], byteorder='little')
            self.write_u32(address + offset, word)

            # After writing 31st word (i==30), set PGSTRT
            if i == 30:
                cr = self.read_u32(self.FLASH_CR)
                cr |= self.FLASH_CR_PGSTRT
                self.write_u32(self.FLASH_CR, cr)

        # Write 32nd word (last 4 bytes) - triggers programming
        word_32 = int.from_bytes(data[124:128], byteorder='little')
        self.write_u32(address + 124, word_32)

        # Wait for BSY to clear
        self.flash_wait_not_busy()

        # Clear PG and EOPIE bits
        cr = self.read_u32(self.FLASH_CR)
        cr &= ~(self.FLASH_CR_PG | self.FLASH_CR_EOPIE)
        self.write_u32(self.FLASH_CR, cr)

    # ========================================================================
    # Option Bytes
    # ========================================================================

    def read_option_bytes(self):
        """
        Read option bytes register

        Returns:
            int: FLASH_OPTR value
        """
        return self.read_u32(self.FLASH_OPTR)

    def write_option_bytes(self, value):
        """
        Write option bytes (simplified interface)

        Args:
            value: New FLASH_OPTR value

        Note: This performs the complete sequence including unlock, write,
              trigger, and reload
        """
        # Unlock flash and option bytes
        self.flash_unlock()
        self.flash_unlock_option_bytes()

        # Write new value
        self.write_u32(self.FLASH_OPTR, value)

        # Set OPTSTRT
        cr = self.read_u32(self.FLASH_CR)
        cr |= self.FLASH_CR_OPTSTRT
        self.write_u32(self.FLASH_CR, cr)

        # Trigger programming
        self.write_u32(self.FLASH_TRIGGER, 0x00000000)

        # Wait for completion
        self.flash_wait_not_busy()

        # Reload option bytes
        cr = self.read_u32(self.FLASH_CR)
        cr |= self.FLASH_CR_OBL_LAUNCH
        self.write_u32(self.FLASH_CR, cr)

    # ========================================================================
    # High-Level Helper Functions
    # ========================================================================

    def prepare_for_interactive_debug(self, reset_first=True):
        """
        Prepare MCU for interactive debugging with unlimited halt time

        This function:
        - (Optional) Resets the MCU to known state
        - Halts the CPU
        - Enables GPIO clocks
        - Configures debug freeze (prevents watchdog during halt)
        - Extends watchdog timeout

        Args:
            reset_first: If True, reset MCU before configuring (recommended)
                        This ensures MCU is in a known state. Set to False if
                        you've already reset or want to preserve current state.
        """
        if self.verbose:
            print("\n" + "="*60)
            print("Preparing for interactive debugging...")
            print("="*60)

        # Reset first (critical for reliable GPIO clock configuration!)
        if reset_first:
            if self.verbose:
                print("\n[1] Resetting MCU to known state...")
            self.reset(halt_immediately=False)
            time.sleep(0.05)  # Brief delay after reset
            if self.verbose:
                print("    ✓ Reset complete")

        # Halt CPU
        if self.verbose:
            step = "[2]" if reset_first else "[1]"
            print(f"\n{step} Halting CPU...")
        self.halt()

        # Enable GPIO clocks
        if self.verbose:
            step = "[3]" if reset_first else "[2]"
            print(f"\n{step} Enabling GPIO clocks...")
        self.enable_gpio_clocks()

        # Enable ADC clock (required for ADC access during halt)
        if self.verbose:
            step = "[4]" if reset_first else "[3]"
            print(f"\n{step} Enabling ADC clock...")
        self.enable_adc_clock()

        # Configure debug freeze
        if self.verbose:
            step = "[5]" if reset_first else "[4]"
            print(f"\n{step} Configuring debug freeze...")
        self.configure_debug_freeze()

        # Extend watchdog
        if self.verbose:
            step = "[6]" if reset_first else "[5]"
            print(f"\n{step} Extending watchdog timeout...")
        self.extend_watchdog()

        if self.verbose:
            print("\n" + "="*60)
            print("Setup complete - ready for unlimited-time debugging!")
            print("="*60)

    def prepare_for_flash_programming(self):
        """
        Prepare MCU for flash programming

        This function:
        - Halts CPU
        - Configures HSI to 24MHz
        - Loads flash timing from factory calibration
        - Unlocks flash
        """
        if self.verbose:
            print("\n" + "="*60)
            print("Preparing for flash programming...")
            print("="*60)

        # Halt CPU
        if self.verbose:
            print("\n[1] Halting CPU...")
        self.halt()

        # Configure ACR
        if self.verbose:
            print("\n[2] Configuring Flash ACR...")
        self.write_u32(self.FLASH_ACR, 0)

        # Unlock flash
        if self.verbose:
            print("\n[3] Unlocking flash...")
        self.flash_unlock()

        # Configure HSI to 24MHz
        if self.verbose:
            print("\n[4] Configuring HSI to 24MHz...")
        self.configure_hsi_24mhz()

        # Configure flash timing
        if self.verbose:
            print("\n[5] Configuring flash timing...")
        self.flash_configure_timing_24mhz()

        if self.verbose:
            print("\n" + "="*60)
            print("Setup complete - ready for flash programming!")
            print("="*60)

    # ========================================================================
    # ADC Operations
    # ========================================================================

    def adc_enable(self):
        """
        Enable ADC peripheral

        Note: PY32F003 does not have an ADRDY flag - the ADC is ready to use
        after a brief stabilization delay following ADEN being set.
        """
        cr = self.read_u32(self.ADC_CR)

        # If already enabled, we are good
        if cr & self.ADC_CR_ADEN:
            if self.verbose:
                print("ADC already enabled")
            return

        # 1. Clear any pending ready flag (bit 0 is reserved in PY32F003, but harmless)
        self.write_u32(self.ADC_ISR, 1)

        # 2. Enable ADC
        cr |= self.ADC_CR_ADEN
        self.write_u32(self.ADC_CR, cr)

        # 3. Wait for stabilization
        # Hardware would set ADRDY if implemented, but since it's reserved,
        # we use a fixed delay (datasheet specifies ADC stabilization time)
        time.sleep(0.002)

        # Verify ADEN is set
        cr = self.read_u32(self.ADC_CR)
        if cr & self.ADC_CR_ADEN:
            if self.verbose:
                print("ADC enabled and ready")
        else:
            raise RuntimeError("Failed to enable ADC (ADEN not set)")

    def adc_disable(self):
        """
        Disable ADC peripheral

        Properly stops any ongoing conversion before disabling to prevent
        leaving the ADC in an undefined state.
        """
        cr = self.read_u32(self.ADC_CR)

        # Check if ADC is enabled
        if not (cr & self.ADC_CR_ADEN):
            if self.verbose:
                print("ADC already disabled")
            return

        # Stop any ongoing conversion first
        if cr & self.ADC_CR_ADSTART:
            cr |= self.ADC_CR_ADSTP
            self.write_u32(self.ADC_CR, cr)

            # Wait for ADSTART to clear
            timeout = 100
            for _ in range(timeout):
                cr = self.read_u32(self.ADC_CR)
                if not (cr & self.ADC_CR_ADSTART):
                    break
                time.sleep(0.001)

        # Disable ADC
        cr = self.read_u32(self.ADC_CR)
        cr &= ~self.ADC_CR_ADEN
        self.write_u32(self.ADC_CR, cr)

        # Wait for disable to take effect
        time.sleep(0.001)

        if self.verbose:
            print("ADC disabled")

    def adc_calibrate(self):
        """
        Calibrate ADC

        CRITICAL: ADC must be DISABLED (ADEN=0) before calibration can be performed.
        This is a hardware requirement - calibration will not work if ADC is enabled.

        After calibration completes, the hardware requires waiting at least 4 ADC
        clock cycles before enabling the ADC.
        """
        # 1. Ensure ADC is DISABLED (required for calibration)
        cr = self.read_u32(self.ADC_CR)
        if cr & self.ADC_CR_ADEN:
            if self.verbose:
                print("Disabling ADC for calibration...")
            self.adc_disable()

        # 2. Start calibration
        cr = self.read_u32(self.ADC_CR)
        cr |= self.ADC_CR_ADCAL
        self.write_u32(self.ADC_CR, cr)

        # 3. Wait for calibration to complete (ADCAL bit cleared by hardware)
        timeout = 1000
        for _ in range(timeout):
            cr = self.read_u32(self.ADC_CR)
            if not (cr & self.ADC_CR_ADCAL):
                if self.verbose:
                    print("ADC calibration complete")

                # 4. CRITICAL: Wait at least 4 ADC clock cycles after calibration
                # before enabling. 1ms is more than sufficient for any ADC clock.
                time.sleep(0.001)
                return
            time.sleep(0.001)

        raise RuntimeError("ADC calibration timeout")

    def adc_init(self, resolution_bits=12):
        """
        Initialize ADC with correct sequence

        Performs complete initialization:
        1. Disables ADC if currently enabled
        2. Configures resolution and clock mode (while disabled)
        3. Performs calibration (must be done while disabled)
        4. Enables ADC

        Args:
            resolution_bits: ADC resolution (6, 8, 10, or 12 bits). Default 12.

        This should be called once before using ADC functions.
        """
        if self.verbose:
            print("Initializing ADC...")

        # 1. Disable ADC if currently enabled
        self.adc_disable()

        # 2a. Configure Resolution in CFGR1 (must be done while disabled)
        cfgr1 = self.read_u32(self.ADC_CFGR1)
        cfgr1 &= ~self.ADC_CFGR1_RES_MASK  # Clear resolution bits

        if resolution_bits == 12:
            cfgr1 |= self.ADC_CFGR1_RES_12BIT
        elif resolution_bits == 10:
            cfgr1 |= self.ADC_CFGR1_RES_10BIT
        elif resolution_bits == 8:
            cfgr1 |= self.ADC_CFGR1_RES_8BIT
        elif resolution_bits == 6:
            cfgr1 |= self.ADC_CFGR1_RES_6BIT
        else:
            raise ValueError(f"Invalid resolution: {resolution_bits}. Must be 6, 8, 10, or 12")

        self.write_u32(self.ADC_CFGR1, cfgr1)

        # 2b. Configure Clock Mode in CFGR2 (must be done while disabled)
        # CKMODE bits [31:30] in ADC_CFGR2
        # 00: Asynchronous clock mode (default)
        # 01: Synchronous clock mode (PCLK/2)
        # 10: Synchronous clock mode (PCLK/4)
        # 11: Reserved
        # Default (0) uses asynchronous mode which is typically fine
        cfgr2 = self.read_u32(self.ADC_CFGR2)
        cfgr2 &= ~(0x3 << 30)  # Clear CKMODE bits
        # Leave as 0 (asynchronous) for maximum compatibility
        self.write_u32(self.ADC_CFGR2, cfgr2)

        # 3. Calibrate (must be done while disabled)
        self.adc_calibrate()

        # 4. Enable ADC
        self.adc_enable()

        if self.verbose:
            print(f"ADC initialization complete ({resolution_bits}-bit resolution)")

    def adc_get_resolution_bits(self):
        """
        Get current ADC resolution in bits

        Returns:
            int: Current resolution (6, 8, 10, or 12 bits)
        """
        cfgr1 = self.read_u32(self.ADC_CFGR1)
        res_bits = (cfgr1 >> 3) & 0b11

        res_map = {
            0b00: 12,
            0b01: 10,
            0b10: 8,
            0b11: 6,
        }
        return res_map[res_bits]

    def adc_get_max_value(self):
        """
        Get maximum ADC value based on current resolution

        Returns:
            int: Maximum value (63 for 6-bit, 255 for 8-bit, 1023 for 10-bit, 4095 for 12-bit)
        """
        resolution = self.adc_get_resolution_bits()
        return (1 << resolution) - 1

    def adc_select_channel(self, channel):
        """
        Select ADC channel for conversion

        Args:
            channel: Channel number (0-13) or channel mask (e.g., ADC_CHANNEL_0)
        """
        if isinstance(channel, int) and channel < 16:
            # Convert channel number to bit mask
            channel_mask = (1 << channel)
        else:
            # Use the mask directly
            channel_mask = channel

        self.write_u32(self.ADC_CHSELR, channel_mask)

    def adc_read_channel(self, channel, configure_pin=True):
        """
        Read a single ADC channel

        This method properly handles channel switching by:
          1. Stopping any ongoing conversion
          2. Ensuring single-conversion mode (not continuous)
          3. Selecting only the requested channel
          4. Starting a fresh conversion
          5. Reading the result

        Args:
            channel: Channel number (0-13)
                    0-7: PA0-PA7
                    8-9: PB0-PB1
                    10: Internal temperature sensor
                    11: Internal voltage reference (VREFINT)
                    12: PA13 (SWDIO)
                    13: PA14 (SWCLK)
            configure_pin: If True, configure corresponding GPIO pin as analog input
                          (only for external channels, ignored for internal channels)

        Returns:
            int: ADC value (0-4095 for 12-bit ADC)
        """
        # Pin mapping for external channels only
        pin_mapping = {
            0: ('A', 0),   # CH0 = PA0
            1: ('A', 1),   # CH1 = PA1
            2: ('A', 2),   # CH2 = PA2
            3: ('A', 3),   # CH3 = PA3
            4: ('A', 4),   # CH4 = PA4
            5: ('A', 5),   # CH5 = PA5
            6: ('A', 6),   # CH6 = PA6
            7: ('A', 7),   # CH7 = PA7
            8: ('B', 0),   # CH8 = PB0
            9: ('B', 1),   # CH9 = PB1
            12: ('A', 13), # CH12 = PA13 (SWDIO)
            13: ('A', 14), # CH13 = PA14 (SWCLK)
        }
        # Internal channels (10, 11) do not require pin configuration

        # Configure pin as analog if requested
        if configure_pin and channel in pin_mapping:
            port, pin = pin_mapping[channel]
            self.gpio_set_mode(port, pin, 3)  # Mode 3 = analog

        # Stop any ongoing conversion and clear all flags
        cr = self.read_u32(self.ADC_CR)
        if cr & self.ADC_CR_ADSTART:
            # Stop the conversion by setting ADSTP
            cr |= self.ADC_CR_ADSTP
            self.write_u32(self.ADC_CR, cr)

            # Wait for ADSTART to clear
            timeout = 100
            for _ in range(timeout):
                cr = self.read_u32(self.ADC_CR)
                if not (cr & self.ADC_CR_ADSTART):
                    break
                time.sleep(0.001)

        # Clear all status flags (only write to defined bits)
        # Bits: EOSMP(1), EOC(2), EOSEQ(3), OVR(4), AWD(7)
        # Do NOT write to reserved bits (0, 5, 6, 8+) as this causes undefined behavior
        clear_mask = self.ADC_ISR_EOSMP | self.ADC_ISR_EOC | self.ADC_ISR_EOSEQ | self.ADC_ISR_OVR | self.ADC_ISR_AWD
        self.write_u32(self.ADC_ISR, clear_mask)

        # Ensure ADC is in single-conversion mode (not continuous)
        cfgr1 = self.read_u32(self.ADC_CFGR1)
        if cfgr1 & (1 << 13):  # CONT bit set (continuous mode)
            cfgr1 &= ~(1 << 13)  # Clear CONT bit
            self.write_u32(self.ADC_CFGR1, cfgr1)

        # Select ONLY the requested channel (clear all others)
        self.adc_select_channel(channel)

        # Start a fresh single conversion
        cr = self.read_u32(self.ADC_CR)
        cr |= self.ADC_CR_ADSTART
        self.write_u32(self.ADC_CR, cr)

        # Wait for conversion to complete (EOC flag)
        timeout = 1000
        for _ in range(timeout):
            isr = self.read_u32(self.ADC_ISR)
            if isr & self.ADC_ISR_EOC:
                # Read data register (this clears EOC on some MCUs)
                # Mask based on current resolution to handle 6/8/10/12-bit modes
                max_value = self.adc_get_max_value()
                value = self.read_u32(self.ADC_DR) & max_value

                # In single-conversion mode, ADSTART clears automatically when conversion completes
                # Do NOT use ADSTP - on PY32F003 it can disable the ADC (clear ADEN)
                # Just wait for ADSTART to clear naturally
                for _ in range(100):
                    cr = self.read_u32(self.ADC_CR)
                    if not (cr & self.ADC_CR_ADSTART):
                        break
                    time.sleep(0.001)

                # Clear EOC and any other flags (only defined bits)
                clear_mask = self.ADC_ISR_EOSMP | self.ADC_ISR_EOC | self.ADC_ISR_EOSEQ | self.ADC_ISR_OVR | self.ADC_ISR_AWD
                self.write_u32(self.ADC_ISR, clear_mask)

                return value
            time.sleep(0.001)

        raise RuntimeError("ADC conversion timeout")

    def adc_read_voltage(self, channel, vref=3.3):
        """
        Read ADC channel and convert to voltage

        Args:
            channel: Channel number (0-13)
            vref: Reference voltage in volts (default 3.3V)

        Returns:
            float: Voltage in volts
        """
        raw_value = self.adc_read_channel(channel)
        max_value = self.adc_get_max_value()
        voltage = (raw_value / float(max_value)) * vref
        return voltage

    # ========================================================================
    # Dynamic Pin Access
    # ========================================================================

    def __getattr__(self, name):
        """
        Dynamically create Pin objects for GPIO access

        This allows elegant syntax like:
            mcu.PA0.high()
            mcu.PB5.toggle()
            value = mcu.PA7.read()

        Supports: PA0-PA15, PB0-PB15, PF0-PF3
        """
        # Match pattern like PA0, PB5, PF2
        match = re.match(r'^P([ABF])(\d+)$', name)
        if match:
            port, pin_str = match.groups()
            pin = int(pin_str)

            # Validate pin number
            if port in ('A', 'B') and 0 <= pin <= 15:
                return Pin(self, port, pin)
            elif port == 'F' and 0 <= pin <= 3:
                return Pin(self, port, pin)

        # Not a pin - raise normal AttributeError
        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")


# ========================================================================
# Automatic Cleanup on Interpreter Exit
# ========================================================================

def _cleanup_all_connections():
    """
    Cleanup handler called when Python interpreter exits.

    This ensures all active debug connections are properly closed,
    preventing devices from being left in a halted state.
    """
    for mcu in list(_active_connections):
        try:
            if not mcu._closed:
                mcu.close()
        except:
            pass  # Silently ignore errors during shutdown


# Register cleanup handler to run on interpreter exit
atexit.register(_cleanup_all_connections)


# Convenience alias
PY32F030 = PY32F003


if __name__ == '__main__':
    # Example usage
    print("PY32F003 Interactive Debugger Class")
    print("="*60)
    print("\nExample usage:")
    print()
    print("from utils.pyrsp_py32f003 import PY32F003")
    print()
    print("with PY32F003() as mcu:")
    print("    # CPU control")
    print("    mcu.halt()")
    print("    pc = mcu.read_pc()")
    print("    print(f'PC: 0x{pc:08X}')")
    print()
    print("    # GPIO control")
    print("    mcu.gpio_set_output('A', 1)")
    print("    mcu.gpio_set_high('A', 1)")
    print()
    print("    # Memory access")
    print("    value = mcu.read_u32(mcu.GPIOA_ODR)")
    print(f"    print(f'GPIOA_ODR: 0x{{value:04X}}')")
    print()
    print("    # Register access")
    print("    mcu.dump_registers()  # Show all registers")
    print("    lr = mcu.read_register('lr')  # Read link register")
    print()
    print("="*60)
