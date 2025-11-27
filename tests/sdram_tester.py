# Copyright 2015-2025 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.
# SDRAM Tester for ISSI IS42S16400 Synchronous DRAM with exact pin interface
import Pyxsim as px
from typing import Sequence, Dict, List, Optional
from numbers import Number
# from clock import Clock
import time
import random

def nor_gate(xsi: px.pyxsim.Xsi, ports: str):
    if ports.find("NOR_GATE:") == -1:
        return xsi.sample_port_pins(ports)
    else:
        port_array = ports.replace("NOR_GATE:","").split("|")
        ret = 0
        for p in port_array:
            ret |= xsi.sample_port_pins(p)
        ret = not ret
        return ret
        

class SDRAMTester(px.SimThread):
    """
    This simulator thread acts as the ISSI IS42S16400 SDRAM device.
    It supports all SDRAM operations with timing characteristics matching the real device.
    
    EXACT INTERFACE (as specified):
    - RAS_N, CAS_N, WE_N (active low control signals)
    - DQML, DQMH (data mask pins for low/high bytes)
    - CKE (clock enable)
    - DQ0-15 (16-bit data bus)
    - A0-12 (13 address lines)
    - BA0, BA1 (2 bank address lines)
    
    NO CS pin - this simulates the SDRAM being always selected (typical for many XMOS designs)
    """
    
    # SDRAM Commands (based on RAS_N, CAS_N, WE_N levels)
    CMD_NOOP = 0x0     # RAS_N=1, CAS_N=1, WE_N=1
    CMD_ACTIVE = 0x1   # RAS_N=0, CAS_N=1, WE_N=1 - row activate
    CMD_READ = 0x2     # RAS_N=1, CAS_N=0, WE_N=1 - read
    CMD_WRITE = 0x3    # RAS_N=1, CAS_N=0, WE_N=0 - write
    CMD_PRECHARGE = 0x5 # RAS_N=0, CAS_N=1, WE_N=0 - precharge
    CMD_AUTO_REFRESH = 0x6 # RAS_N=0, CAS_N=0, WE_N=1 - auto refresh
    CMD_LOAD_MODE = 0x7 # RAS_N=0, CAS_N=0, WE_N=0 - load mode register
    
    def __init__(self,
                 clk: str,
                 cke: str,
                 ras_n: str,
                 cas_n: str,
                 we_n: str,
                 dqml: str,      # Data Mask Low (DQ0-DQ7)
                 dqmh: str,      # Data Mask High (DQ8-DQ15)
                 addr: Sequence[str], # Address pins [A0..A12]
                 ba0: str,       # Bank Address 0
                 ba1: str,       # Bank Address 1
                 dq: Sequence[str],   # Data pins [DQ0..DQ15]
                #  c: Clock,
                 speed_grade: str = "166",  # "166" for -6 grade, "143" for -7 grade
                 memory_size_mb: int = 4,   # Memory size per bank in MB (simplified)
                 finish_cb=None):
        """
        Initialize the SDRAM tester with exact ISSI IS42S16400 pin interface.
        
        Args:
            clk: Clock pin name
            cke: Clock enable pin name
            ras_n: Row address strobe (active low)
            cas_n: Column address strobe (active low) 
            we_n: Write enable (active low)
            dqml: Data mask low (for DQ0-DQ7)
            dqmh: Data mask high (for DQ8-DQ15)
            addr: Address pins [A0..A12] (13 address lines)
            ba0: Bank address bit 0
            ba1: Bank address bit 1
            dq: Data pins [DQ0..DQ15] (16-bit data bus)
            c: Clock object
            speed_grade: Speed grade - "166" for 166MHz (-6), "143" for 143MHz (-7)
            memory_size_mb: Memory size per bank in MB (simplified for simulation)
            finish_cb: Callback function when simulation completes
        """
        self._clk = clk
        self._cke = cke
        self._ras_n = ras_n
        self._cas_n = cas_n
        self._we_n = we_n
        self._dqml = dqml    # DQM Low - masks DQ0-DQ7
        self._dqmh = dqmh    # DQM High - masks DQ8-DQ15
        self._addr = addr
        self._ba0 = ba0
        self._ba1 = ba1
        self._dq = dq
        # self._clk_obj = c
        self._finish_cb = finish_cb
        
        # Validate pin counts
        if len(self._addr) != 13:
            raise ValueError(f"IS42S16400 requires 13 address pins (A0-A12), got {len(self._addr)}")
        if len(self._dq) != 16:
            raise ValueError(f"IS42S16400 requires 16 data pins (DQ0-DQ15), got {len(self._dq)}")
        
        # SDRAM timing parameters based on IS42S16400 datasheet
        # Values in clock cycles for -6 grade (166MHz)
        self._timing = {
            'tRC': 10,   # Row cycle time (min 55ns @ 166MHz = 9.13 cycles -> 10 cycles)
            'tRCD': 3,   # RAS to CAS delay (min 18ns @ 166MHz = 3 cycles)
            'tRP': 3,    # Row precharge time (min 18ns @ 166MHz = 3 cycles)
            'tRAS': 8,   # Row active time (min 42ns @ 166MHz = 7 cycles -> 8 cycles)
            'tWR': 2,    # Write recovery time (min 12ns @ 166MHz = 2 cycles)
            'tRFC': 11,  # Refresh cycle time (min 66ns @ 166MHz = 11 cycles)
            'tWTR': 2,   # Write to read delay (min 10ns @ 166MHz = 2 cycles)
            'tRRD': 2,   # Row to row delay (min 12ns @ 166MHz = 2 cycles)
            'tDPL': 1,   # Data-in to precharge delay (min 1 cycle)
            'tDAL': 3,   # Data-in to active delay (tWR + tRP)
        }
        
        # CAS latency based on speed grade
        if speed_grade == "166":
            self._cas_latency = 3  # 3 cycles for 166MHz
            self._clock_period = 6.0  # ns (166MHz)
        elif speed_grade == "143":
            self._cas_latency = 2  # 2 cycles for 143MHz
            self._clock_period = 7.0  # ns (143MHz)
        else:
            raise ValueError(f"Unsupported speed grade: {speed_grade}")
        
        # SDRAM internal state
        self._bank_active = [False] * 4  # 4 banks
        self._bank_row = [0] * 4         # Active row in each bank
        self._mode_register = 0x0000     # Default mode register value
        self._burst_length = 1           # Default burst length (1, 2, 4, 8)
        self._burst_type = 0             # 0=sequential, 1=interleaved
        self._refresh_count = 0          # Refresh counter
        self._output_enabled = True      # Global output enable state
        
        # Memory array - simplified for simulation
        self._bytes_per_bank = memory_size_mb * 1024 * 1024
        self._memory_size_words = self._bytes_per_bank // 2  # 16-bit words
        self._memory = [bytearray(self._bytes_per_bank) for _ in range(4)]  # 4 banks
        
        # Timing tracking
        self._last_active_time = {i: 0 for i in range(4)}  # Last active command time per bank
        self._last_precharge_time = {i: 0 for i in range(4)}  # Last precharge time per bank
        self._last_refresh_time = 0  # Last refresh time
        self._last_write_time = 0    # Last write time for tWR timing
        
        # Operation tracking
        self._read_queue = []  # [(bank, row, col, burst_length, dqm_low, dqm_high, start_time), ...]
        self._write_queue = []  # [(bank, row, col, data, dqm_low, dqm_high, start_time), ...]
        
        # DQM state tracking
        self._dqml_state = 0  # Current state of DQML pin (0=active, 1=masked)
        self._dqmh_state = 0  # Current state of DQMH pin (0=active, 1=masked)
        self._dqm_sample_time = 0 # When DQM was last sampled
        
        # Statistics
        self._cmd_count = {cmd: 0 for cmd in ['ACTIVE', 'READ', 'WRITE', 'PRECHARGE', 'REFRESH', 'MODE']}
        self._error_count = 0
        self._dqm_mask_count = 0  # Count of DQM mask operations
        self._total_cycles = 0
        self._simulation_start_time = time.time()

    def _get_command(self, xsi: px.pyxsim.Xsi) -> int:
        """Decode the current command based on control signals (active low)"""
        ras_val = xsi.sample_port_pins(self._ras_n)
        cas_val = xsi.sample_port_pins(self._cas_n)
        we_val = xsi.sample_port_pins(self._we_n)
        
        # Convert active low signals to command decoding
        # Command decoding: RAS_N, CAS_N, WE_N (active low)
        cmd_bits = (ras_val << 2) | (cas_val << 1) | we_val
        return cmd_bits

    def _get_bank_address(self, xsi: px.pyxsim.Xsi) -> int:
        """Get the bank address from BA0 and BA1 pins"""
        ba0_val = xsi.sample_port_pins(self._ba0)
        ba1_val = xsi.sample_port_pins(self._ba1)
        return (ba1_val << 1) | ba0_val

    def _get_address(self, xsi: px.pyxsim.Xsi) -> int:
        """Get the address from address pins A0-A12"""
        addr_val = 0
        for i in range(13):  # A0-A12 (13 bits)
            if i < len(self._addr):
                addr_val |= (xsi.sample_port_pins(self._addr[i]) << i)
        return addr_val

    def _get_dqm_state(self, xsi: px.pyxsim.Xsi) -> tuple[int, int]:
        """Get the current DQM pin states (active low: 0=active, 1=masked)"""
        dqml_val = nor_gate(xsi, self._dqml)
        dqmh_val = nor_gate(xsi, self._dqmh)
        return (dqml_val, dqmh_val)

    def _get_data(self, xsi: px.pyxsim.Xsi) -> int:
        """Get data from DQ pins DQ0-DQ15"""
        data_val = 0
        for i in range(16):  # DQ0-DQ15 (16 bits)
            if i < len(self._dq):
                data_val |= (xsi.sample_port_pins(self._dq[i]) << i)
        return data_val

    def _set_data(self, xsi: px.pyxsim.Xsi, data: int, dqml_mask: int, dqmh_mask: int):
        """
        Set data on DQ pins with DQM mask control.
        DQM pins are active low: 0 = drive data, 1 = high-Z (masked)
        
        Args:
            dqml_mask: 0 = drive DQ0-DQ7, 1 = high-Z DQ0-DQ7
            dqmh_mask: 0 = drive DQ8-DQ15, 1 = high-Z DQ8-DQ15
        """
        for i, pin in enumerate(self._dq):
            if i >= 16:  # 16-bit data bus
                break
            
            # Determine which DQM controls this byte
            if i < 8:  # DQ0-DQ7 (low byte) - controlled by DQML
                dqm_bit = dqml_mask
            else:      # DQ8-DQ15 (high byte) - controlled by DQMH
                dqm_bit = dqmh_mask
            
            if dqm_bit == 1:
                # DQM active (masked) - set to high-Z
                xsi.sample_port_pins(pin)
            else:
                # DQM inactive (active) - drive data
                xsi.drive_port_pins(pin, (data >> i) & 1)

    def _set_all_data_high_z(self, xsi: px.pyxsim.Xsi):
        """Set all DQ pins to high-Z (tri-state)"""
        for pin in self._dq:
            xsi.sample_port_pins(pin)

    def _check_timing_constraints(self, bank: int, command: int) -> bool:
        """Check if timing constraints are satisfied for the given command"""
        current_time = self._total_cycles
        
        if command == self.CMD_ACTIVE:
            # Check tRC (Row Cycle Time) - time since last precharge
            if self._last_precharge_time[bank] > 0 and current_time - self._last_precharge_time[bank] < self._timing['tRC']:
                print(f"Timing violation (cycle {current_time}): tRC not met for bank {bank} "
                      f"({current_time - self._last_precharge_time[bank]} < {self._timing['tRC']})")
                self._error_count += 1
                return False
            
            # Check tRRD (Row-to-Row Delay) - time since last active on any bank
            for b in range(4):
                if b != bank and self._last_active_time[b] > 0:
                    if current_time - self._last_active_time[b] < self._timing['tRRD']:
                        print(f"Timing violation (cycle {current_time}): tRRD not met between banks {b} and {bank} "
                              f"({current_time - self._last_active_time[b]} < {self._timing['tRRD']})")
                        self._error_count += 1
                        return False
        
        elif command == self.CMD_READ or command == self.CMD_WRITE:
            # Check if bank is active
            if not self._bank_active[bank]:
                print(f"Timing violation (cycle {current_time}): Bank {bank} not active for read/write")
                self._error_count += 1
                return False
            
            # Check tRCD (RAS to CAS Delay) - time since last active
            if current_time - self._last_active_time[bank] < self._timing['tRCD']:
                print(f"Timing violation (cycle {current_time}): tRCD not met for bank {bank} "
                      f"({current_time - self._last_active_time[bank]} < {self._timing['tRCD']})")
                self._error_count += 1
                return False
        
        elif command == self.CMD_WRITE:
            # Additional write timing checks
            if self._last_write_time > 0 and current_time - self._last_write_time < self._timing['tDPL']:
                print(f"Timing violation (cycle {current_time}): tDPL not met after last write "
                      f"({current_time - self._last_write_time} < {self._timing['tDPL']})")
                self._error_count += 1
                return False
        
        elif command == self.CMD_PRECHARGE:
            # Check tRAS (Row Active Time) - time since last active
            if self._bank_active[bank] and current_time - self._last_active_time[bank] < self._timing['tRAS']:
                print(f"Timing violation (cycle {current_time}): tRAS not met for bank {bank} "
                      f"({current_time - self._last_active_time[bank]} < {self._timing['tRAS']})")
                self._error_count += 1
                return False
        
        elif command == self.CMD_AUTO_REFRESH:
            # Check tRFC (Refresh Cycle Time)
            if self._last_refresh_time > 0 and current_time - self._last_refresh_time < self._timing['tRFC']:
                print(f"Timing violation (cycle {current_time}): tRFC not met "
                      f"({current_time - self._last_refresh_time} < {self._timing['tRFC']})")
                self._error_count += 1
                return False
        
        return True

    def _calculate_memory_address(self, bank: int, row: int, col: int) -> int:
        """Calculate the linear memory address from bank, row, and column"""
        # IS42S16400 organization: 4 banks × 2048 rows × 256 columns × 16 bits
        # For simulation, we use simplified addressing
        rows_per_bank = 2048    # 2K rows per bank
        cols_per_row = 256     # 256 columns per row
        
        # Calculate row and column within bounds
        row_addr = row % rows_per_bank
        col_addr = col % cols_per_row
        
        # Each column is 16 bits (2 bytes)
        bytes_per_column = 2
        
        # Calculate address in bytes
        bank_offset = bank * rows_per_bank * cols_per_row * bytes_per_column
        row_offset = row_addr * cols_per_row * bytes_per_column
        col_offset = col_addr * bytes_per_column
        
        address = bank_offset + row_offset + col_offset
        
        # Ensure address stays within bounds
        return address % self._bytes_per_bank

    def _execute_command(self, xsi: px.pyxsim.Xsi, command: int, bank: int, addr: int):
        """Execute the decoded SDRAM command with DQM support"""
        current_time = self._total_cycles
        dqml_val, dqmh_val = self._get_dqm_state(xsi)
        self._dqml_state = dqml_val
        self._dqmh_state = dqmh_val
        self._dqm_sample_time = current_time
        
        if not self._check_timing_constraints(bank, command):
            return
        
        if command == self.CMD_ACTIVE:
            row = addr & 0x7FF  # 11-bit row address (A10-A0) for IS42S16400
            if self._bank_active[bank]:
                # Precharge bank first if already active
                self._bank_active[bank] = False
            
            self._bank_active[bank] = True
            self._bank_row[bank] = row
            self._last_active_time[bank] = current_time
            self._cmd_count['ACTIVE'] += 1
            # print(f"[{current_time}] ACTIVE: Bank {bank}, Row {row:03X}")
        
        elif command == self.CMD_READ:
            col = addr & 0xFF  # 8-bit column address (A7-A0) for IS42S16400
            auto_precharge = (addr >> 10) & 0x1  # A10 selects auto-precharge
            
            if self._bank_active[bank]:
                # Schedule read data to appear after CAS latency
                read_time = current_time + self._cas_latency
                
                # DQM for read operations controls output enable
                # DQM=1 (masked) means output is disabled (high-Z), DQM=0 means output is enabled
                self._read_queue.append((bank, self._bank_row[bank], col, self._burst_length, 
                                       dqml_val, dqmh_val, read_time))
                
                # Set DQ pins to high-Z immediately (before data arrives)
                self._set_all_data_high_z(xsi)
                
                self._cmd_count['READ'] += 1
                print(f"[{current_time}] READ: Bank {bank}, Row {self._bank_row[bank]:03X}, Col {col:02X}, "
                      f"BL {self._burst_length}, DQML={dqml_val}, DQMH={dqmh_val}, AutoPrecharge={auto_precharge}")
            else:
                print(f"[{current_time}] Error: Read to inactive bank {bank}")
                self._error_count += 1
        
        elif command == self.CMD_WRITE:
            col = addr & 0xFF  # 8-bit column address (A7-A0)
            auto_precharge = (addr >> 10) & 0x1  # A10 selects auto-precharge
            data = self._get_data(xsi)
            
            if self._bank_active[bank]:
                # DQM during write operations masks the corresponding data bytes
                # DQM=1 (masked) means the byte is NOT written, DQM=0 means the byte IS written
                # DQML masks DQ0-DQ7 (LSB byte), DQMH masks DQ8-DQ15 (MSB byte)
                self._write_queue.append((bank, self._bank_row[bank], col, data, 
                                        dqml_val, dqmh_val, current_time))
                
                self._last_write_time = current_time
                self._cmd_count['WRITE'] += 1
                
                # Count DQM mask operations for statistics
                if dqml_val == 1 or dqmh_val == 1:
                    self._dqm_mask_count += 1
                
                print(f"[{current_time}] WRITE: Bank {bank}, Row {self._bank_row[bank]:03X}, Col {col:02X}, "
                      f"Data {data:04X}, DQML={dqml_val}, DQMH={dqmh_val}, AutoPrecharge={auto_precharge}")
            else:
                print(f"[{current_time}] Error: Write to inactive bank {bank}")
                self._error_count += 1
        
        elif command == self.CMD_PRECHARGE:
            if addr & 0x400:  # A10=1 means precharge all banks
                for b in range(4):
                    if self._bank_active[b]:
                        self._bank_active[b] = False
                        self._last_precharge_time[b] = current_time
                print(f"[{current_time}] PRECHARGE ALL BANKS")
            else:
                if self._bank_active[bank]:
                    self._bank_active[bank] = False
                    self._last_precharge_time[bank] = current_time
                print(f"[{current_time}] PRECHARGE: Bank {bank}")
            self._cmd_count['PRECHARGE'] += 1
        
        elif command == self.CMD_AUTO_REFRESH:
            self._refresh_count = (self._refresh_count + 1) % 4096  # 4096 refresh cycles required
            self._last_refresh_time = current_time
            self._cmd_count['REFRESH'] += 1
            print(f"[{current_time}] AUTO REFRESH: Cycle {self._refresh_count}/4096")
        
        elif command == self.CMD_LOAD_MODE:
            # IS42S16400 mode register format:
            # M9 M8 M7 M6 M5 M4 M3 M2 M1 M0
            # M2-M0: Burst Length (000=1, 001=2, 010=4, 011=8)
            # M3: Burst Type (0=sequential, 1=interleaved)
            # M6-M4: CAS Latency (010=2, 011=3)
            # M9,M8,M7,M5: Reserved (0)
            self._mode_register = addr & 0x3FF  # 10-bit mode register
            
            # Decode mode register
            burst_length_code = self._mode_register & 0x7
            self._burst_length = [1, 2, 4, 8, 0, 0, 0, 0][burst_length_code]
            if self._burst_length == 0:
                self._burst_length = 1  # 0 means single location
            
            self._burst_type = (self._mode_register >> 3) & 0x1
            
            cas_latency_code = (self._mode_register >> 4) & 0x7
            # Set CAS latency based on speed grade
            if self._clock_period <= 7.0:  # 143MHz or faster
                self._cas_latency = 3 if cas_latency_code >= 3 else 2
            else:  # Slower speeds
                self._cas_latency = 2
            
            self._cmd_count['MODE'] += 1
            # print(f"[{current_time}] LOAD MODE: BL={self._burst_length}, BT={self._burst_type}, "
            #       f"CL={self._cas_latency}, MR={self._mode_register:03X}")
        
        else:
            print(f"[{current_time}] Unknown command: {command:03b} (RAS_N={xsi.sample_port_pins(self._ras_n)}, "
                  f"CAS_N={xsi.sample_port_pins(self._cas_n)}, WE_N={xsi.sample_port_pins(self._we_n)})")

    def _process_read_queue(self, xsi: px.pyxsim.Xsi):
        """Process pending read operations with DQM output control"""
        current_time = self._total_cycles
        completed_reads = []
        
        for i, read in enumerate(self._read_queue):
            bank, row, col, burst_length, dqml_mask, dqmh_mask, read_time = read
            
            if current_time >= read_time:
                # Generate read data for the entire burst
                for burst_offset in range(burst_length):
                    current_col = col + (burst_offset if self._burst_type == 0 else (burst_offset ^ (burst_offset >> 1)))
                    
                    # Calculate memory address
                    mem_addr = self._calculate_memory_address(bank, row, current_col)
                    
                    # Read 16-bit word from memory (with bounds checking)
                    data = 0
                    if mem_addr + 1 < self._bytes_per_bank:
                        data = (self._memory[bank][mem_addr + 1] << 8) | self._memory[bank][mem_addr]
                    
                    # Apply DQM mask for output enable
                    # DQM=1 (masked) means output is disabled (high-Z), DQM=0 means output is enabled
                    self._set_data(xsi, data, dqml_mask, dqmh_mask)
                    
                    print(f"[{current_time}] READ DATA[{burst_offset}]: Bank {bank}, Addr {mem_addr:06X}, "
                          f"Data {data:04X}, DQML={dqml_mask}, DQMH={dqmh_mask}")
                
                completed_reads.append(i)
        
        # Remove completed reads (in reverse order to avoid index issues)
        for i in sorted(completed_reads, reverse=True):
            self._read_queue.pop(i)

    def _process_write_queue(self, xsi: px.pyxsim.Xsi):
        """Process pending write operations with DQM data masking"""
        current_time = self._total_cycles
        completed_writes = []
        
        for i, write in enumerate(self._write_queue):
            bank, row, col, data, dqml_mask, dqmh_mask, write_time = write
            
            if current_time >= write_time:
                # Calculate memory address
                mem_addr = self._calculate_memory_address(bank, row, col)
                
                # Apply DQM mask for write operations
                # DQM=1 (masked) means the byte is NOT written, DQM=0 means the byte IS written
                # DQML masks DQ0-DQ7 (LSB byte), DQMH masks DQ8-DQ15 (MSB byte)
                
                # Write LSB byte (DQ0-DQ7) if not masked by DQML
                if dqml_mask == 0 and mem_addr < self._bytes_per_bank:  # DQML=0 means write enabled
                    self._memory[bank][mem_addr] = data & 0xFF
                    # print(f"WROTE: Bank {bank}, Addr {mem_addr:06X}, LSB {02X} (DQML={dqml_mask})")
                
                # Write MSB byte (DQ8-DQ15) if not masked by DQMH
                if dqmh_mask == 0 and mem_addr + 1 < self._bytes_per_bank:  # DQMH=0 means write enabled
                    self._memory[bank][mem_addr + 1] = (data >> 8) & 0xFF
                    # print(f"WROTE: Bank {bank}, Addr {mem_addr+1:06X}, MSB {02X} (DQMH={dqmh_mask})")
                
                completed_writes.append(i)
        
        # Remove completed writes (in reverse order to avoid index issues)
        for i in sorted(completed_writes, reverse=True):
            self._write_queue.pop(i)

    def _background_operations(self, xsi: px.pyxsim.Xsi):
        """Handle background operations like refresh requirement checking"""
        current_time = self._total_cycles
        
        # Refresh requirement: 64ms / 4096 = 15.625us per refresh
        # At 166MHz, 15.625us = 2593 cycles
        refresh_interval = 2600  # approx 15.625us at 166MHz
        
        # if current_time - self._last_refresh_time > refresh_interval:
        #     print(f"WARNING (cycle {current_time}): Refresh requirement not met! "
        #           f"Last refresh at cycle {self._last_refresh_time}, "
        #           f"delta={current_time - self._last_refresh_time} cycles")
        #     # Don't count as error since this is background requirement
        #     # self._error_count += 1

    def run(self):
        xsi: px.pyxsim.Xsi = self.xsi
        print("SDRAM Tester Started - Simulating ISSI IS42S16400 SDRAM")
        print(f"Interface: RAS_N, CAS_N, WE_N, DQML, DQMH, CKE, DQ0-15, A0-12, BA0, BA1")
        print(f"Speed Grade: {self._clock_period}ns ({1000/self._clock_period:.1f}MHz)")
        print(f"CAS Latency: {self._cas_latency}")
        print(f"Memory size: {self._bytes_per_bank//1024//1024}MB per bank")
        print(f"Total memory: {self._bytes_per_bank//1024//1024 * 4}MB")
        print("Waiting for clock activity...")
        
        # Initialize all DQ pins to high-Z (input mode)
        self._set_all_data_high_z(xsi)
        
        # Wait for first clock edge to synchronize
        self.wait_for_port_pins_change([self._clk])
        
        # Main simulation loop
        last_clk_val = xsi.sample_port_pins(self._clk)
        
        try:
            while True:
                # Wait for clock edge
                self.wait_for_port_pins_change([self._clk])
                clk_val = xsi.sample_port_pins(self._clk)
                
                # Process on rising edge (typical SDRAM behavior)
                if clk_val == 1 and last_clk_val == 0:
                    self._total_cycles += 1
                    
                    # Sample CKE signal
                    cke_val = 1 if self._cke == "3V3" else xsi.sample_port_pins(self._cke)
                    
                    if cke_val == 1:  # Clock enabled
                        command = self._get_command(xsi)
                        bank = self._get_bank_address(xsi)
                        addr = self._get_address(xsi)
                        
                        # Execute command if clock is enabled
                        self._execute_command(xsi, command, bank, addr)
                    
                    # Process pending operations
                    self._process_read_queue(xsi)
                    self._process_write_queue(xsi)
                    
                    # Handle background operations
                    self._background_operations(xsi)
                    
                    # # Periodically report status
                    # if self._total_cycles % 10000 == 0:
                    #     elapsed_time = time.time() - self._simulation_start_time
                    #     cycles_per_sec = self._total_cycles / elapsed_time if elapsed_time > 0 else 0
                    #     print(f"\n=== Simulation Status at cycle {self._total_cycles} ===")
                    #     print(f"Time: {elapsed_time:.2f}s, Speed: {cycles_per_sec/1e6:.2f}M cycles/sec")
                    #     print(f"Active banks: {[i for i, active in enumerate(self._bank_active) if active]}")
                    #     print(f"Command counts: {self._cmd_count}")
                    #     print(f"DQM mask operations: {self._dqm_mask_count}")
                    #     print(f"Timing errors: {self._error_count}")
                    #     print(f"Read queue: {len(self._read_queue)} pending")
                    #     print(f"Write queue: {len(self._write_queue)} pending")
                    #     print(f"Last refresh: {self._last_refresh_time}, Current: {self._total_cycles}")
                
                last_clk_val = clk_val
        
        except Exception as e:
            print(f"Simulation error: {e}")
            self._error_count += 1
        
        # Final statistics
        elapsed_time = time.time() - self._simulation_start_time
        print("\n" + "="*60)
        print("SDRAM Simulation Complete")
        print("="*60)
        print(f"Total cycles: {self._total_cycles:,}")
        print(f"Simulation time: {elapsed_time:.2f} seconds")
        print(f"Average speed: {self._total_cycles/elapsed_time/1e6:.2f}M cycles/sec")
        print(f"Command statistics: {self._cmd_count}")
        print(f"DQM mask operations: {self._dqm_mask_count}")
        print(f"Timing errors: {self._error_count}")
        print(f"Refresh cycles executed: {self._refresh_count}/4096")
        
        # Memory usage statistics
        memory_used = 0
        for bank in range(4):
            # Count non-zero bytes as used memory (simplified)
            used_bytes = sum(1 for b in self._memory[bank] if b != 0)
            memory_used += used_bytes
            print(f"Bank {bank} memory used: {used_bytes/1024:.1f}KB / {self._bytes_per_bank/1024:.1f}KB")
        
        print(f"Total memory used: {memory_used/1024/1024:.2f}MB / {self._bytes_per_bank*4/1024/1024:.2f}MB")
        
        # Call finish callback if provided
        if self._finish_cb:
            result = {
                'total_cycles': self._total_cycles,
                'cmd_counts': self._cmd_count,
                'dqm_mask_count': self._dqm_mask_count,
                'error_count': self._error_count,
                'refresh_count': self._refresh_count,
                'memory_used_bytes': memory_used,
                'simulation_time_sec': elapsed_time
            }
            self._finish_cb(result)
        
        # Terminate simulation
        print("Terminating simulation...")
        xsi.terminate()