# Copyright 2025 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.

import pytest
import Pyxsim
from Pyxsim import testers
from pathlib import Path
from sdram_tester import SDRAMTester

SDRAM_PORT_DQ = [f"tile[0]:XS1_PORT_16B.{i}" for i in range(16)]    # DQ0-DQ15
SDRAM_PORT_ADDR = [f"tile[0]:XS1_PORT_16B.{i}" for i in range(13)]  # A0-A12
SDRAM_PORT_BA = ["tile[0]:XS1_PORT_16B.13", "tile[0]:XS1_PORT_16B.14"]
SDRAM_PORT_CAS = "tile[0]:XS1_PORT_1J"
SDRAM_PORT_RAS = "tile[0]:XS1_PORT_1I"
SDRAM_PORT_WE = "tile[0]:XS1_PORT_1K"
SDRAM_PORT_CLK = "tile[0]:XS1_PORT_1L"

def test_sdram_benchmark(level, capfd):

    # if level == 'smoke':
    #     pytest.skip("level == 'smoke'")

    binary = Path(__file__).parent / "sdram_benchmark" / "bin" / "sdram_benchmark.xe"

    sdram_tester = SDRAMTester(
        clk = SDRAM_PORT_CLK,
        cke = "3V3",
        ras_n = SDRAM_PORT_RAS,
        cas_n = SDRAM_PORT_CAS,
        we_n = SDRAM_PORT_WE,
        dqml = f"NOR_GATE:{SDRAM_PORT_CAS}|{SDRAM_PORT_WE}",
        dqmh = f"NOR_GATE:{SDRAM_PORT_CAS}|{SDRAM_PORT_WE}",
        addr = SDRAM_PORT_ADDR,
        ba0 = SDRAM_PORT_BA[0],
        ba1 = SDRAM_PORT_BA[1],
        dq = SDRAM_PORT_DQ
    )

    max_cycles = 15000000

    simargs = [
        "--max-cycles",
        str(max_cycles),
    ]

    Pyxsim.run_on_simulator_(
        binary,
        do_xe_prebuild=False,
        simthreads=[sdram_tester],
        simargs=simargs,
        timeout=None
    )

    # result = Pyxsim.run_on_simulator(
    #     binary,
    #     do_xe_prebuild=False,
    #     # cmake=True,
    #     simargs=simargs,
    #     tester=None,
    #     simthreads=[sdram_tester],
    #     # capfd=capfd,
    #     # clean_before_build=False,
    #     timeout=None)
    
if __name__ == "__main__":
    test_sdram_benchmark(None, None)