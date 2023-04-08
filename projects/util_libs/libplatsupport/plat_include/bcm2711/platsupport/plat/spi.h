/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <platsupport/spi.h>

enum spi_id {
    SPI0 = 0,
    NSPI,
};

#define print_spi_regs(spi_regs) { \
    ZF_LOGE("cs: %08X, fifo: %08X, clk: %08X, dlen: %08X, ltoh: %08X, dc: %08X",  \
        spi_regs->cs, spi_regs->fifo, spi_regs->clk, spi_regs->dlen, spi_regs->ltoh, spi_regs->dc \
    ); \
}

int bcm2711_spi_init(enum spi_id id, volatile void* base, mux_sys_t* mux_sys, clock_sys_t* clock_sys, spi_bus_t** ret_spi_bus);
