/*
 * Author: Thomas Ingleby <thomas.c.ingleby@intel.com>
 * Author: Michael Ring <mail@michael-ring.org>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <mraa/common.h>
#include "mips/qca_mux_data.h"

#include "mraa_internal.h"

#include "common.h"

#define PLATFORM_8DEVICES_CARAMBOLA2	1
#define PLATFORM_8DEVICES_CENTIPEDE		2
#define PLATFORM_8DEVICES_LIMA			3
#define PLATFORM_8DEVICES_RAMBUTAN		4


#define QCA955X_GPIO_BASE   0xb8040000
#define QCA955X_GPIO_BLOCK_SIZE   0x70
#define QCA955X_GPIO_IN     0x4
#define QCA955X_GPIO_SET    0xc
#define QCA955X_GPIO_CLEAR  0x10
#define AR9331_GPIO_BLOCK_SIZE   0x30
#define MMAP_PATH          "/dev/mem"
#define OUT_FUNC_REG_ADDR 0x2c
#define IN_FUNC_REG_ADDR 0x44

// MMAP
static int8_t* mmap_reg = NULL;
static int mmap_fd = 0;
static int mmap_size;
static uint8_t* gpio_mmap_reg = NULL;
static int gpio_mmap_fd = 0;
static int gpio_mmap_size;
static unsigned int mmap_count = 0;
static int platform_detected = 0;

typedef struct pinmux {
    char name[MRAA_PIN_NAME_SIZE];
    int pinmap;
    mraa_pincapabilities_t caps;
} pinmux;

mraa_board_t* mraa_8dev_common(char *name, int platform, int pin_count, pinmux *mux);

pinmux rambutan_mux[] = {
    {"INVALID", 0, { 0, 0, 0, 0, 0, 0, 0, 0 }},
    //J1-1
    {"GPIO0",        0, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG
    {"GPIO1",        1, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG
    {"GPIO2",        2, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG
    {"GPIO3",        3, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG
    {"JTAG_RST",     0, { 1, 0, 0, 0, 0, 0, 0, 0 }},//JTAG
    {"SYS_RST_L",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB2_DM",      0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB2_DP",      0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    //J1-11
    {"GPIO11",      11, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO12",      12, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO13",      13, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO14",      14, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO15",      15, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO16",      16, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO18",      18, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO21",      21, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO22",      22, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO4",        4, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    //J2-1
    {"GPIO9",        9, { 1, 1, 0, 0, 0, 0, 0, 0 }},//UART
    {"GPIO10",      10, { 1, 1, 0, 0, 0, 0, 0, 0 }},//UART
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"NC?",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"NC?",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"3.3VD",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"3.3VD",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"5VD",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"5VD",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    //J2-11
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"2.5VD_IO",     0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"2.5VD_IO",     0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"PCIE_LED",     0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GPIO6",        6, { 1, 1, 0, 0, 0, 0, 0, 0 }},//SPI
    {"GPIO5",        5, { 1, 1, 0, 0, 0, 0, 0, 0 }},//SPI
    {"GPIO8",        8, { 1, 1, 0, 0, 0, 0, 0, 0 }},//SPI
    {"GPIO7",        7, { 1, 1, 0, 0, 0, 0, 0, 0 }},//SPI*/
};

pinmux carambola2_mux[] = {
    {"INVALID", 0, { 0, 0, 0, 0, 0, 0, 0, 0 }},
    //J12-1
    {"GPIO17",      17, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED6
    {"GPIO16",      16, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED5
    {"GPIO15",      15, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED4
    {"GPIO14",      14, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED3
    {"GPIO13",      13, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED2
    {"GPIO1",        1, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED1
    {"GPIO0",        0, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED0
    {"GPIO11",      11, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO12",      12, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO18",      18, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    //J12-11
    {"GPIO19",      19, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO20",      20, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO21",      21, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO22",      22, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO23",      23, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"3.3VD",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"3.3VD",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"5VD",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"5VD",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    //J13-1
    {"UART_TX",      9, { 1, 1, 0, 0, 0, 0, 0, 1 }},//SPI_CS2
    {"UART_RX",     10, { 1, 1, 0, 0, 0, 0, 0, 1 }},//SPI_CS1
    {"UART_TX_C",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"UART_RX_C",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB+",         0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB-",         0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB+_C",       0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB-_C",       0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    //J13-11
    {"SPI_CS0",      2, { 1, 1, 0, 0, 1, 0, 0, 0 }},
    {"SPI_CLK",      3, { 1, 1, 0, 0, 1, 0, 0, 0 }},
    {"SPI_MOSI",     4, { 1, 1, 0, 0, 1, 0, 0, 0 }},
    {"SPI_MISO",     5, { 1, 1, 0, 0, 1, 0, 0, 0 }},
    {"RESET",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"3.3VD",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"3.3VD",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"5VD",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"5VD",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
};

pinmux centipede_mux[] = {
    {"INVALID", 0, { 0, 0, 0, 0, 0, 0, 0, 0 }},
    //J1-1
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB+",         0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB-",         0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GPIO18",      18, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"RESET",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"SPI_CLK",      2, { 1, 0, 0, 0, 1, 0, 0, 0 }},
    {"SPI_MISO",     5, { 1, 0, 0, 0, 1, 0, 0, 0 }},
    {"SPI_MOSI",     4, { 1, 0, 0, 0, 1, 0, 0, 0 }},
    {"SPI_CS0",      2, { 1, 0, 0, 0, 1, 0, 0, 0 }},
    {"GPIO20",      20, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    //J1-11
    {"GPIO19",      19, { 1, 1, 0, 0, 0, 0, 0, 0 }},//-----------
    {"UART_TX",      9, { 1, 0, 0, 0, 0, 0, 0, 1 }},//
    {"UART_RX",     10, { 1, 0, 0, 0, 0, 0, 0, 1 }},// FTDI
    {"5VD",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},// connection
    {"GPIO21",      21, { 1, 1, 0, 0, 0, 0, 0, 0 }},//
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},//------------
    {"ETH_PWR_1",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"ETH_PWR_1",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"ETH_PWR_2",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"ETH_PWR_2",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    //J2-1
    {"GPIO13",      13, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED2
    {"GPIO15",      15, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED4
    {"GPIO17",      17, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED6
    {"GPIO0",        0, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED0
    {"GPIO1",        1, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED1
    {"GPIO14",      14, { 1, 1, 0, 0, 0, 0, 0, 0 }},//LED3
    {"3.3VD",        0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GPIO22",      22, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO23",      23, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO24",      24, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    //J2-11
    {"GPIO11",      11, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO7",        7, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG_TDO
    {"GPIO6",        6, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG_TDI
    {"JTAG_TCK",     0, { 1, 0, 0, 0, 0, 0, 0, 0 }},//JTAG_TCK
    {"GPIO8",        8, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG_TMS
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"NC",           0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"NC",           0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"NC",           0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"NC",           0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
};

pinmux lima_mux[] = {
    {"INVALID", 0, { 0, 0, 0, 0, 0, 0, 0, 0 }},
    //J1-1
    {"GPIO11",      11, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO12",      12, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO13",      13, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO14",      14, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO15",      15, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GPIO17",      17, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"UART_TX",      9, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"UART_RX",     10, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"C_UART_TX",    9, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    //J1-11
    {"C_UART_RX",   10, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"5V",           0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"5V",           0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"3.3V",         0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"3.3V",         0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    //J2-1
    {"C_USB_D_N",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"C_USB_D_P",    0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB_D_N",      0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"USB_D_P",      0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"SPI_MOSI",     7, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"SPI_MISO",     8, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"SPI_CLK",      6, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"SPI_CS0",      5, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"RST",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GPIO0",        0, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG_TCK
    //J2-11
    {"GPIO1",        1, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG_TDI
    {"GPIO2",        2, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG_TDO
    {"GPIO3",        3, { 1, 1, 0, 0, 0, 0, 0, 0 }},//JTAG_TMS
    {"GPIO4",        4, { 1, 1, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
    {"GND",          0, { 1, 0, 0, 0, 0, 0, 0, 0 }},
};

struct busy_gpio_mode {
        int gpio;
        int mode;
};

enum {
        BUSY_GPIO_INVALID,
        BUSY_GPIO_GPIO,
        BUSY_GPIO_I2C,
        BUSY_GPIO_SPI,
        BUSY_GPIO_UART
};

mraa_result_t
mraa_8dev_mmap_write(mraa_gpio_context dev, int value)
{
    volatile uint32_t* addr;
    if (value) {
        *(volatile uint32_t*) (mmap_reg + QCA955X_GPIO_SET + (dev->pin / 32) * 4) =
        (uint32_t)(1 << (dev->pin % 32));
    } else {
        *(volatile uint32_t*) (mmap_reg + QCA955X_GPIO_CLEAR + (dev->pin / 32) * 4) =
        (uint32_t)(1 << (dev->pin % 32));
    }
    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_8dev_mmap_unsetup()
{
    if (mmap_reg == NULL) {
        syslog(LOG_ERR, "8dev mmap: null register cant unsetup");
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    munmap(mmap_reg, mmap_size);
    mmap_reg = NULL;
    if (close(mmap_fd) != 0) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    return MRAA_SUCCESS;
}

int
mraa_8dev_mmap_read(mraa_gpio_context dev)
{
    uint32_t value = *(volatile uint32_t*) (mmap_reg + QCA955X_GPIO_IN + (dev->pin / 32) * 4);
    if (value & (uint32_t)(1 << (dev->pin % 32))) {
        return 1;
    }
    return 0;
}

int8_t
mraa_8dev_mmap()
{
    if (mmap_reg == NULL) {
        if ((mmap_fd = open(MMAP_PATH, O_RDWR)) < 0) {
            syslog(LOG_ERR, "8dev map: unable to open resource0 file");
            return MRAA_ERROR_INVALID_HANDLE;
        }

      if (platform_detected == PLATFORM_8DEVICES_RAMBUTAN ||
          platform_detected == PLATFORM_8DEVICES_LIMA) {
            mmap_reg = (uint8_t*) mmap(NULL, QCA955X_GPIO_BLOCK_SIZE, PROT_READ | PROT_WRITE,
                                       MAP_FILE | MAP_SHARED, mmap_fd, QCA955X_GPIO_BASE);
      } else if(platform_detected == PLATFORM_8DEVICES_CARAMBOLA2 ||
                platform_detected == PLATFORM_8DEVICES_CENTIPEDE) {
            mmap_reg = (uint8_t*) mmap(NULL, AR9331_GPIO_BLOCK_SIZE, PROT_READ | PROT_WRITE,
                                       MAP_FILE | MAP_SHARED, mmap_fd, QCA955X_GPIO_BASE);
      }
        if (mmap_reg == MAP_FAILED) {
            syslog(LOG_ERR, "8dev mmap: failed to mmap");
            mmap_reg = NULL;
            close(mmap_fd);
            return MRAA_ERROR_NO_RESOURCES;
        }
    }

    return MRAA_SUCCESS;
}


mraa_result_t
mraa_8dev_mmap_setup(mraa_gpio_context dev, mraa_boolean_t en)
{
    int8_t mmap_ret;

    if (dev == NULL) {
        syslog(LOG_ERR, "8dev mmap: context not valid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (en == 0) {
        if (dev->mmap_write == NULL && dev->mmap_read == NULL) {
            syslog(LOG_ERR, "8dev mmap: can't disable disabled mmap gpio");
            return MRAA_ERROR_INVALID_PARAMETER;
        }
        dev->mmap_write = NULL;
        dev->mmap_read = NULL;
        mmap_count--;
        if (mmap_count == 0) {
            return mraa_8dev_mmap_unsetup();
        }
        return MRAA_SUCCESS;
    }

    if (dev->mmap_write != NULL && dev->mmap_read != NULL) {
        syslog(LOG_ERR, "8dev mmap: can't enable enabled mmap gpio");
        return MRAA_ERROR_INVALID_PARAMETER;
    }

    // Might need to make some elements of this thread safe.
    // For example only allow one thread to enter the following block
    // to prevent mmap'ing twice.
    
    mmap_ret = mraa_8dev_mmap();

    if (mmap_ret != MRAA_SUCCESS)
	    return mmap_ret;

    dev->mmap_write = &mraa_8dev_mmap_write;
    dev->mmap_read = &mraa_8dev_mmap_read;
    mmap_count++;

    return MRAA_SUCCESS;
}


int mraa_8dev_detect_busy_gpio (struct busy_gpio_mode **gpio_detected, int max_gpio)
{
    int i;
    int ret;
    int read;
    int gpio_int;
    char gpio[10];
    char role[50];
    int j = 0;
    size_t len=3;

    char* line=calloc(100,1);
    FILE *f = fopen("/sys/kernel/debug/gpio", "r");
    if (!f)
        return -1;

    *gpio_detected = calloc(max_gpio, sizeof(struct busy_gpio_mode));
    while ((read = getline(&line, &len, f)) != -1) {
        ret = sscanf(line , " gpio-%5[^'('](%40[^')'])", gpio, role);
        if (ret != 2)
            continue;

        ret = sscanf(gpio, "%d", &gpio_int);
        if (ret != 1)
            continue;
        (*gpio_detected)[j].gpio = gpio_int;

        if (strncmp("sda",role,3) == 0 || strncmp("scl",role,3) == 0)
            (*gpio_detected)[j].mode = BUSY_GPIO_I2C;
        if (strncmp("spi",role,3) == 0)
            (*gpio_detected)[j].mode = BUSY_GPIO_SPI;
        if (strncmp("sysfs",role,5) == 0)
            (*gpio_detected)[j].mode = BUSY_GPIO_GPIO;
        j++;
        if (j>max_gpio)
            break;
    }
    fclose(f);

    return 0;
}

int mraa_8dev_get_pin_by_gpio(mraa_pininfo_t *pins, int max_pins, int gpio, int *pin)
{
    int i;

    for (i=0; i<max_pins; i++){
        if (pins[i].gpio.pinmap == gpio && pins[i].capabilites.gpio==1){
            *pin = i;
            return 0;
        }
    }
    return -1;
}


mraa_pincapabilities_t get_caps(const char* func_name, mraa_pincapabilities_t caps){

        if (func_name == NULL)
                return caps;

        if(strncmp(func_name ,"SPI",3) == 0){
                caps.spi=1;
                caps.gpio=0;
        }
        else if(strncmp(func_name , "UART",4) == 0){
                caps.uart=1;
                caps.gpio=0;
        }
        else if(strncmp(func_name ,"I2C",3) == 0){
                caps.i2c=1;
                caps.gpio=0;
        }
        else if(strncmp(func_name ,"GPIO", 4) == 0){
                caps.gpio=1;
        }
        else{
                caps.gpio=0;
                caps.i2c=0;
                caps.uart=0;
                caps.spi=0;
        }

        return caps;
}

//Get pin capability from gpio FUNCTION register
mraa_pincapabilities_t get_function_cap(int gpio,
                                        mraa_pincapabilities_t caps,
                                        struct gpio_function *func_groups,
                                        int func_group_cnt,
                                        struct function_reg *gpio_functions,
                                        int gpio_functions_cnt )
{
    int i, n, status;
    uint32_t reg;

    for (n=0; n < func_group_cnt; n++){
        if(func_groups[n].gpio != gpio)
            continue;

        for (i=0; i<gpio_functions_cnt; i++){
            if(strcmp(gpio_functions[i].name, func_groups[n].en_func_name)==0){
                reg=*(volatile uint32_t*) (mmap_reg+gpio_functions[i].reg);
                status=(reg>>gpio_functions[i].offset) & 0x1;

                if (func_groups[n].flags & FUNC_FLAG_INVERTED)
                    status = !status;

                if (status==1)
                    return(get_caps(func_groups[n].name, caps));
            }
        }
    }

    return caps;
}

mraa_pincapabilities_t 
get_out_cap(int gpio, 
            struct gpio_func_name *dev_func_names,
            int out_signal_count,
            mraa_pincapabilities_t caps)
{
	uint32_t reg;
	int i, func;
	const char* func_name = NULL;

	reg=*(volatile uint32_t*) (mmap_reg + OUT_FUNC_REG_ADDR + (gpio / 4)*0x04);
	
	func = (reg >> 8*(gpio % 4)) & 0xFF;

	for(i=0; i < out_signal_count; i++){
		if(func == dev_func_names[i].id){
			func_name = dev_func_names[i].name;
		}
	}

	caps = get_caps(func_name, caps);

	return caps;
}	

	
mraa_pincapabilities_t 
get_in_cap(int gpio, struct gpio_input_reg *dev_gpio_input_func, mraa_pincapabilities_t caps)
{
	uint32_t reg_value, test_pin;
	int i = 0;
	const char* func_name = NULL;

	while(dev_gpio_input_func[i].name != NULL){
		reg_value=*(volatile uint32_t*) (mmap_reg + dev_gpio_input_func[i].reg );
		test_pin=(reg_value >> (dev_gpio_input_func[i].offset)) & 0xff;
		if( gpio == test_pin){
			func_name = dev_gpio_input_func[i].name;
			break;
		}
		i++;
	}

	caps=get_caps(func_name, caps);

	return caps;
}


int
get_dev_busses (mraa_board_t* b){

    int i,n;
    char devpath[30];

    // BUS DEFINITIONS
    b->i2c_bus_count = 0;
    for (i = 0; i < 12; i++) {
        sprintf(devpath, "/dev/i2c-%u", i);
        if (mraa_file_exist(devpath) && b->i2c_bus_count < 12) {
            b->i2c_bus[b->i2c_bus_count].bus_id = i;
            b->i2c_bus_count++;
        }
    }

    b->spi_bus_count = 0;
    for (i = 0; i < 6; i++) {
        for (n = 0; n < 6; n++) {
            sprintf(devpath, "/dev/spidev%u.%u", i, n);
            if (mraa_file_exist(devpath) && b->spi_bus_count < 12) {
                b->spi_bus[b->spi_bus_count].bus_id = i;
                b->spi_bus_count++;
                break;
            }
        }
    }

    b->uart_dev_count =0 ;
    for(i=0; i<6; i++){
        sprintf(devpath, "/dev/ttyATH%u", i);
        if(mraa_file_exist(devpath) && b->uart_dev_count < 6){
                b->uart_dev[b->uart_dev_count].device_path = devpath;
                b->uart_dev_count++;
        }
     }

    for(i=0; i<6; i++){
        sprintf(devpath, "/dev/ttyS%u",i);
        if(mraa_file_exist(devpath) && b->uart_dev_count < 6){
                b->uart_dev[b->uart_dev_count].device_path = devpath;
                b->uart_dev_count++;
        }
     }

    return 0;
}

void
mraa_8dev_setup_gpio_capabilities(mraa_board_t *b, struct device_in_out_func *info, int use_signal_mux)
{
    int i, n;
    uint32_t  oe_reg;

    if (mmap_reg == NULL) {
        syslog(LOG_ERR, "8dev: gpio capabilities unavailable, due to failed mmap");
        return;
    }

    b->gpio_count = 0;
    oe_reg = *(volatile uint32_t *)mmap_reg;

    for (i = 0; i < b->phy_pin_count; i++) {
        if (b->pins[i].capabilites.gpio)
            b->pins[i].capabilites = get_function_cap(
                                            b->pins[i].gpio.pinmap,
                                            b->pins[i].capabilites,
                                            info->gpio_function_groups,
                                            info->gpio_function_group_count,
                                            info->gpio_functions,
                                            info->gpio_function_count);

        if (b->pins[i].capabilites.gpio && use_signal_mux == 1) {
            if (oe_reg & (0x1 << b->pins[i].gpio.pinmap))
                b->pins[i].capabilites = get_in_cap(b->pins[i].gpio.pinmap,
                                                    info->input_signals,
                                                    b->pins[i].capabilites);
            else
                b->pins[i].capabilites = get_out_cap(b->pins[i].gpio.pinmap,
                                                        info->output_signals,
                                                        info->output_signal_count,
                                                        b->pins[i].capabilites);
        }

        if (b->pins[i].capabilites.gpio)
            b->gpio_count++;
    }
}

mraa_board_t*
mraa_8dev_rambutan()
{
    mraa_board_t* b = mraa_8dev_common("Rambutan", PLATFORM_8DEVICES_RAMBUTAN, 41, rambutan_mux);
    if (!b)
        return NULL;

    mraa_8dev_setup_gpio_capabilities(b, &dev_rambutan, 1);

    return b;
}

mraa_board_t*
mraa_8dev_carambola2(void)
{
    mraa_board_t *b = mraa_8dev_common("Carambola2", PLATFORM_8DEVICES_CARAMBOLA2, 41, carambola2_mux);
    if (!b)
        return NULL;

    mraa_8dev_setup_gpio_capabilities(b, &dev_carambola, 0);

    return b;
}

mraa_board_t*
mraa_8dev_centipede(void)
{
    mraa_board_t *b = mraa_8dev_common("Centipede", PLATFORM_8DEVICES_CENTIPEDE, 41, centipede_mux);
    if (!b)
        return NULL;

    mraa_8dev_setup_gpio_capabilities(b, &dev_carambola, 0);

    return b;
}

mraa_board_t*
mraa_8dev_lima(void)
{
    mraa_board_t *b = mraa_8dev_common("Lima", PLATFORM_8DEVICES_LIMA, 33, lima_mux);
    if (!b)
            return NULL;

    mraa_8dev_setup_gpio_capabilities(b, &dev_lima, 1);

    return b;
}

mraa_board_t* mraa_8dev_common(char *name, int platform, int pin_count, pinmux *mux)
{
    int i, ret;
    struct busy_gpio_mode *gpio_detected = NULL;

    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (!b)
        return NULL;

    b->platform_name = name;
    platform_detected = platform;
    b->phy_pin_count = pin_count;

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (!b->adv_func)
        return NULL;

    b->pins = (mraa_pininfo_t*) calloc(1, sizeof(mraa_pininfo_t) * b->phy_pin_count);
    if (!b->pins)
        return NULL;

    for (i = 1; i < b->phy_pin_count; i++) {
        strncpy(b->pins[i].name, mux[i].name, MRAA_PIN_NAME_SIZE);
        b->pins[i].capabilites = mux[i].caps;
        b->pins[i].gpio.pinmap = mux[i].pinmap;
    }

    ret = mraa_8dev_detect_busy_gpio(&gpio_detected, pin_count);
    if (!ret) {
        for (i=0; i<41; i++){
            int pin;
            if (mraa_8dev_get_pin_by_gpio(b->pins, pin_count, gpio_detected[i].gpio, &pin))
                continue;

            if (gpio_detected[i].mode == BUSY_GPIO_I2C) {
                b->pins[pin].capabilites.gpio = 0;
                b->pins[pin].capabilites.i2c = 1;
            }
            if (gpio_detected[i].mode == BUSY_GPIO_SPI) {
                b->pins[pin].capabilites.gpio = 0;
                b->pins[pin].capabilites.spi = 1;
            }
        }
        free (gpio_detected);
    }
    else
        syslog(LOG_ERR, "Can't detect busy GPIOs (check /sys/kernel/debug/gpio)");

    get_dev_busses(b);

    b->adv_func->gpio_mmap_setup = &mraa_8dev_mmap_setup;

    mraa_8dev_mmap();

    return b;
}

mraa_board_t*
mraa_qca_8devices(void)
{
    char* line = calloc(1, 100);
    FILE* fh = fopen("/tmp/sysinfo/board_name", "r");
    if(fh != NULL && line != NULL) {
        if (fread(line, 100, 1, fh) >= 0) {
            fclose(fh);
            if (strncmp(line,"carambola2", 10) == 0)
                return mraa_8dev_carambola2();
            else if (strncmp(line,"centipede", 9) == 0)
                return mraa_8dev_centipede();
            else if (strncmp(line,"lima", 4) == 0)
                return mraa_8dev_lima();
            else if (strncmp(line,"rambutan", 8) == 0)
                return mraa_8dev_rambutan();
            else{
                syslog(LOG_ERR, "Unknown 8devices board");
                return NULL;
            }
        }
    }
    syslog(LOG_ERR, "Failed to detect 8devices board");
    return NULL;
}


