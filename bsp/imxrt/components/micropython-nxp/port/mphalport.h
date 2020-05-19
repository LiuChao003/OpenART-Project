/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Armink (armink.ztl@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <rtthread.h>
#include <drivers/pin.h>
#include "pin_defs_mcu.h"


#define MP_HAL_UNIQUE_ID_ADDRESS (0x1ff0f420)
#define MP_HAL_CLEANINVALIDATE_DCACHE(addr, size) (SCB_CleanInvalidateDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)((uint8_t*)addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))
#define MP_HAL_CLEAN_DCACHE(addr, size)                     (SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)((uint8_t*)addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))


#define MP_HAL_PIN_FMT                 "%s"

extern void mp_hal_set_interrupt_char (int c);
extern void mp_pin_od_write(void *machine_pin, int stat);
extern void mp_hal_pin_open_set(void *machine_pin, int mode);
extern void mp_hal_stdout_tx_strn_stream(const char *str, size_t len);

#define mp_hal_quiet_timing_enter()         MICROPY_BEGIN_ATOMIC_SECTION()
#define mp_hal_quiet_timing_exit(irq_state) MICROPY_END_ATOMIC_SECTION(irq_state)

// needed for machine.I2C
#define mp_hal_delay_us_fast(us) mp_hal_delay_us(us)
#define mp_hal_pin_od_low(pin)   mp_pin_od_write(pin, PIN_LOW)
#define mp_hal_pin_od_high(pin)  mp_pin_od_write(pin, PIN_HIGH)
#define mp_hal_pin_open_drain(p) mp_hal_pin_open_set(p, PIN_MODE_OUTPUT_OD)

// needed for soft machine.SPI
#define mp_hal_pin_output(p)     mp_hal_pin_open_set(p, PIN_MODE_OUTPUT)
#define mp_hal_pin_input(p)      mp_hal_pin_open_set(p, PIN_MODE_INPUT)
#define mp_hal_pin_name(p)       mp_hal_pin_get_name(p)
#define mp_hal_pin_high(p)       mp_hal_pin_write(p, 1)

#define mp_hal_pin_obj_t const pin_obj_t*
#define mp_hal_get_pin_obj(o)   pin_find(o)
//#define mp_hal_pin_name(p)      ((p)->name)

#define REG_READ32(reg)  (*((volatile uint32_t *)(reg)))
#define REG_WRITE32(reg,val32)  (*((volatile uint32_t *)(reg))) = val32
#define REG_OR32(reg,val32)		(*((volatile uint32_t *)(reg))) |= val32
#define REG_AND32(reg,val32)		(*((volatile uint32_t *)(reg))) &= val32

void mp_hal_gpio_clock_enable(uint32_t portNum);
void mp_hal_pin_config(const pin_obj_t *p, const pin_af_obj_t *af, uint32_t alt, uint32_t padCfgVal );
//bool mp_hal_pin_config_alt(mp_hal_pin_obj_t pin, uint32_t padCfg,  uint8_t fn);

void mp_hal_ConfigGPIO(const pin_obj_t *p, uint32_t gpioModeAndPadCfg, uint32_t isInitialHighForOutput);

#define MP_HAL_PIN_FMT                  "%q"
#define MP_HAL_PIN_MODE_INPUT           (GPIO_MODE_INPUT)
#define MP_HAL_PIN_MODE_OUTPUT          (GPIO_MODE_OUTPUT_PP)
// #define MP_HAL_PIN_MODE_INV             (GPIO_INVERTER)
// #define MP_HAL_PIN_MODE_DIGITAL         (GPIO_MODE_DIGITAL)
#define MP_HAL_PIN_MODE_OPEN_DRAIN      (GPIO_MODE_OUTPUT_OD)
#define MP_HAL_PIN_MODE_ALT_OPEN_DRAIN  (GPIO_MODE_OUTPUT_OD)
#define MP_HAL_PIN_PULL_NONE             0 // (GPIO_NOPULL)
#define MP_HAL_PIN_PULL_UP               GPIO_PULLUP // (GPIO_PULLUP)
#define MP_HAL_PIN_PULL_DOWN             GPIO_PULLDOWN // (GPIO_PULLDOWN)
#define MP_HAL_PIN_PULL_REPEATER         GPIO_REPEATER // (GPIO_PULLDOWN)