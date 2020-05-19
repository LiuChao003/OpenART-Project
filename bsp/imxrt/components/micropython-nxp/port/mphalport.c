/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Armink (armink.ztl@gmail.com)
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

#include <stdio.h>
#include <string.h>

#include <rtthread.h>
#include <py/mpconfig.h>
#include <py/runtime.h>
#include "mphalport.h"
#include "mpgetcharport.h"
#include "mpputsnport.h"
#include "drivers/pin.h"
#ifdef NXP_USING_OPENMV
#include "drv_usb_omv.h"
#endif
#include "pin_defs_mcu.h"

const char rtthread_help_text[] =
"Welcome to MicroPython on RT-Thread!\n"
"\n"
"Control commands:\n"
"  CTRL-A        -- on a blank line, enter raw REPL mode\n"
"  CTRL-B        -- on a blank line, enter normal REPL mode\n"
"  CTRL-C        -- interrupt a running program\n"
"  CTRL-D        -- on a blank line, do a soft reset of the board\n"
"  CTRL-E        -- on a blank line, enter paste mode\n"
"\n"
"For further help on a specific object, type help(obj)\n"
;

int mp_hal_stdin_rx_chr(void) {
    char ch;
    while (1) {
        ch = mp_getchar();
        if (ch != (char)0xFF) {
            break;
        }
        MICROPY_EVENT_POLL_HOOK;
        rt_thread_delay(1);
    }
    return ch;
}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
    mp_putsn(str, len);
#ifdef NXP_USING_OPENMV	
	if (usb_vcp_is_enabled()) {
		// rocky: if send through VCP, on windows, MUST open the port,
		// otherwise, a buffer on windows will finally overrun, and host 
		// will no longer accept data from us!
		usb_vcp_send_strn(str, len);
	}
#endif	
}


void mp_hal_stdout_tx_strn_stream(const char *str, size_t len) {
    mp_putsn_stream(str, len);
#ifdef NXP_USING_OPENMV		
    if (usb_vcp_is_enabled()) {
		// rocky: if send through VCP, on windows, MUST open the port,
		// otherwise, a buffer on windows will finally overrun, and host 
		// will no longer accept data from us!		
        usb_vcp_send_strn_cooked(str, len);
    } else {

    }
#endif
}

mp_uint_t mp_hal_ticks_us(void) {
    return rt_tick_get() * 1000000UL / RT_TICK_PER_SECOND;
}

mp_uint_t mp_hal_ticks_ms(void) {
    return rt_tick_get() * 1000 / RT_TICK_PER_SECOND;
}

mp_uint_t mp_hal_ticks_cpu(void) {
    return rt_tick_get();
}

void mp_hal_delay_us(mp_uint_t us) {
    rt_tick_t t0 = rt_tick_get(), t1, dt;
    uint64_t dtick = us * RT_TICK_PER_SECOND / 1000000L;
    while (1) {
        t1 = rt_tick_get();
        dt = t1 - t0;
        if (dt >= dtick) {
            break;
        }
        mp_handle_pending();
    }
}

void mp_hal_delay_ms(mp_uint_t ms) {
    rt_tick_t t0 = rt_tick_get(), t1, dt;
    uint64_t dtick = ms * RT_TICK_PER_SECOND / 1000L;
    while (1) {
        t1 = rt_tick_get();
        dt = t1 - t0;
        if (dt >= dtick) {
            break;
        }
        MICROPY_EVENT_POLL_HOOK;
        rt_thread_delay(1);
    }
}

//void mp_hal_gpio_clock_enable(uint32_t portNum) {
//	 // i.MX RT's GPIO port starts from 1, and clock gate is not ordered in GPIO order
//	const static clock_ip_name_t tab[] = {(clock_ip_name_t)0, kCLOCK_Gpio1, kCLOCK_Gpio2, kCLOCK_Gpio3, kCLOCK_Gpio4, kCLOCK_Gpio5};
//	CLOCK_EnableClock(tab[portNum]); 

//}

//void mp_hal_pin_config(const pin_obj_t *p, const pin_af_obj_t *af, uint32_t alt, uint32_t padCfgVal ) {
//	uint32_t isInputPathForcedOn = 0;
//	padCfgVal &= ~(1UL<<31);	// clear MSb, as it is used to mark input/output for GPIO
//	CLOCK_EnableClock(kCLOCK_Iomuxc);
//	
//	if (alt == PIN_ALT_NC) 
//		alt = REG_READ32(p->afReg) & 7;
//	isInputPathForcedOn =  (alt == 5 || (padCfgVal | 1<<11) );	// Alt 5 is GPIO or OD mode is selected
//	IOMUXC_SetPinMux(p->afReg, alt, af->inSelReg, af->inSelVal, p->cfgReg, isInputPathForcedOn);
//	IOMUXC_SetPinConfig(p->afReg,alt,af->inSelReg, af->inSelVal, p->cfgReg, padCfgVal);
//}

//bool mp_hal_pin_config_alt(mp_hal_pin_obj_t pin, uint32_t padCfg,  uint8_t fn) {
//    const pin_af_obj_t *af = pin_find_af(pin, fn);
//    if (af == NULL) {
//        return false;
//    }
//	mp_hal_pin_config(pin, af, af->idx, padCfg);
//    return true;
//}

//void mp_hal_ConfigGPIO(const pin_obj_t *p, uint32_t gpioModeAndPadCfg, uint32_t isInitialHighForOutput)
//{
//	GPIO_Type *pGPIO = p->gpio;
//	uint32_t pinMask = 1 << p->pin;
//	mp_hal_gpio_clock_enable(p->port);
//	pGPIO->IMR &= ~pinMask;	 // disable pin IRQ
//	if (gpioModeAndPadCfg & (GPIO_PAD_OUTPUT_MASK)) {
//		// output
//		if (isInitialHighForOutput)
//			pGPIO->DR |= pinMask;
//		else
//			pGPIO->DR &= ~pinMask;
//		pGPIO->GDIR |= pinMask;
//		
//	} else {
//		// input
//		pGPIO->GDIR &= ~pinMask;
//	}
//	mp_hal_pin_config_alt(p, gpioModeAndPadCfg, AF_FN_GPIO);
//}



