/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 ChenYong (chenyong@rt-thread.com)
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
#include <stdint.h>
#include <string.h>

#include "py/nlr.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"

#ifdef MICROPYTHON_USING_MACHINE_ADC

#include <rtthread.h>
#include <drivers/adc.h>
#include "aia_cmm/cfg_mux_mgr.h"
#include "pin_defs_mcu.h"
#include "py/mphal.h"
#include "extmod/virtpin.h"
extern const mp_obj_type_t machine_adc_type;
const pin_obj_t *pin_find(mp_obj_t user_obj);
typedef struct _machine_adc_obj_t {
    mp_obj_base_t base;
    struct rt_adc_device *adc_device;
    uint8_t channel;
    uint8_t is_init;
} machine_adc_obj_t;

STATIC void error_check(bool status, const char *msg) {
    if (!status) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, msg));
    }
}

static inline uint32_t adc_get_internal_channel(uint32_t channel) {
    return channel;
}

STATIC mp_obj_t machine_adc_make_new(const mp_obj_type_t *type,
                                  size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // create ADC object from the given pin
    machine_adc_obj_t *self = m_new_obj(machine_adc_obj_t);
    struct rt_adc_device *adc_device = RT_NULL;
    char adc_dev_name[RT_NAME_MAX] = {0};
		char *ADCSignal[1];
    rt_err_t result = RT_EOK;

    // init machine adc object information
    self->channel = 0;
    self->is_init = RT_FALSE;
    self->base.type = &machine_adc_type;
    mp_arg_check_num(n_args, n_kw, 1, 2, true);
		
    // check input ADC device name or ID
    if (mp_obj_is_small_int(args[0])) {
        rt_snprintf(adc_dev_name, sizeof(adc_dev_name), "adc%d", mp_obj_get_int(args[0]));
    } else if (mp_obj_is_qstr(args[0])) {
        rt_strncpy(adc_dev_name, mp_obj_str_get_str(args[0]), RT_NAME_MAX);
    } else {
        error_check(0, "Input ADC device name or ID error.");
    }

    adc_device = (struct rt_adc_device *) rt_device_find(adc_dev_name);
    if (adc_device == RT_NULL || adc_device->parent.type != RT_Device_Class_Miscellaneous) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
                                                "ADC(%s) don't exist", adc_dev_name));
    }
		self->adc_device = adc_device;
    if (n_args == 2) {
				mp_obj_t pin_obj = args[1];
		if (MP_OBJ_IS_INT(pin_obj)) {
             self->channel = adc_get_internal_channel(mp_obj_get_int(pin_obj));
    } 
			else {
						 const pin_obj_t *pin = pin_find(pin_obj);  
             self->channel = pin->adc_channel;
    }
		result = rt_adc_enable(self->adc_device, self->channel);
		error_check(result == RT_EOK, "ADC enable error");
		switch (self->channel){
				case 0: ADCSignal[0]  = 	"A1";                break;
				case 1: ADCSignal[0]  = 	"DBG_TXD";					 break;
				case 2: ADCSignal[0]  = 	"DBG_RXD";					 break;
				case 5: ADCSignal[0]  = 	"A5";								 break;
				case 6: ADCSignal[0]  = 	"A4";								 break;
				case 7: ADCSignal[0]  = 	"D6_PWM2";    			 break;
				case 8: ADCSignal[0]  = 	"D7_PWM3";				   break;
				case 9: ADCSignal[0]  = 	"A2";								 break;
				case 10:ADCSignal[0]  = 	"A3";							   break;
				case 11:ADCSignal[0]  = 	"D1_TX";						 break;
				case 12:ADCSignal[0]  = 	"D0_RX";						 break;
				case 13:ADCSignal[0]  = 	"D3_INT1";					 break;
				case 15:ADCSignal[0]  = 	"A0";								 break;
		}	
		MuxItem_t mux_ADC;
		Mux_Take(self,"adc",-1,ADCSignal[0],&mux_ADC);
		mp_hal_ConfigGPIO(mux_ADC.pPinObj, 2 /*MP_HAL_PIN_MODE_ADC*/, MP_HAL_PIN_PULL_NONE); //MP_HAL_PIN_PULL_NONE  GPIO_MODE_INPUT_PUP_WEAK				
    self->is_init = RT_TRUE;
		if(mux_ADC.szHint[0]==0x31)
				self->adc_device->parent.user_data=((ADC_Type *)(0x400C4000));
		else if (mux_ADC.szHint[0]==0x32)
				self->adc_device->parent.user_data=((ADC_Type *)(0x400C8000));		
    }
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t machine_adc_init(size_t n_args, const mp_obj_t *args) {
		machine_adc_obj_t *self = MP_OBJ_TO_PTR(args[0]);
		char *ADCSignal[1];
		mp_obj_t pin_obj = args[1];
		if (MP_OBJ_IS_INT(pin_obj)) {
             self->channel = adc_get_internal_channel(mp_obj_get_int(pin_obj));
    } 
			else {
						 const pin_obj_t *pin = pin_find(pin_obj);  
						 self->channel = pin->adc_channel;
    }
    rt_err_t result = RT_EOK;
    result = rt_adc_enable(self->adc_device, self->channel);
    error_check(result == RT_EOK, "ADC enable error");
		switch (self->channel){
				case 0: ADCSignal[0]  = 	"A1";                break;
				case 1: ADCSignal[0]  = 	"DBG_TXD";					 break;
				case 2: ADCSignal[0]  = 	"DBG_RXD";					 break;
				case 5: ADCSignal[0]  = 	"A5";								 break;
				case 6: ADCSignal[0]  = 	"A4";								 break;
				case 7: ADCSignal[0]  = 	"D6_PWM2";    			 break;
				case 8: ADCSignal[0]  = 	"D7_PWM3";				   break;
				case 9: ADCSignal[0]  = 	"A2";								 break;
				case 10:ADCSignal[0]  = 	"A3";							   break;
				case 11:ADCSignal[0]  = 	"D1_TX";						 break;
				case 12:ADCSignal[0]  = 	"D0_RX";						 break;
				case 13:ADCSignal[0]  = 	"D3_INT1";					 break;
				case 15:ADCSignal[0]  = 	"A0";								 break;
		}	
		MuxItem_t mux_ADC;
		Mux_Take(self,"adc",-1,ADCSignal[0],&mux_ADC);
		mp_hal_ConfigGPIO(mux_ADC.pPinObj, 2 /*MP_HAL_PIN_MODE_ADC*/, MP_HAL_PIN_PULL_NONE); //MP_HAL_PIN_PULL_NONE  GPIO_MODE_INPUT_PUP_WEAK
    self->is_init = RT_TRUE;
		if(mux_ADC.szHint[0]==0x31)
				self->adc_device->parent.user_data=((ADC_Type *)(0x400C4000));
		else if (mux_ADC.szHint[0]==0x32)
				self->adc_device->parent.user_data=((ADC_Type *)(0x400C8000));
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_adc_init_obj, 2, 2, machine_adc_init);

STATIC mp_obj_t machine_adc_deinit(mp_obj_t self_in) {
    machine_adc_obj_t *self = MP_OBJ_TO_PTR(self_in);
    rt_err_t result = RT_EOK;

    if (self->is_init == RT_TRUE) {
        result = rt_adc_disable(self->adc_device, self->channel);
        error_check(result == RT_EOK, "ADC disable error");
        self->is_init = RT_FALSE;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_adc_deinit_obj, machine_adc_deinit);

STATIC mp_obj_t machine_adc_read(mp_obj_t self_in) {
    machine_adc_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int tval = 0;

    error_check(self->is_init == RT_TRUE, "ADC device uninitialized");

    tval = rt_adc_read(self->adc_device, self->channel);
    return MP_OBJ_NEW_SMALL_INT(tval);
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_adc_read_obj, machine_adc_read);

STATIC const mp_rom_map_elem_t machine_adc_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init),    MP_ROM_PTR(&machine_adc_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),  MP_ROM_PTR(&machine_adc_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),    MP_ROM_PTR(&machine_adc_read_obj) },
};

STATIC MP_DEFINE_CONST_DICT(machine_adc_locals_dict,
                            machine_adc_locals_dict_table);

const mp_obj_type_t machine_adc_type = {
    { &mp_type_type },
    .name = MP_QSTR_ADC,
    .make_new = machine_adc_make_new,
    .locals_dict = (mp_obj_dict_t *) &machine_adc_locals_dict,
};

#endif // MICROPYTHON_USING_MACHINE_adc
