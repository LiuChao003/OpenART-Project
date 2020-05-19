/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       		Notes
 * 2020-04-13     Tony.Zhang(NXP)	
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "fsl_csi_camera_adapter.h"
#include "fsl_camera.h"
#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"
#include "fsl_ov7725.h"
#include "fsl_elcdif.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "omv_boardconfig.h"
#include "fsl_cache.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_csi.h"
#include "py/mpprint.h"
#include "irq.h"
#include "sensor.h"
#include "drv_camera.h"
#include "ov9650.h"
#include "ov2640.h"
#include "ov7725.h"
#include "ov7725_regs.h"
#include "mt9v034.h"
#include "framebuffer.h"
#define cam_echo(...) //mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)
#define cam_err(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)
#define BUFFER_SIZE	(100000)
#if defined(__CC_ARM)
	extern unsigned int Image$$MPY_SENSOR_BUFER_START$$Base;
#elif defined(__ICCARM__)
	extern unsigned int MPY_SENSOR_BUFER_START$$Limit[];
#elif defined(__GNUC__)

#endif

enum{
	RT_CAMERA_DEVICE_INIT = 0,
	RT_CAMERA_DEVICE_SUSPEND,
	RT_CAMERA_DEVICE_RUNING,
}RT_CAMERA_DEVICE_ST;

//				 			8bit | PixRisEdge | gatedClk  | SyncClrFifo| HSyncActHigh|SofOnVsyncRis|ExtVSync
#define CSICR1_INIT_VAL 	0<<0 | 1<<1	      | 1<<4	  | 1<<8	   | 1<<11		 | 1<<17	   |1<<30   
struct fb_mem
{
	uint32_t tick;
	uint8_t ptr[];
};

struct fb_mem_list
{
	uint16_t bbp;
	uint16_t w;
	uint16_t h;
	
	uint16_t count;
	uint32_t idx;
	uint32_t frame_size;
	uint32_t *buffer_start;
	uint32_t total_frame_count;
};

typedef struct _CSIIrq_t
{
	uint8_t isStarted;
	uint8_t isGray;
	uint32_t base0;
	uint32_t linePerFrag;
	uint32_t cnt;
	uint32_t dmaBytePerLine;
	uint32_t dmaBytePerFrag;
	uint32_t dmaFragNdx;

	uint32_t datBytePerLine;
	uint32_t datBytePerFrag;
	uint32_t datFragNdx;

	uint32_t datCurBase;

	uint32_t fragCnt;
	// in color mode, dmaFragNdx should == datLineNdx
	// in gray mode, to save memory, move backword nextDmaBulk every 4 lines
	
}CSIIrq_t;
;

struct imxrt_camera
{
    char *name;
    CSI_Type *	csi_base;
    IRQn_Type irqn;
	struct rt_camera_device *rtt_device;
	volatile CSIIrq_t s_irq;
	struct fb_mem_list fb_list;
	sensor_t	sensor;
	
	uint8_t		sensor_id;
	uint8_t     sensor_addr;
	GPIO_Type * sensor_pwn_io;
	uint8_t     sensor_pwn_io_pin;
	char 		*sensor_bus_name;
	uint32_t	fb_buffer_start;
	uint32_t	fb_buffer_end;
};

static struct imxrt_camera *pCam = NULL;
#define CAM_NUM		1
static struct imxrt_camera cams[1] = {
	{
		.name = "camera0",
		.csi_base = CSI,
		
		.sensor_id = 0,
		.sensor_addr = 0x21U,
		.sensor_pwn_io = GPIO1,
		.sensor_pwn_io_pin = 18,
		.sensor_bus_name = "i2c1",
#if defined(__CC_ARM)		
		.fb_buffer_start = (uint32_t) &Image$$MPY_SENSOR_BUFER_START$$Base,
		.fb_buffer_end = (uint32_t) &Image$$MPY_SENSOR_BUFER_START$$Base + BUFFER_SIZE,
#elif defined(__ICCARM__)	
		.fb_buffer_start = (uint32_t) MPY_SENSOR_BUFER_START$$Limit,
		.fb_buffer_end = (uint32_t) MPY_SENSOR_BUFER_START$$Limit + BUFFER_SIZE,
#endif		
	},
};


const int resolution[][2] = {
    {0,    0   },
    // C/SIF Resolutions
    {88,   72  },    /* QQCIF     */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {88,   60  },    /* QQSIF     */
    {176,  120 },    /* QSIF      */
    {352,  240 },    /* SIF       */
    // VGA Resolutions
    {40,   30  },    /* QQQQVGA   */
    {80,   60  },    /* QQQVGA    */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {640,  480 },    /* VGA       */
    {60,   40  },    /* HQQQVGA   */
    {120,  80  },    /* HQQVGA    */
    {240,  160 },    /* HQVGA     */
    // FFT Resolutions
    {64,   32  },    /* 64x32     */
    {64,   64  },    /* 64x64     */
    {128,  64  },    /* 128x64    */
    {128,  128 },    /* 128x64    */
    // Other
    {128,  160 },    /* LCD       */
    {128,  160 },    /* QQVGA2    */
    {720,  480 },    /* WVGA      */
    {752,  480 },    /* WVGA2     */
    {800,  600 },    /* SVGA      */
    {1280, 1024},    /* SXGA      */
    {1600, 1200},    /* UXGA      */
};
#define FRAMEBUFFER_SIZE (720*480*3)
#define FRAMEBUFFER_COUNT 3
static uint64_t s_dmaFragBufs[2][640 * 2 / 8];	// max supported line length
static struct rt_mailbox  csi_frame_mb;
static rt_uint32_t csi_frame_mb_pool[4];
//==================================================================================================



void CsiFragModeInit(CSI_Type *s_pCSI ) {

	CLOCK_EnableClock(kCLOCK_Csi);
	CSI_Reset(s_pCSI);
	
	s_pCSI->CSICR1 = CSICR1_INIT_VAL;
	s_pCSI->CSICR2 = 3U << 30;	// INCR16 for RxFIFO DMA
	s_pCSI->CSICR3 = 2U << 4;	// 16 double words to trigger DMA request
	s_pCSI->CSIFBUF_PARA = 0;	// no stride


	s_pCSI->CSICR18 = 13<<12 | 1<<18;	// HProt AHB bus protocol, write to memory when CSI_ENABLE is 1

	NVIC_SetPriority(CSI_IRQn, IRQ_PRI_CSI);
}

void imx_cam_sensor_io_init(GPIO_Type *base, uint32_t pin)
{
    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };
	
	GPIO_PinInit(base, pin, &config);	
}

static void imx_cam_sensor_io_set(GPIO_Type *base, uint32_t pin,bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(base, pin, 1);
    }
    else
    {
        GPIO_PinWrite(base, pin, 0);
    }
}

int imx_cam_sensor_read_reg(struct rt_i2c_bus_device *i2c_bus,rt_uint16_t addr, rt_uint8_t reg, rt_uint8_t *data)
{
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3];
	
	
	buf[0] = reg; //cmd
    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 1;
	
	msgs[1].addr = addr;
    msgs[1].flags = RT_I2C_RD|RT_I2C_NO_START;
    msgs[1].buf = data;
    msgs[1].len = 1;
	
//	count = rt_i2c_master_send(i2c_bus,addr,0,buf,1);
//	count = rt_i2c_master_recv(i2c_bus,addr,RT_I2C_NO_START,data,1);
    
	
    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
    {
        return 0 ;
    }
    else
    {
		cam_err("[%s] error\r\n",__func__);
        return -1;
    }
	
}

int imx_cam_sensor_write_reg(struct rt_i2c_bus_device *i2c_bus,rt_uint16_t addr, rt_uint8_t reg, rt_uint8_t data)
{
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3];

	buf[0] = reg; //cmd
	buf[1] = data;

	msgs.addr = addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 2;

	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
	{
		return 0;
	}
	else
	{
		cam_err("[%s] error\r\n",__func__);
		return -1;
	}
}

int imx_cam_sensor_cambus_writeb(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)
{
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	return imx_cam_sensor_write_reg(i2c_bus,slv_addr,reg_addr,reg_data);
}
int imx_cam_sensor_cambus_readb(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data)
{
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	return imx_cam_sensor_read_reg(i2c_bus,slv_addr,reg_addr,reg_data);
}

int imx_cam_sensor_cambus_readw(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint16_t *reg_data)
{
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3];
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	buf[0] = reg_addr; //cmd
    msgs[0].addr = slv_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 1;
	
	msgs[1].addr = reg_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = (uint8_t *)reg_data;
    msgs[1].len = 1;
	
    
    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
    {
        return 0 ;
    }
    else
    {
		cam_err("[%s] error\r\n",__func__);
        return -1;
    }
}

int imx_cam_sensor_cambus_writew(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint16_t reg_data)
{
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3];
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	buf[0] = reg_addr; //cmd
	buf[1] = reg_data & 0x00ff;
	buf[2] = reg_data >> 8;
	
	msgs.addr = slv_addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 3;

	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
	{
		return 0;
	}
	else
	{
		cam_err("[%s] error\r\n",__func__);
		return -1;
	}
}


void imx_cam_sensor_init(struct imxrt_camera *cam)
{
	struct rt_i2c_bus_device *i2c_bus;
	
	
    
	imx_cam_sensor_io_init(cam->sensor_pwn_io,cam->sensor_pwn_io_pin);
	
	i2c_bus = rt_i2c_bus_device_find(cam->sensor_bus_name);
	if(i2c_bus == RT_NULL)
	{
		cam_err("[%s]driver can not find %s bus\r\n",__func__,cam->sensor_bus_name);
		return ;
	}
	cam->sensor.i2c_bus = (uint32_t *)i2c_bus;
	cam->sensor.cambus_readb = imx_cam_sensor_cambus_readb;
	cam->sensor.cambus_writeb = imx_cam_sensor_cambus_writeb;
	cam->sensor.cambus_readw = imx_cam_sensor_cambus_readw;
	cam->sensor.cambus_writew = imx_cam_sensor_cambus_writew;
	cam->sensor_addr= 0x21;
	cam->sensor.slv_addr = cam->sensor_addr;
	
	CLOCK_SetMux(kCLOCK_CsiMux, 0);
    CLOCK_SetDiv(kCLOCK_CsiDiv, 0);
	
	imx_cam_sensor_read_reg(i2c_bus,cam->sensor_addr,ON_CHIP_ID, &cam->sensor_id);
	if(cam->sensor_id == MT9V034_ID)
	{
		mt9v034_init(&cam->sensor);
	}
	else
	{
		imx_cam_sensor_read_reg(i2c_bus,cam->sensor_addr,OV_CHIP_ID, &cam->sensor_id);
		cam_echo("Camera Device id:0x%x\r\n",cam->sensor_id);
		switch(cam->sensor_id)
		{
			case MT9V034_ID:
				mt9v034_init(&cam->sensor);
				break;
			case OV9650_ID:
				ov9650_init(&cam->sensor);
				break;
			case OV2640_ID:
				ov2640_init(&cam->sensor);
				break;
			case OV7725_ID:
				ov7725_init(&cam->sensor);
				break;
			default:
				cam_err("[%s] sensor id:0x%2x not support\r\n",__func__,cam->sensor_id);
		}
	
	}
}

int imxrt_camera_set_framerate(struct imxrt_camera *cam,framerate_t framerate)
{
    if (cam->sensor.framerate == framerate) {
       /* no change */
        return 0;
    }
	if (framerate & 0x80000000)
		CCM->CSCDR3 = framerate & (0x1F<<9);

    /* call the sensor specific function */
    if (cam->sensor.set_framerate == NULL
        || cam->sensor.set_framerate(&cam->sensor, framerate) != 0) {
        /* operation not supported */
        return -1;
    }

    /* set the frame rate */
    cam->sensor.framerate = framerate;

    return 0;
}

void imx_cam_reset(struct imxrt_camera *cam)
{
	//sensor init
	imx_cam_sensor_init(cam);
	//csi init
	CsiFragModeInit(cam->csi_base);
	
	cam->sensor.isWindowing = 0;
	cam->sensor.wndH = cam->sensor.fb_h;
	cam->sensor.wndW = cam->sensor.fb_w;
	cam->sensor.wndX = cam->sensor.wndY = 0;	
	imxrt_camera_set_framerate(cam,0x80000000 | (2<<9|(8-1)<<11));
	cam->sensor.sde          = 0xFF;
    cam->sensor.pixformat    = 0xFF;
    cam->sensor.framesize    = 0xFF;
    cam->sensor.framerate    = 0xFF;
    cam->sensor.gainceiling  = 0xFF;


    // Call sensor-specific reset function; in the moment,we use our init function and defaults regs
    cam->sensor.reset(&cam->sensor);
}

void imxrt_cam_buffer_list_init(struct imxrt_camera *cam)
{
	uint8_t bpp;
	int16_t w,h;
	uint32_t size;
	
	w = resolution[cam->sensor.framesize][0];
	h = resolution[cam->sensor.framesize][1];
	switch (cam->sensor.pixformat) {
        case PIXFORMAT_GRAYSCALE:
            bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            bpp = 2;
            break;
        case PIXFORMAT_BAYER:
            bpp = 3;
            break;
        case PIXFORMAT_JPEG:
            // Read the number of data items transferred
            // MAIN_FB()->bpp = (MAX_XFER_SIZE - __HAL_DMA_GET_COUNTER(&DMAHandle))*4;
            break;
		default:
			break;
    }
	
	size = w * h * bpp;
	cam->fb_list.w = w;
	cam->fb_list.h = h;
	cam->fb_list.bbp = bpp;
	cam->fb_list.frame_size = size;
	cam->fb_list.buffer_start = (rt_uint32_t *)cam->fb_buffer_start;
	cam->fb_list.idx = 0;
#if 1
	struct fb_mem *fb_mem = (struct fb_mem *)cam->fb_list.buffer_start;
	
	cam->s_irq.base0 = (uint32_t)fb_mem->ptr;
	cam->fb_list.count = FRAMEBUFFER_COUNT;
	memset(cam->fb_list.buffer_start,0x00,FRAMEBUFFER_COUNT*FRAMEBUFFER_SIZE);
	cam->fb_list.total_frame_count = 0;
	cam_echo("Cam buffer start at 0x%x\r\n",fb_mem->ptr);
#else		
	cam->fb_list.count = (cam->fb_buffer_end - cam->fb_buffer_start) / (size + sizeof(framebuffer_t));
	
	if(cam->fb_list.count <= 1)
		cam_err("FrameBufer too small !!");
	cam_echo("Cam Frame size %d buffer count %d\r\n",size, cam->fb_list.count);
	cam->fb_list.rd_idx = 0;
	cam->fb_list.wr_idx = 0;
	struct fb_mem *fb_mem = (struct fb_mem *)cam->fb_list.buffer_start;
	cam->s_irq.base0 = (uint32_t)&fb_mem->ptr;
#endif	
}

#ifdef __CC_ARM
#define ARMCC_ASM_FUNC	__asm
ARMCC_ASM_FUNC __attribute__((section(".ram_code"))) uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	push	{r4-r7, lr}
10
	LDMIA	R0!, {r3-r6}
	// schedule code carefully to allow dual-issue on Cortex-M7
	bfi		r7, r3, #0, #8	// Y0
	bfi		ip, r5, #0, #8	// Y4
	lsr		r3,	r3,	#16
	lsr		r5,	r5,	#16
	bfi		r7, r3, #8, #8	// Y1
	bfi		ip, r5, #8, #8  // Y5
	bfi		r7, r4, #16, #8 // Y2
	bfi		ip, r6, #16, #8 // Y6
	lsr		r4,	r4,	#16
	lsr		r6,	r6,	#16
	bfi		r7, r4, #24, #8 // Y3
	bfi		ip, r6, #24, #8	// Y7
	STMIA	r1!, {r7, ip}
	
	subs	r2,	#1
	bne		%b10
	mov		r0,	r1
	pop		{r4-r7, pc}
}
#else
__attribute__((naked))
RAM_CODE uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	__asm volatile (
		"	push	{r1-r7, ip, lr}  \n "
		"10:  \n "
		"	ldmia	r0!, {r3-r6}  \n "
			// schedule code carefully to allow dual-issue on Cortex-M7
		"	bfi		r7, r3, #0, #8  \n "	// Y0
		"	bfi		ip, r5, #0, #8  \n "	// Y4
		"	lsr		r3,	r3,	#16  \n "
		"	lsr		r5,	r5,	#16  \n "
		"	bfi		r7, r3, #8, #8  \n "	// Y1
		"	bfi		ip, r5, #8, #8  \n "  // Y5
		"	bfi		r7, r4, #16, #8  \n " // Y2
		"	bfi		ip, r6, #16, #8  \n " // Y6
		"	lsr		r4,	r4,	#16  \n "
		"	lsr		r6,	r6,	#16  \n "
		"	bfi		r7, r4, #24, #8  \n " // Y3
		"	bfi		ip, r6, #24, #8  \n "	// Y7
		"	stmia	r1!, {r7, ip}  \n "	
		"	subs	r2,	#1  \n "
		"	bne		10b  \n "
		"	mov		r0,	r1  \n "
		"	pop		{r1-r7, ip, pc}  \n "		
	);
}

#endif

#ifdef __CC_ARM
__attribute__((section(".ram_code"))) void CSI_IRQHandler(void) {
#else
void CSI_IRQHandler(void) {
#endif	
	if (pCam == NULL)
		return;
	
    uint32_t csisr = pCam->csi_base->CSISR;
    /* Clear the error flags. */
    pCam->csi_base->CSISR = csisr;

	if (csisr & (1<<16)) {
		// VSync
		//               SOF    | FB1    | FB2    irqEn
		pCam->csi_base->CSICR1 = 1U<<16 | 1U<<19 | 1U<<20 | CSICR1_INIT_VAL;
		//				 16 doubleWords| RxFifoDmaReqEn| ReflashRFF|ResetFrmCnt
		pCam->csi_base->CSICR3 = 2<<4          | 1<<12         | 1<<14     |1<<15;
	} else if (csisr & (3<<19))
	{
		uint32_t dmaBase, lineNdx = pCam->s_irq.dmaFragNdx * pCam->s_irq.linePerFrag;
			if (pCam->s_irq.dmaFragNdx & 1)
				dmaBase = pCam->csi_base->CSIDMASA_FB2;
			else
				dmaBase = pCam->csi_base->CSIDMASA_FB1;
		if (dmaBase >= 0x20200000)
			DCACHE_CleanInvalidateByRange(dmaBase, pCam->s_irq.dmaBytePerFrag);
		if (pCam->s_irq.isGray || 
			(pCam->sensor.isWindowing &&  lineNdx >= pCam->sensor.wndY && lineNdx - pCam->sensor.wndY <= pCam->sensor.wndH) )
		{

			dmaBase += pCam->sensor.wndX * 2 * pCam->s_irq.linePerFrag;	// apply line window offset
			if (pCam->s_irq.isGray) {
				
				pCam->s_irq.datCurBase = ExtractYFromYuv(dmaBase, pCam->s_irq.datCurBase, (pCam->sensor.wndW * pCam->s_irq.linePerFrag) >> 3);
			} else {
				uint32_t byteToCopy = (pCam->sensor.wndW * pCam->s_irq.linePerFrag) << 1;
				memcpy((void*)pCam->s_irq.datCurBase, (void*)dmaBase, byteToCopy);
				pCam->s_irq.datCurBase += byteToCopy;
			}
		}
		
		if (++pCam->s_irq.dmaFragNdx == pCam->s_irq.fragCnt || (csisr & (3<<19)) == 3<<19 )
		{
			CSI_Stop(CSI);
			//				 16 doubleWords| ReflashRFF
			pCam->csi_base->CSICR3 = 2<<4		   | 1<<14;
			NVIC_DisableIRQ(CSI_IRQn);
			
			//send msg to cam thread
			rt_mb_send(&csi_frame_mb, 0);
			goto Cleanup;
		}
		
		if (csisr & (1<<19) ) {
			if (!pCam->s_irq.isGray && !pCam->sensor.isWindowing)
				pCam->csi_base->CSIDMASA_FB1 += 2 * pCam->s_irq.dmaBytePerFrag;
		} else {
			if (!pCam->s_irq.isGray && !pCam->sensor.isWindowing)
				pCam->csi_base->CSIDMASA_FB2 += 2 * pCam->s_irq.dmaBytePerFrag;
			pCam->csi_base->CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;	// reflash DMA
		}
	}
Cleanup:
	return;
}

void imx_cam_csi_fragmode_calc(struct imxrt_camera *cam) {
	
	cam->s_irq.datBytePerLine = cam->s_irq.dmaBytePerLine = cam->sensor.fb_w * 2;
	if (cam->sensor.pixformat == PIXFORMAT_GRAYSCALE) {
		cam->s_irq.datBytePerLine /= 2;	// only contain Y
		cam->s_irq.isGray = 1;
		cam->sensor.gs_bpp = 1;
	} else {
		cam->s_irq.isGray = 0;
		cam->sensor.gs_bpp = 2;
	}
	if (cam->sensor.fb_w == 0 || cam->sensor.fb_h == 0)
		return;

	// calculate max bytes per DMA frag
	uint32_t dmaBytePerFrag, byteStep, dmaByteTotal;
	uint32_t maxBytePerLine = sizeof(s_dmaFragBufs) / ARRAY_SIZE(s_dmaFragBufs);
	dmaByteTotal = cam->sensor.fb_w * cam->sensor.fb_h * 2;	
	if (cam->sensor.wndX == 0 && cam->sensor.wndY == 0) // (s_irq.isGray)
	{
		dmaBytePerFrag = cam->s_irq.dmaBytePerLine;  // set a minial default value
		for (byteStep = cam->s_irq.dmaBytePerLine; byteStep < maxBytePerLine; byteStep += cam->s_irq.dmaBytePerLine) {
			if (0 == byteStep % 32 )
			{
				// find maximum allowed bytes per frag
				dmaBytePerFrag = (maxBytePerLine / byteStep) * byteStep;
				for (; dmaBytePerFrag >= byteStep; dmaBytePerFrag -= byteStep) {
					if (dmaByteTotal % dmaBytePerFrag == 0)
						break;
				}
				if (dmaBytePerFrag < byteStep) {
					dmaBytePerFrag = byteStep;
					while (1) {}
				}
				break;
			}
		}
	} 
	else {
		// for window mode, we only accept 1 line per frag
		dmaBytePerFrag = cam->s_irq.dmaBytePerLine;
	}
	cam->s_irq.linePerFrag = dmaBytePerFrag / cam->s_irq.dmaBytePerLine;
	cam->s_irq.dmaBytePerFrag = dmaBytePerFrag;
	cam->s_irq.datBytePerLine = cam->s_irq.isGray ? dmaBytePerFrag / 2 : dmaBytePerFrag;

	// >>> calculate how many lines per fragment (DMA xfer unit)
	uint32_t burstBytes;
	if (!(cam->s_irq.dmaBytePerLine % (8 * 16)))
	{
		burstBytes = 128;
		cam->csi_base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(3U);
		cam->csi_base->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((2U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else if (!(cam->s_irq.dmaBytePerLine % (8 * 8)))
	{
		burstBytes = 64;
		cam->csi_base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(2U);
		cam->csi_base->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((1U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else
	{
		burstBytes = 32;
		cam->csi_base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(1U);
		cam->csi_base->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((0U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	cam->s_irq.fragCnt = cam->sensor.fb_h / cam->s_irq.linePerFrag;
	// <<<
}

void imx_cam_csi_start_frame(struct imxrt_camera *cam)
{	
	imx_cam_csi_fragmode_calc(cam);
	cam->s_irq.dmaFragNdx = 0;
	cam->s_irq.cnt++;
	// DMA also writes to this cache line, to avoid being invalidated, clean MAIN_FB header.
	DCACHE_CleanByRange((uint32_t)cam->s_irq.base0, 32);
	if (cam->s_irq.isGray || cam->sensor.isWindowing) {
		cam->csi_base->CSIDMASA_FB1 = (uint32_t) s_dmaFragBufs[0];
		cam->csi_base->CSIDMASA_FB2 = (uint32_t) s_dmaFragBufs[1];
	} else {
		cam->csi_base->CSIDMASA_FB1 = cam->s_irq.base0;
		cam->csi_base->CSIDMASA_FB2 = cam->s_irq.base0 + cam->s_irq.dmaBytePerFrag;
	}
	cam->s_irq.datCurBase = cam->s_irq.base0; // + s_sensor.wndY * s_irq.datBytePerLine + s_sensor.wndX * s_sensor.gs_bpp;
	cam->csi_base->CSICR1 = CSICR1_INIT_VAL | 1<<16;	// enable SOF iRQ
	if (cam->s_irq.dmaBytePerFrag & 0xFFFF0000) {
		
		uint32_t l16 = cam->s_irq.linePerFrag , h16 = cam->s_irq.dmaBytePerLine << 16;
		cam->csi_base->CSIIMAG_PARA = l16 | h16;
	} else {
		cam->csi_base->CSIIMAG_PARA = 1U | cam->s_irq.dmaBytePerFrag << 16;	// set xfer cnt
	}
	__set_PRIMASK(1);
	cam->csi_base->CSISR = cam->csi_base->CSISR;
	cam->csi_base->CSICR18 |= 1U<<31;	// start CSI
	NVIC_EnableIRQ(CSI_IRQn);
	__set_PRIMASK(0);
}

void imx_cam_start_snapshot(struct imxrt_camera *cam)
{
	pCam = cam;
	//init buffer list
	imxrt_cam_buffer_list_init(cam);
	//start csi
	imx_cam_csi_start_frame(cam);
}

void imx_cam_csi_stop(struct rt_camera_device *cam)
{
	cam->status = RT_CAMERA_DEVICE_SUSPEND;
	CSI_Stop(CSI);
	rt_mb_control(&csi_frame_mb,RT_IPC_CMD_RESET,NULL);
}

void imx_cam_sensor_set_contrast(struct imxrt_camera *cam, uint32_t level)
{
	if (cam->sensor.set_contrast != NULL)
		cam->sensor.set_contrast(&cam->sensor,level);
}

void imx_cam_sensor_set_gainceiling(struct imxrt_camera *cam, gainceiling_t gainceiling)
{
	if (cam->sensor.set_gainceiling != NULL && !cam->sensor.set_gainceiling(&cam->sensor, gainceiling))
		cam->sensor.gainceiling = gainceiling;
}

void imx_cam_sensor_set_framesize(struct imxrt_camera *cam, framesize_t framesize)
{
	if(cam->sensor.set_framesize == NULL || cam->sensor.set_framesize(&cam->sensor,framesize) != 0)
		return;
	
	cam->sensor.framesize = framesize;
	
	cam->fb_list.w = cam->sensor.fb_w = resolution[framesize][0];
	cam->fb_list.h = cam->sensor.fb_h = resolution[framesize][1];
	
	cam->sensor.wndX = 0; cam->sensor.wndY = 0 ; cam->sensor.wndW = cam->sensor.fb_w ; cam->sensor.wndH = cam->sensor.fb_h;
}

void imx_cam_sensor_set_pixformat(struct imxrt_camera *cam, pixformat_t pixformat)
{
	if (cam->sensor.set_pixformat == NULL || cam->sensor.set_pixformat(&cam->sensor,pixformat) != 0)
		return;
	if (cam->sensor.pixformat == pixformat)
		return;
	
	cam->sensor.pixformat = pixformat;
}

static rt_err_t imx_cam_camera_control(struct rt_camera_device *cam, rt_uint32_t cmd, rt_uint32_t parameter)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)cam->imx_cam;
	switch(cmd)
	{
		case RT_DRV_CAM_CMD_RESET:
			if(cam->status == RT_CAMERA_DEVICE_RUNING)
			{
				imx_cam_csi_stop(cam);
			}
			imx_cam_reset(imx_cam);
			cam->status = RT_CAMERA_DEVICE_INIT;
			break;
		case RT_DRV_CAM_CMD_SET_CONTRAST:
			imx_cam_sensor_set_contrast(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_GAINCEILING:
			imx_cam_sensor_set_gainceiling(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_FRAMESIZE:
			imx_cam_sensor_set_framesize(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_PIXFORMAT:
			imx_cam_sensor_set_pixformat(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SNAPSHOT:
		{
			if(cam->status != RT_CAMERA_DEVICE_RUNING)
			{
				//rt_sem_init(cam->sem,"cam", 0, RT_IPC_FLAG_FIFO);
				imx_cam_start_snapshot((struct imxrt_camera *)cam->imx_cam);
				cam->status = RT_CAMERA_DEVICE_RUNING;
			}
			//rest total count 
			imx_cam->fb_list.total_frame_count = 0;
		}
			break;
	}
	
	return RT_EOK;
}

static rt_size_t imx_cam_get_frame_jpeg(struct rt_camera_device *cam, void *frame_ptr)
{
	return 0;
}

static rt_size_t imx_cam_get_frame(struct rt_camera_device *cam, image_t * image)
{
	struct fb_mem *fb_mem,*new_fb_mem; 
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)cam->imx_cam;
	uint32_t tick = 0;
	rt_sem_take(cam->sem, RT_WAITING_FOREVER);
	
	for(int i=0;i<imx_cam->fb_list.count;i++)
	{
		fb_mem = (struct fb_mem *)(imx_cam->fb_buffer_start + i * FRAMEBUFFER_SIZE );
		if(fb_mem->tick > tick)
			new_fb_mem = fb_mem;
	}
	
	image->bpp = imx_cam->fb_list.bbp;
	image->w = imx_cam->fb_list.w;
	image->h = imx_cam->fb_list.h;
	image->pixels = (uint8_t *)new_fb_mem->ptr;
	cam_echo("Cam buf addr:0x%x,tick:%d\r\n",image->pixels,new_fb_mem->tick);
	new_fb_mem->tick = 0;

	return 0;
}

static const struct rt_camera_device_ops imxrt_cam_ops =
{
    .get_frame_jpeg = imx_cam_get_frame_jpeg,
	.get_frame = imx_cam_get_frame,
	.camera_control = imx_cam_camera_control,
};


static void cam_thread_entry(void *parameter)
{
	rt_uint32_t buffer = 0;
	struct rt_camera_device *cam = (struct rt_camera_device *)parameter;
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)cam->imx_cam;
	while(1)
	{
		if (rt_mb_recv(&csi_frame_mb, (rt_ubase_t *)&buffer, RT_WAITING_FOREVER) == RT_EOK)
        {//csi buffer end
			//move wr idx in list
			struct fb_mem *fb_mem = (struct fb_mem *)((uint32_t)imx_cam->fb_list.buffer_start + imx_cam->fb_list.idx * FRAMEBUFFER_SIZE);
			fb_mem->tick = rt_tick_get();
			cam_echo("[%d]Cam Buf ready:0x%x pix addr:0x%x",fb_mem->tick,(fb_mem->ptr),MAIN_FB()->pixels);
			MAIN_FB()->bpp = imx_cam->fb_list.bbp;
			MAIN_FB()->w = imx_cam->fb_list.w;
			MAIN_FB()->h = imx_cam->fb_list.h;
			memcpy(MAIN_FB()->pixels , (fb_mem->ptr), imx_cam->fb_list.frame_size);
			
			imx_cam->fb_list.idx = (imx_cam->fb_list.idx + 1) % imx_cam->fb_list.count;
			fb_mem = (struct fb_mem *)((uint32_t)imx_cam->fb_list.buffer_start + imx_cam->fb_list.idx * FRAMEBUFFER_SIZE);
			imx_cam->s_irq.base0 = (uint32_t)fb_mem->ptr;
			//
			cam_echo(", Cam Buf next:0x%x\r\n",fb_mem->ptr);
			rt_sem_release(cam->sem);
			fb_update_jpeg_buffer(); 
			rt_thread_mdelay(10);
			if (imx_cam->fb_list.total_frame_count < 40)
			{
				imx_cam_csi_start_frame((struct imxrt_camera *)cam->imx_cam);
				imx_cam->fb_list.total_frame_count ++;
			}
			else
			{
				imx_cam_csi_stop(cam);
			}
		}
	}
}

static void imxrt_cam_device_init(struct imxrt_camera *cam)
{
	struct rt_camera_device *device = NULL;
	rt_err_t ret;
	
	device = (struct rt_camera_device *)rt_malloc(sizeof(struct rt_camera_device));
	if (device == NULL)
	{
		cam_err("malloc failed in %s\r\n",__func__);
		return;
	}
	cam_echo("Camera device: %s init\r\n",cam->name);
	device->imx_cam = (uint32_t *)cam;
	cam->rtt_device = device;
	device->status = RT_CAMERA_DEVICE_INIT;
	device->ops = &imxrt_cam_ops;
	imxrt_cam_buffer_list_init(cam);
	device->sem = rt_sem_create("cam", 0, RT_IPC_FLAG_FIFO);
	
	ret = rt_mb_init(&csi_frame_mb, "cam",
        &csi_frame_mb_pool[0], sizeof(csi_frame_mb_pool) / sizeof(csi_frame_mb_pool[0]),
        RT_IPC_FLAG_FIFO);
    RT_ASSERT(ret == RT_EOK);
		
	device->cam_tid = rt_thread_create(cam->name, cam_thread_entry, (void*)device,
                            RT_MAIN_THREAD_STACK_SIZE, RT_MAIN_THREAD_PRIORITY-6, 20);
	
	RT_ASSERT(device->cam_tid != RT_NULL);
	rt_thread_startup(device->cam_tid);

	device->status = RT_CAMERA_DEVICE_SUSPEND;
	
}

struct rt_camera_device * imxrt_camera_device_find(char *name)
{
	int i;
	
	for(i = 0; i < sizeof(cams);i++)
	{
		if(strcmp(name,cams[i].name) == 0)
		{
			return cams[i].rtt_device;
		}
	}
	
	return NULL;
}

int imxrt_camera_width(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_height(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_chip_id(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_pixformat(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_framesize(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_set_windowing(struct rt_camera_device *sensor, int x,int y, int w,int h)
{
	return 0;
}
int imxrt_camera_set_auto_gain(struct rt_camera_device *sensor,int enable, float gain_db, float gain_db_ceiling)
{
	return 0;
}
int imxrt_camera_sensor_get_gain_db(struct rt_camera_device *sensor,float *gain_db)
{
	return 0;
}
int imxrt_camera_set_auto_exposure(struct rt_camera_device *sensor,int enable, int exposure_us)
{
	return 0;
}
int imxrt_camera_get_exposure_us(struct rt_camera_device *sensor, int *us)
{
	return 0;
}
int imxrt_camera_set_auto_whitebal(struct rt_camera_device *sensor,int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
	return 0;
}
int imxrt_camera_get_rgb_gain_db(struct rt_camera_device *sensor,float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
	return 0;
}
int imxrt_camera_set_lens_correction(struct rt_camera_device *sensor,int enable, int radi, int coef)
{
	return 0;
}
uint16_t * imxrt_camera_get_color_palette(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_write_reg(struct rt_camera_device *sensor,uint16_t reg_addr, uint16_t reg_data)
{
	return 0;
}
int imxrt_camera_read_reg(struct rt_camera_device *sensor,uint16_t reg_addr)
{
	return 0;
}

int imxrt_camera_ioctl(struct rt_camera_device *sensor,int request, ... /* arg */)
{
    int ret = -1;
	
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)sensor->imx_cam;
    if (imx_cam->sensor.ioctl != NULL) {
        va_list ap;
        va_start(ap, request);
        /* call the sensor specific function */
        ret = imx_cam->sensor.ioctl(&imx_cam->sensor, request, ap);
        va_end(ap);
    }
    return ret;
}

int rt_camear_init(void)
{
	int i;
	
	for(i = 0; i < CAM_NUM;i++)
	{
		imxrt_cam_device_init(&cams[i]);
	}
	
	return 0;
}


INIT_DEVICE_EXPORT(rt_camear_init);
