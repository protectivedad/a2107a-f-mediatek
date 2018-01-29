/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 10 13 2010 sean.cheng
 * [ALPS00021684] [Need Patch] [Volunteer Patch] CCT new feature
 * .
 *
 * 09 10 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .10y dual sensor
 *
 * 09 02 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .roll back dual sensor
 *
 * 07 27 2010 sean.cheng
 * [ALPS00003112] [Need Patch] [Volunteer Patch] ISP/Sensor driver modification for Customer support
 * .1. add master clock switcher 
 *  2. add master enable/disable 
 *  3. add dummy line/pixel for sensor 
 *  4. add sensor driving current setting
 *
 * 07 01 2010 sean.cheng
 * [ALPS00121215][Camera] Change color when switch low and high 
 * .Add video delay frame.
 *
 * 07 01 2010 sean.cheng
 * [ALPS00002805][Need Patch] [Volunteer Patch]10X Patch for DS269 Video Frame Rate 
 * .Change the sensor clock to let frame rate to be 30fps in vidoe mode
 *
 * 06 13 2010 sean.cheng
 * [ALPS00002514][Need Patch] [Volunteer Patch] ALPS.10X.W10.11 Volunteer patch for E1k Camera 
 * .
 * 1. Add set zoom factor and capdelay frame for YUV sensor 
 * 2. Modify e1k sensor setting
 *
 * 05 25 2010 sean.cheng
 * [ALPS00002250][Need Patch] [Volunteer Patch] ALPS.10X.W10.11 Volunteer patch for YUV video frame rate 
 * .
 * Add 15fps option for video mode
 *
 * 05 03 2010 sean.cheng
 * [ALPS00001357][Meta]CameraTool 
 * .
 * Fix OV2659 YUV sensor frame rate to 30fps in vidoe mode
 *
 * Mar 4 2010 mtk70508
 * [DUMA00154792] Sensor driver
 * 
 *
 * Mar 4 2010 mtk70508
 * [DUMA00154792] Sensor driver
 * 
 *
 * Mar 1 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Feb 24 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Nov 24 2009 mtk02204
 * [DUMA00015869] [Camera Driver] Modifiy camera related drivers for dual/backup sensor/lens drivers.
 * 
 *
 * Oct 29 2009 mtk02204
 * [DUMA00015869] [Camera Driver] Modifiy camera related drivers for dual/backup sensor/lens drivers.
 * 
 *
 * Oct 27 2009 mtk02204
 * [DUMA00015869] [Camera Driver] Modifiy camera related drivers for dual/backup sensor/lens drivers.
 * 
 *
 * Aug 13 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Aug 5 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Jul 17 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Jul 7 2009 mtk01051
 * [DUMA00008051] [Camera Driver] Add drivers for camera high ISO binning mode.
 * Add ISO query info for Sensor
 *
 * May 18 2009 mtk01051
 * [DUMA00005921] [Camera] LED Flashlight first check in
 * 
 *
 * May 16 2009 mtk01051
 * [DUMA00005921] [Camera] LED Flashlight first check in
 * 
 *
 * May 16 2009 mtk01051
 * [DUMA00005921] [Camera] LED Flashlight first check in
 * 
 *
 * Apr 7 2009 mtk02204
 * [DUMA00004012] [Camera] Restructure and rename camera related custom folders and folder name of came
 * 
 *
 * Mar 27 2009 mtk02204
 * [DUMA00002977] [CCT] First check in of MT6516 CCT drivers
 *
 *
 * Mar 25 2009 mtk02204
 * [DUMA00111570] [Camera] The system crash after some operations
 *
 *
 * Mar 20 2009 mtk02204
 * [DUMA00002977] [CCT] First check in of MT6516 CCT drivers
 *
 *
 * Mar 2 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 *
 *
 * Feb 24 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 *
 *
 * Dec 27 2008 MTK01813
 * DUMA_MBJ CheckIn Files
 * created by clearfsimport
 *
 * Dec 10 2008 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 *
 *
 * Oct 27 2008 mtk01051
 * [DUMA00000851] Camera related drivers check in
 * Modify Copyright Header
 *
 * Oct 24 2008 mtk02204
 * [DUMA00000851] Camera related drivers check in
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"

#include "ov2659yuv_Sensor.h"
#include "ov2659yuv_Camera_Sensor_para.h"
#include "ov2659yuv_CameraCustomized.h"

#define OV2659YUV_DEBUG
#ifdef OV2659YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

//extern bool g_bIsAtvStart ;
//extern bool g_bIsNtscArea ;
bool g_bIsAtvStart = 0;
bool g_bIsNtscArea = 0;
#define OV2659_write_cmos_sensor_2(addr, para, bytes) iWriteReg((u16) addr , (u32) para ,bytes,OV2659_WRITE_ID)
void OV2659_write_cmos_sensor(kal_uint32 addr, kal_uint8 para) //iWriteReg((u16) addr , (u32) para ,1,OV2659_WRITE_ID)
{
	if (g_bIsAtvStart)
		return;
	iWriteReg((u16) addr , (u32) para ,1,OV2659_WRITE_ID);
}
kal_uint16 OV2659_read_cmos_sensor(kal_uint32 addr)
{
	if (g_bIsAtvStart) 
		return 0;
	
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV2659_WRITE_ID);
    
    return get_byte;
}


/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/


#define	OV2659_LIMIT_EXPOSURE_LINES				(1253)
#define	OV2659_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	OV2659_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	OV2659_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 
static kal_uint8 OV2659_exposure_line_h = 0, OV2659_exposure_line_l = 0,OV2659_extra_exposure_line_h = 0, OV2659_extra_exposure_line_l = 0;

static kal_bool OV2659_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool OV2659_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool OV2659_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 OV2659_dummy_pixels=0, OV2659_dummy_lines=0;

static kal_uint16 OV2659_exposure_lines=0, OV2659_extra_exposure_lines = 0;


static kal_int8 OV2659_DELAY_AFTER_PREVIEW = -1;

static kal_uint8 OV2659_Banding_setting = AE_FLICKER_MODE_50HZ;  //Wonder add

/****** OVT 6-18******/
static kal_uint16 OV2659_Capture_Max_Gain16= 6*16;
static kal_uint16 OV2659_Capture_Gain16=0 ;    
static kal_uint16 OV2659_Capture_Shutter=0;
static kal_uint16 OV2659_Capture_Extra_Lines=0;

static kal_uint16  OV2659_PV_Dummy_Pixels =0, OV2659_Capture_Dummy_Pixels =0, OV2659_Capture_Dummy_Lines =0;
static kal_uint16  OV2659_PV_Gain16 = 0;
static kal_uint16  OV2659_PV_Shutter = 0;
static kal_uint16  OV2659_PV_Extra_Lines = 0;

kal_uint16 OV2659_sensor_gain_base=0,OV2659_FAC_SENSOR_REG=0,OV2659_iOV2659_Mode=0,OV2659_max_exposure_lines=0;
kal_uint32 OV2659_capture_pclk_in_M=520,OV2659_preview_pclk_in_M=390,OV2659_PV_dummy_pixels=0,OV2659_PV_dummy_lines=0,OV2659_isp_master_clock=0;


static kal_uint32  OV2659_sensor_pclk=390;
static kal_bool OV2659_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV2659_AE_ENABLE = KAL_TRUE; 


int m_iCombo_Night_mode = 0;
// 0: off, 1: on
int XVCLK = 2600;
// real clock/10000
int preview_sysclk, preview_HTS;

typedef struct 
{
    u16 nReg;
    u32 nVal;
}sensor_register;

sensor_register OV2659_init_array[] =
{
    {0x3000, 0x0f},
    {0x3001, 0xff},
    {0x3002, 0xff},
    {0x0100, 0x01},
    {0x3633, 0x3d},
    {0x3620, 0x02},
    {0x3631, 0x11},
    {0x3612, 0x04},
    {0x3630, 0x20},
    {0x4702, 0x02},
    {0x370c, 0x34},
    {0x3004, 0x10},
    {0x3005, 0x16},
    // Timing   
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0x00},
    {0x3804, 0x06},
    {0x3805, 0x5f},
    {0x3806, 0x04},
    {0x3807, 0xb7},
    {0x3808, 0x03},
    {0x3809, 0x20},
    {0x380a, 0x02},
    {0x380b, 0x58},
    {0x380c, 0x05},
    {0x380d, 0x14},
    {0x380e, 0x02},
    {0x380f, 0x68},
    {0x3811, 0x08},
    {0x3813, 0x02},
    {0x3814, 0x31},
    {0x3815, 0x31},

    // banding filter
    {0x3a00, 0x38}, // disable exposure less than 1 line. enable banding, exposure less than banding
    {0x3a08, 0x00},// B50
    {0x3a09, 0x5c},// B50
    {0x3a0a, 0x00},// B60
    {0x3a0b, 0x4d},// B60
    {0x3a0d, 0x08},// max band 60Hz
    {0x3a0e, 0x06},// max band 50Hz
    {0x3a05, 0x30}, // select 50hz banding filter
    // night mode // max exposure 60Hz, 5fps
    {0x3a02, 0x07}, // max exposure 60Hz
    {0x3a03, 0x30}, // max exposure 50Hz, 5fps
    {0x3a14, 0x07}, // max exposure 50Hz
    {0x3a15, 0x30}, 
    {0x3623, 0x00},
    {0x3634, 0x76},
    {0x3701, 0x44},
    {0x3702, 0x18},
    {0x3703, 0x24},
    {0x3704, 0x24},
    {0x3705, 0x0c},
    // define group 1 for preview
    {0x3208, 0x01},
    // enable group 1
    {0x3702, 0x18},
    {0x3703, 0x24},
    {0x3704, 0x24},
    {0x3208, 0x11},// end group 1
    // define group 2 for capture
    {0x3208, 0x02},// enable group 2
    {0x3702, 0x30},
    {0x3703, 0x48},
    {0x3704, 0x48},
    {0x3208, 0x12},// end group 1
    // mirror, flip, binning
    {0x3820, 0x87},
    {0x3821, 0x07},
    {0x370a, 0x52},
    {0x4608, 0x00},
    {0x4609, 0x80},
    {0x4300, 0x30},
    {0x5086, 0x02},

    // ISP      
    {0x5000, 0xff},
    {0x5001, 0x1f}, // ISP on, Gamma on, AWB statistic on, AWB gain on, Lenc on,
    {0x5002, 0x00}, // LCD adjustment off, BPC on, WPC on      // Lenc gain on, SDE on, UV average on, color matrix on, interpolation on
                      // Scale off
    // AEC // Stable in high
    {0x3a0f,0x42},
    {0x3a10,0x3A},
    {0x3a11,0x71},
    {0x3a1b,0x42},
    {0x3a1e,0x3A},
    {0x3a1f,0x10},

    {0x5060, 0x69}, // Window 4~7
    {0x5061, 0x7d}, // window 8~11
    {0x5062, 0x7d}, // window 12~15
    {0x5063, 0x69}, // gain ceiling
    {0x3a18, 0x00}, // gain ceiling 4x
    {0x3a19, 0x40}, 

    // Lens Correction
    {0x500c,0x03},  
    {0x500d,0x20},  
    {0x500e,0x02},  
    {0x500f,0x5C},  
    {0x5010,0x5a},  
    {0x5011,0x00},  
    {0x5012,0x66}, 
    {0x500c,0x03},  
    {0x500d,0x20},  
    {0x500e,0x02},  
    {0x500f,0x5C},  
    {0x5010,0x5a},  
    {0x5011,0x00},  
    {0x5012,0x66},  
    {0x5013,0x03},
    {0x5014,0x20},
    {0x5015,0x02},
    {0x5016,0x5c},
                  
    {0x5017,0x54},
    {0x5018,0x00},
    {0x5019,0x66},
                  
    {0x501a,0x03},
    {0x501b,0x20},
    {0x501c,0x02},
    {0x501d,0x5c},
                  
    {0x501e,0x4e},
    {0x501f,0x00},
    {0x5020,0x66},

    // AWB      
    {0x5035,0x68},
    {0x5036,0x11},
    {0x5037,0x92},
    {0x5038,0x21},
    {0x5039,0xe1},
    {0x503a,0x1 },
    {0x503c,0x3 },
    {0x503d,0xd },
    {0x503e,0x17},
    {0x503f,0x67},
    {0x5040,0x54},
    {0x5041,0x7 },
    {0x5042,0x8c},
    {0x5043,0x3a},
    {0x5044,0x43},
    {0x5045,0x63},
    {0x5046,0x30},
    {0x5047,0xf8},
    {0x5048,0x8 },
    {0x5049,0x70},
    {0x504a,0xf0},
    {0x504b,0xf0},

    // Gamma of OV2655
    {0x5025, 0x0f},
    {0x5026, 0x1b},
    {0x5027, 0x28},
    {0x5028, 0x39},
    {0x5029, 0x4a},
    {0x502a, 0x55},
    {0x502b, 0x64},
    {0x502c, 0x71},
    {0x502d, 0x7e},
    {0x502e, 0x92},
    {0x502f, 0xa1},
    {0x5030, 0xaf},
    {0x5031, 0xc5},
    {0x5032, 0xd7},
    {0x5033, 0xe5},
    {0x5034, 0x24},
    // Color Matrix
    {0x5070,0x20}, 
    {0x5071,0x58}, 
    {0x5072,0x8 }, 
    {0x5073,0x20}, 
    {0x5074,0xac}, 
    {0x5075,0xcc}, 
    {0x5076,0xb4}, 
    {0x5077,0xb7}, 
    {0x5078,0x4 }, 
    {0x5079,0x98}, 
    {0x507a,0x0 }, 

    // SharpnessDe-noise
    {0x506e, 0x44},
    {0x5064, 0x08},
    {0x5065, 0x10},
    {0x5066, 0x0c},
    {0x5067, 0x01},
    {0x506c, 0x08},
    {0x506d, 0x10},
    {0x506f, 0xa6},
    {0x5068, 0x08},
    {0x5069, 0x10},
    {0x506a, 0x04},
    {0x506b, 0x12},
    {0x4003, 0x88},
    // Effect   
    {0x507e, 0x40},
    {0x507f, 0x10},
    {0x507b, 0x02},
    {0x507a, 0x00},
    {0x5084, 0x1f},
    {0x5085, 0x3f},
    {0x5005, 0x80},
    {0x3004, 0x20}, // divider
    // LCD Adjustment // LCD red gain
    {0x5051, 0x40}, // LCD green gain
    {0x5052, 0x40}, // LCD blue gain
    {0x5053, 0x40}, 

};
#if WINMO_USE
kal_uint8 OV2659_sensor_write_I2C_address = OV2659_WRITE_ID;
kal_uint8 OV2659_sensor_read_I2C_address = OV2659_READ_ID;

UINT32 OV2659GPIOBaseAddr;
HANDLE OV2659hGPIO;
HANDLE OV2659hDrvI2C;
I2C_TRANSACTION OV2659I2CConfig;
#endif 
UINT8 OV2659_PixelClockDivider=0;

//SENSOR_REG_STRUCT OV2659SensorCCT[FACTORY_END_ADDR]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
//SENSOR_REG_STRUCT OV2659SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
//	camera_para.SENSOR.cct	SensorCCT	=> SensorCCT
//	camera_para.SENSOR.reg	SensorReg
MSDK_SENSOR_CONFIG_STRUCT OV2659SensorConfigData;

#if WINMO_USE
void OV2659_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	OV2659I2CConfig.operation=I2C_OP_WRITE;
	OV2659I2CConfig.slaveAddr=OV2659_sensor_write_I2C_address>>1;
	OV2659I2CConfig.transfer_num=1;	/* TRANSAC_LEN */
	OV2659I2CConfig.transfer_len=3;
	OV2659I2CConfig.buffer[0]=(UINT8)(addr>>8);
	OV2659I2CConfig.buffer[1]=(UINT8)(addr&0xFF);
	OV2659I2CConfig.buffer[2]=(UINT8)para;
	DRV_I2CTransaction(OV2659hDrvI2C, &OV2659I2CConfig);

}	/* OV2659_write_cmos_sensor() */

kal_uint32 OV2659_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint8 get_byte=0xFF;

	OV2659I2CConfig.operation=I2C_OP_WRITE;
	OV2659I2CConfig.slaveAddr=OV2659_sensor_write_I2C_address>>1;
	OV2659I2CConfig.transfer_num=1;	/* TRANSAC_LEN */
	OV2659I2CConfig.transfer_len=2;
	OV2659I2CConfig.buffer[0]=(UINT8)(addr>>8);
	OV2659I2CConfig.buffer[1]=(UINT8)(addr&0xFF);
	DRV_I2CTransaction(OV2659hDrvI2C, &OV2659I2CConfig);

	OV2659I2CConfig.operation=I2C_OP_READ;
	OV2659I2CConfig.slaveAddr=OV2659_sensor_read_I2C_address>>1;
	OV2659I2CConfig.transfer_num=1;	/* TRANSAC_LEN */
	OV2659I2CConfig.transfer_len=1;
	DRV_I2CTransaction(OV2659hDrvI2C, &OV2659I2CConfig);
	get_byte=OV2659I2CConfig.buffer[0];

	return get_byte;
}	/* OV2659_read_cmos_sensor() */
#endif 
void OV2659_set_dummy(kal_uint32 dummy_pixels, kal_uint32 dummy_lines)
{
    /*kal_uint8 temp_reg1, temp_reg2;
    kal_uint16 temp_reg;

    if (pixels >= 2156) 
        pixels = 2155;
    if (pixels < 0x100)
    {
        OV2659_write_cmos_sensor(0x302c,(pixels&0xFF)); //EXHTS[7:0]
        temp_reg = OV2659_FULL_PERIOD_PIXEL_NUMS;
        OV2659_write_cmos_sensor(0x3029,(temp_reg&0xFF));         //H_length[7:0]
        OV2659_write_cmos_sensor(0x3028,((temp_reg&0xFF00)>>8));  //H_length[15:8]
    }
    else
    {
        OV2659_write_cmos_sensor(0x302c,0);
        temp_reg = pixels + OV2659_FULL_PERIOD_PIXEL_NUMS;
        OV2659_write_cmos_sensor(0x3029,(temp_reg&0xFF));         //H_length[7:0]
        OV2659_write_cmos_sensor(0x3028,((temp_reg&0xFF00)>>8));  //H_length[15:8]
    }

    // read out and + line
    temp_reg1 = OV2659_read_cmos_sensor(0x302B);    // VTS[b7~b0]
    temp_reg2 = OV2659_read_cmos_sensor(0x302A);    // VTS[b15~b8]
    temp_reg = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

    temp_reg += lines;

    OV2659_write_cmos_sensor(0x302B,(temp_reg&0xFF));         //VTS[7:0]
    OV2659_write_cmos_sensor(0x302A,((temp_reg&0xFF00)>>8));  //VTS[15:8]*/

	kal_uint32 temp_reg1, temp_reg2;
	kal_uint32 temp_reg, base_shutter = 0x9B;
	
	if (dummy_pixels > 0)
	{
		temp_reg1 = OV2659_read_cmos_sensor(0x380D);    // HTS[b7~b0]
		temp_reg2 = OV2659_read_cmos_sensor(0x380C);    // HTS[b15~b8]
		temp_reg = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
	
		temp_reg += dummy_pixels;
	
		OV2659_write_cmos_sensor(0x380D,(temp_reg&0xFF));         //HTS[7:0]
		OV2659_write_cmos_sensor(0x380C,((temp_reg&0xFF00)>>8));  //HTS[15:8]
	}

	if (dummy_lines > 0)
	{
		temp_reg1 = OV2659_read_cmos_sensor(0x380F);    // VTS[b7~b0]
		temp_reg2 = OV2659_read_cmos_sensor(0x380E);    // VTS[b15~b8]
		temp_reg = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
	
		temp_reg += dummy_lines;
	
		OV2659_write_cmos_sensor(0x380F,(temp_reg&0xFF));         //VTS[7:0]
		OV2659_write_cmos_sensor(0x380E,((temp_reg&0xFF00)>>8));  //VTS[15:8]
	}

}    /* OV2659_set_dummy */

kal_uint16 OV2659_get_gain16(void)
{
    // read gain, 16 = 1x
    int gain16;
    gain16 = OV2659_read_cmos_sensor(0x350a) & 0x03;
    gain16 = (gain16<<8) + OV2659_read_cmos_sensor(0x350b);
    return gain16;
}  /* OV2659_read_OV2659_gain */

kal_uint16 OV2659_get_shutter(void)
{
    // read shutter, in number of line period
    int shutter;
    shutter = (OV2659_read_cmos_sensor(0x03500) & 0x0f);
    shutter = (shutter<<8) + OV2659_read_cmos_sensor(0x3501);

    shutter = (shutter<<4) + (OV2659_read_cmos_sensor(0x3502)>>4);
    return shutter ;

}    /* OV2659_read_shutter */

void OV2659_set_gain16(kal_uint16 gain16)
{    
    // write gain, 16 = 1x
    int temp;
    gain16 = gain16 & 0x3ff;
    temp = gain16 & 0xff;
    OV2659_write_cmos_sensor(0x350b, temp);
    temp = gain16>>8;
    OV2659_write_cmos_sensor(0x350a, temp);
    return 0;

} 

static void OV2659_set_shutter(kal_uint16 shutter)
{
    // write shutter, in number of line period
    int temp;
    shutter = shutter & 0xffff;
    temp = shutter & 0x0f;
    temp = temp<<4;
    OV2659_write_cmos_sensor(0x3502, temp);
    temp = shutter & 0xfff;
    temp = temp>>4;
    OV2659_write_cmos_sensor(0x3501, temp);
    temp = shutter>>12;
    OV2659_write_cmos_sensor(0x3500, temp);
    return 0;

}    /* OV2659_write_shutter */

int OV2659_get_sysclk()
{
    // calculate sysclk
    int PCLK, temp1, temp2;
    int Pre_div2x, PLLDiv, FreqDiv2x, Bit8Div, SysDiv, ScaleDiv, VCO;
    int Pre_div2x_map[] = {
    2, 3, 4, 6, 4, 6, 8, 12};
    int FreqDiv2x_map[] = {
    2, 3, 4, 6};
    int Bit8Div_map[] = {
    1, 1, 4, 5};
    int SysDiv_map[] = {
    1, 2, 8, 16};
    int ScaleDiv_map[] = {
    1, 2, 4, 6, 8, 10, 12, 14};
    temp1 = OV2659_read_cmos_sensor(0x3003);
    temp2 = temp1>>6;
    FreqDiv2x = FreqDiv2x_map[temp2];
    temp2 = temp1 & 0x03;
    Bit8Div = Bit8Div_map[temp2];
    temp1 = OV2659_read_cmos_sensor(0x3004);
    temp2 = temp1 >>4;
    ScaleDiv = ScaleDiv_map[temp2];
    temp1 = OV2659_read_cmos_sensor(0x3005);
    PLLDiv = temp1 & 0x3f;
    temp1 = OV2659_read_cmos_sensor(0x3006);
    temp2 = temp1 & 0x07;
    Pre_div2x = Pre_div2x_map[temp2];
    temp2 = temp1 & 0x18;
    temp2 = temp2>>3;
    SysDiv = SysDiv_map[temp2];
    VCO = XVCLK * PLLDiv * FreqDiv2x * Bit8Div / Pre_div2x;
    PCLK = VCO / SysDiv / ScaleDiv / 4;
    return PCLK;
}

int OV2659_get_HTS()
{
    // read HTS from register settings
    int HTS;
    HTS = OV2659_read_cmos_sensor(0x380c);
    HTS = (HTS<<8) + OV2659_read_cmos_sensor(0x380d);
    return HTS;
}
int OV2659_get_VTS()
{
    // read VTS from register settings
    int VTS;
    VTS = OV2659_read_cmos_sensor(0x380e);
    VTS = (VTS<<8) + OV2659_read_cmos_sensor(0x380f);
    return VTS;
}

int OV2659_set_VTS(int VTS)
{
    // write VTS to registers
    int temp;
    temp = VTS & 0xff;
    OV2659_write_cmos_sensor(0x380f, temp);
    temp = VTS>>8;
    OV2659_write_cmos_sensor(0x380e, temp);
    return 0;
}
int OV2659_get_binning_factor()
{
    // read VTS from register settings
    int temp, binning;
    temp = OV2659_read_cmos_sensor(0x3821);
    if (temp & 0x01) {
        binning = 2;
    }
        else {
        binning = 1;
    }
    return binning;
}

int OV2659_get_light_frequency()
{
    // get light frequency
    int temp, light_frequency;
    temp = OV2659_read_cmos_sensor(0x3a05);
    if (temp & 0x80) {
        // 60Hz
        light_frequency = 60;
    }
    else {
        // 50Hz
        light_frequency = 50;
    }
    return light_frequency;
}
void OV2659_set_bandingfilter()
{
    int preview_VTS;
    int band_step60, max_band60, band_step50, max_band50;
    // read preview PCLK
    preview_sysclk = OV2659_get_sysclk();
    // read preview HTS
    preview_HTS = OV2659_get_HTS();
    // read preview VTS
    preview_VTS = OV2659_get_VTS();
    // calculate banding filter
    // 60Hz
    band_step60 = preview_sysclk * 100/preview_HTS * 100/120;
    OV2659_write_cmos_sensor(0x3a0a, (band_step60 >> 8));
    OV2659_write_cmos_sensor(0x3a0b, (band_step60 & 0xff));
    max_band60 = (int)((preview_VTS-4)/band_step60);
    OV2659_write_cmos_sensor(0x3a0d, max_band60);
    // 50Hz
    band_step50 = preview_sysclk * 100/preview_HTS;
    OV2659_write_cmos_sensor(0x3a08, (band_step50 >> 8));
    OV2659_write_cmos_sensor(0x3a09, (band_step50 & 0xff));
    max_band50 = (int)((preview_VTS-4)/band_step50);
    OV2659_write_cmos_sensor(0x3a0e, max_band50);
}
void OV2659_ChangeNightMode(int NightMode)
{
    int temp;
    switch (NightMode)
    {
        case 0://Off
            temp = OV2659_read_cmos_sensor(0x3a00);
            temp = temp & 0xfb;
            OV2659_write_cmos_sensor(0x3a00, temp);
        // night mode off, bit[2] = 0
        break;
        case 1:// on
            temp = OV2659_read_cmos_sensor(0x3a00);
            temp = temp | 0x04;
            OV2659_write_cmos_sensor(0x3a00, temp);
        break;
        // night mode on, bit[2] = 1
        default:
        break;
    }
}

/*************************************************************************
* FUNCTION
*	OV2659_night_mode
*
* DESCRIPTION
*	This function night mode of OV2659.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV2659_night_mode(kal_bool enable)
{
    SENSORDB("[Enter]:OV2659_night_mode enable=%d",enable);

    m_iCombo_Night_mode=enable;
    if (enable)
    {
    	OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL@@ SVGA night mode on 15~5fps
    	OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
    	OV2659_write_cmos_sensor(0x3005,0x16);//	; PLL
    	OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider
    	OV2659_write_cmos_sensor(0x3a00,0x7c);//	; night mode on
    	OV2659_write_cmos_sensor(0x3a02,0x07);//	; max exposure 60Hz
    	OV2659_write_cmos_sensor(0x3a03,0x38);//	; max exposure 60Hz
    	OV2659_write_cmos_sensor(0x3a14,0x07);//	; max exposure 50Hz
    	OV2659_write_cmos_sensor(0x3a15,0x30);//	; max exposure 50Hz
    }
    else
    {
    	OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL@@ SVGA 15fps
    	OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
    	OV2659_write_cmos_sensor(0x3005,0x16);//	; PLL
    	OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider
    	OV2659_write_cmos_sensor(0x3a00,0x38);//	; night mode off
    }


}	/* OV2659_night_mode */



/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	OV2659Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2659Open(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
    u16 nOV2659_Init_Register_Length = 0;
       SENSORDB("[Enter]:OV2659Open Open func:");
	if (g_bIsAtvStart)
	{
		 return ERROR_NONE;
	}
	zoom_factor = 0; 
	OV2659_write_cmos_sensor(0x0103,0x01);// Reset sensor
    Sleep(5);

       SENSORDB("[Enter]:OV2659Open 0x0103");
	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (OV2659_read_cmos_sensor(0x300A) << 8) | OV2659_read_cmos_sensor(0x300B);
	    SENSORDB("OV2659Open, sensor_id:%x \n",sensor_id);
		if(sensor_id != OV2659_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	
	RETAILMSG(1, (TEXT("OV2659 Sensor Read ID OK \r\n")));
    OV2659_write_cmos_sensor(0x0103,0x01);//	; software reset
    Sleep(5);
    OV2659_write_cmos_sensor(0x3014,0x20);//
    //delay(5ms)
    #if 0
    OV2659_write_cmos_sensor(0x3000,0x0f);//	; D[9:8] output enable
    OV2659_write_cmos_sensor(0x3001,0xff);//	; D[7:0] output enable
    OV2659_write_cmos_sensor(0x3002,0xff);//	; Vsync, Href, PCLK output enable
    OV2659_write_cmos_sensor(0x0100,0x01);//	; Wake up from software power down
    OV2659_write_cmos_sensor(0x3633,0x3d);//
    OV2659_write_cmos_sensor(0x3620,0x02);//
    OV2659_write_cmos_sensor(0x3631,0x11);//
    OV2659_write_cmos_sensor(0x3612,0x04);//
    OV2659_write_cmos_sensor(0x3630,0x20);//
    OV2659_write_cmos_sensor(0x4702,0x02);//
    OV2659_write_cmos_sensor(0x370c,0x34);//
    OV2659_write_cmos_sensor(0x3004,0x10);//	; PLL
    OV2659_write_cmos_sensor(0x3005,0x16);//	; PLL
    OV2659_write_cmos_sensor(0x3800,0x00);//	; HS
    OV2659_write_cmos_sensor(0x3801,0x00);//	; HS
    OV2659_write_cmos_sensor(0x3802,0x00);//	; VS
    OV2659_write_cmos_sensor(0x3803,0x00);//	; VS
    OV2659_write_cmos_sensor(0x3804,0x06);//	; HW = 1631
    OV2659_write_cmos_sensor(0x3805,0x5f);//	; HW
    OV2659_write_cmos_sensor(0x3806,0x04);//	; VH = 1207
    OV2659_write_cmos_sensor(0x3807,0xb7);//	; VH
    OV2659_write_cmos_sensor(0x3808,0x03);//	; DVPHO = 800
    OV2659_write_cmos_sensor(0x3809,0x20);//	; DVPHO
    OV2659_write_cmos_sensor(0x380a,0x02);//	; DVPVO = 600
    OV2659_write_cmos_sensor(0x380b,0x58);//	; DVPVO
    OV2659_write_cmos_sensor(0x380c,0x05);//	; HTS = 1300
    OV2659_write_cmos_sensor(0x380d,0x14);//	; HTS
    OV2659_write_cmos_sensor(0x380e,0x02);//	; VTS = 616
    OV2659_write_cmos_sensor(0x380f,0x68);//	; VTS
    OV2659_write_cmos_sensor(0x3811,0x08);//	; H offset
    OV2659_write_cmos_sensor(0x3813,0x02);//	; H offset
    OV2659_write_cmos_sensor(0x3814,0x31);//	; V offset
    OV2659_write_cmos_sensor(0x3815,0x31);//	; V offset
    OV2659_write_cmos_sensor(0x3a02,0x02);//	; max exposure for 60Hz
    OV2659_write_cmos_sensor(0x3a03,0x68);//	; max exposure for 60Hz
    OV2659_write_cmos_sensor(0x3a08,0x00);//	; B50
    OV2659_write_cmos_sensor(0x3a09,0x5c);//	; B50
    OV2659_write_cmos_sensor(0x3a0a,0x00);//	; B60
    OV2659_write_cmos_sensor(0x3a0b,0x4d);//	; B60
    OV2659_write_cmos_sensor(0x3a0d,0x08);//	; max band for 60Hz
    OV2659_write_cmos_sensor(0x3a0e,0x06);//	; max band for 50Hz
    OV2659_write_cmos_sensor(0x3a14,0x02);//	; max exposure for 50Hz
    OV2659_write_cmos_sensor(0x3a15,0x28);//	; max exposure for 50Hz
    OV2659_write_cmos_sensor(0x3623,0x00);//
    OV2659_write_cmos_sensor(0x3634,0x76);//
    OV2659_write_cmos_sensor(0x3701,0x44);//
    OV2659_write_cmos_sensor(0x3702,0x18);//
    OV2659_write_cmos_sensor(0x3703,0x24);//
    OV2659_write_cmos_sensor(0x3704,0x24);//
    OV2659_write_cmos_sensor(0x3705,0x0c);//
    OV2659_write_cmos_sensor(0x3820,0x86);//	; Horizontal binning on
    OV2659_write_cmos_sensor(0x3821,0x06);//	; Vertical binning on
    OV2659_write_cmos_sensor(0x370a,0x52);//
    OV2659_write_cmos_sensor(0x4608,0x00);//	; FIFO read start
    OV2659_write_cmos_sensor(0x4609,0x80);//	; FIFO read start
    OV2659_write_cmos_sensor(0x4300,0x30);//	; YUV 422 YUYV
    OV2659_write_cmos_sensor(0x5086,0x02);//
    OV2659_write_cmos_sensor(0x5000,0xfb);//	; ISP on, Gamma on, AWB statistic on, AWB gain on, Lenc on, LCD adjustment off, BPC on, WPC on
    OV2659_write_cmos_sensor(0x5001,0x1f);//	; Lenc gain on, SDE on, UV average on, color matrix on, interpolation on
    OV2659_write_cmos_sensor(0x5002,0x00);//	; Scale off

    OV2659_write_cmos_sensor(0x3406,0x00);//	; Light mode auto
    OV2659_write_cmos_sensor(0x500c,0x03);                          
    OV2659_write_cmos_sensor(0x500d,0x20);                          
    OV2659_write_cmos_sensor(0x500e,0x02);                          
    OV2659_write_cmos_sensor(0x500f,0x5C);                          
    OV2659_write_cmos_sensor(0x5010,0x5a);                          
    OV2659_write_cmos_sensor(0x5011,0x00);                          
    OV2659_write_cmos_sensor(0x5012,0x66);                          
    //G                                                             
    OV2659_write_cmos_sensor(0x5013,0x03);                          
    OV2659_write_cmos_sensor(0x5014,0x20);//;32                     
    OV2659_write_cmos_sensor(0x5015,0x02);                          
    OV2659_write_cmos_sensor(0x5016,0x5c);                          
                                                                    
    OV2659_write_cmos_sensor(0x5017,0x54);                          
    OV2659_write_cmos_sensor(0x5018,0x00);                          
    OV2659_write_cmos_sensor(0x5019,0x66);                          
    //B                                                             
    OV2659_write_cmos_sensor(0x501a,0x03);                          
    OV2659_write_cmos_sensor(0x501b,0x20);                          
    OV2659_write_cmos_sensor(0x501c,0x02);                          
    OV2659_write_cmos_sensor(0x501d,0x5c);                          
                                                                    
    OV2659_write_cmos_sensor(0x501e,0x4e);                          
    OV2659_write_cmos_sensor(0x501f,0x00);                          
    OV2659_write_cmos_sensor(0x5020,0x66);                          
                                                                    
    //cmx 115%                                                      
    OV2659_write_cmos_sensor(0x5070,0x20);                          
    OV2659_write_cmos_sensor(0x5071,0x58);                          
    OV2659_write_cmos_sensor(0x5072,0x8 );                          
    OV2659_write_cmos_sensor(0x5073,0x20);                          
    OV2659_write_cmos_sensor(0x5074,0xac);                          
    OV2659_write_cmos_sensor(0x5075,0xcc);                          
    OV2659_write_cmos_sensor(0x5076,0xb4);                          
    OV2659_write_cmos_sensor(0x5077,0xb7);                          
    OV2659_write_cmos_sensor(0x5078,0x4 );                          
    OV2659_write_cmos_sensor(0x5079,0x98);                          
    OV2659_write_cmos_sensor(0x507a,0x0 );                          
                                                                    
    //GAMMA                                                         
    OV2659_write_cmos_sensor(0x5025,0xf );
    OV2659_write_cmos_sensor(0x5026,0x1b);
    OV2659_write_cmos_sensor(0x5027,0x28);
    OV2659_write_cmos_sensor(0x5028,0x39);
    OV2659_write_cmos_sensor(0x5029,0x4a);
    OV2659_write_cmos_sensor(0x502a,0x55);
    OV2659_write_cmos_sensor(0x502b,0x64);
    OV2659_write_cmos_sensor(0x502c,0x71);
    OV2659_write_cmos_sensor(0x502d,0x7e);
    OV2659_write_cmos_sensor(0x502e,0x92);
    OV2659_write_cmos_sensor(0x502f,0xa1);
    OV2659_write_cmos_sensor(0x5030,0xaf);
    OV2659_write_cmos_sensor(0x5031,0xc5);
    OV2659_write_cmos_sensor(0x5032,0xd7);
    OV2659_write_cmos_sensor(0x5033,0xe5);
    OV2659_write_cmos_sensor(0x5034,0x24);
                                                                    
                                                                    
                                                                    
    //awb                                                           
    OV2659_write_cmos_sensor(0x5035,0x68);//;6a                     
    OV2659_write_cmos_sensor(0x5036,0x11);                          
    OV2659_write_cmos_sensor(0x5037,0x92);                          
    OV2659_write_cmos_sensor(0x5038,0x21);                          
    OV2659_write_cmos_sensor(0x5039,0xe1);                          
    OV2659_write_cmos_sensor(0x503a,0x1 );                          
    OV2659_write_cmos_sensor(0x503c,0x3 );                          
    OV2659_write_cmos_sensor(0x503d,0xd );                          
    OV2659_write_cmos_sensor(0x503e,0x17);                          
    OV2659_write_cmos_sensor(0x503f,0x67);                          
    OV2659_write_cmos_sensor(0x5040,0x54);                          
    OV2659_write_cmos_sensor(0x5041,0x7 );                          
    OV2659_write_cmos_sensor(0x5042,0x8c);                          
    OV2659_write_cmos_sensor(0x5043,0x3a);                          
    OV2659_write_cmos_sensor(0x5044,0x43);                          
    OV2659_write_cmos_sensor(0x5045,0x63);                          
    OV2659_write_cmos_sensor(0x5046,0x30);                          
    OV2659_write_cmos_sensor(0x5047,0xf8);                          
    OV2659_write_cmos_sensor(0x5048,0x8 );                          
    OV2659_write_cmos_sensor(0x5049,0x70);                          
    OV2659_write_cmos_sensor(0x504a,0xf0);                          
    OV2659_write_cmos_sensor(0x504b,0xf0);                          
                                                                    
                                                                    
    OV2659_write_cmos_sensor(0x3a00,0x38);                          
                                                                    
    OV2659_write_cmos_sensor(0x3a19,0x40);//;maximum 4x gain        
                                                                    
                                                                    
           //AE target                                              
    OV2659_write_cmos_sensor(0x3a0f,0x42);//; 48 ;80                
    OV2659_write_cmos_sensor(0x3a10,0x3A);//;40 ;74                 
    OV2659_write_cmos_sensor(0x3a11,0x71);// ;d4                    
    OV2659_write_cmos_sensor(0x3a1b,0x42);//;40 ;80                 
    OV2659_write_cmos_sensor(0x3a1e,0x3A);//;38 ;74                 
    OV2659_write_cmos_sensor(0x3a1f,0x10);// ;d4                    
                                                                    
                                                                    
     //  Auto Sharpness                                             
    OV2659_write_cmos_sensor(0x5064,0x08);
    OV2659_write_cmos_sensor(0x5065,0x10);
    OV2659_write_cmos_sensor(0x5066,0x0c);
    OV2659_write_cmos_sensor(0x5067,0x01);
    OV2659_write_cmos_sensor(0x506c,0x08);
    OV2659_write_cmos_sensor(0x506d,0x10);
    OV2659_write_cmos_sensor(0x506e,0x44);
    OV2659_write_cmos_sensor(0x506f,0xA6);
                                                                    
      //DNS Auto Default                                            
    //OV2659_write_cmos_sensor(0x605,0x6e);// 00 20 ;auto-denoise    
    OV2659_write_cmos_sensor(0x5068,0x08);                          
    OV2659_write_cmos_sensor(0x5069,0x10);                          
    OV2659_write_cmos_sensor(0x506a,0x04);                          
    OV2659_write_cmos_sensor(0x506b,0x12);                          
                                                                    
     //auto UV                                                      
    OV2659_write_cmos_sensor(0x5001,0x1f);                        
    OV2659_write_cmos_sensor(0x507e,0x40);	                        
    OV2659_write_cmos_sensor(0x507f,0x10);	                        
    OV2659_write_cmos_sensor(0x507b,0x02);                          
    OV2659_write_cmos_sensor(0x507a,0x00);                          
    OV2659_write_cmos_sensor(0x5084,0x1f);	                        
    OV2659_write_cmos_sensor(0x5085,0x3f); 

    OV2659_write_cmos_sensor(0x3406,0x00);//	; Light mode auto
    OV2659_write_cmos_sensor(0x5060,0x69);//	; window 0~3
    OV2659_write_cmos_sensor(0x5061,0x7d);//	; Window 4~7
    OV2659_write_cmos_sensor(0x5062,0x7d);//	; window 8~11
    OV2659_write_cmos_sensor(0x5063,0x69);//	; window 12~15
    OV2659_write_cmos_sensor(0x3a18,0x00);//	; gain ceiling
    OV2659_write_cmos_sensor(0x3a19,0x3e);//	; gain ceiling 4x
    OV2659_write_cmos_sensor(0x3004,0x20);//	; divider

    #else
    if (sizeof(OV2659_init_array[0]) > 0)
    {
        nOV2659_Init_Register_Length = sizeof(OV2659_init_array) / sizeof(OV2659_init_array[0]);
    }
    
    for (i=0; i<nOV2659_Init_Register_Length; i++)
    {
        OV2659_write_cmos_sensor(OV2659_init_array[i].nReg, OV2659_init_array[i].nVal);
    }    
    
    #endif
	return ERROR_NONE;
}	/* OV2659Open() */

/*************************************************************************
* FUNCTION
*	OV2659Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2659Close(void)
{
//	CISModulePowerOn(FALSE);

#if WINMO_USE
	#ifndef SOFTWARE_I2C_INTERFACE	/* software I2c MODE */
	DRV_I2CClose(OV2659hDrvI2C);
	#endif
#endif 	
	return ERROR_NONE;
}	/* OV2659Close() */

/*************************************************************************
* FUNCTION
*	OV2659Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2659Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  	int i;
  	int regPreview[] =
    {
        // SVGA preview
        // Timing
        0x3800, 0x00,
        0x3801, 0x00,
        0x3802, 0x00,
        0x3803, 0x00,
        0x3804, 0x06,
        0x3805, 0x5f,
        0x3806, 0x04,
        0x3807, 0xb7,
        0x3808, 0x03,
        0x3809, 0x20,
        0x380a, 0x02,
        0x380b, 0x58,
        0x380c, 0x05,
        0x380d, 0x14,
        0x380e, 0x02,
        0x380f, 0x68,
        0x3811, 0x08,

        0x3813, 0x02,                              
        0x3814, 0x31,                              
        0x3815, 0x31,                              
        0x3623, 0x00,                              
        0x3634, 0x76,                              
        0x3701, 0x44,                              
        0x3208, 0xa1,                              
        0x3702, 0x18,                            
        0x3703, 0x24,                            
        0x3704, 0x24,                            
        0x3705, 0x0c,                           

        0x3820, 0x87,
        0x3821, 0x07,
        
        0x370a, 0x52, // VFIFO start               
        0x4608, 0x00, // VFIFO start               
        0x4609, 0x80, // Scale enable              
        0x5002, 0x10, // PLL                       
        0x3005, 0x16, // Divider                   
        0x3004, 0x20, // AEC auto, AGC auto        
        0x3503, 0x00,                              
    };        
  	// Write preview table
    for (i=0; i<sizeof(regPreview)/sizeof(int); i+=2)
    {
        OV2659_write_cmos_sensor(regPreview[i], regPreview[i+1]);
    }
    OV2659_set_bandingfilter();
    //OV2659_ChangeNightMode(m_iCombo_Night_mode); // set night mode,
    // m_iCombo_Night_mode is the night mode setting from UI
    return 0;

}	/* OV2659Preview() */

UINT32 OV2659Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    #if 0
    volatile kal_uint32 shutter = OV2659_exposure_lines, temp_reg;
    kal_uint8 temp_AE_reg, temp;
    kal_uint16 AE_setting_delay = 0;

    OV2659_sensor_cap_state = KAL_TRUE;
    OV2659_write_cmos_sensor(0x3800,0x00);//	; HS
    OV2659_write_cmos_sensor(0x3801,0x00);//	; HS
    OV2659_write_cmos_sensor(0x3802,0x00);//	; VS
    OV2659_write_cmos_sensor(0x3803,0x00);//	; VS
    OV2659_write_cmos_sensor(0x3804,0x06);//	; HW = 1631
    OV2659_write_cmos_sensor(0x3805,0x5f);//	; HW
    OV2659_write_cmos_sensor(0x3806,0x04);//	; VH = 1211
    OV2659_write_cmos_sensor(0x3807,0xbb);//	; VH
    OV2659_write_cmos_sensor(0x3808,0x06);//	; DVPHO = 1600
    OV2659_write_cmos_sensor(0x3809,0x40);//	; DVPHO
    OV2659_write_cmos_sensor(0x380a,0x04);//	; DVPVO = 1200
    OV2659_write_cmos_sensor(0x380b,0xb0);//	; DVPVO
    OV2659_write_cmos_sensor(0x380c,0x07);//	; HTS = 1951
    OV2659_write_cmos_sensor(0x380d,0x9f);//	; HTS
    OV2659_write_cmos_sensor(0x380e,0x04);//	; VTS = 1232
    OV2659_write_cmos_sensor(0x380f,0xd0);//	; VTS
    OV2659_write_cmos_sensor(0x3811,0x10);//	; H offset
    OV2659_write_cmos_sensor(0x3813,0x06);//	; V offset
    OV2659_write_cmos_sensor(0x3814,0x11);//	; X inc
    OV2659_write_cmos_sensor(0x3815,0x11);//	; Y inc
    OV2659_write_cmos_sensor(0x3a02,0x04);//	; max exposure 60Hz
    OV2659_write_cmos_sensor(0x3a03,0xd0);//	; max exposure 60Hz
    OV2659_write_cmos_sensor(0x3a08,0x00);//	; B50
    OV2659_write_cmos_sensor(0x3a09,0xb8);//	; B50
    OV2659_write_cmos_sensor(0x3a0a,0x00);//	; B60
    OV2659_write_cmos_sensor(0x3a0b,0x9a);//	; B60
    OV2659_write_cmos_sensor(0x3a0d,0x08);//	; max band 60Hz
    OV2659_write_cmos_sensor(0x3a0e,0x06);//	; max band 50Hz
    OV2659_write_cmos_sensor(0x3a14,0x04);//	; max exposure 50Hz
    OV2659_write_cmos_sensor(0x3a15,0x50);//	; max exposure 50Hz
    OV2659_write_cmos_sensor(0x3623,0x00);//
    OV2659_write_cmos_sensor(0x3634,0x44);//
    OV2659_write_cmos_sensor(0x3701,0x44);//
    OV2659_write_cmos_sensor(0x3702,0x30);//
    OV2659_write_cmos_sensor(0x3703,0x48);//
    OV2659_write_cmos_sensor(0x3704,0x48);//
    OV2659_write_cmos_sensor(0x3705,0x18);//
    OV2659_write_cmos_sensor(0x3820,0x86);//	; Horizontal binning off
    OV2659_write_cmos_sensor(0x3821,0x06);//	; Vertical binning off
    OV2659_write_cmos_sensor(0x370a,0x12);//
    OV2659_write_cmos_sensor(0x4608,0x00);//	; VFIFO start
    OV2659_write_cmos_sensor(0x4609,0x80);//	; VFIFO start
    OV2659_write_cmos_sensor(0x5002,0x00);//	; scale off
    OV2659_write_cmos_sensor(0x3005,0x24);//	; PLL
    OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
    OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL
    OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
    OV2659_write_cmos_sensor(0x3005,0x21);//	; PLL
    OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider
    OV2659_write_cmos_sensor(0x3503,0x00);//	;
    
    temp_reg = OV2659_read_cmos_sensor(0x3014);
    OV2659_write_cmos_sensor(0x3014, temp_reg & 0xf7); //Disable night mode
        
    // turn off AEC/AGC
    //OV2659_set_AE_mode(KAL_FALSE);
    //temp_AE_reg = OV2659_read_cmos_sensor(0x3013);
    //OV2659_write_cmos_sensor(0x3013, (temp_AE_reg&(~0x05)) );        

    //OV2659_set_AWB_mode(KAL_FALSE); 
    OV2659_exposure_line_h = OV2659_read_cmos_sensor(0x3002);
    OV2659_exposure_line_l = OV2659_read_cmos_sensor(0x3003);
    OV2659_extra_exposure_line_h = OV2659_read_cmos_sensor(0x302D);
    OV2659_extra_exposure_line_l = OV2659_read_cmos_sensor(0x302E);

    shutter = OV2659_read_shutter();

    if ((image_window->ImageTargetWidth<=OV2659_IMAGE_SENSOR_PV_WIDTH)&&
        (image_window->ImageTargetHeight<=OV2659_IMAGE_SENSOR_PV_HEIGHT))
    {    /* Less than PV Mode */


        OV2659_gPVmode=KAL_TRUE;

        OV2659_dummy_pixels = 0;
        OV2659_dummy_lines = 0;

        OV2659_capture_pclk_in_M = OV2659_preview_pclk_in_M;   //Don't change the clk

        shutter = (shutter*OV2659_capture_pclk_in_M)/OV2659_preview_pclk_in_M;
        shutter = (shutter*OV2659_PV_PERIOD_PIXEL_NUMS)/(OV2659_PV_PERIOD_PIXEL_NUMS+OV2659_dummy_pixels/2);
        // set dummy
        OV2659_set_dummy(OV2659_dummy_pixels, OV2659_dummy_lines);
        if (shutter < 1) 
        {
            shutter = 1;
        }
        // set shutter OVT
        OV2659_write_shutter(shutter);
        
        image_window->GrabStartX = 1;
        image_window->GrabStartY = 1;
        image_window->ExposureWindowWidth= OV2659_IMAGE_SENSOR_PV_WIDTH - 3;
        image_window->ExposureWindowHeight = OV2659_IMAGE_SENSOR_PV_HEIGHT - 3;

    }
    else 
    {    
    
    	 /* 2M FULL Mode */
        image_window->GrabStartX=1;
        image_window->GrabStartY=6;
        image_window->ExposureWindowWidth=OV2659_IMAGE_SENSOR_FULL_WIDTH - image_window->GrabStartX - 2;
        image_window->ExposureWindowHeight=OV2659_IMAGE_SENSOR_FULL_HEIGHT -image_window->GrabStartY - 2;    	 
        //calculator auto uv

        OV2659_gPVmode = KAL_FALSE;
             
        if ((image_window->ImageTargetWidth<=OV2659_IMAGE_SENSOR_FULL_WIDTH)&&
        (image_window->ImageTargetHeight<=OV2659_IMAGE_SENSOR_FULL_HEIGHT))
        {     
	        if (zoom_factor  <  3) 
	        {  
		     //48Mhz Full size Capture CLK
				OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL
				OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
				OV2659_write_cmos_sensor(0x3005,0x21);//	; PLL
				OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider
	            OV2659_sensor_pclk = 520;

       	     OV2659_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
	            OV2659_dummy_lines=0;

	            OV2659_capture_pclk_in_M = 520;   //Don't change the clk

	            //10 fps 1 frame = 100ms = 30
	            AE_setting_delay = 26; 	        
	        }
	        else 
	        {
		     //48Mhz Full size Capture CLK/2
				OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL
				OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
				OV2659_write_cmos_sensor(0x3005,0x21);//	; PLL
				OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider
	            OV2659_sensor_pclk = 260;
	            
	            OV2659_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
	            OV2659_dummy_lines=0;

	            OV2659_capture_pclk_in_M = 260;   //Don't change the clk

	            //9.3 fps, 1 frame = 200ms
	            AE_setting_delay = 30;

	        }                  
        }
        else//Interpolate to 3M
        {
  	        if (image_window->ZoomFactor >= 340)
	        {  
	        
		     //48Mhz Full size Capture CLK/4
				OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL
				OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
				OV2659_write_cmos_sensor(0x3005,0x21);//	; PLL
				OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider		      
	            OV2659_sensor_pclk = 130;

       	     OV2659_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
	            OV2659_dummy_lines=0;

	            OV2659_capture_pclk_in_M = 130;   //Don't change the clk
	            
	            //9.3 fps, 1 frame = 200ms
	            AE_setting_delay = 34;
	        }
	        else if (image_window->ZoomFactor >= 240) 
	        {  
	        
		     //48Mhz Full size Capture CLK/2
				OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL
				OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
				OV2659_write_cmos_sensor(0x3005,0x21);//	; PLL
				OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider
	            OV2659_sensor_pclk = 260;

       	     OV2659_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
	            OV2659_dummy_lines=0;

	            OV2659_capture_pclk_in_M = 260;   //Don't change the clk
	            
	            //9.3 fps, 1 frame = 200ms
	            AE_setting_delay = 30;
	        }
	        else 
	        {
	        
		     //48Mhz Full size Capture CLK
				OV2659_write_cmos_sensor(0x3003,0x80);//	; PLL
				OV2659_write_cmos_sensor(0x3004,0x20);//	; Divider
				OV2659_write_cmos_sensor(0x3005,0x21);//	; PLL
				OV2659_write_cmos_sensor(0x3006,0x0d);//	; Divider		     
	            OV2659_sensor_pclk = 520;
	            OV2659_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
	            OV2659_dummy_lines=0;

	            OV2659_capture_pclk_in_M = 520;   //Don't change the clk

	            //10 fps 1 frame = 100ms = 30
	            AE_setting_delay = 26; 
	        }                  
        }
   
        OV2659_Capture_Dummy_Pixels = OV2659_dummy_pixels ;
        OV2659_Capture_Dummy_Lines = OV2659_dummy_lines;
        //Jerry It need to change gain to shutter
        OV2659_Computer_AECAGC(OV2659_preview_pclk_in_M, OV2659_capture_pclk_in_M);
        shutter = OV2659_Capture_Shutter + OV2659_Capture_Extra_Lines;

        // set dummy
        OV2659_set_dummy(OV2659_dummy_pixels, OV2659_dummy_lines);
       
        if (shutter < 1) 
        {
            shutter = 1;
        }
        if (OV2659_AE_ENABLE == KAL_TRUE)
        {
            Capture_Shutter = shutter; 
            Capture_Gain = OV2659_Capture_Gain16;
        }
        
        // set shutter OVT
        OV2659_write_shutter(Capture_Shutter);
		/*
        if(OV2659_Capture_Gain16>62)
            OV2659_write_OV2659_gain(16); 
        else
            OV2659_write_OV2659_gain((OV2659_Capture_Gain16+5)); 
        */
        //kal_sleep_task(23);  //delay 1frame
       // Sleep(95);
        OV2659_write_OV2659_gain(Capture_Gain); 
    }

    //printk("Capture Shutter = %d\, Capture Gain = %d\n", Capture_Shutter, Capture_Gain); 
    // AEC/AGC/AWB will be enable in preview and param_wb function
    /* total delay 4 frame for AE stable */

    OV2659_DELAY_AFTER_PREVIEW = 2;

	// copy sensor_config_data
	memcpy(&OV2659SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
	#else
	int regCapture[] =
    {
        // UXGA Capture
        // OV2659_UXGA_YUV7.5 fps
        // 26 MHz input clock, 36Mhz output clock
        // Timing
        0x3800, 0x00,
        0x3801, 0x00,
        0x3802, 0x00,
        0x3803, 0x00,
        0x3804, 0x06,
        0x3805, 0x5f,
        0x3806, 0x04,
        0x3807, 0xbb,
        0x3808, 0x06,
        0x3809, 0x40,
        0x380a, 0x04,
        0x380b, 0xb0,
        0x380c, 0x07,

        0x380d, 0x9f,
        0x380e, 0x04,
        0x380f, 0xd0,
        0x3811, 0x10,
        0x3813, 0x06,
        0x3814, 0x11,
        0x3815, 0x11,
        
        0x3623, 0x00,
        0x3634, 0x44,
        0x3701, 0x44,
        0x3208, 0xa2,

        0x3705, 0x18,
        
        0x3820, 0x86,
        0x3821, 0x06,
        
        0x370a, 0x12, // VFIFO start
        0x4608, 0x00, // VFIFO start
        0x4609, 0x80, // scale off
        0x5002, 0x00, // PLL
        0x3005, 0x21, // Divider
        0x3004, 0x20, 
        0x3503, 0x03, // AEC manual, AGC manual
    };

    int i,preview_shutter, preview_gain16, preview_binning;
    int capture_shutter, capture_gain16, capture_sysclk, capture_HTS, capture_VTS;
    int light_frequency, capture_bandingfilter, capture_max_band;
    long capture_gain16_shutter;
    // read preview shutter
    preview_shutter = OV2659_get_shutter();
    // read preview gain
    preview_gain16 = OV2659_get_gain16();
    OV2659_PV_Gain16 = preview_gain16;
    OV2659_PV_Shutter = preview_shutter;

    // get preview binning factor
    preview_binning = OV2659_get_binning_factor();
    // turn off night mode for capture
    //OV2659_ChangeNightMode(0);
    // Write preview table
    for (i=0; i<sizeof(regCapture)/sizeof(int); i+=2)
    {
        OV2659_write_cmos_sensor(regCapture[i], regCapture[i+1]);
    }
    // read capture PCLK
    capture_sysclk = OV2659_get_sysclk();
    // read capture HTS
    capture_HTS = OV2659_get_HTS();
    // read capture VTS
    capture_VTS = OV2659_get_VTS();
    // read capture banding filter
    light_frequency = OV2659_get_light_frequency();
    if (light_frequency == 60) {
    // 60Hz
        capture_bandingfilter = capture_sysclk * 100/capture_HTS * 100/120;
    }
    else {
        // 50Hz
        capture_bandingfilter = capture_sysclk * 100/capture_HTS;
    }
    capture_max_band = (int)((capture_VTS - 4)/capture_bandingfilter);
    // calculate capture shutter
    capture_shutter = preview_shutter;
    
    // gain to shutter
    capture_gain16 = preview_gain16 * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS * preview_binning;
    if (capture_gain16 < 16) {
        capture_gain16 = 16;
    }
    capture_gain16_shutter = capture_gain16 * capture_shutter;
    
    if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
        // shutter < 1/100
        capture_shutter = capture_gain16_shutter/16;
        capture_gain16 = capture_gain16_shutter/capture_shutter;
    }
    else {
        if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
            // exposure reach max
            capture_shutter = capture_bandingfilter*capture_max_band;
            capture_gain16 = capture_gain16_shutter / capture_shutter;
        }
        else {
            // 1/100 < capture_shutter < max, capture_shutter = n/100
            capture_shutter = (int)((capture_gain16_shutter/16/capture_bandingfilter)) * capture_bandingfilter;
            capture_gain16 = capture_gain16_shutter / capture_shutter;
        }
    }
    // write capture gain
    OV2659_set_gain16(capture_gain16);
    // write capture shutter
    if (capture_shutter > (capture_VTS - 4)) {
        capture_VTS = capture_shutter + 4;
        OV2659_set_VTS(capture_VTS);
    }
    OV2659_set_shutter(capture_shutter);
    // skip 2 vysnc
    // start capture at 3rd vsync
    return 0;

	#endif
}	/* OV2659Capture() */

UINT32 OV2659GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorPreviewWidth=OV2659_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorPreviewHeight=OV2659_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	
	pSensorResolution->SensorFullWidth=OV2659_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;  //modify by yanxu
	pSensorResolution->SensorFullHeight=OV2659_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	return ERROR_NONE;
}	/* OV2659GetResolution() */

UINT32 OV2659GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=OV2659_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=OV2659_IMAGE_SENSOR_PV_HEIGHT;	

	pSensorInfo->SensorFullResolutionX=OV2659_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=OV2659_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=TRUE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=TRUE;
	pSensorInfo->CaptureDelayFrame = 3; 
	pSensorInfo->PreviewDelayFrame = 2; 
	pSensorInfo->VideoDelayFrame = 4; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
       pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;   		

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
                     pSensorInfo->SensorGrabStartX = 4; 
                     pSensorInfo->SensorGrabStartY = 2;             
			
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
                     pSensorInfo->SensorGrabStartX = 4; 
                     pSensorInfo->SensorGrabStartY = 2;             
			
		break;
		default:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
                     pSensorInfo->SensorGrabStartX = 4; 
                     pSensorInfo->SensorGrabStartY = 2;             
			
		break;
	}
	OV2659_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	memcpy(pSensorConfigData, &OV2659SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* OV2659GetInfo() */

static kal_bool OV2659_sensor_burstcap_flag = KAL_FALSE; //Preview or Capture

UINT32 OV2659Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			OV2659Preview(pImageWindow, pSensorConfigData);
			OV2659_sensor_burstcap_flag = KAL_FALSE;
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			if(!OV2659_sensor_burstcap_flag)
			{
				OV2659Capture(pImageWindow, pSensorConfigData);
				OV2659_sensor_burstcap_flag = KAL_TRUE;
			}
		break;
		default:
		    break; 
	}
	return TRUE;
}	/* OV2659Control() */

/* [TC] YUV sensor */	
#if WINMO_USE
void OV2659Query(PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo)
{
	MSDK_FEATURE_TYPE_RANGE_STRUCT *pFeatureRange;
	MSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT *pFeatureMultiSelection;
	switch (pSensorFeatureInfo->FeatureId)
	{
		case ISP_FEATURE_DSC_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX;
			pFeatureMultiSelection->DefaultSelection = CAM_AUTO_DSC_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_AUTO_DSC_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_NIGHTSCENE_MODE));			
		break;
		case ISP_FEATURE_WHITEBALANCE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_WB;
			pFeatureMultiSelection->DefaultSelection = CAM_WB_AUTO;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_WB_AUTO)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_CLOUD)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_DAYLIGHT)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_INCANDESCENCE)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_TUNGSTEN)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_FLUORESCENT));
		break;
		case ISP_FEATURE_IMAGE_EFFECT:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_EFFECT_ENC;
			pFeatureMultiSelection->DefaultSelection = CAM_EFFECT_ENC_NORMAL;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_NORMAL)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYSCALE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_COLORINV)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIAGREEN)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIABLUE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIA));	
		break;
		case ISP_FEATURE_AE_METERING_MODE:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_BRIGHTNESS:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_RANGE;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureRange = (PMSDK_FEATURE_TYPE_RANGE_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureRange);
			pFeatureRange->MinValue = CAM_EV_NEG_4_3;
			pFeatureRange->MaxValue = CAM_EV_POS_4_3;
			pFeatureRange->StepValue = CAMERA_FEATURE_ID_EV_STEP;
			pFeatureRange->DefaultValue = CAM_EV_ZERO;
		break;
		case ISP_FEATURE_BANDING_FREQ:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_BANDING;
			pFeatureMultiSelection->DefaultSelection = CAM_BANDING_50HZ;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_BANDING_50HZ)|
				CAMERA_FEATURE_SUPPORT(CAM_BANDING_60HZ));
		break;
		case ISP_FEATURE_AF_OPERATION:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_AF_RANGE_CONTROL:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_FLASH:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		case ISP_FEATURE_VIDEO_SCENE_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX;
			pFeatureMultiSelection->DefaultSelection = CAM_VIDEO_AUTO_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_VIDEO_AUTO_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_VIDEO_NIGHT_MODE));
		break;
		case ISP_FEATURE_ISO:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		default:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
	}
}
#endif 
/*************************************************************************
* FUNCTION
*   OV2659GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2659GetSensorID(UINT32 *sensorID) 
{
	OV2659_write_cmos_sensor(0x0103,0x01);// Reset sensor
    Sleep(10);

	//  Read sensor ID to adjust I2C is OK?
//	for(i=0;i<3;i++)
	{
		*sensorID = (OV2659_read_cmos_sensor(0x300A) << 8) | OV2659_read_cmos_sensor(0x300B);
	    SENSORDB("OV2659Open, sensor_id:%x \n",*sensorID);
		if(*sensorID != OV2659_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
   return ERROR_NONE;
}

BOOL OV2659_set_param_wb(UINT16 para)
{
    kal_uint8  temp_reg;


    switch (para)
    {
        /*case AWB_MODE_OFF:
            OV2659_AWB_ENABLE = KAL_FALSE; 
            OV2659_set_AWB_mode(OV2659_AWB_ENABLE);
            break; */                    
        case AWB_MODE_AUTO:
            OV2659_AWB_ENABLE = KAL_TRUE;  //
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Auto
			OV2659_write_cmos_sensor(0x3406,0x00);//	; AWB auto
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
			break;

        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Cloudy
			OV2659_write_cmos_sensor(0x3406,0x01);//
			OV2659_write_cmos_sensor(0x3400,0x06);// 
			OV2659_write_cmos_sensor(0x3401,0x28);// 
			OV2659_write_cmos_sensor(0x3402,0x04);// 
			OV2659_write_cmos_sensor(0x3403,0x00);// 
			OV2659_write_cmos_sensor(0x3404,0x04);// 
			OV2659_write_cmos_sensor(0x3405,0x60);// 
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AWB_MODE_DAYLIGHT: //sunny
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Sunny
			OV2659_write_cmos_sensor(0x3406,0x01);//	; awb manual
			OV2659_write_cmos_sensor(0x3400,0x06);// 	; R gain
			OV2659_write_cmos_sensor(0x3401,0x02);//	; R gain
			OV2659_write_cmos_sensor(0x3402,0x04);//	; G gain
			OV2659_write_cmos_sensor(0x3403,0x00);//	; G gain
			OV2659_write_cmos_sensor(0x3404,0x04);//	; B gain
			OV2659_write_cmos_sensor(0x3405,0x65);//	; B gain
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
			break;

        case AWB_MODE_INCANDESCENT: //office
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Office
			OV2659_write_cmos_sensor(0x3406,0x01);//
			OV2659_write_cmos_sensor(0x3400,0x05);// 
			OV2659_write_cmos_sensor(0x3401,0x10);// 
			OV2659_write_cmos_sensor(0x3402,0x04);// 
			OV2659_write_cmos_sensor(0x3403,0x00);// 
			OV2659_write_cmos_sensor(0x3404,0x05);// 
			OV2659_write_cmos_sensor(0x3405,0x94);// 
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AWB_MODE_TUNGSTEN: //home
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Home
			OV2659_write_cmos_sensor(0x3406,0x01);//
			OV2659_write_cmos_sensor(0x3400,0x04);// 
			OV2659_write_cmos_sensor(0x3401,0x48);// 
			OV2659_write_cmos_sensor(0x3402,0x04);// 
			OV2659_write_cmos_sensor(0x3403,0x00);// 
			OV2659_write_cmos_sensor(0x3404,0x06);// 
			OV2659_write_cmos_sensor(0x3405,0x20);// 
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AWB_MODE_FLUORESCENT:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Office
			OV2659_write_cmos_sensor(0x3406,0x01);//
			OV2659_write_cmos_sensor(0x3400,0x06);// 
			OV2659_write_cmos_sensor(0x3401,0x2a);// 
			OV2659_write_cmos_sensor(0x3402,0x04);// 
			OV2659_write_cmos_sensor(0x3403,0x00);// 
			OV2659_write_cmos_sensor(0x3404,0x07);// 
			OV2659_write_cmos_sensor(0x3405,0x24);// 
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;
#if WINMO_USE
        case AWB_MODE_MANUAL:
            // TODO
            break;
#endif 

        default:
            return FALSE;
    }

    return TRUE;
} /* OV2659_set_param_wb */

BOOL OV2659_set_param_effect(UINT16 para)
{
   BOOL  ret = TRUE;
   //UINT8  temp_reg;
   //temp_reg=OV2659_read_cmos_sensor(0x3391);
    switch (para)
    {
        case MEFFECT_OFF:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Normal
			OV2659_write_cmos_sensor(0x507b,0x06);//	; Y contrast enable, saturation enable
			OV2659_write_cmos_sensor(0x507e,0x40);//	; Saturation U, max saturation
			OV2659_write_cmos_sensor(0x507f,0x20);// 	; Saturation V, min saturation
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
			break;

        case MEFFECT_SEPIA:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Sepia(antique)
			OV2659_write_cmos_sensor(0x507b,0x18);// 	; Fix U, Fix V
			OV2659_write_cmos_sensor(0x507e,0x40);// 	; U
			OV2659_write_cmos_sensor(0x507f,0xa0);// 	; V
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
			break;
        case MEFFECT_NEGATIVE:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Negative
			OV2659_write_cmos_sensor(0x507b,0x40);// 	; negative enable
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x507e,0x40);//
			OV2659_write_cmos_sensor(0x507f,0x20);//
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
			break;

        case MEFFECT_SEPIAGREEN:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Greenish
			OV2659_write_cmos_sensor(0x507b,0x18);// 
			OV2659_write_cmos_sensor(0x507e,0x60);// 
			OV2659_write_cmos_sensor(0x507f,0x60);// 
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case MEFFECT_SEPIABLUE:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Reddish
			OV2659_write_cmos_sensor(0x507b,0x18);// 
			OV2659_write_cmos_sensor(0x507e,0xa0);// 
			OV2659_write_cmos_sensor(0x507f,0x40);// 
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;
		case MEFFECT_MONO: //B&W
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ B&W
			OV2659_write_cmos_sensor(0x507b,0x20);// 	; BW enable
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x507e,0x40);//
			OV2659_write_cmos_sensor(0x507f,0x20);//
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
			break;
#if WINMO_USE
        case CAM_EFFECT_ENC_GRAYINV:
        case CAM_EFFECT_ENC_COPPERCARVING:
              case CAM_EFFECT_ENC_BLUECARVING:


        case CAM_EFFECT_ENC_EMBOSSMENT:
        case CAM_EFFECT_ENC_SKETCH:
        case CAM_EFFECT_ENC_BLACKBOARD:
        case CAM_EFFECT_ENC_WHITEBOARD:
        case CAM_EFFECT_ENC_JEAN:
        case CAM_EFFECT_ENC_OIL:
#endif 
        default:
            ret = FALSE;
    }

    return ret;

} /* OV2659_set_param_effect */

BOOL OV2659_set_param_banding(UINT16 para)
{

    kal_uint8 banding;

    banding = OV2659_read_cmos_sensor(0x3A05);
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
			
           OV2659_Banding_setting = AE_FLICKER_MODE_50HZ;
		   OV2659_write_cmos_sensor(0x3a05,0x30);

            break;

        case AE_FLICKER_MODE_60HZ:			
            OV2659_preview_pclk_in_M = 390;
            OV2659_Banding_setting = AE_FLICKER_MODE_60HZ;			
			OV2659_write_cmos_sensor(0x3a05,0xB0);

          default:
              return FALSE;
    }

    return TRUE;
} /* OV2659_set_param_banding */

BOOL OV2659_set_param_exposure(UINT16 para)
{
    kal_uint8  temp_reg;

    temp_reg=OV2659_read_cmos_sensor(0x3391);

    switch (para)
    {
        case AE_EV_COMP_n13:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness -3
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x40);//
			OV2659_write_cmos_sensor(0x507b,0x06);// 
			OV2659_write_cmos_sensor(0x5083,0x08);// 08
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0 
			break;

        case AE_EV_COMP_n10:    
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness -2
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x30);//
			OV2659_write_cmos_sensor(0x507b,0x06);// 
			OV2659_write_cmos_sensor(0x5083,0x08);// 08
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AE_EV_COMP_n07:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness -2
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x20);//
			OV2659_write_cmos_sensor(0x507b,0x06);// 
			OV2659_write_cmos_sensor(0x5083,0x08);// 08
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AE_EV_COMP_n03:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness -1
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x10);//
			OV2659_write_cmos_sensor(0x507b,0x06);// 
			OV2659_write_cmos_sensor(0x5083,0x08);// 08
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
			break;

        case AE_EV_COMP_00:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness 0
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x00);//
			OV2659_write_cmos_sensor(0x507b,0x06);// 
			OV2659_write_cmos_sensor(0x5083,0x00);// 08
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AE_EV_COMP_03:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness +1
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x10);//
			OV2659_write_cmos_sensor(0x507b,0x06);// 
			OV2659_write_cmos_sensor(0x5083,0x00);// 08
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AE_EV_COMP_07:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness +2
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x20);//
			OV2659_write_cmos_sensor(0x507b,0x06);// 
			OV2659_write_cmos_sensor(0x5083,0x00);// 08
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AE_EV_COMP_10:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness +3
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x30);//
			OV2659_write_cmos_sensor(0x507b,0x06);// ;bit[2] enable
			OV2659_write_cmos_sensor(0x5083,0x00);// 08 ;bit[3],sign
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        case AE_EV_COMP_13:
			OV2659_write_cmos_sensor(0x3208,0x00);//	; enable group 0@@ Brightness +3
			OV2659_write_cmos_sensor(0x5001,0x1f);// 
			OV2659_write_cmos_sensor(0x5082,0x40);//
			OV2659_write_cmos_sensor(0x507b,0x06);// ;bit[2] enable
			OV2659_write_cmos_sensor(0x5083,0x00);// 08 ;bit[3],sign
			OV2659_write_cmos_sensor(0x3208,0x10);//	; end group 0
			OV2659_write_cmos_sensor(0x3208,0xa0);//	; launch group 0
            break;

        default:
            return FALSE;
    }

    return TRUE;
} /* OV2659_set_param_exposure */



UINT32 OV2659YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
//   if( OV2659_sensor_cap_state == KAL_TRUE)
//	   return TRUE;

	switch (iCmd) {
	case FID_SCENE_MODE:	    
	    printk("Set Scene Mode:%d\n", iPara); 
	    if (iPara == SCENE_MODE_OFF)
	    {
	        OV2659_night_mode(0); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
               OV2659_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
	    printk("Set AWB Mode:%d\n", iPara); 	    
           OV2659_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
	    printk("Set Color Effect:%d\n", iPara); 	    	    
           OV2659_set_param_effect(iPara);
	break;
	case FID_AE_EV:
#if WINMO_USE	    
	case ISP_FEATURE_EXPOSURE:
#endif 	    
           printk("Set EV:%d\n", iPara); 	    	    
           OV2659_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
           printk("Set Flicker:%d\n", iPara); 	    	    	    
           OV2659_set_param_banding(iPara);
	break;
        case FID_AE_SCENE_MODE: 
            if (iPara == AE_MODE_OFF) {
                OV2659_AE_ENABLE = KAL_FALSE; 
            }
            else {
                OV2659_AE_ENABLE = KAL_TRUE; 
	    }
            break; 
	case FID_ZOOM_FACTOR:
	    zoom_factor = iPara; 
        break; 
	default:
	break;
	}
	return TRUE;
}   /* OV2659YUVSensorSetting */

UINT32 OV2659YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    /* to fix VSYNC, to fix frame rate */
    //printk("Set YUV Video Mode \n");  
    iTemp = OV2659_read_cmos_sensor(0x3014);
    OV2659_write_cmos_sensor(0x3014, iTemp & 0xf7); //Disable night mode

    if (u2FrameRate == 30)
    {
        OV2659_write_cmos_sensor(0x302d, 0x00);
        OV2659_write_cmos_sensor(0x302e, 0x00);
    }
    else if (u2FrameRate == 15)       
    {
        OV2659_write_cmos_sensor(0x300e, 0x34);
        OV2659_write_cmos_sensor(0x302A, OV2659_VIDEO_15FPS_FRAME_LENGTH>>8);  /*  15fps*/
        OV2659_write_cmos_sensor(0x302B, OV2659_VIDEO_15FPS_FRAME_LENGTH&0xFF);
                
        // clear extra exposure line
        OV2659_write_cmos_sensor(0x302d, 0x00);
        OV2659_write_cmos_sensor(0x302e, 0x00);   
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }
    OV2659_VEDIO_encode_mode = KAL_TRUE; 
        
    return TRUE;
}

void OV2659GetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{
	*pFeatureReturnPara32 = 0;
	SENSORDB(" OV2659GetAFMaxNumFocusAreas, *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

void OV2659GetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{
	*pFeatureReturnPara32 = 0;
	SENSORDB(" OV2659GetAFMaxNumMeteringAreas, *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}
	
UINT32 OV2659FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];

#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=OV2659_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=OV2659_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=OV2659_PV_PERIOD_PIXEL_NUMS+OV2659_PV_dummy_pixels;
			*pFeatureReturnPara16=OV2659_PV_PERIOD_LINE_NUMS+OV2659_PV_dummy_lines;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = OV2659_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			OV2659_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			OV2659_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
		       printk("SENSOR_FEATURE_SET_REGISTER \n");
			OV2659_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
		       printk("SENSOR_FEATURE_GET_REGISTER \n");
			//pSensorRegData->RegData = OV2659_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
		       printk("SENSOR_FEATURE_GET_CONFIG_PARA \n");
			//memcpy(pSensorConfigData, &OV2659SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			//*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
		       printk("SENSOR_FEATURE_GET_GROUP_COUNT %d \n");
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		       printk("SENSOR_FEATURE_GET_LENS_DRIVER_ID %d \n");
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		       printk("OV2659 YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			OV2659YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			//OV2659YUVSensorSetting((FEATURE_ID)(Tony_Temp1), Tony_Temp2);//Tony_Modify
		break;
#if WINMO_USE		    		
		case SENSOR_FEATURE_QUERY:
			OV2659Query(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
		break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
		break;		
#endif 			
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       OV2659YUVSetVideoMode(*pFeatureData16);
		       break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			//*pFeatureData32=OV2655_SENSOR_ID;
          OV2659GetSensorID(pFeatureReturnPara32); 
			break; 
		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			OV2659GetAFMaxNumFocusAreas(pFeatureReturnPara32);
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			OV2659GetAFMaxNumMeteringAreas(pFeatureReturnPara32);
			*pFeatureParaLen=4;
			break;
		default:
			break;			
	}
	return ERROR_NONE;
}	/* OV2659FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncOV2659=
{
	OV2659Open,
	OV2659GetInfo,
	OV2659GetResolution,
	OV2659FeatureControl,
	OV2659Control,
	OV2659Close
};

UINT32 OV2659_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV2659;

	return ERROR_NONE;
}	/* SensorInit() */


