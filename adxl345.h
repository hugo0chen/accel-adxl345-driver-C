#ifndef __adx1345_h
#define __adx1345_h
#include "stm32l1xx.h"

// if communication select I2C, the device id is setting by alt/sdo pin
// alt = 0, device address = 0x53  
// alt = 1, device address = 0x1d

struct ADXL345_Define{

	GPIO_TypeDef* pin_of_alt_group;
	uint16_t pin_of_alt;
	uint32_t pin_of_alt_clk;
};	

// register mapping
#define DEVICE_ID		0X00 	//器件ID,0XE5
#define THRESH_TAP		0X1D   	//敲击阀值
#define OFSX			0X1E
#define OFSY			0X1F
#define OFSZ			0X20
#define DUR				0X21
#define LATENT			0X22
#define WINDOW  		0X23 
#define THRESH_ACK		0X24
#define THRESH_INACT	0X25 
#define TIME_INACT		0X26
#define ACT_INACT_CTL	0X27	 
#define THRESH_FF		0X28	
#define TIME_FF			0X29 
#define TAP_AXES		0X2A  
#define ACT_TAP_STATUS  0X2B 
#define BW_RATE			0X2C 
#define POWER_CTL		0X2D 

#define INT_ENABLE		0X2E
#define INT_MAP			0X2F
#define INT_SOURCE  	0X30
#define DATA_FORMAT	    0X31
#define DATA_X0			0X32
#define DATA_X1			0X33
#define DATA_Y0			0X34
#define DATA_Y1			0X35
#define DATA_Z0			0X36
#define DATA_Z1			0X37
#define FIFO_CTL		0X38
#define FIFO_STATUS		0X39

// ----registrer bit definitioin ------//
/* ADXL345_ACT_INACT_CTL definition */
#define ADXL345_ACT_ACDC        (1 << 7)
#define ADXL345_ACT_X_EN        (1 << 6)
#define ADXL345_ACT_Y_EN        (1 << 5)
#define ADXL345_ACT_Z_EN        (1 << 4)
#define ADXL345_INACT_ACDC      (1 << 3)
#define ADXL345_INACT_X_EN      (1 << 2)
#define ADXL345_INACT_Y_EN      (1 << 1)
#define ADXL345_INACT_Z_EN      (1 << 0)

/* ADXL345_TAP_AXES definition */
#define ADXL345_SUPPRESS        (1 << 3)
#define ADXL345_TAP_X_EN        (1 << 2)
#define ADXL345_TAP_Y_EN        (1 << 1)
#define ADXL345_TAP_Z_EN        (1 << 0)

/* ADXL345_ACT_TAP_STATUS definition */
#define ADXL345_ACT_X_SRC       (1 << 6)
#define ADXL345_ACT_Y_SRC       (1 << 5)
#define ADXL345_ACT_Z_SRC       (1 << 4)
#define ADXL345_ASLEEP          (1 << 3)
#define ADXL345_TAP_X_SRC       (1 << 2)
#define ADXL345_TAP_Y_SRC       (1 << 1)
#define ADXL345_TAP_Z_SRC       (1 << 0)

/* ADXL345_BW_RATE definition */
#define ADXL345_LOW_POWER       (1 << 4)
#define ADXL345_RATE(x)         ((x) & 0xF)

/* ADXL345_POWER_CTL definition */
#define ADXL345_PCTL_LINK       (1 << 5)
#define ADXL345_PCTL_AUTO_SLEEP (1 << 4)
#define ADXL345_PCTL_MEASURE    (1 << 3)
#define ADXL345_PCTL_SLEEP      (1 << 2)
#define ADXL345_PCTL_WAKEUP(x)  ((x) & 0x3)

/* ADXL345_INT_ENABLE / ADXL345_INT_MAP / ADXL345_INT_SOURCE definition */
#define ADXL345_DATA_READY      (1 << 7)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_OVERRUN         (1 << 0)

/* ADXL345_DATA_FORMAT definition */
#define ADXL345_SELF_TEST       (1 << 7)
#define ADXL345_SPI             (1 << 6)
#define ADXL345_INT_INVERT      (1 << 5)
#define ADXL345_FULL_RES        (1 << 3)
#define ADXL345_JUSTIFY         (1 << 2)
#define ADXL345_RANGE(x)        ((x) & 0x3)

/* ADXL345_RANGE(x) options */
#define ADXL345_RANGE_PM_2G     0
#define ADXL345_RANGE_PM_4G     1
#define ADXL345_RANGE_PM_8G     2
#define ADXL345_RANGE_PM_16G    3

/* ADXL345_FIFO_CTL definition */
#define ADXL345_FIFO_MODE(x)    (((x) & 0x3) << 6)
#define ADXL345_TRIGGER         (1 << 5)
#define ADXL345_SAMPLES(x)      ((x) & 0x1F)

/* ADXL345_FIFO_MODE(x) options */
#define ADXL345_FIFO_BYPASS     0
#define ADXL345_FIFO_FIFO       1
#define ADXL345_FIFO_STREAM     2
#define ADXL345_FIFO_TRIGGER    3

/* ADXL345_FIFO_STATUS definition */
#define ADXL345_FIFO_TRIG       (1 << 7)
#define ADXL345_ENTRIES(x)      ((x) & 0x3F)

/* ADXL345 Full Resolution Scale Factor */
#define ADXL345_SCALE_FACTOR    4
/*ADXL345 Power Save Mode */
#define MEASURE_MODE 0x01
#define STANBY_MODE 0x00
// ----  I2C address definition ----//
//如果ALT ADDRESS脚(12脚)接地,IIC地址为0X53(不包含最低位).
//如果接V3.3,则IIC地址为0X1D(不包含最低位).
//因为开发板接GND,所以转为读写地址后,为0XA7和0XA6(如果接VCC=3.3V,则为0X3B和0X3A)  

#define ADXL_READ    0XA7
#define ADXL_WRITE   0XA6

// ---- 				function					------//
void ADXL345_WR_Reg(uint8_t regaddr,uint8_t val);				    //写ADXL345寄存器
uint8_t ADXL345_RD_Reg(uint8_t regaddr);							      //读ADXL345寄存器
void ADXL345_RD_XYZ(short *x,short *y,short *z);	          //读取一次值
void ADXL345_RD_Avval(short *x,short *y,short *z);			  	//读取平均值
void ADXL345_AUTO_Adjust(char*xval,char*yval,char*zval);		//自动校准
void ADXL345_Read_Average(short *x,short *y,short *z,uint8_t times);//连续读取times次,取平均
short ADXL345_Get_Angle(float x,float y,float z,uint8_t dir);
void Adxl_Show_Num(uint16_t x,uint16_t y,short num,uint8_t mode);

void accel_init(void);
void get_accel_val(short *accelxyz,short *angxyz,uint8_t times);
uint8_t get_adjust_flag(void);
void ADXL345_GetGxyz(short* x,short* y,short* z,uint8_t times);
char get_intsource(void);

#endif
