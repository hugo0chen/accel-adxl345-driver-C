
//  PB10    ->   IIC2_SCL  
//  PB11    ->   IIC2_SDA
//  PB14    ->   ALT ADDRESS/ SDO
//  INT1    ->   PB13  no-use
//  INT2    ->   PC0  no-use

#include "adxl345.h"
#include "math.h" 
#include "i2c.h"
#include "OLED.h"
#include "delay.h"

#define ZOFFSET 1345
uint8_t adjust_ok_flag = 0;    // self adjust ok flag
char selectedRange     = 0;    // scale factor  +/-2g ,4g,8g,16g
char fullResolutionSet = 0;    // resolution select 10bit/13bit/
	
struct ADXL345_Define ADXL345_def={GPIOB,GPIO_Pin_14,RCC_AHBPeriph_GPIOB};	

static void ADXL345_Module_Alt_GPIO_Init(struct ADXL345_Define defines)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
/* Enable GPIOs clock */ 	
	RCC_AHBPeriphClockCmd(defines.pin_of_alt_clk, ENABLE);	 //使能Port端口时钟
	
 /* Configure the ALT ADDRESS Pin PB14*/
	GPIO_InitStructure.GPIO_Pin = defines.pin_of_alt;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(defines.pin_of_alt_group, &GPIO_InitStructure);
	
	GPIO_ResetBits(defines.pin_of_alt_group,defines.pin_of_alt);  //ALT =0 
}
static void ADXL345_INT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIOs clock */ 	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource13);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
static void ADXL345_Module_GPIO_Init(void)
{
	ADXL345_Module_Alt_GPIO_Init(ADXL345_def);
//	ADXL345_INT_Init();
	I2Cx_Init();							//初始化IIC总线	
}

/*************************************************************
  Function   : ADXL345_WR_Reg
  Description: 写ADXL345寄存器		
  Input      : addr:寄存器地址
							 val:要写入的值	        
  return     : none    
*************************************************************/
void ADXL345_WR_Reg(uint8_t regaddr,uint8_t val) 
{
	I2C_Start();  				 
	I2C_WriteByte(ADXL_WRITE);     	//发送写器件指令	 
	I2C_WaiteForAck();	   
  I2C_WriteByte(regaddr);   			//发送寄存器地址
	I2C_WaiteForAck(); 	 										  		   
	I2C_WriteByte(val);     		//发送值					   
	I2C_WaiteForAck();  		    	   
  I2C_Stop();						//产生一个停止条件 	   
}
/***************************************************************************//**
 * @brief read adxl345 register value
 *
 * @param regaddr --- register address
 *
 * @return the value in register
*******************************************************************************/
uint8_t ADXL345_RD_Reg(uint8_t regaddr) 		
{
	uint8_t temp=0;
	
	I2C_Start();  				 
	I2C_WriteByte(ADXL_WRITE);				//发送写器件指令	 
	temp = I2C_WaiteForAck();
if(!temp) return 0;	
  I2C_WriteByte(regaddr);   					//发送寄存器地址
	temp = I2C_WaiteForAck();
if(!temp) return 0;		
	I2C_Start();  	 	   								//重新启动
	I2C_WriteByte(ADXL_READ);						//发送读器件指令	 
	temp = I2C_WaiteForAck();
if(!temp) return 0;		
	temp = I2C_ReadByte(I2C_NACK);		//读取一个字节,不继续再读,发送NACK 		
  I2C_Stop();												//产生一个停止条件 	    
	return temp;											//返回读到的值
}  
/***************************************************************************//**
 * @brief read value of the x,y,z fifo
 *
 * @param  x,y,z :x,y,z axis value
 *
 * @return None
*******************************************************************************/
void ADXL345_RD_XYZ(short *x,short *y,short *z)
{
	uint8_t buf[6];
	uint8_t i;
	I2C_Start();  				 
	I2C_WriteByte(ADXL_WRITE);						//发送写器件指令	 
	I2C_WaiteForAck();	   
   I2C_WriteByte(0x32);   							//发送寄存器地址(数据缓存的起始地址为0X32)
	I2C_WaiteForAck(); 	 										  		   
 
 	I2C_Start();  	 	   									//重新启动
	I2C_WriteByte(ADXL_READ);							//发送读器件指令
	I2C_WaiteForAck();
	for(i=0;i<6;i++)
	{
		if(i==5)buf[i]=I2C_ReadByte(I2C_NACK);//读取一个字节,不继续再读,发送NACK  
		else buf[i]=I2C_ReadByte(I2C_ACK);	//读取一个字节,继续读,发送ACK 
 	}	        	   
  I2C_Stop();													  //产生一个停止条件
	*x=(short)(((uint16_t)buf[1]<<8)+buf[0]); 	    
	*y=(short)(((uint16_t)buf[3]<<8)+buf[2]); 	    
	*z=(short)(((uint16_t)buf[5]<<8)+buf[4]); 	 
}
/***************************************************************************//**
 * @brief read adxl345 value 10 times ,and caculate the average 
 *
 * @param x,y,z:读取10次后取平均值 
 *
 * @return None.
*******************************************************************************/
void ADXL345_RD_Avval(short *x,short *y,short *z)
{
	short tx=0,ty=0,tz=0;	   
	uint8_t i;  
	for(i=0;i<10;i++)
	{
		ADXL345_RD_XYZ(x,y,z);
		delay_ms(10);
		tx+=(short)*x;
		ty+=(short)*y;
		tz+=(short)*z;	   
	}
	*x=tx/10;
	*y=ty/10;
	*z=tz/10;
}
/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *			Example: 0x0 - standby mode.
 *				 0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetPowerMode(unsigned char pwrMode)
{
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;
    
    oldPowerCtl = ADXL345_RD_Reg(POWER_CTL);
    newPowerCtl = oldPowerCtl & ~ADXL345_PCTL_MEASURE;
    newPowerCtl = newPowerCtl | (pwrMode * ADXL345_PCTL_MEASURE);
    ADXL345_WR_Reg(POWER_CTL, newPowerCtl);
}

/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param gRange  - Range option.
 *                  Example: ADXL345_RANGE_PM_2G  - +-2 g
 *                           ADXL345_RANGE_PM_4G  - +-4 g
 *                           ADXL345_RANGE_PM_8G  - +-8 g
 *                           ADXL345_RANGE_PM_16G - +-16 g
 * @param fullRes - Full resolution option.
 *                   Example: 0x0 - Disables full resolution.
 *                            ADXL345_FULL_RES - Enables full resolution.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetRangeResolution(unsigned char gRange, unsigned char fullRes)
{
    unsigned char oldDataFormat = 0;
    unsigned char newDataFormat = 0;
    
    oldDataFormat = ADXL345_RD_Reg(DATA_FORMAT);
    newDataFormat = oldDataFormat & ~(ADXL345_RANGE(0x3) | ADXL345_FULL_RES);
    newDataFormat =  newDataFormat | ADXL345_RANGE(gRange) | fullRes;
    ADXL345_WR_Reg(DATA_FORMAT, newDataFormat);
    selectedRange = (1 << (gRange + 1));
    fullResolutionSet = fullRes ? 1 : 0;
}
/***************************************************************************//**
 * @brief Enables/disables the free-fall detection.
 *
 * @param ffOnOff  - Enables/disables the free-fall detection.
 *			Example: 0x0 - disables the free-fall detection.
 *				 0x1 - enables the free-fall detection.
 * @param ffThresh - Threshold value for free-fall detection. The scale factor 
                     is 62.5 mg/LSB.
 * @param ffTime   - Time value for free-fall detection. The scale factor is 
                     5 ms/LSB.
 * @param ffInt    - Interrupts pin.
 *		        Example: 0x0 - free-fall interrupts on INT1 pin.
 *			         ADXL345_FREE_FALL - free-fall interrupts on 
 *                                                   INT2 pin.   
 * @return None.
*******************************************************************************/
void ADXL345_SetFreeFallDetection(unsigned char ffOnOff,
                                  unsigned char ffThresh,
                                  unsigned char ffTime,
                                  unsigned char ffInt)
{
    unsigned char oldIntMap    = 0;
    unsigned char newIntMap    = 0;
    unsigned char oldIntEnable = 0;
    unsigned char newIntEnable = 0;
    
    ADXL345_WR_Reg(THRESH_FF, ffThresh);
    ADXL345_WR_Reg(TIME_FF, ffTime);
    oldIntMap = ADXL345_RD_Reg(INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_FREE_FALL);
    newIntMap = newIntMap | ffInt;
    ADXL345_WR_Reg(INT_MAP, newIntMap);
    oldIntEnable = ADXL345_RD_Reg(INT_ENABLE);
    newIntEnable = oldIntEnable & ~ADXL345_FREE_FALL;
    newIntEnable = newIntEnable | (ADXL345_FREE_FALL * ffOnOff);
    ADXL345_WR_Reg(INT_ENABLE, newIntEnable);	
}
/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param tapType   - Tap type (none, single, double).
 *			Example: 0x0 - disables tap detection.	
 *				ADXL345_SINGLE_TAP - enables single tap 
 *                                                   detection.
 *				ADXL345_DOUBLE_TAP - enables double tap 
 *                                                   detection.
 * @param tapAxes   - Axes which participate in tap detection.
 *			Example: 0x0 - disables axes participation.
 *				ADXL345_TAP_X_EN - enables x-axis participation.
 *				ADXL345_TAP_Y_EN - enables y-axis participation.
 *				ADXL345_TAP_Z_EN - enables z-axis participation.
 * @param tapDur    - Tap duration. The scale factor is 625us is/LSB.
 * @param tapLatent - Tap latency. The scale factor is 1.25 ms/LSB.
 * @param tapWindow - Tap window. The scale factor is 1.25 ms/LSB.
 * @param tapThresh - Tap threshold. The scale factor is 62.5 mg/LSB.
 * @param tapInt    - Interrupts pin.
 *			Example: 0x0 - interrupts on INT1 pin.
 *				ADXL345_SINGLE_TAP - single tap interrupts on 
 *						     INT2 pin.
 *				ADXL345_DOUBLE_TAP - double tap interrupts on
 *						     INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetTapDetection(unsigned char tapType,
                             unsigned char tapAxes,
                             unsigned char tapDur,
                             unsigned char tapLatent,
                             unsigned char tapWindow,
                             unsigned char tapThresh,
                             unsigned char tapInt)
{
    unsigned char oldTapAxes   = 0;
    unsigned char newTapAxes   = 0;
    unsigned char oldIntMap    = 0;
    unsigned char newIntMap    = 0;
    unsigned char oldIntEnable = 0;
    unsigned char newIntEnable = 0;
    
    oldTapAxes = ADXL345_RD_Reg(TAP_AXES);
    newTapAxes = oldTapAxes & ~(ADXL345_TAP_X_EN |
                                ADXL345_TAP_Y_EN |
                                ADXL345_TAP_Z_EN);
    newTapAxes = newTapAxes | tapAxes;
    ADXL345_WR_Reg(TAP_AXES, newTapAxes);
    ADXL345_WR_Reg(DUR, tapDur);
    ADXL345_WR_Reg(LATENT, tapLatent);
    ADXL345_WR_Reg(WINDOW, tapWindow);
    ADXL345_WR_Reg(THRESH_TAP, tapThresh);
    oldIntMap = ADXL345_RD_Reg(INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntMap = newIntMap | tapInt;
    ADXL345_WR_Reg(INT_MAP, newIntMap);
    oldIntEnable = ADXL345_RD_Reg(INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntEnable = newIntEnable | tapType;
    ADXL345_WR_Reg(INT_ENABLE, newIntEnable);
}
/***************************************************************************//**
 * @brief Enables/disables the activity detection.
 *
 * @param actOnOff  - Enables/disables the activity detection.
 *				Example: 0x0 - disables the activity detection.
 *					 0x1 - enables the activity detection.
 * @param actAxes   - Axes which participate in detecting activity.
 *			Example: 0x0 - disables axes participation.
 *				ADXL345_ACT_X_EN - enables x-axis participation.
 *				ADXL345_ACT_Y_EN - enables y-axis participation.
 *				ADXL345_ACT_Z_EN - enables z-axis participation.
 * @param actAcDc   - Selects dc-coupled or ac-coupled operation.
 *			Example: 0x0 - dc-coupled operation.
 *				ADXL345_ACT_ACDC - ac-coupled operation.
 * @param actThresh - Threshold value for detecting activity. The scale factor 
                      is 62.5 mg/LSB.
 * @patam actInt    - Interrupts pin.
 *			Example: 0x0 - activity interrupts on INT1 pin.
 *				ADXL345_ACTIVITY - activity interrupts on INT2 
 *                                                 pin.
 * @return None.
*******************************************************************************/
void ADXL345_SetActivityDetection(unsigned char actOnOff,
                                  unsigned char actAxes,
                                  unsigned char actAcDc,
                                  unsigned char actThresh,
                                  unsigned char actInt)
{
    unsigned char oldActInactCtl = 0;
    unsigned char newActInactCtl = 0;
    unsigned char oldIntMap      = 0;
    unsigned char newIntMap      = 0;
    unsigned char oldIntEnable   = 0;
    unsigned char newIntEnable   = 0;
    
    oldActInactCtl = ADXL345_RD_Reg(INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_ACT_ACDC |
                                        ADXL345_ACT_X_EN |
                                        ADXL345_ACT_Y_EN |
                                        ADXL345_ACT_Z_EN);
    newActInactCtl = newActInactCtl | (actAcDc | actAxes);
    ADXL345_WR_Reg(ACT_INACT_CTL, newActInactCtl);
    ADXL345_WR_Reg(THRESH_ACK, actThresh);
    oldIntMap = ADXL345_RD_Reg(INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_ACTIVITY);
    newIntMap = newIntMap | actInt;
    ADXL345_WR_Reg(INT_MAP, newIntMap);
    oldIntEnable = ADXL345_RD_Reg(INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_ACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_ACTIVITY * actOnOff);
    ADXL345_WR_Reg(INT_ENABLE, newIntEnable);
}
/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param inactOnOff  - Enables/disables the inactivity detection.
 *			  Example: 0x0 - disables the inactivity detection.
 *				   0x1 - enables the inactivity detection.
 * @param inactAxes   - Axes which participate in detecting inactivity.
 *			  Example: 0x0 - disables axes participation.
 *				ADXL345_INACT_X_EN - enables x-axis.
 *				ADXL345_INACT_Y_EN - enables y-axis.
 *				ADXL345_INACT_Z_EN - enables z-axis.
 * @param inactAcDc   - Selects dc-coupled or ac-coupled operation.
 *			  Example: 0x0 - dc-coupled operation.
 *				ADXL345_INACT_ACDC - ac-coupled operation.
 * @param inactThresh - Threshold value for detecting inactivity. The scale 
                        factor is 62.5 mg/LSB.
 * @param inactTime   - Inactivity time. The scale factor is 1 sec/LSB.
 * @patam inactInt    - Interrupts pin.
 *		          Example: 0x0 - inactivity interrupts on INT1 pin.
 *				ADXL345_INACTIVITY - inactivity interrupts on
 *						     INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetInactivityDetection(unsigned char inactOnOff,
                                    unsigned char inactAxes,
                                    unsigned char inactAcDc,
                                    unsigned char inactThresh,
                                    unsigned char inactTime,
                                    unsigned char inactInt)
{
    unsigned char oldActInactCtl = 0;
    unsigned char newActInactCtl = 0;
    unsigned char oldIntMap      = 0;
    unsigned char newIntMap      = 0;
    unsigned char oldIntEnable   = 0;
    unsigned char newIntEnable   = 0;
    
    oldActInactCtl = ADXL345_RD_Reg(INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_INACT_ACDC |
                                        ADXL345_INACT_X_EN |
                                        ADXL345_INACT_Y_EN |
                                        ADXL345_INACT_Z_EN);
    newActInactCtl = newActInactCtl | (inactAcDc | inactAxes);
    ADXL345_WR_Reg(ACT_INACT_CTL, newActInactCtl);
    ADXL345_WR_Reg(THRESH_INACT, inactThresh);
    ADXL345_WR_Reg(TIME_INACT, inactTime);
    oldIntMap = ADXL345_RD_Reg(INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_INACTIVITY);
    newIntMap = newIntMap | inactInt;
    ADXL345_WR_Reg(INT_MAP, newIntMap);
    oldIntEnable = ADXL345_RD_Reg(INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_INACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_INACTIVITY * inactOnOff);
    ADXL345_WR_Reg(INT_ENABLE, newIntEnable);
}
/***************************************************************************//**
 * @brief Sets an offset value for each axis (Offset Calibration).
 *
 * @param xOffset - X-axis's offset.
 * @param yOffset - Y-axis's offset.
 * @param zOffset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetOffset(unsigned char xOffset,
                       unsigned char yOffset,
                       unsigned char zOffset)
{
    ADXL345_WR_Reg(OFSX, xOffset);
    ADXL345_WR_Reg(OFSY, yOffset);
    ADXL345_WR_Reg(OFSZ, yOffset);
}
/***************************************************************************//**
 * @brief get trig soucr for register INT_SOURCE.
 *
 * @param  none
 *
 * @return source value
 *        @example ADXL345_FREE_FALL
 *				         ADXL345_SINGLE_TAP					
 *								 ADXL345_DOUBLE_TAP
*******************************************************************************/
char get_intsource(void)
{
	char intSource;
	
	intSource = ADXL345_RD_Reg(INT_SOURCE);
	return intSource;
}

/***************************************************************************//**
 * @brief  auto adjust the x,y,z fifo value
 *
 * @param  xval,yval,zval:x,y,z轴的校准值
 *
 * @return None
*******************************************************************************/
void ADXL345_AUTO_Adjust(char*xval,char*yval,char*zval)
{
	short tx,ty,tz;
	uint8_t i;
	short offx=0,offy=0,offz=0;

	
	ADXL345_WR_Reg(POWER_CTL,0x00);	   	//先进入休眠模式.
	delay_ms(100);
	//ADXL345_SetRangeResolution(ADXL345_RANGE_PM_16G, ADXL345_FULL_RES);
	ADXL345_WR_Reg(DATA_FORMAT,0X2B);		//force seft-test,4-wire spi,低电平中断输出,13位全分辨率4mg/lSB,输出数据右对齐,+/-16g量程 
	ADXL345_WR_Reg(BW_RATE,0x0A);				//normal(power) op-mode ,数据输出速度为100 0x0a bandwidth= 50HZ
	ADXL345_WR_Reg(POWER_CTL,0x28);	   	//链接使能,测量模式
	//ADXL345_SetPowerMode(MEASURE_MODE);    // measure mode
	
	ADXL345_SetOffset(0,0,0);
	delay_ms(12);
	for(i=0;i<10;i++)
	{
		ADXL345_RD_Avval(&tx,&ty,&tz);
		offx+=tx;
		offy+=ty;
		offz+=tz;
	}
	offx/=10;
	offy/=10;
	offz/=10;
	
	*xval=-offx/4;
	*yval=-offy/4;
	*zval=-(offz-256)/4;	 
	
	ADXL345_SetOffset(*xval,*yval,*zval);	
	
	adjust_ok_flag = 1;
}

uint8_t get_adjust_flag(void)
{
	return adjust_ok_flag;
}

/***************************************************************************//**
 * @brief init adxl345 
 *						read device id  and init parameters
 * @param  none
 *
 * @return  0  -- init successful
 *   				1  -- init failed			
*******************************************************************************/
static uint8_t ADXL345_Init(void)
{				 
	uint8_t t;

	t = ADXL345_RD_Reg(DEVICE_ID);
	if(t==0XE5)	//读取器件ID
	{  		
		printf("device ID:%x \n",t);
		
		selectedRange = 2; // Measurement Range: +/- 2g (reset default).
    fullResolutionSet = 0;
		
		ADXL345_SetFreeFallDetection(0x01,  // Free-fall detection enabled.
                                 0x05,  // Free-fall threshold.
                                 0x14,  // Time value for free-fall detection.
                                 0x00); // Interrupt Pin.
		ADXL345_SetTapDetection(ADXL345_SINGLE_TAP |
                            ADXL345_DOUBLE_TAP, // Tap type.
                            ADXL345_TAP_X_EN,   // Axis control.
                            0x10,               // Tap duration.
                            0x10,               // Tap latency.
                            0x40,               // Tap window. 
                            0x10,               // Tap threshold.
                            0x00);              // Interrupt Pin.
		 /* Set the range and the resolution. */
    ADXL345_SetRangeResolution(ADXL345_RANGE_PM_16G, ADXL345_FULL_RES);
		//ADXL345_SetRangeResolution(ADXL345_RANGE_PM_16G, 0);
  	ADXL345_SetPowerMode(MEASURE_MODE);          // Measure mode.

		return 0;
	}			
	return 1;	   								 
}  
/***************************************************************************//**
 * @brief accel init gpio ,device 
 *						
 * @param  none
 *
 * @return  none
*******************************************************************************/
void accel_init(void)
{
	short x,y,z;
	
	ADXL345_Module_GPIO_Init();	
	
	delay_ms(100);
	while(ADXL345_Init()) {};
	delay_ms(100);
	
	//ADXL345_AUTO_Adjust();
	printf("accel init ok\n");
}
/***************************************************************************//**
 * @brief 读取ADXL345的数据times次,再取平均
 *						
 * @param  x,y,z:读到的数据
 *
 * @return  none
*******************************************************************************/
void ADXL345_Read_Average(short *x,short *y,short *z,uint8_t times)
{
	uint8_t i;
	short tx,ty,tz;
	*x=0;
	*y=0;
	*z=0;
	if(times)						//读取次数不为0
	{
		for(i=0;i<times;i++)//连续读取times次
		{
			ADXL345_RD_XYZ(&tx,&ty,&tz);
			*x+=tx;
			*y+=ty;
			*z+=tz;
			delay_ms(10);
		}
		*x/=times;
		*y/=times;
		*z/=times;
	}
}
/***************************************************************************//**
 * @brief 得到角度
 *						
 * @param  x,y,z:x,y,z方向的重力加速度分量(不需要单位,直接数值即可)
 *				 dir:要获得的角度.0,与Z轴的角度;1,与X轴的角度;2,与Y轴的角度.
 *
 * @return  角度值.单位0.1°.
*******************************************************************************/
short ADXL345_Get_Angle(float x,float y,float z,uint8_t dir)
{
	float temp;
 	float res=0;
	switch(dir)
	{
		case 0://与自然Z轴的角度
 			temp=sqrt((x*x+y*y))/z;
 			res=atan(temp);
 			break;
		case 1://与自然X轴的角度
 			temp=x/sqrt((y*y+z*z));
 			res=atan(temp);
 			break;
 		case 2://与自然Y轴的角度
 			temp=y/sqrt((x*x+z*z));
 			res=atan(temp);
 			break;
 	}
	return res*1800/3.14;
}

/***************************************************************************//**
 * @brief x,y:开始显示的坐标位置
 *						
 * @param   x -- OLED display x-axis position
 *					y -- OLED display y-axis position
 *					num:要显示的数据
 *					mode 
 *					example  0 -- accel value display mode
 *                   1 -- angle value display mode
 *
 * @return  角度值.单位0.1°.
*******************************************************************************/
void Adxl_Show_Num(uint16_t x,uint16_t y,short num,uint8_t mode)
{
	if(mode==0)	//显示加速度值
	{
		if(num<0)
		{			
			printf("-");//显示负号
			OLED_ShowString(x,y,"-");
			num=-num;						//转为正数
		}else 
		{
			printf(" "); //去掉负号		
			OLED_ShowString(x,y,"  ");
		}
		printf("%d mg\n\r",num);	
		OLED_ShowNum(x+16,y,num,4,16);
					
 	}
	else 		//显示角度值
	{
		if(num<0)
		{				
			printf("-");//显示负号
			OLED_ShowString(x,y,"-");
			num=-num;						//转为正数
		}
		else
		{
			printf(" ");	//去掉负号	
			OLED_ShowString(x,y,"  ");
		}
		OLED_ShowNum(x+16,y,num,4,16);		
		printf("%d.%d °\n\r",num/10,num%10); 		   
	}
}		
/***************************************************************************//**
 * @brief  get accel & angle value ,the average value
 *						
 * @param   accelxyz ---  accel value of  x-axis y-axis z-axis
 *					anglexyz ---  angle value of  x-axis y-axis z-axis
 *					times  --- get value times ,the average factor
 *
 * @return  角度值.单位0.1°.
*******************************************************************************/
void get_accel_val(short *accelxyz,short *angxyz,uint8_t times)
{
		short *p1 = accelxyz;  	    
		short *p2 = angxyz; 
	  short x,y,z;
		
		ADXL345_Read_Average(&x,&y,&z,times);	//读取X,Y,Z三个方向的加速度值 
	  *p1 = x;	p1++;
	  *p1 = y;	p1++;
	  *p1 = z;
 		//得到角度值,
		*p2 = ADXL345_Get_Angle(x,y,z,1); //x
		p2++;	
		*p2 = ADXL345_Get_Angle(x,y,z,2); //Y  
		p2++;
		*p2 = ADXL345_Get_Angle(x,y,z,0); //Z
}
/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to g.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void ADXL345_GetGxyz(short* x,short* y,short* z,uint8_t times)
{
    short xData = 0;  // X-axis's output data.
    short yData = 0;  // Y-axis's output data.
    short zData = 0;  // Z-axis's output data.
 
    ADXL345_Read_Average(&xData, &yData, &zData,times);
	
    *x = fullResolutionSet ? (xData * ADXL345_SCALE_FACTOR) :
            (xData * ADXL345_SCALE_FACTOR * (selectedRange >> 1));
    *y = fullResolutionSet ? (yData * ADXL345_SCALE_FACTOR) :
            (yData * ADXL345_SCALE_FACTOR * (selectedRange >> 1));
    *z = fullResolutionSet ? (zData * ADXL345_SCALE_FACTOR) :
            (zData * ADXL345_SCALE_FACTOR * (selectedRange >> 1));
	//	*z = *z - ZOFFSET;
}

