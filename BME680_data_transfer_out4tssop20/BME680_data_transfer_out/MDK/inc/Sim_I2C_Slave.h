///////////////////////////////////////////////////////////////////////////////
// \author (c) 微信公众号：固件工人
//             2022, Shanghai, China
//
// \license The MIT License (MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// \brief
//	sw_slave_i2c.h
//
// \version
// v0.0.1: 2022.05.01, Initial version.
///////////////////////////////////////////////////////////////////////////////

#ifndef __SIM_I2C_SLAVE_H__
#define __SIM_I2C_SLAVE_H__

#ifdef __cplusplus
extern "C" {
#endif

//#include "stm32f4xx_hal.h"
#include "CMS32L051.h"
#include "gpio.h"



#define SW_SLAVE_ADDR			0xEC

//#define SW_SLAVE_SCL_CLK_EN()	__HAL_RCC_GPIOB_CLK_ENABLE()
// #define SW_SLAVE_SDA_CLK_EN()	__HAL_RCC_GPIOB_CLK_ENABLE()

// #define SW_SLAVE_SCL_PRT		GPIOB
// #define SW_SLAVE_SCL_PIN		GPIO_PIN_6
// #define SW_SLAVE_SDA_PRT		GPIOB
// #define SW_SLAVE_SDA_PIN		GPIO_PIN_7

#define GPIO_MODE_MSK           0x00000003U

#define I2C_STA_IDLE			0
#define I2C_STA_START			1
#define I2C_STA_DATA			2
#define I2C_STA_ACK				3
#define I2C_STA_NACK			4
#define I2C_STA_STOP			5

#define I2C_READ				1
#define I2C_WRITE				0

#define GPIO_DIR_IN				0
#define GPIO_DIR_OUT			1

// #define SET_SCL_DIR(Temp, InOut)					\
// 	Temp = SW_SLAVE_SCL_PRT->MODER;					\
// 	Temp &= ~(GPIO_MODER_MODER6);					\
// 	Temp |= ((InOut & GPIO_MODE_MSK) << (6 * 2U));	\
// 	SW_SLAVE_SCL_PRT->MODER = temp;
#define SET_SCL_DIR( InOut )		if(InOut)								\
                                    PORT -> PM1 |= (1<<4);                  \
								    else									\
                                    PORT ->PM1 &= ~(1<<4);
								

// #define SET_SDA_DIR(Temp, InOut)					
// 	Temp = SW_SLAVE_SDA_PRT->MODER;					
// 	Temp &= ~(GPIO_MODER_MODER7);					
// 	Temp |= ((InOut & GPIO_MODE_MSK) << (7 * 2U));	
// 	SW_SLAVE_SDA_PRT->MODER = Temp;
#define SET_SDA_DIR_OUT()		    PORT-> PM1 &= ~(1<<3)
										
#define SET_SDA_DIR_IN()            PORT->PM1 |= (1<<3)
																        
									


// #define CLR_SDA_PIN()			(SW_SLAVE_SDA_PRT->BSRR = SW_SLAVE_SDA_PIN << 16)
// #define SET_SDA_PIN()			(SW_SLAVE_SDA_PRT->BSRR = SW_SLAVE_SDA_PIN)
#define CLR_SDA_PIN()				do{PORT -> P1 &= ~(1<<3);}while(0)
#define SET_SDA_PIN()				do{PORT -> P1 |= (1<<3);}while(0)

typedef struct _SwSlaveI2C_t
{
	uint8_t State;					// I2C通信状态
	uint8_t Rw;						// I2C读写标志：0-写，1-读
	uint8_t SclFallCnt;				// SCL下降沿计数
    uint8_t SclRaiseCnt;            // SCL上升沿计数
	uint8_t Flag;					// I2C状态标志，BIT0：0-地址无效，1-地址匹配
	uint32_t StartMs;				// I2C通信起始时间，单位ms，用于判断通信是否超时
	uint8_t* RxBuf;					// 指向接收缓冲区的指针
	uint8_t* TxBuf;		            // 指向发送缓冲区的指针
	uint8_t RxIdx;					// 接收缓冲区数据写入索引，最大值255
	uint8_t TxIdx;					// 发送缓冲区数据读取索引，最大值255
}SwSlaveI2C_t;

extern SwSlaveI2C_t SwSlaveI2C;

void InitSwSlaveI2C(void);
void I2C_intp1(void);
void I2C_intp2(void);
void CheckSwSlaveI2cTimeout(void);

#ifdef __cplusplus
}
#endif

#endif /* __SIM_I2C_SLAVE_H__ */
