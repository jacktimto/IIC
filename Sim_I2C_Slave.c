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
//	software simulation I2C slave device
//
// \version
// v0.0.1: 2022.05.01, Initial version.
///////////////////////////////////////////////////////////////////////////////

#include "Sim_I2C_Slave.h"
//#include "delay.h"
#include "userdefine.h"
#include "CMS32L051.h"
#include "intp.h"
#include "gpio.h"
#include <stdio.h>

void IRQ02_Handler(void) __attribute__((alias("intp_1_interrupt")));
void IRQ03_Handler(void) __attribute__((alias("intp_2_interrupt")));
extern volatile uint8_t bme680_data[11];
uint8_t RxDataBuf[200];
uint8_t TxDataBuf[200];

SwSlaveI2C_t SwSlaveI2C =
{
	I2C_STA_IDLE,		// State
	I2C_WRITE,			// Rw
	0, 					// SclFallCnt
	0,					// Flag
	0,					// StartMs
	RxDataBuf,			// RxBuf
	TxDataBuf,			// TxBuf
	0,					// RxIdx
	0					// TxIdx
};

/**
  *外部中断1
  *
  */
void intp_1_interrupt(void)
{
	I2cGpioIsr();
    INTC_ClearPendingIRQ(INTP1_IRQn);
}

/**
  *外部中断2
  *
  */
void intp_2_interrupt(void)
{
	I2cGpioIsr();
    INTC_ClearPendingIRQ(INTP2_IRQn);
}



/**
  * @brief  Configures slave I2C GPIO pin in interrupt mode
  * @param  None
  * @retval None
  */
void InitSwSlaveI2C(void)
{
	INTP_Init(1<<1, INTP_BOTH);//初始化INTP1
	INTP_Init(1<<2, INTP_BOTH);//初始化INTP2
	INTP_Start(1<<1);
	INTP_Start(1<<2);

}

/**
  * @brief EXTI line interruption service
  * @param None
  * @retval None
  */
void I2cGpioIsr(void)
{
	// 处理SCL的上下沿中断
	if(INTC_GetPendingIRQ(INTP2_IRQn) != 0)
	{
		//uint8_t fall_cnt = 0;
		//__HAL_GPIO_EXTI_CLEAR_IT(SW_SLAVE_SCL_PIN);
		INTC_ClearPendingIRQ(INTP2_IRQn);
		// 更新通信起始时间
		//SwSlaveI2C.StartMs = HAL_GetTick();
		// SCL的下降沿事件处理，此时需要更新要传输的数据
		if(PORT_GetBit(PORT1, PIN4) == 0)
		{
			switch(SwSlaveI2C.State)
			{
				case I2C_STA_START:		                    // 起始信号的下降沿，初始化相关参数并转到接收比特数据状态
					SwSlaveI2C.SclFallCnt = 0;
					SwSlaveI2C.RxIdx = 0;
					SwSlaveI2C.TxIdx = 0;
					SwSlaveI2C.Flag = 0;	                // 默认地址不匹配
					SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
					SwSlaveI2C.Rw = I2C_WRITE;	            // 第1字节为设备地址，一定是写入
					SwSlaveI2C.State = I2C_STA_DATA;
					break;
				case I2C_STA_DATA:
					SwSlaveI2C.SclFallCnt++;

					if(8 > SwSlaveI2C.SclFallCnt)
					{
						// 如果是主机读取数据，则在SCL低电平时更新比特数据
						if(SwSlaveI2C.Rw == I2C_READ)
						{
							
//							switch (RxDataBuf[1])
//                            {
//;
//                                case 0x00:
//                                    TxDataBuf[0] = bme680_data[0];
//                                    break;
//                                case 0x01:
//                                    TxDataBuf[0] = bme680_data[1];
//                                    break;
//                                case 0x02:
//                                    TxDataBuf[0] = bme680_data[8];
//									break;
//                                case 0x03:
//                                    TxDataBuf[0] = bme680_data[9];
//                                    break;
//                                case 0x04:
//                                    TxDataBuf[0] = bme680_data[4];
//                                    break;
//                                
//                            }


									
								
							if(SwSlaveI2C.TxBuf[SwSlaveI2C.TxIdx] & (1 << (7 - SwSlaveI2C.SclFallCnt)))
							{
								SET_SDA_PIN();
							}
							else
							{
								CLR_SDA_PIN();
							}
						}
					}
					else if(8 == SwSlaveI2C.SclFallCnt)
					{
						if(SwSlaveI2C.Rw == I2C_WRITE)
						{
							// 从第一个地址字节中获取读写标志位，并判断地址是否匹配
							if(SwSlaveI2C.RxIdx == 0)
							{
								if((SwSlaveI2C.RxBuf[0] & 0xFE) == SW_SLAVE_ADDR)
								{
									SwSlaveI2C.Flag = 1;	// 地址匹配
									SwSlaveI2C.Rw = SwSlaveI2C.RxBuf[0] & 0x01;//判断后一个数据是读还是写
								}
							}
							if(SwSlaveI2C.Flag)
							{
								// 如果是主机写入数据，且地址匹配，则接收完8比特数据后，需要发送ACK信号进行应答
                              // SwSlaveI2C.Flag = 0;//jason
								SET_SDA_DIR_OUT();
								CLR_SDA_PIN();//输出低
								//SET_SDA_PIN();
							}
						}
						else
						{
							// 如果是主机读取数据，需要将SDA设置成输入以便判断应答标志位状态
							SET_SDA_DIR_IN();
							// 如果是主机读取数据，准备发送下一个字节的数据
							SwSlaveI2C.TxIdx++;
						}
						
						// 接收或发送完8比特数据后，准备发送或接收应答信号
						SwSlaveI2C.State = I2C_STA_ACK;
						//SwSlaveI2C.State = I2C_STA_NACK
					}
					break;
				case I2C_STA_ACK:
					SwSlaveI2C.SclFallCnt = 0;
					if(SwSlaveI2C.Rw == I2C_WRITE)
					{
						// 如果是主机写入数据，且ACK发送完毕，则SDA设置成输入，继续接收数据
						SET_SDA_DIR_IN();
						SwSlaveI2C.RxIdx++;
						SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
					}
					else
					{
						// 如果是主机读取数据，且ACK接收完毕，则SDA设置成输出，继续发送数据
						SET_SDA_DIR_OUT();
						if(SwSlaveI2C.TxBuf[SwSlaveI2C.TxIdx] & 0x80)//输出第一个数据
						{
							SET_SDA_PIN();
						}
						else
						{
							CLR_SDA_PIN();
						}
					}
					SwSlaveI2C.State = I2C_STA_DATA;
					break;
				case I2C_STA_NACK:		// 如果收到了NACK，则后面将是STOP或者ReSTART信号，需要将SDA设置成输入
					SwSlaveI2C.SclFallCnt = 0;
					SET_SDA_DIR_IN();
					break;
			}
		}
		// SCL的上升沿事件处理，此时需要采集数据，而且在数据阶段，SCL高电平时数据必须保持不变,上升沿
		else
		{
			switch(SwSlaveI2C.State)
			{
				case I2C_STA_DATA:	// 数据阶段，如果是主机写入数据，则采集比特数据
					if((I2C_WRITE == SwSlaveI2C.Rw) && (8 > SwSlaveI2C.SclFallCnt))
					{
						if(PORT_GetBit(PORT1, PIN3)&0x08)
						{
							SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] |= (1 << (7 - SwSlaveI2C.SclFallCnt));
						}
					}
					break;
				case I2C_STA_ACK:	// 应答阶段，如果是主机读取数据，则判断ACK/NACK信号，默认状态是ACK
					if((SwSlaveI2C.Rw == I2C_READ) && (PORT_GetBit(PORT1, PIN3)&0x04))
					{
						SwSlaveI2C.State = I2C_STA_NACK;
					}
					break;
			}
		}
	}
	else if(INTC_GetPendingIRQ(INTP1_IRQn)  != 0)
	{
		//__HAL_GPIO_EXTI_CLEAR_IT(SW_SLAVE_SDA_PIN);
		INTC_ClearPendingIRQ(INTP1_IRQn);
		if(PORT_GetBit(PORT1, PIN3) == 0)
		{
			// SCL为高电平时，SDA从高变低，说明是起始信号
			if(PORT_GetBit(PORT1, PIN4)>>4)
			{
				SwSlaveI2C.State = I2C_STA_START;
				//INTP_Stop(1<<1);
			}
		}
		else
		{
			// SCL为高电平时，SDA从低变高，说明是停止信号，一次I2C通信结束，直接将状态设置成空闲
			if(PORT_GetBit(PORT1, PIN4)>>4)
			{
				SwSlaveI2C.State = I2C_STA_IDLE;
				//INTP_Start(1<<1);
			}
		}
	}
}

// void CheckSwSlaveI2cTimeout(void)
// {
// 	uint32_t TimeMs, TimeCurMs;

// 	if(SwSlaveI2C.State != I2C_STA_IDLE)
// 	{
// 		TimeCurMs = HAL_GetTick();
// 		if(TimeCurMs >= SwSlaveI2C.StartMs)
// 		{
// 			TimeMs = TimeCurMs - SwSlaveI2C.StartMs;
// 		}
// 		else
// 		{
// 			TimeMs = ~(SwSlaveI2C.StartMs - TimeCurMs) + 1;
// 		}
// 		if(500 <= TimeMs)
// 		{
// 			// I2C通信超时的话，重置状态机，并把SDA设置成输入
// 			SwSlaveI2C.State = I2C_STA_IDLE;
// 			SET_SDA_DIR(TimeMs, GPIO_DIR_IN);
// 		}
// 	}
// }
