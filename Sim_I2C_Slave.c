///////////////////////////////////////////////////////////////////////////////
// \author (c) ΢�Ź��ںţ��̼�����
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
  *�ⲿ�ж�1
  *
  */
void intp_1_interrupt(void)
{
	I2cGpioIsr();
    INTC_ClearPendingIRQ(INTP1_IRQn);
}

/**
  *�ⲿ�ж�2
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
	INTP_Init(1<<1, INTP_BOTH);//��ʼ��INTP1
	INTP_Init(1<<2, INTP_BOTH);//��ʼ��INTP2
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
	// ����SCL���������ж�
	if(INTC_GetPendingIRQ(INTP2_IRQn) != 0)
	{
		//uint8_t fall_cnt = 0;
		//__HAL_GPIO_EXTI_CLEAR_IT(SW_SLAVE_SCL_PIN);
		INTC_ClearPendingIRQ(INTP2_IRQn);
		// ����ͨ����ʼʱ��
		//SwSlaveI2C.StartMs = HAL_GetTick();
		// SCL���½����¼�������ʱ��Ҫ����Ҫ���������
		if(PORT_GetBit(PORT1, PIN4) == 0)
		{
			switch(SwSlaveI2C.State)
			{
				case I2C_STA_START:		                    // ��ʼ�źŵ��½��أ���ʼ����ز�����ת�����ձ�������״̬
					SwSlaveI2C.SclFallCnt = 0;
					SwSlaveI2C.RxIdx = 0;
					SwSlaveI2C.TxIdx = 0;
					SwSlaveI2C.Flag = 0;	                // Ĭ�ϵ�ַ��ƥ��
					SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
					SwSlaveI2C.Rw = I2C_WRITE;	            // ��1�ֽ�Ϊ�豸��ַ��һ����д��
					SwSlaveI2C.State = I2C_STA_DATA;
					break;
				case I2C_STA_DATA:
					SwSlaveI2C.SclFallCnt++;

					if(8 > SwSlaveI2C.SclFallCnt)
					{
						// �����������ȡ���ݣ�����SCL�͵�ƽʱ���±�������
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
							// �ӵ�һ����ַ�ֽ��л�ȡ��д��־λ�����жϵ�ַ�Ƿ�ƥ��
							if(SwSlaveI2C.RxIdx == 0)
							{
								if((SwSlaveI2C.RxBuf[0] & 0xFE) == SW_SLAVE_ADDR)
								{
									SwSlaveI2C.Flag = 1;	// ��ַƥ��
									SwSlaveI2C.Rw = SwSlaveI2C.RxBuf[0] & 0x01;//�жϺ�һ�������Ƕ�����д
								}
							}
							if(SwSlaveI2C.Flag)
							{
								// ���������д�����ݣ��ҵ�ַƥ�䣬�������8�������ݺ���Ҫ����ACK�źŽ���Ӧ��
                              // SwSlaveI2C.Flag = 0;//jason
								SET_SDA_DIR_OUT();
								CLR_SDA_PIN();//�����
								//SET_SDA_PIN();
							}
						}
						else
						{
							// �����������ȡ���ݣ���Ҫ��SDA���ó������Ա��ж�Ӧ���־λ״̬
							SET_SDA_DIR_IN();
							// �����������ȡ���ݣ�׼��������һ���ֽڵ�����
							SwSlaveI2C.TxIdx++;
						}
						
						// ���ջ�����8�������ݺ�׼�����ͻ����Ӧ���ź�
						SwSlaveI2C.State = I2C_STA_ACK;
						//SwSlaveI2C.State = I2C_STA_NACK
					}
					break;
				case I2C_STA_ACK:
					SwSlaveI2C.SclFallCnt = 0;
					if(SwSlaveI2C.Rw == I2C_WRITE)
					{
						// ���������д�����ݣ���ACK������ϣ���SDA���ó����룬������������
						SET_SDA_DIR_IN();
						SwSlaveI2C.RxIdx++;
						SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
					}
					else
					{
						// �����������ȡ���ݣ���ACK������ϣ���SDA���ó������������������
						SET_SDA_DIR_OUT();
						if(SwSlaveI2C.TxBuf[SwSlaveI2C.TxIdx] & 0x80)//�����һ������
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
				case I2C_STA_NACK:		// ����յ���NACK������潫��STOP����ReSTART�źţ���Ҫ��SDA���ó�����
					SwSlaveI2C.SclFallCnt = 0;
					SET_SDA_DIR_IN();
					break;
			}
		}
		// SCL���������¼�������ʱ��Ҫ�ɼ����ݣ����������ݽ׶Σ�SCL�ߵ�ƽʱ���ݱ��뱣�ֲ���,������
		else
		{
			switch(SwSlaveI2C.State)
			{
				case I2C_STA_DATA:	// ���ݽ׶Σ����������д�����ݣ���ɼ���������
					if((I2C_WRITE == SwSlaveI2C.Rw) && (8 > SwSlaveI2C.SclFallCnt))
					{
						if(PORT_GetBit(PORT1, PIN3)&0x08)
						{
							SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] |= (1 << (7 - SwSlaveI2C.SclFallCnt));
						}
					}
					break;
				case I2C_STA_ACK:	// Ӧ��׶Σ������������ȡ���ݣ����ж�ACK/NACK�źţ�Ĭ��״̬��ACK
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
			// SCLΪ�ߵ�ƽʱ��SDA�Ӹ߱�ͣ�˵������ʼ�ź�
			if(PORT_GetBit(PORT1, PIN4)>>4)
			{
				SwSlaveI2C.State = I2C_STA_START;
				//INTP_Stop(1<<1);
			}
		}
		else
		{
			// SCLΪ�ߵ�ƽʱ��SDA�ӵͱ�ߣ�˵����ֹͣ�źţ�һ��I2Cͨ�Ž�����ֱ�ӽ�״̬���óɿ���
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
// 			// I2Cͨ�ų�ʱ�Ļ�������״̬��������SDA���ó�����
// 			SwSlaveI2C.State = I2C_STA_IDLE;
// 			SET_SDA_DIR(TimeMs, GPIO_DIR_IN);
// 		}
// 	}
// }
