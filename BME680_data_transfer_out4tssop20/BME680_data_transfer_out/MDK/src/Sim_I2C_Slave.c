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
uint8_t RxDataBuf[20];
uint8_t TxDataBuf[20];

SwSlaveI2C_t SwSlaveI2C =
{
	I2C_STA_IDLE,		// State
	I2C_WRITE,			// Rw
	0, 					// SclFallCnt
    0,                  // SclRaiseCnt
	0,					// Flag
	0,					// StartMs
	RxDataBuf,			// RxBuf
	TxDataBuf,			// TxBuf
	0,					// RxIdx
	0					// TxIdx
};


void intp_1_interrupt(void)
{
    //清中断
    INTC_ClearPendingIRQ(INTP1_IRQn);
	I2C_intp1();
}

/**
  *外部中断2
  *
  */
void intp_2_interrupt(void)
{
    INTC_ClearPendingIRQ(INTP2_IRQn);
    I2C_intp2();
}

void InitSwSlaveI2C(void)
{
	INTP_Init(1<<1, INTP_BOTH);//初始化INTP1
	INTP_Init(1<<2, INTP_BOTH);//初始化INTP2
	INTP_Start(1<<1);
	INTP_Start(1<<2);

}
void I2C_intp1(void)
{
    //如果是sda下降沿,并且scl是高
    if ((PORT_GetBit(PORT1, PIN3)&0x08) == 0)
    {
        if(PORT_GetBit(PORT1, PIN4)&0x10)
        {
            SwSlaveI2C.State = I2C_STA_START;
        }
    }
    else
    {
        //sda 高电平,scl高电平,stop信号
        if (PORT_GetBit(PORT1, PIN4)&0x10)
        {
            SwSlaveI2C.State = I2C_STA_IDLE;
        }
    }

    //TODO:关闭intp1
}


void I2C_intp2(void)
{
    //scl下降沿,p14
    if ((PORT_GetBit(PORT1, PIN4) & 0x10) == 0)
    {
        switch (SwSlaveI2C.State)
        {
            case I2C_STA_START:
                SwSlaveI2C.SclFallCnt = 0;
                SwSlaveI2C.RxIdx = 0;
                SwSlaveI2C.TxIdx = 0;
                SwSlaveI2C.Flag = 0;
                SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
                SwSlaveI2C.Rw = I2C_WRITE;
                SwSlaveI2C.State = I2C_STA_DATA;
                break;
            case I2C_STA_DATA:
                SwSlaveI2C.SclFallCnt++;
                //如果主机是读数据,从机更新数据
                if ((8 > SwSlaveI2C.SclFallCnt))
                {
                    if (SwSlaveI2C.Rw == I2C_READ)
                    {

                        if ((SwSlaveI2C.TxBuf[0]) & (1 << (7 - SwSlaveI2C.SclFallCnt)))
                        {
                            PORT->P1 |= 0x08;//SDA 输出高电平
                        } else
                        {
                            PORT->P1 &= 0xF7;//SDA 输出低电平
                        }
                    }
                } else if (8 == SwSlaveI2C.SclFallCnt)//下降沿计数第8个,说明数据接收完毕
                {
                    if (SwSlaveI2C.Rw == I2C_WRITE)
                    {
                        //从RxBuf[0]中读取数据
                        if((SwSlaveI2C.RxBuf[0] & 0xFE) == 0xEC)
                        {
                            SwSlaveI2C.Flag = 1;
                            SwSlaveI2C.Rw = SwSlaveI2C.RxBuf[0] & 0x01;//判断是write还是read
                        }

                        if (SwSlaveI2C.Flag)
                        {
                            //INTM->EGN0 &= 0xFD;
                            //INTM->EGP0 &= 0xFD;//SDA 改为输出
							PORT->POM1 |= 0x08;
                            PORT->PM1  &= 0xF7;
                            PORT->P1   &= 0xF7;//输出ACK信号
                        }
                    }
                    else //I2C_read
                    {
                        PORT->PM1 |= 0x08;//SDA 设为输入,,判断主机的应答
                        //INTM->EGN0 |= 0x02;
                        //INTM->EGP0 |= 0x02;
                        SwSlaveI2C.TxIdx++;//发送下一个数据
                    }

                    SwSlaveI2C.State = I2C_STA_ACK;
                }
                break;
            case I2C_STA_ACK:
                SwSlaveI2C.SclFallCnt = 0;
                if (SwSlaveI2C.Rw == I2C_WRITE)
                {
                    PORT->PM1 |= 0x08;
                    //INTM->EGN0 |= 0x02;
                    //INTM->EGP0 |= 0x02;//SDA设置为输入
                    SwSlaveI2C.RxIdx++;
                    SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
                }
                else //I2C_read
                {
                    //INTM->EGN0 &= 0xFD;
                    //INTM->EGP0 &= 0xFD;//SDA 改为输出
				    PORT->POM1 |= 0x08;
                    PORT->PM1  &= 0xF7;
                    switch (SwSlaveI2C.RxBuf[1])
                        {
                            case 0x00:
                                SwSlaveI2C.TxBuf[0] = bme680_data[0];
                                break;
                            case 0x01:
                                SwSlaveI2C.TxBuf[0] = bme680_data[1];
                                break;
                            case 0x02:
                                SwSlaveI2C.TxBuf[0] = bme680_data[2];
                                break;
                            case 0x03:
                                SwSlaveI2C.TxBuf[0] = bme680_data[3];
                                break;
                            case 0x04:
                                SwSlaveI2C.TxBuf[0] = bme680_data[4];
                                break;
					    }

                    if (SwSlaveI2C.TxBuf[0] & 0x80)
                    {
                        PORT->P1 |= 0x08;//sda输出高
                    }
                    else
                    {
                        PORT->P1 &= 0xF7;//sda输出低
                    }
                }
                SwSlaveI2C.State = I2C_STA_DATA;
                break;
            case I2C_STA_NACK:
                SwSlaveI2C.SclFallCnt = 0;
                PORT->PM1 |= 0x08;//SDA输入
                break;
			
        }
    }
    else  //scl上升沿
    {
        switch (SwSlaveI2C.State)
        {
            case I2C_STA_DATA:
                if ((I2C_WRITE == SwSlaveI2C.Rw) && (8 > SwSlaveI2C.SclFallCnt))
                {   //读SDA的数据
                    if (PORT_GetBit(PORT1, PIN3) & 0x08)
                    {
                        SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] |= (1 << (7 - SwSlaveI2C.SclFallCnt));
                    }
                }
                break;
            case I2C_STA_ACK://判断主机在应答阶段有没有应答  master ACK
                if ((SwSlaveI2C.Rw == I2C_READ) && (PORT_GetBit(PORT1, PIN3) & 0x08))
                {//判断到SDA是高电平,认为是非应答
                    SwSlaveI2C.State = I2C_STA_NACK;
                }
                break;
        }
    }

}

