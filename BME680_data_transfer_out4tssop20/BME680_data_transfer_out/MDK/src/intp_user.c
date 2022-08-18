#include "userdefine.h"
#include "CMS32L051.h"
#include "intp.h"
#include "gpio.h"

extern uint8_t bme680_data[11];
uint8_t send_data = 0xFF;

uint8_t IIC_SCL_rise_cnt;//scl上升沿计数
uint8_t IIC_SCL_fall_cnt;//scl下降沿计数

uint8_t IIC_RW;//读写标志位

#define write 0
#define read 1

uint8_t IIC_CMD_Flag ;//IIC CMD标志位
uint8_t IIC_Send_Data_Flag;//IIC发送标志位
uint8_t IIC_STA_STOP_Flag;//IIC stop标志位

uint8_t rx_temp_buff[6];
uint8_t rx_temp;
uint16_t rx_temp_1;

uint8_t IIC_STA = I2C_STA_IDLE;//默认IIC为idle


void IRQ02_Handler(void) __attribute__((alias("intp_1_interrupt")));
void IRQ03_Handler(void) __attribute__((alias("intp_2_interrupt")));

volatile uint32_t g_intp1Taken;  	/* INTP1 taken */
volatile uint32_t g_intp2Taken;  	/* INTP2 taken */

void intp_1_interrupt(void)
{
    INTC_ClearPendingIRQ(INTP1_IRQn);

    g_intp1Taken++;//进中断次数

    if (PORT_GetBit(PORT1,PIN4) == 0x10)//start
    {
        INTC_ClearPendingIRQ(INTP1_IRQn);//清标志

        INTP_Stop(1<<1);//Intp1 stop

        IIC_STA = I2C_STA_START;

        INTC_ClearPendingIRQ(INTP2_IRQn);//清标志
        INTM->EGN0 &= 0xFB;//scl上升沿
        INTM->EGP0 |= 0x04;
    }
    else
    {
        //清除状态
        IIC_STA = I2C_STA_IDLE;
    }
}


void intp_2_interrupt(void)//CLK
{
    INTC_ClearPendingIRQ(INTP2_IRQn);
    g_intp2Taken++;
    static uint8_t bit_temp = 0;

    if (IIC_CMD_Flag == 1)
    {
        IIC_CMD_Flag = 0;
        IIC_STA = I2C_STA_Cmd;
    } else if (IIC_Send_Data_Flag == 1)
    {
        IIC_Send_Data_Flag = 0;
        IIC_STA = I2C_STA_SEND_DATA;
    } else if (IIC_STA_STOP_Flag == 1)
    {
        IIC_STA_STOP_Flag =0;
        IIC_STA = I2C_STA_STOP;
    }

    switch (IIC_STA) 
		{
        case I2C_STA_START:
			//PORT->P2 &= ~(1 << 1);//输出低
            //__NOP();
            //PORT->P2 |= (1 << 1);//输出高
            bit_temp = (PORT_GetBit(PORT1,PIN3) == 0x08);
		
            rx_temp = rx_temp << 1;
            rx_temp = rx_temp + bit_temp;
			IIC_SCL_rise_cnt ++;
		
            if (IIC_SCL_rise_cnt > 8)
            {
                IIC_SCL_rise_cnt = 0;
                switch (rx_temp) 
					{
                    case 0xEC:
                        rx_temp_buff[0] = rx_temp;
						rx_temp = 0;
                        IIC_CMD_Flag = 1;
                        break;
                    case 0xED:
						if(rx_temp_buff[0] == 0xEC)
						{
							rx_temp_buff[2] = rx_temp;
							rx_temp = 0;
                            INTP_Stop(0x04);//stop P14
//                          PORT_Init(PORT1, PIN3,OPENDRAIN_OUTPUT);//P13改为输出
//							PORT -> PM1 &= ~(1<<3);
//							PORT -> PMC1 &= ~(1<<3);
//                          PORT -> PM1 &= ~(1<<3);
//                          PORT -> P1 &= ~(1<<3);
//                          PORT -> POM1 &= ~(1<<3);
							delay_us(80);
							
							PORT_Init(PORT1, PIN3,OPENDRAIN_OUTPUT);//P13改为输出模式
							PORT -> P1 &= ~(1<<3);//输出低 
							
                            switch (rx_temp_buff[1])
                            {
                                case 0x00:
                                    send_data = bme680_data[0];
                                    break;
                                case 0x01:
                                    send_data = bme680_data[1];
                                    break;
                                case 0x02:
                                    send_data = bme680_data[2];
                                    break;
                                case 0x03:
                                    send_data = bme680_data[3];
                                    break;
                                case 0x04:
                                    send_data = bme680_data[4];
                                    break;
                            }
							
							IIC_Send_Data_Flag = 1;
							INTM ->EGN0 |= (1<<4);//SCL下降沿触发
                            INTM ->EGP0 &= ~(1<<4);
                            INTP_Start(0x04);//start P14
                            
						}	

                        break;
                }

            }
            break;
        case I2C_STA_Cmd:
            bit_temp = (PORT_GetBit(PORT1,PIN3) == 0x08);
            rx_temp_1 = rx_temp_1 << 1;
            rx_temp_1 = rx_temp_1 + bit_temp;
			IIC_SCL_rise_cnt ++;
            if (IIC_SCL_rise_cnt > 9)
            {
                IIC_SCL_rise_cnt = 0;
                rx_temp_buff[1] = (uint8_t)rx_temp;
                IIC_STA = I2C_STA_START;
            }
            break;
			
        case I2C_STA_SEND_DATA:
            IIC_SCL_fall_cnt ++;
            Sim_IIC_Send_Byte(send_data);
            if (IIC_SCL_fall_cnt > 9)
            {
                IIC_SCL_fall_cnt = 0;
				PORT -> PM1 |= (1<<3);
                INTP_Stop(0x04);
                IIC_STA_STOP_Flag = 1;
            }

            break;
        case I2C_STA_STOP:
            Sim_IIC_Init();
			rx_temp_buff[0] = 0;
			rx_temp_buff[1] = 0;
			rx_temp_buff[2] = 0;
			send_data = 0xFF;
            IIC_STA = I2C_STA_IDLE;
            break;
        default:
            break;

    }




}


	










