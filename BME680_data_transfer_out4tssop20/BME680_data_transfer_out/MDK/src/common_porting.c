#include <string.h>
#include "userdefine.h"
#include "CMS32L051.h"
#include "sci.h"
#include "common_porting.h"


uint8_t tx_buf[256];
uint8_t rx_buf[256];

void sleep(uint32_t t_ms)
{
	g_ticks = t_ms;
	while(g_ticks);
}
int8_t IIC11_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	//uint16_t slave_addr=(uint8_t)(dev_id<<1);
	tx_buf[0] = reg_addr;
    g_iic11_tx_end = 0;
    IIC11_MasterSend((dev_id<<1), tx_buf,1);
	while(g_iic11_tx_end == 0){;};
    g_iic11_rx_end = 0;
	IIC11_MasterReceive((dev_id<<1),data,len);
	while(g_iic11_rx_end == 0);
	return 0;
}

int8_t IIC11_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	//uint8_t slave_addr=(uint8_t)(dev_id<<1);
	tx_buf[0]=reg_addr;
	memcpy(&tx_buf[1],data,len);
    g_iic11_tx_end = 0;
	IIC11_MasterSend((dev_id<<1), tx_buf,len+1);
	while(g_iic11_tx_end == 0);

	
	return 0;

}

void DelayUs(uint32_t Delay)
{
	uint32_t i;

	while(Delay--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}





