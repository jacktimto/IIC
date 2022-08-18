#include <stdio.h>
#include <string.h>
#include "CMS32L051.h"
#include "userdefine.h"
#include "sci.h"
#include "gpio.h"
#include "common_porting.h"
#include "bme680.h"
#include "bsec_integration.h"
#include "intp.h"
#include "Sim_I2C_Slave.h"
#include "wdt.h"


uint32_t tick_cnt;
uint16_t iaq_temp;
uint32_t g_ticks;
volatile uint8_t bme680_data[11] ;//[0]:temperature [1]:humidity [2]:TVOC_MSB [3]:TVOC_LSB [4]:checksum



int8_t bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	IIC11_read(dev_id, reg_addr, data, len);
	return 0;
}

int8_t bus_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	IIC11_write(dev_id, reg_addr, data, len);
	return 0;
}

int64_t get_timestamp_us(void)
{
	int64_t temp = 0;
	temp = 1000*(int64_t)tick_cnt;
    return temp;
}

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                  float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
                  float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{  	

	
	printf("timestamp:%lld,IAQ:%0.2f,iaq_accuracy:%d\n",timestamp,iaq,iaq_accuracy);
	printf("temperature:%0.2f,humidity:%0.2f,pressure:%0.2f\n",temperature,humidity,pressure);
	printf("BME680 VERSION:V1.0.2\n");
	bme680_data[0]=(uint8_t)temperature;
	bme680_data[1]=(uint8_t)humidity+10;
	bme680_data[2]=(uint8_t)((uint32_t)iaq>>8);
	bme680_data[3]=(uint8_t)((uint32_t)iaq);
	bme680_data[4]=bme680_data[0]+bme680_data[1]+bme680_data[2]+bme680_data[3];
}

//state_load
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{return 0;}
//state_save
void state_save(const uint8_t *state_buffer, uint32_t length)
{

}
//config_load
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{return 0;}


int main()
{	
	//设置INTP1,2的优先级
	NVIC_SetPriority(INTP1_IRQn,0x00000001);
	NVIC_SetPriority(INTP2_IRQn,0x00000001);
    //NVIC_SetPriority(IIC11_IRQn,0x00000000);
	
	//SystemCoreClockUpdate();
    IIC11_Init();
	InitSwSlaveI2C();
	
	SystemCoreClockUpdate();

	UART0_Init(SystemCoreClock, 19200);
    printf("Hello, UART\n");
	
	uint32_t msCnt; 	// count value of 1ms
	g_ticks = 1000; 	// 1000ms
	//SystemCoreClockUpdate();
	msCnt = SystemCoreClock / 1000;//除1000是1ms
	SysTick_Config(msCnt);
	

	return_values_init ret;
	//PDEBUG("StartBME680Task\r\n");
	/* Call to the function which initializes the BSEC library 
	* Switch on low-power mode and provide no temperature offset */
	ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 3.0f, bus_write, bus_read, sleep, state_load, config_load);
	if (ret.bme680_status)
	{
		/* Could not intialize BME680 */
		//PDEBUG("Could not intialize BSEC library, bme680_status=%d\r\n", ret.bme680_status);
		//printf("BME680_NOT_OK");
		//return (int)ret.bme680_status;
	}
	else if (ret.bsec_status)
	{
		/* Could not intialize BSEC library */
		//PDEBUG("Could not intialize BSEC library, bsec_status=%d\r\n", ret.bsec_status);
		//return (int)ret.bsec_status;
		//printf("BSEC_NOT_OK");
	}

	if((ret.bme680_status == 0) && (ret.bsec_status == 0))
	{
		//PDEBUG("intialize BSEC library successful\r\n");
		/* Call to endless loop function which reads and processes data based on sensor settings */
		/* State is saved every 10000 samples, which means every 10000 * 3 secs = 500 minutes  */
		bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
	}


	

}//end main


/***********************************************************************************************************************
* Function Name: SysTick Handler
* Description  : Decreament the g_ticks value
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void SysTick_Handler(void)
{
	g_ticks--;
	tick_cnt++;		
}

/***********************************************************************************************************************
* Function Name: HardFault_Handler
* Description  : Hard Fault handler to report stacked register values
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
// Hard Fault handler in C, with stack frame location and LR value
// extracted from the assembly wrapper as input parameters
void hard_fault_handler_c(unsigned int * hardfault_args, unsigned lr_value)
{
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;
	
	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);
	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);
	
	printf ("[Hard fault handler]\r\n");
	printf ("R0 = %x\r\n", stacked_r0);
	printf ("R1 = %x\r\n", stacked_r1);
	printf ("R2 = %x\r\n", stacked_r2);
	printf ("R3 = %x\r\n", stacked_r3);
	printf ("R12 = %x\r\n", stacked_r12);
	printf ("Stacked LR = %x\r\n", stacked_lr);
	printf ("Stacked PC = %x\r\n", stacked_pc);
	printf ("Stacked PSR = %x\r\n", stacked_psr);
	printf ("Current LR = %x\r\n", lr_value);
	
	while(1); // endless loop
}

/***********************************************************************************************************************
* Function Name: HardFault_Handler
* Description  : Assembly wrapper using Embedded Assembler in Keil MDK
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
// Hard Fault handler wrapper in assembly
// It extracts the location of stack frame and passes it to handler
// in C as a pointer. We also extract the LR value as second
// parameter.
__asm void HardFault_Handler(void)
{
		MOVS 	r0, #4
		MOV 	r1, LR
		TST 	r0, r1
		BEQ 	stacking_used_MSP
		MRS 	R0, PSP ; first parameter - stacking was using PSP
		B 		get_LR_and_branch
stacking_used_MSP
		MRS 	R0, MSP ; first parameter - stacking was using MSP
get_LR_and_branch
		MOV 	R1, LR ; second parameter is LR current value
		LDR 	R2,=__cpp(hard_fault_handler_c)
		BX 		R2
}


