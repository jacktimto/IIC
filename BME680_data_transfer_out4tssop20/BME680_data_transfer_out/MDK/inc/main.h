#ifndef __MAIN_H__
#define	__MAIN_H__
#include "CMS32L051.h"
#include <stdio.h>
#include <string.h>
#include "userdefine.h"
#include "sci.h"
#include "iica.h"
#include "bme680.h"
#include "bme680_defs.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "bsec_integration.h"



//int8_t bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);//scl P60 SDA P61
//int8_t bus_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
//void delay_ms(uint32_t t_ms);
//void sleep(uint32_t t_ms);
////state_load
//uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer);
////config_load
//uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer);
//void state_save(const uint8_t *state_buffer, uint32_t length);
//int64_t get_timestamp_us(void);
//void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
//     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
//     float static_iaq, float co2_equivalent, float breath_voc_equivalent);

#endif	
