//#include <stdio.h>
//#include <stdint.h>
//#include <stdlib.h>
//
//#include "common.h"
//#include "bmi2_defs.h"
//
//#include "main.h"
//#include <stdio.h>
//#include <stdint.h>
//#include <string.h>
//
//#include "stm32wbxx_hal_i2c.h"
//
//I2C_HandleTypeDef hi2c1;
////static void MX_I2C1_Init(void);
////MX_I2C1_Init();
//
//
//BMI2_INTF_RETURN_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t * reg_data, uint32_t len, void * intf_ptr);
//BMI2_INTF_RETURN_TYPE bmp3_i2c_write(uint8_t reg_addr,
//  const uint8_t * reg_data, uint32_t len, void * intf_ptr);
//
//uint8_t GTXBuffer[512], GRXBuffer[2048];
//int8_t coines_read_i2c(uint8_t subaddress, uint8_t * pBuffer, uint32_t ReadNumbr, void * intf_ptr) {
//  uint8_t dev_addr = * (uint8_t * ) intf_ptr;
//  uint16_t DevAddress = dev_addr << 1;
//
//  // send register address
//  HAL_I2C_Master_Transmit( & hi2c1, DevAddress, & subaddress, 1, 100);
//  HAL_I2C_Master_Receive( & hi2c1, DevAddress, pBuffer, ReadNumbr, 100);
//  return 0;
//}
//
//int8_t coines_write_i2c(uint8_t subaddress, uint8_t * pBuffer, uint32_t WriteNumbr, void * intf_ptr) {
//  uint8_t dev_addr = * (uint8_t * ) intf_ptr;
//  uint16_t DevAddress = dev_addr << 1;
//
//  GTXBuffer[0] = subaddress;
//  memcpy( & GTXBuffer[1], pBuffer, WriteNumbr);
//
//  // send register address
//  HAL_I2C_Master_Transmit( & hi2c1, DevAddress, GTXBuffer, WriteNumbr + 1, 100);
//  return 0;
//}
