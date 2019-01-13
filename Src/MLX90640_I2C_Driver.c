/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "MLX90640_I2C_Driver.h"
#include "stm32f1xx_hal.h"
#define MAX_BURST_IO 120u
extern I2C_HandleTypeDef hi2c1;

void MLX90640_I2CInit()
{   
	
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    if (nMemAddressRead <= MAX_BURST_IO) {
        HAL_I2C_Mem_Read(&hi2c1, slaveAddr, startAddress, 2u, (uint8_t*)data, nMemAddressRead * 2u, 500u);
    } else {
        uint8_t dataToRead = nMemAddressRead;

        for (uint16_t i = 0u; i <= (nMemAddressRead / MAX_BURST_IO) + 1u; i++) {
            if (dataToRead > MAX_BURST_IO) {
                HAL_I2C_Mem_Read(&hi2c1, slaveAddr, startAddress + i * MAX_BURST_IO, 2u, (uint8_t*)data + i * MAX_BURST_IO, MAX_BURST_IO  * 2u, 500u);
                dataToRead -= MAX_BURST_IO;
            } else {
                HAL_I2C_Mem_Read(&hi2c1, slaveAddr, startAddress + i * MAX_BURST_IO, 2u, (uint8_t*)data + i * MAX_BURST_IO, dataToRead * 2u, 500u);
            }
        }
    }

    return 0;
} 

void MLX90640_I2CFreqSet(int freq)
{
	
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{   
    int retVal = 0;
		uint8_t sendWordArray[2u];
		
		sendWordArray[0u] = (uint8_t)data;
		sendWordArray[1u] = (uint8_t)(data >> 8u);
	
    HAL_I2C_Mem_Write(&hi2c1, slaveAddr, writeAddress, 2u, sendWordArray, 2u, 500u);
    
    return retVal;
}

