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

//extern I2C_HandleTypeDef hi2c1;

void convertBytesToWords(uint16_t* sourceArray, uint16_t arraySize){
  uint16_t swapBuf = 0u;

	for (uint16_t byteCntr = 0u; byteCntr < arraySize; byteCntr++) {
    swapBuf = *sourceArray;
    *sourceArray = (swapBuf >> 8u) | ((swapBuf & 0xFFu) << 8u);
		sourceArray++;
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
 #if 0
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0x00U;

  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();
  
  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if(hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);
    
    /* Check if the I2C is already enabled */
    if((hi2c->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
      /* Enable I2C peripheral */
      __HAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    hi2c->Instance->CR1 &= ~I2C_CR1_POS;

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size * 2;//Size; //Multiply by two to get words
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferSize    = hi2c->XferCount; // XferSize also multiplied by two

    /* Send Slave Address and Memory Address */
    if(I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_TIMEOUT;
      }
    }

    if(hi2c->XferSize == 0U)
    {
      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
      
      /* Generate Stop */
      hi2c->Instance->CR1 |= I2C_CR1_STOP;
    }
    else if(hi2c->XferSize == 1U) // not possible now
    {
      /* Disable Acknowledge */
      hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

      /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
         software sequence must complete before the current byte end of transfer */
      __disable_irq();

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Stop */
      hi2c->Instance->CR1 |= I2C_CR1_STOP;

      /* Re-enable IRQs */
      __enable_irq();
    }
    else if(hi2c->XferSize == 2U)
    {
      /* Enable Pos */
      hi2c->Instance->CR1 |= I2C_CR1_POS;

      /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
         software sequence must complete before the current byte end of transfer */
      __disable_irq();

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
      
      /* Disable Acknowledge */
      hi2c->Instance->CR1 &= ~I2C_CR1_ACK;
      
       /* Re-enable IRQs */
       __enable_irq(); 
    }
    else
    {
      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CR1, I2C_CR1_ACK);

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }

    while(hi2c->XferSize > 0U)
    {
      if(hi2c->XferSize <= 3U)
      {
        /* One byte */
        if(hi2c->XferSize== 1U)
        {
          /* Wait until RXNE flag is set */
          if(I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)      
          {
            if(hi2c->ErrorCode == HAL_I2C_ERROR_TIMEOUT)
            {
              return HAL_TIMEOUT;
            }
            else
            {
              return HAL_ERROR;
            }
          }

          /* Read data from DR */
          (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
        /* Two bytes --> one word */
        else if(hi2c->XferSize == 2U)
        {
          /* Wait until BTF flag is set */
          if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
             software sequence must complete before the current byte end of transfer */
           __disable_irq();

          /* Generate Stop */
          hi2c->Instance->CR1 |= I2C_CR1_STOP;

          /* Read data from DR */
          (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Re-enable IRQs */
          __enable_irq();

          /* Read data from DR */
          (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
        /* 3 Last bytes */
        else
        {
          /* Wait until BTF flag is set */
          if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          /* Disable Acknowledge */
          hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

          /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
             software sequence must complete before the current byte end of transfer */
          __disable_irq();

          /* Read data from DR */
          (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Wait until BTF flag is set */
          if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          /* Generate Stop */
          hi2c->Instance->CR1 |= I2C_CR1_STOP;

          /* Read data from DR */
          (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Re-enable IRQs */
          __enable_irq();

          /* Read data from DR */
          (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
      }
      else
      {
        /* Wait until RXNE flag is set */
        if(I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
        {
          if(hi2c->ErrorCode == HAL_I2C_ERROR_TIMEOUT)
          {
            return HAL_TIMEOUT;
          }
          else
          {
            return HAL_ERROR;
          }
        }

        /* Read data from DR */
        (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
        hi2c->XferSize--;
        hi2c->XferCount--;

        if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET)
        {
          /* Read data from DR */
          (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
      }
    }

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode = HAL_I2C_MODE_NONE;
    
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
#endif
void MLX90640_I2CInit()
{   
	
}

#if 0
int I2C_ReadBurst(uint8_t i2cAddr, uint16_t memAddr, uint16_t* dataBuf)
{
  LL_I2C_GenerateStartCondition(I2C1);
  /* Send I2C address, write */
  LL_I2C_TransmitData8(I2C1, (uint8_t)(i2cAddr << 1u));
  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddr >> 8u));
  LL_I2C_TransmitData8(I2C1, (uint8_t)memAddr);
  /* Send repeated start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Send I2C address, read */
  LL_I2C_TransmitData8(I2C1, (uint8_t)(i2cAddr << 1u) | 0x01u );
  /* Read one word in a time */
  uint16_t word = LL_I2C_ReceiveData8(I2C1);
  word <<= 8u;
  word |= LL_I2C_ReceiveData8(I2C1);
  /* Send stop when all done */
  LL_I2C_GenerateStopCondition(I2C1);

  *dataBuf = word;
  dataBuf++;

  return (int)dataBuf;
  //LL_I2C_ReceiveData8()
  //LL_I2C_Init
  //LL_I2C_DeInit
  //LL_I2C_StructInit
}
#endif 

uint16_t MLX90640_I2CReadWord(uint8_t slave_address, uint16_t start_address) {

	uint16_t puff;
	volatile uint8_t reg_m,reg_l,dat_m,dat_l;

	reg_m = (uint8_t) ((start_address & 0xFF00) >> 8);	//Address MSB
	reg_l = (uint8_t) (start_address & 0x00FF); 	    //Address LSB


	while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
	}

	//LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
	}

	//Send  device address
	LL_I2C_TransmitData8(I2C1, slave_address);
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
	}
	LL_I2C_ClearFlag_ADDR(I2C1);

	while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
	}

	//Send start address MSB
	LL_I2C_TransmitData8(I2C1, reg_m);
	while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
	}

	//Send start address LSB
	LL_I2C_TransmitData8(I2C1, reg_l);
	while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
	}

	//Repeat start condition
	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
	}

	//Send device address again + read
	LL_I2C_TransmitData8(I2C1, slave_address | 0x01);
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
	}

	//Read out data MSB, send ACK
	LL_I2C_ClearFlag_ADDR(I2C1);
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
	while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
	}
	dat_m = LL_I2C_ReceiveData8(I2C1);

	//Read out data LSB, send NACK
	while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
	}
	dat_l = LL_I2C_ReceiveData8(I2C1);
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
	LL_I2C_GenerateStopCondition(I2C1);

	return ((uint16_t) (dat_m << 8)) | ((uint16_t)((dat_l) & 0x00FF));

}

/*
int MLX90640_I2CRead(uint8_t slave_address, uint16_t start_address, uint16_t numb_words, uint16_t *data){

	uint16_t temp_address = start_address;
	uint16_t temp_data;
	for(int i=0; i < numb_words; i++){

		temp_data = MLX90640_I2CReadWord(slave_address,temp_address);
		temp_address++;
		*(data + i) = temp_data;

	}
	return 0;
}*/

int MLX90640_I2CWrite(uint8_t slave_address, uint16_t start_address, uint16_t data) {

	uint8_t reg_m,reg_l,dat_m,dat_l;
	reg_m = (uint8_t) ((start_address & 0xFF00) >> 8);			//Address MSB
	reg_l = (uint8_t) (start_address & 0x00FF); 				//Address LSB
	dat_m = (uint8_t) ((data & 0xFF00) >> 8);	// Data MSB
	dat_l = (uint8_t) (data & 0x00FF);			//Data LSB

	//reg_m = 0x80;
	//reg_l = 0x0D;
	//dat_m = 0x09;
	//dat_l = 0x01;
	//Check bus busy flag
	while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
	}

	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
	}

	//Send device address
	LL_I2C_TransmitData8(I2C1, slave_address);
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
	}
	LL_I2C_ClearFlag_ADDR(I2C1);

	while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
	}

	//Send write address MSB
	LL_I2C_TransmitData8(I2C1, reg_m);
	while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
	}

	//Send write address LSB
	LL_I2C_TransmitData8(I2C1, reg_l);
	while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
	}

	//Send data MSB
	LL_I2C_TransmitData8(I2C1, dat_m);
	while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
	}

	//Send data LSB
	LL_I2C_TransmitData8(I2C1, dat_l);
	while (!LL_I2C_IsActiveFlag_BTF(I2C1)) {
	}

	LL_I2C_GenerateStopCondition(I2C1);

	return 0;
}



int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
	#if 0
    if (nMemAddressRead <= MAX_BURST_IO) {
        HAL_I2C_Mem_Read(&hi2c1, slaveAddr, startAddress, 2u, (uint8_t*)data, nMemAddressRead * 2u, 500u);
        convertBytesToWords((uint16_t*)data, 1u);
    } else {
        uint16_t dataToRead = nMemAddressRead;

        for (uint16_t i = 0u; i <= (nMemAddressRead / MAX_BURST_IO) + 1u; i++) {
            if (dataToRead > MAX_BURST_IO) {
                HAL_I2C_Mem_Read(&hi2c1, slaveAddr, startAddress + i * MAX_BURST_IO, 2u, (uint8_t*)data + i * MAX_BURST_IO, MAX_BURST_IO  * 2u, 500u);
                dataToRead -= MAX_BURST_IO;
            } else {
                HAL_I2C_Mem_Read(&hi2c1, slaveAddr, startAddress + i * MAX_BURST_IO, 2u, (uint8_t*)data + i * MAX_BURST_IO, dataToRead * 2u, 500u);
            }
            convertBytesToWords((uint16_t*)data + i * MAX_BURST_IO, 1u);
        }
    }
	#endif
    return 0;
} 

void MLX90640_I2CFreqSet(int freq)
{
	
}

/*
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{   
    int retVal = 0;
		uint8_t sendWordArray[2u];
		
		sendWordArray[0u] = (uint8_t)data;
		sendWordArray[1u] = (uint8_t)(data >> 8u);
	
    HAL_I2C_Mem_Write(&hi2c1, slaveAddr, writeAddress, 2u, sendWordArray, 2u, 500u);
    
    return retVal;
}
*/
