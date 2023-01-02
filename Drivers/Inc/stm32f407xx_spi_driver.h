/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jan 2nd, 2023
 *      Author: saikiran
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_ClockSpeed;
    uint8_t SPI_DataFrameFormat;
    uint8_t SPI_ClockPolarity;
    uint8_t SPI_ClockPhase;
    uint8_t SPI_SlaveSelectManagement; // HW orSW Slave select Management
} SPI_Config_T;

typedef struct
{
    SPI_Register_Def_T* pSPIx;
    SPI_Config_T        SPIHandle;
} SPI_Handle_T;

/**************************************************************************'
 *  				APIS supported by the driver
 *
 ****************************************************************************/

/*
 * Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_T* pToSPIHandle);
void SPI_DeInit(SPI_Register_Def_T* pSPIx);
/*
 * clock control
 */

/*
 * Read and Write
 */

/*
 * ISR and Interrupt
 */

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */