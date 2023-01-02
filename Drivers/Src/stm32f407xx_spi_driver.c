/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 2nd, 2023
 *      Author: saikiran
 */

#include "stm32f407xx_spi_driver.h"

void SPI_Init(SPI_Handle_T* pToSPIHandle) {}

void SPI_DeInit(SPI_Register_Def_T* pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
    else
    {
        /*Do Nothing*/
    }
}