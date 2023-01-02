/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Mar 18, 2022
 *      Author: saikiran
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * clock control
 */

void GPIO_PClkControl(GPIO_Register_Def_T* pGPIOx, uint8_t EnorDi)
{
    if (pGPIOx == GPIOA)
    {
        if (EnorDi == ENABLE)
        {
            GPIOA_PCLK_EN();
        }
        else
        {
            GPIOA_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOB)
    {
        if (EnorDi == ENABLE)
        {
            GPIOB_PCLK_EN();
        }
        else
        {
            GPIOB_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOC)
    {
        if (EnorDi == ENABLE)
        {
            GPIOC_PCLK_EN();
        }
        else
        {
            GPIOC_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOD)
    {
        if (EnorDi == ENABLE)
        {
            GPIOD_PCLK_EN();
        }
        else
        {
            GPIOD_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOE)
    {
        if (EnorDi == ENABLE)
        {
            GPIOE_PCLK_EN();
        }
        else
        {
            GPIOE_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOF)
    {
        if (EnorDi == ENABLE)
        {
            GPIOF_PCLK_EN();
        }
        else
        {
            GPIOF_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOG)
    {
        if (EnorDi == ENABLE)
        {
            GPIOG_PCLK_EN();
        }
        else
        {
            GPIOG_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOH)
    {
        if (EnorDi == ENABLE)
        {
            GPIOH_PCLK_EN();
        }
        else
        {
            GPIOH_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOI)
    {
        if (EnorDi == ENABLE)
        {
            GPIOI_PCLK_EN();
        }
        else
        {
            GPIOI_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOJ)
    {
        if (EnorDi == ENABLE)
        {
            GPIOJ_PCLK_EN();
        }
        else
        {
            GPIOJ_PCLK_DS();
        }
    }
    else if (pGPIOx == GPIOK)
    {
        if (EnorDi == ENABLE)
        {
            GPIOK_PCLK_EN();
        }
        else
        {
            GPIOK_PCLK_DS();
        }
    }
}

/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_T* pToGPIOHandle)
{
    uint32_t temp               = 0;
    uint8_t  portcode           = 0;
    uint32_t altFunRegHighOrLow = 0;
    uint32_t altFunRegPinNumber = 0;

    if (pToGPIOHandle->pGPIOHandle.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pToGPIOHandle->pGPIOHandle.GPIO_PinMode << (2 * pToGPIOHandle->pGPIOHandle.GPIO_PinNumber));
        pToGPIOHandle->pGPIOx->MODER &= ~(0x3 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
        pToGPIOHandle->pGPIOx->MODER |= temp;
        temp = 0;
    }
    else
    {
        // Interrupt Modes
        if (pToGPIOHandle->pGPIOHandle.GPIO_PinMode == GPIO_MODE_INT_FT)
        {
            EXTI->FTSR |= (1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
            EXTI->RTSR &= ~(1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
        }
        else if (pToGPIOHandle->pGPIOHandle.GPIO_PinMode == GPIO_MODE_INT_RT)
        {
            EXTI->RTSR |= (1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
            EXTI->FTSR &= ~(1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
        }
        else if (pToGPIOHandle->pGPIOHandle.GPIO_PinMode == GPIO_MODE_INT_RFT)
        {
            EXTI->RTSR |= (1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
        }
        else
        {
            /*Do Nothing*/
        }

        // Enable Sys config Clock
        SYSCFG_PCLK_EN();
        // Assigning the GPIO pin to EXTI Line by configuring SYSCFG register
        altFunRegHighOrLow = (pToGPIOHandle->pGPIOHandle.GPIO_PinNumber / 4U);
        altFunRegPinNumber = (pToGPIOHandle->pGPIOHandle.GPIO_PinNumber % 4U);
        portcode           = GPIO_BASEADDRESS_TO_PORT_CODE(pToGPIOHandle->pGPIOx);

        if (altFunRegHighOrLow == 0U)
        {
            SYSCFG->EXTICR1 = portcode << (altFunRegPinNumber * 4);
        }
        else if (altFunRegHighOrLow == 1U)
        {
            SYSCFG->EXTICR2 = portcode << (altFunRegPinNumber * 4);
        }
        else if (altFunRegHighOrLow == 2U)
        {
            SYSCFG->EXTICR3 = portcode << (altFunRegPinNumber * 4);
        }
        else if (altFunRegHighOrLow == 3U)
        {
            SYSCFG->EXTICR4 = portcode << (altFunRegPinNumber * 4);
        }

        // EXTI IMR - Enable Interrupt Mask
        EXTI->IMR |= (1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
    }

    if (pToGPIOHandle->pGPIOHandle.GPIO_PinSpeed <= GPIO_SPEED_HIGH)
    {
        temp = (pToGPIOHandle->pGPIOHandle.GPIO_PinSpeed << (2 * pToGPIOHandle->pGPIOHandle.GPIO_PinNumber));
        pToGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
        pToGPIOHandle->pGPIOx->OSPEEDR |= temp;
        temp = 0;
    }

    // Output type as push pull or open drain. PP -0, OD - 1
    temp = (pToGPIOHandle->pGPIOHandle.GPIO_PinOpType << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
    pToGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
    pToGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;

    if (pToGPIOHandle->pGPIOHandle.GPIO_PinPuPdControl <= GPIO_PIN_RSRVD)
    {
        temp = (pToGPIOHandle->pGPIOHandle.GPIO_PinPuPdControl << (2 * pToGPIOHandle->pGPIOHandle.GPIO_PinNumber));
        pToGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pToGPIOHandle->pGPIOHandle.GPIO_PinNumber);
        pToGPIOHandle->pGPIOx->PUPDR = temp;
        temp                         = 0;
    }

    // Alternate Functionality
    if (pToGPIOHandle->pGPIOHandle.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        altFunRegHighOrLow = (pToGPIOHandle->pGPIOHandle.GPIO_PinNumber / 8U);
        altFunRegPinNumber = (pToGPIOHandle->pGPIOHandle.GPIO_PinNumber % 8U);

        if (altFunRegHighOrLow == 1u)
        {
            temp = (pToGPIOHandle->pGPIOHandle.GPIO_PinAltFunctionMode << (4 * altFunRegPinNumber));
            pToGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4 * altFunRegPinNumber));
            pToGPIOHandle->pGPIOx->AFRH |= temp;
            temp = 0;
        }
        else
        {
            temp = (pToGPIOHandle->pGPIOHandle.GPIO_PinAltFunctionMode << (4 * altFunRegPinNumber));
            pToGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4 * altFunRegPinNumber));
            pToGPIOHandle->pGPIOx->AFRL |= temp;
            temp = 0;
        }
    }
}

/*
 * Init and Deinit
 */
void GPIO_DeInit(GPIO_Register_Def_T* pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
    else if (pGPIOx == GPIOJ)
    {
        GPIOJ_REG_RESET();
    }
    else if (pGPIOx == GPIOK)
    {
        GPIOK_REG_RESET();
    }
    else
    {
        /*Do Nothing*/
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_Register_Def_T* pGPIOx, uint8_t PinNumber)
{
    return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001U);
}

uint16_t GPIO_ReadFromInputPort(GPIO_Register_Def_T* pGPIOx) { return (uint16_t)(pGPIOx->IDR); }

void GPIO_WriteToOutputPin(GPIO_Register_Def_T* pGPIOx, uint8_t PinNumber, uint8_t value)
{
    if (value == SET)
    {
        pGPIOx->ODR |= (value << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_Register_Def_T* pGPIOx, uint32_t value) { pGPIOx->ODR = value; }

void GPIO_ToggleOutPutPin(GPIO_Register_Def_T* pGPIOx, uint8_t PinNumber) { pGPIOx->ODR ^= (1 << PinNumber); }

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            (*NVIC_ISER0) |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            (*NVIC_ISER1) |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            (*NVIC_ISER2) |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            (*NVIC_ICER0) |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            (*NVIC_ICER1) |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            (*NVIC_ICER2) |= (1 << (IRQNumber % 64));
        }
    }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IrqPriority)
{
    uint8_t  iprx;
    uint8_t  iprx_section;
    uint16_t shift_amout;

    iprx         = IRQNumber / 4;
    iprx_section = IRQNumber % 4;

    // This is implemented because Interrupt priority register MSB 4 bits are not
    // supported in any of the available 239 registers
    shift_amout = (8 * iprx_section) + (8 - NUM_OF_PR_BITS_IMPLEMNTD);

    *((NVIC_PRIORITY_REG_BASE) + (iprx * 4)) |= (IrqPriority << shift_amout);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    if (EXTI->PR & (1 << PinNumber))
    {
        // CLEAR. writing 1 to this bit clear the PR
        EXTI->PR |= ((1 << PinNumber));
    }
}
