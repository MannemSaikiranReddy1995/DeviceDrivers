/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Mar 18, 2022
 *      Author: sai31
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

#define ENABLE  1U
#define DISABLE 0U
#define SET     1U
#define RESET   0U

#define PRESSED     1U
#define NOT_PRESSED 0U

#define GPIO_MODE_IN      0U
#define GPIO_MODE_OUT     1U
#define GPIO_MODE_ALTFN   2U
#define GPIO_MODE_ANALOG  3U
#define GPIO_MODE_INT_FT  4U
#define GPIO_MODE_INT_RT  5U
#define GPIO_MODE_INT_RFT 6U

#define GPIO_OP_TYPE_PUSHPULL  0U
#define GPIO_OP_TYPE_OPENDRAIN 1U

#define GPIO_SPEED_LOW    0U
#define GPIO_SPEED_MEDIUM 1U
#define GPIO_SPEED_FAST   2U
#define GPIO_SPEED_HIGH   3U

#define GPIO_NO_PUPD   0U
#define GPIO_PIN_PU    1U
#define GPIO_PIN_PD    2U
#define GPIO_PIN_RSRVD 3U

#define GPIO_PIN_NUM_6  6U
#define GPIO_PIN_NUM_7  7U
#define GPIO_PIN_NUM_3  3U
#define GPIO_PIN_NUM_4  4U
#define GPIO_PIN_NUM_12 12U
#define GPIO_PIN_NUM_13 13U
#define GPIO_PIN_NUM_14 14U
#define GPIO_PIN_NUM_15 15U
#define GPIO_PIN_NUM_0  0U

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOpType;
    uint8_t GPIO_PinAltFunctionMode;
} GPIO_PinConfig_T;

typedef struct
{
    GPIO_Register_Def_T* pGPIOx;
    GPIO_PinConfig_T     pGPIOHandle;
} GPIO_Handle_T;

/**************************************************************************'
 *  				APIS supported by the driver
 *
 ****************************************************************************/

/*
 * Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_T* pToGPIOHandle);
void GPIO_DeInit(GPIO_Register_Def_T* pGPIOx);

/*
 * clock control
 */
void GPIO_PClkControl(GPIO_Register_Def_T* pGPIOx, uint8_t EnorDi);

/*
 * Read and Write
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_Register_Def_T* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Register_Def_T* pGPIOx);
void     GPIO_WriteToOutputPin(GPIO_Register_Def_T* pGPIOx, uint8_t PinNumber, uint8_t value);
void     GPIO_WriteToOutputPort(GPIO_Register_Def_T* pGPIOx, uint32_t value);
void     GPIO_ToggleOutPutPin(GPIO_Register_Def_T* pGPIOx, uint8_t PinNumber);
/*
 * ISR and Interrupt
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IrqPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
