#include "stm32f407xx.h"
#include "string.h"
#include "stm32f407xx_gpio_driver.h"

void delay()
{
	for(volatile uint32_t i=0;i< 500000;i++);
}

int main()
{
	GPIO_Handle_T GPIO_LED_BTN;
	GPIO_Handle_T GPIOLED_D4;

	GPIOLED_D4.pGPIOx = GPIOD;
	GPIOLED_D4.pGPIOHandle.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIOLED_D4.pGPIOHandle.GPIO_PinMode =	GPIO_MODE_OUT;
	GPIOLED_D4.pGPIOHandle.GPIO_PinSpeed =	GPIO_SPEED_FAST;
	GPIOLED_D4.pGPIOHandle.GPIO_PinOpType = GPIO_OP_TYPE_PUSHPULL;
	GPIOLED_D4.pGPIOHandle.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkControl(GPIOD,ENABLE);
	GPIO_Init(&GPIOLED_D4);

	GPIO_LED_BTN.pGPIOx = GPIOA;
	GPIO_LED_BTN.pGPIOHandle.GPIO_PinNumber = GPIO_PIN_NUM_0;
	GPIO_LED_BTN.pGPIOHandle.GPIO_PinMode =	GPIO_MODE_IN;
	GPIO_LED_BTN.pGPIOHandle.GPIO_PinSpeed =	GPIO_SPEED_FAST;
	GPIO_LED_BTN.pGPIOHandle.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&GPIO_LED_BTN);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NUM_0) == PRESSED)
		{
			delay();
			GPIO_ToggleOutPutPin(GPIOD,GPIO_PIN_NUM_12);
		}

	}

	return 0;

}
