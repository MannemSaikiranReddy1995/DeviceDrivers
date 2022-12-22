/*
 * stm32f407xx.h
 *
 *  Created on: Mar 18, 2022
 *      Author: saikiran
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo 	volatile

/*ARM CORTEX MX ISER NVIC Register addresses. this can be found in ARM CORTEX M4 Manual*/

#define	NVIC_ISER0 						((__vo uint32_t*)0XE000E100)
#define	NVIC_ISER1 						((__vo uint32_t*)0XE000E104)
#define	NVIC_ISER2 						((__vo uint32_t*)0XE000E108)
#define	NVIC_ISER3 						((__vo uint32_t*)0XE000E10C)

#define	NVIC_ICER0 						((__vo uint32_t*)0XE000E180)
#define	NVIC_ICER1 						((__vo uint32_t*)0XE000E184)
#define	NVIC_ICER2 						((__vo uint32_t*)0XE000E188)
#define	NVIC_ICER3 						((__vo uint32_t*)0XE000E18C)

#define	NVIC_PRIORITY_REG_BASE 						((__vo uint32_t*)0XE000E400)
#define	NUM_OF_PR_BITS_IMPLEMNTD 			4U

#define FLASH_BASEADDRESS 	0x08000000U
#define SRAM1_BASEADDRESS 	0x20000000U
#define SRAM2_BASEADDRESS 	0x20001C00U //  112KB from SRAM1 base address.
#define ROM_BASEADDRESS 	0x1FFF0000U //  System memory address is nothing but ROM


/* APB and AHB peripherals base address*/

#define PERIPH_BASE 		0x40000000U
#define APB1PERIPH_BASE 	PERIPH_BASE
#define APB2PERIPH_BASE 	0x40010000U
#define AHB1PERIPH_BASE 	0x40020000U
#define AHB2PERIPH_BASE 	0x50000000U



/*
 * Base addresses of peripherals hanging on AHB1 bus
 */

#define GPIOA_BASEADDRESS	(AHB1PERIPH_BASE + 0X0000)
#define GPIOB_BASEADDRESS	(AHB1PERIPH_BASE + 0X0400)
#define GPIOC_BASEADDRESS	(AHB1PERIPH_BASE + 0X0800)
#define GPIOD_BASEADDRESS	(AHB1PERIPH_BASE + 0X0C00)
#define GPIOE_BASEADDRESS	(AHB1PERIPH_BASE + 0X1000)
#define GPIOF_BASEADDRESS	(AHB1PERIPH_BASE + 0X1400)
#define GPIOG_BASEADDRESS	(AHB1PERIPH_BASE + 0X1800)
#define GPIOH_BASEADDRESS	(AHB1PERIPH_BASE + 0X1C00)
#define GPIOI_BASEADDRESS	(AHB1PERIPH_BASE + 0X2000)
#define GPIOJ_BASEADDRESS	(AHB1PERIPH_BASE + 0X2400)
#define GPIOK_BASEADDRESS	(AHB1PERIPH_BASE + 0X2800)

/*
 * Base addresses of peripherals hanging on APB1 bus
 */

#define I2C1_BASEADDRESS	(APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDRESS	(APB1PERIPH_BASE + 0X5800)
#define I2C3_BASEADDRESS	(APB1PERIPH_BASE + 0X5C00)
#define CAN1_BASEADDRESS	(APB1PERIPH_BASE + 0X6400)
#define CAN2_BASEADDRESS	(APB1PERIPH_BASE + 0X6800)
#define SPI2_BASEADDRESS	(APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDRESS	(APB1PERIPH_BASE + 0X3C00)
#define USART2_BASEADDRESS	(APB1PERIPH_BASE + 0X4400)
#define USART3_BASEADDRESS	(APB1PERIPH_BASE + 0X4800)
#define UART4_BASEADDRESS	(APB1PERIPH_BASE + 0X4C00)
#define UART5_BASEADDRESS	(APB1PERIPH_BASE + 0X5000)
#define UART7_BASEADDRESS	(APB1PERIPH_BASE + 0X7800)
#define UART8_BASEADDRESS	(APB1PERIPH_BASE + 0X7C00)

/*
 * Base addresses of peripherals hanging on APB2 bus
 */

#define SPI1_BASEADDRESS	(APB2PERIPH_BASE + 0X3000)
#define USART1_BASEADDRESS	(APB2PERIPH_BASE + 0X1000)
#define USART6_BASEADDRESS	(APB2PERIPH_BASE + 0X1400)
#define EXTI_BASEADDRESS	(APB2PERIPH_BASE + 0X3C00)
#define SYSCFG_BASEADDRESS	(APB2PERIPH_BASE + 0X3800)

#define RCC_BASEADDRESS 	(AHB1PERIPH_BASE + 0X3800)

#define IRQ_NO_EXTI0	 	6U
#define IRQ_NO_EXTI1	 	7U
#define IRQ_NO_EXTI2	 	8U
#define IRQ_NO_EXTI3	 	9U
#define IRQ_NO_EXTI4	 	10U
#define IRQ_NO_EXTI5_9	 	23U
#define IRQ_NO_EXTI10_15	40U

#define NVIC_IRQ_PR_0	 	0U
#define NVIC_IRQ_PR_1	 	1U
#define NVIC_IRQ_PR_2	 	2U
#define NVIC_IRQ_PR_3	 	3U
#define NVIC_IRQ_PR_4	 	4U
#define NVIC_IRQ_PR_5	 	5U
#define NVIC_IRQ_PR_6	 	6U
#define NVIC_IRQ_PR_7	 	7U
#define NVIC_IRQ_PR_8	 	8U
#define NVIC_IRQ_PR_9	 	9U
#define NVIC_IRQ_PR_10	 	10U
#define NVIC_IRQ_PR_11	 	11U
#define NVIC_IRQ_PR_12	 	12U
#define NVIC_IRQ_PR_13	 	13U
#define NVIC_IRQ_PR_14	 	14U
#define NVIC_IRQ_PR_15	 	15U




/******************************peripheral registers definition structures*************************/
/*
 * Register definitions are specific to Microcontroller. Please check the device before defining the strucyure.
 */

typedef struct
{
	__vo uint32_t MODER;		/* !<GPIO Mode register offset: 0x00*/
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;			/* !<GPIO Alternate Mode register*/
}GPIO_Register_Def_T;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t PCIRUPDR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_Register_Def_T;

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_Register_Def_T;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR1;
	__vo uint32_t EXTICR2;
	__vo uint32_t EXTICR3;
	__vo uint32_t EXTICR4;
	__vo uint32_t RESERVED1;
	__vo uint32_t RESERVED2;
	__vo uint32_t CMPCR;
}SYSCFG_Register_Def_T;



/*
 * Peripheral definition ( base address type casted to xxx_Reg_Def_T)
 */

#define GPIOA	((GPIO_Register_Def_T *)GPIOA_BASEADDRESS)
#define GPIOB	((GPIO_Register_Def_T *)GPIOB_BASEADDRESS)
#define GPIOC	((GPIO_Register_Def_T *)GPIOC_BASEADDRESS)
#define GPIOD	((GPIO_Register_Def_T *)GPIOD_BASEADDRESS)
#define GPIOE	((GPIO_Register_Def_T *)GPIOE_BASEADDRESS)
#define GPIOF	((GPIO_Register_Def_T *)GPIOF_BASEADDRESS)
#define GPIOG	((GPIO_Register_Def_T *)GPIOG_BASEADDRESS)
#define GPIOH	((GPIO_Register_Def_T *)GPIOH_BASEADDRESS)
#define GPIOI	((GPIO_Register_Def_T *)GPIOI_BASEADDRESS)
#define GPIOJ	((GPIO_Register_Def_T *)GPIOJ_BASEADDRESS)
#define GPIOK	((GPIO_Register_Def_T *)GPIOK_BASEADDRESS)

#define RCC		((RCC_Register_Def_T *)RCC_BASEADDRESS)
#define EXTI	((EXTI_Register_Def_T *)EXTI_BASEADDRESS)
#define SYSCFG	((SYSCFG_Register_Def_T *)SYSCFG_BASEADDRESS)

/*
 * GPIO Clock Enable Macros
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()			(RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()			(RCC->AHB1ENR |= (1<<10))

/*
 * SPI Clock Enable Macros
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))

/*
 * I2C Clock Enable Macros
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1<<23))

/*
 * USART Clock Enable Macros
 */
#define USART1_PCLK_EN()		(RCC->APB1ENR |= (1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define USART6_PCLK_EN()		(RCC->APB1ENR |= (1<<5))

/*
 * UART Clock Enable Macros
 */
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1<<20))

/*
 * SYSCFG Clock Enable Macros
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))

/*
 * GPIO Clock Disable Macros
 */

#define GPIOA_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	0))
#define GPIOB_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	1))
#define GPIOC_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	2))
#define GPIOD_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	3))
#define GPIOE_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	4))
#define GPIOF_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	5))
#define GPIOG_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	6))
#define GPIOH_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	7))
#define GPIOI_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	8))
#define GPIOJ_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	9))
#define GPIOK_PCLK_DS()			(RCC->AHB1ENR |= (0	<<	10))

/*
 * SPI Clock Disable Macros
 */
#define SPI1_PCLK_DS()			(RCC->APB2ENR |= (0	<<	12))
#define SPI2_PCLK_DS()			(RCC->APB1ENR |= (0	<<	14))
#define SPI3_PCLK_DS()			(RCC->APB1ENR |= (0	<<	15))

/*
 * I2C Clock Disable Macros
 */
#define I2C1_PCLK_DS()			(RCC->APB1ENR |= (0	<<	21))
#define I2C2_PCLK_DS()			(RCC->APB1ENR |= (0	<<	22))
#define I2C3_PCLK_DS()			(RCC->APB1ENR |= (0	<<	23))

/*
 * USART Clock Disable Macros
 */
#define USART1_PCLK_DS()		(RCC->APB1ENR |= (0	<<	4))
#define USART2_PCLK_DS()		(RCC->APB1ENR |= (0	<<	17))
#define USART3_PCLK_DS()		(RCC->APB1ENR |= (0	<<	18))
#define USART6_PCLK_DS()		(RCC->APB1ENR |= (0	<<	5))

/*
 * UART Clock Disable Macros
 */
#define UART4_PCLK_DS()			(RCC->APB1ENR |= (0	<<	19))
#define UART5_PCLK_DS()			(RCC->APB1ENR |= (0	<<	20))

/*
 * SYSCFG Clock Disable Macros
 */
#define SYSCFG_PCLK_DS()		(RCC->APB2ENR |= (0	<<	14))

/*
 * SYSCFG Clock Disable Macros
 */
#define GPIO_BASEADDRESS_TO_PORT_CODE(x)	(	(x == GPIOA)?0:\
												(x == GPIOB)?1:\
												(x == GPIOC)?2:\
												(x == GPIOD)?3:\
												(x == GPIOE)?4:\
												(x == GPIOF)?5:\
												(x == GPIOG)?6:\
												(x == GPIOH)?7:\
												(x == GPIOI)?8: 0)


#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));} while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));} while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));} while(0)
#define GPIOI_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));} while(0)
#define GPIOJ_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<9)); (RCC->AHB1RSTR &= ~(1<<9));} while(0)
#define GPIOK_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<10)); (RCC->AHB1RSTR &= ~(1<<10));} while(0)

#endif /* INC_STM32F407XX_H_ */
