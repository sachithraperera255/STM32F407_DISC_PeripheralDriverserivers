/*
 * stm32f407xx.h
 *
 * THIS IS MCU HEADER FILE WHICH CONTAIN MCU SPECIFIC DATA
 *
 *  Created on: Aug 9, 2024
 *      Author: Sachithra
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include<stdint.h>

#define __vo volatile



/*
 * base addresses of Flash and SRAM memory
 */

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U
#define ROM_BASEADDR				0x1FFF0000U // Also known as system memory base addr.
#define SRAM 						SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */


#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
//#define GPIOJ_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2400)
//#define GPIOK_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 */


#define I2C1_BASEADDR				(APB1PERTPH_BASEADDR + 0X5400)
#define I2C2_BASEADDR				(APB1PERTPH_BASEADDR + 0X5800)
#define I2C3_BASEADDR				(APB1PERTPH_BASEADDR + 0X5C00)

#define SPI2_BASEADDR				(APB1PERTPH_BASEADDR + 0X3800)
#define SPI3_BASEADDR				(APB1PERTPH_BASEADDR + 0X3C00)

#define USART2_BASEADDR				(APB1PERTPH_BASEADDR + 0X4400)
#define USART3_BASEADDR				(APB1PERTPH_BASEADDR + 0X4800)

#define UART4_BASEADDR				(APB1PERTPH_BASEADDR + 0X4C00)
#define UART5_BASEADDR				(APB1PERTPH_BASEADDR + 0X5000)




/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 */


#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0X3000)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0X1400)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0X3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0X3800)


/********************************************
 * peripheral register definition structures
 * ******************************************
*/


/*
 *	NOTE: Register of a peripheral are specific to MCU
 *	e.g : Number of registers of SPI peripheral of STM32fXX family of MCUs may be different(more or less)
 *	Please check your Device reference manual
 */


typedef struct{

	__vo uint32_t MODER;			/*|< GPIO port mode register  --> OFFSET = 0x00	 */
	__vo uint32_t OTYPER;			/*|< GPIO port output type register  --> OFFSET = 0x04	 */
	__vo uint32_t OSPEEDR;			/*|< GPIO port output speed register  --> OFFSET = 0x08	 */
	__vo uint32_t PUPDR;			/*|< GPIO port pull-up/pull-down register  --> OFFSET = 0x0C	 */
	__vo uint32_t IDR;				/*|< GPIO port input data register  --> OFFSET = 0x10	 */
	__vo uint32_t ODR;				/*|< GPIO port output data register  --> OFFSET = 0x14	 */
	__vo uint32_t BSRR;				/*|< GPIO port bit set/reset register  --> OFFSET = 0x18	 */
	__vo uint32_t LCKR;				/*|< GPIO port configuration lock register  --> OFFSET = 0x1C	 */
	__vo uint32_t AFR[2];			/*|< GPIO alternate function low register  --> OFFSET = 0x20  --> AFR[0]  && GPIO alternate function high register  --> OFFSET = 0x24  --> AFR[1]	 */

}GPIO_RegDef_t;



typedef struct{

	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;				/*|< RESERVED0	OFFSET = 0X1C */
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];			/*|< RESERVED1	OFFSET = 0X28 --> RESERVED[0]	&&	OFFSET = 0X2C --> RESERVED[1]*/
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED2[2];			/*|< RESERVED2	OFFSET = 0X48 --> RESERVED[0]	&&	OFFSET = 0X4C --> RESERVED[1]*/
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED3[2];			/*|< RESERVED3	OFFSET = 0X68 --> RESERVED[0]	&&	OFFSET = 0X6C --> RESERVED[1]*/
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED4[2];			/*|< RESERVED4	OFFSET = 0X78 --> RESERVED[0]	&&	OFFSET = 0X7C --> RESERVED[1]*/
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;




}RCC_RegDef_t;



/*
 *	peripheral definition (Peripheral base addresses type casted to XXX_RegDef_t )
 */


#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)
//#define GPIOJ			((GPIO_RegDef_t*)GPIOJ_BASEADDR)
//#define GPIOK			((GPIO_RegDef_t*)GPIOK_BASEADDR)


#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))


/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))

#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))


/*
 * Clock Enable Macros for USARTx/UARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2 |= (1 << 4))
#define USART6_PCLK_EN()	(RCC->APB2 |= (1 << 5))

#define USART2_PCLK_EN()	(RCC->APB1 |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1 |= (1 << 18))

#define UART4_PCLK_EN()		(RCC->APB1 |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1 |= (1 << 20))



/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2 |= (1 << 14))








/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))



/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21)



/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))

#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))




/*
 * Clock Disable Macros for USARTx/UARTx peripherals
 */

#define USART1_PCLK_DI()	(RCC->APB2 &= ~(1 << 4))
#define USART6_PCLK_DI()	(RCC->APB2 &= ~(1 << 5))

#define USART2_PCLK_DI()	(RCC->APB1 &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1 &= ~(1 << 18))

#define UART4_PCLK_DI()		(RCC->APB1 &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1 &= ~(1 << 20))




/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2 &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOC_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOD_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOE_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOF_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOG_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOH_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOI_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));} while(0)

/*
 * Some generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define set				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET



#endif /* INC_STM32F407XX_H_ */
