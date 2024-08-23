/*
 * stm32f407xx_gpio_driver.c
 *
 *	APIs definitions/implementation
 *
 *  Created on: Aug 14, 2024
 *      Author: sachithra
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*************************************************
 * @fn			- GPIO_PeriClockControl.
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[in]	- Base address of the gpio peripheral.
 * @param[in]	- Enable or Disable macros.
 *
 *
 * @return 		- none
 *
 * @Note		- none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == pGPIOA) {
			GPIOA_PCLK_EN();

		} else if (pGPIOx == pGPIOB) {
			GPIOB_PCLK_EN();

		} else if (pGPIOx == pGPIOC) {
			GPIOC_PCLK_EN();

		} else if (pGPIOx == pGPIOD) {
			GPIOD_PCLK_EN();

		} else if (pGPIOx == pGPIOE) {
			GPIOE_PCLK_EN();

		} else if (pGPIOx == pGPIOF) {
			GPIOF_PCLK_EN();

		} else if (pGPIOx == pGPIOG) {
			GPIOG_PCLK_EN();

		} else if (pGPIOx == pGPIOH) {
			GPIOH_PCLK_EN();

		} else if (pGPIOx == pGPIOI) {
			GPIOI_PCLK_EN();

		}

	} else {
		if (pGPIOx == pGPIOA) {
			GPIOA_PCLK_DI();

		} else if (pGPIOx == pGPIOB) {
			GPIOB_PCLK_DI();

		} else if (pGPIOx == pGPIOC) {
			GPIOC_PCLK_DI();

		} else if (pGPIOx == pGPIOD) {
			GPIOD_PCLK_DI();

		} else if (pGPIOx == pGPIOE) {
			GPIOE_PCLK_DI();

		} else if (pGPIOx == pGPIOF) {
			GPIOF_PCLK_DI();

		} else if (pGPIOx == pGPIOG) {
			GPIOG_PCLK_DI();

		} else if (pGPIOx == pGPIOH) {
			GPIOH_PCLK_DI();

		} else if (pGPIOx == pGPIOI) {
			GPIOI_PCLK_DI();

		}

	}
}

/*
 * Init and DeInit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	//1. configure the mode of the gpio pin

	//2. configure the speed

	//3. configure the pull and pull down settings

	//4. configure the out put type

	//5. configure the alt functionality

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

}

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

}

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}

void GPIO_IRQHandling(uint8_t PinNumber) {

}
