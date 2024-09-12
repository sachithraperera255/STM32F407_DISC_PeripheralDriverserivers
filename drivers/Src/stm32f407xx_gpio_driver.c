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
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();

		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();

		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();

		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();

		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();

		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();

		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();

		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();

		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();

		}

	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();

		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();

		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();

		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();

		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();

		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();

		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();

		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();

		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();

		}

	}
}

/*
 * Init and DeInit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;	//TEMP. REGISTER

	//1. configure the mode of the gpio pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing the register
		pGPIOHandle->pGPIOx->MODER |= temp; // setting the register

	} else {

		// This part will code later
	}
	temp = 0;

	//2. configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing the register
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pull and pull down settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing the register
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the output type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing the register
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the ALT functionality

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {

		//configure the alt function
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));

		pGPIOHandle->pGPIOx->AFR[temp1] |=
				pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << (4 * temp2);
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();

	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();

	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();

	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();

	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();

	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();

	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();

	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();

	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();

	}

}


/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	uint8_t value;
	value = (uint8_t )((pGPIOx->IDR >> PinNumber) &  0x00000001);

	return value;

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;
	value = (uint16_t )pGPIOx->IDR;

	return value;

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
