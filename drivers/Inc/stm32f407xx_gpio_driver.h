/*
 * stm32f407xx_gpio_driver.h
 *
 * THIS HEADER FILE CONTAIN DRIVER SPECIFIC DATA
 *
 *  Created on: Aug 14, 2024
 *      Author: sachithra
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"



/*
 * This is the Configuration Structure for GPIO pin
 */

typedef struct
{
	// We have used uint8_t instead of uint32_t because 1 byte is sufficient here, since pin numbers from 0 - 15
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunction;

}GPIO_PinConfig_t;





/*
 * This is the Handle Structure for GPIO pin
 */

typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;						/*|< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;			/*|< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;







/******************************************************************************************
 * 								APIs supported by this driver
 * 					For more information about the APIs check the function definitions
 ******************************************************************************************
 */



/*
 * Peripheral Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);



/*
 * Init and DeInit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);



/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
