/*
 * pencil_lib.c
 *
 *  Created on: Sep 24, 2024
 *      Author: Dhiru
 */
#include "pencil_lib.h"


void Machine_Status()
{
    if (Diagnostics_PASS())
    {
        currentState = HOPPER;
    }
    else
    {
        currentState = FAULT_TRIGGER;
    }
}

bool Diagnostics_PASS()
{
    if (!HAL_GPIO_ReadPin(GPIOG, HOPPER_SENSOR_Pin) && !HAL_GPIO_ReadPin(GPIOG, PENCIL_COUNTER_SENSOR_Pin) && !HAL_GPIO_ReadPin(GPIOG, EXTRA_SENSOR_Pin))
    {
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_1_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_2_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_3_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_4_Pin, GPIO_PIN_SET);
    	HAL_Delay(100);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_1_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_2_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_3_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_4_Pin, GPIO_PIN_RESET);
    	HAL_Delay(100);
    	return true;
    }
    else
    {
    	return false;
    }
}

void Hopper()
{
    if (Check_Hopper_Empty())
    {
    	currentState = FAULT_TRIGGER;
    }
    else
    {
    	HAL_GPIO_WritePin(GPIOC, DC_DIR_Pin, GPIO_PIN_SET);
    	for(uint8_t i=0 ; i<=200; i++)
    	{
    		HAL_GPIO_WritePin(GPIOC, DC_PULSE_Pin, GPIO_PIN_SET);
    		HAL_Delay(1);
    		HAL_GPIO_WritePin(GPIOC, DC_PULSE_Pin, GPIO_PIN_RESET);
    		HAL_Delay(1);
    	}
    	currentState = ROTARY_ACTUATOR;
    }
}

bool Check_Hopper_Empty()
{
	if (HAL_GPIO_ReadPin(GPIOG, HOPPER_SENSOR_Pin))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Rotary_Actuator()
{
	if(pencilCount < FIXED_VALUE)
	{
		pencilCount = 0;
		currentState = COUNTER_BOX;
	}
}

void Counter_Box()
{
    if (pencilCount == FIXED_VALUE)
    {
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_1_Pin, GPIO_PIN_SET);
    	HAL_Delay(1000);
    	HAL_GPIO_WritePin(GPIOA, ACTUATOR_1_Pin, GPIO_PIN_RESET);
        currentState = CONVEYOR_BELT;
    }
    else if(HAL_GPIO_ReadPin(GPIOG, PENCIL_COUNTER_SENSOR_Pin) == 1)
    {
    	pencilCount++;
    }
}

void Conveyor_Belt()
{
	HAL_GPIO_WritePin(GPIOC, CONVEYOR1_DIR_Pin, GPIO_PIN_SET);
	for(uint8_t i=0 ; i<=800; i++)
	{
		HAL_GPIO_WritePin(GPIOC, CONVEYOR_PULSE_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, CONVEYOR_PULSE_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
	currentState = WEIGHING_MACHINE;
}

void Weighing_Machine(uint16_t adc_read) {
    // Simulated weighing logic
    calWeight = pencilCount * adc_read;
    if (((calWeight-5) <= realWeight) &&  ((calWeight+5) >= realWeight-5))
    {
        currentState = PACKING_CONVEYOR;
    }
    else
    {
        currentState = WASTE_CONTAINER;
    }
}

void Packing_Conveyor()
{
	HAL_GPIO_WritePin(GPIOA, ACTUATOR_2_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, ACTUATOR_2_Pin, GPIO_PIN_RESET);
    currentState = FINAL_CONTAINER;
}

void Final_Container()
{
	HAL_GPIO_WritePin(GPIOC, CONVEYOR2_DIR_Pin, GPIO_PIN_SET);
	for(uint8_t i=0 ; i<=800; i++)
	{
		HAL_GPIO_WritePin(GPIOC, CONVEYOR2_PULSE_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, CONVEYOR2_PULSE_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
    currentState = MACHINE_STATUS; // Restart the process
}

uint8_t Fault_Trigger()
{
    currentState = MACHINE_STATUS; // Return to diagnostics
    return 1;
}

void Waste_Container()
{
	HAL_GPIO_WritePin(GPIOA, ACTUATOR_3_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, ACTUATOR_3_Pin, GPIO_PIN_RESET);
	currentState = HOPPER; // Return to diagnostics
}



