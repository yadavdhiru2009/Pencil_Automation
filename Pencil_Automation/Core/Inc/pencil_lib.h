/*
 * pencil_lib.h
 *
 *  Created on: Sep 24, 2024
 *      Author: Dhiru
 */

#ifndef INC_PENCIL_LIB_H_
#define INC_PENCIL_LIB_H_

#include "stdbool.h"
#include "stdint.h"
#include "main.h"

#define DC_DIR_Pin GPIO_PIN_0
#define DC_DIR_GPIO_Port GPIOC
#define DC_PULSE_Pin GPIO_PIN_1
#define DC_PULSE_GPIO_Port GPIOC
#define CONVEYOR1_DIR_Pin GPIO_PIN_2
#define CONVEYOR1_DIR_GPIO_Port GPIOC
#define CONVEYOR_PULSE_Pin GPIO_PIN_3
#define CONVEYOR_PULSE_GPIO_Port GPIOC
#define ACTUATOR_1_Pin GPIO_PIN_0
#define ACTUATOR_1_GPIO_Port GPIOA
#define ACTUATOR_2_Pin GPIO_PIN_1
#define ACTUATOR_2_GPIO_Port GPIOA
#define ACTUATOR_3_Pin GPIO_PIN_2
#define ACTUATOR_3_GPIO_Port GPIOA
#define ACTUATOR_4_Pin GPIO_PIN_3
#define ACTUATOR_4_GPIO_Port GPIOA
#define CONVEYOR2_DIR_Pin GPIO_PIN_4
#define CONVEYOR2_DIR_GPIO_Port GPIOC
#define CONVEYOR2_PULSE_Pin GPIO_PIN_5
#define CONVEYOR2_PULSE_GPIO_Port GPIOC
#define HOPPER_SENSOR_Pin GPIO_PIN_0
#define HOPPER_SENSOR_GPIO_Port GPIOG
#define PENCIL_COUNTER_SENSOR_Pin GPIO_PIN_1
#define PENCIL_COUNTER_SENSOR_GPIO_Port GPIOG
#define EXTRA_SENSOR_Pin GPIO_PIN_2
#define EXTRA_SENSOR_GPIO_Port GPIOG
#define SYSTEM_PAUSE_Pin GPIO_PIN_0
#define SYSTEM_PAUSE_GPIO_Port GPIOE


#define FIXED_VALUE 12
#define MAX_HOPPER_LIMIT 2000
#define MIN_HOPPER_LIMIT 1000

typedef enum {
    MACHINE_STATUS,
    HOPPER,
    ROTARY_ACTUATOR,
    COUNTER_BOX,
    CONVEYOR_BELT,
    WEIGHING_MACHINE,
    PACKING_CONVEYOR,
    FINAL_CONTAINER,
    FAULT_TRIGGER,
    WASTE_CONTAINER
} State;

State currentState = MACHINE_STATUS;

int pencilCount = 0;
int realWeight = 60; // Assume 60[5*12] value is known and set elsewhere
int calWeight = 0;

void Machine_Status();
void Hopper();
void Rotary_Actuator();
void Counter_Box();
void Conveyor_Belt();
void Weighing_Machine(uint16_t adc_read);
void Packing_Conveyor();
void Final_Container();
uint8_t Fault_Trigger();
void Waste_Container();
bool Diagnostics_PASS();
bool Check_Hopper_Empty();
void Fill_Hopper();
bool Status_True();
bool Count_Reach_Fixed_Value();
bool Counted_Equals_Fixed();
bool Cal_Weight_Equals_Real_Weight();
bool Container_Full();
void Trigger_Buzzer();
void Display_Fault();
void Push_To_Waste_Container();
//void pencil_handler_init(ADC_HandleTypeDef *hadc1, I2C_HandleTypeDef *hi2c1);
//bool i2c_write_data(char *str);
//uint8_t read_adc();
#endif /* INC_PENCIL_LIB_H_ */
