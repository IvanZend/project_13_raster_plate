/*
********************************************************************************
* S.P. Helpic Ltd
* 117997, Profsoyuznaya st.86 b.2, Moscow, Russia
* (c) Copyright 2020, S.P. Helpic Ltd, Moscow, Russia
*
* Fiename       : user_file_13.h
* Programmer(s) : Ivan Bikashov (IB)
* Created       : 2020/09/25
* Description   : Bucky plate firmware.
*
********************************************************************************
*/

#ifndef INC_USER_FILE_11_H_
#define INC_USER_FILE_11_H_

/*
********************************************************************************
* 								INCLUDE
********************************************************************************
*/

#include "universal_extern_lib_1.h"

/*
********************************************************************************
* 								DEFINE CONSTANTS
********************************************************************************
*/

#define SIGNALS_CHECK_TIMER_POINTER 	&htim2
#define MOTOR_TIMER_POINTER 			&htim3
#define UART_MESSAGE_SIZE				8

/*
********************************************************************************
*								GLOBAL MACROS
********************************************************************************
*/

/*
********************************************************************************
*								GLOBAL DATA TYPES
********************************************************************************
*/

typedef enum
{
	LIMIT_SWITCH_ENABLED,
	LIMIT_SWITCH_DISABLED

} LimitSwitchState_EnumTypeDef;

typedef enum
{
	GRID_NOT_REPRESENTED,
	GRID_TYPE_120,
	GRID_TYPE_180

} GridState_EnumTypeDef;

typedef enum
{
	DEVICE_STARTS,
	DEVICE_INITIAL_MOVEMENT,
	DEVICE_STANDBY,
	DEVICE_GRID_SUPPLY,
	DEVICE_BUCKYBRAKE,
	DEVICE_SCANING_TOMO_OFF,
	DEVICE_SCANING_TOMO_ON,
	DEVICE_RETURN_TO_INITIAL_STATE,
	DEVICE_ERROR

} DeviceState_EnumTypeDef;


typedef enum
{
	MOTOR_PURPOSE_TAKE_INITIAL_POSITION,
	MOTOR_PURPOSE_INITIAL_MOVEMENT,
	MOTOR_PURPOSE_GRID_INSERTION,
	MOTOR_PURPOSE_GRID_EXTRACTION,
	MOTOR_PURPOSE_EXPOSITION_TOMO_OFF,
	MOTOR_PURPOSE_EXPOSITION_TOMO_ON,
	MOTOR_PURPOSE_EMERGENCY_SUPPLY

} MotorMovementPurpose_EnumTypeDef;

typedef enum
{
	MOTOR_MOVEMENT_IN_PROGRESS,
	MOTOR_MOVEMENT_COMPLETED

} MotorMovementStatus_EnumTypeDef;

typedef enum
{
	ON_TOMO_WAS_NOT_ENABLED,
	ON_TOMO_WAS_ENABLED,
	ON_TOMO_WAS_ENABLED_AND_DISABLED

} OnTomoSignalFlag_EnumTypeDef;

typedef enum
{
	EXPOSITION_MOVEMENT_FROM_INITIAL_POSITION,
	ON_TOMO_MOVEMENT_TO_INITIAL_POSITION

} OnTomoMovementDirectionFlag_EnumTypeDef;

typedef enum
{
	NO_ERROR,							// ошибки нет
	GRID_TYPE_ERROR,					// определены одновременно два типа растра
	LIMIT_SWITCH_ERROR,					// концевик не сработал
	STANDBY_MOVEMENT_ERROR,				// происходит движение во время режима ожидания
	ON_TOMO_BUCKY_CALL_ERROR			// в режиме ON_TOMO сигнал BUCKY_CALL выключился прежде, чем сигнал ON_TOMO

} ErrorCode_EnumTypeDef;

typedef enum
{
	BUCKY_READY_COUNTER_INCREMENTING,
	BUCKY_READY_COUNNTER_DISABLED

} BuckyReadyFlag_EnumTypeDef;

/*
********************************************************************************
*								CLASSES AND OBJECTS
********************************************************************************
*/

/*
**********************************************************
*   Уровень 3. Классы сигналов на выводах контроллера
**********************************************************
*/

/*
 * Датчик типа растра
 */
typedef struct
{
	InSignalAttributes_StructTypeDef GRID_120_DETECT_IN_signal;
	InSignalAttributes_StructTypeDef GRID_180_DETECT_IN_signal;

} GridSensor_TypeDef;

/*
 * DIP-переключатель
 */
typedef struct
{
	InSignalAttributes_StructTypeDef DIP_SWITCH_1_IN_signal;
	InSignalAttributes_StructTypeDef DIP_SWITCH_2_IN_signal;
	InSignalAttributes_StructTypeDef DIP_SWITCH_3_IN_signal;

} DIPSwitch_TypeDef;

/*
********************************************************************************
*								GLOBAL VARIABLES
********************************************************************************
*/

_Bool led_blink_enabled;
uint32_t led_blink_counter;
_Bool limit_switch_enabled_once;
DeviceState_EnumTypeDef device_current_state;
ErrorCode_EnumTypeDef error_code;
GridSensor_TypeDef grid_sensor;
ButtonAttributes_StructTypeDef grid_supply_button;
InSignalAttributes_StructTypeDef ON_TOMO_IN_signal;
InSignalAttributes_StructTypeDef BUCKY_CALL_IN_signal;
OnTomoSignalFlag_EnumTypeDef ON_TOMO_IN_flag;
uint8_t bucky_ready_delay_counter;
ButtonAttributes_StructTypeDef pushbutton_buckybrake;
DIPSwitch_TypeDef DIP_switch;
MotorMovementPurpose_EnumTypeDef motor_movement_purpose;
MotorMovementStatus_EnumTypeDef motor_movement_status;
OnTomoMovementDirectionFlag_EnumTypeDef exposition_movement_direction;

MotorObject_StructTypeDef motor_instance_1;
MotorMovementProfile_StructTypeDef movement_profile_1_default;
MotorMovementProfile_StructTypeDef movement_profile_2_exposition;
MotorMovementProfile_StructTypeDef movement_profile_3_supply;

/*
********************************************************************************
*								EXTERNALS
********************************************************************************
*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


void device_init(void);
void input_pins_init(void);
void device_modules_init(void);
void check_input_signals(void);
void dip_switch_state_update(void);
void input_signals_state_update(void);
void output_signals_state_init(SignalLogicLevel_EnumTypeDef signal_level_to_set);
void signals_check_timer_interrupts_start(void);
void signals_check_timer_interrupt_handler(void);
void buttons_state_update(void);
void enable_pin_set(void);
void enable_pin_clear(void);
void device_error_handler(void);
void read_input_signals_and_set_device_state(void);
void set_grid_out_signal(void);
void buckybreak_laser_disable(void);
void bucky_ready_delay_set(void);
void bucky_ready_enable(void);
void bucky_ready_dsable(void);
void motor_timer_interrupts_start(void);
void motor_timer_interrupts_stop(void);
void motor_movement_start(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile);
void motor_check_conditions_and_step(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile);
void motor_timer_interrupt_handler(void);
void led_toggle(void);
uint32_t ms_per_step(uint32_t ticks_per_sec);

#endif /* INC_USER_FILE_11_H_ */
