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

#include "main.h"

/*
********************************************************************************
* 								DEFINE CONSTANTS
********************************************************************************
*/

#define MOTOR_TIMER_POINTER 			&htim2
#define SIGNALS_CHECK_TIMER_POINTER 	&htim3
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
	MOVE_TO_COORD_ORIGIN,
	MOVE_TO_COORD_END

} MotorMoveDirection_EnumTypeDef;

typedef enum
{
	STEP_LOW_PHASE,
	STEP_HIGH_PHASE

} StepPinPhase_EnumTypeDef;

typedef enum
{
	LOGIC_LEVEL_LOW,
	LOGIC_LEVEL_HIGH

} SignalLogicLevel_EnumTypeDef;

typedef enum
{
	BUTTON_RELEASED,
	BUTTON_SHORT_PRESS,
	BUTTON_LONG_PRESS

} ButtonState_EnumTypeDef;

typedef enum
{
	GRID_NOT_REPRESENTED,
	GRID_TYPE_120,
	GRID_TYPE_180

} GridState_EnumTypeDef;

typedef enum
{
	DEVICE_STARTS,
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
	MOTOR_PURPOSE_GRID_INSERTION,
	MOTOR_PURPOSE_GRID_EXTRACTION,
	MOTOR_PURPOSE_EXPOSITION_TOMO_OFF,
	MOTOR_PURPOSE_EXPOSITION_TOMO_ON,
	MOTOR_PURPOSE_INSTANT_STOP

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

typedef enum
{
	ACCELERATION_MODE_00,				// 7577 шагов/сек
	ACCELERATION_MODE_01,				// 6839 шагов/сек
	ACCELERATION_MODE_10,				// 8188 шагов/сек
	ACCELERATION_MODE_11				// 9924 шагов/сек
} AccelerationMode_EnumTypeDef;

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

typedef struct
{
	GPIO_TypeDef* GPIO_port_pointer;
	uint16_t pin_number;

} PinAttributes_TypeDef;

typedef struct
{
	PinAttributes_TypeDef signal_pin;
	SignalLogicLevel_EnumTypeDef signal_logic_level;

} InSignalAttributes_TypeDef;

typedef struct
{
	InSignalAttributes_TypeDef button_signal;
	SignalLogicLevel_EnumTypeDef button_released_default_signal_level;
	ButtonState_EnumTypeDef button_current_state;
	uint32_t button_pressing_duration_counter;

} ButtonAttributes_TypeDef;

/*
 * Управление шаговым мотором (мотор, концевики, потенциометр)
 */
typedef struct
{
	int32_t steps_distance_from_limit_switch;
	int32_t limit_emergency_counter;
	MotorMoveDirection_EnumTypeDef motor_move_direction;			// направление движения мотора
	StepPinPhase_EnumTypeDef step_pin_current_phase;				// текущее логическое состояние пина шага
	MotorMovementPurpose_EnumTypeDef motor_movement_purpose;
	MotorMovementStatus_EnumTypeDef motor_movement_status;
	OnTomoMovementDirectionFlag_EnumTypeDef exposition_movement_direction;
	AccelerationMode_EnumTypeDef acceleration_mode;

} Motor_TypeDef;

/*
 * Датчик типа растра
 */
typedef struct
{
	InSignalAttributes_TypeDef GRID_120_DETECT_IN_signal;
	InSignalAttributes_TypeDef GRID_180_DETECT_IN_signal;

} GridSensor_TypeDef;


/*
 * Концевик
 */

typedef struct
{
	InSignalAttributes_TypeDef GRID_END_POINT_IN_signal;

} LimitSwitch_TypeDef;

/*
 * DIP-переключатель
 */

typedef struct
{
	InSignalAttributes_TypeDef DIP_SWITCH_1_IN_signal;
	InSignalAttributes_TypeDef DIP_SWITCH_2_IN_signal;
	InSignalAttributes_TypeDef DIP_SWITCH_3_IN_signal;

} DIPSwitch_TypeDef;

/*
********************************************************************************
*								GLOBAL VARIABLES
********************************************************************************
*/

DeviceState_EnumTypeDef device_current_state;
ErrorCode_EnumTypeDef error_code;
Motor_TypeDef motor;
GridSensor_TypeDef grid_sensor;
ButtonAttributes_TypeDef grid_supply_button;
InSignalAttributes_TypeDef ON_TOMO_IN_signal;
InSignalAttributes_TypeDef BUCKY_CALL_IN_signal;
OnTomoSignalFlag_EnumTypeDef ON_TOMO_IN_flag;
uint8_t bucky_ready_delay_counter;
ButtonAttributes_TypeDef pushbutton_buckybrake;
LimitSwitch_TypeDef limit_switch;
DIPSwitch_TypeDef DIP_switch;
int64_t ticks_before_next_step_counter;
uint64_t ticks_since_start_movement_counter;
uint32_t steps_for_acceleration_counter;
uint32_t steps_since_start_movement_counter;
uint64_t ticks_for_acceleration_counter;
uint64_t steps_per_sec;

/*
********************************************************************************
*								EXTERNALS
********************************************************************************
*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/*
********************************************************************************
*								GLOBAL FUNCTION PROTOTYPES
********************************************************************************
*/


/*
***************************************
*   Функции управления модулем Bucky
***************************************
*/

void device_init(void);
void pins_init (void);
void device_modules_init (void);
void check_input_signals(void);
void dip_switch_state_update(void);
void input_signals_state_update(void);
void output_signals_state_init(SignalLogicLevel_EnumTypeDef signal_level_to_set);
void signals_check_timer_interrupts_start(void);
void signals_check_timer_interrupt_handler(void);
void buttons_state_update(void);
void enable_pin_set(void);
void check_input_signal_state(InSignalAttributes_TypeDef* signal_to_check);
void check_button_state(ButtonAttributes_TypeDef* button_to_check);
void set_output_signal_state(GPIO_TypeDef* GPIO_port_pointer, uint16_t pin_number, SignalLogicLevel_EnumTypeDef requied_logic_level);
void device_error_check (void);
void device_error_handler(void);
void read_input_signals_and_set_device_state(void);
void bucky_ready_response_set(SignalLogicLevel_EnumTypeDef);
void dip_switch_value_decode(void);

/*
***************************************
*   Функции мотора
***************************************
*/

void check_limit_switch_and_make_step(void);
void motor_movement_start(void);
void reset_movement_counters(void);
uint64_t movement_time_function(uint64_t time_value);
void calculate_ticks_per_next_step(void);
void motor_check_conditions_and_step(void);
void motor_make_one_step(void);
void motor_make_step_to_direction(MotorMoveDirection_EnumTypeDef move_direction);
void cyclic_movement_step(void);
void motor_direction_pin_set(void);
void step_toggle(void);

/*
***************************************
*   Функции концевика
***************************************
*/
_Bool limit_switch_return_state(void);

/*
***************************************
*   Функции таймера
***************************************
*/

void motor_timer_interrupts_start(void);
void motor_timer_interrupts_stop(void);
void motor_timer_interrupt_handler(void);

#endif /* INC_USER_FILE_11_H_ */
