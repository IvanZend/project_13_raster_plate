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


/*
********************************************************************************
* 								DEFINE CONSTANTS
********************************************************************************
*/

#define SIGNALS_CHECK_TIMER_POINTER 				&htim2
#define MOTOR_TIMER_POINTER 						&htim3
#define UART_MESSAGE_SIZE							8
#define BUTTON_RELEASED_SIGNAL_LEVEL_DEFAULT 		LOGIC_LEVEL_LOW
#define BUTTON_BOUNCE_FILTER_COUNTS_DEFAULT 		5
#define BUTTON_LONG_PRESS_COUNTS_DEFAULT 			20
#define LIMIT_SWTICH_LOGIC_INVERTED_DEFAULT 		0
#define EMERGENCY_STEP_IMPULSES_TO_LIMIT_DEFAULT 	10000
#define STEP_IMPULSES_DEFAULT						4000
#define DIR_PIN_LOGIC_LEVEL_INVERTED_DEFAULT 		1
#define MOTOR_TIMER_TICKS_PER_MS_DEFUALT 			200
#define ACCELERATION_TYPE_DEFAULT 					LINEAR_ACCELERATION
#define SHORT_DISTANCE_STEP_IMPULSES_DEFAULT 		100
#define FAR_DISTANCE_STEP_IMPULSES_DEFAULT 			1826
#define MIN_SPEED_STEP_PER_MS_DEFAULT 				3.138
#define MAX_SPEED_STEP_PER_MS_DEFAULT 				7.746
#define LINEAR_ACCELERATION_COEFFICIENT_DEFAULT 	0
#define QUADRATIC_ACCELERATION_COEFFICIENT_DEFAULT 	0
#define ACCELERATION_DURATION_MS_DEFAULT 			50

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
	NO_ACCELERATION,
	LINEAR_ACCELERATION,
	QUADRATIC_ACCELERATION

} MotorAccelerationType_EnumTypeDef;

typedef enum
{
	BUTTON_RELEASED,
	BUTTON_SHORT_PRESS,
	BUTTON_LONG_PRESS

} ButtonState_EnumTypeDef;


typedef struct
{
	GPIO_TypeDef* GPIO_port_pointer;
	uint16_t pin_number;

} PinAttributes_StructTypeDef;

typedef struct
{
	PinAttributes_StructTypeDef signal_pin;
	SignalLogicLevel_EnumTypeDef signal_logic_level;

} InSignalAttributes_StructTypeDef;

typedef struct
{
	InSignalAttributes_StructTypeDef button_signal;
	SignalLogicLevel_EnumTypeDef button_released_default_signal_level;
	uint32_t button_bounce_filter_counts;
	uint32_t button_long_press_ms;

	ButtonState_EnumTypeDef button_current_state;
	uint32_t button_pressing_duration_counter;

} ButtonAttributes_StructTypeDef;

/*
 * Концевик
 */
typedef struct
{
	InSignalAttributes_StructTypeDef limit_switch_IN_signal;
	_Bool limit_switch_logic_inverted;

} LimitSwitch_StructTypeDef;

typedef struct
{
	InSignalAttributes_StructTypeDef STEP_OUT_signal;
	InSignalAttributes_StructTypeDef DIR_OUT_signal;
	InSignalAttributes_StructTypeDef ENABLE_OUT_signal;

} MotorMovementSignals_StructTypeDef;

/*
 * Профиль движения. Создается независимо от класса мотора, не привязан к аппаратной конфигурации. Можно создать любое количество профилей.
 */
typedef struct
{
	MotorAccelerationType_EnumTypeDef acceleration_type;
	int32_t short_distance_step_impulses;
	int32_t far_distance_step_impulses;
	float min_speed_step_per_ms;
	float max_speed_step_per_ms;
	float linear_acceleration_coefficient;
	uint32_t quadratic_acceleration_coefficient;
	uint32_t acceleration_duration_ms;

} MotorMovementProfile_StructTypeDef;

/*
 * Класс мотора. Включает в себя конфигурацию мотора и концевика, связанного с ним.
 */
typedef struct
{
	/*
	 * Неизменные величины, задаются один раз при старте программы
	 */
	LimitSwitch_StructTypeDef limit_switch;
	MotorMovementSignals_StructTypeDef motor_signals;
	int32_t step_impulses_acceptable_error;
	int32_t emergency_step_impulses_to_limit;
	_Bool DIR_pin_logic_level_inverted;
	float motor_timer_ticks_per_ms;

	/*
	 * Величины, изменяемые по ходу движения
	 */
	int32_t step_impulses_distance_from_limit_switch;
	int32_t limit_emergency_counter;
	MotorMoveDirection_EnumTypeDef motor_movement_direction;		// направление движения мотора
	StepPinPhase_EnumTypeDef step_pin_current_phase;				// текущее логическое состояние пина шага
	int32_t ticks_before_next_step_counter;							// ??? уточнить размер переменной
	uint64_t ticks_since_start_movement_counter;
	uint32_t step_impulses_since_start_movement_counter;
	float current_speed_step_per_ms;
	uint32_t ticks_for_acceleration_counter;
	uint32_t step_impulses_for_acceleration_counter;
	MotorMoveDirection_EnumTypeDef cyclic_movement_direction;

} MotorObject_StructTypeDef;

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
void set_output_signal_state(GPIO_TypeDef* GPIO_port_pointer, uint16_t pin_number, SignalLogicLevel_EnumTypeDef requied_logic_level);
void check_input_signal_state(InSignalAttributes_StructTypeDef* signal_to_check);
void button_init(ButtonAttributes_StructTypeDef* button_object);
void check_and_update_button_state(ButtonAttributes_StructTypeDef* button_to_check, uint32_t ticks_per_sec);
void motor_init(MotorObject_StructTypeDef* motor_object);
void movement_profile_init(MotorMovementProfile_StructTypeDef* movement_profile);
void motor_movement_init(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile);
void calculate_acceleration_coefficient(MotorMovementProfile_StructTypeDef* movement_profile);
void cyclic_movement_step(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile);
void reset_movement_counters(MotorObject_StructTypeDef* motor_object);
void motor_check_counter_and_make_step_to_direction(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile, MotorMoveDirection_EnumTypeDef movement_direction);
void motor_direction_pin_set(MotorObject_StructTypeDef* motor_object);
void check_limit_switch_and_make_step(MotorObject_StructTypeDef* motor_object);
void calculate_ticks_per_next_step(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile);
float movement_time_function(uint32_t ticks_value, MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile);
_Bool limit_switch_active(MotorObject_StructTypeDef* motor_object);
void check_input_signal_state(InSignalAttributes_StructTypeDef* signal_to_check);
void step_toggle(MotorObject_StructTypeDef* motor_object);
uint32_t convert_ms_to_ticks(uint32_t milliseconds, uint32_t ticks_per_ms);
extern void bucky_ready_enable(void);

#endif /* INC_USER_FILE_11_H_ */
