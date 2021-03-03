
#ifndef INC_UNIVERSAL_EXTERN_LIB_1_H_
#define INC_UNIVERSAL_EXTERN_LIB_1_H_

//#include "main.h"


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

#endif /* INC_UNIVERSAL_EXTERN_LIB_1_H_ */
