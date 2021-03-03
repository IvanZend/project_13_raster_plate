/*
********************************************************************************
* S.P. Helpic Ltd
* 117997, Profsoyuznaya st.86 b.2, Moscow, Russia
* (c) Copyright 2020, S.P. Helpic Ltd, Moscow, Russia
*
* Fiename       : user_file_13.c
* Programmer(s) : Ivan Bikashov (IB)
* Created       : 2020/09/25
* Description   : Bucky plate firmware.
*
********************************************************************************
*/

/*
********************************************************************************
* 								INCLUDE
********************************************************************************
*/

#include "main.h"
#include "stdio.h"

/*
********************************************************************************
*								DEFINE CONSTANTS
********************************************************************************
*/

#define FRONTS_PER_STEP								2
#define DIR_PIN_LOGIC_LEVEL_INVERTED				1			// инвертирован ли логический уровень направления (зависит от аппаратной конфигурации драйвера)
#define ENABLE_PIN_LOGIC_LEVEL_INVERTED				1
#define LIMIT_SWITCH_LOGIC_LEVEL_INVERTED			1			// если концевик при размокнутом состоянии выдаёт "1", выставляем флаг инверсии
#define RASTER_SUPPLY_DISTANCE_STEP_IMPULSES		1937		// расстояние от концевика, на которое растр выдвигается для подачи
#define EMERGENCY_STEP_IMPULSES_TO_LIMIT			3500		// максимальное расстояние, которое ШД может проехать до концевика. После него выполняем аварийное торможение.
#define BUTTON_BOUNCE_FILTER_COUNTS					0			// количество отсчетов, после которого решаем, что дребезг закончился и кнопка нажата
#define BUTTON_LONG_PRESS_DURATION_SEC				1			// количество миллисекунд, после которого фиксируем долгое нажатие кнопки
#define BUCKY_READY_DELAY_STEP_IMPULSES				3			// количество шагов, после которых растр разгоняется, и загорается сигнал BUCKY_READY
#define SIGNALS_CHECK_TIMER_TICKS_PER_SEC			10
#define MOTOR_TIMER_TICKS_PER_MS					200
#define STEP_IMPULSES_ACCEPTABLE_ERROR				100
#define STEP_IMPULSES_DISTANCE_INITIAL				EMERGENCY_STEP_IMPULSES_TO_LIMIT - STEP_IMPULSES_ACCEPTABLE_ERROR
#define SHORT_DISTANCE_STEP_IMPULSES		 		0	// 0
#define FAR_DISTANCE_STEP_IMPULSES 					1826
#define CONSTANT_SPEED_STEP_PER_MS					1.325 * FRONTS_PER_STEP
#define RASTER_SUPPLY_SPEED_STEP_PER_MS				1.985 * FRONTS_PER_STEP
#define MIN_SPEED_STEP_PER_MS_ALL_MODES 			1.569 * FRONTS_PER_STEP
#define MAX_SPEED_STEP_PER_MS_MODE_00		 		4.535 * FRONTS_PER_STEP
#define MAX_SPEED_STEP_PER_MS_MODE_01		 		3.969 * FRONTS_PER_STEP
#define MAX_SPEED_STEP_PER_MS_MODE_10		 		5.003 * FRONTS_PER_STEP
#define MAX_SPEED_STEP_PER_MS_MODE_11		 		7.129 * FRONTS_PER_STEP
#define ACCELERATION_DURATION_MS_MODE_00			40
#define ACCELERATION_DURATION_MS_MODE_01			32
#define ACCELERATION_DURATION_MS_MODE_10			43
#define ACCELERATION_DURATION_MS_MODE_11			52
#define LINEAR_ACCELERATION_COEFFICIENT_INITIAL 	0
#define QUADRATIC_ACCELERATION_COEFFICIENT_INITIAL 	0


/*
 * Определяем выходные пины, исходя из инициализации, созданной конфигуратором пинов
 */
#define MOTOR_ENABLE_OUT_PORT				GPIOB
#define MOTOR_ENABLE_OUT_PIN				ENABLE_Pin

#define MOTOR_STEP_OUT_PORT					GPIOB
#define MOTOR_STEP_OUT_PIN					STEP_Pin

#define MOTOR_DIR_OUT_PORT					GPIOB
#define MOTOR_DIR_OUT_PIN					DIR_Pin

#define MOTOR_RESET_OUT_PORT				GPIOB
#define MOTOR_RESET_OUT_PIN					RESET_Pin

#define MOTOR_CURRENT_WIND_OUT_PORT			GPIOB
#define MOTOR_CURRENT_WIND_OUT_PIN			CURRENT_WIND_Pin

#define LASER_CENTERING_OUT_PORT			GPIOB
#define LASER_CENTERING_OUT_PIN				LASER_CENTERING_Pin

#define BUCKYBRAKE_OUT_PORT					GPIOB
#define BUCKYBRAKE_OUT_PIN					BUCKY_BRAKE_Pin

#define BUCKY_READY_OUT_PORT				GPIOA
#define BUCKY_READY_OUT_PIN					BUCKY_READY_Pin

#define GRID_120_OUT_PORT					GPIOA
#define GRID_120_OUT_PIN					GRID_120_Pin

#define GRID_180_OUT_PORT					GPIOA
#define GRID_180_OUT_PIN					GRID_180_Pin

/*
********************************************************************************
*								GLOBAL FUNCTIONS
********************************************************************************
*/

/*
***************************************
*   Функции управления модулем Bucky
***************************************
*/

/*
 * Инициализация устройства
 */

void device_init(void)
{
	device_current_state = DEVICE_STARTS;						// выставляем состояние устройства: устройство стартует
	input_pins_init();											// инициализируем сигналы (указываем пины и порты, инициализируем единый массив сигналов)
	output_signals_state_init(LOGIC_LEVEL_HIGH);				// выставляем состояние выходных сигналов
	input_signals_state_update();								// считываем состояние входных сигналов
	device_modules_init();										// инициализируем аппаратные модули (кнопки, датчики, мотор, интерфейс А1, DIP-переключатели)
	buttons_state_update();										// обновляем состояние кнопок
	set_grid_out_signal();										// выставляем светодиоды датчика типа растра
	buckybreak_laser_disable();									// выключаем сигнал buckybreak и лазер
	dip_switch_state_update();									// проверка направления и скорости движения
	bucky_ready_dsable();
	error_code = NO_ERROR;										// выставляем отсутствие ошибки
	signals_check_timer_interrupts_start();						// запускаем таймер считывания состояний сигналов
}

void enable_pin_set(void)
{
	if (ENABLE_PIN_LOGIC_LEVEL_INVERTED)
	{
		set_output_signal_state(MOTOR_ENABLE_OUT_PORT, MOTOR_ENABLE_OUT_PIN, LOGIC_LEVEL_LOW);
	}
	else
	{
		set_output_signal_state(MOTOR_ENABLE_OUT_PORT, MOTOR_ENABLE_OUT_PIN, LOGIC_LEVEL_HIGH);
	}
}

void enable_pin_clear(void)
{
	if (ENABLE_PIN_LOGIC_LEVEL_INVERTED)
	{
		set_output_signal_state(MOTOR_ENABLE_OUT_PORT, MOTOR_ENABLE_OUT_PIN, LOGIC_LEVEL_HIGH);
	}
	else
	{
		set_output_signal_state(MOTOR_ENABLE_OUT_PORT, MOTOR_ENABLE_OUT_PIN, LOGIC_LEVEL_LOW);
	}
}

/*
 * Определяем входные пины, исходя из инициализации, созданной конфигуратором пинов
 */
void input_pins_init(void)
{
	grid_sensor.GRID_180_DETECT_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	grid_sensor.GRID_180_DETECT_IN_signal.signal_pin.pin_number = GRID_180_DETECT_Pin;					// пин датчика Холла (растр типа 180)

	grid_sensor.GRID_120_DETECT_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	grid_sensor.GRID_120_DETECT_IN_signal.signal_pin.pin_number = GRID_120_DETECT_Pin;					// пин датчика Холла (растр типа 120)

	grid_supply_button.button_signal.signal_pin.GPIO_port_pointer = GPIOA;
	grid_supply_button.button_signal.signal_pin.pin_number = GRID_BUTTON_Pin;							// пин кнопки подачи растра

	ON_TOMO_IN_signal.signal_pin.GPIO_port_pointer = ON_TOMO_GPIO_Port;
	ON_TOMO_IN_signal.signal_pin.pin_number = ON_TOMO_Pin;												// пин сигнала ON_TOMO

	BUCKY_CALL_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	BUCKY_CALL_IN_signal.signal_pin.pin_number = BUCKY_CALL_Pin;										// пин сигнала BUCKYCALL

	pushbutton_buckybrake.button_signal.signal_pin.GPIO_port_pointer = GPIOA;
	pushbutton_buckybrake.button_signal.signal_pin.pin_number = PUSHBUTTON_BUCKYBRAKE_Pin;				// пин кнопки тормоза кассетоприёмника

	motor_instance_1.limit_switch.limit_switch_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	motor_instance_1.limit_switch.limit_switch_IN_signal.signal_pin.pin_number = GRID_END_POINT_Pin;	// пин концевика
	motor_instance_1.motor_signals.STEP_OUT_signal.signal_pin.GPIO_port_pointer = GPIOB;
	motor_instance_1.motor_signals.STEP_OUT_signal.signal_pin.pin_number = STEP_Pin;
	motor_instance_1.motor_signals.DIR_OUT_signal.signal_pin.GPIO_port_pointer = GPIOB;
	motor_instance_1.motor_signals.DIR_OUT_signal.signal_pin.pin_number = DIR_Pin;
	motor_instance_1.motor_signals.ENABLE_OUT_signal.signal_pin.GPIO_port_pointer = GPIOB;
	motor_instance_1.motor_signals.ENABLE_OUT_signal.signal_pin.pin_number = ENABLE_Pin;

	DIP_switch.DIP_SWITCH_1_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	DIP_switch.DIP_SWITCH_1_IN_signal.signal_pin.pin_number = CONFIG_1_Pin;

	DIP_switch.DIP_SWITCH_2_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	DIP_switch.DIP_SWITCH_2_IN_signal.signal_pin.pin_number = CONFIG_2_Pin;

	DIP_switch.DIP_SWITCH_3_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	DIP_switch.DIP_SWITCH_3_IN_signal.signal_pin.pin_number = CONFIG_3_Pin;
}

/*
 * Инициализируем аппаратные модули (кнопки, датчики, мотор, интерфейс А1)
 */
void device_modules_init(void)
{
	limit_switch_enabled_once = 0;
	motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;					// даём двигателю задание занять начальное положение
	motor_movement_status = MOTOR_MOVEMENT_IN_PROGRESS;							// выставляем флаг, что мотор находится в движении
	exposition_movement_direction = EXPOSITION_MOVEMENT_FROM_INITIAL_POSITION;	// задаём начальное направление циклического движения при экспозиции
	grid_supply_button.button_released_default_signal_level = LOGIC_LEVEL_LOW;			// выставляем флаг, что при отпущенной кнопке на пине "1"
	grid_supply_button.button_pressing_duration_counter = 0;							// обнуляем счётчик продолжительности нажатия
	grid_supply_button.button_bounce_filter_counts = BUTTON_BOUNCE_FILTER_COUNTS;
	grid_supply_button.button_long_press_ms = BUTTON_LONG_PRESS_DURATION_SEC;
	ON_TOMO_IN_flag = ON_TOMO_WAS_NOT_ENABLED;											// выставляем флаг, что сигнала ON_TOMO не было
	bucky_ready_delay_counter = 0;														// обнуляем счётчик шагов, после которых выставляем BUCKY_READY в "1"
	pushbutton_buckybrake.button_released_default_signal_level = LOGIC_LEVEL_LOW;		// выставляем флаг, что при отпущенной кнопке на пине "1"
	pushbutton_buckybrake.button_pressing_duration_counter = 0;							// обнуляем счётчик продолжительности нажатия
	pushbutton_buckybrake.button_bounce_filter_counts = BUTTON_BOUNCE_FILTER_COUNTS;
	pushbutton_buckybrake.button_long_press_ms = BUTTON_LONG_PRESS_DURATION_SEC;

	motor_instance_1.limit_switch.limit_switch_logic_inverted = LIMIT_SWITCH_LOGIC_LEVEL_INVERTED;
	motor_instance_1.emergency_step_impulses_to_limit = EMERGENCY_STEP_IMPULSES_TO_LIMIT;
	motor_instance_1.DIR_pin_logic_level_inverted = DIR_PIN_LOGIC_LEVEL_INVERTED;
	motor_instance_1.motor_timer_ticks_per_ms = MOTOR_TIMER_TICKS_PER_MS;

	motor_instance_1.step_impulses_acceptable_error = STEP_IMPULSES_ACCEPTABLE_ERROR;
	motor_instance_1.step_impulses_distance_from_limit_switch = STEP_IMPULSES_DISTANCE_INITIAL;
	motor_instance_1.limit_emergency_counter = 0;
	motor_instance_1.motor_movement_direction = MOVE_TO_COORD_END;
	motor_instance_1.step_pin_current_phase = STEP_LOW_PHASE;
	motor_instance_1.ticks_before_next_step_counter = 0;
	motor_instance_1.ticks_since_start_movement_counter = 0;
	motor_instance_1.step_impulses_since_start_movement_counter = 0;
	motor_instance_1.current_speed_step_per_ms = 0;
	motor_instance_1.ticks_for_acceleration_counter = 0;
	motor_instance_1.step_impulses_for_acceleration_counter = 0;
	motor_instance_1.cyclic_movement_direction = MOVE_TO_COORD_END;

	movement_profile_1_default.acceleration_type = NO_ACCELERATION;
	movement_profile_1_default.short_distance_step_impulses = SHORT_DISTANCE_STEP_IMPULSES;
	movement_profile_1_default.far_distance_step_impulses = FAR_DISTANCE_STEP_IMPULSES;
	movement_profile_1_default.min_speed_step_per_ms = CONSTANT_SPEED_STEP_PER_MS;
	movement_profile_1_default.max_speed_step_per_ms = CONSTANT_SPEED_STEP_PER_MS;
	movement_profile_1_default.linear_acceleration_coefficient = LINEAR_ACCELERATION_COEFFICIENT_INITIAL;
	movement_profile_1_default.quadratic_acceleration_coefficient = QUADRATIC_ACCELERATION_COEFFICIENT_INITIAL;
	movement_profile_1_default.acceleration_duration_ms = ACCELERATION_DURATION_MS_MODE_00;

	movement_profile_2_exposition.acceleration_type = LINEAR_ACCELERATION;
	movement_profile_2_exposition.short_distance_step_impulses = SHORT_DISTANCE_STEP_IMPULSES;
	movement_profile_2_exposition.far_distance_step_impulses = FAR_DISTANCE_STEP_IMPULSES;
	movement_profile_2_exposition.min_speed_step_per_ms = MIN_SPEED_STEP_PER_MS_ALL_MODES;
	movement_profile_2_exposition.max_speed_step_per_ms = MAX_SPEED_STEP_PER_MS_MODE_00;
	movement_profile_2_exposition.linear_acceleration_coefficient = LINEAR_ACCELERATION_COEFFICIENT_INITIAL;
	movement_profile_2_exposition.quadratic_acceleration_coefficient = QUADRATIC_ACCELERATION_COEFFICIENT_INITIAL;
	movement_profile_2_exposition.acceleration_duration_ms = ACCELERATION_DURATION_MS_MODE_00;

	movement_profile_3_supply.acceleration_type = NO_ACCELERATION;
	movement_profile_3_supply.short_distance_step_impulses = SHORT_DISTANCE_STEP_IMPULSES;
	movement_profile_3_supply.far_distance_step_impulses = RASTER_SUPPLY_DISTANCE_STEP_IMPULSES;
	movement_profile_3_supply.min_speed_step_per_ms = RASTER_SUPPLY_SPEED_STEP_PER_MS;
	movement_profile_3_supply.max_speed_step_per_ms = RASTER_SUPPLY_SPEED_STEP_PER_MS;
	movement_profile_3_supply.linear_acceleration_coefficient = LINEAR_ACCELERATION_COEFFICIENT_INITIAL;
	movement_profile_3_supply.quadratic_acceleration_coefficient = QUADRATIC_ACCELERATION_COEFFICIENT_INITIAL;
	movement_profile_3_supply.acceleration_duration_ms = ACCELERATION_DURATION_MS_MODE_00;
}

/*
 * Обновляем состояние входных сигналов и аппаратных модулей
 */
void check_input_signals(void)
{
	input_signals_state_update();					// считываем состояние входов, обновляем их состояние в объекте устройства
	buttons_state_update();							// обновляем состояние аппаратных модулей
	read_input_signals_and_set_device_state();		// изменяем состояние устройства в зависимости от входных сигналов
}

void dip_switch_state_update(void)
{
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
	{
		movement_profile_2_exposition.max_speed_step_per_ms = MAX_SPEED_STEP_PER_MS_MODE_00;
		movement_profile_2_exposition.acceleration_duration_ms = ACCELERATION_DURATION_MS_MODE_00;
	}
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
	{
		movement_profile_2_exposition.max_speed_step_per_ms = MAX_SPEED_STEP_PER_MS_MODE_01;
		movement_profile_2_exposition.acceleration_duration_ms = ACCELERATION_DURATION_MS_MODE_01;
	}
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
	{
		movement_profile_2_exposition.max_speed_step_per_ms = MAX_SPEED_STEP_PER_MS_MODE_10;
		movement_profile_2_exposition.acceleration_duration_ms = ACCELERATION_DURATION_MS_MODE_10;
	}
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
	{
		movement_profile_2_exposition.max_speed_step_per_ms = MAX_SPEED_STEP_PER_MS_MODE_11;
		movement_profile_2_exposition.acceleration_duration_ms = ACCELERATION_DURATION_MS_MODE_11;
	}
	switch (DIP_switch.DIP_SWITCH_3_IN_signal.signal_logic_level)
	{
	case LOGIC_LEVEL_LOW:
	{
		motor_instance_1.DIR_pin_logic_level_inverted = 1;
		break;
	}
	case LOGIC_LEVEL_HIGH:
	{
		motor_instance_1.DIR_pin_logic_level_inverted = 0;
		break;
	}
	}
}

/*
 * Опрашиваем состояние входных сигналов
 */

void input_signals_state_update(void)
{
	check_input_signal_state(&grid_sensor.GRID_180_DETECT_IN_signal);
	check_input_signal_state(&grid_sensor.GRID_120_DETECT_IN_signal);
	check_input_signal_state(&grid_supply_button.button_signal);
	check_input_signal_state(&ON_TOMO_IN_signal);
	check_input_signal_state(&BUCKY_CALL_IN_signal);
	check_input_signal_state(&pushbutton_buckybrake.button_signal);
	check_input_signal_state(&motor_instance_1.limit_switch.limit_switch_IN_signal);
	check_input_signal_state(&DIP_switch.DIP_SWITCH_1_IN_signal);
	check_input_signal_state(&DIP_switch.DIP_SWITCH_2_IN_signal);
	check_input_signal_state(&DIP_switch.DIP_SWITCH_3_IN_signal);
}

/*
 * Выставляем одно состояние на всех выходных пинах
 */

void output_signals_state_init(SignalLogicLevel_EnumTypeDef signal_level_to_set)
{
	set_output_signal_state(MOTOR_ENABLE_OUT_PORT, MOTOR_ENABLE_OUT_PIN, signal_level_to_set);
	set_output_signal_state(MOTOR_STEP_OUT_PORT, MOTOR_STEP_OUT_PIN, signal_level_to_set);
	set_output_signal_state(MOTOR_DIR_OUT_PORT, MOTOR_DIR_OUT_PIN, signal_level_to_set);
	set_output_signal_state(LASER_CENTERING_OUT_PORT, LASER_CENTERING_OUT_PIN, signal_level_to_set);
	set_output_signal_state(BUCKYBRAKE_OUT_PORT, BUCKYBRAKE_OUT_PIN, signal_level_to_set);
	set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, signal_level_to_set);
	set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, signal_level_to_set);
	set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, signal_level_to_set);
}

/*
 * Запускаем прерывания, по которым будем опрашивать состояние входных сигналов
 */
void signals_check_timer_interrupts_start(void)
{
	HAL_TIM_Base_Start_IT(SIGNALS_CHECK_TIMER_POINTER);
}

/*
 * ОБработчик прерываний таймера, отвечающего за опрос входных сигналов
 */
void signals_check_timer_interrupt_handler(void)
{
	check_input_signals();
}

/*
 * Обновляем состояние кнопок
 */
void buttons_state_update(void)
{
	check_and_update_button_state(&grid_supply_button, SIGNALS_CHECK_TIMER_TICKS_PER_SEC);
	check_and_update_button_state(&pushbutton_buckybrake, SIGNALS_CHECK_TIMER_TICKS_PER_SEC);
}

/*
 * Обработчик ошибок
 */
void device_error_handler(void)
{
	switch (error_code)					// если код ошибки
	{
	case NO_ERROR:						// если нет ошибки
	{
		device_current_state = DEVICE_STANDBY;	// возвращаемся в состояние ожидания
		break;
	}
	case GRID_TYPE_ERROR:				// если ошибка типа растра
	{
		set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_HIGH);				// выставляем "1" на выводе GRID_120_OUT_PIN
		set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_HIGH);				// выставляем "1" на выводе GRID_180_OUT_PIN

		/*
		 * если отсутствует растр типа 120 и типа 180
		 */
		if ((grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) &&
				(grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
		{
			set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_LOW);			// выставляем "0" на выводе GRID_120_OUT_PIN
			set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_LOW);			// выставляем "0" на выводе GRID_180_OUT_PIN
			error_code = NO_ERROR;																	// выставляем флаг отсутствия ошибки
		}
		break;
	}
	case LIMIT_SWITCH_ERROR:			// если ошибка концевика
	{
		if (motor_movement_purpose != MOTOR_PURPOSE_EMERGENCY_SUPPLY)
		{
			motor_instance_1.step_impulses_distance_from_limit_switch = 0;
			motor_movement_purpose = MOTOR_PURPOSE_EMERGENCY_SUPPLY;
		}
		break;
	}
	case STANDBY_MOVEMENT_ERROR:		// если ошибка движения в режиме ожидания
	{
		/*
		 * если была нажата какая-либо кнопка, выходим из состояния ошибки
		 */
		if ((grid_supply_button.button_current_state != BUTTON_RELEASED) || \
				(pushbutton_buckybrake.button_current_state != BUTTON_RELEASED))
		{
			error_code = NO_ERROR;		// выставляем флаг отсутствия ошибки
		}
		break;
	}
	case ON_TOMO_BUCKY_CALL_ERROR:		// если ошибка сигнала ON_TOMO
	{
		/*
		 * если сигнал ON_TOMO в "0", и сигнал BUCKY_CALL в "0", и мотор завершил движение
		 */
		if ((ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
			(BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
			(motor_movement_status == MOTOR_MOVEMENT_COMPLETED))
		{
			error_code = NO_ERROR;		// выставляем флаг отсутствия ошибки
		}
		break;
	}
	}
}

/*
 * Изменяем состояние устройства в зависимости от входных сигналов
 */
void read_input_signals_and_set_device_state(void)
{
	switch (device_current_state)													// если состояние устройства
	{
	case DEVICE_STARTS:																// если устройство стартует
	{
		device_current_state = DEVICE_INITIAL_MOVEMENT;
		motor_movement_purpose = MOTOR_PURPOSE_INITIAL_MOVEMENT;
		motor_movement_start(&motor_instance_1, &movement_profile_1_default);
		break;
	}
	case DEVICE_ERROR:																// если возникла ошибка
	{
		device_error_handler();														// вызываем обработчик ошибок
		break;
	}
	case DEVICE_STANDBY:															// если устройство в режиме ожидания
	{
		set_grid_out_signal();
		/*
		 * если сигнал ON_TOMO не активен и сигнал ON_TOMO был активен ранее
		 */
		if ((ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
			(ON_TOMO_IN_flag != ON_TOMO_WAS_NOT_ENABLED))
		{
			ON_TOMO_IN_flag = ON_TOMO_WAS_NOT_ENABLED;								// выставляем флаг: сигнал ON_TOMO не был активен
		}
		/*
		 * иначе если кнопка подачи растра нажата долго и наличие/отсутствие растра определено
		 */
		else if (grid_supply_button.button_current_state == BUTTON_LONG_PRESS)
		{
			device_current_state = DEVICE_GRID_SUPPLY;								// выставляем состояние устройства: подача растра

			/*
			 * если растр был извлечён и кнопка подачи растра нажата долго
			 */
			if (motor_instance_1.step_impulses_distance_from_limit_switch >= RASTER_SUPPLY_DISTANCE_STEP_IMPULSES)
			{
				motor_movement_purpose = MOTOR_PURPOSE_GRID_INSERTION;						// назначение движения: вставить растр
				motor_movement_start(&motor_instance_1, &movement_profile_3_supply);																// начинаем движение
			}
			/*
			 * если растр был вставлен и кнопка подачи растра нажата долго
			 */
			if (motor_instance_1.step_impulses_distance_from_limit_switch < RASTER_SUPPLY_DISTANCE_STEP_IMPULSES)
			{
				set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_120
				set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_180
				motor_movement_purpose = MOTOR_PURPOSE_GRID_EXTRACTION;						// назначение движения: извлечь растр
				motor_movement_start(&motor_instance_1, &movement_profile_3_supply);																// начинаем движение
			}
		}
		/*
		 * иначе если сигнал BUCKY_CALL активен и сигнал ON_TOMO неактивен и сигнал ON_TOMO не был активен
		 */
		else if ((BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
				(ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
				(motor_instance_1.step_impulses_distance_from_limit_switch < RASTER_SUPPLY_DISTANCE_STEP_IMPULSES))
		{
			device_current_state = DEVICE_SCANING_TOMO_OFF;											// выставляем состояние устройства: экспозиция без ON_TOMO
			motor_movement_purpose = MOTOR_PURPOSE_EXPOSITION_TOMO_OFF;						// назначение движения: экспозиция без ON_TOMO
			motor_movement_start(&motor_instance_1, &movement_profile_2_exposition);																	// начинаем движение
		}
		/*
		 * иначе если сигнал ON_TOMO активен и сигнал BUCKY_CALL активен
		 */
		else if ((ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
				(BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
		{
			ON_TOMO_IN_flag = ON_TOMO_WAS_ENABLED;
			device_current_state = DEVICE_SCANING_TOMO_ON;											// выставляем состояние устройства: экспозиция с ON_TOMO
			motor_movement_purpose = MOTOR_PURPOSE_EXPOSITION_TOMO_ON;						// назначние движения: экспозиция с ON_TOMO
			motor_movement_start(&motor_instance_1, &movement_profile_2_exposition);																	// начинаем движение
		}
		/*
		 * иначе если концевик неактивен и мы не в положении подачи растра
		 */
		/*
		else if ((!(limit_switch_active(&motor_instance_1))) && \
				(!(motor_instance_1.step_impulses_distance_from_limit_switch >= RASTER_SUPPLY_DISTANCE_STEP_IMPULSES)))
		{
			device_current_state = DEVICE_RETURN_TO_INITIAL_STATE;									// выставляем состояние устройства: возврат в начальное положение
			motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;						// назначение движения: возврат в начальное положение
			motor_movement_start(&motor_instance_1, &movement_profile_1_default);																	// начинаем движение
		}
		*/
		/*
		 * иначе если кнопка тормоза кассетоприёмника нажата
		 */
		else if (pushbutton_buckybrake.button_current_state != BUTTON_RELEASED)
		{
			device_current_state = DEVICE_BUCKYBRAKE;												// выставляем состояние устройства: отпустить тормоз кассетоприёмника
			set_output_signal_state(LASER_CENTERING_OUT_PORT, LASER_CENTERING_OUT_PIN, LOGIC_LEVEL_HIGH);	// выставляем в "1" выходной сигнал LASER_CENTERING
			set_output_signal_state(BUCKYBRAKE_OUT_PORT, BUCKYBRAKE_OUT_PIN, LOGIC_LEVEL_HIGH);		// выставляем в "1" выходной сигнал BUCKYBRAKE
		}
		break;
	}
	case DEVICE_BUCKYBRAKE:																			// если устройство в состоянии "отпустить тормоз кассетоприёмника"
	{
		/*
		 *	если кнопка тормоза кассетоприёмника отпущена
		 */
		if (pushbutton_buckybrake.button_current_state == BUTTON_RELEASED)
		{
			buckybreak_laser_disable();
			device_current_state = DEVICE_STANDBY;													// выставляем состояние устройства: режим ожидания
		}
		break;
	}
	default:
	{
		if (motor_movement_status == MOTOR_MOVEMENT_COMPLETED)			// если статус мотора "движение завершено"
		{
			device_current_state = DEVICE_STANDBY;						// выставляем состояние устройства: "режим ожидания"
		}
		break;
	}
	}
}

void set_grid_out_signal(void)
{
	/*
	 * если растр не представлен
	 */
	if ((grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
		(grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
	{
		set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_120
		set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_180
	}
	/*
	* если тип растра 120
	*/
	if ((grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
		(grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
	{
		set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_HIGH);		// выставляем в "1" выходной сигнал GRID_120
		set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_180
	}
	/*
	* если тип растра 180
	*/
	if ((grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
		(grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
	{
		set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_120
		set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_HIGH);		// выставляем в "1" выходной сигнал GRID_180
	}
	if ((grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
		(grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
	{
		set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_HIGH);		// выставляем в "1" выходной сигнал GRID_120
		set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_HIGH);		// выставляем в "1" выходной сигнал GRID_180
		error_code = GRID_TYPE_ERROR;									// выставляем флаг ошибки типа растра
		device_current_state = DEVICE_ERROR;							// переключаем устройство в режим ожидания
	}
}

void buckybreak_laser_disable(void)
{
	set_output_signal_state(LASER_CENTERING_OUT_PORT, LASER_CENTERING_OUT_PIN, LOGIC_LEVEL_LOW);	// выставляем в "0" выходной сигнал LASER_CENTERING
	set_output_signal_state(BUCKYBRAKE_OUT_PORT, BUCKYBRAKE_OUT_PIN, LOGIC_LEVEL_LOW);
}

/*
***************************************
*   Функции мотора
***************************************
*/

/*
 * Запускаем прерывания, по которым шагает мотор
 */
void motor_timer_interrupts_start(void)
{
	HAL_TIM_Base_Start_IT(MOTOR_TIMER_POINTER);
}

/*
 * Останавливаем прерывания, по которым шагает мотор
 */
void motor_timer_interrupts_stop(void)
{
	HAL_TIM_Base_Stop_IT(MOTOR_TIMER_POINTER);
}

/*
 * Начинаем движение мотора
 */
void motor_movement_start(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile)
{
	if (device_current_state == DEVICE_STANDBY)							// если устройство в режиме ожидания
	{
		device_current_state = DEVICE_ERROR;
		error_code = STANDBY_MOVEMENT_ERROR;							// выставляем ошибку (нельзя двигаться в режиме ожидания)
	}
	else
	{
		enable_pin_set();
		if ((motor_movement_purpose == MOTOR_PURPOSE_EXPOSITION_TOMO_OFF) || (motor_movement_purpose == MOTOR_PURPOSE_EXPOSITION_TOMO_ON))
		{
			dip_switch_state_update();
		}
		motor_movement_init(motor_object, movement_profile);
		motor_movement_status = MOTOR_MOVEMENT_IN_PROGRESS;					// выставляем флаг, что мотор находится в движении
		motor_timer_interrupts_start();
	}						// запускаем прерывания, по которым мотор будет шагать
}

/*
 * Прекращаем движение мотора
 */
void motor_movement_complete(void)
{
	motor_timer_interrupts_stop();										// останавливаем прерывания, по которым шагает мотор
	enable_pin_clear();
	motor_movement_status = MOTOR_MOVEMENT_COMPLETED;					// выставляем флаг, что движение завершено
}

/*
 * Начинаем отсчёт шагов до выставления сигнала BUCKY_READY
 */
void bucky_ready_delay_set(void)
{
	if (bucky_ready_delay_counter != BUCKY_READY_DELAY_STEP_IMPULSES)
	{
		bucky_ready_delay_counter++;
		if (bucky_ready_delay_counter == BUCKY_READY_DELAY_STEP_IMPULSES)
		{
			set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_HIGH);
		}
	}
}

void bucky_ready_dsable(void)
{
	bucky_ready_delay_counter = 0;
	set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_LOW);
}

void motor_check_conditions_and_step(MotorObject_StructTypeDef* motor_object, MotorMovementProfile_StructTypeDef* movement_profile)
{
	switch (motor_movement_purpose)												// если назначение движения мотора
	{
	case MOTOR_PURPOSE_INITIAL_MOVEMENT:
	{
		if (!limit_switch_enabled_once)
		{
			if(!limit_switch_active(&motor_instance_1))
			{
				motor_check_counter_and_make_step_to_direction(&motor_instance_1, &movement_profile_1_default, MOVE_TO_COORD_ORIGIN);
			}
			else
			{
				limit_switch_enabled_once = 1;
			}
		}
		else
		{
			if (motor_object->step_impulses_distance_from_limit_switch < FAR_DISTANCE_STEP_IMPULSES)
			{
				motor_check_counter_and_make_step_to_direction(&motor_instance_1, &movement_profile_1_default, MOVE_TO_COORD_END);
			}
			else
			{
				motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;
			}
		}
		break;
	}
	case MOTOR_PURPOSE_GRID_INSERTION:													// если назначение движения мотора - вставить растр
	{
		if (!(limit_switch_active(&motor_instance_1)))												// если концевик не активен
		{
			motor_check_counter_and_make_step_to_direction(&motor_instance_1,  &movement_profile_3_supply, MOVE_TO_COORD_ORIGIN);							// двигаемся к начальной точке
		}
		else
		{
			set_grid_out_signal();
			if (grid_supply_button.button_current_state == BUTTON_RELEASED)				// иначе если кнопка подачи растра отпущена
			{
				motor_movement_complete();												// завершаем движение
			}
		}
		break;
	}
	case MOTOR_PURPOSE_GRID_EXTRACTION:													// если назначение движения мотора - извлечь растр
	{
		if (motor_object->step_impulses_distance_from_limit_switch < RASTER_SUPPLY_DISTANCE_STEP_IMPULSES)		// если мы не дошли до крайнего положения
		{
			motor_check_counter_and_make_step_to_direction(&motor_instance_1,  &movement_profile_3_supply, MOVE_TO_COORD_END);							// движемся от начальной точки (наружу)
		}
		else
		{
			if (grid_supply_button.button_current_state == BUTTON_RELEASED)				// иначе если кнопка подачи растра отпущена
			{
				motor_movement_complete();												// завершаем движение
			}
		}
		break;
	}
	case MOTOR_PURPOSE_EXPOSITION_TOMO_OFF:												// если назначение движения - экспозиция без сигнала ON_TOMO
	{
		if (BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW)					// если сигнал BUCKY_CALL в "1"
		{
			cyclic_movement_step(&motor_instance_1, &movement_profile_2_exposition);														// делаем шаг
			bucky_ready_delay_set();
		}
		else
		{
			bucky_ready_dsable();								// иначе выключаем сигнал BUCKY_READY
			motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;				// выставляем назначение движения - двигаться в начальное положение
		}
		break;
	}
	case MOTOR_PURPOSE_EXPOSITION_TOMO_ON:												// если назначение движения - экспозиция с сигналом ON_TOMO
	{
		if (BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW)
		{
			cyclic_movement_step(&motor_instance_1, &movement_profile_2_exposition);

			if ((ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
					(ON_TOMO_IN_flag != ON_TOMO_WAS_ENABLED_AND_DISABLED))				// если сигнал ON_TOMO в "0"
			{
				ON_TOMO_IN_flag = ON_TOMO_WAS_ENABLED_AND_DISABLED;						// выставляем флаг, что ON_TOMO был в "1", а затем в "0"
				set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_HIGH);
			}
			// если сигнал ON_TOMO был включён и выключен, и сигнал ON_TOMO включён
			if ((ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && (ON_TOMO_IN_flag == ON_TOMO_WAS_ENABLED_AND_DISABLED))
			{
				ON_TOMO_IN_flag = ON_TOMO_WAS_NOT_ENABLED;
				set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_LOW);
			}
		}
		else
		{
			set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_LOW);
			motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;				// выставляем назначение движения - двигаться в начальное положение
		}
		break;
	}
	case MOTOR_PURPOSE_TAKE_INITIAL_POSITION:											// если назначение движения - вернуться в начальную позицию
	{
		if(!(limit_switch_active(&motor_instance_1)))
		{
			motor_check_counter_and_make_step_to_direction(&motor_instance_1, &movement_profile_1_default, MOVE_TO_COORD_ORIGIN);		// делаем шаг в направлении начального положения
		}
		else
		{
			motor_instance_1.limit_emergency_counter = 0;
			motor_movement_complete();
		}
		break;
	}
	case MOTOR_PURPOSE_EMERGENCY_SUPPLY:
	{
		set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_LOW);
		if (motor_object->step_impulses_distance_from_limit_switch < (RASTER_SUPPLY_DISTANCE_STEP_IMPULSES + STEP_IMPULSES_ACCEPTABLE_ERROR))
		{
			motor_check_counter_and_make_step_to_direction(&motor_instance_1, &movement_profile_1_default, MOVE_TO_COORD_END);
		}
	}
	}
}

/*
 * Обработчик прерываний таймера, отвечающего за шаги мотора
 */
void motor_timer_interrupt_handler(void)
{
	motor_check_conditions_and_step(&motor_instance_1,  &movement_profile_2_exposition);
}
