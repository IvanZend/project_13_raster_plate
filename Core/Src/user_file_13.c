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

#include "user_file_13.h"
#include "stdio.h"

/*
********************************************************************************
*								DEFINE CONSTANTS
********************************************************************************
*/

#define DIR_PIN_LOGIC_LEVEL_INVERTED		1			// инвертирован ли логический уровень направления (зависит от аппаратной конфигурации драйвера)
#define ENABLE_PIN_LOGIC_LEVEL_INVERTED		1
#define LIMIT_SWITCH_LOGIC_LEVEL_INVERTED	1			// если концевик при размокнутом состоянии выдаёт "1", выставляем флаг инверсии
#define RASTER_SUPPLY_DISTANCE_STEPS		1000		// расстояние от концевика, на которое растр выдвигается для подачи
#define EXPOSITION_MAX_DISTANCE_STEPS		900			// крайнее положение растра при экспозиции без ТОМО
#define STEP_DISTANCE_INIT_VALUE			10			// начальное значение количества шагов до концевика (чтобы растр доехал до концевика и определилось истинное расстояние)
#define EMERGENCY_STEPS_TO_LIMIT			10000		// максимальное расстояние, которое ШД может проехать до концевика. После него выполняем аварийное торможение.
#define BUTTON_BOUNCE_FILTER_COUNTS			5			// количество отсчетов, после которого решаем, что дребезг закончился и кнопка нажата
#define BUTTON_LONG_PRESS_COUNTS			50			// количество тиков, после которого фиксируем долгое нажатие кнопки
#define BUCKY_READY_DELAY_STEPS				3			// количество шагов, после которых растр разгоняется, и загорается сигнал BUCKY_READY
#define MOVEMENT_EQUATION_COEFFICIENT		375/100
#define MIN_STEPS_PER_SEC_ALL_MODES			1600
#define MAX_STEPS_PER_SEC_MODE_00			7600
#define MAX_STEPS_PER_SEC_MODE_01			5200
#define MAX_STEPS_PER_SEC_MODE_10			4000
#define MAX_STEPS_PER_SEC_MODE_11			4600
#define MOTOR_TIMER_TICKS_PER_SECOND		1000000
#define CONSTANT_SPEED_TICKS_PER_STEP		1000
#define LINEAR_ACCELERATION_COEFFICIENT		15/100
#define EXPOSITITON_MOVEMENT_STEPS			913

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
	device_current_state = DEVICE_STARTS;										// выставляем состояние устройства: устройство стартует
	pins_init();																// инициализируем сигналы (указываем пины и порты, инициализируем единый массив сигналов)
	output_signals_state_init(LOGIC_LEVEL_LOW);									// выставляем состояние выходных сигналов
	input_signals_state_update();												// считываем состояние входных сигналов
	device_modules_init();														// инициализируем аппаратные модули (кнопки, датчики, мотор, интерфейс А1, DIP-переключатели)
	buttons_state_update();														// обновляем состояние аппаратных модулей
	enable_pin_set();															// навсегда выставляем "1" на входе ШД "Enable"
	error_code = NO_ERROR;														// выставляем отсутствие ошибки
	signals_check_timer_interrupts_start();										// запускаем таймер считывания состояний сигналов
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

/*
 * Определяем входные пины, исходя из инициализации, созданной конфигуратором пинов
 */
void pins_init(void)
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

	limit_switch.GRID_END_POINT_IN_signal.signal_pin.GPIO_port_pointer = GPIOA;
	limit_switch.GRID_END_POINT_IN_signal.signal_pin.pin_number = GRID_END_POINT_Pin;					// пин концевика

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
	motor.steps_distance_from_limit_switch = STEP_DISTANCE_INIT_VALUE;					// Задаём условное начальное расстояние от концевика, отличное от нуля. Чтобы мотор доехал до концевика и начал отсчёт.
	motor.limit_emergency_counter = 0;													// обнуляем аварийный счётчик шагов
	motor.motor_move_direction = MOVE_TO_COORD_END;										// задаём направление движения: двигаться ОТ начального положения
	motor.step_pin_current_phase = STEP_LOW_PHASE;										// задаём фазу сигнала STEP
	motor.motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;					// даём двигателю задание занять начальное положение
	motor.motor_movement_status = MOTOR_MOVEMENT_IN_PROGRESS;							// выставляем флаг, что мотор находится в движении
	motor.exposition_movement_direction = EXPOSITION_MOVEMENT_FROM_INITIAL_POSITION;	// задаём начальное направление циклического движения при экспозиции
	motor.acceleration_mode = ACCELERATION_MODE_00;										// задаём начальный режим ускорения
	grid_supply_button.button_released_default_signal_level = LOGIC_LEVEL_LOW;			// выставляем флаг, что при отпущенной кнопке на пине "1"
	grid_supply_button.button_pressing_duration_counter = 0;							// обнуляем счётчик продолжительности нажатия
	ON_TOMO_IN_flag = ON_TOMO_WAS_NOT_ENABLED;											// выставляем флаг, что сигнала ON_TOMO не было
	bucky_ready_delay_counter = 0;														// обнуляем счётчик шагов, после которых выставляем BUCKY_READY в "1"
	pushbutton_buckybrake.button_released_default_signal_level = LOGIC_LEVEL_LOW;		// выставляем флаг, что при отпущенной кнопке на пине "1"
	pushbutton_buckybrake.button_pressing_duration_counter = 0;							// обнуляем счётчик продолжительности нажатия
	ticks_before_next_step_counter = 0;
	ticks_since_start_movement_counter = 0;
	steps_for_acceleration_counter = 0;
	steps_since_start_movement_counter = 0;
	ticks_for_acceleration_counter = 0;
	steps_per_sec = 0;
}

/*
 * Обновляем состояние входных сигналов и аппаратных модулей
 */
void check_input_signals(void)
{
	input_signals_state_update();					// считываем состояние входов, обновляем их состояние в объекте устройства
	buttons_state_update();							// обновляем состояние аппаратных модулей
	dip_switch_state_update();
	device_error_check();							// проверяем текущее состояние устройства на наличие ошибок
	read_input_signals_and_set_device_state();		// изменяем состояние устройства в зависимости от входных сигналов
}

void dip_switch_state_update(void)
{
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
	{
		motor.acceleration_mode = ACCELERATION_MODE_00;
	}
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
	{
		motor.acceleration_mode = ACCELERATION_MODE_01;
	}
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
	{
		motor.acceleration_mode = ACCELERATION_MODE_10;
	}
	if ((DIP_switch.DIP_SWITCH_1_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && (DIP_switch.DIP_SWITCH_2_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
	{
		motor.acceleration_mode = ACCELERATION_MODE_11;
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
	check_input_signal_state(&limit_switch.GRID_END_POINT_IN_signal);
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
	check_button_state(&grid_supply_button);
	check_button_state(&pushbutton_buckybrake);
}

/*
 * Считывание и запись состояния входного пина
 */
void check_input_signal_state(InSignalAttributes_TypeDef* signal_to_check)
{
	GPIO_PinState current_logic_state = HAL_GPIO_ReadPin(signal_to_check->signal_pin.GPIO_port_pointer, signal_to_check->signal_pin.pin_number);

	switch (current_logic_state)
	{
	case GPIO_PIN_SET:
	{
		signal_to_check->signal_logic_level = LOGIC_LEVEL_HIGH;
		break;
	}
	case GPIO_PIN_RESET:
	{
		signal_to_check->signal_logic_level = LOGIC_LEVEL_LOW;
		break;
	}
	}
}

/*
 * Выставляем логическое состояние на выходном пине
 */
void set_output_signal_state(GPIO_TypeDef* GPIO_port_pointer, uint16_t pin_number, SignalLogicLevel_EnumTypeDef requied_logic_level)
{
	if (requied_logic_level == LOGIC_LEVEL_LOW)
	{
		HAL_GPIO_WritePin(GPIO_port_pointer, pin_number, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIO_port_pointer, pin_number, GPIO_PIN_SET);
	}
}

/*
 * Проверяем состояние кнопки
 */
void check_button_state(ButtonAttributes_TypeDef* button_to_check)
{
	if (button_to_check->button_released_default_signal_level == LOGIC_LEVEL_LOW)						// если при отпущенной кнопке логическое состояние пина "0"
	{
		if (button_to_check->button_signal.signal_logic_level == LOGIC_LEVEL_HIGH)						// если текущий уровень сигнала на пине кнопки "1"
		{
			if (button_to_check->button_current_state != BUTTON_LONG_PRESS)								// если ещё не было зафиксировано долгое нажатие
			{
				if (button_to_check->button_pressing_duration_counter >= BUTTON_BOUNCE_FILTER_COUNTS)	// если нажатие длится дольше, чем нужно для фильтрации дребезга
				{
					button_to_check->button_current_state = BUTTON_SHORT_PRESS;							// выставляем флаг короткого нажатия
				}
			}
			if (button_to_check->button_pressing_duration_counter >= BUTTON_LONG_PRESS_COUNTS)			// если счётчик продолжительности нажатия дошёл до значения длительного нажатия
			{
				button_to_check->button_pressing_duration_counter = BUTTON_LONG_PRESS_COUNTS;			// удерживаем счётчик от дальнейшего увеличения
				button_to_check->button_current_state = BUTTON_LONG_PRESS;								// выставляем флаг долгого нажатия
			}
			button_to_check->button_pressing_duration_counter = button_to_check->button_pressing_duration_counter + 1;	// инкрементируем счётчик продолжительности нажатия
		}
		else																							// если текущий уровень сигнала на пине кнопки "0"
		{
			button_to_check->button_pressing_duration_counter = 0;										// обнуляем счётчик продолжительности нажатия
			button_to_check->button_current_state = BUTTON_RELEASED;									// выставляем флаг, что кнопка отпущена
		}
	}
	else																								// если при отпущенной кнопке логическое состояние пина "1"
	{
		if (button_to_check->button_signal.signal_logic_level == LOGIC_LEVEL_HIGH)						// если текущий уровень сигнала на пине кнопки "1"
		{
			button_to_check->button_pressing_duration_counter = 0;										// обнуляем счётчик продолжительности нажатия
			button_to_check->button_current_state = BUTTON_RELEASED;									// выставляем флаг, что кнопка отпущена
		}
		else																							// если текущий уровень сигнала на пине кнопки "0"
		{
			if (button_to_check->button_current_state != BUTTON_LONG_PRESS)								// если ещё не было зафиксировано долгое нажатие
			{
				if (button_to_check->button_pressing_duration_counter >= BUTTON_BOUNCE_FILTER_COUNTS)	// если нажатие длится дольше, чем нужно для фильтрации дребезга
				{
					button_to_check->button_current_state = BUTTON_SHORT_PRESS;							// выставляем флаг короткого нажатия
				}
			}
			if (button_to_check->button_pressing_duration_counter >= BUTTON_LONG_PRESS_COUNTS)			// если счётчик продолжительности нажатия дошёл до значения длительного нажатия
			{
				button_to_check->button_pressing_duration_counter = BUTTON_LONG_PRESS_COUNTS;			// удерживаем счётчик от дальнейшего увеличения
				button_to_check->button_current_state = BUTTON_LONG_PRESS;								// выставляем флаг долгого нажатия
			}
			button_to_check->button_pressing_duration_counter = button_to_check->button_pressing_duration_counter + 1;	// инкрементируем счётчик продолжительности нажатия
		}
	}
}

/*
 * Проверка текущего состояния устройства на наличие ошибок
 */

void device_error_check(void)
{
	/*
	 * если определён тип растра 120 и 180 одновременно
	 */
	if ((grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
		(grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
	{
		error_code = GRID_TYPE_ERROR;									// выставляем флаг ошибки типа растра
		device_current_state = DEVICE_STANDBY;							// переключаем устройство в режим ожидания
	}
	if (motor.limit_emergency_counter >= EMERGENCY_STEPS_TO_LIMIT)		// если прошагали критическое количество шагов в сторону концевика
	{
		motor.limit_emergency_counter = EMERGENCY_STEPS_TO_LIMIT;		// удерживаем аварийный счётчик шагов от дальнейшего увеличения
		error_code = LIMIT_SWITCH_ERROR;								// выставляем ошибку концевика (решаем, что концевик неисправен)
		device_current_state = DEVICE_ERROR;							// переключаем устройство в состояние ошибки
	}
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
		break;							// остаёмся в этом состоянии до перезагрузки
	}
	case STANDBY_MOVEMENT_ERROR:		// если ошибка движения в режиме ожидания
	{
		/*
		 * если была нажата какая-либо кнопка, выходим из состояния ошибки
		 */
		if ((grid_supply_button.button_current_state == BUTTON_SHORT_PRESS) || \
				(pushbutton_buckybrake.button_current_state == BUTTON_SHORT_PRESS))
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
			(motor.motor_movement_status == MOTOR_MOVEMENT_COMPLETED))
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
		device_current_state = DEVICE_RETURN_TO_INITIAL_STATE;						// выставляем состояние устройства: возврат в начальное положение
		motor.motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;			// назначение движения: возврат в начальное положение
		motor_movement_start();
		break;
	}
	case DEVICE_ERROR:																// если возникла ошибка
	{
		motor.motor_movement_purpose = MOTOR_PURPOSE_INSTANT_STOP;					// останавливаем мотор
		device_error_handler();														// вызываем обработчик ошибок
		break;
	}
	case DEVICE_STANDBY:															// если устройство в режиме ожидания
	{
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
			if (motor.steps_distance_from_limit_switch >= RASTER_SUPPLY_DISTANCE_STEPS)
			{
				motor.motor_movement_purpose = MOTOR_PURPOSE_GRID_INSERTION;						// назначение движения: вставить растр
				motor_movement_start();																// начинаем движение
			}
			/*
			 * если растр был вставлен и кнопка подачи растра нажата долго
			 */
			if (motor.steps_distance_from_limit_switch < RASTER_SUPPLY_DISTANCE_STEPS)
			{
				set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_120
				set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_180
				motor.motor_movement_purpose = MOTOR_PURPOSE_GRID_EXTRACTION;						// назначение движения: извлечь растр
				motor_movement_start();																// начинаем движение
			}
		}
		/*
		 * иначе если сигнал BUCKY_CALL активен и сигнал ON_TOMO неактивен и сигнал ON_TOMO не был активен
		 */
		else if ((BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
				(ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
				(ON_TOMO_IN_flag == ON_TOMO_WAS_NOT_ENABLED) && \
				(motor.steps_distance_from_limit_switch < RASTER_SUPPLY_DISTANCE_STEPS))
		{
			device_current_state = DEVICE_SCANING_TOMO_OFF;											// выставляем состояние устройства: экспозиция без ON_TOMO
			motor.motor_movement_purpose = MOTOR_PURPOSE_EXPOSITION_TOMO_OFF;						// назначение движения: экспозиция без ON_TOMO
			motor_movement_start();																	// начинаем движение
		}
		/*
		 * иначе если сигнал ON_TOMO активен и сигнал ON_TOMO не был активен ранее
		 */
		else if ((ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
				(ON_TOMO_IN_flag == ON_TOMO_WAS_NOT_ENABLED))
		{
			ON_TOMO_IN_flag = ON_TOMO_WAS_ENABLED;													// выставляем флаг: сигнал ON_TOMO активен
		}
		/*
		 * иначе если сигнал ON_TOMO активен и сигнал BUCKY_CALL активен и сигнал ON_TOMO был активен ранее
		 */
		else if ((ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
				(BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW) && \
				(ON_TOMO_IN_flag == ON_TOMO_WAS_ENABLED) && \
				(motor.steps_distance_from_limit_switch < RASTER_SUPPLY_DISTANCE_STEPS))
		{
			device_current_state = DEVICE_SCANING_TOMO_ON;											// выставляем состояние устройства: экспозиция с ON_TOMO
			motor.motor_movement_purpose = MOTOR_PURPOSE_EXPOSITION_TOMO_ON;						// назначние движения: экспозиция с ON_TOMO
			motor_movement_start();																	// начинаем движение
		}
		/*
		 * иначе если концевик неактивен и мы не в положении подачи растра
		 */
		else if ((!(limit_switch_return_state())) && \
				(!(motor.steps_distance_from_limit_switch >= RASTER_SUPPLY_DISTANCE_STEPS)))
		{
			device_current_state = DEVICE_RETURN_TO_INITIAL_STATE;									// выставляем состояние устройства: возврат в начальное положение
			motor.motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;						// назначение движения: возврат в начальное положение
			motor_movement_start();																	// начинаем движение
		}
		/*
		 * иначе если кнопка тормоза кассетоприёмника нажата
		 */
		else if (pushbutton_buckybrake.button_current_state == BUTTON_SHORT_PRESS)
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
			set_output_signal_state(LASER_CENTERING_OUT_PORT, LASER_CENTERING_OUT_PIN, LOGIC_LEVEL_LOW);	// выставляем в "0" выходной сигнал LASER_CENTERING
			set_output_signal_state(BUCKYBRAKE_OUT_PORT, BUCKYBRAKE_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал BUCKYBRAKE

			device_current_state = DEVICE_STANDBY;													// выставляем состояние устройства: режим ожидания
		}
		break;
	}
	case DEVICE_GRID_SUPPLY:																		// если устройство в состоянии "подача растра"
	{
		/*
		 * если назначение движения "вставить растр" и статус движения "движение завершено"
		 */
		if ((motor.motor_movement_purpose == MOTOR_PURPOSE_GRID_INSERTION) && \
			(motor.motor_movement_status == MOTOR_MOVEMENT_COMPLETED))
		{
			/*
			 * если растр не представлен
			 */
			if ((grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW)&& \
					(grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
			{
				set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_120
				set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_180
			}
			/*
			* если тип растра 120
			*/
			if (grid_sensor.GRID_120_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH)
			{
				set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_HIGH);		// выставляем в "1" выходной сигнал GRID_120
				set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_180
			}
			/*
			* если тип растра 180
			*/
			if (grid_sensor.GRID_180_DETECT_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH)
			{
				set_output_signal_state(GRID_120_OUT_PORT, GRID_120_OUT_PIN, LOGIC_LEVEL_LOW);		// выставляем в "0" выходной сигнал GRID_120
				set_output_signal_state(GRID_180_OUT_PORT, GRID_180_OUT_PIN, LOGIC_LEVEL_HIGH);		// выставляем в "1" выходной сигнал GRID_180
			}
		}

		if (motor.motor_movement_status == MOTOR_MOVEMENT_COMPLETED) 	// если статус мотора "движение завершено"
		{

			device_current_state = DEVICE_STANDBY;						// выставляем состояние устройства: "режим ожидания"
		}
		break;
	}
	case DEVICE_RETURN_TO_INITIAL_STATE:
	{
		if (motor.motor_movement_status == MOTOR_MOVEMENT_COMPLETED) 	// если статус мотора "движение завершено"
		{
			device_current_state = DEVICE_STANDBY;						// выставляем состояние устройства: "режим ожидания"
		}
		break;
	}
	case DEVICE_SCANING_TOMO_OFF:
	{
		if (motor.motor_movement_status == MOTOR_MOVEMENT_COMPLETED) 	// если статус мотора "движение завершено"
		{
			device_current_state = DEVICE_STANDBY;						// выставляем состояние устройства: "режим ожидания"
		}
		break;
	}
	case DEVICE_SCANING_TOMO_ON:
	{
		if (motor.motor_movement_status == MOTOR_MOVEMENT_COMPLETED)	// если статус мотора "движение завершено"
		{
			device_current_state = DEVICE_STANDBY;						// выставляем состояние устройства: "режим ожидания"
		}
		break;
	}
	}
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
void motor_movement_start(void)
{
	if (device_current_state == DEVICE_STANDBY)							// если устройство в режиме ожидания
	{
		error_code = STANDBY_MOVEMENT_ERROR;							// выставляем ошибку (нельзя двигаться в режиме ожидания)
	}
	motor.motor_movement_status = MOTOR_MOVEMENT_IN_PROGRESS;			// выставляем флаг, что мотор находится в движении
	motor_timer_interrupts_start();										// запускаем прерывания, по которым мотор будет шагать
}

/*
 * Прекращаем движение мотора
 */
void motor_movement_complete(void)
{
	motor_timer_interrupts_stop();										// останавливаем прерывания, по которым шагает мотор
	motor.motor_movement_status = MOTOR_MOVEMENT_COMPLETED;				// выставляем флаг, что движение завершено
	reset_movement_counters();
}

/*
 * Делаем шаг в заданном направлении
 */
void motor_make_step_to_direction(MotorMoveDirection_EnumTypeDef move_direction)
{
	motor.motor_move_direction = move_direction;						// выставляем направление шага
	motor_direction_pin_set();											// выставляем нужное состояние на пине направления
	check_limit_switch_and_make_step();									// проверяем состояние концевика и совершаем шаг
}

/*
 * Начинаем отсчёт шагов до выставления сигнала BUCKY_READY
 */
void bucky_ready_response_set(SignalLogicLevel_EnumTypeDef logic_level_to_set)
{
	switch (logic_level_to_set)											// если требуемый логический уровень сигнала BUCKY_READY
	{
	case LOGIC_LEVEL_HIGH:												// если требуемый логический уровень "1"
	{
		bucky_ready_delay_counter++;									// инкрементируем счётчик шагов
		if (bucky_ready_delay_counter >= BUCKY_READY_DELAY_STEPS)		// если досчитали до нужного количества шагов
		{
			bucky_ready_delay_counter = BUCKY_READY_DELAY_STEPS;		// удерживаем счётчик от дальнейшего увеличения
		}
		break;
	}
	case LOGIC_LEVEL_LOW:												// если требуемый логический уровень "0"
	{
		bucky_ready_delay_counter = 0;									// обнуляем счётчик шагов
		break;
	}
	}
}

/*
 * Проверяем счётчик шагов до выставления сигнала BUCKY_READY (по таймеру)
 */
void bucky_ready_response_check(void)
{
	if (bucky_ready_delay_counter == BUCKY_READY_DELAY_STEPS)									// если прошли достаточное количество шагов
	{
		set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_HIGH);	// выставляем сигнал BUCKY_READY в "1"
	}
	else
	{
		set_output_signal_state(BUCKY_READY_OUT_PORT, BUCKY_READY_OUT_PIN, LOGIC_LEVEL_LOW);	// иначе выставляем сигнал BUCKY_READY в "0"
	}
}

void reset_movement_counters(void)
{
	ticks_before_next_step_counter = 0;
	ticks_since_start_movement_counter = 0;
	steps_for_acceleration_counter = 0;
	steps_since_start_movement_counter = 0;
	ticks_for_acceleration_counter = 0;
}

uint64_t movement_time_function(uint64_t time_value)
{
	uint64_t calculated_steps_per_sec = 0;
	calculated_steps_per_sec = (((time_value*time_value)*MOVEMENT_EQUATION_COEFFICIENT)/MOTOR_TIMER_TICKS_PER_SECOND) + MIN_STEPS_PER_SEC_ALL_MODES;
	//calculated_steps_per_sec = time_value * LINEAR_ACCELERATION_COEFFICIENT + MIN_STEPS_PER_SEC_ALL_MODES;
	return calculated_steps_per_sec;
}

void calculate_ticks_per_next_step(void)
{
	if (motor.motor_movement_status == MOTOR_MOVEMENT_IN_PROGRESS)
	{
		if ((motor.motor_movement_purpose == MOTOR_PURPOSE_EXPOSITION_TOMO_OFF) || (motor.motor_movement_purpose == MOTOR_PURPOSE_EXPOSITION_TOMO_ON))
		{
			//uint64_t steps_per_sec = (((ticks_since_start_movement_counter*ticks_since_start_movement_counter)*MOVEMENT_EQUATION_COEFFICIENT)/MOTOR_TIMER_TICKS_PER_SECOND) + MIN_STEPS_PER_SEC_ALL_MODES;
			if ((EXPOSITITON_MOVEMENT_STEPS - steps_since_start_movement_counter) >= steps_for_acceleration_counter)
			{
				steps_per_sec = movement_time_function(ticks_since_start_movement_counter);
				if (steps_per_sec < MAX_STEPS_PER_SEC_MODE_00)
				{
					ticks_before_next_step_counter = MOTOR_TIMER_TICKS_PER_SECOND/steps_per_sec;
					steps_for_acceleration_counter++;
				}
				else
				{
					ticks_before_next_step_counter = MOTOR_TIMER_TICKS_PER_SECOND/MAX_STEPS_PER_SEC_MODE_00;
				}
			}
			else
			{
				steps_per_sec = movement_time_function(ticks_for_acceleration_counter);
				ticks_before_next_step_counter = MOTOR_TIMER_TICKS_PER_SECOND/steps_per_sec;
			}
		}
		else
		{
			ticks_before_next_step_counter = CONSTANT_SPEED_TICKS_PER_STEP;
		}
		steps_since_start_movement_counter++;
	}
}

void motor_check_conditions_and_step(void)
{
	switch (motor.motor_movement_purpose)												// если назначение движения мотора
	{
	case MOTOR_PURPOSE_INSTANT_STOP:													// если назначение движения мотора - мгновенная остановка
	{
		motor_movement_complete();														// завершаем движение
		break;
	}
	case MOTOR_PURPOSE_GRID_INSERTION:													// если назначение движения мотора - вставить растр
	{
		if (!(limit_switch_return_state()))												// если концевик не активен
		{
			motor_make_step_to_direction(MOVE_TO_COORD_ORIGIN);							// двигаемся к начальной точке
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
	case MOTOR_PURPOSE_GRID_EXTRACTION:													// если назначение движения мотора - извлечь растр
	{
		if (motor.steps_distance_from_limit_switch < RASTER_SUPPLY_DISTANCE_STEPS)		// если мы не дошли до крайнего положения
		{
			motor_make_step_to_direction(MOVE_TO_COORD_END);							// движемся от начальной точки (наружу)
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
		if (BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW)				// если сигнал BUCKY_CALL в "1"
		{
			cyclic_movement_step();														// делаем шаг
			bucky_ready_response_set(LOGIC_LEVEL_HIGH);									// запускаем счётчик шагов до выставления сигнала BUCKY_READY
		}
		else
		{
			bucky_ready_response_set(LOGIC_LEVEL_LOW);									// иначе выключаем сигнал BUCKY_READY
			motor.motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;			// выставляем назначение движения - двигаться в начальное положение
		}
		break;
	}
	case MOTOR_PURPOSE_EXPOSITION_TOMO_ON:												// если назначение движения - экспозиция с сигналом ON_TOMO
	{
		/*
		 * если сигнал ON_OMO был включён, и сигнал BUCKY_CALL включён
		 */
		if ((ON_TOMO_IN_flag == ON_TOMO_WAS_ENABLED) && \
			(BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
		{
			if (ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH)				// если сигнал ON_TOMO в "0"
			{
				ON_TOMO_IN_flag = ON_TOMO_WAS_ENABLED_AND_DISABLED;						// выставляем флаг, что ON_TOMO был в "1", а затем в "0"
				bucky_ready_response_set(LOGIC_LEVEL_HIGH);								// запускаем счётчик шагов до выставления сигнала BUCKY_READY
			}
			cyclic_movement_step();														// делаем шаг
		}
		/*
		 * если сигнал ON_TOMO был включён и выключен, и сигнал ON_TOMO включён
		 */
		if ((ON_TOMO_IN_flag == ON_TOMO_WAS_ENABLED_AND_DISABLED) && \
			(ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW))
		{
			bucky_ready_response_set(LOGIC_LEVEL_LOW);									// выключаем сигнал BUCKY_READY
			motor.motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;			// выставляем назначение движения - двигаться в начальное положение
		}
		/*
		 * если сигнал BUCKY_CALL выключен, и сигнал ON_TOMO был включён и выключен, и сигнал ON_TOMO сейчас выключен
		 */
		if ((BUCKY_CALL_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH) && \
			(ON_TOMO_IN_flag == ON_TOMO_WAS_ENABLED_AND_DISABLED) && \
			(ON_TOMO_IN_signal.signal_logic_level == LOGIC_LEVEL_HIGH))
		{
			device_current_state = DEVICE_ERROR;										// переключаем устройство в состояние ошибки
			error_code = ON_TOMO_BUCKY_CALL_ERROR;										// выставляем ошибку (BUCKY_CALL выключился прежде, чем ON_TOMO включился повторно)
			bucky_ready_response_set(LOGIC_LEVEL_LOW);									// выключаем сигнал BUCKY_READY
			motor.motor_movement_purpose = MOTOR_PURPOSE_TAKE_INITIAL_POSITION;			// выставляем назначение движения - двигаться в начальное положение
		}
		break;
	}
	case MOTOR_PURPOSE_TAKE_INITIAL_POSITION:											// если назначение движения - вернуться в начальную позицию
	{
		if(!(limit_switch_return_state()))												// если концевик не активен
		{
			motor_make_step_to_direction(MOVE_TO_COORD_ORIGIN);							// делаем шаг в направлении начального положения
		}
		else
		{
			motor_movement_complete();													// иначе завершаем движение
		}
		break;
	}
	}
}

void motor_make_one_step(void)
{
	bucky_ready_response_check();														// проверяем, надо ли выставить сигнал BUCKY_READY в "1"
	motor_check_conditions_and_step();
	calculate_ticks_per_next_step();
}

/*
 * Обработчик прерываний таймера, отвечающего за шаги мотора
 */
void motor_timer_interrupt_handler(void)
{
	ticks_since_start_movement_counter++;
	ticks_before_next_step_counter--;
	if ((EXPOSITITON_MOVEMENT_STEPS - steps_since_start_movement_counter) >= steps_for_acceleration_counter)
	{
		if (steps_per_sec < MAX_STEPS_PER_SEC_MODE_00)
		ticks_for_acceleration_counter++;
	}
	else
	{
		ticks_for_acceleration_counter--;
	}

	if (ticks_before_next_step_counter <= 0)
	{
		motor_make_one_step();
	}
}

/*
 * Циклическое движение мотора в режиме экспозиции
 */
void cyclic_movement_step(void)
{
	if (motor.steps_distance_from_limit_switch <= 0)					// если мы в крайней точке точке, ближайшей к начальному положению
	{
		motor.exposition_movement_direction = EXPOSITION_MOVEMENT_FROM_INITIAL_POSITION;			// выставляем флаг движения от начального положения
		motor_make_step_to_direction(MOVE_TO_COORD_END);											// делаем шаг в сторону от начального положения
		reset_movement_counters();
	}
	/*
	 * если мы находимся в промежутке между крайними положениями растра (ближнее и дальнее)
	 */
	if ((motor.steps_distance_from_limit_switch > 0) && \
			(motor.steps_distance_from_limit_switch < EXPOSITITON_MOVEMENT_STEPS))
	{
		if (motor.exposition_movement_direction == EXPOSITION_MOVEMENT_FROM_INITIAL_POSITION)		// если выставлен флаг движения от начального положения
		{
			motor_make_step_to_direction(MOVE_TO_COORD_END);										// делаем шаг от начального положения
		}
		else
		{
			motor_make_step_to_direction(MOVE_TO_COORD_ORIGIN);										// иначе делаем шаг в сторону начального положения
		}
	}
	if (motor.steps_distance_from_limit_switch >= EXPOSITITON_MOVEMENT_STEPS)						// если мы в крайней точке, дальней от начального положения
	{
		motor.exposition_movement_direction = ON_TOMO_MOVEMENT_TO_INITIAL_POSITION;					// выставляем флаг движения к начальному положению
		motor_make_step_to_direction(MOVE_TO_COORD_ORIGIN);											// делаем шаг в сторону начального положения
		reset_movement_counters();
	}
}

/*
 * выставляем пин направления мотора
 */
void motor_direction_pin_set()
{
	switch (motor.motor_move_direction)																// если направление движения
	{
	case MOVE_TO_COORD_ORIGIN:																		// если направление движения к начальному положению
	{
		if (DIR_PIN_LOGIC_LEVEL_INVERTED)		// !! ifdef											// если логический уровень направления инвертирован аппаратно
		{
			HAL_GPIO_WritePin(MOTOR_DIR_OUT_PORT, MOTOR_DIR_OUT_PIN, GPIO_PIN_RESET);				// выставляем "0" на пине направления
		}
		else
		{
			HAL_GPIO_WritePin(MOTOR_DIR_OUT_PORT, MOTOR_DIR_OUT_PIN, GPIO_PIN_SET);					// иначе выставляем "1" на пине направления
		}
		break;
	}
	case MOVE_TO_COORD_END:																			// если направление движения от начального положения
	{
		if (DIR_PIN_LOGIC_LEVEL_INVERTED)															// если логический уровень направления инвертирован аппаратно
		{
			HAL_GPIO_WritePin(MOTOR_DIR_OUT_PORT, MOTOR_DIR_OUT_PIN, GPIO_PIN_SET);					// выставляем "1" на пине направления
		}
		else
		{
			HAL_GPIO_WritePin(MOTOR_DIR_OUT_PORT, MOTOR_DIR_OUT_PIN, GPIO_PIN_RESET);				// иначе выставляем "0" на пине направления
		}
		break;
	}
	}
}

/*
 * проверяем состояние концевика и совершаем шаг
 */
void check_limit_switch_and_make_step()
{
	/*
	 * если направление движения к начальному положению, и концевик не активен, и не пройдено аварийное количество шагов к начальному положению
	 */
	if ((motor.motor_move_direction == MOVE_TO_COORD_ORIGIN) && \
		(!(limit_switch_return_state())) && \
		(motor.limit_emergency_counter < EMERGENCY_STEPS_TO_LIMIT))
	{
		step_toggle();																				// совершаем шаг
		motor.steps_distance_from_limit_switch = motor.steps_distance_from_limit_switch - 1;		// декрементируем счётчик расстояния от начального положения
		motor.limit_emergency_counter = motor.limit_emergency_counter + 1;							// инкрементируем аварийный счётчик шагов
	}
	if (motor.motor_move_direction == MOVE_TO_COORD_END)											// если направлениение движения от начального положения
	{
		step_toggle();																				// совершаем шаг
		motor.steps_distance_from_limit_switch = motor.steps_distance_from_limit_switch + 1;		// инкрементируем счётчик расстояния от начального положения
		motor.limit_emergency_counter = 0;															// обнуляем аварийный счётчик шагов
	}
}

/*
 * совершаем шаг
 */
void step_toggle()
{
	switch (motor.step_pin_current_phase)															// если текущее логическое состояние на пине шага
	{
	case STEP_LOW_PHASE:																			// если текущеее логическое состояние "0"
	{
		HAL_GPIO_WritePin(MOTOR_STEP_OUT_PORT, MOTOR_STEP_OUT_PIN, GPIO_PIN_SET);					// выставляем "1" на пине шага
		motor.step_pin_current_phase = STEP_HIGH_PHASE;												// выставляем флаг, что пин шага находится в логическом состоянии "1"
		break;
	}
	case STEP_HIGH_PHASE:																			// если текущеее логическое состояние "1"
	{
		HAL_GPIO_WritePin(MOTOR_STEP_OUT_PORT, MOTOR_STEP_OUT_PIN, GPIO_PIN_RESET);					// выставляем "0" на пине шага
		motor.step_pin_current_phase = STEP_LOW_PHASE;												// выставляем флаг, что пин шага находится в логическом состоянии "0"
		break;
	}
	}
}

/*
 * опрашиваем и возрващаем состояние концевика
 */
_Bool limit_switch_return_state()
{
	_Bool current_state;																			// флаг состояния концевика
	check_input_signal_state(&limit_switch.GRID_END_POINT_IN_signal);								// опрашиваем состояние пина концевика
	if (LIMIT_SWITCH_LOGIC_LEVEL_INVERTED)															// если логический уровень концевика инвертирован аппаратно
	{
		if (limit_switch.GRID_END_POINT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW)			// если на пине концевика "0"
		{
			current_state = 1;																		// выставляем флаг концевика в "1"
			motor.steps_distance_from_limit_switch = 0;												// обнуляем счётчик расстояния до концевика
		}
		else
		{
			current_state = 0;																		// иначе выставляем флаг концевика в "0"
		}
	}
	else
	{
		if (limit_switch.GRID_END_POINT_IN_signal.signal_logic_level == LOGIC_LEVEL_LOW)			// иначе если на пине концевика "0"
		{
			current_state = 0;																		// выставляем флаг концевика в "0"
		}
		else
		{
			current_state = 1;																		// иначе выставляем флаг концевика в "1"
			motor.steps_distance_from_limit_switch = 0;												// обнуляем счётчик расстояния до концевика
		}
	}
	return current_state;																			// возвращаем флаг состояния концевика
}
