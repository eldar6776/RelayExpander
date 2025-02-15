/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Struktura za pracenje stanja releja
typedef struct {
    uint8_t last_state;
    uint32_t timer;  // Vreme kada je relej poslednji put pre�ao u PWM
} RelayState;

RelayState relayStates[8];  // Struktura za 8 releja
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIMRXBUF_SIZE 16
#define NUMBER_OF_RELAY 8
#define MENU_TIMEOUT 10000 // 10 sekundi timeout
#define SHORT_PRESS_MIN 10  // 10 ms
#define SHORT_PRESS_MAX 200 // 200 ms
#define LONG_PRESS_MIN 1000 // 1 sekunda
#define RESET_PRESS_MIN 5000 // 5 sekunda
#define LONG_PRESS_INTERVAL 5 // Brzina promjene kod dugog pritiska
#define INCREMENT_STEP 5   // Korak inkrementacije za dugi pritisak
#define TOGGLE_INTERVAL 200 // Period treptanja LED
#define ERROR_TOGGLE_INTERVAL 50 // Period treptanja LED kod greï¿½ke
#define BUTTON_PAUSE    200 // dodatni debouncing 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
TinyFrame tfapp;

GPIO_TypeDef *LED_PORT[8] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB};
uint16_t LED_PIN[8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,GPIO_PIN_4 ,GPIO_PIN_5 ,GPIO_PIN_4 ,GPIO_PIN_5};

volatile uint8_t selected_channel = 7;
volatile uint8_t menu_activ = 0;
volatile uint32_t menu_timer = 0;
volatile uint8_t toggle_state = 0;
volatile uint8_t error_state = 0;
volatile uint32_t last_press_time = 0;
volatile uint32_t last_toggle_time = 0;
volatile uint32_t last_check_time = 0;
volatile uint32_t last_err_toggle_time = 0;
volatile uint32_t last_increment_time = 0;
uint8_t button_select_prev = 0, button_onoff_prev = 0;

// Definicije za stanja releja
uint8_t relay_states[8] = {0};      // Trenutnostanje releja
uint8_t relay_old_states[8] = {0};  // Flagovi promjene stanja releja
uint8_t relay_duty_cycle = 50;      // Pocetni duty cycle (50%)

static uint32_t current_tick = 0;   // trenutni vremenski otisak
static uint32_t pause_tick = 0;     // vremenski otisak za pauzu od 10000ms
static uint32_t pause = 100;
static uint8_t i = 0;               // trenutni broj releja
static uint8_t t = 0;               // stanje releja (0 ili 1)

static uint8_t my_address = 0;
static uint16_t rel_start = 0;
static uint16_t rel_end = 0;
bool init_tf = false;
bool restart = false;
uint8_t rec, multiplier = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void RS485_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  staticka inline funkcija pauze 1~2ms za kašnjenje odgovora za stabilne repeatere
  *         u interruptu se ne može koristiti HAL_GetTick ni HAL_Delay tako da ova funkcija
  *         obezbjeduje kašnjenje oko 1 ms bez hal drivera gubljenjem vremena izvršavanjem
  *         ciklusa koji ne rade ništa
  * @param
  * @retval
  */
static inline void delay_us(uint32_t us) {
    volatile uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 20000000) * us;  // Smanjen faktor
    while (cycles--) {
        __asm volatile("NOP");  // Izbegava optimizaciju
    }
}
/**
  * @brief  mali efekt sa LED diodama prednjeg panela kod ukljucenja
  * @param
  * @retval
  */
void start_effect(void) {
    const uint8_t order[][2] = {
        {4, 5}, {3, 6}, {2, 7}, {1, 8}, // Paljenje
        {4, 5}, {3, 6}, {2, 7}, {1, 8}  // Gašenje
    };
    HAL_Delay(300);
    for (int i = 0; i < 8; i++) {
        GPIO_PinState state = (i < 4) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(LED_PORT[order[i][0] - 1], LED_PIN[order[i][0] - 1], state);
        HAL_GPIO_WritePin(LED_PORT[order[i][1] - 1], LED_PIN[order[i][1] - 1], state);
        HAL_Delay(50);
#ifdef USE_WATCHDOG
        HAL_IWDG_Refresh(&hiwdg);
#endif
    }
}
/**
  * @brief  podesi adresu modula dimera na busu prema poziciji DIP switcha
  * @param
  * @retval
  */
void get_address(void)
{
    my_address = 0;
    if ((HAL_GPIO_ReadPin(ADDRESS_0_GPIO_Port, ADDRESS_0_Pin) == GPIO_PIN_RESET))   my_address |= 0x01;
    if ((HAL_GPIO_ReadPin(ADDRESS_1_GPIO_Port, ADDRESS_1_Pin) == GPIO_PIN_RESET))   my_address |= 0x02;
    if ((HAL_GPIO_ReadPin(ADDRESS_2_GPIO_Port, ADDRESS_2_Pin) == GPIO_PIN_RESET))   my_address |= 0x04;
    if ((HAL_GPIO_ReadPin(ADDRESS_3_GPIO_Port, ADDRESS_3_Pin) == GPIO_PIN_RESET))   my_address |= 0x08;
    if ((HAL_GPIO_ReadPin(ADDRESS_4_GPIO_Port, ADDRESS_4_Pin) == GPIO_PIN_RESET))   my_address |= 0x10;
    if ((HAL_GPIO_ReadPin(ADDRESS_5_GPIO_Port, ADDRESS_5_Pin) == GPIO_PIN_RESET))   my_address |= 0x20;
    if(my_address)
    {
        rel_start = (NUMBER_OF_RELAY*multiplier*(my_address-1))+1;
        rel_end = rel_start + (NUMBER_OF_RELAY*multiplier)-1;
    }
}
/**
  * @brief  funkcija za kontrolu releja koja kod uključenja relej starta sa punim izlazom a nakon
  *         inicijalnog stanja prelazi u 60% pwm mod, tako da je grijanje releja svedeno na minimum
  * @param
  * @retval
  */
void relay_update(uint8_t relay_num, uint8_t state)
{
    static uint32_t last_state[8] = {0};        // flag prošlog stanaja tako da sam o prva promjena utiče na funkciju
    static uint32_t last_change_time[8] = {0};  // Čuvanje vremena poslednje promene stanja za svaki relej
    static uint32_t pwm_duty_cycle[8] = {0};  // Početni duty cycle za svaki relej (100% kada je uključen)
    uint32_t current_tick = HAL_GetTick();      // Vreme u milisekundama

    // Proveri sve releje svaki put kada se funkcija pozove
    for (int i = 0; i < 8; i++) {
        // Ako je prošlo više od 1000 ms (1 sekunda) od poslednje promene za taj relej
        if (current_tick - last_change_time[i] >= 1000) {
            if (pwm_duty_cycle[i] == 100) {
                pwm_duty_cycle[i] = 60;  // Prebaci PWM na 60% nakon 1 sekunde
            }
        }
    }

    // Ako je stanje 1 (uključi relej)
    if ((state == 1) && !last_state[relay_num]) {
        last_state[relay_num] = state;
        last_change_time[relay_num] = current_tick;  // Ažuriraj poslednje vreme kada je stanje promenjeno
        pwm_duty_cycle[relay_num] = 100;  // PWM na 100%
    }
    // Ako je stanje 0 (isključi relej)
    else if ((state == 0) && last_state[relay_num]) {
        last_state[relay_num] = state;
        pwm_duty_cycle[relay_num] = 0;  // Postavi PWM na 0%
        last_change_time[relay_num] = current_tick;  // Ažuriraj poslednje vreme kada je stanje promenjeno
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (htim1.Init.Period + 1) * pwm_duty_cycle[0] / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (htim1.Init.Period + 1) * pwm_duty_cycle[1] / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (htim1.Init.Period + 1) * pwm_duty_cycle[2] / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (htim1.Init.Period + 1) * pwm_duty_cycle[3] / 100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (htim3.Init.Period + 1) * pwm_duty_cycle[4] / 100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (htim3.Init.Period + 1) * pwm_duty_cycle[5] / 100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (htim3.Init.Period + 1) * pwm_duty_cycle[6] / 100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (htim3.Init.Period + 1) * pwm_duty_cycle[7] / 100);

    for(int i = 0; i < 8; i++)
    {
        relay_states[i] = (pwm_duty_cycle[i] > 0) ? 1:0;

    }
}
/**
  * @brief  kopira stanje izlaza na LED prednjeg panela
  * @param
  * @retval
  */
void refresh_led(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], (relay_states[i] > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
/**
  * @brief  reaguje na pritisak tastera na prednjem panelu modula
  * @param
  * @retval
  */
void handle_buttons(void)
{
    uint32_t current_time = HAL_GetTick();
    uint8_t button_select = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    uint8_t button_onoff = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

    if (button_select == GPIO_PIN_RESET) {
        if (!button_select_prev) {
            button_select_prev = 1; // lock press time load
            menu_activ = 1;
            toggle_state = 1;
            if(++selected_channel > 7) selected_channel = 0;
            menu_timer = current_time + MENU_TIMEOUT;
            last_press_time = current_time; // load first press time
            refresh_led();
            get_address();
            HAL_Delay(BUTTON_PAUSE);
        }
        else if (error_state && (current_time - last_press_time > RESET_PRESS_MIN))
        {

        }
    }
    else if ((button_select == GPIO_PIN_SET) && button_select_prev)
    {
        button_select_prev = 0;
        HAL_Delay(BUTTON_PAUSE);
        refresh_led();
    }

    if (menu_activ && (button_onoff == GPIO_PIN_RESET)) {
        if (!button_onoff_prev) {
            button_onoff_prev = 1; // lock press time load
            last_press_time = current_time; // load first press time
            if(relay_states[selected_channel] > 0) relay_states[selected_channel] = 0;
            else relay_states[selected_channel] = 1;

            HAL_Delay(BUTTON_PAUSE);
        }
        menu_timer = current_time + MENU_TIMEOUT;
    }
    else if ((button_onoff == GPIO_PIN_SET) && button_onoff_prev)
    {
        button_onoff_prev = 0; // release button press
        HAL_Delay(BUTTON_PAUSE);
    }
}
/**
  * @brief  logika korisnickog menija sa led diodama i tasterima na prednjem panelu modula
  * @param
  * @retval
  */
void menu_logic(void)
{
    uint32_t current_time = HAL_GetTick();

    if (current_time > menu_timer) {
        selected_channel = 7;
        menu_activ = 0;
        toggle_state = 0;
        refresh_led();
    }

    // Definiši vremenske intervale zavisno od relay_states
    uint32_t on_time, off_time;
    if (relay_states[selected_channel] == 1) {
        on_time = TOGGLE_INTERVAL * 3;  // LED ON 3x duže
        off_time = TOGGLE_INTERVAL;     // LED OFF normalno
    } else {
        on_time = TOGGLE_INTERVAL;      // LED ON normalno
        off_time = TOGGLE_INTERVAL * 3; // LED OFF 75% duže
    }

    uint32_t toggle_period = toggle_state ? on_time : off_time;

    if (current_time - last_toggle_time >= toggle_period) {
        HAL_GPIO_TogglePin(LED_PORT[selected_channel], LED_PIN[selected_channel]);
        toggle_state = !toggle_state;  // Promena stanja
        last_toggle_time = current_time;
    }
}
/**
  * @brief  funkcija obezbjeđuje ispravan poziv update releja kod 
  *         brze izmjene stanja sa busa, svaki relej će preći iz 
  *         100% u 60% pwm mod kada je uključen
  * @param
  * @retval
  */
void set_relay(void)
{
    static uint32_t set_tmr = 0;
    for(int i = 0; i < NUMBER_OF_RELAY; i++)
    {
        if(relay_states[i] != relay_old_states[i])
        {
            set_tmr = HAL_GetTick();
            relay_old_states[i] = relay_states[i];
            relay_update(i, relay_states[i]);
        }
    }
    if(set_tmr && ((HAL_GetTick() - set_tmr) >= 1000))
    {
        set_tmr = 0;
        for(int i = 0; i < NUMBER_OF_RELAY; i++)
        {
            relay_update(i, relay_states[i]);
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CRC_Init();
    MX_I2C1_Init();
    MX_IWDG_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    RS485_Init();
    start_effect();
    start_effect();
    start_effect();
    get_address();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if(restart == true) Error_Handler();
        handle_buttons();
        menu_logic();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        set_relay();

#ifdef USE_WATCHDOG
        HAL_IWDG_Refresh(&hiwdg);
#endif
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x0010020A;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

    /* USER CODE BEGIN IWDG_Init 0 */
#ifdef USE_WATCHDOG
    /* USER CODE END IWDG_Init 0 */

    /* USER CODE BEGIN IWDG_Init 1 */

    /* USER CODE END IWDG_Init 1 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN IWDG_Init 2 */
#endif
    /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 150;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */
    // Pokretanje PWM-a
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 150;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                      |LED5_Pin|LED6_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED7_Pin|LED8_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                             LED5_Pin LED6_Pin */
    GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : BUTTON_SELECT_Pin ADDRESS_0_Pin ADDRESS_1_Pin ADDRESS_2_Pin
                             ADDRESS_3_Pin ADDRESS_4_Pin ADDRESS_5_Pin BUTTON_ON_OFF_Pin */
    GPIO_InitStruct.Pin = BUTTON_SELECT_Pin|ADDRESS_0_Pin|ADDRESS_1_Pin|ADDRESS_2_Pin
                          |ADDRESS_3_Pin|ADDRESS_4_Pin|ADDRESS_5_Pin|BUTTON_ON_OFF_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : LED7_Pin LED8_Pin */
    GPIO_InitStruct.Pin = LED7_Pin|LED8_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Sluša bus za paket sa BINARY_SET tipom i provjerava adresirani izlaz 
  *         ako je u sopstvenom opsegu reaguje na ostale ne odgovara
  * @param
  * @retval
  */
TF_Result BINARY_SET_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t res = 0;
    uint16_t adr = (uint16_t) (msg->data[0] << 8) | msg->data[1]; // Kombinacija dva bajta u adresu
    uint8_t idx = adr - rel_start; // Izračunavanje indeksa releja
    uint8_t ret[4]; // Niz za vraćanje stanja

    // Proveravamo da li je adresa unutar opsega rel_start i rel_end
    if (my_address && adr && (adr >= rel_start) && (adr <= rel_end)) {

        if (msg->data[2] == 0x01) {
            relay_states[idx] = 1;  // Uključivanje releja
        } else if (msg->data[2] == 0x02) {
            relay_states[idx] = 0;  // Isključivanje releja
        }
        res++;  // Povećavamo broj uspešnih operacija
    }

    // Ako je došlo do promene stanja, vraćamo novo stanje
    if (res == 1) {
        ret[0] = msg->data[0];
        ret[1] = msg->data[1];
        if (relay_states[idx] == 1) {
            ret[2] = 1;  // Stanje 1 (ON)
        } else {
            ret[2] = 2;  // Stanje 2 (OFF)
        }
        ret[3] = ACK;
        msg->data = ret;  // Postavljamo novi niz za odgovor
        msg->len = 4;  // Dužina odgovora je 4
        TF_Respond(tf, msg);  // Odgovaramo sa novim stanjem
    }

    return TF_STAY;  // Održavanje trenutnog stanja
}
/**
  * @brief  Sluša bus za paket sa BINARY_GET tipom i provjerava adresirani izlaz 
  *         ako je u sopstvenom opsegu odgovara sa stanjem na ostale ne odgovara
  * @param
  * @retval
  */
TF_Result BINARY_GET_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint16_t adr = (uint16_t) (msg->data[0] << 8) | msg->data[1]; // Kombinacija dva bajta u adresu
    uint8_t idx = adr - rel_start; // Izračunavanje indeksa releja
    uint8_t ret[3]; // Niz za vraćanje stanja

    // Proveravamo da li je adresa unutar opsega rel_start i rel_end
    if (my_address && adr && (adr >= rel_start) && (adr <= rel_end)) {
        ret[0] = msg->data[0];
        ret[1] = msg->data[1];
        if (relay_states[idx] == 1) {
            ret[2] = 1;  // Stanje 1 (ON)
        } else {
            ret[2] = 2;  // Stanje 2 (OFF)
        }
        msg->data = ret;  // Postavljamo novi niz za odgovor
        msg->len = 3;  // Dužina odgovora je 3
        TF_Respond(tf, msg);  // Odgovaramo sa novim stanjem
    }

    return TF_STAY;  // Održavanje trenutnog stanja
}
/**
  * @brief  Sluša bus za paket sa BINARY_RESET tipom i provjerava adresirani izlaz 
  *         ako je u sopstvenom opsegu setuje flag odgovara i nakon odogovora pokreće reset
  * @param
  * @retval
  */
TF_Result BINARY_RESET_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint16_t adr = (uint16_t) (msg->data[0] << 8) | msg->data[1]; // Kombinacija dva bajta u adresu
    uint8_t idx = adr - rel_start; // Izračunavanje indeksa releja
    uint8_t ret[3]; // Niz za vraćanje stanja

    // Proveravamo da li je adresa unutar opsega rel_start i rel_end
    if (my_address && adr && (adr >= rel_start) && (adr <= rel_end)) {
        restart = true; // setujemo flag a ne restartujemo odmah da bi vratili odgovor
        ret[0] = msg->data[0];
        ret[1] = msg->data[1];
        ret[2] = ACK; 
        msg->data = ret;  // Postavljamo novi niz za odgovor
        msg->len = 3;  // Dužina odgovora je 3
        TF_Respond(tf, msg);  // Odgovaramo sa novim stanjem
    }

    return TF_STAY;  // Održavanje trenutnog stanja
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Init(void)
{
    if(!init_tf) {
        init_tf = TF_InitStatic(&tfapp, TF_SLAVE);
        TF_AddTypeListener(&tfapp, BINARY_GET, BINARY_GET_Listener);
        TF_AddTypeListener(&tfapp, BINARY_SET, BINARY_SET_Listener);
        TF_AddTypeListener(&tfapp, BINARY_RESET, BINARY_RESET_Listener);
    }
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Tick(void)
{
    if (init_tf == true) {
        TF_Tick(&tfapp);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    delay_us(2000); // puza kod odgovora zbog repeatera na linji 
    HAL_UART_Transmit(&huart1,(uint8_t*)buff, len, RESP_TOUT);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    TF_AcceptChar(&tfapp, rec);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    HAL_CRC_MspDeInit(&hcrc);
    HAL_I2C_MspDeInit(&hi2c1);
    HAL_TIM_Base_MspDeInit(&htim1);
    HAL_TIM_Base_MspDeInit(&htim3);
    HAL_UART_MspDeInit(&huart1);
    while (1)
    {
        HAL_NVIC_SystemReset();
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
