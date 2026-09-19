/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "cordic.h"
#include "hrtim.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the mapping constants based on a 170MHz / 4 (42.5MHz) HRTIM clock
#define ADC_MIN 0
#define ADC_MAX 4095
#define HRTIM_TICKS_MIN 425      // ~100kHz BBD clock (42.5MHz / 425)
#define HRTIM_TICKS_MAX 10625    // ~4kHz BBD clock (42.5MHz / 10625)

/* --- Wow & Flutter LFO rate ----------------------------------------------
 * The LFO phase accumulator is 32 bits and one full wrap is exactly one
 * sine cycle, so the per-tick step for a given rate is
 *
 *     step = 2^32 * f_lfo / f_tick        (f_tick = 1 kHz, the TIM6 rate)
 *
 * The previous value (100 + rate*5) peaked at 20575, i.e. one cycle every
 * 209 seconds, which is far too slow to hear as modulation.
 */
#define LFO_STEP_MIN      429497u     // 0.1 Hz - slow tape "wow"
#define LFO_STEP_MAX      42949673u   // 10 Hz  - fast "flutter"
#define LFO_STEP_PER_LSB  ((LFO_STEP_MAX - LFO_STEP_MIN) / ADC_MAX)   // 10383
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ==============================================================================
 *                         DSP & CONTROL VARIABLES
 * ============================================================================== */

/* --- Potentiometer Readings (0-4095) --- */
uint16_t time_pot_value = 0;       // Controls the main BBD delay time
uint16_t pot_mod_rate = 0;         // Controls the speed of the Wow & Flutter LFO
uint16_t pot_mod_depth = 0;        // Controls the intensity of the pitch modulation

/* --- Delay State --- */
// Base delay time calculated from the Time pot (before LFO modulation)
uint32_t current_base_hrtim_period = HRTIM_TICKS_MAX;

/* --- LFO State --- */
// Phase tracking for the CORDIC sine wave generator
uint32_t lfo_phase_accumulator = 0;

/* --- Tap Tempo State --- */
uint8_t tap_mode_active = 0;       // 1 if Tap controls delay, 0 if Pot controls delay
uint16_t last_time_pot_value = 0;  // Used to detect when the user turns the knob to override tap

#define MAX_TAP_SAMPLES 4
uint32_t tap_intervals[MAX_TAP_SAMPLES] = {0};
uint8_t tap_count = 0;
uint32_t ms_since_last_tap = 0;
uint8_t button_history = 0x00;     // Shift register for debouncing PC13 (Starts 0 for active high)
uint8_t button_pressed = 0;
uint8_t tap_averaging_complete = 0; // Ensures we only update tempo once after tapping stops

// LED Blink state
uint32_t led_blink_counter = 0;
uint32_t current_tempo_ms = 500;   // Default to 500ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t ADC1_Read_Channel(uint32_t channel);
void HRTIM_Update_Frequency(uint32_t targetPeriodTicks);
void Update_Delay_From_Pot_Audio_Taper(uint16_t adc_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==============================================================================
 *                          CUSTOM DSP & HARDWARE FUNCTIONS
 * ============================================================================== */

/**
  * @brief  Reads a single ADC channel by reconfiguring the multiplexer on the fly.
  *         This allows polling multiple single-ended channels on the same ADC without DMA.
  * @param  channel The ADC channel macro (e.g., ADC_CHANNEL_1)
  * @retval 12-bit ADC value (0-4095)
  */
uint32_t ADC1_Read_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  // 247.5 cycles at the 42.5 MHz ADC clock is ~5.8us of sampling. A pot
  // presents up to a quarter of its track resistance as source impedance
  // (2.5k for a 10k pot, 25k for a 100k), and the old 2.5 cycles gave only
  // 59ns - far too short to charge the sampling cap, which showed up as
  // noisy readings and crosstalk between the three channels.
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  
  // Try to configure. If it fails, return 0 instead of freezing the whole MCU
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return 0; 
  }

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
      HAL_ADC_Stop(&hadc1);
      return 0;
  }
  
  uint32_t value = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1); // STOP the ADC so it can be reconfigured for the next channel!
  return value;
}

/**
  * @brief  Updates the HRTIM Master Period and Compare registers safely.
  *         Ensures the duty cycle remains exactly 50%.
  * @param  targetPeriodTicks The new total cycle duration in HRTIM ticks.
  */
void HRTIM_Update_Frequency(uint32_t targetPeriodTicks)
{
    if (targetPeriodTicks % 2 != 0)
    {
        targetPeriodTicks++;
    }
    // Update Timer A directly (it's driving our pin)
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, targetPeriodTicks);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, (targetPeriodTicks / 2));
    
    // Also update Master just to keep them in sync
    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, targetPeriodTicks);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, (targetPeriodTicks / 2));

    // Force an immediate update so the new frequency takes effect instantly!
    HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_MASTER);
}

/**
  * @brief  Reads the Time ADC value, applies a quadratic audio taper curve,
  *         and updates the base HRTIM frequency.
  * @param  adc_value The raw 12-bit ADC reading (0-4095).
  */
void Update_Delay_From_Pot_Audio_Taper(uint16_t adc_value)
{
    if(adc_value > ADC_MAX) adc_value = ADC_MAX;
    uint32_t tapered_adc = ((uint32_t)adc_value * (uint32_t)adc_value) / ADC_MAX;
    uint32_t target_ticks = HRTIM_TICKS_MIN +
                           ((uint64_t)(tapered_adc) * (HRTIM_TICKS_MAX - HRTIM_TICKS_MIN))
                           / ADC_MAX;

    current_base_hrtim_period = target_ticks;
    HRTIM_Update_Frequency(target_ticks);
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
  MX_LPUART1_UART_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // FIX: Force TA1 to toggle based on its OWN Timer A events (50% duty cycle)
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg);
  // Start the HRTIM Master and Timer A (BBD Clock Output on TA1)
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1);
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A);

  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  
  // Start the LFO timer (1kHz interrupt)
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* 1. Poll the Hardware Potentiometers */
      time_pot_value = ADC1_Read_Channel(ADC_CHANNEL_1);   // PA0: Delay Time
      pot_mod_rate   = ADC1_Read_Channel(ADC_CHANNEL_2);   // PA1: LFO Rate
      pot_mod_depth  = ADC1_Read_Channel(ADC_CHANNEL_15);  // PB0: LFO Depth

      /* 2. Pot vs Tap Priority Logic */
      // If the user turns the knob by more than ~2.5% (100 units), override Tap Mode
      int pot_diff = (int)time_pot_value - (int)last_time_pot_value;
      if (pot_diff < 0) pot_diff = -pot_diff;
      
      if (pot_diff > 100)
      {
          tap_mode_active = 0;
          last_time_pot_value = time_pot_value;
      }

      // Only follow the physical knob if we are NOT in Tap Mode
      if (tap_mode_active == 0)
      {
          Update_Delay_From_Pot_Audio_Taper(time_pot_value);
      }

      /* 3. Toggle PA5 (Heartbeat LED) */
      HAL_GPIO_TogglePin(LED_OUT_GPIO_Port, LED_OUT_Pin);

      /* 4. Small delay to prevent thrashing the ADC/Timer registers */
      HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* ============================================================================== *
 * ============================================================================== */

/**
  * @brief  1kHz LFO Interrupt (TIM6). Generates Wow & Flutter via CORDIC.
  *         This callback runs 1000 times a second.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* --- 1. Tap Tempo Button Debounce (PC13) --- */
        // Read PC13, shift into history (1 = pressed because it is Active High)
        button_history = (button_history << 1) | (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET ? 1 : 0);
        
        // If history is 0x0F (00001111), it means it was just solidly pressed for 4ms
        if (button_history == 0x0F && !button_pressed)
        {
            button_pressed = 1;
            
            // If it's been more than 2000ms since the last tap, restart the sequence
            if (ms_since_last_tap > 2000)
            {
                tap_count = 0;
            }
            
            if (tap_count > 0 && tap_count <= MAX_TAP_SAMPLES)
            {
                tap_intervals[tap_count - 1] = ms_since_last_tap;
            }
            
            if (tap_count < MAX_TAP_SAMPLES + 1) tap_count++;
            
            ms_since_last_tap = 0;
            tap_averaging_complete = 0; // Wait for tapping to stop
        }
        else if (button_history == 0xF0) // Solidly released for 4ms
        {
            button_pressed = 0;
        }

        /* --- 2. Tap Timeout & Averaging --- */
        if (ms_since_last_tap < 5000) ms_since_last_tap++;
        
        // If user hasn't tapped for 1500ms, and we have gathered taps, calculate the new tempo!
        if (ms_since_last_tap > 1500 && tap_count > 1 && !tap_averaging_complete)
        {
            uint32_t total_ms = 0;
            int valid_taps = tap_count - 1;
            for (int i = 0; i < valid_taps; i++)
            {
                total_ms += tap_intervals[i];
            }
            uint32_t avg_ms = total_ms / valid_taps;
            
            // Limit to physical pedal bounds (20ms to 600ms)
            if (avg_ms < 20) avg_ms = 20;
            if (avg_ms > 600) avg_ms = 600;
            
            current_tempo_ms = avg_ms;
            
            // Convert tapped tempo (ms) to BBD HRTIM ticks
            // Formula: ticks = avg_ms * (42.5MHz / 2048000) = avg_ms * 20.7519f
            uint32_t target_ticks = (uint32_t)((float)avg_ms * 20.7519f);
            
            if(target_ticks < HRTIM_TICKS_MIN) target_ticks = HRTIM_TICKS_MIN;
            if(target_ticks > HRTIM_TICKS_MAX) target_ticks = HRTIM_TICKS_MAX;
            
            current_base_hrtim_period = target_ticks;
            tap_mode_active = 1;         // Lock into Tap Mode
            tap_averaging_complete = 1;  // Only calculate once per tap sequence
            
            // Immediately sync the LED blink to the new beat
            led_blink_counter = 0; 
        }

        /* --- 3. LED Tempo Blinker (PA5) --- */
        led_blink_counter++;
        if (led_blink_counter >= current_tempo_ms)
        {
            led_blink_counter = 0;
        }
        // Turn LED ON for the first 50ms of every beat
        if (led_blink_counter < 50)
        {
            HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);
        }

        /* --- 4. Wow & Flutter LFO --- */
        /* Quadratic taper on the Rate pot, matching the Time pot idiom, so the
         * slow wow rates occupy most of the knob travel instead of being
         * squeezed into the first few percent. */
        uint32_t tapered_rate = ((uint32_t)pot_mod_rate * (uint32_t)pot_mod_rate) / ADC_MAX;
        uint32_t phase_step   = LFO_STEP_MIN + (tapered_rate * LFO_STEP_PER_LSB);
        lfo_phase_accumulator += phase_step;

        int32_t cordic_input = (int32_t)lfo_phase_accumulator;
        int32_t cordic_output;

        HAL_CORDIC_Calculate(&hcordic, &cordic_input, &cordic_output, 1, 0);

        float sine_wave = (float)cordic_output / 2147483648.0f;

        // Modulate up to +/- 10% of the current period to prevent underflow at high frequencies
        float max_tick_deviation = (pot_mod_depth / 4095.0f) * (current_base_hrtim_period * 0.1f);

        int32_t tick_modulation = (int32_t)(sine_wave * max_tick_deviation);

        uint32_t modulated_hrtim_period = current_base_hrtim_period + tick_modulation;

        HRTIM_Update_Frequency(modulated_hrtim_period);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // Blink PA5 rapidly if we hit an error!
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = LED_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_OUT_GPIO_Port, &GPIO_InitStruct);

  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(LED_OUT_GPIO_Port, LED_OUT_Pin);
      for(volatile int i=0; i<500000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
