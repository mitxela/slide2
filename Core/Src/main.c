/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t calibrateMode = 0;
uint8_t AX12_TorqueStatus = 0;
uint16_t poweroff_timeout = 0;

#define NOTESTACK_LENGTH 128
uint8_t notestack[4][NOTESTACK_LENGTH] = {};
uint8_t nsp[4] = {0};

uint8_t lastnote[4]={72,82,72,82};

int8_t bend[4] = {0};

// units of 10ms
#define FAN_WARMUP_TIME 50
#define FAN_BOOST_SPEED 1200
uint16_t fan_target[4]={0};
uint8_t fan_warmup[4]={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void hang_error(){
  while (1){
    HAL_Delay(200);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}

void flash(){
  for (uint8_t i=0;i<10;i++) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);
  }
}

/*
  MIDI Channels are zero-indexed, uint8_t chan will be zero-indexed
  Whistles are indexed from 1, uint8_t n is 1 to 4
  AX12 IDs are 1 to 8, e.g. whistle 3 is servos 5 and 6
*/

#define set_valve_servo(n, x) \
  htim1.Instance->CCR ## n = (x)

void set_valve_open(uint8_t chan) {
  if (chan==0)      set_valve_servo(1, valve_open[0]);
  else if (chan==1) set_valve_servo(2, valve_open[1]);
  else if (chan==2) set_valve_servo(3, valve_open[2]);
  else if (chan==3) set_valve_servo(4, valve_open[3]);
}

void set_valve_closed(uint8_t chan) {
  if (chan==0)      set_valve_servo(1, valve_closed[0]);
  else if (chan==1) set_valve_servo(2, valve_closed[1]);
  else if (chan==2) set_valve_servo(3, valve_closed[2]);
  else if (chan==3) set_valve_servo(4, valve_closed[3]);
}

#define _set_fan_speed(n, x) \
  htim3.Instance->CCR ## n = 5000 - (x)

#define enable_fan_motors() \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)

#define disable_fan_motors() \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

void set_fan_speed(uint8_t n, uint16_t x){

  //*(&(htim3.Instance->CCR1) +4*n) = 5000 - x;

  enable_fan_motors();

  if (n==1)      _set_fan_speed(1, x);
  else if (n==2) _set_fan_speed(2, x);
  else if (n==3) _set_fan_speed(3, x);
  else if (n==4) _set_fan_speed(4, x);
}

void fan_speed_warmup( uint8_t n, uint16_t x ) {

  uint8_t a=n-1;
  fan_target[a] = x;

  if (fan_warmup[a] < FAN_WARMUP_TIME) {
    set_fan_speed(n, FAN_BOOST_SPEED);
  } else {
    set_fan_speed(n, x);
  }

}

#define AX12_Transmit(a) _AX12_Transmit( a, sizeof(a)-1 )

void _AX12_Transmit(uint8_t * data, uint8_t length){

  uint8_t sum=0;
  for (uint8_t i=2; i<length; i++){
    sum += data[i];
  }
  data[length] = ~sum;
  HAL_UART_Transmit(&huart2, data, length+1, 100);
}

void AX12_TorqueEnable(){
  uint8_t torqueEnable[] = { 0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x18, 0x01, 0 };
  AX12_Transmit( torqueEnable );
  AX12_TorqueStatus=1;
}
void AX12_TorqueDisable(){
  uint8_t torqueEnable[] = { 0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x18, 0x00, 0 };
  AX12_Transmit( torqueEnable );
  AX12_TorqueStatus=0;
}

void AX12_SetSlidePos(uint8_t id, uint16_t position){

  if (position > 0x330 || position < 0x230) return;//hang_error();
  if (!AX12_TorqueStatus) AX12_TorqueEnable();

  uint16_t antipos = 0x3FF - position;

  uint8_t setPosition[] = { 0xFF, 0xFF, 0xFE,
      0x0A, // Length
      0x83, // instruction sync write
      0x1E, // param 1
      0x02, // length of data
      id+1,
      position & 0xFF,
      position >> 8,
      id,
      antipos & 0xFF,
      antipos >> 8,
      0
  };

  AX12_Transmit(setPosition);
}

void setPitch(uint8_t chan, uint8_t note){

  if (note<69 || note >=89) return;

  //lookup in table
  uint16_t x = (note-67)*TABLE_SCALE + bend[chan];

  AX12_SetSlidePos( chan*2 +1 , 0x230 + mainLut[chan][ x ].angle );
  fan_speed_warmup( chan+1, mainLut[chan][ x ].speed );

}

void whistlePitchBend(uint8_t chan, uint8_t pb) {
  bend[chan] = pb-64;
  if (nsp[chan]>0) {
    setPitch(chan, notestack[chan][nsp[chan]-1]);
  }
}

void whistleNoteOn(uint8_t chan, uint8_t note) {

  if (nsp[chan]>=NOTESTACK_LENGTH) return;

  //push
  notestack[chan][nsp[chan]++] = note;
  lastnote[chan]=note;

  set_valve_open(chan);

  setPitch(chan, note);
  poweroff_timeout = 0;

}

void whistleNoteOff(uint8_t chan, uint8_t note) {

  // if note in notestack, remove it

  for (int i=nsp[chan]-1; i>=0; i--) {
    if (notestack[chan][i]==note) {
      for (uint8_t j=i; j<nsp[chan]; j++) {
        notestack[chan][j] = notestack[chan][j+1];
      }
      nsp[chan]--;
      //break;
    }
  }

  if (nsp[chan]>0) {
    setPitch(chan, notestack[chan][nsp[chan]-1]);
  } else {
    set_valve_closed(chan);

    // if all notes off, start timeout
    if (nsp[0]==0 && nsp[1]==0 && nsp[2]==0 && nsp[3]==0) {
      poweroff_timeout = 1000;
    }
  }

}

int abs(int x){
  return  (x<0) ? -x : x ;
}
#define isPlaying(chan) nsp[chan]>0

void autoNoteOn(uint8_t note, uint8_t vel) {
  if (note<69 || note >=89) return;

  uint8_t target =0;
  uint8_t lowestCost = 255;

  for (uint8_t chan=0; chan<4; chan++) {
    uint8_t cost = abs(lastnote[chan]-note);
    if (isPlaying(chan)) cost += 100;
    if (cost < lowestCost) {
      lowestCost = cost;
      target = chan;
    }
  }
  if (lowestCost<255) {
    whistleNoteOn(target, note);
  }
}

uint8_t notestackContains(uint8_t chan, uint8_t note){
  for (int8_t i=0; i<nsp[chan]; i++) {
    if (notestack[chan][i] == note) return 1;
  }
  return 0;
}

void autoNoteOff(uint8_t note) {

  // do a first pass to turn off notes that are overridden
  for (uint8_t chan=0; chan<4; chan++) {
    if (isPlaying(chan)) {
      if (lastnote[chan] != note && notestackContains(chan, note)) {
        whistleNoteOff(chan, note);
        return;
      }
    }
  }

  for (uint8_t chan=0; chan<4; chan++) {
    if (isPlaying(chan) && lastnote[chan]==note) {
      whistleNoteOff(chan, note);
    }
  }
}


void noteOn(uint8_t chan, uint8_t note, uint8_t vel) {
  if (chan==4) {
    autoNoteOn(note, vel);
    return;
  }
  if (chan>4) return;

  whistleNoteOn(chan, note);
}

void noteOff(uint8_t chan, uint8_t note) {
  if (chan==4) {
    autoNoteOff(note);
    return;
  }
  if (chan>4) return;

  whistleNoteOff(chan, note);
}

void pitchBend(uint8_t chan, uint8_t pb) {
  if (chan==4) {
    whistlePitchBend(0, pb);
    whistlePitchBend(1, pb);
    whistlePitchBend(2, pb);
    whistlePitchBend(3, pb);
    return;
  }
  if (chan>4) return;

  whistlePitchBend(chan, pb);
}


void processMIDI(uint8_t i) {
  static uint8_t status=0;
  static uint8_t bytenumber=0;
  static uint8_t bytetwo =0;

  if (i==0xFF) NVIC_SystemReset();
  if (i>=0xF8) return; //system real-time

  if (i & 0x80) {
    status = i;
    bytenumber = 1;
  } else {
    uint8_t chan = status&0x0F;
    if (bytenumber == 1) {
      // check two-byte messages...
      // switch (status&0xF0){}

      bytetwo=i;
      bytenumber=2;
    } else if (bytenumber ==2) {
      switch (status&0xF0) {

      case 0x90: //Note on
        if (i == 0) noteOff(chan, bytetwo);
        else noteOn(chan, bytetwo, i);
        break;

      case 0x80: //Note off
        noteOff(chan, bytetwo);
        break;

      case 0xE0: //Pitch bend
        // bend = pbSensitivity * (((i<<7) + bytetwo) - 0x2000);

        // throw away low byte
        pitchBend( chan, i );
        break;

      }
      bytenumber = 1; //running status
    }
  }
}

void processCalByte(uint8_t i) {
  static uint8_t buf[32];
  static uint8_t p=0, sum=0;

  buf[p++]=i;
  if (buf[0]!=0xFF) {
    p=0;
    sum=0;
    return;
  }

  sum += i;
  if (p>=23) {
    if (buf[1]==0xAA && sum==0xFF){

      set_valve_servo( 1, (buf[2]<<8)+buf[3]);
      set_valve_servo( 2, (buf[4]<<8)+buf[5]);
      set_valve_servo( 3, (buf[6]<<8)+buf[7]);
      set_valve_servo( 4, (buf[8]<<8)+buf[9]);

      AX12_SetSlidePos( 1, 0x230 + buf[10] );
      AX12_SetSlidePos( 3, 0x230 + buf[11] );
      AX12_SetSlidePos( 5, 0x230 + buf[12] );
      AX12_SetSlidePos( 7, 0x230 + buf[13] );

      set_fan_speed(1, (buf[14]<<8)+buf[15]);
      set_fan_speed(2, (buf[16]<<8)+buf[17]);
      set_fan_speed(3, (buf[18]<<8)+buf[19]);
      set_fan_speed(4, (buf[20]<<8)+buf[21]);

      poweroff_timeout=1000;
    }
    p=0;
    sum=0;
  }

}

void USART1_IRQHandler(void) {
  uint8_t i = LL_USART_ReceiveData8(USART1);

  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  //hang_error();


  if (calibrateMode) {
    processCalByte(i);
  } else {
    processMIDI(i);
  }

}

void EXTI9_5_IRQHandler(void)
{
  // MIDI panic / home servos

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  // Turn off all fans, close valves, set servos to up-down-up-down pattern
  // zero all note memory
  nsp[0]=0;
  nsp[1]=0;
  nsp[2]=0;
  nsp[3]=0;

  fan_target[0]=0;
  fan_target[1]=0;
  fan_target[2]=0;
  fan_target[3]=0;

  _set_fan_speed(1, 0);
  _set_fan_speed(2, 0);
  _set_fan_speed(3, 0);
  _set_fan_speed(4, 0);

  set_valve_servo(1, valve_closed[0]);
  set_valve_servo(2, valve_closed[1]);
  set_valve_servo(3, valve_closed[2]);
  set_valve_servo(4, valve_closed[3]);

  AX12_SetSlidePos( 1, 0x230 + 45 );
  AX12_SetSlidePos( 3, 0x230 + 145 );
  AX12_SetSlidePos( 5, 0x230 + 45 );
  AX12_SetSlidePos( 7, 0x230 + 145 );

  lastnote[0] = 72;
  lastnote[0] = 82;
  lastnote[0] = 72;
  lastnote[0] = 82;

  while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)); // wait for release

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  set_valve_servo(1, 0);
  set_valve_servo(2, 0);
  set_valve_servo(3, 0);
  set_valve_servo(4, 0);

  AX12_TorqueDisable();

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  LL_USART_EnableIT_RXNE(USART1);



  //uint8_t setID[] = { 0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, id, 0 };
  //uint8_t ledOn[] = { 0xFF, 0xFF, id, 0x04, 0x03, 0x19, 0x01, 0 };

  // TIM1 for Servo PWM
  // Prescaler 83 -> 1MHz clock
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  htim1.Instance->ARR = 10000;

  htim1.Instance->CCR1 = 0; // 1000 to 2000 ? For 1ms to 2ms
  htim1.Instance->CCR2 = 0;
  htim1.Instance->CCR3 = 0;
  htim1.Instance->CCR4 = 0;

  // TIM3 for Fan Motor PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  htim3.Instance->ARR = 5000;

  htim3.Instance->CCR1 = 5000; //PA6
  htim3.Instance->CCR2 = 5000; //PA7
  htim3.Instance->CCR3 = 5000; //PB0
  htim3.Instance->CCR4 = 5000; //PB1

  //AX12_TorqueEnable();

  disable_fan_motors();

  HAL_Delay(500); //Give debounce cap a chance to charge
  if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) { // pulled high when not pressed
    calibrateMode = 1;
  }
  uint8_t calFlash=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_Delay(10);


    if (calibrateMode) {
      if (++calFlash == 10) {
        calFlash=0;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }
    }

    for (uint8_t a=0; a<4; a++){
      if (fan_target[a]>0) {
        if (fan_warmup[a] < FAN_WARMUP_TIME) {
          fan_warmup[a]++;
        } else {
          set_fan_speed(a+1, fan_target[a]);
        }
      } else {
        if (fan_warmup[a]>0) fan_warmup[a]--;
      }
    }


    if (poweroff_timeout>0) {

      if (--poweroff_timeout==0) {

        disable_fan_motors();

        fan_target[0]=0;
        fan_target[1]=0;
        fan_target[2]=0;
        fan_target[3]=0;

        _set_fan_speed( 1, 0);
        _set_fan_speed( 2, 0);
        _set_fan_speed( 3, 0);
        _set_fan_speed( 4, 0);

        set_valve_servo( 1, 0);
        set_valve_servo( 2, 0);
        set_valve_servo( 3, 0);
        set_valve_servo( 4, 0);

        AX12_TorqueDisable();

      }
    }


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 31250;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
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

