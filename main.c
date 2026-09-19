/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @project AAC_final  (NUCLEO-F446RE)
  * @brief   sEMG gesture -> Bangla phrase AAC device
  *
  *  Chain:  EMG (PA0, ADC1_IN0, TIM2-triggered @100Hz)
  *            -> MAV over 3.3 s capture window
  *            -> nearest-centroid classifier (3 gestures)
  *            -> USART3 "P,NN" to ESP32   : ILI9341 Bangla phrase + Blynk cloud
  *            -> USART1 DFPlayer folder 1 : Bangla audio track NN
  *            -> USART2 PuTTY             : live trace + manual fallback
  *
  *  PuTTY commands (type + Enter):
  *      1 | 2 | 3   speak phrase manually (same path as a gesture)
  *      c           clear display
  *      s           stop DFPlayer playback
  *      + | -       volume up / down (step 2, range 0-30)
  *      ?           print help / current state
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define GESTURE_COUNT     3U
#define PHRASE_COUNT      3U          /* must match PHRASE_COUNT in phrases.h */

#define CAPTURE_SAMPLES   330U        /* 330 @ 100 Hz = 3.3 s hold          */
#define MAV_MAX_DIST      150U        /* reject if |MAV-centroid| > this.
                                         set to 4095 to disable rejection    */
#define BTN_LOCKOUT_MS    500U        /* button debounce / re-trigger guard  */

#define LINE_MAX          32U
#define ACK_TIMEOUT_MS    2000U       /* ESP32 ACKs only after the bitmap is
                                         drawn; 1 s was marginal for 300x48  */
/* USER CODE END PD */

/* USER CODE BEGIN PV */
/* --- classifier model (raw 12-bit ADC units, measured on real captures) --- */
static const uint16_t centroid[GESTURE_COUNT]          = { 509u, 680u, 924u };
static const char *const gesture_name[GESTURE_COUNT]   = { "THUMBS_UP", "MAKE_FIST", "WRIST_IN_FIST" };

/* romanised phrase labels for the terminal (PuTTY will not render Bangla) */
static const char *const phrase_label[PHRASE_COUNT + 1] = {
  "-",
  "AMI THIK ACHI",
  "HYAN",
  "AMAKE SAHAJJO KORUN"   /* = HELP_ID on the ESP32 -> critical Blynk event */
};

/* --- EMG capture state (written from ADC / EXTI ISRs) --- */
static volatile uint8_t  capturing     = 0;
static volatile uint8_t  capture_done  = 0;
static volatile uint8_t  capture_start = 0;   /* ISR -> main: announce capture */
static volatile uint32_t sample_sum    = 0;
static volatile uint32_t sample_count  = 0;
static uint32_t last_btn_ms = 0;

/* --- USART2 (PuTTY) line assembly --- */
static uint8_t  pc_rx_byte;
static char     pc_acc[LINE_MAX];
static uint8_t  pc_acc_len = 0;
static char     pc_line[LINE_MAX];
static volatile uint8_t pc_line_ready = 0;

/* --- USART3 (ESP32) line assembly --- */
static uint8_t  esp_rx_byte;
static char     esp_acc[LINE_MAX];
static uint8_t  esp_acc_len = 0;
static char     esp_line[LINE_MAX];
static volatile uint8_t esp_line_ready = 0;

/* --- ESP32 ACK tracking --- */
static uint8_t  ack_pending = 0;
static char     ack_expect[LINE_MAX];
static uint32_t ack_t0 = 0;
static uint8_t  esp_seen = 0;          /* set once the ESP32 has said anything */

/* --- DFPlayer --- */
static uint8_t current_volume = 15;    /* 0..30 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void pc_print(const char *s);
static void DFPlayer_SendCommand(uint8_t cmd, uint8_t p1, uint8_t p2);
static void DFPlayer_PlayFolderTrack(uint8_t folder, uint8_t track);
static void DFPlayer_SetVolume(uint8_t volume);
static void DFPlayer_Stop(void);
static void send_to_esp(const char *frame);
static void speak_phrase(uint8_t id);
static void handle_pc_cmd(const char *cmd);
static void handle_esp_line(const char *line);
static void print_help(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ============================ terminal ============================ */
static void pc_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void print_help(void)
{
  char msg[200];
  snprintf(msg, sizeof(msg),
           "[SYS] USER button = capture gesture (%lu ms hold)\r\n"
           "[SYS] keys: 1..%u speak | c clear | s stop | +/- volume | ? help\r\n"
           "[SYS] volume=%u  esp=%s\r\n",
           (unsigned long)(CAPTURE_SAMPLES * 10U), (unsigned)PHRASE_COUNT,
           current_volume, esp_seen ? "up" : "no reply yet");
  pc_print(msg);
}

/* ============================ DFPlayer (USART1) ============================ */
static void DFPlayer_SendCommand(uint8_t cmd, uint8_t p1, uint8_t p2)
{
  uint8_t frame[10];

  frame[0] = 0x7E;
  frame[1] = 0xFF;
  frame[2] = 0x06;
  frame[3] = cmd;
  frame[4] = 0x00;
  frame[5] = p1;
  frame[6] = p2;

  uint16_t sum      = frame[1] + frame[2] + frame[3] + frame[4] + frame[5] + frame[6];
  uint16_t checksum = (uint16_t)(0 - sum);

  frame[7] = (uint8_t)(checksum >> 8);
  frame[8] = (uint8_t)(checksum & 0xFF);
  frame[9] = 0xEF;

  HAL_UART_Transmit(&huart1, frame, sizeof(frame), 100);
}

static void DFPlayer_PlayFolderTrack(uint8_t folder, uint8_t track)
{
  DFPlayer_SendCommand(0x0F, folder, track);
}

static void DFPlayer_SetVolume(uint8_t volume)
{
  if (volume > 30) volume = 30;
  DFPlayer_SendCommand(0x06, 0x00, volume);
}

static void DFPlayer_Stop(void)
{
  DFPlayer_SendCommand(0x16, 0x00, 0x00);
}

/* ============================ ESP32 link (USART3) ============================ */
static void send_to_esp(const char *frame)
{
  char buf[LINE_MAX + 4];
  int  n = snprintf(buf, sizeof(buf), "%s\n", frame);
  HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 100);

  char msg[64];
  snprintf(msg, sizeof(msg), "[STM] -> ESP: %s\r\n", frame);
  pc_print(msg);

  snprintf(ack_expect, sizeof(ack_expect), "A,%s", frame);
  ack_t0      = HAL_GetTick();
  ack_pending = 1;
}

/* one place where a phrase is emitted: display+cloud first, then audio.
   the ESP32 frame is 5 bytes (~0.4 ms); the DFPlayer frame is 10 bytes at
   9600 baud (~10 ms), so display and speech start effectively together. */
static void speak_phrase(uint8_t id)
{
  char frame[16];

  if (id < 1 || id > PHRASE_COUNT) return;

  snprintf(frame, sizeof(frame), "P,%02u", (unsigned)id);
  send_to_esp(frame);

  DFPlayer_PlayFolderTrack(1, id);

  char msg[64];
  snprintf(msg, sizeof(msg), "[DFP] play 01/%03u  \"%s\"\r\n", (unsigned)id, phrase_label[id]);
  pc_print(msg);
}

static void handle_esp_line(const char *line)
{
  char msg[80];

  esp_seen = 1;

  if (strcmp(line, "R") == 0)
  {
    pc_print("[ESP] R  (ESP32 booted, display ready)\r\n");
    return;
  }

  if (ack_pending && strcmp(line, ack_expect) == 0)
  {
    snprintf(msg, sizeof(msg), "[ESP] %s  ack in %lu ms\r\n",
             line, (unsigned long)(HAL_GetTick() - ack_t0));
    pc_print(msg);
    ack_pending = 0;
    return;
  }

  snprintf(msg, sizeof(msg), "[ESP] %s\r\n", line);
  pc_print(msg);
}

/* ============================ PuTTY commands ============================ */
static void handle_pc_cmd(const char *cmd)
{
  char msg[72];

  if (cmd[1] == '\0')
  {
    switch (cmd[0])
    {
      case 'c': case 'C':
        send_to_esp("C");
        return;

      case 's': case 'S':
        DFPlayer_Stop();
        pc_print("[DFP] stop\r\n");
        return;

      case '+':
        if (current_volume <= 28) current_volume += 2;
        DFPlayer_SetVolume(current_volume);
        snprintf(msg, sizeof(msg), "[DFP] volume=%u\r\n", current_volume);
        pc_print(msg);
        return;

      case '-':
        if (current_volume >= 2) current_volume -= 2;
        DFPlayer_SetVolume(current_volume);
        snprintf(msg, sizeof(msg), "[DFP] volume=%u\r\n", current_volume);
        pc_print(msg);
        return;

      case '?':
        print_help();
        return;

      default:
        break;
    }
  }

  char *end;
  long id = strtol(cmd, &end, 10);
  if (end != cmd && *end == '\0' && id >= 1 && id <= PHRASE_COUNT)
  {
    snprintf(msg, sizeof(msg), "[PC ] manual phrase %ld\r\n", id);
    pc_print(msg);
    speak_phrase((uint8_t)id);
    return;
  }

  snprintf(msg, sizeof(msg), "[PC ] invalid: '%s'  (? for help)\r\n", cmd);
  pc_print(msg);
}

/* ============================ UART plumbing ============================ */
/* byte-wise line assembly; terminators \r or \n; empty lines ignored */
static void line_accumulate(uint8_t b, char *acc, uint8_t *len,
                            char *out, volatile uint8_t *ready)
{
  if (b == '\r' || b == '\n')
  {
    if (*len > 0)
    {
      if (!*ready)                    /* drop if main loop hasn't consumed the last line */
      {
        memcpy(out, acc, *len);
        out[*len] = '\0';
        *ready = 1;
      }
      *len = 0;
    }
  }
  else if (*len < LINE_MAX - 1)
  {
    acc[(*len)++] = (char)b;
  }
  else
  {
    *len = 0;                         /* overlong line: discard */
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    line_accumulate(pc_rx_byte, pc_acc, &pc_acc_len, pc_line, &pc_line_ready);
    HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);
  }
  else if (huart->Instance == USART3)
  {
    line_accumulate(esp_rx_byte, esp_acc, &esp_acc_len, esp_line, &esp_line_ready);
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
  }
}

/* overrun/framing/noise aborts IT reception in HAL: restart it */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    pc_acc_len = 0;
    HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);
  }
  else if (huart->Instance == USART3)
  {
    esp_acc_len = 0;
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
  }
}

/* ============================ EMG acquisition ============================ */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) return;

  uint16_t adc_val = HAL_ADC_GetValue(&hadc1);

  if (capturing)
  {
    sample_sum += adc_val;
    sample_count++;
    if (sample_count >= CAPTURE_SAMPLES)
    {
      capturing    = 0;
      capture_done = 1;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_13) return;          /* B1 user button, PC13 */

  uint32_t now = HAL_GetTick();
  if ((now - last_btn_ms) < BTN_LOCKOUT_MS) return;
  last_btn_ms = now;

  if (capturing || capture_done) return;        /* busy */

  sample_sum    = 0;
  sample_count  = 0;
  capture_start = 1;
  capturing     = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  /* arm both receivers before anything blocking, so the ESP32 boot "R"
     that arrives during the DFPlayer mount delay is not lost */
  HAL_UART_Receive_IT(&huart2, &pc_rx_byte, 1);
  HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);

  pc_print("\r\n=== AAC_final : sEMG -> Bangla phrase ===\r\n");
  pc_print("[SYS] booting, waiting for DFPlayer SD mount (2 s)\r\n");

  HAL_Delay(2000);                       /* DFPlayer SD-card mount time */
  DFPlayer_SetVolume(current_volume);

  HAL_TIM_Base_Start(&htim2);            /* 100 Hz sample clock, free running */
  HAL_ADC_Start_IT(&hadc1);              /* ADC idles until a capture starts   */

  print_help();
  pc_print("[SYS] READY\r\n");
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- PuTTY command ---- */
    if (pc_line_ready)
    {
      char cmd[LINE_MAX];
      strcpy(cmd, pc_line);
      pc_line_ready = 0;
      handle_pc_cmd(cmd);
    }

    /* ---- ESP32 reply ---- */
    if (esp_line_ready)
    {
      char line[LINE_MAX];
      strcpy(line, esp_line);
      esp_line_ready = 0;
      handle_esp_line(line);
    }

    /* ---- ESP32 ACK watchdog ---- */
    if (ack_pending && (HAL_GetTick() - ack_t0) > ACK_TIMEOUT_MS)
    {
      ack_pending = 0;
      pc_print("[STM] no ACK from ESP32 (timeout) - display may be stale\r\n");
    }

    /* ---- capture started (announced here, never from the ISR) ---- */
    if (capture_start)
    {
      capture_start = 0;
      BSP_LED_On(LED2);
      char msg[64];
      snprintf(msg, sizeof(msg), "[EMG] capturing %lu ms - hold the gesture...\r\n",
               (unsigned long)(CAPTURE_SAMPLES * 10U));
      pc_print(msg);
    }

    /* ---- capture finished: classify and act ---- */
    if (capture_done)
    {
      capture_done = 0;
      BSP_LED_Off(LED2);

      uint32_t mav = sample_sum / sample_count;

      uint8_t  best = 0;
      uint32_t best_dist = (mav > centroid[0]) ? (mav - centroid[0]) : (centroid[0] - mav);
      for (uint8_t i = 1; i < GESTURE_COUNT; i++)
      {
        uint32_t dist = (mav > centroid[i]) ? (mav - centroid[i]) : (centroid[i] - mav);
        if (dist < best_dist) { best_dist = dist; best = i; }
      }

      char msg[96];
      if (best_dist > MAV_MAX_DIST)
      {
        snprintf(msg, sizeof(msg),
                 "[EMG] MAV:%lu  REJECTED (dist %lu > %u) - retry\r\n",
                 (unsigned long)mav, (unsigned long)best_dist, (unsigned)MAV_MAX_DIST);
        pc_print(msg);
      }
      else
      {
        snprintf(msg, sizeof(msg),
                 "[EMG] MAV:%lu  dist:%lu  -> %s  (phrase %u)\r\n",
                 (unsigned long)mav, (unsigned long)best_dist,
                 gesture_name[best], (unsigned)(best + 1));
        pc_print(msg);
        speak_phrase((uint8_t)(best + 1));
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16;
  RCC_OscInitStruct.PLL.PLLN            = 336;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ            = 2;
  RCC_OscInitStruct.PLL.PLLR            = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
