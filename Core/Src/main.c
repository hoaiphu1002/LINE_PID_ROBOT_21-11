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
#include "hienthi.h"
#include "loadcell.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
LoadCell_HandleTypeDef loadcell;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int32_t encR = 0;
volatile int32_t encL = 0;
volatile uint16_t adc_dma_val[5];   // Mảng chứa giá trị ADC đọc được
volatile uint16_t cam0, cam1, cam2, cam3, cam4;   // Dễ theo dõi trên Watch
volatile int32_t last_encL = 0;
volatile float speedL = 0;
volatile int duty = 0;
volatile uint32_t last = 0;
volatile uint32_t encoder_cpl = 1320;       // 330 xung/vòng TRỤC RA (đã tính hộp số)
volatile float sample_time = 0.1;          // 100 ms
volatile uint16_t sensor_val[5];
volatile uint8_t line[5];   // 1 = trắng, 0 = đen
volatile uint16_t threshold = 2000; // tùy cảm biến, bạn test

uint16_t cam_raw[5];  // ADC đọc trực tiếp
uint16_t cam_min[5] = {229,227,233,246,294};
uint16_t cam_max[5] = {4095,4095,4095,4095,4095};
#define ENCODER_PPR   1320.0f   // Số xung/vòng TRỤC RA
#define WHEEL_DIAMETER 0.065f   // m (65 mm) - thay theo xe của bạn
#define WHEEL_CIRCUMFERENCE (3.14159f * WHEEL_DIAMETER)

volatile float speed_left_ms = 0.0f;
volatile float speed_right_ms = 0.0f;

static int32_t enc_left_prev = 0;
static int32_t enc_right_prev = 0;
static float rpm_left = 0.0f;
static float rpm_right = 0.0f;

volatile int32_t total_left_count = 0;
volatile int32_t total_right_count = 0;


uint16_t cam_norm[5]; // 0 → 1000
float Kp = 1.8;
float Ki = 0;
float Kd = 0.35;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PID_t;
PID_t pid_left  = { .Kp = 790.0f, .Ki = 7480.0f, .Kd = 0.0f, .integral = 0, .prev_error = 0 };
PID_t pid_right = { .Kp = 905.5f, .Ki = 8511.0f, .Kd = 0.0f, .integral = 0, .prev_error = 0 };


float integral = 0;
float last_error = 0;
volatile float left_speed = 0.0f;
volatile float right_speed = 0.0f;

float dt1=0.02f;
float dt2=1.0f;

typedef enum {
    KHONGCOHANG = 0,
    HANG_1KG,
    HANG_2KG
} Phanloaihang;

volatile Phanloaihang hang_hien_tai = KHONGCOHANG;
volatile uint8_t dang_o_diem_nhan_hang = 0;
volatile uint8_t dang_o_diem_re = 0;
volatile uint8_t dang_re = 0;
volatile uint32_t thoi_gian_bat_dau_re = 0;



void chieu_thuan_dongco_trai(){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
}
void chieu_thuan_dongco_phai(){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
}

void chay_thang()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
}

void Motor_ReadSpeed(void)
{
    int32_t enc_left_now  = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t enc_right_now = __HAL_TIM_GET_COUNTER(&htim4);

    int32_t delta_left  = enc_left_now - enc_left_prev;
    int32_t delta_right = enc_right_now - enc_right_prev;

    // Xử lý tràn (overflow)
    if (delta_left >  32767) delta_left -= 65536;
    if (delta_left < -32768) delta_left += 65536;
    if (delta_right > 32767) delta_right -= 65536;
    if (delta_right < -32768) delta_right += 65536;


    total_left_count  += delta_left;
    total_right_count += delta_right;

    enc_left_prev  = enc_left_now;
    enc_right_prev = enc_right_now;

    // vòng / giây
    float rps_left  = (-delta_left  / ENCODER_PPR) / sample_time;
    float rps_right = (delta_right / ENCODER_PPR) / sample_time;

    rpm_left  = rps_left  * 60.0f;
    rpm_right = rps_right * 60.0f;

    // đổi sang m/s
    speed_left_ms  = rps_left  * WHEEL_CIRCUMFERENCE;
    speed_right_ms = rps_right * WHEEL_CIRCUMFERENCE;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        cam0 = adc_dma_val[0];   // PA0 (ADC1_IN1)
        cam1 = adc_dma_val[1];   // PA1 (ADC1_IN2)
        cam2 = adc_dma_val[2];   // PA2 (ADC1_IN3)
        cam3 = adc_dma_val[3];   // PA3 (ADC1_IN4)
        cam4 = adc_dma_val[4];   // PA4 (ADC1_IN5)

    }
}

void read_line_sensor() {
    sensor_val[0] = cam0;
    sensor_val[1] = cam1;
    sensor_val[2] = cam2;
    sensor_val[3] = cam3;
    sensor_val[4] = cam4;

}
void normalize_sensor()
{
    // LẤY GIÁ TRỊ ADC THỰC VÀO cam_raw
    cam_raw[0] = cam0;
    cam_raw[1] = cam1;
    cam_raw[2] = cam2;
    cam_raw[3] = cam3;
    cam_raw[4] = cam4;


    for(int i = 0; i < 5; i++)
    {
        int32_t val = cam_raw[i];

        if(val < cam_min[i]) val = cam_min[i];
        if(val > cam_max[i]) val = cam_max[i];

        cam_norm[i] = (val - cam_min[i]) * 1000 / (cam_max[i] - cam_min[i]);
    }
}
int16_t get_line_position()
{
    int32_t sum_val = 0;
    int32_t sum_weight = 0;

    int16_t weight[5] = {-260,-130,0,130,260};

    for(int i=0;i<5;i++)
    {
        sum_val += cam_norm[i] * weight[i];
        sum_weight += cam_norm[i];
    }

    if(sum_weight == 0)
        return 0; // không thấy line → giữ hướng cũ

    return sum_val / sum_weight;
}

void print_line_sensor_data()
{
    char buffer[300];

    // In giá trị raw
    print_uart("Raw: ");
    for (int i = 0; i < 5; i++)
    {
        sprintf(buffer, "%4d ", cam_raw[i]);
        print_uart(buffer);
    }
    print_uart("\r\n");

    // In giá trị normalized
    print_uart("Norm: ");
    for (int i = 0; i < 5; i++)
    {
        sprintf(buffer, "%4d ", cam_norm[i]);
        print_uart(buffer);
    }
    print_uart("\r\n");

    // In độ lệch line
    int16_t pos = get_line_position();
    sprintf(buffer, "Position: %d\r\n", pos);
    print_uart(buffer);

    sprintf(buffer, "EncL: %ld  EncR: %ld\r\n",
            (long)total_left_count,
            (long)total_right_count);
    print_uart(buffer);



}

//ĐỌC BLUETOOTH

void line_pid_control(float dt2)
{
    normalize_sensor();

    float error = get_line_position();  // -300 → +300

    integral += error * dt2;   // <--- ĐÚNG
    float derivative = (error - last_error) / dt2;  // <--- ĐÚNG

    float correction = Kp*error + Ki*integral + Kd*derivative;

    last_error = error;

    // scale correction ra m/s
    float corr_ms = correction * 0.00032f;

    float base_ms = 0.2f;

    left_speed  = base_ms - corr_ms;
    right_speed = base_ms + corr_ms;

    if(left_speed  < 0) left_speed  = 0;
    if(right_speed < 0) right_speed = 0;

    if(left_speed > 0.35f)  left_speed = 0.35f;
    if(right_speed > 0.35f) right_speed = 0.35f;
}

float PID_Speed(PID_t *pid, float set, float measure, float dt1)
{
    float error = set - measure;

    pid->integral += error * dt1;
    float derivative = (error - pid->prev_error) / dt1;

    float output = pid->Kp*error + pid->Ki*pid->integral + pid->Kd*derivative;

    pid->prev_error = error;

    // Giới hạn PWM
    if(output < 0) output = 0;
    if(output > 999) output = 999;

    return output;
}


Phanloaihang kiem_tra_loai_hang(void)
{
    float khoi_luong = LoadCell_ReadGram(&loadcell, 5);

    if (khoi_luong >= 800.0f && khoi_luong <= 1200.0f) {
        return HANG_1KG;
    } else if (khoi_luong >= 1800.0f && khoi_luong <= 2200.0f) {
        return HANG_2KG;
    }

    return KHONGCOHANG;
}

void xu_ly_diem_nhan_hang(void)
{
    // Dừng động cơ
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

    // Chờ cho đến khi có hàng 1kg hoặc 2kg
    while (1) {
        hang_hien_tai = kiem_tra_loai_hang();
        if (hang_hien_tai == HANG_1KG || hang_hien_tai == HANG_2KG) {
            break;
        }
        HAL_Delay(100);
    }

    dang_o_diem_nhan_hang = 0;

    // Reset PID để bắt đầu bám line lại
    integral = 0;
    last_error = 0;

    left_speed=0.15;
    right_speed=0.15;
    float pwm_trai = PID_Speed(&pid_left, left_speed, speed_left_ms, dt1);
    float pwm_phai = PID_Speed(&pid_right, right_speed, speed_right_ms, dt1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_trai);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_phai);

    // Xe sẽ tự động chạy tiếp nhờ PID trong vòng lặp chính
}

void re_trai(void)
{
    // Rẽ trái: bánh phải chạy nhanh hơn
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 300);
    left_speed = 0.1;
    right_speed=0.15;
    // Cập nhật PWM ngay lập tức
    float pwm_trai = PID_Speed(&pid_left, left_speed, speed_left_ms, dt1);
    float pwm_phai = PID_Speed(&pid_right, right_speed, speed_right_ms, dt1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_trai);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_phai);

}

void re_phai(void)
{
    // Rẽ phải: bánh trái chạy nhanh hơn
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 50);
    left_speed = 0.25;
    right_speed=0.1;
    // Cập nhật PWM ngay lập tức
    float pwm_trai = PID_Speed(&pid_left, left_speed, speed_left_ms, dt1);
    float pwm_phai = PID_Speed(&pid_right, right_speed, speed_right_ms, dt1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_trai);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_phai);
}
void xu_ly_diem_re(void)
{
    char buffer[100];

    if (hang_hien_tai == HANG_1KG) {
        sprintf(buffer, "RE TRAI - Hang 1kg\r\n");
        print_uart(buffer);
        re_trai();
    } else if (hang_hien_tai == HANG_2KG) {
        sprintf(buffer, "RE PHAI - Hang 2kg\r\n");
        print_uart(buffer);
        re_phai();
    } else {
        sprintf(buffer, "KHONG CO HANG - Khong re\r\n");
        print_uart(buffer);
        return;  // Không làm gì nếu không có hàng
    }

    dang_re = 1;
    thoi_gian_bat_dau_re = HAL_GetTick();
    dang_o_diem_re = 0;
}

void ket_thuc_re(void)
{
    dang_re = 0;
    // Reset PID để bám line lại từ đầu
    integral = 0;
    last_error = 0;
}

uint8_t check_stop_simple(void)
{
    int16_t vi_tri_line = get_line_position();

    // Điểm nhận hàng
    if ((total_right_count > 12450 && total_right_count < 13000) &&
        (vi_tri_line < 65 && vi_tri_line > -65)&&(hang_hien_tai == KHONGCOHANG)) {
        dang_o_diem_nhan_hang = 1;
        return 1;
    }

    // Điểm rẽ
    if ((total_right_count > 25000 && total_right_count < 26000)&&(hang_hien_tai != KHONGCOHANG)) {
        dang_o_diem_re = 1;
        return 1;
    }

    // Dừng khẩn cấp khi mất line
    if (cam_norm[0] < 50 && cam_norm[1] < 50 && cam_norm[2] < 50 &&
        cam_norm[3] < 50 && cam_norm[4] < 50) {
        return 1;
    }

    return 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
//  HAL_Delay(5000);  // Cho thời gian mở Serial Monitor
  // BẮT ĐẦU ĐỌC ADC BẰNG DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_val, 5);
  // Khởi tạo cân
  LoadCell_Init(&loadcell,
                GPIOB, GPIO_PIN_8,   // DOUT
                GPIOB, GPIO_PIN_9,   // SCK
                415.713012f, -321367);

  	chieu_thuan_dongco_trai();
  	chieu_thuan_dongco_phai();
  	uint32_t last_pid = 0;
  	uint32_t last_line = 0;
  	uint32_t last_speed = 0;
  	uint32_t last_cell = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t thoi_gian_hien_tai = HAL_GetTick();

	     // Cập nhật encoder và tốc độ
	     if(thoi_gian_hien_tai - last_speed >= 100) {
	         last_speed = thoi_gian_hien_tai;
	         Motor_ReadSpeed();
	     }

	     // Đọc cảm biến line
	     if(thoi_gian_hien_tai - last_line >= 100) {
	         last_line = thoi_gian_hien_tai;
	         normalize_sensor();
	          read_line_sensor();
	          print_line_sensor_data();
	     }

	     // Kiểm tra điểm dừng
	     if(check_stop_simple()) {
	         if (dang_o_diem_nhan_hang) {
	             xu_ly_diem_nhan_hang();
	             continue;
	         }
	         else if (dang_o_diem_re) {
	             xu_ly_diem_re();
	             continue;
	         }
	         else {
                 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
                 continue;
	         }
	     }

	     // Kết thúc lượt rẽ sau 0.5 giây
	     if (dang_re && (thoi_gian_hien_tai - thoi_gian_bat_dau_re >= 400)) {
	         ket_thuc_re();
	     }

	     // Điều khiển PID bám line (chỉ khi không đang rẽ)
	     if(!dang_o_diem_nhan_hang && !dang_re && thoi_gian_hien_tai - last_pid >= 100) {
	         last_pid = thoi_gian_hien_tai;

	         line_pid_control(dt2);

	         float pwm_trai = PID_Speed(&pid_left, left_speed, speed_left_ms, dt1);
	         float pwm_phai = PID_Speed(&pid_right, right_speed, speed_right_ms, dt1);

	         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_trai);
	         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_phai);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
