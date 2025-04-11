#include "stm32f4xx.h"

// PID Control Variables
float Kp = 190.0f;  // Proportional gain
float Ki = 0.0f;    // Integral gain
float Kd = 210.0f;  // Derivative gain

int baseSpeed = 400;  // Base motor speed (0-255)
int maxSpeed = 1000;  // Maximum motor speed (0-255)

float lastError = 0.0f;  // Previous error for derivative calculation
float integral = 0.0f;   // Accumulated error for integral calculation
uint32_t lastTime = 0;   // For calculating dt in integral term

// Line position variables
int position = 0;      // Current position of the line
int setPoint = 0;      // Desired position (center)
int lastPosition = 0;  // Last known position before line was lost

// Line loss tracking variables
static uint32_t lineLostTime = 0;  // Time when line was lost
static int lineFoundFlag = 1;      // Flag to track if line was previously found
static int lastDirection = 0;      // Last direction of travel (-1: left, 0: straight, 1: right)
static int searchPhase = 0;        // Search pattern phase

// Function prototypes
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_PWM_Init(void);
uint32_t my_micros(void);
void delayMs(uint32_t ms);
void motorControl(int leftSpeed, int rightSpeed);
void handleLineLoss(uint32_t currentTime);

int main(void) {
  // Initialize system
  SystemClock_Config();
  GPIO_Init();
  TIM_PWM_Init();

  // Standby pin high (enable motor driver)
  GPIOB->BSRR = GPIO_BSRR_BS6;  // Set PB6 (STBY) high

  // Delay for stability at startup
  delayMs(1000);

  // Serial.begin(9600);

  while (1) {
    // Read sensor values (active LOW for black line, so invert)
    // int s1 = !(GPIOA->IDR & GPIO_IDR_ID0);  // PA0 - Left-most sensor
    // int s2 = !(GPIOA->IDR & GPIO_IDR_ID2);  // PA2
    // int s3 = !(GPIOA->IDR & GPIO_IDR_ID4);  // PA4
    // int s4 = !(GPIOA->IDR & GPIO_IDR_ID6);  // PA6 - Right-most sensor

    // For white line
    uint32_t sensorPort = GPIOA->IDR;
    int s1 = (sensorPort & GPIO_IDR_ID0) != 0;
    int s2 = (sensorPort & GPIO_IDR_ID2) != 0;
    int s3 = (sensorPort & GPIO_IDR_ID4) != 0;
    int s4 = (sensorPort & GPIO_IDR_ID6) != 0;

    // Serial.println(s1);
    // Serial.println(s2);
    // Serial.println(s3);
    // Serial.println(s4);
    // Serial.println("__________");

    // Calculate position of the line
    int activeSensors = s1 + s2 + s3 + s4;
    uint32_t currentTime = my_micros();

    if (activeSensors == 0) {
      // Line is lost
      if (lineFoundFlag) {
        // If line was previously found, record the loss time
        lineLostTime = currentTime;
        lineFoundFlag = 0;
      }

      // Handle line loss behavior
      handleLineLoss(currentTime);

    } else {
      // Line is found
      lineFoundFlag = 1;
      searchPhase = 0;  // Reset search phase

      // Calculate weighted average of sensor positions
      position = (-3 * s1 + -1 * s2 + 1 * s3 + 3 * s4) / (float)(s1 + s2 + s3 + s4);

      // Store last position for use during line loss
      lastPosition = position;

      // Determine direction for line loss recovery
      if (position < -1) lastDirection = -1;     // Line is to the left
      else if (position > 1) lastDirection = 1;  // Line is to the right
      else lastDirection = 0;                    // Line is centered

      // Calculate error
      int error = position - setPoint;

      // Calculate time delta
      float dt = (currentTime - lastTime) / 1000000.0f;  // Convert to seconds
      lastTime = currentTime;

      // Calculate integral (with anti-windup)
      integral += error * dt;
      if (integral > 50.0f) integral = 50.0f;
      if (integral < -50.0f) integral = -50.0f;

      // Calculate derivative
      float derivative = 0.0f;
      if (dt > 0) {
        derivative = (error - lastError) / dt;
      }
      lastError = error;

      // PID calculation
      float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

      // Calculate motor speeds
      int rightSpeed = baseSpeed - (int)pidOutput;
      int leftSpeed = baseSpeed + (int)pidOutput;

      // Constrain speeds to prevent overflow
      if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
      if (rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;
      if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
      if (leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;

      // Set motor speeds and direction
      motorControl(leftSpeed, rightSpeed);
    }

    // Small delay for stability
    delayMs(10);
  }
}

// Line loss handling function
void handleLineLoss(uint32_t currentTime) {
  // Calculate how long the line has been lost (in microseconds)
  uint32_t lostDuration = currentTime - lineLostTime;

  // Convert to milliseconds for easier comparison
  uint32_t lostMillis = lostDuration / 1000;

  // Different strategies based on how long the line has been lost
  if (lostMillis < 500) {
    // Phase 1 (0-500ms): Continue in the last known direction
    // This helps with small gaps in the line or momentary sensor misreadings
    if (lastDirection < 0) {
      // Line was to the left, turn left
      motorControl(baseSpeed - 200, baseSpeed + 200);
    } else if (lastDirection > 0) {
      // Line was to the right, turn right
      motorControl(baseSpeed + 200, baseSpeed - 200);
    } else {
      // Line was centered, go straight
      motorControl(baseSpeed, baseSpeed);
    }
  } else if (lostMillis < 2000) {
    // Phase 2 (500-2000ms): Gradual spiral search
    // Gradually widen the search pattern by increasing the turn radius
    int turnIntensity = 200 + (lostMillis - 500) / 5;  // Increases over time

    if (lastDirection <= 0) {
      // Search to the left (counterclockwise)
      motorControl(baseSpeed - turnIntensity, baseSpeed + turnIntensity);
    } else {
      // Search to the right (clockwise)
      motorControl(baseSpeed + turnIntensity, baseSpeed - turnIntensity);
    }
  } else {
    // Phase 3 (after 2000ms): Alternate zigzag search pattern
    // The robot will zigzag in wider and wider arcs to find the line

    // Update search phase every 500ms
    searchPhase = (lostMillis - 2000) / 500;

    // Determine search direction based on search phase
    int searchDirection = searchPhase % 2 ? 1 : -1;

    // Increase turn intensity based on phase number
    int turnIntensity = 250 + (searchPhase * 50);
    if (turnIntensity > 400) turnIntensity = 400;  // Cap at maximum

    // Zigzag search pattern
    if (searchDirection < 0) {
      // Turn left
      motorControl(baseSpeed / 2, baseSpeed + turnIntensity);
    } else {
      // Turn right
      motorControl(baseSpeed + turnIntensity, baseSpeed / 2);
    }
  }
}

void SystemClock_Config(void) {
  // Enable HSI
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;

  // Configure PLL (assuming 16MHz HSI)
  // PLL_M = 16, PLL_N = 192, PLL_P = 2 -> 96MHz system clock
  RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLN_Pos) | (8 << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLSRC_HSI;

  // Enable PLL
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  // Configure flash latency for 96MHz
  FLASH->ACR = FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

  // Configure AHB, APB1, APB2 dividers
  RCC->CFGR = RCC_CFGR_HPRE_DIV1 |   // AHB prescaler = 1
              RCC_CFGR_PPRE1_DIV2 |  // APB1 prescaler = 2
              RCC_CFGR_PPRE2_DIV1;   // APB2 prescaler = 1

  // Select PLL as system clock
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;

  // Update SystemCoreClock variable (96MHz)
  SystemCoreClockUpdate();

  // Configure SysTick for 1ms interrupts
  SysTick_Config(SystemCoreClock / 1000);
}

void GPIO_Init(void) {
  // Enable GPIOA and GPIOB clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

  // Configure sensor pins (PA0, PA2, PA4, PA6) as inputs
  // Reset MODER bits (00: Input mode)
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER2 | GPIO_MODER_MODER4 | GPIO_MODER_MODER6);

  // Enable pull-up resistors for sensor pins
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR6);
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR6_0;

  // Configure motor driver control pins
  // STBY = PB6, AIN1 = PB4, AIN2 = PB5, BIN1 = PB7, BIN2 = PB8
  // Set as outputs (01: General purpose output mode)
  GPIOB->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8);
  GPIOB->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0;

  // Configure PWMA (PB3) and PWMB (PB10) as alternate function for PWM
  // Set as alternate function (10: Alternate function mode)
  GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER10);
  GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER10_1;

  // Configure alternate function for TIM2_CH2 (PB3) and TIM2_CH3 (PB10)
  GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL3;  // Clear AF for PB3
  GPIOB->AFR[0] |= (1 << (3 * 4));    // Set AF1 for PB3 (TIM2)

  GPIOB->AFR[1] &= ~GPIO_AFRH_AFRH2;  // Clear AF for PB10
  GPIOB->AFR[1] |= (1 << (2 * 4));    // Set AF1 for PB10 (TIM2)
}

void TIM_PWM_Init(void) {
  // Enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Configure TIM2
  // Set prescaler, period for suitable PWM frequency
  // With 96MHz system clock, prescaler=96, period=999 gives 1kHz PWM
  TIM2->PSC = 96 - 1;  // Prescaler = 96
  TIM2->ARR = 999;     // Auto-reload = 999 (0-999 = 1000 steps)

  // Configure channel 2 (PB3) for PWM mode 1
  TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
  TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // PWM mode 1
  TIM2->CCMR1 |= TIM_CCMR1_OC2PE;                      // Enable preload

  // Configure channel 3 (PB10) for PWM mode 1
  TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
  TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;  // PWM mode 1
  TIM2->CCMR2 |= TIM_CCMR2_OC3PE;                      // Enable preload

  // Enable outputs
  TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E;

  // Enable auto-reload preload
  TIM2->CR1 |= TIM_CR1_ARPE;

  // Enable counter
  TIM2->CR1 |= TIM_CR1_CEN;
}

// Time keeping variables
volatile uint32_t tickWW = 0;

// SysTick handler - increments the millisecond counter
//void SysTick_Handler(void) {
//  tickWW++;
//}

// Microsecond timer function
uint32_t my_micros(void) {
  // Return milliseconds * 1000 + fraction of millisecond
  uint32_t ticks = SysTick->VAL;
  uint32_t pend = (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) ? 1 : 0;
  uint32_t ms = tickWW;

  // If PENDSTSET and the tick counter reloaded from zero, add 1ms
  if (pend && ticks > (SysTick->LOAD / 2)) {
    ms++;
  }

  // Convert reload value to microseconds
  uint32_t reload = SysTick->LOAD;

  // Calculate microseconds from tick value and milliseconds
  return (ms * 1000) + ((reload - ticks) * 1000) / reload;
}

// Millisecond delay function
void delayMs(uint32_t ms) {
  uint32_t start = uwTick;
  while ((uwTick - start) < ms)
    ;
}

// Motor control function
void motorControl(int leftSpeed, int rightSpeed) {
  // Right Motor (Motor A)
  if (rightSpeed >= 0) {
    // Forward
    GPIOB->BSRR = GPIO_BSRR_BS4;  // AIN1 = High
    GPIOB->BSRR = GPIO_BSRR_BR5;  // AIN2 = Low
    TIM2->CCR2 = rightSpeed;      // PWM duty cycle
  } else {
    // Reverse
    GPIOB->BSRR = GPIO_BSRR_BR4;  // AIN1 = Low
    GPIOB->BSRR = GPIO_BSRR_BS5;  // AIN2 = High
    TIM2->CCR2 = -rightSpeed;     // PWM duty cycle (positive value)
  }

  // Left Motor (Motor B)
  if (leftSpeed >= 0) {
    // Forward
    GPIOB->BSRR = GPIO_BSRR_BS7;  // BIN1 = High
    GPIOB->BSRR = GPIO_BSRR_BR8;  // BIN2 = Low
    TIM2->CCR3 = leftSpeed;       // PWM duty cycle
  } else {
    // Reverse
    GPIOB->BSRR = GPIO_BSRR_BR7;  // BIN1 = Low
    GPIOB->BSRR = GPIO_BSRR_BS8;  // BIN2 = High
    TIM2->CCR3 = -leftSpeed;      // PWM duty cycle (positive value)
  }
}
