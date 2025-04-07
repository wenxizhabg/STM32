//#include <stdint.h>
//
//typedef struct {
//    volatile uint32_t ACR;			volatile uint32_t KEYR;
//    volatile uint32_t OPTKEYR;		volatile uint32_t SR;
//    volatile uint32_t CR;    		volatile uint32_t OPTCR;
//} FLASH_TypeDef;
//
//
//typedef struct {
//    volatile uint32_t CR;			volatile uint32_t PLLCFGR;		volatile uint32_t CFGR;				volatile uint32_t CIR;
//    volatile uint32_t AHB1RSTR;    	volatile uint32_t AHB2RSTR;	    volatile uint32_t RESERVED0x18;    	volatile uint32_t RESERVED0x1C;
//    volatile uint32_t APB1RSTR;	    volatile uint32_t APB2RSTR;	    volatile uint32_t RESERVED0x28;		volatile uint32_t RESERVED0x2C;
//    volatile uint32_t AHB1ENR;	    volatile uint32_t AHB2ENR;	    volatile uint32_t RESERVED0x38;	    volatile uint32_t RESERVED0x3C;
//    volatile uint32_t APB1ENR;	    volatile uint32_t APB2ENR;	    volatile uint32_t RESERVED0x48;	    volatile uint32_t RESERVED0x4C;
//    volatile uint32_t AHB1LENR;	    volatile uint32_t AHB2LENR;	    volatile uint32_t RESERVED0x58;	    volatile uint32_t RESERVED0x5C;
//    volatile uint32_t APB1LENR;	    volatile uint32_t APB2LENR;	    volatile uint32_t RESERVED0x68;	    volatile uint32_t RESERVED0x6C;
//    volatile uint32_t BDCR;		    volatile uint32_t CSR;		    volatile uint32_t RESERVED0x78;	    volatile uint32_t RESERVED0x7C;
//    volatile uint32_t SSCGR;	    volatile uint32_t PLLI2SCFGR;	volatile uint32_t RESERVED0x88;	    volatile uint32_t DCKCFGR;
//} RCC_TypeDef;
//
//typedef struct {
//    volatile uint32_t MODER;	volatile uint32_t OTYPER;
//    volatile uint32_t OSPEEDR;	volatile uint32_t PUPDR;
//    volatile uint32_t IDR;	    volatile uint32_t ODR;
//    volatile uint32_t BSRR;		volatile uint32_t LCKR;
//    volatile uint32_t AFRL;		volatile uint32_t AFRH;
//} GPIO_TypeDef;
//
//typedef struct {
//    volatile uint32_t CR1;			volatile uint32_t CR2;			volatile uint32_t SMCR;				volatile uint32_t DIER;
//    volatile uint32_t SR;		    volatile uint32_t EGR;          volatile uint32_t CCMR1;     	    volatile uint32_t CCMR2;
//    volatile uint32_t CCER;     	volatile uint32_t CNT;       	volatile uint32_t PSC;      		volatile uint32_t ARR;
//    volatile uint32_t RCR;		    volatile uint32_t CCR1;		    volatile uint32_t CCR2;      	    volatile uint32_t CCR3;
//    volatile uint32_t CCR4;		    volatile uint32_t BDTR;      	volatile uint32_t DCR;       		volatile uint32_t DMAR;      // DMA address for full transfer
//} TIM_TypeDef;
//#define TIM2	((TIM_TypeDef *)	0x40000000)
//
//#define FLASH 	((FLASH_TypeDef *)	0x40023C00)
//#define GPIOB 	((GPIO_TypeDef *)	0x40020400)
//#define RCC 	((RCC_TypeDef *)	0x40023800)
//
//#define PWR_CR (volatile uint32_t*) 0x40007000
//
//
//void set100MHzClock () {
//			// Cấu hình Clock control
//			// Mặc định HSI bật và HSE tắt
//
//			RCC->CR |= (1<<0);	// bật hsi
//
//
//			*PWR_CR |= (3 << 14);
//			FLASH->ACR |= (3 << 0);					// Đặt độ trễ bộ nhớ flash = 3
//			FLASH->ACR |= (0b111 << 8);
//
//			// Mặc định HSI ở trạng thái bật
//			// Mặc định HSE, PLL và PLLI2S ở trạng thái tắt
//			// PLL cần tắt trước khi cấu hình PLL
//
//			RCC->APB1ENR |= (1 << 28);
//
//
//			// Mặc định HSI là main source PLL
//			RCC->PLLCFGR &= ~(1<<22);
//
//			// Cấu hình PLL:
//			RCC->PLLCFGR = 0;
//						RCC->PLLCFGR |= (8 << 0);				// Đặt PLLM = 8
//						RCC->PLLCFGR |= (100 << 6);				// Đặt PLLN = 100
//						// Mặc định PLLP = 2
//						RCC->PLLCFGR &= ~(0b11 << 16);
//			// Cấu hình System Clock:
//			// Mặc định  AHB prescaler = 1
//			RCC->CFGR |= (0b100 << 10);				// Đặt APB1 prescaler = 2
//			// Mặc định APB2 precscaler = 1
//
//			RCC->CR |= (1 << 24);					// Bật PLL
//			while ((RCC->CR & (1 << 25)) == 0 );	// Đợi PLL sẵn sàng
//
//
//
//			RCC->CFGR |= 0b10;						// Đặt System clock là PLL
//			while ((RCC->CFGR & (0b10 << 2)) == 0);	// Đợi trạng thái system clock là PLL
//}
//
//void setGPIOB () {
//	GPIOB->MODER = 0;
//
//	GPIOB->AFRL		|=	(0b0001 << 12);					// Set AF01 ở B3
//	GPIOB->AFRH		|=	(0b0001 << 8);					// Set AF01 ở B10
//	GPIOB->MODER	|=	(0b10 << 6) | (0b10 << 20);		// Mở AF ở B3 & B10
//	GPIOB->MODER 	|=	(0b0101010101 << 8);				// Set Ouutput chân B45678
//	GPIOB->OSPEEDR	|=	(0b1111111111 << 8);  			// Set highspeed chaan B45678.
//
//}
//
//void setTimer2 () {
//		// Thiết lập fPWM = 20kHz
//		TIM2->PSC 	 = 50 - 1;
//		TIM2->ARR 	 = 100 - 1;
//
//		// Thiết lập dutyCycle:
//		TIM2->CCR2 	 = 100; 				// Kênh 2
//		TIM2->CCR3 	 = 100; 				// Kênh 3
//
//		// Mặc định TIM2_CCMR được thiết lập chế độ output ở kênh2 và kênh 3
//		// Thiết lập chế độ kênh 3
//		TIM2->CCMR2    |= (1 << 3);			// Bật preload
//		TIM2->CCMR2    |= (0b110 << 4);		// Đặt chế độ PWM1
//		TIM2->CCER 	   |= (1 << 8);			// Kích hoạt bộ so sánh
//
//		// Thiết lập chế độ kênh 2
//		TIM2->CCMR1    |= (1 << 11);		// Bật preload
//		TIM2->CCMR1    |= (0b110 << 12);	// Đặt chế độ PWM1
//		TIM2->CCER 	   |= (1 << 4);			// Kích hoạt bộ so sánh
//
//		// Mở bộ đếm
//		TIM2->CR1	  |= (1 << 0);
//
//}
//
//void enableCLOCK () {
//	RCC->AHB1ENR  |= (0b1 << 1 );  	// Bật clock GPIOB
//	RCC->APB1ENR  |= (0b1 << 0 ); 	// Bật clock TIM2
//}
//int main () {
//	set100MHzClock ();
//	enableCLOCK ();
//
//	setGPIOB ();
//	setTimer2();
//	GPIOB->ODR |= (0b01101 << 4);
//
//
//
//
//
//	TIM2->CCR3 	 = 50;
//	TIM2->CCR2 	 = 50;
//	while (1) {
//		TIM2->CCR3 	 = 50;
//		TIM2->CCR2 	 = 50;
//		if (((GPIOB->IDR >> 1)& 1) == 0) {
//			GPIOB->ODR |= (1 <<5);
//			GPIOB->ODR &= ~(1 <<4);
//		} else {
//			GPIOB->ODR &= ~(1 << 5);
//			GPIOB->ODR |= (1 <<4);
//		}
//
//		if (((GPIOB->IDR >> 0)& 1) == 0) {
//			GPIOB->ODR |= (1 <<8);
//			GPIOB->ODR &= ~(1 <<7);
//		} else {
//			GPIOB->ODR |= (1 <<7);
//			GPIOB->ODR &= ~(1 <<8);
//		}
//	}
//
//
//
//}


#include <stdint.h>

//-------------------- Struct Peripheral --------------------
typedef struct {
    volatile uint32_t ACR;			volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;		volatile uint32_t SR;
    volatile uint32_t CR;    		volatile uint32_t OPTCR;
} FLASH_TypeDef;

typedef struct {
    volatile uint32_t CR;			volatile uint32_t PLLCFGR;		volatile uint32_t CFGR;				volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;    	volatile uint32_t AHB2RSTR;	    volatile uint32_t RESERVED0x18;    	volatile uint32_t RESERVED0x1C;
    volatile uint32_t APB1RSTR;	    volatile uint32_t APB2RSTR;	    volatile uint32_t RESERVED0x28;		volatile uint32_t RESERVED0x2C;
    volatile uint32_t AHB1ENR;	    volatile uint32_t AHB2ENR;	    volatile uint32_t RESERVED0x38;	    volatile uint32_t RESERVED0x3C;
    volatile uint32_t APB1ENR;	    volatile uint32_t APB2ENR;	    volatile uint32_t RESERVED0x48;	    volatile uint32_t RESERVED0x4C;
    volatile uint32_t AHB1LENR;	    volatile uint32_t AHB2LENR;	    volatile uint32_t RESERVED0x58;	    volatile uint32_t RESERVED0x5C;
    volatile uint32_t APB1LENR;	    volatile uint32_t APB2LENR;	    volatile uint32_t RESERVED0x68;	    volatile uint32_t RESERVED0x6C;
    volatile uint32_t BDCR;		    volatile uint32_t CSR;		    volatile uint32_t RESERVED0x78;	    volatile uint32_t RESERVED0x7C;
    volatile uint32_t SSCGR;	    volatile uint32_t PLLI2SCFGR;	volatile uint32_t RESERVED0x88;	    volatile uint32_t DCKCFGR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t MODER;	volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;	volatile uint32_t PUPDR;
    volatile uint32_t IDR;	    volatile uint32_t ODR;
    volatile uint32_t BSRR;		volatile uint32_t LCKR;
    volatile uint32_t AFRL;		volatile uint32_t AFRH;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1;			volatile uint32_t CR2;			volatile uint32_t SMCR;				volatile uint32_t DIER;
    volatile uint32_t SR;		    volatile uint32_t EGR;          volatile uint32_t CCMR1;     	    volatile uint32_t CCMR2;
    volatile uint32_t CCER;     	volatile uint32_t CNT;       	volatile uint32_t PSC;      		volatile uint32_t ARR;
    volatile uint32_t RCR;		    volatile uint32_t CCR1;		    volatile uint32_t CCR2;      	    volatile uint32_t CCR3;
    volatile uint32_t CCR4;		    volatile uint32_t BDTR;      	volatile uint32_t DCR;       		volatile uint32_t DMAR;
} TIM_TypeDef;

typedef struct {
    volatile uint32_t MEMRMP;        volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];     volatile uint32_t CMPCR;
} SYSCFG_TypeDef;

typedef struct {
    volatile uint32_t IMR;       volatile uint32_t EMR;
    volatile uint32_t RTSR;      volatile uint32_t FTSR;
    volatile uint32_t SWIER;     volatile uint32_t PR;
} EXTI_TypeDef;

//-------------------- Define Address --------------------
#define FLASH 	((FLASH_TypeDef *)	0x40023C00)
#define GPIOB 	((GPIO_TypeDef *)	0x40020400)
#define RCC 	((RCC_TypeDef *)	0x40023800)
#define TIM2	((TIM_TypeDef *)	0x40000000)
#define SYSCFG  ((SYSCFG_TypeDef *) 0x40013800)
#define EXTI    ((EXTI_TypeDef *)   0x40013C00)
#define PWR_CR  (volatile uint32_t*) 0x40007000

//-------------------- Clock Setup --------------------
void set100MHzClock () {
	RCC->CR |= (1<<0);	// bật HSI
	*PWR_CR |= (3 << 14);
	FLASH->ACR |= (3 << 0);					// Flash latency = 3
	FLASH->ACR |= (0b111 << 8);

	RCC->APB1ENR |= (1 << 28);				// Power interface clock

	RCC->PLLCFGR &= ~(1<<22);				// PLL source = HSI
	RCC->PLLCFGR = 0;
	RCC->PLLCFGR |= (8 << 0);				// PLLM = 8
	RCC->PLLCFGR |= (100 << 6);				// PLLN = 100
	RCC->PLLCFGR &= ~(0b11 << 16);			// PLLP = 2

	RCC->CFGR |= (0b100 << 10);				// APB1 prescaler = 2

	RCC->CR |= (1 << 24);					// Enable PLL
	while ((RCC->CR & (1 << 25)) == 0 );	// Wait for PLL ready

	RCC->CFGR |= 0b10;						// Set system clock to PLL
	while ((RCC->CFGR & (0b10 << 2)) == 0);	// Wait for system clock ready
}

//-------------------- GPIOB Setup --------------------
void setGPIOB () {
	GPIOB->MODER = 0;

	GPIOB->AFRL |= (0b0001 << 12);					// AF1 - PB3
	GPIOB->AFRH |= (0b0001 << 8);					// AF1 - PB10
	GPIOB->MODER |= (0b10 << 6) | (0b10 << 20);		// Alternate function PB3, PB10

	GPIOB->MODER |= (0b0101010101 << 8);			// Output mode PB4-PB8
	GPIOB->OSPEEDR |= (0b1111111111 << 8);			// High speed PB4-PB8

	GPIOB->PUPDR |= (0b01 << 0) | (0b01 << 2);		// Pull-up for PB0, PB1
}

//-------------------- Timer2 Setup --------------------
void setTimer2 () {
	TIM2->PSC = 50 - 1;
	TIM2->ARR = 100 - 1;

	TIM2->CCR2 = 100;
	TIM2->CCR3 = 100;

	TIM2->CCMR2 |= (1 << 3) | (0b110 << 4);		// Channel 3: Preload + PWM1
	TIM2->CCER |= (1 << 8);

	TIM2->CCMR1 |= (1 << 11) | (0b110 << 12);	// Channel 2: Preload + PWM1
	TIM2->CCER |= (1 << 4);

	TIM2->CR1 |= (1 << 0);						// Counter enable
}

//-------------------- Enable Peripheral Clocks --------------------
void enableCLOCK () {
	RCC->AHB1ENR |= (1 << 1);	// GPIOB
	RCC->APB1ENR |= (1 << 0);	// TIM2
}

//-------------------- EXTI Setup --------------------
void enableEXTIforPB0_PB1() {
	RCC->APB2ENR |= (1 << 14); // SYSCFG clock

	SYSCFG->EXTICR[0] &= ~(0xF << 0); // EXTI0 = PB0
	SYSCFG->EXTICR[0] |=  (0x1 << 0);

	SYSCFG->EXTICR[0] &= ~(0xF << 4); // EXTI1 = PB1
	SYSCFG->EXTICR[0] |=  (0x1 << 4);

	EXTI->IMR |= (1 << 0) | (1 << 1);	// Unmask EXTI0 & EXTI1
	EXTI->RTSR |= (1 << 0) | (1 << 1);	// Rising edge trigger
	EXTI->FTSR |= (1 << 0) | (1 << 1);	// Falling edge trigger

	*(uint32_t *)0xE000E100 |= (1 << 6); // Enable EXTI0 IRQ in NVIC
	*(uint32_t *)0xE000E100 |= (1 << 7); // Enable EXTI1 IRQ in NVIC
}

//-------------------- Interrupt Handlers --------------------
void EXTI0_IRQHandler(void) {
			if (((GPIOB->IDR >> 0)& 1) == 0) {
				GPIOB->ODR |= (1 <<8);
				GPIOB->ODR &= ~(1 <<7);
			}else{
				GPIOB->ODR |= (1 <<7);
				GPIOB->ODR &= ~(1 <<8);
			}
}

void EXTI1_IRQHandler(void) {
			if (((GPIOB->IDR >> 1)& 1) == 0) {
				GPIOB->ODR |= (1 <<5);
				GPIOB->ODR &= ~(1 <<4);}else{
					GPIOB->ODR &= ~(1 << 5);
						GPIOB->ODR |= (1 <<4);
				}
}

//-------------------- MAIN --------------------
int main () {
	set100MHzClock ();
	enableCLOCK ();
	setGPIOB ();
	setTimer2 ();
	enableEXTIforPB0_PB1();

	GPIOB->ODR |= (0b01101 << 4);  // Set initial state PB4-PB8
	TIM2->CCR3 = 30;
	TIM2->CCR2 = 30;

	while (1) {



	}
}
