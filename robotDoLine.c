#include <stdint.h>

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
    volatile uint32_t CCR4;		    volatile uint32_t BDTR;      	volatile uint32_t DCR;       		volatile uint32_t DMAR;      // DMA address for full transfer
} TIM_TypeDef;
#define TIM9	((TIM_TypeDef *)	0x40014000)

#define FLASH 	((FLASH_TypeDef *)	0x40023C00)
#define GPIOA 	((GPIO_TypeDef *)	0x40020000)
#define RCC 	((RCC_TypeDef *)	0x40023800)

void set100MHzClock () {
			// Cấu hình Clock control
			// Mặc định HSI bật và HSE tắt


			// Mặc định HSI ở trạng thái bật
			// Mặc định HSE, PLL và PLLI2S ở trạng thái tắt
			// PLL cần tắt trước khi cấu hình PLL


			// Cấu hình PLL:
			RCC->PLLCFGR = (16 << 0);				// Đặt PLLM = 16
			RCC->PLLCFGR = (200 << 6);				// Đặt PLLN = 200
			// Mặc định PLLP = 2
			// Mặc định HSI là main source PLL

			// Cấu hình System Clock:
			// Mặc định  AHB prescaler = 1
			RCC->CFGR |= (0b100 << 10);				// Đặt APB1 prescaler = 2
			// Mặc định APB2 precscaler = 1

			RCC->CR |= (1 << 24);					// Bật PLL
			while ((RCC->CR & (1 << 25)) == 0 );	// Đợi PLL sẵn sàng

			FLASH->ACR |= (3 << 0);					// Đặt độ trễ bộ nhớ flash = 3

			RCC->CFGR |= 0b10;						// Đặt System clock là PLL
			while ((RCC->CFGR & (0b10 << 2)) == 0);	// Đợi trạng thái system clock là PLL
}

void setGPIOA () {
	GPIOA->MODER	|=	(0b10 << 4) | (0b10 << 6);		// Mở AF ở A2 & A3
	GPIOA->AFRL		|=	(0b11 << 8) | (0b11 << 12);		// Set AF03 ở A2 & A3

}

void setTimer9 () {
		// Thiết lập fPWM = 20kHz
		TIM9->PSC 	 = 50 - 1;
		TIM9->ARR 	 = 100 - 1;

		// Thiết lập dutyCycle:
		TIM9->CCR1 	 = 100; 				// Kênh 1
		TIM9->CCR2 	 = 100; 				// Kênh 2

		// Mặc định TIM9_CCMR được thiết lập chế độ output ở kênh1 và kênh 2
		// Thiết lập chế độ kênh 1
		TIM9->CCMR1    |= (1 << 3);			// Bật preload
		TIM9->CCMR1    |= (0b110 << 4);		// Đặt chế độ PWM1
		TIM9->CCER 	   |= (1 << 0);			// Kích hoạt bộ so sánh

		// Thiết lập chế độ kênh 2
		TIM9->CCMR1    |= (1 << 11);		// Bật preload
		TIM9->CCMR1    |= (0b110 << 12);	// Đặt chế độ PWM1
		TIM9->CCER 	   |= (1 << 4);			// Kích hoạt bộ so sánh

		// Mở bộ đếm
		TIM9->CR1	  |= (1 << 0);

}

void enableCLOCK () {
	RCC->AHB1ENR  |= (0b1 << 0 );  	// Bật clock GPIOA
	RCC->APB2ENR  |= (0b1 << 16 ); 	// Bật clock TIM9
}
int main () {
	set100MHzClock ();
	enableCLOCK ();

	setGPIOA ();
	setTimer9();

	while (1) {
		if (GPIOA->IDR & (1 << 9)) TIM9->CCR1 = 50;
		else TIM9->CCR1 = 100;

		if (GPIOA->IDR & (1 << 10)) TIM9->CCR2 = 50;
		else TIM9->CCR2 = 100;



	}



}
