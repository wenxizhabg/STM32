#include <stdio.h>
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

#define FLASH 	((FLASH_TypeDef *)	0x40023C00)
#define GPIOA 	((GPIO_TypeDef *)	0x40020000)
#define RCC 	((RCC_TypeDef *)	0x40023800)

void delay(long time) {
    for (volatile long i = 0; i < time*150000; i++); // Delay đơn giản
}

void setClock100MHz () {
	// Cấu hình Clock control
		// Mặc định HSI bật và HSE tắt

		// HSI bật
		RCC->CR |= (1 << 0);					// Bật HSI
		while ((RCC->CR & (1 << 1)) == 0 ); 	// Đợi HSI bật

		// Mặc định HSI ở trạng thái bật
		// Mặc định HSE, PLL và PLLI2S ở trạng thái tắt
		// PLL cần tắt trước khi cấu hình PLL


		// Cấu hình PLL:
		RCC->PLLCFGR |= (16 << 0);			// Đặt PLLM = 16
		RCC->PLLCFGR |= (200 << 6);			// Đặt PLLN = 200
		// Mặc định PLLP = 2
		// Mặc định HSI là main source PLL

		// Cấu hình System Clock:
		// Mặc định  AHB prescaler = 1
		RCC->CFGR |= (0b100 << 10);			// Đặt APB1 prescaler = 2
		// Mặc định APB2 precscaler = 1

		RCC->CR |= (1 << 24);					// Bật PLL
		while ((RCC->CR & (1 << 25)) == 0 );		// Đợi PLL sẵn sàng

		FLASH->ACR |= (3 << 0);					// Đặt độ trễ bộ nhớ flash = 3

		RCC->CFGR |= 0b10;						// Đặt System clock là PLL
		while ((RCC->CFGR & (0b10 << 2)) == 0);	// Đợi trạng thái system clock là PLL

}

int main () {
		////////



    RCC->AHB1ENR  |= (0b1 << 0 );  // Bật clock GPIOA

    GPIOA->MODER |= 0b01010101;				// Thiết lập mode out ở chân A0123
    GPIOA->OSPEEDR |= 0b11111111;			// Thiết lập speed ở chân A0123

    while (1) {

    	GPIOA->ODR |= (1 << 0);
    	GPIOA->ODR &= ~(1 << 2);
    	delay(3);
    	GPIOA->ODR |= (1 << 1);
    	GPIOA->ODR &= ~(1 << 3);
    	delay(3);
    	GPIOA->ODR |= (1 << 2);
    	GPIOA->ODR &= ~(1 << 0);
    	delay(3);
    	GPIOA->ODR |= (1 << 3);
    	GPIOA->ODR &= ~(1 << 1);
    	delay(3);
    }

}
