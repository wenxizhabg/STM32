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
#define GPIOB 	((GPIO_TypeDef *)	0x40020400)
#define RCC 	((RCC_TypeDef *)	0x40023800)


void delay(int time) {
    for (volatile long i = 0; i < time*100000; i++); // Delay đơn giản
}
void turnOnLed () {
	GPIOA->ODR |= (1 << 0);
	delay(1);

	GPIOA->ODR |= (1 << 1);
	delay(1);

	GPIOA->ODR |= (1 << 2);
	delay(1);

	GPIOA->ODR |= (1 << 3);
	delay(1);
}

void turnOffLed () {
	delay(1);
	GPIOA->ODR &= ~(1 << 3);

	delay(1);
	GPIOA->ODR &= ~(1 << 2);

	delay(1);
	GPIOA->ODR &= ~(1 << 1);

	delay(1);
	GPIOA->ODR &= ~(1 << 0);
}

int main(void) {

    // 1. Bật clock cho GPIOB
    RCC->AHB1ENR|= (3 << 0);  // Bật clock GPIOB

    GPIOB->MODER |=  (0 << 20); // Đặt chế độ input ở chân B10

    GPIOB->MODER |=  (1 << 24); // Đặt chế độ output ở chân B12
    GPIOB->OTYPER |=  (0 << 12); // Đặt kiểu output B12 là push-pull
    GPIOB->OSPEEDR |= (3 << 24); // Đặt speed output chân B12 laf high

    GPIOA->MODER |= 0b01010101;			// Thiết lập mode out ở chân A0123
    GPIOA->OSPEEDR |= 0b11111111;			// Thiết lập speed ở chân A0123



    printf("Bắt đầu chạy...\n");

    int trangThaiDen = 0;
    int process = 0;
    while (1) {

    	    	if ((GPIOB->IDR & (1 << 10)) != 0) { //Đọc giá trị từ cảm biến dò line ở chân B10
    		trangThaiDen = (trangThaiDen + 1) % 2;
    	}

    	if (trangThaiDen == 1 && process == 0) {
    		turnOnLed();
    		process = 1;
    	}

    	if (trangThaiDen == 0 && process == 1) {
    	    turnOffLed();
    		process = 0;
       	}



        delay(0); // Thêm độ trễ để không in quá nhanh
    }
}
