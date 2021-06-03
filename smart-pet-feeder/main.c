#include "stm32f4xx.h"
#include "usart.h"
#include "LCD.h"
#include "delay.h"
#include "adc.h"
#include "cs43l22.h"

#define TIME_BETWEN_FEED_IN_MIN 180
#define TIME_BETWEN_DISPLAYING_TEMP 10
#define ONE_MINUTE_IN_MS 60000  // stavit na 5000 i TIME_BETWEN_FEED_IN_MIN na 1 da bi svako 5 sekundi otvorio servo
#define DAC_BUFF_SIZE		300                            

void initIRsensor();
uint8_t getIR1state();
uint8_t getIR2state();
void initServo();
void lcdInitPins();
void openHatch();
uint8_t isTimeToOpenHatch();
void checkIRandSetLEDs();
uint8_t isTimeToDisplayTemp();
void printTempAndFoodLevel();
void getSoundForFull(void);
void getSoundForEmpty(void);

volatile uint32_t servo_timer, temp_timer;
volatile uint16_t minute_counter, temp_counter;
volatile uint8_t FULL, HALF_FULL, EMPTY;
volatile uint32_t TEMP;
volatile uint16_t g_dac_buff[DAC_BUFF_SIZE];

int main(void){
   
    initSYSTIM();
	initUSART2(USART2_BAUDRATE_115200);
    enIrqUSART2();
	initADC1Temp();
    initServo();
    initIRsensor();
    initCS43L22(50, 48000, (uint16_t *)g_dac_buff, DAC_BUFF_SIZE);          //volumen, fs, niz, velicina
    // lcdInitPins();
    // LCDInit(BITMODE4);

    FULL = HALF_FULL = 0;
    EMPTY = 1;
	minute_counter = 0;
    
    servo_timer = getSYSTIM();
    temp_timer = getSYSTIM();

    while(1){

        TEMP = getADC1Temp();
        if(isTimeToDisplayTemp())
            printTempAndFoodLevel(TEMP);
       
		checkIRandSetLEDs();
        if(isTimeToOpenHatch())
            openHatch();

    }
    return 0;
}


void printTempAndFoodLevel(){
    // LCDPrintInt(TEMP);
    // LCDSecondLine();
    printUSART2("Temp: %d C\n", TEMP);

    if(FULL){
        // LCDPrintString("Food: FULL");
        printUSART2("Food: FULL\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    }
    else if(HALF_FULL) {
        // LCDPrintString("Food: HALF FULL");
        printUSART2("Food: HALF FULL\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    }
    else {
        // LCDPrintString("Food: EMPTY");
        printUSART2("Food: EMPTY\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    }
}

void initIRsensor(){

    // init PD12 & PD13 LED's as output
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  								
	GPIOD->MODER &= ~(0x0F000000);
    GPIOD->MODER |= 0x05000000;             						
    GPIOD->OTYPER |= ~(0x3000);									 
    GPIOD->OSPEEDR |= 0x0F000000; 	
    GPIOD->ODR &= ~0x3FFF;

    // init PA1 & PC13 as input
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~0x0000000C;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~0x0C000000; // PC13
}

uint8_t getIR1state(){
    if((GPIOA->IDR & 0x0002) == 0x0002)
        return 1;
    else 
        return 0;
}
uint8_t getIR2state(){
    if((GPIOC->IDR & 0x2000) == 0x2000)
        return 1;  
    else 
        return 0;
}

void initServo(){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  							
    GPIOD->MODER |= 0x80000000;             						
    GPIOD->OTYPER |= 0x00000000; 									
    GPIOD->OSPEEDR |= 0xC0000000;                                   // spped high
    GPIOD->PUPDR |= 0x40000000;                                     // pull up
	GPIOD->AFR[1] |= 0x20000000;									  
    // GPIOD->AFR[0] |= 0x20000000;									  

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; 							// enable TIM4 on APB1 
    TIM4->PSC = 0x0054 - 0x0001;									// set TIM4 counting prescaler 
                                                                    // 84MHz/84 = 1MHz -> count each 1us
    TIM4->ARR = 0x4e20;												// period of the PWM 20ms
    
    TIM4->CCR4 = 500;
    
    TIM4->CCMR1 |= (TIM_CCMR1_OC1PE)|(TIM_CCMR1_OC1M_2)|(TIM_CCMR1_OC1M_1);
    TIM4->CCMR1 |= (TIM_CCMR1_OC2PE)|(TIM_CCMR1_OC2M_2)|(TIM_CCMR1_OC2M_1);	
    TIM4->CCMR2 |= (TIM_CCMR2_OC3PE)|(TIM_CCMR2_OC3M_2)|(TIM_CCMR2_OC3M_1);	
    TIM4->CCMR2 |= (TIM_CCMR2_OC4PE)|(TIM_CCMR2_OC4M_2)|(TIM_CCMR2_OC4M_1);					
                                                                        // set PWM 1 mod, enable OC1PE preload mode 
                                                                        
    // set active mode high for pulse polarity
    TIM4->CCER &= ~((TIM_CCER_CC1P)|(TIM_CCER_CC2P)|(TIM_CCER_CC3P)|(TIM_CCER_CC4P));
    TIM4->CR1 |= (TIM_CR1_ARPE)|(TIM_CR1_URS);
    
    // update event, reload all config 
    TIM4->EGR |= TIM_EGR_UG;											
    // activate capture compare mode
    TIM4->CCER |= (TIM_CCER_CC1E)|(TIM_CCER_CC2E)|(TIM_CCER_CC3E)|(TIM_CCER_CC4E);
    // start counter										
    TIM4->CR1 |= TIM_CR1_CEN;		
}

void lcdInitPins(){
    //INITILIZE START;
	// Enable the GPIO Clock for GPIO
	RCC->AHB1ENR |= (1 << 2);
	RCC->AHB1ENR |= (1 << 0);
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  								//
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  								//

	// Set any Control Registers for GPIO pins
	// Moder
	GPIOA->MODER |= 0b01010101; // 0000 1011 0000 0001 0000 0001 0000 0001 0000 0001
	GPIOC->MODER |= 0b0101010101010101;  // 0000 1011 0000 0001 0000 0001 0000 0001 0000 0001 0000 0001 0000 0001 0000 0001 0000 0001
	// GPIOA->MODER |= 0x00000015; 
	// GPIOC->MODER |= 0x00005555;
	
    // OTyper
	GPIOA->OTYPER = 0;
	GPIOC->OTYPER = 0;
	// // OSpeedr
	GPIOA->OSPEEDR = 0b10101010;
	GPIOC->OSPEEDR = 0b1010101010101010;
	// PUPDr
	GPIOA->PUPDR = 0;
	GPIOC->PUPDR = 0b1010101010101010;
	//INITILIZE END
}

void openHatch(){
    // servo - 90, 0
    GPIOC->MODER |= 0x01000000; // PC12 za dc motor
    TIM4->CCR4 = 1500;
    delay_ms(10000);
    TIM4->CCR4 = 500;
    GPIOC->MODER &= ~0x01000000; // PC12 za dc motor

    checkIRandSetLEDs();
    if(FULL) 
        getSoundForFull();
    else 
        getSoundForEmpty();
    
    printTempAndFoodLevel(TEMP);
}

uint8_t isTimeToOpenHatch(){
    if(chk4TimeoutSYSTIM(servo_timer, ONE_MINUTE_IN_MS) == SYSTIM_TIMEOUT){
        ++minute_counter;
        servo_timer = getSYSTIM();
    }
    if(minute_counter == TIME_BETWEN_FEED_IN_MIN){
        minute_counter = 0;
        return 1;
    }
    return 0;
}

uint8_t isTimeToDisplayTemp(){
    if(chk4TimeoutSYSTIM(temp_timer, ONE_MINUTE_IN_MS) == SYSTIM_TIMEOUT){
        ++temp_counter;
        temp_timer = getSYSTIM();
    }
    if(temp_counter == TIME_BETWEN_DISPLAYING_TEMP){
        temp_counter = 0;
        return 1;
    }
    return 0;
}

void checkIRandSetLEDs(){
    if(!getIR1state()){ 
        GPIOD->ODR &= ~0x3FFF;
        GPIOD->ODR |= 0x1FFF;
        if(!getIR2state()){ // puna
            FULL = 1;
            HALF_FULL = EMPTY = 0;
        }else{              // polu puna
            FULL = EMPTY = 0;
            HALF_FULL = 1;
        }
    }else{ // prazna
        GPIOD->ODR &= ~0x3FFF;
        GPIOD->ODR |= 0x2000;
        EMPTY = 1;
        FULL = HALF_FULL = 0;
    }
    
}

void getSoundForFull(void) {
    uint16_t n;
	float y, t =0;
	float freq = 400;
         for(n=0;n<(DAC_BUFF_SIZE/3);n++)
	{
	    y = 0.4*sinf(2*PI*freq*t)+0.5;
	
		g_dac_buff[n] = (uint16_t)(y*32767);
		
		t = t + 0.1e-4;
	}
        freq= 800;
         for(n=100;n<(DAC_BUFF_SIZE/3);n++)
	{
	    y = 0.4*sinf(2*PI*freq*t)+0.5;
	
		g_dac_buff[n] = (uint16_t)(y*32767);
		
		t = t + 0.1e-4;
	}
        freq = 400;
        for(n=200;n<(DAC_BUFF_SIZE/3);n++)
	{
	    y = 0.4*sinf(2*PI*freq*t)+0.5;
	
		g_dac_buff[n] = (uint16_t)(y*32767);
		
		t = t + 0.1e-4;
	}
}

void getSoundForEmpty(void) {
    uint16_t n;
	float y, t =0;
	float freq = 800; 
   for(n=0;n<(DAC_BUFF_SIZE/2);n++)
	{
	    y = 0.4*sinf(2*PI*freq*t)+0.5;
	
		g_dac_buff[n] = (uint16_t)(y*32767);
		
		t = t + 0.1e-4;
	}
         freq= 200;
         for(n=100;n<(DAC_BUFF_SIZE/2);n++)
	{
	    y = 0.4*sinf(2*PI*freq*t)+0.5;
	
		g_dac_buff[n] = (uint16_t)(y*32767);
		
		t = t + 0.1e-4;
	}
}
