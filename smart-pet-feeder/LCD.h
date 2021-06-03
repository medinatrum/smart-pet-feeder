#ifndef LCD_H
#define LCD_H

#define D0 0b00000001
#define D1 0b00000010
#define D2 0b00000100
#define D3 0b00001000
#define D4 0b00010000
#define D5 0b00100000
#define D6 0b01000000
#define D7 0b10000000

#define GPIODATA GPIOC 	//GPIO port to use for data
#define GPIODATAPIND0 0 //Pin for D0 data 
#define GPIODATAPIND1 1 //Pin for D1 data 
#define GPIODATAPIND2 2 //Pin for D2 data 
#define GPIODATAPIND3 3 //Pin for D3 data 
#define GPIODATAPIND4 4 //Pin for D4 data 
#define GPIODATAPIND5 5 //Pin for D5 data 
#define GPIODATAPIND6 6 //Pin for D6 data 
#define GPIODATAPIND7 7 //Pin for D7 data 

#define GPIOCOMMAND GPIOA //GPIO port for enable, R/W and RS
#define ENABLEPIN 0 //Pin for Enable on LCD
#define RWPIN 1 //Pin for reading and writing on LCD
#define RSPIN 2 //Pin for Register select on LCD

#define BITMODE8 0 //8 bit mode
#define BITMODE4 1 //4 bit mode

#include "stm32f4xx.h"

// #include "stm32f407xx.h"

void LCDSetPin(GPIO_TypeDef *GPIO, uint32_t pin, char passedchar, uint8_t port); //Sets pins to high or low
void LCDPrintCharPins(char chartoprint); //Processes each command or data input
void LCDPrintChar(char chartoprint); //Prints a character to the LCD
void LCDPrintString(char* string); //Prints a string to the LCD
void LCDPrintInt(int number); // Prints an int to the LCD
void LCDSendCommand(char commmand); //Send command to LCD
void LCDInit(uint32_t bittype); //Initilizion of LCD driver
void LCDCommandMode(void); //Sets LCD to recieve commands
void LCDDataMode(void); //Sets LCD to recieve Data
void LCDWriteMode(void); //Sets LCD to write data/commands
void LCDReadMode(void); //Sets LCD to read data
void LCDResetCursor(void); //Reset cursor to initial position
void LCDCursorRight(void); //Moves cursor right by one
void LCDCursorLeft(void); //Moves cursor left by one
void LCDDisplayRight(void); //Shifts whole display to the right by one
void LCDDisplayLeft(void); //Shifts whole display to the left by one
void LCDEnable(void); //Enables LCD to recieve commands or data
void LCDClear(void); //Clears the Display
void LCDSecondLine(void); //Moves LCD to second line
void Delay(int time); //Delay for setup
#endif

