/* ------------------------------------------
       Demo of LCD-SHIELD buttons
             
   This test program uses one external (GPIO) button. Each time the button is 
     pressed it measures the voltage on ADC channel 8 (PTB0). This
     is the pin used for the LCD-SHIELD buttons. The result is displayed on the LCD. 
     
     HOLD THE LCD-SHIELD BUTTON DOWN and then press the external button.
     
     Hardware
     --------
     Arduino LCD shield
     Button1 : PTD pin 6
     LCD is driven using standard pins for shield
     Analog input is on PTB0 (Arduino AD0)
     GND connection to pin on LCD shield
  -------------------------------------------- */

#include <MKL25Z4.H>
#include "../include/gpio_defs.h"
#include "../include/SysTick.h"
#include "../include/LCD.h"
#include "../include/adc_defs.h"
#include <stdbool.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define BUTTONOPEN (0)	
#define BUTTONCLOSED (1)
#define BUTTONBOUNCE (2)

#define BUTTONOPEN2 (0)
#define BUTTONCLOSED2 (1)
#define BUTTONBOUNCE2 (2)

/*----------------------------------------------------------------------------
  GPIO Input Configuration

  Initialse a Port D pin as an input, with no interrupt
  Bit number given by BUTTON_POS
 *----------------------------------------------------------------------------*/ 
void configureGPIOinput(void) {
    SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; /* enable clock for port D */

    /* Select GPIO and enable pull-up resistors and no interrupts */
    PORTD->PCR[BUTTON1_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
    PORTD->PCR[BUTTON2_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
    
    /* Set port D switch bit to inputs */
    PTD->PDDR &= ~(MASK(BUTTON1_POS) | MASK(BUTTON2_POS));
}


/*----------------------------------------------------------------------------
  isPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isPressed(int button) {
    uint32_t mask = button ? MASK(BUTTON2_POS) : MASK(BUTTON1_POS) ;
    if (PTD->PDIR & mask) {
            return false ;
    }
    return true ;
}


/*----------------------------------------------------------------------------
  task 1: Poll the input

   Detect changes in the switch state.
     isPressed and not closed --> new press; 
     ~isPressed and closed -> not closed
*----------------------------------------------------------------------------*/
int b_state = BUTTONOPEN ;
int pressed = 0 ; // signal
int bounceCounter = 0 ;

int b_state2 = BUTTONOPEN2 ;
int bounceCounter2 = 0;

char test[] = "Entered Value:";
char final_value[8]; //To save final value entered
char testChar = '0';

int integer_value[8];
    

//debounce and polling period for the external button
void task1PollInput(){

  if (bounceCounter > 0) bounceCounter -- ;    

	switch (b_state) {
        
			case BUTTONOPEN :    
					if (isPressed(0)) {
									pressed = 1 ;  				// create a 'pressed' event
									b_state = BUTTONCLOSED ;
								
									lcdHome(true) ;  			// reset cursor and shift
									lcdCntrl(C_BLINK) ; 	// reset cursor
									
									setLCDAddress(0,4);
									writeLCDString(test);
									
								
								setLCDAddress(2,0);
								writeLCDString(final_value);
					}
					break ;
        
			case BUTTONCLOSED :
					if (!isPressed(0)) {
							b_state = BUTTONBOUNCE ;
							bounceCounter = 20 ;
					}
					break ;
        
			case BUTTONBOUNCE :
           if (isPressed(0)) {
               b_state = BUTTONCLOSED ;
           }
           if (bounceCounter == 0) {
               b_state = BUTTONOPEN ;
           }
           break ;  
	}	//EOF switch
} //eof function


void task2PollInput(uint32_t check_value)  {
    int i = 0;
		while(check_value    < 3100){ //Open loop if button is pressed	
				for (i = 0; i < 16; i++) { 
						// measure the voltage
						MeasureVoltage() ;
						check_value = check_value + sres ;		
				}
				check_value = check_value >> 4 ; // take average
				check_value = (1000 * check_value * VREF) / ADCRANGE ;																	
		}
}

/*----------------------------------------------------------------------------
    When button pressed, display voltage on AD0

    If the keypad button is held down, this shows the voltage
    for each keypad button.
 *----------------------------------------------------------------------------*/
// Return the character representing the least significant
// 4 bits of the parameter
char hexChar(unsigned int h) {
    h = h & 0xF ;
    if (h < 10) return h + 0x30 ;
    else return h + 0x37 ; // 10 -> 65
}

// Return the character representing the digit
// If the number is > 9, gives a '?'
char decChar(unsigned int h) {
    if (h > 9) return 0x3F ;
    return h + 0x30 ;
}

char statement[] = "Input Value:" ;		//Initital statement is printed on the screen
char statement2[] = "Entered Value:" ;//Initital statement is printed on the screen
int inc_value = 0;
int decrement = 0;
int store_inc_value[9];
int store_dec_value[9];
int button2presses = 0 ;
    int i = 0 ;
    int j = 0;
    int stop = 0;
    int value = 0;
    float check;
    uint32_t res = 0 ;
    int count = 0;
    
void task2MeasureKeypad() {    
			int j =0;
			
			if (res<3100) {
					for (j = 0; j < 16; j++){
							MeasureVoltage() ;				// Pre-defined: measure the voltage
							res = res + sres ;
					}			
					res = res >> 4 ; 							// takes average
					res = (1000 * res * VREF) / ADCRANGE ;

					//If no Button has been pressed, except the external button
					if(stop == 0){
							writeLCDString(statement);
							setLCDAddress(2,0);
							stop = 1;
					}
					
					//If the left button is pressed, else if statement is executed
					else if((2200<res) && (res <2416)){
							if(count>0){
									cursorShift(D_Left);
									store_inc_value[count] = inc_value;
									final_value[count] = integer_value[count] + '0';
									count--;
									inc_value = integer_value[count];
									decrement = integer_value[count];
									testChar = integer_value[count] + '0';
									task2PollInput(res);
							}
					}
					
					//If the up button has been pressed the else if statement is executed
					else if((504<res) && (res<740)){
							if((inc_value>=0) && (inc_value<9)){
									testChar++;                        
									integer_value[count] = testChar - '0';
									writeLCDChar(testChar);
									cursorShift(D_Left);
									inc_value++;
									task2PollInput(res);
							}
							else{
									inc_value = 0;
									testChar = '0';
									integer_value[count] = testChar - '0';
									writeLCDChar(testChar);
									cursorShift(D_Left);
									task2PollInput(res);
							}
					}
									
					//If the down button has been pressed the else if statement is executed
					else if((1370<res) && (res<1585)){
							if(((decrement>0) && (decrement<10)) ){
									testChar--;
									decrement--;
									integer_value[count] = testChar - '0';
									writeLCDChar(testChar);
									cursorShift(D_Left);
									task2PollInput(res);
							}
							else{
									decrement = 9;
									testChar = '9';
									integer_value[count] = testChar - '0';
									writeLCDChar(testChar);
									cursorShift(D_Left);
									task2PollInput(res);
							}
					}

					//If the right button has been pressed the else if statement is executed
					else if(res<0015){    
							if(count<8){
									cursorShift(D_Right);
									final_value[count] = integer_value[count] + '0';
									count++;
									inc_value = integer_value[count];
									decrement = integer_value[count];
									testChar = integer_value[count] + '0';
									task2PollInput(res);
							}
					}				
					button2presses = button2presses + 1 ;	
			} //eof big if

			for (i = 0; i < 16; i++) { 
					// measure the voltage
					MeasureVoltage() ;
					res = res + sres ;		
			}
			res = res >> 4 ; // take average
			res = (1000 * res * VREF) / ADCRANGE ;
}		//END OF task2MeasureKeypad()


/*----------------------------------------------------------------------------
  MAIN function

  Configure and then run tasks every 10ms

 *----------------------------------------------------------------------------*/

int main (void) {
    uint8_t calibrationFailed ; // zero expected
    configureGPIOinput() ;
    Init_SysTick(1000) ; 
    initLCD() ;
		Init_ADC() ;
    calibrationFailed = ADC_Cal(ADC0) ; // calibrate the ADC 
    while (calibrationFailed) 
        ; // block progress if calibration failed
		Init_ADC() ;
    lcdClear(true) ;
		waitSysTickCounter(10) ; // initialise counter
    
    while (1) {        
        task1PollInput() ;       // poll button 1
        task2MeasureKeypad() ;   // Measure ADC input and display 
      waitSysTickCounter(10) ; // cycle every 10 ms
    }
}