/* ------------------------------------------
     Motor Demo
	 Show green	 
	 When button pressed, run motor slowly clockwise; show red
   Stop on button press
   Run motor quickly backwards to starting position 

   Stepping is done by counting 10ms cycles
	 Fast: every cycle - 1 Rev in 480ms - 100pps, 125 RPM
	 Slow: every 5 cycles - 20 pps, 25 RPM
	 
	 The following GPIO pins are used for the motor
	    Motor Cnnctn   Port E Pin
			-----------------------------
         IN1           pin 30       (phase A+)
         IN2           pin 29       (phase A-)
         IN3           pin 23       (phase B+)
         IN4           pin 22       (phase B-)

	 -------------------------------------------- */

#include <MKL25Z4.H>
#include "../include/gpio_defs.h"
#include "../include/stepperMotor.h"
#include "../include/SysTick.h"
#include <stdbool.h>

#include "../include/pit.h"

#define STATESTART (0)
#define STATERUNNING (1)
#define STATERETURN (2)
#define STATESTOPPED (3)

#define BUTTONOPEN (0)
#define BUTTONCLOSED (1)
#define BUTTONBOUNCE (2)

/*----------------------------------------------------------------------------
  GPIO Configuration

  Configure the port B pin for the on-board red & green leds as an output
 *----------------------------------------------------------------------------*/
void configureGPIOoutput() {
		// Configuration steps
	//   1. Enable clock to GPIO ports
	//   2. Enable GPIO ports
	//   3. Set GPIO direction to output
	//   4. Ensure LEDs are off

	// Enable clock to ports B 
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK ;
	
	// Make the pin GPIO
	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);          
	
	// Set ports to outputs
	PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;

	// Turn off the LED
	PTB->PSOR = MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;
}

/*----------------------------------------------------------------------------
  GPIO Input Configuration

  Initialse a Port D pin as an input, with no interrupt
  Bit number given by BUTTON_POS
 *----------------------------------------------------------------------------*/ 
void configureGPIOinput(void) {
	SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; /* enable clock for port D */

	/* Select GPIO and enable pull-up resistors and no interrupts */
	PORTD->PCR[BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
	
	//usert-defined: for the external button
	PORTD->PCR[EXTRA_BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
	
	/* Set port D switch bit to inputs */
	PTD->PDDR &= ~MASK(BUTTON_POS);
	PTD->PDDR &= ~MASK(EXTRA_BUTTON_POS);	//user-defined: for the external button
}

/*----------------------------------------------------------------------------
  Motor Configuration

 *----------------------------------------------------------------------------*/
motorType mcb ;   // motor control block
MotorId m1 ;      // motor id

void configureMotor() {
	m1 = & mcb ;
	m1->port = PTE ;
  m1->bitAp = MOTOR_IN1 ;
  m1->bitAm = MOTOR_IN2 ;
  m1->bitBp = MOTOR_IN3 ;
  m1->bitBm = MOTOR_IN4 ;

	// Enable clock to port E
	SIM->SCGC5 |=  SIM_SCGC5_PORTE_MASK; /* enable clock for port E */
	
	// Initialise motor data and set to state 1
  initMotor(m1) ; // motor initially stopped, with step 1 powered
}

/*----------------------------------------------------------------------------
  ledOn: Set led LED on, assumes port B

  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/	
void ledOn(int pos)
{
	   // set led on without changing anything else
	   // LED is actve low
		 PTB->PCOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  ledOff: Set LED off, assumes port B

  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/
void ledOff(int pos)
{
	   // set led off with changing anything else
	   // LED is actve low
		 PTB->PSOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  isPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isPressed(void) {
	if (PTD->PDIR & MASK(BUTTON_POS)) {
			return false ;
	}
	return true ;
}

// user-defined: Detect external button press
bool isPressed_extraButton_2(void) {
	if (PTD->PDIR & MASK(EXTRA_BUTTON_POS)) {
			return false ;
	}
	return true ;
}

/*----------------------------------------------------------------------------
  Poll the input

 Detect changes in the switch state.
    isPressed and not closed --> new press; 
     ~isPressed and closed -> not closed
*----------------------------------------------------------------------------*/
int b_state = BUTTONOPEN ;
int pressed = 0 ;
int bounceCounter = 0;

int button2_pressed = 0;
int position = 0;
bool motor_running = false;

void task1PollInput(void)
{
	if (bounceCounter > 0) bounceCounter -- ;
	
	switch (b_state) {
		case BUTTONOPEN :
			if (isPressed() && !motor_running) {
				pressed = 1 ;  // create a 'pressed' event
				b_state = BUTTONCLOSED ;
				if (position > 7)
				{
					position = 0;
				}
			}
			else if (isPressed_extraButton_2())
			{
				button2_pressed = 1;
				b_state = BUTTONCLOSED ;
			}
			
			break ;
		case BUTTONCLOSED :
			if (!isPressed() || !isPressed_extraButton_2()) {
				b_state = BUTTONBOUNCE ;
				bounceCounter = 50 ;
			}
			break ;
		case BUTTONBOUNCE :
			if (isPressed() || isPressed_extraButton_2()) {
				b_state = BUTTONCLOSED ;
			}
			if (bounceCounter == 0) {
				b_state = BUTTONOPEN ;
			}
			break ;
	}
}



/*-----------------------------------------------------------------
  task 2 - control the LEDs

  State      LED
  -----      ---
  start      green
  running    red
  stopped    red
  return     none
 *------------------------------------------------------------------ */

int sys_state = STATESTART;

void task2ControlLight(void)
{
	switch (sys_state) {
		case STATESTART :
  			ledOn(GREEN_LED_POS) ;
	  		ledOff(RED_LED_POS) ;
			  break ;
		  case STATERUNNING:
		case STATESTOPPED:
  			ledOn(RED_LED_POS) ;
	  		ledOff(GREEN_LED_POS) ;
			  break ;
		case STATERETURN :
  			ledOff(RED_LED_POS) ;
	  		ledOff(GREEN_LED_POS) ;
			  break;
	}
}


/*----------------------------------------------------------------------------
   task3 Motor Control
       initially stopped
       STATESTART && pressed --> 
         run at speed 1, clockwise; new state STATERUNNING
       STATERUNNING  && pressed -->
         stop; get steps and command steps back; new state STATERETURN
       STATERETURN   off
         when motor stopped --> next state
       STATESTOPPED  red on
*----------------------------------------------------------------------------*/
#define FASTDELAY (0) 
#define SLOWDELAY (4) 

int counter = 0;
int delay = 0;

volatile int steps_calc ;
volatile int steps_counter = 0; 
int reset_flag = false;


int mov_arr[8] = { 64, 272, 480, 976, 512, 960, 1456, 1952 };
int mov_rot[8] = { 0, 0, 1, 1, 1, 1, 0, 0 };
int mov_time[8] = { 3276562, 770955, 436874, 214856, 204784, 109218, 72011, 53713 };
int mov_flag;

void task3ControlMotor(void)
{ 
	switch (sys_state) {
    case STATESTART :
			if (pressed && !motor_running) {
				pressed = false ; // acknowledge
				moveSteps(m1, mov_arr[position], mov_rot[position]) ; // set number of steps, set direction of turn
				setTimer(0, mov_time[position]) ; //set timing period for move
				position++;
				mov_flag = 1; //sets if motor has made a move
				button2_pressed = false; //if restart button pressed before start button, then reset state.
				
			}
			else if (button2_pressed && motor_running)//stop motor if running and restart button is pressed
			{
				stopMotor(m1) ;
				button2_pressed = false;
			}
			else if(button2_pressed && !motor_running && mov_flag)//if the motor has stopped moving and the restart button is pressed, restart motor to start position
			{
					sys_state = STATERUNNING ;
			}
			
		  break ;
			
	  case STATERUNNING:			
				if (button2_pressed)
				{
					steps_calc = steps_counter;
					steps_calc = steps_counter % 48;
					if (steps_calc > 24 )
					{
						steps_calc = 48-steps_calc;
						if (mov_rot[position - 1] == true)	//check to see if it goes counter clockwise. Reverse conditions for this.
						{
							moveSteps(m1, steps_calc, true /*mov_rot[position]*/) ; 
						}
						else
						{
							moveSteps(m1, steps_calc, false /*mov_rot[position]*/) ; 
						}
						reset_flag = true;
					}
				
					else
					{
						if (mov_rot[position - 1] == true)
						{
							moveSteps(m1, steps_calc, false /*mov_rot[position]*/) ; 
						}
						else
						{
							moveSteps(m1, steps_calc, true /*!mov_rot[position]*/) ; 
						}
						reset_flag = true;
					}
				}
				if (!motor_running)
				{
					sys_state = STATESTART ;
					button2_pressed = false ; // acknowledge
					mov_flag = 0;
				}
		  break;
	
			
	}
}

/* --------------------------------------
     Update motor state - take steps - at appropriate intervals
   -------------------------------------- */

void task4UpdateMotor() {
	if (motor_running && mov_flag)
	{
		steps_counter++;
	}
	if (!motor_running && reset_flag)
	{
		steps_counter = 0;
		reset_flag = false;
	}
	if (counter == 0) {
		counter = delay ;
		updateMotor(m1) ;
		motor_running = isMoving(m1) ;
	} else {
		if (counter > 0) counter-- ;
	}
}
	

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/

int main (void) {
	configureGPIOoutput() ;
	configureGPIOinput() ;
	configureMotor() ;
	Init_SysTick(1000) ; // SysTick every ms
	waitSysTickCounter(5) ; // initialise counter
	
	configurePIT(0) ;            // Configure PIT channel 0
	setTimer(0, 200000) ;	
	startTimer(0);
	
	while (1) {		
		task1PollInput() ;
		task2ControlLight() ;
		task3ControlMotor() ;
		
		waitSysTickCounter(5); // initialise counter
	}
}

