#include <stdbool.h>

// Definitions for GPIO
#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H


#define MASK(x) (1UL << (x))

// Button is on port D, pin 6
#define BUTTON_POS (6)
// Button is on port D, pin 7
#define BUTTON_POS_2 (7)

// GPIO output used for the frequency, port A pin 2
#define AUDIO_POS (2)


// Function prototypes
void configureGPIOinput(void) ;       // Initialise button
void configureGPIOoutput(void) ;      // Initialise output	
void audioToggle(void) ;          // Toggle the output GPIO
bool isPressed(void) ;

#endif
