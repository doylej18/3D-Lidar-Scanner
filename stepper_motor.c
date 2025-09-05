// COMPENG 2DX3
// This program illustrates the interfacing of the Stepper Motor with the microcontroller
//  Written by Ama Simons
//  January 18, 2020
// 	Last Update by Dr. Shahrukh Athar on February 2, 2025
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
int current_position = 0;  // Tracks the stepper motor's position
const int home_position = 0;  // Predefined "home" position
 
void PortN_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};
    GPIO_PORTN_DIR_R = 0b00000011;  														// PN1 and PN0 as outputs
    GPIO_PORTN_DEN_R = 0b00000011;															// D1 is PN1 and D2 is PN0
		return;
}
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    GPIO_PORTF_DIR_R = 0b00010001;  														// PF4 and PF0 as outputs
    GPIO_PORTF_DEN_R = 0b00010001;															// D3 is PF4 and D4 is PF0
		return;
}
void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;	
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};
	GPIO_PORTM_DIR_R = 0b00000000;       								// PM1 and PM0 as input
  GPIO_PORTM_DEN_R = 0b00000011;        							// enable digital I/O on PM1 and PM0
	GPIO_PORTM_PUR_R |= 0b00000011;														// enable weak pull-up resistor
		return;
}
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;									// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};					// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R = 0b00000000;       											// PJ1 and PJ0 as input
  GPIO_PORTJ_DEN_R = 0b00000011;        										// enable digital I/O on PJ1 and PJ0
	GPIO_PORTJ_PCTL_R &= ~0x000000FF;													// configure PJ1 and PJ0 as GPIO
	GPIO_PORTJ_AMSEL_R &= ~0b00000011;								// Disable analog functionality on PJ1 and PJ0		
	GPIO_PORTJ_PUR_R |= 0b00000011;										// Enable weak pull up resistor on PJ1 and PJ0
	return;
}
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;							// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};			// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        											// configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     											// disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;        											// enable digital I/O on Port H pins (PH0-PH3)
																												// configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;     											// disable analog functionality on Port H	pins (PH0-PH3)	
	return;
}


 

void motor(int steps, int motorDirection, int motorRunning, int angleControl) {
    uint32_t delay = 1;
    for (int i = 0; i < steps; i++) {
        if ((GPIO_PORTJ_DATA_R & 0b00000001) == 0) {  			// Check if Button 0 (PJ0) is pressed (active low)
            while ((GPIO_PORTJ_DATA_R & 0b00000001) == 0);  // Wait for button release (debounce)
            return;
        }
				else if ((GPIO_PORTJ_DATA_R & 0b00000010) == 0) {  	// Check if Button 1 (PJ1) is pressed (active low)
            while ((GPIO_PORTJ_DATA_R & 0b00000010) == 0);  // Wait for button release (debounce)
            motorDirection ^= 1;  // Toggle direction
            if (motorDirection == 0) {
                GPIO_PORTN_DATA_R |= 0b00000001;   // Turn on LED1 (PN0) (CW)
            } else {
                GPIO_PORTN_DATA_R &= ~0b00000001;  // Turn off LED1 (PN0) (CCW)
            }
				}
				if ((GPIO_PORTM_DATA_R & 0b00000010) == 0) {  
            while ((GPIO_PORTM_DATA_R & 0b00000010) == 0);  // Wait for button release
            angleControl ^= 1;
            // Toggle LED2 (PF4) based on angle control
            if (angleControl == 0) {
                GPIO_PORTF_DATA_R |= 0b00010000;   // Turn ON LED2 (PF4)
            } else {
                GPIO_PORTF_DATA_R &= ~0b00010000;  // Turn OFF LED2 (PF4)
            }
        }
        if (motorDirection == 0) {  				// CW direction
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
						current_position++;
        } 
        else {  																// CCW direction
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
						current_position--;
					}
						if (angleControl == 0 && (i % 16 == 0)) { // 32 blinks per rotation
								GPIO_PORTF_DATA_R |= 0b00000001; // Turn on LED3 (PF0)
								SysTick_Wait10ms(5);  // Small delay to make blink visible
								GPIO_PORTF_DATA_R &= ~0b00000001; // Turn off LED3 (PF0)
						}
						else if (angleControl == 1 && (i % 64 == 0)) { // 8 blinks per rotation
								GPIO_PORTF_DATA_R |= 0b00000001; // Turn on LED3 (PF0)
								SysTick_Wait10ms(5);  // Small delay
								GPIO_PORTF_DATA_R &= ~0b00000001; // Turn off LED3 (PF0)
        }
    }
 
}
 
 
void returnHome(int angleControl){
	if ((GPIO_PORTM_DATA_R & 0b00000001) == 0){
		while((GPIO_PORTM_DATA_R & 0b00000001) == 0){ //debounce

	int steps_needed = current_position - home_position;
 
  if (steps_needed > 0) {
        motor(steps_needed, 1, 1, angleControl);  // Move backward to home
    } 
	else if (steps_needed < 0) {
				steps_needed *= -1;
        motor(steps_needed, 0, 1, angleControl);  // Move forward to home
    }
 
    current_position = home_position;  // Reset position after homing
	}
	}
}
 
int main(void) {
    PLL_Init();      // Default Set System Clock to 120MHz
    SysTick_Init();  // Initialize SysTick configuration
    PortH_Init();
    PortM_Init();
    PortN_Init();
    PortJ_Init();
		PortF_Init();
		uint8_t motorRunning = 0;  // Motor state: 0 = Stopped, 1 = Running
		uint8_t motorDirection = 0; // Motor direction: 0 = CW, 1 = CCW
		uint8_t angleControl = 0; 	// 0 = 11.25, 1 = 45

    while(1) {
        if ((GPIO_PORTJ_DATA_R & 0b00000001) == 0) {  // Check if Button 0 (PJ0) is pressed (active low)
            while ((GPIO_PORTJ_DATA_R & 0b00000001) == 0);  // Wait for button release
            motorRunning ^= 1;  // Toggle motor state
            if (motorRunning) {
                GPIO_PORTN_DATA_R |= 0b00000011;  							// Turn on LED0 and LED1 (PN1 and PN0)
								GPIO_PORTF_DATA_R |= 0b00010000;
                motor(512, motorDirection, motorRunning, angleControl);              // Start motor (256 steps forward)
								GPIO_PORTN_DATA_R &= ~0b00000010;
								motorRunning ^= 1;
            } 
        }
				returnHome(angleControl);
    }
}