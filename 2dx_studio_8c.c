#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h>
 
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define PI 3.14159265 //pi
 
#define NUM_MEASUREMENTS 32 //360 degrees /32 measurments = 11.25 deg / measuremnt
#define STEPS_PER_DEGREES 16 // 512 steps / 32 measurments = 16 steps / measurement
 
float x_coords[NUM_MEASUREMENTS];
float y_coords[NUM_MEASUREMENTS];
uint16_t distances[100000];

 
 
#define MAXRETRIES              5           // number of receive attempts before giving up
 
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                                                                                      // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                                                                              // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};                                                                                                       // ready?
 
    GPIO_PORTB_AFSEL_R |= 0x0C;                                                                                                                 // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;                                                                                                                   // 4) enable open drain on PB3 only
 
    GPIO_PORTB_DEN_R |= 0x0C;                                                                                                                   // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;                                                                                                        // 7) disable analog functionality on PB2,3
 
                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                                                                                                 // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                          // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                                                       // 8) configure for 100 kbps clock
}

void PortM_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x01;        								// make PM0 out 
  GPIO_PORTM_AFSEL_R &= ~0x01;     								// disable alt funct on PM0
  GPIO_PORTM_DEN_R |= 0x01;        								// enable digital I/O on PM0

  GPIO_PORTM_AMSEL_R &= ~0x01;     								// disable analog functionality on PM0		
	
	GPIO_PORTM_DATA_R ^= 0b00000001; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTM_DATA_R ^= 0b00000001;	
	return;
}
 
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0
 
    return;
}
 
void PortH_Init(void){
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                                      // activate clock for Port H
      while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};                 // allow time for clock to stabilize
      GPIO_PORTH_DIR_R |= 0x0F;                                                                       // configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;                                                                  // disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;                                                                     // enable digital I/O on Port H pins (PH0-PH3)
                                                                                                                                                                        // configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;                                                                  // disable analog functionality on Port H pins (PH0-PH3)    
      return;
}
 
void PortN_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};
    GPIO_PORTN_DIR_R = 0b00000011;                                                                                      // PN1 and PN0 as outputs
    GPIO_PORTN_DEN_R = 0b00000011;                                                                                      // D1 is PN1 and D2 is PN0
            return;
 
}
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    GPIO_PORTF_DIR_R = 0b00010001;                                                                                      // PF4 and PF0 as outputs
    GPIO_PORTF_DEN_R = 0b00010001;                                                                                      // D3 is PF4 and D4 is PF0
            return;
}

void PortJ_Init(void){
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                                                  // Activate clock for Port J
      while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};                             // Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R = 0b00000000;                                                                      // PJ1 and PJ0 as input
  GPIO_PORTJ_DEN_R = 0b00000011;                                                                // enable digital I/O on PJ1 and PJ0
      GPIO_PORTJ_PCTL_R &= ~0x000000FF;                                                                           // configure PJ1 and PJ0 as GPIO
      GPIO_PORTJ_AMSEL_R &= ~0b00000011;                                            // Disable analog functionality on PJ1 and PJ0        
      GPIO_PORTJ_PUR_R |= 0b00000011;                                                           // Enable weak pull up resistor on PJ1 and PJ0
      return;
}
//XSHUT     This pin is an active-low shutdown input;
//                            the board pulls it up to VDD to enable the sensor by default.
//                            Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}
 

void motor_step_deg(int clockwise) {
    uint32_t delay = 1;
    for (int i = 0; i < STEPS_PER_DEGREES; i++) { // 512 steps / 128 measurments = 4 steps / measurement
        if (clockwise) { //clockwise movement
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
        } else {  // reverse sequence for counterclockwise movement
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
        }
    }
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********                             MAIN Function                       *****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t    dev = 0x29;             //address of the ToF sensor as an I2C slave peripheral
int status=0;
 
int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum;
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	
      int input = 0; //set input from UART to 0 initially
      int clockwise = 1;  // start in one direction
      int mynumber = 1;
 
      //initialize
      PLL_Init(); 
      SysTick_Init();
      onboardLEDs_Init();
      I2C_Init();
		  PortM_Init();   //for AD3 bus speed confmiration
      UART_Init();
      PortH_Init();  // Stepper motor uses port H
      PortJ_Init();  //for button PJ0
      PortN_Init();   //for LEDs
			
          
 
/* Those basic I2C read functions can be used to check your own I2C functions */
      status = VL53L1X_GetSensorId(dev, &wordData);
 
 
 
      // 1 Wait for device ToF booted
      while(sensorState==0){
            status = VL53L1X_BootState(dev, &sensorState);
            SysTick_Wait10ms(10);
  }
      FlashAllLEDs();
      status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
      Status_Check("SensorInit", status);

 
  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
      
      
while(1){
	  
    if ((GPIO_PORTJ_DATA_R & 0b00000001) == 0) {                  // checks if button press PJ0 occured
            while ((GPIO_PORTJ_DATA_R & 0b00000001) == 0){};  // button debounce
                  
                  FlashLED1(1);
                  
                  for (int layer = 0; layer < 3; layer++) { //for loop from 0-3 for layers
                  
                  sprintf(printf_buffer, "%c\n", 'x');  // format 'c' into the buffer
                  UART_printf(printf_buffer);         // send it via UART

                  
        for (int i = 0; i < NUM_MEASUREMENTS; i++) { //for loop from 0-32
            dataReady = 0;

            while (dataReady == 0) {
                status = VL53L1X_CheckForDataReady(dev, &dataReady);
                VL53L1_WaitMs(dev, 5);
            }

            status = VL53L1X_GetDistance(dev, &distances[i]); //gets distance value
            FlashLED4(1); //flash LED after distance value taken
            VL53L1X_ClearInterrupt(dev);
                                    
            sprintf(printf_buffer, "%d,%d\n", i, distances[i]);  // index first, then distance
            UART_printf(printf_buffer); //send data through UART
            FlashLED3(1); //flash LED after UART transmission

            motor_step_deg(clockwise); //rotate motor 
						
						//toggles port M data to verify bus frequency
						GPIO_PORTM_DATA_R |= 0x01;
            SysTick_Wait10ms(50);
						GPIO_PORTM_DATA_R &= ~0x01;
						
						
        }

				
        VL53L1X_StopRanging(dev);     // Stop ranging after each scan
        VL53L1X_ClearInterrupt(dev);  // Optional cleanup
        VL53L1X_StartRanging(dev);    // Restart for next round
        clockwise = !clockwise;  // flip direction
        UART_printf("SCAN_DONE\r\n"); //signal sent through UART to notify PC that no more distance measurments will be sent
				SysTick_Wait10ms(50);

    }
            
            
}
}
}
