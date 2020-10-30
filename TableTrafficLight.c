// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on TM4C123G microcontroller
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Albert Zhong
// Oct 29, 2020

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

//FSM States
#define GO_WEST 						0
#define WAIT_WEST 					1
#define GO_SOUTH 						2
#define WAIT_SOUTH 					3
#define WALK 								4
#define WALK_FLASH_ON_1			5
#define WALK_FLASH_OFF_1		6
#define WALK_FLASH_ON_2			7
#define WALK_FLASH_OFF_2		8
#define DONT_WALK						9

#define NUM_STATES					10	//the total number of states in the FSM

//FSM Declaration
struct state_def {
	unsigned long PBOut;	//PB5-0 output for the traffic lights
	unsigned long PFOut;	//output for PF3, 1 for the walk lights
	unsigned long time;	//wait time for each state
	unsigned long Next[8];		//next state given 3 bit input
};
typedef const struct state_def state;


state	FSM[NUM_STATES] = {
	{0x0C, 0x2, 50, {GO_WEST, GO_WEST, WAIT_WEST, GO_WEST, WAIT_WEST, GO_WEST, WAIT_WEST, WAIT_WEST}},
	{0x14, 0x2, 25, {WALK, WALK, GO_SOUTH, GO_SOUTH, WALK, WALK, WALK, WALK}},
	{0x21, 0x2, 50, {GO_SOUTH, WAIT_SOUTH, GO_SOUTH, GO_SOUTH, WAIT_SOUTH, WAIT_SOUTH, GO_SOUTH, WAIT_SOUTH}},
	{0x22, 0x2, 25, {GO_WEST, GO_WEST, GO_WEST, GO_WEST, WALK, GO_WEST, WALK, GO_WEST}},
	{0x24, 0x8, 50, {WALK, WALK_FLASH_ON_1, WALK_FLASH_ON_1, WALK_FLASH_ON_1, WALK, WALK, WALK, WALK_FLASH_ON_1}},
	{0x24, 0x2, 8, {WALK_FLASH_OFF_1, WALK_FLASH_OFF_1, WALK_FLASH_OFF_1, WALK_FLASH_OFF_1, WALK_FLASH_OFF_1, WALK_FLASH_OFF_1, WALK_FLASH_OFF_1, WALK_FLASH_OFF_1}},
	{0x24, 0x0, 8, {WALK_FLASH_ON_2, WALK_FLASH_ON_2, WALK_FLASH_ON_2, WALK_FLASH_ON_2, WALK_FLASH_ON_2, WALK_FLASH_ON_2, WALK_FLASH_ON_2, WALK_FLASH_ON_2}},
	{0x24, 0x2, 8, {WALK_FLASH_OFF_2, WALK_FLASH_OFF_2, WALK_FLASH_OFF_2, WALK_FLASH_OFF_2, WALK_FLASH_OFF_2, WALK_FLASH_OFF_2, WALK_FLASH_OFF_2, WALK_FLASH_OFF_2}},
	{0x24, 0x0, 8, {DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK}},
	{0x24, 0x2, 8, {GO_SOUTH, GO_WEST, GO_SOUTH, GO_SOUTH, GO_SOUTH, GO_WEST, GO_SOUTH, GO_SOUTH}}	
};

unsigned long State;	//index to the current state
unsigned long Input;	//input from Port E, used to determine the next state


// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

void Port_Init(void);				// Enable Port B, E, and F

void SysTick_Init(void);											//initialize SysTick
void SysTick_Wait(unsigned long n);						//SysTick Wait
void SysTick_Wait10ms(unsigned long delay);		//SysTick Wait 10ms

// ***** 3. Subroutines Section *****

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz 
  EnableInterrupts();
	Port_Init();
	SysTick_Init();
	
	
	State = GO_WEST;	//set initial state
  while(1){
		Input = GPIO_PORTE_DATA_R;
    GPIO_PORTB_DATA_R = FSM[State].PBOut;
		GPIO_PORTF_DATA_R = FSM[State].PFOut;
		SysTick_Wait10ms(FSM[State].time);
		State = FSM[State].Next[Input];
  }
}


void Port_Init(void) {
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000032;     // 1) Initialize clock for Ports B, E, and F
  delay = SYSCTL_RCGC2_R;           // delay to give time for clock to initialize  
	
	
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF. Port B, and E do not need to be unlocked 
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3, 1	(Walk Lights)    
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0A;          // 5) PF3, 1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function  
  GPIO_PORTF_DEN_R = 0x0A;          // 7) enable digital pins PF4-PF0   

	GPIO_PORTB_LOCK_R  = 0x4C4F434B;
	GPIO_PORTB_CR_R = 0x3F;						//allow changed to PB1-6
	GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
	GPIO_PORTB_DIR_R = 0x3F;          // 5) PB1-6 set as output
	GPIO_PORTB_AFSEL_R = 0x00;
	GPIO_PORTB_DEN_R = 0x3F;					//Enable digital pins for PB5-0
	
	GPIO_PORTE_LOCK_R  = 0x4C4F434B;
	GPIO_PORTE_CR_R = 0x07;						//allow changed to PE2-0
	GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTE_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
	GPIO_PORTE_DIR_R = 0x00;          // 5) PE2-0 are inputs
	GPIO_PORTE_AFSEL_R = 0x00;
	//GPIO_PORTE_PUR_R = 0x07; 					// will use external pullup resistors
	GPIO_PORTE_DEN_R = 0x07;
}


void SysTick_Init(void) {
	
	NVIC_ST_CTRL_R = 0; //Clear the ENABLE bit to turn off SysTick during initialization
	NVIC_ST_RELOAD_R = 0xF42400; //We set the RELOAD register
	NVIC_ST_CURRENT_R = 0; //We write the NVIC_ST_CURRENT_R value to clear the counter.
	NVIC_ST_CTRL_R = 0x5; // We write the desired mode to the control register
}

void SysTick_Wait(unsigned long n) {
	NVIC_ST_RELOAD_R = n-1;
	NVIC_ST_CURRENT_R = 0;
	while((NVIC_ST_CTRL_R & 0x10000) == 0)	//if COUNT flag is OFF, do nothing
		;
}

void SysTick_Wait10ms(unsigned long delay) {
	unsigned long i;
	for(i = 0; i < delay; i++) {
		SysTick_Wait(800000); //10ms. Apparently system runs at 80 MHz
	}
}
