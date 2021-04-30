// Project 5 - Sorting System  
// main.c 
// Created: 2020-11-16 8:09:01 PM 
// William Vu Nguyen 
// V00904378 
 
 
// Include libraries 
#include <avr/io.h> 
#include <avr/interrupt.h> 
#include <stdlib.h> 
#include <stdbool.h> 
#include "lcd.h" 
#include "LinkedQueue.h" 
#include "myutils.h" 
 

#define RAMP_STEP (MINIMUM_SPEED - MAXIMUM_SPEED) 


// Sensor pins 
#define HALL_SENSOR_BIT 	 	0x02 
#define OR_SENSOR_BIT 	 	 	0x04 
#define EX_SENSOR_BIT 	 	 	0x08
 
 
// Reflective sensor - Threshold Values 	
#define BLACK_MAX  	 	 	 	1023 
#define BLACK_MIN  	 	 	 	956 
#define WHITE_MAX  	 	 	 	955 
#define WHITE_MIN  	 	 	 	900 
#define STEEL_MAX  	 	 	 	899  	 	 	 	 
#define STEEL_MIN  	 	 	 	256 
#define ALUMINUM_MAX  	 	 	255  	 	 	 	 
#define ALUMINUM_MIN  	 		0
 
 
// DC motor 	 	 	
#define DCMOTOR_FW_ROTATION  	0x07 
#define DCMOTOR_BW_ROTATION  	0x0B 
#define DCMOTOR_BRAKE_HIGH  	0x0F 
#define DCMOTOR_DISABLED  	 	0x00 
#define DCMOTOR_FIXED_SPEED  	0x50
 
 
// Stepper motor 	 	 	 
#define NUMBER_OF_COILS 	 	4 
#define STEP1  	 	 	 	 	0b00110110  
#define STEP2  	 	 	 	 	0b00101110 
#define STEP3  	 	 	 	 	0b00101101 
#define STEP4  	 	 	 	 	0b00110101 
#define STEP_POSITION_BLACK  	0 
#define STEP_POSITION_ALUMINUM 	50 
#define STEP_POSITION_WHITE  	100 
#define STEP_POSITION_STEEL  	150 
#define DEFAULT_STEP_PER_REV 	200 
#define HALF_WAY 	 	 		100 

 
// define enum item type 
typedef enum  
{ 
 	ALUMINUM_ITEM = 0, 
 	STEEL_ITEM, 
 	WHITE_ITEM, 
 	BLACK_ITEM, 
 	INVALID_ITEM 
}item_type_t; 
 
 
// define enum state 
typedef enum  
{ 
 	STOP = 0, 
 	START,  
 	DISABLE 
}DCmotor_state_t; 
 
 
// define enum direction 
typedef enum  
{ 
 	CLOCKWISE_ROTATION = 0, 
 	COUNTER_CLOCKWISE_ROTATION = 1 
}steppermotor_direction_t; 
 
 
// define enum speed 
typedef enum  
{ 
 	MAXIMUM_SPEED = 5, 
 	NORMAL_SPEED = 10, 
 	MINIMUM_SPEED = 14,  
}steppermotor_speed_t; 
 

// Declare global variables 	 	 	 
volatile bool pause_flag = false; 
volatile bool ramp_down_flag = false; 
volatile bool timer_is_running_flag = false; 
volatile bool object_at_exit_flag = false; 
volatile bool object_type_detected_flag = false; 
volatile uint16_t steppermotor_current_position  = 0; 
volatile int16_t  steppermotor_current_coil = 0; 
volatile uint16_t steppermotor_home_position = 0; 
volatile uint16_t step = 0; 
volatile uint16_t number_of_black_items	= 0; 
volatile uint16_t number_of_aluminum_items = 0; 
volatile uint16_t number_of_white_items	= 0; 
volatile uint16_t number_of_steel_items	= 0; 
volatile uint16_t material_type = 0; 
volatile uint16_t lowest_ADC_result	= 0; 
volatile uint16_t new_ADC_result = 0; 
 
 
// Declare user-defined functions void initialize_PWM(); void initialize_ADC(); 
void initialize_external_interrupts(); 
void initialize_steppermotor_homing_position(); 
void control_steppermotor_step(uint16_t total_steps, steppermotor_direction_t rotational_direction); 
void rotate_tray(item_type_t item_type); 
void control_DCmotor_speed(uint16_t DCmotor_speed); 
void control_DCmotor_state(DCmotor_state_t DCmotor_state); 
item_type_t determine_material_type(uint16_t reflectivity); 
uint16_t convert_material_to_step(item_type_t material); 
void count_sorted_item(item_type_t material); 
void display_sorted_item(link **head, link **tail); 
const char* get_item_name(item_type_t item_type); 
void write_lines_to_LCD(const char* line_1_string, const char* line_2_string); 


// Declare user-defined functions 
void ms_timer(uint16_t delay_amount); 
void system_rampdown_delay_ms(uint16_t delay_amount); 
 
 
// Start program execution 
int main(void)  
{ 
    CLKPR = 0x80; // set CLKPCE = clock pre-scaler change enable to 1 
    CLKPR = 0x01; // set the main clock to /2 = 8MHz 
 	 
 	link *head; // Sets up head of queue  	link *tail; // Sets up tail of queue 
 	link *deQueuedLink; // Creating one pointer handle to be reused multiple times 
 	 
 	setup(&head, &tail); // Set up link list 
     
    cli(); // disable global interrupt 
     
    DDRA = 0x0F; // stepper motor driver 
    DDRB = 0xFF; // DC motor driver 
    DDRC = 0xFF; // LEDs display 
    DDRD = 0x00; // external interrupt 
 	DDRE = 0x00; // external interrupt 
    DDRF = 0x00; // ADC control 
      
 	initialize_LCD(LS_BLINK | LS_ULINE); // set parameters to operate the LCD  	initialize_PWM(); // set PWM parameters to control DC motor speed  	initialize_ADC(); // set ADC parameters to begin conversion 
    initialize_external_interrupts(); // set external interrupt pins 	 
 	 	     
    sei(); // enable global interrupt 
 	 
 	initialize_steppermotor_homing_position(); // set stepper motor to locate the home position 
 	 
 	control_DCmotor_state(START); // turn on the DC motor  	control_DCmotor_speed(DCMOTOR_FIXED_SPEED); // set DC motor's speed 
 	 	 
    while(1)  
 	{ 
 	 	// Check if the object has passed the OR sensor and PIND2 is Active High  
 	 	// and if it has been classified    	 	
		// If it has, then: 
 	 	// - Create a new link and add the item type to it 
 	 	// - Clear the optical reflective sensor flag  	 
	 	if((object_type_detected_flag == true) && ((PIND & OR_SENSOR_BIT) == 0x00)) 
	 	{ 
			item_type_t item_type = material_type;
			link *new_link;
			initLink(&new_link); 
			new_link->e.item_type = (char)item_type;
			enqueue(&head, &tail, &new_link);
			object_type_detected_flag = false;
  	 	 	write_lines_to_LCD("Item type", get_item_name(item_type)); 
 	 	} 
 	 	 
 	 	// Check if the EX sensor has detected an object at the exit and PIND3 is Active Low 
 	 	// If it has, then 
 	 	// - Stop the conveyor belt 
 	 	// - Take the item off the link list, save it, and free the memory 
 	 	// - Turn the tray 
 	 	// - Count the number of items for each type 
 	 	// - Clear the exit sensor flag 
 	 	// - Start the conveyor belt again to drop the object to the correct bin  
 	 	// if(object_at_exit_flag == true) 
 	 	if((object_at_exit_flag == true) && ((PIND & EX_SENSOR_BIT) == 0x00)) 
 	 	{ 
 	 	 	control_DCmotor_state(STOP);
  	 	 	item_type_t item_type;
  	 	 	dequeue(&head, &tail, &deQueuedLink); 
 	 	 	item_type = deQueuedLink->e.item_type;
  	 	 	free(deQueuedLink); 
 	 	 	write_lines_to_LCD("Rotating tray", NULL);
  	 	 	write_an_int_value_To_LCD_xy_position(0, 1, item_type, 3);
  	 	 	rotate_tray(item_type);
  	 	 	count_sorted_item(item_type); 
 	 	 	object_at_exit_flag = false; 
 	 	 	control_DCmotor_state(START); 	 	 	 
 	 	} 
 	 	 	 	 
 	 	// Check if the ramp down button has been pressed and PINE 5 is Active Low  
 	 	// If it has, then: 
 	 	// - Check if the link list is empty 
 	 	// - Delay for about 500ms to clear all objects still on system 
 	 	// - Stop the conveyor belt 
 	 	// - Disable global interrupt 
 	 	// - Display the number of items for each type 
 	 	// - Clear the ramp down flag 
 	 	if(ramp_down_flag == true) 
 	 	{ 
 	 	 	if(isEmpty(&head) == 1)  
 	 	 	{ 	 	 	 
 	 	 	 	cli(); 
 	 	 	 	write_lines_to_LCD("Ramping down", NULL);
  	 	 	 	control_DCmotor_state(STOP);
  	 	 	 	ms_timer(10); 
 	 	 	 	control_DCmotor_state(DISABLE);
  	 	 	 	display_sorted_item(&head, &tail); 
 	 	 	 	ramp_down_flag = false; 
 	 	 	 	 
 	 	 	 	while(1) 
 	 	 	 	{ 
 	 	 	 	 	// system has been disabled, stay here until reset 
 	 	 	 	} 
 	 	 	} 
 	 	} 
 	 
		// Check if the pause button has been pressed and PINE 4 is Active Low   	
		// If it has, then: 
	 	// - Stop the conveyor belt 
	 	// - Display the number of items for each type 
	 	// - Wait for flag change 
		// - Start the conveyor belt again to resume normal system operation  	
		if(pause_flag == true) 
 	 	{ 
 	 	 	control_DCmotor_state(STOP); 
 	 	 	// ms_timer(10); 
 	 	 	write_lines_to_LCD("System Paused", NULL); 
 	 	 	ms_timer(20); 
 	 	 	display_sorted_item(&head, &tail); 
 	 	 	 
 	 	 	while(pause_flag != false)  
 	 	 	{ 
 	 	 	 	// wait here until the button is pressed then exit while loop 
 	 	 	} 
 	 	 	 
 	 	 	write_lines_to_LCD("System Resumed", NULL); 
 	 	 	control_DCmotor_state(START); 
 	 	} 
 	} 
} 
 
 
// Initialize the PWM parameters to control the applied voltage to energize the motor’s coils 
void initialize_PWM()  
{ 
 	TCCR0A |= ((1 << WGM01) | (1 << WGM00)); 	// enables:  
												// Timer/Counter Mode: Fast PWM 
												// Maximum (TOP) counter: 0xFF 
 	TCCR0A |= (1 << COM0A1); // clears OC0A on Compare Output, Fast PWM Mode 
 	TCCR0B |= ((1 << CS01) | (1 << CS00)); // sets clock source to 1/64 } 
 
 
// Initialize the ADC parameters to read the material’s reflectivity 
void initialize_ADC()  
{ 
 	ADCSRA |= (1 << ADEN); // enables the ADC 
 	ADCSRA |= ((1 << ADPS2) | (1 << ADPS0)); // set ADC prescaler division factor to 1/32 
 	ADCSRA |= (1 << ADIE); // writing this bit to 1 enables interrupt 
 	ADMUX  |= ((1 << REFS0) | (1 << MUX0)); // selects voltage reference and ADC1 on pin PF1  	  	
	DIDR0  |= 0x02; // disable digital input buffer for analog use 	 	 
} 
 
 
// Initialize and set up external interrupt to control 
// - The sensor reading 
// - The state of the push button  	 	 	 	 	 	 	 	 	 	 
void initialize_external_interrupts()  
{ 
 	// Optical reflective sensor 
 	EIMSK |= (1 << INT2); 
 	EICRA |= ((1 << ISC21) | (1 << ISC20)); // Rising edge interrupt 
 	 
 	// Exit optical sensor 
 	EIMSK |= (1 << INT3); 
 	EICRA |= (1 << ISC31); 	 	 	// Falling edge interrupt 
 
	// Pause/unpause left push-button 
	EIMSK |= (1 << INT4); 
	EICRB |= (1 << ISC41); 	 	 	// Falling edge interrupt 
 
	// Ramp down right push-button 
	EIMSK |= (1 << INT5); 
	EICRB |= (1 << ISC51); 	 	 	// Falling edge interrupt 
} 
 
 
// Initialize and setup stepper motor to find and return to the homing position 
void initialize_steppermotor_homing_position()  
{ 
 	write_lines_to_LCD("Searching for", "homing position"); 
 	 
 	// check if the HE sensor has detected the metallic part's magnetic field  	
	// if it has not, continue to rotate stepper motor until the detection occurs  	
	while((PIND & HALL_SENSOR_BIT) == HALL_SENSOR_BIT) 
 	{ 
 	 	control_steppermotor_step(1, CLOCKWISE_ROTATION); 
 	} 	 
 	 
	steppermotor_current_position = 0;
    write_lines_to_LCD("Found", "homing position"); 
} 
 
 
// Control the number of steps of the stepper motor's rotation to return to the homing position and  
// catch items 
void control_steppermotor_step(uint16_t total_steps, steppermotor_direction_t rotational_direction)  
{ 
 	// stepper motor's mode of operation: dual-phase full step 
 	uint16_t steppermotor_rotation_LUT[NUMBER_OF_COILS] = {STEP1, STEP2, STEP3, STEP4};  	
	uint16_t steps_left = total_steps;  
 	PORTA = steppermotor_rotation_LUT[steppermotor_current_coil];  	
	uint16_t step_delay_ms = MINIMUM_SPEED; 
 	 
 	while(steps_left > 0)  
 	{ 
 	 	// decide the stepper motor's rotational direction  	 	
		// for clockwise direction, step from step 1 to step 4  	 	
		if(rotational_direction == CLOCKWISE_ROTATION)  
 	 	{ 
 	 	 	steppermotor_current_coil++; 
 	 	 	if(steppermotor_current_coil > (NUMBER_OF_COILS - 1))  
 	 	 	{ 
 	 	 	 	steppermotor_current_coil = 0; 
 	 	 	} 
 	 	} 
 	 	// for counter-clockwise direction, step from step 4 to step 1  	 	
		else  
 	 	{ 
 	 	 	steppermotor_current_coil--; 
 	 	 	if(steppermotor_current_coil < 0)  
 	 	 	{ 
 	 	 	 	steppermotor_current_coil = NUMBER_OF_COILS - 1; 
 	 	 	} 
 	 	} 
 	 	 	 	 
 	 	// execute the stepping for each coil following the LUT  	
		PORTA = steppermotor_rotation_LUT[steppermotor_current_coil];  	
		ms_timer(step_delay_ms); 
 	 
	 	// Implement trapezoidal velocity profile 
	 	// solve by comparing the default 15 ramp down/up steps 
	 	// Ramp up speed 
	 	if((step_delay_ms > MAXIMUM_SPEED) && ((total_steps - steps_left) < RAMP_STEP))  
	 	{ 
 	 	 	step_delay_ms--; // decrease the delay to speed up the stepper motor 
 	 	} 
 	 	 
 	 	// Ramp down speed 
 	 	if((step_delay_ms < MINIMUM_SPEED) && (steps_left < RAMP_STEP))  
 	 	{ 
 	 	 	step_delay_ms++; // increase the delay to slow down the stepper motor 
 	 	} 
 	 	 
 	 	steps_left--; 
 	} 
} 
 
 
// Command the tray to rotate once the item type has been identified and converted to number of steps 
void rotate_tray(item_type_t item_type)  
{ 
 	int16_t step_to_take; 
 	steppermotor_direction_t direction = CLOCKWISE_ROTATION; 
 	uint16_t steppermotor_new_position = convert_material_to_step(item_type); 
 	 
 	// calculate the number of steps to take to reach the new position   	
	step_to_take = (steppermotor_new_position - steppermotor_current_position); 
 	 
 	// decide the stepper motor's rotational direction 
 	// solve to find the least number of steps to take 
 	// solve to find direction to take 
 	// once the tray is at the new location, that is also the current position  	
	if(step_to_take < 0)  
 	{ 
 	 	step_to_take = -step_to_take; 
 	 	 
 	 	if(direction == CLOCKWISE_ROTATION)  
 	 	{ 
 	 	 	direction = COUNTER_CLOCKWISE_ROTATION; 
 	 	}  
 	 	else 
 	 	{ 
 	 	 	direction = CLOCKWISE_ROTATION; 
 	 	} 
 	} 
 	 
 	// consider the situation when the number of steps is more than 100 steps 
 	// if it is, find the difference and rotate the opposite direction to get to the new position 
 	// in the shortest path 
 	if(step_to_take > HALF_WAY)  
 	{ 	 
 	 	step_to_take -= HALF_WAY; 
 	 	 
 	 	if(direction == CLOCKWISE_ROTATION) 
 	 	{ 
 	 	 	direction = COUNTER_CLOCKWISE_ROTATION;  
 	 	} 
	 	else 
	 	{ 
	 	 	direction = CLOCKWISE_ROTATION; 
	 	} 
	}  
	write_lines_to_LCD(NULL, NULL); 
	write_an_int_value_To_LCD_xy_position(0, 0, steppermotor_new_position, 5); write_an_int_value_To_LCD_xy_position(0, 1, step_to_take, 5);  control_steppermotor_step(step_to_take, direction); 
 	steppermotor_current_position = steppermotor_new_position; 
} 
 
// Control the speed of DC motor 
void control_DCmotor_speed(uint16_t DCmotor_speed)  
{ 
 	OCR0A = DCmotor_speed; 
} 
 
 
// Control the state of DC motor to turn it ON or OFF 
void control_DCmotor_state(DCmotor_state_t DCmotor_state)  
{ 
 	switch(DCmotor_state)  
 	{ 
 	 	case START: 
 	 	 	PORTB = DCMOTOR_FW_ROTATION; 
 	 	 	break;  	 	case STOP: 
 	 	 	PORTB = DCMOTOR_BRAKE_HIGH; // brake DC motor by setting all bits to 1s  
 	 	 	break;  	 	case DISABLE: 
 	 	 	PORTB = DCMOTOR_DISABLED; // disable DC motor by setting bits ENA and ENB to 0 
 	 	 	break;  	 	default: 
 	 	 	break; 
 	} 
} 
 
 
// Determine material type using reflectivity values 
item_type_t determine_material_type(uint16_t reflectivity)  
{ 	 
 	if(reflectivity >= BLACK_MIN)  
 	{ 
 	 	return BLACK_ITEM; 
 	} 
 	else if(reflectivity >= WHITE_MIN)  
 	{ 
 	 	return WHITE_ITEM; 
 	} 
 	else if(reflectivity >= STEEL_MIN)  
 	{ 
 	 	return STEEL_ITEM; 
 	} 
 	else if(reflectivity >= ALUMINUM_MIN) 
 	{ 
 	 	return ALUMINUM_ITEM; 
 	} 
 	else  
 	{ 
 	 	return INVALID_ITEM; // return error condition! 
	}
} 
 
 
// Determine material type using reflectivity values 
uint16_t convert_material_to_step(item_type_t material)  
{ 
 	uint16_t step; 
 	 
 	switch(material)  
 	{ 
 	 	case ALUMINUM_ITEM: 
 	 	 	step = STEP_POSITION_ALUMINUM;  // 1 
 	 	 	break;  	 	
		
		case WHITE_ITEM:
			step = STEP_POSITION_WHITE;  	// 2 
 	 	 	break;

		case STEEL_ITEM:
			step = STEP_POSITION_STEEL;  	// 3 
 	 	 	break;

  	 	case BLACK_ITEM:
			step = STEP_POSITION_BLACK;  	// 4 
 	 	 	break;

		default: 
 	 	 	step = STEP_POSITION_BLACK;  	// in case of error, go home 
 	 	 	break; 
 	} 
 	 
 	return step; 
} 
 
 
// Count number of objects for each type of four materials in each bin 
// by creating variables and increment by one each time an object is dropped to the correct bin 
void count_sorted_item(item_type_t material)  
{ 
 	switch(material)  
 	{ 
 	 	case ALUMINUM_ITEM: 
 	 	 	number_of_aluminum_items++; 
 	 	 	break;  	 	
		
		case WHITE_ITEM:
			number_of_white_items++; 
 	 	 	break; 

		case STEEL_ITEM:
			number_of_steel_items++; 
 	 	 	break;

		case BLACK_ITEM:
			number_of_black_items++;  	 	 	
			break;

		default: 
 	 	 	break; 
 	} 
} 
 
 
// Display the item's name on the LCD when it passes the sensor const char* get_item_name(item_type_t item_type) 
{ 
 	switch(item_type) 
 	{ 
 	 	case ALUMINUM_ITEM:  	 	 	
			return "ALUMINUM"; 
 	 	 	break;

		case WHITE_ITEM:
			return "WHITE"; 
 	 	 	break; 
	 	
		case STEEL_ITEM: 
	 	 	return "STEEL"; 
 	 	 	break;

		case BLACK_ITEM:
			return "BLACK"; 
 	 	 	break;

		default: 
 	 	 	return "INVALID ITEM"; 
 	 	 	break; 
 	} 
} 
 
 
// Display the number of sorted item for each type of four materials on the LCD screen 
void display_sorted_item(link **head, link **tail)  
{ 	 
 	write_lines_to_LCD("AL WH ST BL #OB", NULL); 
 	write_an_int_value_To_LCD_xy_position(0, 1, number_of_aluminum_items, 2); 
 	write_an_int_value_To_LCD_xy_position(3, 1, number_of_white_items, 2); 
 	write_an_int_value_To_LCD_xy_position(6, 1, number_of_steel_items, 2); 
 	write_an_int_value_To_LCD_xy_position(9, 1, number_of_black_items, 2); 
  	write_an_int_value_To_LCD_xy_position(13, 1, size(head, tail), 2); 
}
 
 
// This function is reused in several places to display two strings on two lines void 
write_lines_to_LCD(const char* line_1_string, const char* line_2_string) 
{ 
 	clear_LCD_homescreen(); // clear previous display on LCD home screen  	ms_timer(10); 
 	return_to_LCD_homescreen(); // return LCD cursor to the position (0, 0)   	ms_timer(10); 
 	 
 	// check if first line is not written 
 	// if it is, write a string to the first line on LCD  
 	if(line_1_string != NULL) 
 	{ 
 	 	write_a_string_To_LCD_xy_position(0, 0, line_1_string); 
 	} 
 	 
 	// check if second line is not written 
 	// if it is, write a string to the second line on LCD 
 	if(line_2_string != NULL) 
 	{ 
 	 	write_a_string_To_LCD_xy_position(0, 1, line_2_string); 
 	} 
} 
 
 
// Enable ADC Interrupt Service Routine to obtain new ADC conversion results 
// ADC conversion results represent material’s reflectivity 
ISR(ADC_vect)  
{ 	 
 	new_ADC_result = ADC; 
 	 
 	// decide the object type by finding the lowest ADC result  	
	if(new_ADC_result < lowest_ADC_result)  
 	{ 
 	 	lowest_ADC_result = new_ADC_result; 
	}  
 	
	// check if the object is still passing the OR sensor with PIND2 is still Active HIGH   
 	// If so, continue another conversion  	
	if((PIND & OR_SENSOR_BIT) == OR_SENSOR_BIT) 
 	{ 
 	 	ADCSRA |= (1 << ADSC); // start a new ADC conversion 
 	} 
	
 	// once the lowest ADC result of an object has been obtained, identify its type by  
 	// calling a function and passing this lowest ADC result to it  	
	else  
 	{ 
 	 	material_type = determine_material_type(lowest_ADC_result); 
 	 	object_type_detected_flag = true; // process object identification when it is detected 
 	} 
} 
 
 
// Enable ISR for OR sensor to detect the edge of an object 
ISR(INT2_vect)  
{ 
 	lowest_ADC_result = 0x3FF; // reset ADC result to the highest value 1023  	
	ADCSRA |= (1 << ADSC); // start a new ADC conversion 
} 
 
 
// Enable ISR for EX sensor to detect the edge of an object 
// then set a flag to stop the conveyor belt and start sorting 
ISR(INT3_vect)  
{ 
 	object_at_exit_flag = true;  
} 
 
 
// Enable ISR for Pause button pressed - Left push-button ACTIVE LOW 
// set a flag to either STOP or START the conveyor belt 
ISR(INT4_vect)  
{ 
 	ms_timer(DEBOUNCE_PERIOD); 
 	while((PINE & PAUSE_BIT) == 0x00); // check for PINE4 button debouncing  	
	ms_timer(DEBOUNCE_PERIOD);  	
	pause_flag = !pause_flag; 
} 
 
 
// Enable ISR for Ramp Down button pressed - Right push-button ACTIVE LOW 
// set a timer to count for 9 seconds before initiating the ramp down sequence 
ISR(INT5_vect)  
{ 
 	ms_timer(DEBOUNCE_PERIOD); 
 	while((PINE & RAMPDOWN_BIT) == 0x00); // check for PINE5 button debouncing  	
	ms_timer(DEBOUNCE_PERIOD);  	
	system_rampdown_delay_ms(0xFFFF); 
} 
 
 
// Enable ISR for Timer 3 to respond to the timer countdown 
// once the 9 seconds has been reached, set a flag to initiate the ramp down sequence 
ISR(TIMER3_COMPA_vect) 
{ 
 	ramp_down_flag = true; 
}
 
// Enable BAD ISR to warn viewers that interrupt failed to trigger correctly 
ISR(BADISR_vect)  
{ 
 	write_lines_to_LCD("ERROR: BAD ISR!", NULL); 	 
 	 
 	while(1)  
 	{ 
 	 	// stay here, there is an error 
 	} 
} 
 
 
// timer delay in ms without interrupt  
void ms_timer(uint16_t delay_amount) 
{ 
 	TCCR1B |= (1 << CS11); // Set clock pre-scaler for timer 1B to /8 = 2MHz 
 	TCCR1B |= (1 << WGM12); // set waveform generation mode to counting only 
 	OCR1A = 0x03E8; // set output compare register 1A at top value of 0x03E8 = 1000 
 	TCNT1 = 0x0000; // set Timer/Counter Register 1 to count from 0 
 	TIFR1 |= (1 << OCF1A); // set to clear the interrupt flag at bit OCF1A and begin new counting 
 	 
 	for(int i = 0; i < delay_amount; i++)  
 	{ 
 	 	while ((TIFR1 & 0x02) != 0x02); // Check if the interrupt flag bit is on?  	 	TIFR1 |= (1 << (OCF1A));  	// if the interrupt flag is on, clear it 
 	} 
} 
 
 
// timer delay in ms with interrupt enable, used for ramp down 
void system_rampdown_delay_ms(uint16_t delay_amount) 
{  	
	TCCR3B |= (1 << WGM32); // set timer 3 to count mode 
 	TCCR3B |= ((1 << CS32) | (1 << CS30)); // divide clock IO by 1024 
 	OCR3A = delay_amount; // set timer 3 to stop counting at 600   
 	TCNT3 = 0x0000; // set timer 3 to count from 0 
 	TIMSK3 |= 0x02; // set interrupt flag in the Status Register (global)  	TIFR3  |= (1 << OCF3A); 
} 
