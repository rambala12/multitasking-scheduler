/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
#include "RTCS.h"

#define FLASH_DELAY 2
#define ACC_SENSITIVITY 90
#define TASK_MOTION_SENSOR_FREQ_HZ (50)
// New tasks
#define TASK_MOTION_SENSOR_FSM_FREQ_HZ (50)
#define TASK_I2C_SERVER_FSM_FREQ_HZ (50)

// Global variables
int state = 0; // state variable for Task_Motion_Sensor_FSM

void Init_Debug_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB->PCR[DBG_0] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_1] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_2] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_3] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_4] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_5] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_6] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_7] |= PORT_PCR_MUX(1);          

	PTB->PDDR |= MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);
	PTB->PCOR = MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);

}


void Init_Config_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG1_POS);

	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG2_POS);

	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG3_POS);
}

void Task_Motion_Sensor(void) {
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	uint8_t rf, gf, bf;
	
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);

	read_full_xyz(&acc_X, &acc_Y, &acc_Z);

	rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

	Control_RGB_LEDs(rf, gf, bf);
	Delay(FLASH_DELAY);
	Control_RGB_LEDs(0, 0, 0);							
	Delay(FLASH_DELAY*2);		

	prev_acc_X = acc_X;
	prev_acc_Y = acc_Y;
	prev_acc_Z = acc_Z;
	
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
}

	/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	// Init_Config_Signals();
	

	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Control_RGB_LEDs(0, 0, 0);	
	
	RTCS_Init(SCHED_FREQ_HZ);

	
	if (PTE->PDIR & MASK(3)) {
		RTCS_Add_Task(Task_Motion_Sensor, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
		RTCS_Run_Scheduler();
	} else {
		// Add new tasks to scheduler 
		// if pull down, use FSM
		
		// Start FSM
		RTCS_Add_Task(Task_Motion_Sensor_FSM, 0, TICKS(TASK_MOTION_SENSOR_FSM_FREQ_HZ));
		RTCS_Add_Task(Task_I2C_Server_FSM, 1, TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));
		RTCS_Run_Scheduler();
	}
	
	
}


void Task_Motion_Sensor_FSM() {
	// initialize variables
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	uint8_t rf, gf, bf;
	int i;
	int16_t temp[3];
	
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	switch(state) {
		case START:
			if (g_I2C_Msg.Status == IDLE) {
				state = SEND_REQUEST;
			}
			break;
		case SEND_REQUEST:
			// set values for data structure
			g_I2C_Msg.Dev_adx = MMA_ADDR;
			g_I2C_Msg.Reg_adx = REG_XHI;
			g_I2C_Msg.Data_count = 6;

			// set command to read
			g_I2C_Msg.Command = READ;
			// this will now trigger I2C_Server to read
			break;
		
		case GET_DATA:
			// enters this state if I2C_Server updates state
			// Task_Motion_Sensor code
			//SET_BIT(DEBUG_TASK_MOTION_SENSOR);
			if (g_I2C_Msg.Status == READ_COMPLETE) {
			//read_full_xyz implementation
				for ( i=0; i<3; i++ ) {
					temp[i] = (int16_t) ((g_I2C_Msg.Data[2*i]<<8) | g_I2C_Msg.Data[2*i+1]);
				}

				// Align for 14 bits
				acc_X = temp[0]/4;
				acc_Y = temp[1]/4;
				acc_Z = temp[2]/4;

				rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
				gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
				bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

				
				prev_acc_X = acc_X;
				prev_acc_Y = acc_Y;
				prev_acc_Z = acc_Z;
				
				Control_RGB_LEDs(rf, gf, bf);
				//Delay(FLASH_DELAY);
				g_I2C_Msg.Status = IDLE;
				// retart state machine
				g_I2C_Msg.Command = NONE;
				state = CHANGE_LED;
			}
			break;
			
		case CHANGE_LED:
			Control_RGB_LEDs(0, 0, 0);							
			//Delay(FLASH_DELAY*2);		
			
			// at this point change status to IDLE
			g_I2C_Msg.Status = IDLE;
			// retart state machine
			g_I2C_Msg.Command = NONE;
			state = DELAY_LED;
			RTCS_Set_Task_Period(Task_I2C_Server_FSM, 1, 0);
			break;
		
		case DELAY_LED:
			RTCS_Set_Task_Period(Task_Motion_Sensor_FSM, TICKS(TASK_MOTION_SENSOR_FSM_FREQ_HZ)-1, 0);
			state = START;
			break;
		
		default:
			state = START;
			break;
	}
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
	
}

