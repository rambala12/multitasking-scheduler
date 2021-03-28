#include	 <MKL25Z4.H>
#include	 "i2c.h"
#include 	"gpio_defs.h"
#include 	"RTCS.h"

int lock_detect = 0;
int i2c_lock	= 0;
extern int state;

// #define ENABLE_LOCK_DETECT // don't include lock detect, simplifying FSM creation

volatile I2C_MESSAGE_T g_I2C_Msg = {0, NONE, IDLE, 0,0};

//init i2c0
void i2c_init( void )
{
 //clock i2c peripheral and port E
	SIM->SCGC4		 |= SIM_SCGC4_I2C0_MASK;
	SIM->SCGC5		 |= SIM_SCGC5_PORTE_MASK;

	//set pins to I2C function
	PORTE->PCR[ 24 ] |= PORT_PCR_MUX( 5 );
	PORTE->PCR[ 25 ] |= PORT_PCR_MUX( 5 );
	
	// configure pull up resistor
	PORTE->PCR[3] |= PORT_PCR_PE_MASK;
	PORTE->PCR[3] |= PORT_PCR_PS_MASK;
	PORTE->PCR[3] |= PORT_PCR_MUX(1);
	PTE->PDDR &= ~MASK(3);
	

	//set baud rate
	//baud = bus freq/(scl_div+mul)
	I2C0->F				= ( I2C_F_ICR( 0x11 ) | I2C_F_MULT( 0 ) );

	//enable i2c and set to master mode
	I2C0->C1		 |= ( I2C_C1_IICEN_MASK );

	// Select high drive mode
	I2C0->C2		 |= ( I2C_C2_HDRS_MASK );
}

void i2c_busy( void )
{
 // Start Signal
	lock_detect	= 0;
	I2C0->C1	&= ~I2C_C1_IICEN_MASK;
	I2C_TRAN;
	
	SET_BIT(DEBUG_MSG_ON_BUS); // set debug bit before sending start
	I2C_M_START;
	I2C0->C1 |= I2C_C1_IICEN_MASK;
	// Write to clear line
	I2C0->C1 |= I2C_C1_MST_MASK;			// set MASTER mode								
	I2C0->C1 |= I2C_C1_TX_MASK;				// set transmit (TX) mode							
	I2C0->D	 = 0xFF;
	
	while( ( I2C0->S & I2C_S_IICIF_MASK ) == 0U ) {// await interrupt									
	}		

	I2C0->S		|= I2C_S_IICIF_MASK;		// Clear interrupt bit							
	I2C0->S		|= I2C_S_ARBL_MASK;			// Clear arbitration error flag						

	
																		// Send start										
	I2C0->C1	&= ~I2C_C1_IICEN_MASK;
	I2C0->C1	|= I2C_C1_TX_MASK;			// Set transmit (TX) mode							
	I2C0->C1	|= I2C_C1_MST_MASK;			// START signal generated							

	I2C0->C1	|= I2C_C1_IICEN_MASK;		// Wait until start is sent						

																		// Send stop										
	I2C0->C1	&= ~I2C_C1_IICEN_MASK;
	I2C0->C1	|= I2C_C1_MST_MASK;
	I2C0->C1	&= ~I2C_C1_MST_MASK;		// Set SLAVE mode									
	I2C0->C1	&= ~I2C_C1_TX_MASK;			// Set Rx											
	I2C0->C1	|= I2C_C1_IICEN_MASK;
	
	CLEAR_BIT(DEBUG_MSG_ON_BUS); // clear bit after stop condition ... double check this one
																		
	I2C0->S		|= I2C_S_IICIF_MASK; 		// Clear arbitration error & interrupt flag			
	I2C0->S		|= I2C_S_ARBL_MASK;
	lock_detect	= 0;
	i2c_lock	 = 1;
}

void i2c_wait( void )
{
	lock_detect = 0;
	SET_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit
	
	while( ( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) & ( lock_detect < LOCK_DETECT_THRESHOLD )) {
		lock_detect++;
	}
	
	
	CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);
#ifdef ENABLE_LOCK_DETECT
	if( lock_detect >= LOCK_DETECT_THRESHOLD )
		i2c_busy( );
#endif
	I2C0->S |= I2C_S_IICIF_MASK;
}

int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	SET_BIT(DEBUG_I2C_CODE);
	I2C_TRAN;													//	set to transmit mode	

	SET_BIT(DEBUG_MSG_ON_BUS); // set debug bit before starting	
	
	I2C_M_START;											//	send start

	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();													//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();													//	wait for completion								

	I2C_M_RSTART;											//	repeated start									
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)							
	i2c_wait();													//	wait for completion								

	I2C_REC;													//	set to receive mode								
	while (num_bytes_read < data_count) {
		is_last_read = (num_bytes_read == data_count-1)? 1: 0;
		if (is_last_read){
			NACK;													// tell HW to send NACK after read							
		} else {
			ACK;													// tell HW to send ACK after read								
		}

		dummy = I2C0->D;								//	dummy read										
		i2c_wait();												//	wait for completion								

		if (is_last_read){
			I2C_M_STOP;										//	send stop	
			CLEAR_BIT(DEBUG_MSG_ON_BUS); // clear debug bit if send stop
		}
		data[num_bytes_read++] = I2C0->D; //	read data										
	}
	CLEAR_BIT(DEBUG_I2C_CODE);
	return 1;
}

int i2c_write_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
	uint8_t num_bytes_written=0;
	
	SET_BIT(DEBUG_I2C_CODE);

	I2C_TRAN;													//	set to transmit mode
	
	SET_BIT(DEBUG_MSG_ON_BUS); // set debug bit before starting	
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();													//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();													//	wait for completion								

	while (num_bytes_written < data_count) {
		I2C0->D = data[num_bytes_written++]; //	write data										
		i2c_wait();												//	wait for completion								
	}
	I2C_M_STOP;												//		send stop	
	
	CLEAR_BIT(DEBUG_MSG_ON_BUS); // clear debug bit after send stop
	CLEAR_BIT(DEBUG_I2C_CODE);

	return 1;
}

// Definition of I2C Server FSM
void Task_I2C_Server_FSM() {
	// initialize variables
	uint8_t dummy;
	static uint8_t num_bytes_read, is_last_read;
	SET_BIT(DEBUG_I2C_CODE);

	switch (g_I2C_Msg.Status) {

		case IDLE:
			// unless the command is READ, return immediately in IDLE state
			if (g_I2C_Msg.Command == READ) {
				g_I2C_Msg.Status = READING;
			} else {
				return;
			}
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case READING:
			//SET_BIT(DEBUG_I2C_CODE);

			I2C_TRAN;													//	set to transmit mode	

			SET_BIT(DEBUG_MSG_ON_BUS); // set debug bit before starting	
			
			I2C_M_START;											//	send start

			I2C0->D = g_I2C_Msg.Dev_adx;								//	send dev address (write)	
			// update status state
			g_I2C_Msg.Status = WAIT_1;
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case WAIT_1:		
			SET_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit				
			if ( ( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 )) {
			} else {
				I2C0->S |= I2C_S_IICIF_MASK;
				// update status state
				g_I2C_Msg.Status = SEND_REGISTER;
			}
			CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case SEND_REGISTER:
			I2C0->D = g_I2C_Msg.Reg_adx;								//	send register address		
			// update status state:
			g_I2C_Msg.Status = WAIT_2;
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case WAIT_2:		
			SET_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit				
			if ( ( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 )) {
			} else {
				I2C0->S |= I2C_S_IICIF_MASK;
				// update status state
				g_I2C_Msg.Status = REPEAT_START;
			}
			CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case REPEAT_START:
			I2C_M_RSTART;											//	repeated start									
			I2C0->D = g_I2C_Msg.Dev_adx | 0x01 ;				//	send dev address (read)	
			// update status state:
			g_I2C_Msg.Status = WAIT_3;
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case WAIT_3:	
			SET_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit				
			if ( ( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 )) {
			} else {
				I2C0->S |= I2C_S_IICIF_MASK;
				// update status state
				g_I2C_Msg.Status = RECEIVE;
			}
			CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;	

		case RECEIVE:
			I2C_REC;													//	set to receive mode		
			g_I2C_Msg.Status = CHECK_READ;
			num_bytes_read = 0;
			is_last_read = 0;
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case CHECK_READ:		
			if (num_bytes_read < g_I2C_Msg.Data_count) {
				is_last_read = (num_bytes_read == g_I2C_Msg.Data_count-1)? 1: 0;
				if (is_last_read){
					NACK;													// tell HW to send NACK after read							
				} else {
					ACK;													// tell HW to send ACK after read								
				}
				dummy = I2C0->D;								//	dummy read		
				g_I2C_Msg.Status = WAIT_4;
			}	
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;	
			
		case WAIT_4:
			SET_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit				
			if ( ( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 )) {
			} else {
				I2C0->S |= I2C_S_IICIF_MASK;
				// update status state
				g_I2C_Msg.Status = LAST_READ;
			}
			CLEAR_BIT(DEBUG_I2C_BUSY_WAIT); // I2C Busy wait loop debug bit
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case LAST_READ:
			if (is_last_read) {
					I2C_M_STOP;										//	send stop	
					CLEAR_BIT(DEBUG_MSG_ON_BUS); // clear debug bit if send stop
				}
				g_I2C_Msg.Data[num_bytes_read++] = I2C0->D; //	read data	
				g_I2C_Msg.Status = CHECK_READ_2;	
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case CHECK_READ_2:
			if (num_bytes_read < g_I2C_Msg.Data_count) {
				is_last_read = (num_bytes_read == g_I2C_Msg.Data_count-1)? 1: 0;
				if (is_last_read){
					NACK;													// tell HW to send NACK after read							
				} else {
					ACK;													// tell HW to send ACK after read								
				}

				dummy = I2C0->D;								//	dummy read	
				g_I2C_Msg.Status = CHECK_READ;
			}	else {
				g_I2C_Msg.Status = READ_COMPLETE;
			}
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		case READ_COMPLETE:
			state = GET_DATA;
			RTCS_Release_Task(Task_I2C_Server_FSM);
			break;

		default: 
			break;
	}
	CLEAR_BIT(DEBUG_I2C_CODE);
}
