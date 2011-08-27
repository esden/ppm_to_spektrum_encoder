/*********************************************************************************************************
 Title  :   C  file for the rc ppm encoder spektrum satellite like serial output generator (spektrum_serial_out.c)
 Author:    Piotr Esden-Tempski
 E-mail:    piotr at esden dot net
 Homepage:  http://www.esden.net
 Date:      16/Aug/2011
 Compiler:  AVR-GCC
 MCU type:  ATmega326p
 Comments:  This software is FREE. Use it at your own risk.
*********************************************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "spektrum_serial_out.h"

/* Usart settings */
#define UART_BAUD_RATE 38400
//#define UART_BAUD_RATE 115200
#define UART_BAUD_SELECT ((F_CPU/UART_BAUD_RATE/16)-1)

#define SSO_FRAME_SIZE 16
#define SSO_MAX_VAL 0x3FF

#define RC_TIM_DIV 8
#define RC_SERVO_MIN 1000
#define RC_SERVO_MAX 2000
#define RC_SERVO_MIN_TIM_VAL ((((F_CPU/1000) * RC_SERVO_MIN)/1000)/RC_TIM_DIV)
#define RC_SERVO_MAX_TIM_VAL ((((F_CPU/1000) * RC_SERVO_MAX)/1000)/RC_TIM_DIV)
#define RC_SERVO_RANGE_TIM_VAL (RC_SERVO_MAX_TIM_VAL - RC_SERVO_MIN_TIM_VAL)
#define RC_TIM_TO_SSO(VAL) (((VAL - RC_SERVO_MIN_TIM_VAL) * SSO_MAX_VAL) / RC_SERVO_RANGE_TIM_VAL)

#define SSO_CID_THROTTLE   0
#define SSO_CID_ROLL       1
#define SSO_CID_PITCH      2
#define SSO_CID_YAW        3
#define SSO_CID_GEAR       4
#define SSO_CID_FLAP       5
#define SSO_CID_AUX1       5
#define SSO_CID_AUX2       6
#define SSO_CID_AUX3       7
#define SSO_CID_AUX4       8
#define SSO_CID_AUX5       9
#define SSO_CID_AUX6       10
#define SSO_CID_AUX7       11
#define SSO_CID_COUNT      12

unsigned char sso_channel_map[SSO_CID_COUNT] = {
	SSO_CID_FLAP,
	SSO_CID_GEAR,
	SSO_CID_YAW,
	SSO_CID_THROTTLE,
	SSO_CID_PITCH,
	SSO_CID_ROLL,
	SSO_CID_AUX2,
	SSO_CID_AUX3,
	SSO_CID_AUX4,
	SSO_CID_AUX5,
	SSO_CID_AUX6,
	SSO_CID_AUX7
};

unsigned char sso_frame[SSO_FRAME_SIZE];

int frame_counter;

void sso_init(void) {
  int i;

  /* frame loss count */
  sso_frame[0] = 0x00; // Let's pretend we are not loosing any frames
  /* Receiver identifier */
  sso_frame[1] = 0x01; // Let's pretend we are DX6i or similar

  // Set all channel data to 0
  for (i=0; i<7; i++) {
    sso_frame[2+(i*2)] = (sso_channel_map[i] << 2);
    sso_frame[2+(i*2)+1] = 0x00;
  }

  /* Enable USART subsystem */
  UCSR0B=(0<<RXCIE0)|(0<<TXCIE0)|(0<<RXEN0)|(1<<TXEN0);

  /* Setup baud rate */
  UBRR0L=((unsigned char)UART_BAUD_SELECT);

  UDR0 = 0xAA;

  /* configure second pin to be output for serial out */
  DDRD |= (1 << 1); // TX

  frame_counter = 0;

}

void sso_send(unsigned int *channel_data) {
  int i;
  unsigned int temp_val;

  for (i=0; i<6; i++){
    if (channel_data[i] <= RC_SERVO_MIN_TIM_VAL) {
      temp_val = 0;
    } else if (channel_data[i] >= RC_SERVO_MAX_TIM_VAL) {
      temp_val = SSO_MAX_VAL;
    } else {
      temp_val = RC_TIM_TO_SSO(channel_data[i]);
    }
    sso_frame[2+(i*2)] = (sso_channel_map[i] << 2) | (temp_val>>8);
    sso_frame[2+(i*2)+1] = temp_val & 0xFF;
  }

  /* Just enable the serial interrupt the rest will be taken care of */
  UCSR0B|=(1<<TXCIE0);
}

ISR(USART_TX_vect){
  asm("sei");

  UDR0 = sso_frame[frame_counter];

  frame_counter++;

  if (frame_counter >= SSO_FRAME_SIZE) { /* Frame is over, let's disable ourselves */

    /* reset */
    frame_counter = 0;

    /* disable */
    UCSR0B=(0<<RXCIE0)|(0<<TXCIE0)|(0<<RXEN0)|(1<<TXEN0);
  }
}
