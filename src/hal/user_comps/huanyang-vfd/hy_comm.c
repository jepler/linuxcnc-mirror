/*      hy_comm.c
 
   By S.Alford
 
   These library of functions are designed to enable a program send and
   receive data from a Huanyang VFD. This device does not use a standard
   Modbus function code or data structure.
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.
 
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                    
 
   This code has its origins with libmodbus.
 
*/  
  
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>
#include "rtapi.h"
#include <rtapi_string.h>

#include "hy_comm.h"
 

 /* Table of CRC values for high-order byte */
static unsigned char table_crc_hi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static unsigned char table_crc_lo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
  0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
  0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
  0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
  0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
  0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
  0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
  0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
  0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
  0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
  0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
  0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
  0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
  0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
  0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
  0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};  

/************************************************************************* 
 
	Function to treat comms errors
	
**************************************************************************/
static void error_treat(hycomm_param_t *hc_param, int code, const char *string)
{
	if (!hc_param->print_errors)
		return;
	
    printf("\nERROR %s (%d)\n", string, code);
	
	/* flush the port */
    tcflush(hc_param->fd, TCIOFLUSH);

}


/************************************************************************* 
 
	Fast CRC function
	
**************************************************************************/

static unsigned short crc16(unsigned char *buffer,
			    unsigned short buffer_length)
{
  unsigned char crc_hi = 0xFF; /* high CRC byte initialized */
  unsigned char crc_lo = 0xFF; /* low CRC byte initialized */
  unsigned int i; /* will index into CRC lookup */

  /* pass through message buffer */
  while (buffer_length--) {
    i = crc_hi ^ *buffer++; /* calculate the CRC  */
    crc_hi = crc_lo ^ table_crc_hi[i];
    crc_lo = table_crc_lo[i];
  }

  return (crc_hi << 8 | crc_lo);
}

/************************************************************************* 
 
	Check CRC function, returns 0 else returns INVALID_CRC
	
**************************************************************************/

static int check_crc16(hycomm_param_t *hc_param, uint8_t *msg,
                       const int msg_length)
{
	int ret;
	uint16_t crc_calc;
	uint16_t crc_received;
			
	crc_calc = crc16(msg, msg_length - 2);
	crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];
	
	/* Check CRC of msg */
	if (crc_calc == crc_received) {
			ret = 0;
	} else {
			char s_error[64];
			snprintf(s_error, sizeof(s_error),
					"invalid crc received %0X - crc_calc %0X", 
					crc_received, crc_calc);
			ret = INVALID_CRC;
			error_treat(hc_param, ret, s_error);
	}

	return ret;
}




/************************************************************************* 
 
	Function to compute the response length based on function code within
	the query sent to the VFD
	
**************************************************************************/

static unsigned int compute_response_length(hycomm_param_t *hc_param, 
					  uint8_t *query)
{
	(void)hc_param;
	int resp_length;

	switch (query[1])
	{
		case 0x01:
		/* Huanyang VFD - Function Read */
		
		case 0x02:
		/* Huanyang VFD - Function Write */
		resp_length = 8;
		break;
		
		case 0x03:
		/* Huanyang VFD - Write Control Data */
		resp_length = 6;
		break;
		
		case 0x04:
		/* Huanyang VFD - Read Control Data */
		resp_length = 8;
		break;	 
		
		case 0x05:
        /* Huanyang VFD - Write Inverter Frequency Data */
		resp_length = 7;
		break;	
		
		case 0x06:
		/* Huanyang VFD - Reserved */
		
		case 0x07:
		/* Huanyang VFD - Reserved */		
		return -1;
        break;
		
		case 0x08:
        /* Huanyang VFD - Loop Test (not implemented)*/
		return -1;
        break;
		
		default:
        return -1;
		break;
	}

  return resp_length;
} 
  
/************************************************************************* 
 
	Function to add a checksum to the end of a query and send. 
 
**************************************************************************/  
  
static int hycomm_send(hycomm_param_t *hc_param, uint8_t *query, int query_length )  
{  
	int ret;

	unsigned short s_crc;
	int i;
	
	/* calculate the CRC */
	s_crc = crc16(query, query_length);
      
    /* append the CRC to then end of the query */
	query[query_length++] = s_crc >> 8;  
    query[query_length++] = s_crc & 0x00FF;  
    
	if (hc_param->debug)
	{
		printf("hycomm query = ");
		for (i = 0; i < query_length; i++)
			printf("[%.2X]", query[i]);
		printf("\n");
	}
	
	/* write the query to the fd */
	ret = write(hc_param->fd, query, query_length);

	/* Return the number of bytes written (0 to n)
	or PORT_SOCKET_FAILURE on error */
	if ((ret == -1) || (ret != query_length))
	{
		error_treat(hc_param, ret, "Write port/socket failure");
		ret = PORT_FAILURE;
	}
	
	return ret;
}  





/***********************************************************************

        Definition to be used multiple times in receive_msg function

***********************************************************************/


#define WAIT_DATA()                                                                \
{                                                                                  \
    while ((select_ret = select(hc_param->fd+1, &rfds, NULL, NULL, &tv)) == -1) {  \
            if (errno == EINTR) {                                                  \
                    printf("A non blocked signal was caught\n");                   \
                    /* Necessary after an error */                                 \
                    FD_ZERO(&rfds);                                                \
                    FD_SET(hc_param->fd, &rfds);                                   \
            } else {                                                               \
                    error_treat(hc_param, SELECT_FAILURE, "Select failure");       \
                    return SELECT_FAILURE;                                         \
            }                                                                      \
    }                                                                              \
                                                                                   \
    if (select_ret == 0) {                                                         \
			printf("WAIT_DATA(): comms time out\n");   			                \
            /* Call to error_treat is done later to manage exceptions */           \
            return COMM_TIME_OUT;                                                  \
    }                                                                              \
}  

  
/*********************************************************************** 
 
	Function to monitor for the reply from the hycomm slave. 
	This function blocks for timeout seconds if there is no reply. 
 
	Returns a negative number is an error occurred.
	The variable msg_length is assigned th number of characters
	received.
	
***********************************************************************/  
  
int receive_msg(hycomm_param_t *hc_param, int msg_length_computed,
						uint8_t *msg, int *msg_length)
{  
  	int select_ret;
	int read_ret;
	fd_set rfds;  
	struct timeval tv;
	int length_to_read;
	unsigned char *p_msg;

	if (hc_param->debug)
		printf("waiting for message (%d bytes)...\n", 
				msg_length_computed);

	/* add a file descriptor to the set */
	FD_ZERO(&rfds);  
    FD_SET(hc_param->fd, &rfds); 
	
	/* wait for a response */
	tv.tv_sec = 0;
	tv.tv_usec = TIME_OUT_BEGIN_OF_FRAME; 

	length_to_read = msg_length_computed;

	WAIT_DATA();
   
	/* read the message */
    (*msg_length) = 0;
    p_msg = msg;	
		  
	while (select_ret) /* loop to receive data until end of msg	or time out */
	{
		read_ret = read(hc_param->fd, p_msg, length_to_read);
		if (read_ret == -1) {
                    error_treat(hc_param, PORT_SOCKET_FAILURE, "Read port/socket failure");
                    return PORT_SOCKET_FAILURE;
		}

		if (read_ret == 0) {
                    error_treat(hc_param, PORT_SOCKET_FAILURE, "Short read");
                    return PORT_SOCKET_FAILURE;
		}

                if (hc_param->debug) {
                    int i;
                    printf("read %d bytes: ", read_ret);
                    for (i = 0; i < read_ret; i ++) {
                        printf(" 0x%02x", p_msg[i]);
                    }
                    printf("\n");
                }

		/* sum bytes received */
		(*msg_length) += read_ret;
		
		if ((*msg_length) < msg_length_computed) {
			/* We can receive a shorter message than msg_length_computed as
		  		some functions return one byte in the data field. Check against
				the received data length stored in msg[2] */
			if ((*msg_length >= 2) && (*msg_length == msg[2]+5)) {
				/* we have received the whole message */
				length_to_read = 0;
			} else {			
				/* Message is incomplete */
				length_to_read = msg_length_computed - (*msg_length);
		 	
		 		if (hc_param->debug) {
				printf("message was incomplete, length still to read = [%.2X]", length_to_read);
				printf("\n");
				}
			}
		} else {
			length_to_read = 0;
		}
		
		/* Moves the pointer to receive other data */
        p_msg = &(p_msg[read_ret]);

		if (length_to_read > 0) {
			/* If no character at the buffer wait
			   TIME_OUT_END_OF_TRAME before to generate an error. */
			tv.tv_sec = 0;
			tv.tv_usec = TIME_OUT_END_OF_FRAME;
			
			WAIT_DATA();
		} else {
				/* All chars are received */
				select_ret = FALSE;
		}
		
	}

        if (hc_param->debug) {
            int i;
            printf("returning %d byte message: ", *msg_length);
            for (i = 0; i < *msg_length; i ++) {
                printf(" 0x%02x", msg[i]);
            }
            printf("\n");
        }

	/* OK */
	return 0;
}  
   
/********************************************************************* 
 
	Function to check the correct response is returned and that the 
	checksum is correct. 
	
	Returns the data byte(s) in the response.
 
**********************************************************************/  
  
static int hycomm_check_response(hycomm_param_t *hc_param,  
						uint8_t *query, uint8_t *response)  
{  
    int response_length_computed;
	int response_length;
	int crc_check;
	int ret;
  
	response_length_computed = compute_response_length(hc_param, query);
	if (hc_param->debug) {
		printf("response_length_computed = %d", response_length_computed);
		printf("\n");
	}
	
    ret = receive_msg(hc_param, response_length_computed, 
					  response, &response_length);  
						 
	if (ret == 0) {
		
		/* good response so check the CRC*/
		crc_check = check_crc16(hc_param, response, response_length);
				if (hc_param->debug) {
			printf("crc check = %.2d", crc_check);
			printf("\n");
		}
					  
		if (crc_check != 0)
			return crc_check;
			
		if (hc_param->debug) {
			printf("we received a message of [%.2X] bytes, with a valid crc", response_length);
			printf("\n");
		}
				
	} else if (ret == COMM_TIME_OUT) {
		error_treat(hc_param, ret, "Communication time out");
		return ret;
	} else {
		return ret;
	}
	
	return 0;
}  
  
/*********************************************************************** 
 
    The following functions construct the required query into 
    a hycomm query packet. 
 
***********************************************************************/  
  
int build_query(hycomm_data_t *hc_data, unsigned char *query )  
{  
	/* build Hunayang request packet based on function code and return the 
	packet length (less CRC - 2 bytes) */
	
	switch (hc_data->function)
	{
		case FUNCTION_READ:
		case FUNCTION_WRITE:
			query[0] = hc_data->slave;
			query[1] = hc_data->function;
			query[2] = 0x03;
			query[3] = hc_data->parameter;
			query[4] = hc_data->data >> 8;
			query[5] = hc_data->data & 0x00FF;		
			return 6;
			break;
		
		case WRITE_CONTROL_DATA:
		case READ_CONTROL_STATUS:
			query[0] = hc_data->slave;
			query[1] = hc_data->function;
			query[2] = 0x01;
			query[3] = hc_data->data & 0x00FF;
			return 4;
			break;	 
		
		case WRITE_FREQ_DATA:
			query[0] = hc_data->slave;
			query[1] = hc_data->function;
			query[2] = 0x02;
			query[3] = hc_data->data >> 8; 
			query[4] = hc_data->data & 0x00FF;
			return 5;
		break;	
		
		case 0x06:
			/* Huanyang VFD - Reserved */
		
		case 0x07:
			/* Huanyang VFD - Reserved */		
			return -1;
        	break;
		
		case LOOP_TEST:
  			return -1;
        	break;
		default:
        	return -1;
			break;
	}


	
}  
  
  
/************************************************************************ 
 
    hy_comm 
 
    sends and receives "hycomm" messages to and from a Huanyang VFD 
     
*************************************************************************/  

int hy_comm(hycomm_param_t *hc_param, hycomm_data_t *hc_data)
{
	int query_length;
	int query_ret;
	int response_ret;
	int msg_function_code;

	unsigned char query[MIN_QUERY_SIZE];
	unsigned char response[MAX_PACKET_SIZE];
	
	/* build the request query */
	query_length = build_query(hc_data, query);
	if (hc_param->debug) {
		printf("\n");
		printf("query_length = %d", query_length);
		printf("\n");
	} 
	
	/* add CRC to the query and send */
	query_ret = hycomm_send(hc_param, query, query_length);
	if (hc_param->debug) {
		printf("query_ret = %d", query_ret);
		printf("\n");
	} 
	
	if (query_ret > 0){
		/* query was sent so get the response from the VFD */
		response_ret = hycomm_check_response(hc_param, query, response);
		
		if (response_ret == 0) {
		
			msg_function_code = response[1];
			if (hc_param->debug) {
				printf("the message function code is = [%.2X]", msg_function_code);
				printf("\n");
			}
			
			/* check that the returned function code is the same as the query */
			if (msg_function_code != hc_data->function)
				return ILLEGAL_FUNCTION;
							
			/* the returned data length */
			hc_data->ret_length = response[2];
		
			switch (msg_function_code)
			{
				case FUNCTION_READ:
				case FUNCTION_WRITE:
					hc_data->ret_parameter = response[3];
					if (hc_data->ret_length == 2) {
						hc_data->ret_data = response[4];
					} else {
						hc_data->ret_data = response[4] << 8 | response[5];
					}
					break;
					
				case WRITE_CONTROL_DATA:
					hc_data->ret_parameter = 0x00;
					hc_data->ret_data = response[3];
					break;
					
				case READ_CONTROL_STATUS:
					hc_data->ret_parameter = response[3];
					hc_data->ret_data = response[4] << 8 | response[5];
					break;
					
				case WRITE_FREQ_DATA:
					hc_data->ret_parameter = response[3];
					hc_data->ret_data = response[3] << 8 | response[4];
					break;	
					
				default:
        			return -1;
					break;
			}
					
		
			if (hc_param->debug) {
				printf("response parameter = [%.2X]", hc_data->ret_parameter);
				printf("\n");
				printf("response data = [%.4X]", hc_data->ret_data);
				printf("\n");
			}
		}
				 
	} else {
		response_ret = query_ret;
	}
	
	return response_ret;
}  
  

/************************************************************************ 
 
	Initializes the hycomm_param_t structure for RTU
	- device: "/dev/ttyS0"
	- baud:   9600, 19200, 57600, 115200, etc
	- parity: "even", "odd" or "none" 
	- data_bits: 5, 6, 7, 8 
	- stop_bits: 1, 2 
 
**************************************************************************/  
  
void hycomm_init(hycomm_param_t *hc_param, const char *device,
                     int baud, const char *parity, int data_bit,
                     int stop_bit)
{
        memset(hc_param, 0, sizeof(hycomm_param_t));
        rtapi_strxcpy(hc_param->device, device);
        hc_param->baud = baud;
        rtapi_strxcpy(hc_param->parity, parity);
        hc_param->debug = FALSE;
        hc_param->data_bit = data_bit;
        hc_param->stop_bit = stop_bit;
}  


/************************************************************************ 
 
	Closes the file descriptor in RTU mode
 
**************************************************************************/ 

void hycomm_close(hycomm_param_t *hc_param)
{
        if (tcsetattr(hc_param->fd, TCSANOW, &(hc_param->old_tios)) < 0)
                perror("tcsetattr");

        close(hc_param->fd);
}
  
  
/************************************************************************ 
 
    Sets up a serial port for RTU communications 
 
**************************************************************************/  

int hycomm_connect(hycomm_param_t *hc_param)
{
        struct termios tios;
        speed_t speed;

        if (hc_param->debug) {
                printf("Opening %s at %d bauds (%s)\n",
                       hc_param->device, hc_param->baud, hc_param->parity);
        }

        /* The O_NOCTTY flag tells UNIX that this program doesn't want
           to be the "controlling terminal" for that port. If you
           don't specify this then any input (such as keyboard abort
           signals and so forth) will affect your process

           Timeouts are ignored in canonical input mode or when the
           NDELAY option is set on the file via open or fcntl */

        hc_param->fd = open(hc_param->device, O_RDWR | O_NOCTTY | O_NDELAY);

        if (hc_param->fd < 0) {
                perror("open");
                printf("ERROR Can't open the device %s (errno %d)\n",
                       hc_param->device, errno);
                return -1;
        }

        /* Save */
        tcgetattr(hc_param->fd, &(hc_param->old_tios));
        memset(&tios, 0, sizeof(struct termios));

        /* C_ISPEED     Input baud (new interface)
           C_OSPEED     Output baud (new interface)
        */
        switch (hc_param->baud) {
        case 110:
                speed = B110;
                break;
        case 300:
                speed = B300;
                break;
        case 600:
                speed = B600;
                break;
        case 1200:
                speed = B1200;
                break;
        case 2400:
                speed = B2400;
                break;
        case 4800:
                speed = B4800;
                break;
        case 9600: 
                speed = B9600;
                break;
        case 19200:
                speed = B19200;
                break;
        case 38400:
                speed = B38400;
                break;
        case 57600:
                speed = B57600;
                break;
        case 115200:
                speed = B115200;
                break;
        default:
                speed = B9600;
                printf("WARNING Unknown baud rate %d for %s (B9600 used)\n",
                       hc_param->baud, hc_param->device);
        }

        /* Set the baud rate */
        if ((cfsetispeed(&tios, speed) < 0) ||
            (cfsetospeed(&tios, speed) < 0)) {
                perror("cfsetispeed/cfsetospeed\n");
                return -1;
        }

        /* C_CFLAG      Control options
           CLOCAL       Local line - do not change "owner" of port
           CREAD        Enable receiver
        */
        tios.c_cflag |= (CREAD | CLOCAL);
        /* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

        /* Set data bits (5, 6, 7, 8 bits)
           CSIZE        Bit mask for data bits
        */
        tios.c_cflag &= ~CSIZE;
        switch (hc_param->data_bit) {
        case 5:
                tios.c_cflag |= CS5;
                break;
        case 6:
                tios.c_cflag |= CS6;
                break;
        case 7:
                tios.c_cflag |= CS7;
                break;
        case 8:
        default:
                tios.c_cflag |= CS8;
                break;
        }

        /* Stop bit (1 or 2) */
        if (hc_param->stop_bit == 1)
                tios.c_cflag &=~ CSTOPB;
        else /* 2 */
                tios.c_cflag |= CSTOPB;

        /* PARENB       Enable parity bit
           PARODD       Use odd parity instead of even */
        if (strncmp(hc_param->parity, "none", 4) == 0) {
                tios.c_cflag &=~ PARENB;
        } else if (strncmp(hc_param->parity, "even", 4) == 0) {
                tios.c_cflag |= PARENB;
                tios.c_cflag &=~ PARODD;
        } else {
                /* odd */
                tios.c_cflag |= PARENB;
                tios.c_cflag |= PARODD;
        }
        
        /* Read the man page of termios if you need more information. */

        /* This field isn't used on POSIX systems 
           tios.c_line = 0; 
        */

        /* C_LFLAG      Line options 

           ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
           ICANON       Enable canonical input (else raw)
           XCASE        Map uppercase \lowercase (obsolete)
           ECHO Enable echoing of input characters
           ECHOE        Echo erase character as BS-SP-BS
           ECHOK        Echo NL after kill character
           ECHONL       Echo NL
           NOFLSH       Disable flushing of input buffers after
           interrupt or quit characters
           IEXTEN       Enable extended functions
           ECHOCTL      Echo control characters as ^char and delete as ~?
           ECHOPRT      Echo erased character as character erased
           ECHOKE       BS-SP-BS entire line on line kill
           FLUSHO       Output being flushed
           PENDIN       Retype pending input at next read or input char
           TOSTOP       Send SIGTTOU for background output

           Canonical input is line-oriented. Input characters are put
           into a buffer which can be edited interactively by the user
           until a CR (carriage return) or LF (line feed) character is
           received.  

           Raw input is unprocessed. Input characters are passed
           through exactly as they are received, when they are
           received. Generally you'll deselect the ICANON, ECHO,
           ECHOE, and ISIG options when using raw input
        */

        /* Raw input */
        tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        /* C_IFLAG      Input options 

           Constant     Description
           INPCK        Enable parity check
           IGNPAR       Ignore parity errors
           PARMRK       Mark parity errors
           ISTRIP       Strip parity bits
           IXON Enable software flow control (outgoing)
           IXOFF        Enable software flow control (incoming)
           IXANY        Allow any character to start flow again
           IGNBRK       Ignore break condition
           BRKINT       Send a SIGINT when a break condition is detected
           INLCR        Map NL to CR
           IGNCR        Ignore CR
           ICRNL        Map CR to NL
           IUCLC        Map uppercase to lowercase
           IMAXBEL      Echo BEL on input line too long
        */
        if (strncmp(hc_param->parity, "none", 4) == 0) {
                tios.c_iflag &= ~INPCK;
        } else {
                tios.c_iflag |= INPCK;
        }

        /* Software flow control is disabled */
        tios.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        /* C_OFLAG      Output options
           OPOST        Postprocess output (not set = raw output)
           ONLCR        Map NL to CR-NL

           ONCLR ant others needs OPOST to be enabled
        */         

        /* Raw output */
        tios.c_oflag &=~ OPOST;

        /* C_CC         Control characters 
           VMIN         Minimum number of characters to read
           VTIME        Time to wait for data (tenths of seconds)

           UNIX serial interface drivers provide the ability to
           specify character and packet timeouts. Two elements of the
           c_cc array are used for timeouts: VMIN and VTIME. Timeouts
           are ignored in canonical input mode or when the NDELAY
           option is set on the file via open or fcntl.

           VMIN specifies the minimum number of characters to read. If
           it is set to 0, then the VTIME value specifies the time to
           wait for every character read. Note that this does not mean
           that a read call for N bytes will wait for N characters to
           come in. Rather, the timeout will apply to the first
           character and the read call will return the number of
           characters immediately available (up to the number you
           request).

           If VMIN is non-zero, VTIME specifies the time to wait for
           the first character read. If a character is read within the
           time given, any read will block (wait) until all VMIN
           characters are read. That is, once the first character is
           read, the serial interface driver expects to receive an
           entire packet of characters (VMIN bytes total). If no
           character is read within the time allowed, then the call to
           read returns 0. This method allows you to tell the serial
           driver you need exactly N bytes and any read call will
           return 0 or N bytes. However, the timeout only applies to
           the first character read, so if for some reason the driver
           misses one character inside the N byte packet then the read
           call could block forever waiting for additional input
           characters.

           VTIME specifies the amount of time to wait for incoming
           characters in tenths of seconds. If VTIME is set to 0 (the
           default), reads will block (wait) indefinitely unless the
           NDELAY option is set on the port with open or fcntl.
        */
        /* Unused because we use open with the NDELAY option */
        tios.c_cc[VMIN] = 0;
        tios.c_cc[VTIME] = 0;

        if (tcsetattr(hc_param->fd, TCSANOW, &tios) < 0) {
                perror("tcsetattr\n");
                return -1;
        }

	return 0;
}
