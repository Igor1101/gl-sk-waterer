/*
 * at_esp.c
 *
 *  Created on: Feb 23, 2020
 *      Author: igor
 */


#include "main.h"
#include <stdbool.h>
static char recv_buf[100];
bool at_available(void)
{
	/*
	uputs(AT_SERIAL, "AT+RST");
	ugets(AT_SERIAL, recv_buf, sizeof recv_buf);
	uputs(DEBUG_SERIAL, "cmd=");
	uputs(DEBUG_SERIAL, "AT");
	uputs(DEBUG_SERIAL, recv_buf);*/
	while(1) {
		char r,t;

		if(HAL_OK == HAL_UART_Receive(DEBUG_SERIAL, &t, 1, 10))
		HAL_UART_Transmit(AT_SERIAL, &t,1,0);
	}
}
