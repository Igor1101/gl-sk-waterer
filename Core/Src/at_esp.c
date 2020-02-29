/*
 * at_esp.c
 *
 *  Created on: Feb 23, 2020
 *      Author: igor
 */


#include "main.h"
#include "at_esp.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <cmsis_os.h>
static char recv_buf[100];
static char at_string[100];
volatile at_t at_wifi = {
		.available = false,
		.connected = false,
		.psk = NULL,
		.ssid = NULL,
		.state = AT_INITIAL,
		.autoprocess = false
};
bool at_available(void){
	at_clearin();
	uputs(AT_SERIAL, "AT");
	at_gets(recv_buf, sizeof recv_buf);
	if(strcmp("AT", recv_buf)) {
		char buf[50];
		snprintf(buf, sizeof buf, "expected \"AT\", recved:\"%s\"", recv_buf);
		uputs(DEBUG_SERIAL, buf);
		return false;
	}
	at_gets(recv_buf, sizeof recv_buf);
	if(!strcmp("OK", recv_buf)) {
		char buf[50];
	snprintf(buf, sizeof buf, "expected \"OK\", recved:\"%s\"", recv_buf);
		return false;
	}
	return true;
}
bool at_todo(void){
	at_clearin();
	at_cmd("AT");
	at_cmd("AT+CWMODE=1");
	at_cmd("AT+CWJAP=\"DOM32\",\"lenochka090576\"");
	at_cmd("AT+CIFSR");
	osDelay(100);
	at_cmd("AT+CIPMUX=1");
	osDelay(100);
	at_cmd("AT+CIPSERVER=1,80");
	osDelay(10000);
	at_webpage("<html><head></head><body>"
			"<h1>GL SK BSP Webserver</h1>"
			"<p>Wireless Firmware 0.0.0</p>"
			"</body></html>"
			);
}

void at_webpage(char* pg) {
	static char cmdbuf[20];
	snprintf(cmdbuf,sizeof cmdbuf, "AT+CIPSEND=0,%d", strlen(pg));
	osDelay(100);
	at_cmd(cmdbuf);
	osDelay(100);
	uputs(AT_SERIAL, pg);
	at_cmd("AT+CIPCLOSE=0");
}
volatile static int at_index=0;
volatile static bool at_strready=false;

void at_addchar(char ch) {
	HAL_UART_Transmit(DEBUG_SERIAL, &ch, 1, 100);
	if(at_strready == true)
		return;
	at_string[at_index] = ch;
	if(ch == '\0' || ch == '\r'|| ch == '\n' || at_index >= sizeof at_string) {
		at_string[at_index] = '\0';
		at_strready = true;
		//debug("string added");
		// process if needed
		at_index = 0;
	} else {
		at_index++;
	}
}

void at_clearin(void) {
	at_index = 0;
	at_strready = false;
	memset(at_string, 0, sizeof at_string);
}

size_t at_gets(char*str, size_t sz){
	if(str == NULL)
		return 0;
	volatile int i=0;
	while(!at_strready && i < TIMEOUT_VALUE) i++;
	strncpy(str, at_string, sz);
	at_clearin();
	return strlen(str);
}

void at_clearout(void)
{
	at_cmd("");
	osDelay(AT_DELAY_DEF);
}

void at_cmd(char*cmd)
{
	do {
		bool first = true;
		if(!first) {
			at_clearin();
		}
		first = false;
		uputs(AT_SERIAL, cmd);
		char buf[100];
		snprintf(buf, sizeof buf, "sent: \"%s\"", cmd);
		debug(buf);
		osDelay(AT_DELAY_DEF);
		at_gets(recv_buf, sizeof recv_buf);
	}
	while(strstr(recv_buf, cmd) == NULL);
}

void at_init(void) {
	at_clearout();
	uputs(AT_SERIAL, "ATE1");
	osDelay(AT_DELAY_DEF);
	at_clearin();
	if(!at_available()) {
		debug("ERR AT NOT AVAILABLE");
		return;
	}
	at_todo();
}
