/*
 * at_esp.h
 *
 *  Created on: Feb 29, 2020
 *      Author: igor
 */

#ifndef SRC_AT_ESP_H_
#define SRC_AT_ESP_H_
#include <stdbool.h>
#define AT_SERIAL &huart3
#define AT_DELAY_DEF 200
#define AT_DELAY_EXT 2000
typedef enum {
	AT_INITIAL,
	AT_NOTAVAIL,
	AT_INITIALIZING,
	AT_CONNECTING,
	AT_CONNECTFAILED,
	AT_CONNECTED,
	AT_WEBSERVER
}at_state_t;
typedef struct {
	bool available:1;
	bool connected:1;
	bool autoprocess:1;
	at_state_t state;
	char* ssid;
	char*psk;
}at_t;
#define TIMEOUT_VALUE 100000
void at_addchar(char ch);
size_t at_gets(char*str, size_t sz);
void at_clearin(void);
void at_init(void);
#endif /* SRC_AT_ESP_H_ */
