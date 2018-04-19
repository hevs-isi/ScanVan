/*
 * keypress.h
 *
 *  Created on: Apr 18, 2018
 *      Author: scanvandev
 */

#ifndef KEYPRESS_H_
#define KEYPRESS_H_

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

void reset_terminal_mode();
void set_conio_terminal_mode();
int kbhit();
int getch();

#endif /* KEYPRESS_H_ */
