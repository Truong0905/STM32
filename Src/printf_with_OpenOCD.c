/*
 * printf.c
 *
 *  Created on: Apr 19, 2022
 *      Author: Truong
 */

#include <stdio.h>
extern void initialise_monitor_handles(void);

int main (void)
{
	initialise_monitor_handles();
	printf("Hello World\n");
	return 0 ;
}
