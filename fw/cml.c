/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "hal/hal.h"
#include "sh.h"
#include "lib.h"

void led(void *pARG)
{
	halLED(LED_RED);
}

void prt(void *pARG)
{
	int		x;

	halLED(LED_GREEN);
	x = 123456789;

	printf("1: long long string %i %i %i \r\n", x, x, x);
	printf("2: long long string %i %i %i \r\n", x, x, x);
	printf("3: long long string %i %i %i \r\n", x, x, x);
	printf("4: long long string %i %i %i \r\n", x, x, x);
	printf("5: long long string %i %i %i \r\n", x, x, x);
	printf("6: long long string %i %i %i \r\n", x, x, x);
	printf("7: long long string %i %i %i \r\n", x, x, x);
	printf("8: long long string %i %i %i \r\n", x, x, x);
	printf("9: long long string %i %i %i \r\n", x, x, x);
}

shCMD_t		cmList[] = {

	{"led", &led},
	{"prt", &prt},
	{NULL, NULL},
};

