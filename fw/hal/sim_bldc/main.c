/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2013 Roman Belov <romblv@gmail.com>

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <unistd.h>
#include <poll.h>

#include "plant.h"
#include "lib.h"
#include "hal/hal.h"

void hal_serial_config(int baud)
{
	/* Do nothing */
}

static void
serial_poll_recv()
{
	struct pollfd	fds[1];
	int		rc, sym;

	fds[0].fd = 0;
	fds[0].events = POLLIN;

	rc = poll(fds, 1, 0);

	if (rc > 0) {

		rc = read(0, (void *) &sym, 1);

		if (rc == 1)
			irq_serial(sym);
	}
}

void hal_serial_send(int sym)
{
	fputc(sym, stdout);
	fflush(stdout);
}

static void
sim(double tend)
{
	int		tel, tl, ts = 0;
	FILE		*fd;

	plant_enable();

	fd = fopen("TEL", "w");

	if (fd == NULL) {

		printf("fopen: %s", strerror(errno));
		exit(1);
	}

	while (plant.tsim < tend) {

		/* Plant model update.
		 * */
		plant_update();

		/* ----------------------------- */

		tl = ts;
		ts = (int) (plant.tsim * 1e+2);

		if (tl < ts) {

			/* 100 Hz.
			 * */
			irq_systick();
			serial_poll_recv();
		}

		/* ----------------------------- */

		//irq_network();

		/* ----------------------------- */

		tel = (plant.tdel > 0.0);

		if (tel) {

			fprintf(fd, "%2.6lf ", plant.tsim);
			fprintf(fd, "%2.3lf %2.3lf %5.4lf %1.4lf %2.2lf ",
					plant.x[0], plant.x[1],
					plant.x[2], plant.x[3], plant.x[4]);
			fprintf(fd, "%i %i ",
					plant.z[0], plant.z[1]);
			fputs("\n", fd);
		}

		/* ----------------------------- */
	}

	fclose(fd);
	puts("\n");
}

int main(int argc, char *argv[])
{
	double		tend = 5.0;

	lib_enable();
	sim(tend);

	return 0;
}

