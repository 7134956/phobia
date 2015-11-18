/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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
#include <math.h>

#include <SDL2/SDL.h>
#include <GL/gl.h>

float			angle;

static void
Read()
{
	int		tel[2], n;

	n = scanf("%i %i \n", tel + 0, tel + 1);

	angle = atan2f(tel[1], tel[0]) * 180.f / M_PI;
}

static void
Draw()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	glRotatef(angle, 0.f, 0.f, 1.f);

	glColor3f(1.f, 1.f, 1.f);

	glBegin(GL_TRIANGLES);

	glVertex2f(0.f, 3e-2f);
	glVertex2f(0.f, - 3e-2f);
	glVertex2f(.9f, 3e-2f);

	glVertex2f(0.f, - 3e-2f);
	glVertex2f(.9f, - 3e-2f);
	glVertex2f(.9f, 3e-2f);

	glEnd();
}

int main(int argc, char *argv[])
{
	SDL_Window	*window;
	SDL_GLContext	glcontext;
	SDL_Event	event;
	int		w, h, running;
	double		temp;

	SDL_Init(SDL_INIT_VIDEO);

	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 16);

	window = SDL_CreateWindow("VIS", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
			0, 0, SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_OPENGL);

	if (window == NULL)
		exit(1);

	SDL_GetWindowSize(window, &w, &h);

	glcontext = SDL_GL_CreateContext(window);

	glClearColor(0.f, 0.f, 0.f, 0.f);
	glDisable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	temp = (double) w / (double) h;
	glOrtho(- temp, temp, - 1., 1., - 1, 1.);
	glMatrixMode(GL_MODELVIEW);

	running = 1;

	while (running) {

		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_QUIT:
					running = 0;
					break;

				case SDL_KEYDOWN:

					switch (event.key.keysym.sym)
					{
						case SDLK_ESCAPE:
						case SDLK_q:
							running = 0;
							break;
					}
					break;
			}
		}

		Read();
		Draw();

		glFlush();
		SDL_GL_SwapWindow(window);
	}

	SDL_GL_DeleteContext(glcontext);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}

