#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <SDL2/SDL.h>

#define MFD_IMPLEMENTATION
#include "miscellus_file_dialog.h"

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f

#define LERP(a,b,t) ((1.0f-(t))*(a) + (t)*(b))

typedef char s8;
typedef short s16;
typedef int s32;
typedef long long s64;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

typedef u32 b32;
#ifndef __cplusplus
enum {false = 0, true = 1};
#endif

typedef u64 umm;
typedef s64 smm;

typedef int check_size8[sizeof(u8)==1&&sizeof(s8)==1 ? 1 : -1];
typedef int check_size16[sizeof(u16)==2&&sizeof(s16)==2 ? 1 : -1];
typedef int check_size32[sizeof(u32)==4&&sizeof(s32)==4 ? 1 : -1];
typedef int check_size64[sizeof(u64)==8&&sizeof(s64)==8 ? 1 : -1];
typedef int check_sizeumm[sizeof(umm)==sizeof((void *)0) ? 1 : -1];
typedef int check_sizesmm[sizeof(smm)==sizeof((void *)0) ? 1 : -1];


typedef struct Length_Buffer {
	umm length;
	u8 *data;
} Length_Buffer;

typedef enum Application_Mode {
	APP_MODE_VIEW = 0,
	COUNT_APP_MODE
} Application_Mode;

typedef struct Car
{
	float x;
	float y;
	float length;
	float width;
	float direction;
	float velocity;
	float acceleration;
	float turning_rate;
	float turning_span;
	float rolling_resistance;
	float breaking_resistance;
	float front_wheel_angle;
	float rear_wheel_angle;
	float half_wheel_base;

	// SDL_Surface *surface;
	SDL_Texture *texture;
} Car;

typedef struct Application_State {
	Application_Mode mode;
	
	u8 *keys;
	int keys_length;

	Car car;

	float target_x;
	float target_y;
	float target_radius;

} Application_State;

static Length_Buffer read_entire_file(s8 *path) {

	Length_Buffer result = {0};

	FILE *file = fopen(path, "rb");

	if (file) {

		u64 length;
		u8 *contents;

		fseek(file, 0, SEEK_END);
		length = ftell(file);
		fseek(file, 0, SEEK_SET);

		contents = malloc(length);

		u64 amount_read = fread(contents, 1, length, file);
		fclose(file);

		if (amount_read == length) {
			result.length = length;
			result.data = contents;
		} else {
			free(contents);
		}
	}

	return result;
}


__attribute__((noreturn)) static void panic(char *format, ...) {
	fprintf(stderr, "[ERROR] ");
	va_list args;
	va_start(args, format);
	vfprintf(stderr, format, args);
	va_end(args);
	exit(-1);
}

int main(int argc, char **argv) {

	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		panic("SDL_Init Error: %s\n", SDL_GetError());
	}

	SDL_Window *window = SDL_CreateWindow(
		"Miscellus 2D Car Physics",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		1024, 768,
		SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

	if (!window) {
		panic("SDL_CreateWindow Error: %s\n", SDL_GetError());
	}

	SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (!renderer) {
		SDL_DestroyWindow(window);
		panic("SDL_CreateRenderer Error: %s\n", SDL_GetError());
	}

	SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

	Application_State app_state = {0};
	u8 *keys = app_state.keys = SDL_GetKeyboardState(&app_state.keys_length);

	s32 window_width;
	s32 window_height;
	SDL_GetWindowSize(window, &window_width, &window_height);

	app_state.car.x = 0.5f*window_width;
	app_state.car.y = 0.5f*window_height;
	app_state.car.width = 96.0f;
	app_state.car.length = 2.0f*app_state.car.width;
	app_state.car.half_wheel_base = app_state.car.length*0.97*0.5f;
	app_state.car.direction = 0.0f;
	app_state.car.velocity = 0.0f;
	app_state.car.acceleration = 0.2f;
	app_state.car.turning_rate = 0.02f;
	app_state.car.turning_span = 1.1f;
	app_state.car.rolling_resistance = 0.005f;
	app_state.car.breaking_resistance = 0.045f;

	app_state.target_radius = 20;
	app_state.target_x = (rand() / (float)(RAND_MAX))*1024;
	app_state.target_y = (rand() / (float)(RAND_MAX))*768;
	{
		s8 file[1024] = "car.bmp";
	
		if (0) {
			if (!miscellus_file_dialog(file, 1024, false)) panic("NO!\n");
		}
	
		SDL_Surface *surface = SDL_LoadBMP(file);
		if (!surface) panic("Also no!\n");
		app_state.car.texture = SDL_CreateTextureFromSurface(renderer, surface);
		SDL_FreeSurface(surface);
	}

	s32 frame_count = 0;

	SDL_Event e;
	b32 quit = false;

	while (!quit) {

		SDL_PumpEvents();

		while (SDL_PollEvent(&e)) {
			if (e.type == SDL_QUIT){
				quit = true;
			}
		}
		

		SDL_SetRenderDrawColor(renderer, 70, 80, 90, 255);

		SDL_RenderClear(renderer);

		SDL_GetWindowSize(window, &window_width, &window_height);

		b32 key_modifier_control = keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL];

		switch (app_state.mode) {
			case APP_MODE_VIEW: {

				if (keys[SDL_SCANCODE_R]) {
					app_state.car.x = 0.5f*window_width;
					app_state.car.y = 0.5f*window_height;
					app_state.car.velocity = 0.0f;
					app_state.car.direction = 0.0f;
				}

				if (key_modifier_control && keys[SDL_SCANCODE_O]) {
					s8 out_path[1024];
					miscellus_file_dialog(out_path, 1024, false);
				}

				if (keys[SDL_SCANCODE_UP]) {
					app_state.car.velocity += app_state.car.acceleration;
				}

				if (keys[SDL_SCANCODE_LEFT]) {
					app_state.car.front_wheel_angle -= app_state.car.turning_rate;
					if (app_state.car.front_wheel_angle < -app_state.car.turning_span) app_state.car.front_wheel_angle = -app_state.car.turning_span;
				}
				if (keys[SDL_SCANCODE_RIGHT]) {
					app_state.car.front_wheel_angle += app_state.car.turning_rate;
					if (app_state.car.front_wheel_angle > app_state.car.turning_span) app_state.car.front_wheel_angle = app_state.car.turning_span;
				}

				{
					float resistance = app_state.car.rolling_resistance;
					
					if (keys[SDL_SCANCODE_DOWN] || keys[SDL_SCANCODE_SPACE]) {

						if (keys[SDL_SCANCODE_SPACE] || (app_state.car.velocity > 1)) {
							resistance += app_state.car.breaking_resistance;
						}
						if (keys[SDL_SCANCODE_DOWN] && (app_state.car.velocity < 1)) {
							app_state.car.velocity -= 0.45f*app_state.car.acceleration;
						}
					}

					resistance += 0.002f*((app_state.car.front_wheel_angle < 0) ? -app_state.car.front_wheel_angle : app_state.car.front_wheel_angle);

					app_state.car.velocity *= (1.0f - resistance);

					// app_state.car.front_wheel_angle *= 0.8f;

					float sin_direction = sinf(app_state.car.direction);
					float cos_direction = cosf(app_state.car.direction);

					float wheel_offset_x = cos_direction*app_state.car.half_wheel_base;
					float wheel_offset_y = sin_direction*app_state.car.half_wheel_base;
					float rear_wheel_x = app_state.car.x - wheel_offset_x;
					float rear_wheel_y = app_state.car.y - wheel_offset_y;
					float front_wheel_x = app_state.car.x + wheel_offset_x;
					float front_wheel_y = app_state.car.y + wheel_offset_y;
					float new_front_wheel_x = front_wheel_x + cosf(app_state.car.direction + app_state.car.front_wheel_angle) * app_state.car.velocity;
					float new_front_wheel_y = front_wheel_y + sinf(app_state.car.direction + app_state.car.front_wheel_angle) * app_state.car.velocity;
					float new_rear_wheel_x = rear_wheel_x + cos_direction * app_state.car.velocity;
					float new_rear_wheel_y = rear_wheel_y + sin_direction * app_state.car.velocity;

					app_state.car.x = (new_front_wheel_x + new_rear_wheel_x)*0.5f;
					app_state.car.y = (new_front_wheel_y + new_rear_wheel_y)*0.5f;

					float new_direction = atan2f((new_front_wheel_y - new_rear_wheel_y), (new_front_wheel_x - new_rear_wheel_x));

					app_state.car.direction = new_direction;


					SDL_Rect tire_rect = {
						(s32)(front_wheel_x),
						(s32)(front_wheel_y),
						(s32)(100),
						(s32)(10),
					};

					SDL_RenderCopyEx(renderer, app_state.car.texture, NULL, &tire_rect, RAD_TO_DEG*(app_state.car.direction + app_state.car.front_wheel_angle), NULL, SDL_FLIP_NONE);

					tire_rect.x = rear_wheel_x;
					tire_rect.y = rear_wheel_y;
					SDL_RenderCopyEx(renderer, app_state.car.texture, NULL, &tire_rect, RAD_TO_DEG*(app_state.car.direction + app_state.car.rear_wheel_angle), NULL, SDL_FLIP_NONE);

				}

				// app_state.car.x += cosf(app_state.car.direction) * app_state.car.acceleration;
				// app_state.car.y += sinf(app_state.car.direction) * app_state.car.acceleration;
				// app_state.car.direction -= app_state.car.turning_rate;

			} break;

			default:
				assert(0);
		}

		SDL_SetRenderDrawColor(renderer, 255, 255, 0, 200);
		SDL_Rect hot_rect = {0, 0, 100, 100};
		SDL_RenderDrawRect(renderer, &hot_rect);

		SDL_Rect car_rect = {
			(s32)(app_state.car.x - 0.5f*app_state.car.length),
			(s32)(app_state.car.y - 0.5f*app_state.car.width),
			(s32)(app_state.car.length),
			(s32)(app_state.car.width),
		};


		// SDL_RenderCopyEx(renderer,
		// 	app_state.car.texture,
		// 	NULL,
		// 	&car_rect,
		// 	app_state.car.direction * RAD_TO_DEG,
		// 	NULL,
		// 	SDL_FLIP_NONE);

		car_rect.x = 0.5*(window_width - app_state.car.length);
		car_rect.y = 0.5*(window_height - app_state.car.width) + 130;

		SDL_RenderCopyEx(renderer, app_state.car.texture, NULL, &car_rect, 0, NULL, SDL_FLIP_NONE);
		car_rect.x -= 3*(app_state.car.length);
		SDL_RenderCopyEx(renderer, app_state.car.texture, NULL, &car_rect, 0, NULL, SDL_FLIP_NONE);

		SDL_Rect target_rect = {
			(s32)(app_state.target_x - app_state.target_radius),
			(s32)(app_state.target_y - app_state.target_radius),
			(s32)(2.0f*app_state.target_radius),
			(s32)(2.0f*app_state.target_radius),
		};
		SDL_SetRenderDrawColor(renderer, 255, 255, 0, 127);
		SDL_RenderFillRect(renderer, &target_rect);

		if ((frame_count & 0xff) == 0) {
			float fps_average = frame_count / ( SDL_GetTicks() / 1000.0f );
			printf("fps_average: %.2f\n", fps_average);
		}

		SDL_RenderPresent(renderer);
		++frame_count;
	}

	SDL_Quit();
	return 0;
}
