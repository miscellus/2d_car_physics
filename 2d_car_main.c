#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_net.h>

// #define MFD_IMPLEMENTATION
// #include "miscellus_file_dialog.h"

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define TAU 6.283185307179586f
#define PI 3.141592653589793f

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

typedef struct Car {
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

	float target_x;
	float target_y;
	// SDL_Surface *surface;
	SDL_Texture *texture;
} Car;

typedef enum Control_Input_Button_Flags
{
	CAR_INPUT_BUTTON_ACCELERATE = 0x01,
	CAR_INPUT_BUTTON_REVERSE =    0x02,
	CAR_INPUT_BUTTON_TURN_LEFT =  0x04,
	CAR_INPUT_BUTTON_TURN_RIGHT = 0x08,
	CAR_INPUT_BUTTON_BREAK =      0x10,
} Control_Input_Button_Flags;

typedef struct Control_Input {
	// Control_Input_Button_Flags buttons;
	s16 acceleration_axis;
	s16 turn_axis;
} Control_Input;

typedef struct Sensor_Data {
	float delta_x;
	float delta_y;
	float heading_direction;
	float velocity;
	u64 time;
} Sensor_Data;

typedef struct Application_State {
	Application_Mode mode;

	const u8 *keys;
	int keys_length;

	UDPsocket udp_socket;
	s32 udp_channel;
	UDPpacket *udp_packet;

	Control_Input car_input;


	b32 human_control;

	Car cars[2];

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

void update_car(Car *car, Control_Input input) {

	if (input.acceleration_axis > 0) {
		car->velocity += car->acceleration * ((float)input.acceleration_axis/32768.0f);
	}

	car->front_wheel_angle += car->turning_rate * ((float)input.turn_axis/32768.0f);
	float effective_turning_span = car->turning_span - fabsf(car->velocity * 0.07f);
	if (car->front_wheel_angle < -effective_turning_span) car->front_wheel_angle = -effective_turning_span;
	if (car->front_wheel_angle > effective_turning_span) car->front_wheel_angle = effective_turning_span;

	float resistance = car->rolling_resistance;

	b32 reverse = input.acceleration_axis < 0;
	b32 breaking = (reverse && car->velocity > 0) || (!reverse && car->velocity < 0);//input.buttons & (CAR_INPUT_BUTTON_BREAK);

	// if (breaking || reverse) {

	if (breaking) {
		resistance += car->breaking_resistance;
	}
	if (reverse) {
		car->velocity -= 0.45f*car->acceleration;
	}
	// }

	resistance += 0.002f*((car->front_wheel_angle < 0) ? -car->front_wheel_angle : car->front_wheel_angle);

	car->velocity *= (1.0f - resistance);

	if (abs(input.turn_axis) < 0xf) {
		car->front_wheel_angle *= 0.9f;
	}

	float sin_direction = sinf(car->direction);
	float cos_direction = cosf(car->direction);

	float wheel_offset_x = cos_direction*car->half_wheel_base;
	float wheel_offset_y = sin_direction*car->half_wheel_base;
	float rear_wheel_x = car->x - wheel_offset_x;
	float rear_wheel_y = car->y - wheel_offset_y;
	float front_wheel_x = car->x + wheel_offset_x;
	float front_wheel_y = car->y + wheel_offset_y;
	float new_front_wheel_x = front_wheel_x + cosf(car->direction + car->front_wheel_angle) * car->velocity;
	float new_front_wheel_y = front_wheel_y + sinf(car->direction + car->front_wheel_angle) * car->velocity;
	float new_rear_wheel_x = rear_wheel_x + cos_direction * car->velocity;
	float new_rear_wheel_y = rear_wheel_y + sin_direction * car->velocity;

	car->x = (new_front_wheel_x + new_rear_wheel_x)*0.5f;
	car->y = (new_front_wheel_y + new_rear_wheel_y)*0.5f;

	car->direction = atan2f((new_front_wheel_y - new_rear_wheel_y), (new_front_wheel_x - new_rear_wheel_x));
}


Sensor_Data car_get_sensor_data(Car *car) {
	Sensor_Data result = {0};

	result.delta_x = car->target_x - car->x;
	result.delta_y = car->target_y - car->y;
	result.heading_direction = car->direction;
	result.velocity = car->velocity;

	return result;

#if 0

	float absolute_angle_to_target = atan2f(dy, dx);

	result.angle_delta_to_target = absolute_angle_to_target - car->direction;
	result.distance_to_target = sqrtf(dx*dx + dy*dy);
	result.velocity = car->velocity;

	float heading_x = cosf(car->direction);
	float heading_y = sinf(car->direction);
	float dot = (dx*heading_x + dy*heading_y)/result.distance_to_target;

	result.dot = dot;
	// float dot2 = result.velocity * result.distance_to_target * cosf(result.angle_delta_to_target);

	// printf("Dot compare: %f - %f = %f\n", dot, dot2, dot - dot2);
	return result;
#endif
}

Control_Input compute_robot_car_input(Sensor_Data car_sensors) {
	static struct {
		float acceleration_direction; // -1 for backwards or 1 for forwards
		u32 mode_switch_time;
	} state = {
		1,
		0
	};

	float angle_to_target = atan2f(car_sensors.delta_y, car_sensors.delta_x);
	float angle_delta = angle_to_target - car_sensors.heading_direction;
	if (angle_delta > PI) {
		angle_delta -= TAU;
	}
	else if (angle_delta < -PI) {
		angle_delta += TAU;
	}

	float abs_angle_delta = fabsf(angle_delta);
	assert(abs_angle_delta < (TAU+0.0001f));


	float heading_x = cosf(car_sensors.heading_direction);
	float heading_y = sinf(car_sensors.heading_direction);
	float distance_to_target = sqrtf(car_sensors.delta_x*car_sensors.delta_x + car_sensors.delta_y*car_sensors.delta_y);
	float dot = (car_sensors.delta_x*heading_x + car_sensors.delta_y*heading_y) / distance_to_target;

	float acceleration_factor = 1.0f;
	if (distance_to_target < 200*car_sensors.velocity) {
		acceleration_factor = distance_to_target/(200*car_sensors.velocity);
		acceleration_factor *= acceleration_factor * acceleration_factor * 0.5f;
	}

	float turn_factor = 1.0;
	{
		float threshold = 0.75f*PI;
		if (abs_angle_delta < threshold) {
			turn_factor = abs_angle_delta/threshold;
			turn_factor *= turn_factor * turn_factor * 0.9f;
		}
	}

	assert(dot > -1.00001f && dot < 1.00001f);

	Control_Input result = {0};

	if (distance_to_target > 10) {

		if (state.acceleration_direction > 0.0f && (dot < -0.8f)) {
			state.acceleration_direction = -1.0f;
		}
		else if (state.acceleration_direction < 0.0f && (dot > 0.5f)) {
			state.acceleration_direction = 1.0f;
		}

		result.acceleration_axis = 0x7fff * acceleration_factor * state.acceleration_direction;
		result.turn_axis = 0x7fff * turn_factor * state.acceleration_direction * ((angle_delta > 0) ? 1.0f : -1.0f);
	}

	return result;
}



int main(int argc, char **argv) {

	const char *controller_ip = argc > 1 ? argv[1] : "127.0.0.1";
	u16 controller_port = argc > 2 ? atoi(argv[2]) : 9001;

	printf("Connecting to controller on socket (%s:%d)\n", controller_ip, controller_port);

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

	if (SDLNet_Init() < 0) {
		panic("SDLNet_Init Error: %s\n", SDL_GetError());
	}

	SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

	Application_State app_state = {0};
	const u8 *keys = app_state.keys = SDL_GetKeyboardState(&app_state.keys_length);

	s32 window_width;
	s32 window_height;
	SDL_GetWindowSize(window, &window_width, &window_height);

	Car *car = &app_state.cars[0];

	car->x = 0.5f*window_width;
	car->y = 0.5f*window_height;
	car->width = 96.0f;
	car->length = 2.0f*car->width;
	car->half_wheel_base = car->length*0.97*0.5f;
	car->direction = 0;
	car->velocity = 0.0f;
	car->acceleration = 0.09f;
	car->turning_rate = 0.34f;
	car->turning_span = 1.2f;
	car->rolling_resistance = 0.005f;
	car->breaking_resistance = 0.035f;
	car->target_x = (rand() / (float)(RAND_MAX))*1024;
	car->target_y = (rand() / (float)(RAND_MAX))*768;

	{
		s8 file[1024] = "car.bmp";
		SDL_Surface *surface = SDL_LoadBMP(file);
		if (!surface) panic("Also no!\n");
		car->texture = SDL_CreateTextureFromSurface(renderer, surface);
		SDL_SetTextureColorMod(car->texture, 255, 200, 200);
		SDL_FreeSurface(surface);
	}

	app_state.cars[1] = *car;

	app_state.udp_socket = SDLNet_UDP_Open(9000);
	if (!app_state.udp_socket) {
		panic("ERROR: Could not open UDP socket on port 9000.\n");
	}

	IPaddress ip;
	SDLNet_ResolveHost(&ip, controller_ip, controller_port);
	app_state.udp_channel = SDLNet_UDP_Bind(app_state.udp_socket, -1, &ip);
	if (app_state.udp_channel < 0) {
		panic("ERROR: Could not bind IP.\n");
	}

	app_state.udp_packet = SDLNet_AllocPacket(32);

	s32 frame_count = 0;

	SDL_Event e;
	b32 quit = false;

	while (!quit) {

		//
		// Input:
		//

		while (SDL_PollEvent(&e)) {
			if (e.type == SDL_QUIT){
				quit = true;
			}
			else if (e.type == SDL_KEYDOWN)
			{
				switch (e.key.keysym.sym)
				{
					case SDLK_r: {
						// car->x = 0.5f*window_width;
						// car->y = 0.5f*window_height;
						// car->velocity = 0.0f;
						// car->direction = 0.0f;
						car->target_x = (rand() / (float)(RAND_MAX))*window_width;
						car->target_y = (rand() / (float)(RAND_MAX))*window_height;
					} break;

					case SDLK_t: {
						app_state.human_control = !app_state.human_control;
					} break;
				}
			}
		}

		SDL_PumpEvents();

		SDL_GetWindowSize(window, &window_width, &window_height);

		s32 mouse_x, mouse_y;
		SDL_GetMouseState(&mouse_x, &mouse_y);

#if 0
		car->target_x = mouse_x;
		car->target_y = mouse_y;
#endif

		b32 key_modifier_control = keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL];

		Sensor_Data car_sensors = car_get_sensor_data(car);
		car_sensors.time = frame_count;
		app_state.car_input = compute_robot_car_input(car_sensors);

		if (0) {
			*(Sensor_Data *)app_state.udp_packet->data = car_sensors;
			app_state.udp_packet->len = sizeof(car_sensors);
			if (0 == SDLNet_UDP_Send(app_state.udp_socket, app_state.udp_channel, app_state.udp_packet)) {
				panic("Error: Could not send udp_packet.\n");
			}

			if (SDLNet_UDP_Recv(app_state.udp_socket, app_state.udp_packet)) {
				app_state.car_input = *(Control_Input *)app_state.udp_packet->data;
			}
		}


		// printf("car_sensors: {%f, %f, %f, %f}\n", car_sensors.angle_delta_to_target / 6.283185307179586f * 360.0f, car_sensors.distance_to_target, car_sensors.velocity, car_sensors.dot);

		Control_Input car_input = (Control_Input){0};

		if (keys[SDL_SCANCODE_UP])    car_input.acceleration_axis = 0x7fff;
		if (keys[SDL_SCANCODE_DOWN])  car_input.acceleration_axis = -0x8000;
		if (keys[SDL_SCANCODE_LEFT])  car_input.turn_axis = -0x8000;
		if (keys[SDL_SCANCODE_RIGHT]) car_input.turn_axis = 0x7fff;
		if (keys[SDL_SCANCODE_SPACE] && car->velocity > 0) car_input.acceleration_axis = -0x8000;

		//
		// Update:
		//

		update_car(car, app_state.human_control ? car_input : app_state.car_input);

		//
		// Rendering:
		//

		SDL_SetRenderDrawColor(renderer, 70, 80, 90, 255);

		SDL_RenderClear(renderer);

		SDL_Rect car_rect = {
			(s32)(car->x - 0.5f*car->length),
			(s32)(car->y - 0.5f*car->width),
			(s32)(car->length),
			(s32)(car->width),
		};

		SDL_RenderCopyEx(renderer,
			car->texture,
			NULL,
			&car_rect,
			car->direction * RAD_TO_DEG,
			NULL,
			SDL_FLIP_NONE);

#if 0
		car_rect.x = 0.5*(window_width - car->length);
		car_rect.y = 0.5*(window_height - car->width) + 130;

		SDL_RenderCopyEx(renderer, car->texture, NULL, &car_rect, 0, NULL, SDL_FLIP_NONE);
		car_rect.x -= 3*(car->length);
		SDL_RenderCopyEx(renderer, car->texture, NULL, &car_rect, 0, NULL, SDL_FLIP_NONE);
#endif

		SDL_Rect target_rect = {
			(s32)(car->target_x - 10),
			(s32)(car->target_y - 10),
			20,
			20,
		};
		SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
		SDL_RenderFillRect(renderer, &target_rect);


		SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
		SDL_RenderDrawLine(renderer, car->x, car->y, car->target_x, car->target_y);


		SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255);
		{
			float heading_x = cosf(car->direction)*car->velocity;
			float heading_y = sinf(car->direction)*car->velocity;
			SDL_RenderDrawLine(renderer, car->x, car->y, car->x + heading_x*50, car->y + heading_y*50);

		}

		if ((frame_count & 0xff) == 0) {
			float fps_average = frame_count / ( SDL_GetTicks() / 1000.0f );
			printf("fps_average: %.2f\n", fps_average);
		}

		SDL_RenderPresent(renderer);
		++frame_count;
	}

	SDLNet_UDP_Close(app_state.udp_socket);
	SDL_Quit();
	return 0;
}
