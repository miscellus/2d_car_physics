#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_net.h>

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define TAU 6.283185307179586f
#define PI 3.141592653589793f

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


typedef struct Control_Input {
	// Control_Input_Button_Flags buttons;
	s16 acceleration_axis;
	s16 turn_axis;
} __attribute__((packed)) Control_Input;

typedef struct Sensor_Data {
	float delta_x;
	float delta_y;
	float heading_direction;
	float velocity;
	u64 time;
} __attribute__((packed)) Sensor_Data;


__attribute__((noreturn)) static void panic(const char *format, ...) {
	fprintf(stderr, "[ERROR] ");
	va_list args;
	va_start(args, format);
	vfprintf(stderr, format, args);
	va_end(args);
	exit((int)(intptr_t)format);
}

int main() {


	if (SDL_Init(0) != 0) {
		panic("SDL_Init Error: %s\n", SDL_GetError());
	}

	if (SDLNet_Init() < 0) {
		panic("SDLNet_Init Error: %s\n", SDL_GetError());
	}

	UDPsocket udp_socket = SDLNet_UDP_Open(9001);
	if (!udp_socket) {
		panic("ERROR: Could not open UDP socket.\n");
	}
	else {
		fprintf(stderr, "Listening for UDP packets on port 9001.\n");
	}


	UDPpacket *udp_packet = SDLNet_AllocPacket(32);

	b32 running = true;

	while (running) {

		static struct {
			float acceleration_direction; // -1 for backwards or 1 for forwards
			u32 mode_switch_time;
		} state = {
			1,
			0
		};


		Sensor_Data sensor_data;

		while (0 == SDLNet_UDP_Recv(udp_socket, udp_packet)) {
			// Spin
		}

		sensor_data = *(Sensor_Data *)udp_packet->data;

		float angle_to_target = atan2f(sensor_data.delta_y, sensor_data.delta_x);
		float angle_delta = angle_to_target - sensor_data.heading_direction;
		if (angle_delta > PI) {
			angle_delta -= TAU;
		}
		else if (angle_delta < -PI) {
			angle_delta += TAU;
		}

		float abs_angle_delta = fabsf(angle_delta);
		assert(abs_angle_delta < (TAU+0.0001f));


		float heading_x = cosf(sensor_data.heading_direction);
		float heading_y = sinf(sensor_data.heading_direction);
		float distance_to_target = sqrtf(sensor_data.delta_x*sensor_data.delta_x + sensor_data.delta_y*sensor_data.delta_y);
		float dot = (sensor_data.delta_x*heading_x + sensor_data.delta_y*heading_y) / distance_to_target;

		float acceleration_factor = 1.0f;
		if (distance_to_target < 200*sensor_data.velocity) {
			acceleration_factor = distance_to_target/(200*sensor_data.velocity);
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

		*(Control_Input *)udp_packet->data = result;
		udp_packet->len = sizeof(result);
		if (0 == SDLNet_UDP_Send(udp_socket, -1, udp_packet)) {
			panic("Error: Could not send sensor data. SDLNet Error: '%s'\n", SDLNet_GetError());
		}
	}


	return 0;
}
