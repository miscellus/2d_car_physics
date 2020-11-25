#!/bin/bash

set -e

defines=""
compile_flags="$defines -g -O0 -std=c99 -Wall -Wextra -pedantic $(pkg-config --cflags sdl2 SDL2_net)"
link_flags="$(pkg-config --libs sdl2 SDL2_net) -lm"

gcc $compile_flags fake_controller_server.c -o fake_controller_server.program $link_flags
gcc $compile_flags 2d_car_main.c -o 2d_car.program $link_flags

termite -e 'bash -c "./fake_controller_server.program"' &
termite -e 'bash -c "./2d_car.program"'