@SET compile_flags=-O0 -std=c99 -Wall -Wextra -pedantic -I.\SDL2-2.0.12\x86_64-w64-mingw32\include
@SET link_flags=-L.\SDL2-2.0.12\x86_64-w64-mingw32\lib -w -Wl,-subsystem,windows -lmingw32 -lSDL2main -lSDL2 -lm -lComdlg32

gcc %compile_flags% 2d_car_main.c -o 2d_car.exe %link_flags%

if %errorlevel% gtr 0 pause & exit

2d_car.exe