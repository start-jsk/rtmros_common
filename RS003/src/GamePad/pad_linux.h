/************************************************************************************
GamePad RT-Component
Copyright (c) 2010, Segway-Japan, Ltd.
All rights reserved.

Contact us if you use this software for sell.
If you use this software not for sell, you can use this software under BSD lisence.
See the files LICENSE.TXT and LICENSE-BSD.TXT for more details.                     
************************************************************************************/
#ifndef __PAD_LINUX_H__
#define __PAD_LINUX_H__




enum {
    PAD_SDL_JS_HAT_UP    = 1<<0,
    PAD_SDL_JS_HAT_RIGHT = 1<<1,
    PAD_SDL_JS_HAT_DOWN  = 1<<2,
    PAD_SDL_JS_HAT_LEFT  = 1<<3,

    PAD_SDL_JS_AXIS1_X = 0,
    PAD_SDL_JS_AXIS1_Y = 1,
    PAD_SDL_JS_AXIS2_X = 2,
    PAD_SDL_JS_AXIS2_Y = 3,
    PAD_SDL_JS_AXIS_MAX = 32767,
    PAD_SDL_JS_AXIS_MIN = -32767
};

int pad_sdl_js_open(const char* device = "/dev/input/js0");
void pad_sdl_js_close();

bool pad_sdl_js_isok();
void pad_sdl_js_setok(bool b);

void pad_sdl_js_update();
int pad_sdl_js_axis(int n);

float pad_sdl_js_axisf(int n);


int pad_sdl_js_button(int n);
int pad_sdl_js_hat(int n);
//push
int pad_sdl_js_hat_push(int n);
int pad_sdl_js_button_push(int n);




#endif
