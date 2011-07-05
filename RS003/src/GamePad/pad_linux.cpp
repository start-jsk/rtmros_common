/************************************************************************************
GamePad RT-Component
Copyright (c) 2010, Segway-Japan, Ltd.
All rights reserved.

Contact us if you use this software for sell.
If you use this software not for sell, you can use this software under BSD lisence.
See the files LICENSE.TXT and LICENSE-BSD.TXT for more details.                     
************************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "pad_linux.h"


#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <linux/joystick.h>

int fd = 0;
unsigned char axes = 2;
unsigned char buttons = 2;


static bool usejoypad =false;//ysuzuki add
int *axis = NULL;
int *button = NULL;

#define NAME_LENGTH 512




int pad_sdl_js_open(const char* device)
{

  int version = 0x000800;
  char name[NAME_LENGTH] = "Unknown";

  if ((fd = open(device, O_RDONLY)) < 0) {
    perror(device);
    exit(1);
  }

  ioctl(fd, JSIOCGVERSION, &version);
  ioctl(fd, JSIOCGAXES, &axes);
  ioctl(fd, JSIOCGBUTTONS, &buttons);
  ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);

  fcntl(fd, F_SETFL, O_NONBLOCK);

  printf("Joystick (%s) has %d axes and %d buttons. Driver version is %d.%d.%d.\n",
	 name, axes, buttons, version >> 16, (version >> 8) & 0xff, version & 0xff);
  //printf("Testing ... (interrupt to exit)\n");
  
  axis = (int*)calloc(axes, sizeof(int));
  button = (int*)calloc(buttons, sizeof(int));

  return 0;
}

void pad_sdl_js_close()
{
  close(fd);
  fd = 0;
}

bool pad_sdl_js_isok()
{ 
  return(fd && usejoypad); 
  //return true;
}

void pad_sdl_js_setok(bool b)
{
}

void pad_sdl_js_update()
{
  if (fd) {

    int i;
    struct js_event js;
    
    for (int i = 0; i < 4; i++) {
      
      int r = 0;
      while (r < sizeof(struct js_event)) {
        r += read(fd, &js, sizeof(struct js_event) -r);
	switch(js.type & ~JS_EVENT_INIT) {
	case JS_EVENT_BUTTON:
	  //ysuzuki debug printf("number value %d\t%d\n",js.number,js.value );
	  button[js.number] = js.value;


	  break;
	case JS_EVENT_AXIS:
	  axis[js.number] = js.value;
	  break;

	}
      }
    }
    //ysuzuki add-
    if(js.number==5 && js.value==1){
      usejoypad=!usejoypad;
      printf("pad:joypad %s\n", (usejoypad)? "enable" : "disable");
    }
    //-add

  }

}

int pad_sdl_js_axis(int n)
{
  if (axis) {
    return axis[n];
  }
  return 0;
}

float pad_sdl_js_axisf(int n)
{
  int j = pad_sdl_js_axis(n);

  return (float)((float)j / (float)PAD_SDL_JS_AXIS_MAX);
}

int pad_sdl_js_hat(int n)
{
  return 0;
}

int pad_sdl_js_button(int n)
{
  if (button && n < buttons) {
    return button[n];
  }
  return 0;
}

int pad_sdl_js_hat_push(int n)
{
  return 0;
}

int pad_sdl_js_button_push(int n)
{
  return 0;
}
