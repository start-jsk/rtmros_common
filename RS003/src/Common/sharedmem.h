#ifndef _SHAREDMEM_H_
#define _SHAREDMEM_H_


#define SHM_KEY 16000


typedef struct robot_status
{
  int		x;
  int		y;
  double	r;
  int		s;
  unsigned char buf[64*64];
  unsigned char buf2[64*64];
} robot_status;

#ifdef __cplusplus
	extern "C" {	/* C++‚Åg—p‚·‚éê‡ŠÖ”Cü–¼‚ª‰ó‚ê‚È‚¢‚æ‚¤‚É */
#endif
unsigned char *InitSharedMem(int key, int size);

#ifdef __cplusplus
}
#endif


#endif

