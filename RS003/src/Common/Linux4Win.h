#ifndef _LINUX_FOR_WIN_H
#define _LINUX_FOR_WIN_H

#ifdef WIN32
#include "Windows.h"
#include "winbase.h"

typedef HANDLE	shm_key_t;

#define usleep(ST)			Sleep(ST / 1000)
#define shmat(H, A, F)	MapViewOfFile(H, FILE_MAP_ALL_ACCESS, 0, 0, 0)
#define shmdt(ADDR)		UnmapViewOfFile(ADDR)

#define shmget(H, S, F)	shmget_win(H, S)

#ifdef __cplusplus
extern "C" {	/* C++‚Åg—p‚·‚éê‡ŠÖ”Cü–¼‚ª‰ó‚ê‚È‚¢‚æ‚¤‚É */
#endif
	shm_key_t shmget_win(int key, size_t size);
#ifdef __cplusplus
}
#endif

#else
/* Linux */
typedef int shm_key_t;
#endif
#endif
