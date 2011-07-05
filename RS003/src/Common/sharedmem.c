#include <stdio.h>
#include <string.h>
#ifndef WIN32
#include<sys/ipc.h>
#include<sys/shm.h>
#endif
#include <stdlib.h>

#include "Linux4Win.h"

unsigned char *InitSharedMem(int key, int size)
{
  shm_key_t shmid;
  unsigned char *p;

  if ((shmid = shmget(key, size, IPC_CREAT | 0666)) == EOF){
    fprintf(stderr, "ShmGet Error\n");
    exit(0);
  }
  if ((p = (unsigned char*)shmat(shmid, 0, 0)) == (void*)(-1)) {
    fprintf(stderr, "ShmAt Error\n");
    exit(0);
  }
  memset(p, 0, size);
  return p;
}
