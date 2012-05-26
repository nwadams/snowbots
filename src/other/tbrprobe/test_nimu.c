#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "hardware/nimu_control.h"

int main (int argc, char* argv[]) 
{
  int port = 5; //serial232.c says this means /dev/ttyACM0
  pthread_t nimu_thread;
  
  if (pthread_create (&nimu_thread, NULL, nIMUThread, &port)) {
    printf("Couldn't create thread.\n");
    exit(1);
  }
  
  usleep(250000);
  pthread_join(nimu_thread, NULL);
  return 0;
}
