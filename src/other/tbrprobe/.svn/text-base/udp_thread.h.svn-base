/** @file udp_thread.h
 
 The UDP thread handles communication with client programs.  See
 examples/c/tbrclient/ for example client code.
**/ 

pthread_t udpThreadId;
pthread_mutex_t udpMutex;

/** The thread function spawned from tbrprobe.c that manages communication
 with all client processes. */
void* udpThread(void *arg);

/**
 * Returns true if the UDP thread is initialized and ready to send/receive
 * commands.
**/
int udpThreadIsReady(void);

