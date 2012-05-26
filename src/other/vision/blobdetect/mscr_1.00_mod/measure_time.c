/*
**  Source file for mscr. (c) 2007 Per-Erik Forssen
**
**  This program is free software; you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation; either version 2 of the License, or
**  (at your option) any later version.
**  
**  See the file COPYING for details.
**
*/

#include <sys/time.h>
#include <stdlib.h>

#include "measure_time.h"

/*
 *  Time, accurate up to microseconds
 */
double accurate_clock() {
  struct timeval tp;
  gettimeofday(&tp,NULL);

  return tp.tv_sec + tp.tv_usec/1e6;
}

timewin *timewin_new(int tsize) {
  timewin *tw;
  tw=(timewin*) calloc(1,sizeof(timewin));
  tw->nticks=tsize;
  tw->ticks=(double*)calloc(tw->nticks,sizeof(double));
  return tw;
}

void timewin_free(timewin *tw) {
  free(tw->ticks);
  free(tw);
}

timewin *timewin_addtime(timewin *tw) {
  int k;
  /* Shift old times */
  for(k=1;k<tw->nticks;k++) {
    tw->ticks[k-1]=tw->ticks[k];
  }
  tw->ticks[tw->nticks-1]=accurate_clock();  /* Add new time */
  return tw;
}

/*
**  Compute rate (typically framerate) from a window of measurements
*/
double timewin_rate(timewin *tw) {
  return (double)(tw->nticks-1.0)/(tw->ticks[tw->nticks-1]-tw->ticks[0]);
}
