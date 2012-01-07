#ifndef SB_UTIL_RANGE_H
#define SB_UTIL_RANGE_H

namespace sb_util
{
  /** Returns val if val is between low and high; otherwise, 
   * returns low or high. */
  double clamp(double val, double low=0.0, double high=1.0);
    

  /** Generates a new value between new_low and new_high that corresponds 
   * to val's position between old_low and old_high. */
  double interpolate(double val, double old_low, double old_high, 
      double new_low, double new_high);


  /** Same as interpolate(), but clamps the return value between new_low
   * and new_high. */
  double saturate(double val, double old_low, double old_high, 
      double new_low, double new_high);

} // end namespace sb_util

# endif


