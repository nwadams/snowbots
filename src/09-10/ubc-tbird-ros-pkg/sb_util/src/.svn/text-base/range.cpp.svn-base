#include <sb_util/range.h>

namespace sb_util
{
  /** Returns val if val is between low and high; otherwise, 
   * returns low or high. */
  double clamp(double val, double low, double high)
  {
    if      (val > high) return high;
    else if (val < low) return low;
    return val;
  }
    

  /** Generates a new value between new_low and new_high that corresponds 
   * to val's position between old_low and old_high. */
  double interpolate(double val, double old_low, double old_high, 
      double new_low, double new_high)
  {
    double slope = (new_high - new_low) / (old_high - old_low);
    double intercept = new_low - ( slope * old_low ); // b = y - mx
    return (slope * val) + intercept;                 // y = mx + b
  }



  /** Same as interpolate(), but clamps the return value between new_low
   * and new_high. */
  double saturate(double val, double old_low, double old_high, 
      double new_low, double new_high)
  {
    return clamp( interpolate(val, old_low, old_high, new_low, new_high),
        new_low, new_high );
  }

} // end namespace sb_util


