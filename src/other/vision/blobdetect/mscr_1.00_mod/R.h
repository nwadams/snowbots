/** 
 * @author Matthew Baumann
 * @date May 30 2009
 * @brief The R2 class represents a 2D vector
 * 
 */

#ifndef MATH_R_H
#define MATH_R_H

#include <math.h>


#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
#define W_INDEX 3

#define ABS(x)    (((x) > 0) ? (x) : (-(x)))
#define MAX(x,y)  (((x) > (y)) ? (x) : (y))
#define MIN(x,y)  (((x) < (y)) ? (x) : (y))
#define CLAMP(val,min,max)	(MAX(MIN(val,max),min))

namespace math{

	class R2{
	
		public:
			R2();
			R2(double x, double y);
			R2(const R2& r);
			
			double norm() const;
			R2 normalize();
			
			double dot(const R2& r) const;
			
			double at(int i) const;
			double& at(int i);
			
			R2 proj(const R2& base) const;
			
			R2 operator+(const R2& r) const;
			R2 operator-(const R2& r) const;
			R2 operator*(double n) const;
			R2 operator/(double n) const;
			double operator*(const R2& r) const;
			R2 operator^(const R2& r) const;
			R2 operator=(const R2& r);
			bool operator==(const R2& r) const;
			double operator[](int i) const;
			double& operator[](int i);
		
		protected:
		
			double val[2];
	};
}

#endif