#include "R.h"

/*********************************************************************/

using namespace math;

R2::R2(){
	val[0] = 0;
	val[1] = 0;
}

R2::R2(double x, double y){
	val[X_INDEX] = x;
	val[Y_INDEX] = y;
}

R2::R2(const R2& r){
	val[0] = r[0];
	val[1] = r[1];
}

double R2::norm() const{
	return sqrtf(val[0]*val[0] + val[1]*val[1]);
}

R2 R2::normalize(){
	double n = 1.0/norm();
	val[0] *= n;
	val[1] *= n;
	return *this;
}

double R2::dot(const R2& r) const{
	return val[0]*r[0] + val[1]*r[1];
}

double R2::at(int i) const{
	return val[ CLAMP(i,0,1) ];
}

double& R2::at(int i){
	return val[ CLAMP(i,0,1) ];
}

R2 R2::proj(const R2& base) const{
	return base*(dot(base)/(base.norm()*base.norm()));
}

R2 R2::operator+(const R2& r) const{
	return R2(val[0]+r[0],val[1]+r[1]);
}

R2 R2::operator-(const R2& r) const{
	return R2(val[0]-r[0],val[1]-r[1]);
}

R2 R2::operator*(double n) const{
	return R2(val[0]*n,val[1]*n);
}

R2 R2::operator/(double n) const{
	return R2(val[0]/n,val[1]/n);
}

double R2::operator*(const R2& r) const{
	return dot(r);
}

R2 R2::operator^(const R2& r) const{
	return R2(0,0);
}

R2 R2::operator=(const R2& r){
	val[0] = r[0];
	val[1] = r[1];
	return *this;
}

bool R2::operator==(const R2& r) const{
	return val[0] == r[0] && val[1] == r[1];
}

double R2::operator[](int i) const{
	return at(i);
}

double& R2::operator[](int i){
	return at(i);
}

/*********************************************************************/