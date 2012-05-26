#include "sonar.h"
namespace sonar
{
	KalmanFilter::KalmanFilter(double pv, double mv)
	{
		this->pv = pv;
		this->mv = mv;
		this->estimate = 0.0;
		this->error = 1.0;
		this->gain = 0.0;
	}

	void KalmanFilter::setPV(double pv)
	{
		this->pv = pv;
	}

	void KalmanFilter::setMV(double mv)
	{
		this->mv = mv;
	}

	double KalmanFilter::getEstimate(double observation)
	{
		double error_pre = error + pv;
		gain = error_pre / ( error_pre + mv);
		estimate = estimate + ( gain * (observation - estimate) );
		error = error_pre * (1.0 - gain);
		return estimate;
	}


	SonarFilter::SonarFilter(
			double min, 
			double max,
			double pv,
			double mv)
	{
		this->min = min;
		this->max = max;
		this->prev = 0.0;
		this->kf.setPV(pv);
		this->kf.setMV(mv);
	}

	double SonarFilter::filter(double raw)
	{
		if ( raw < min ) return prev;
		if ( raw > max ) return prev;
		prev = kf.getEstimate(raw);
		return prev;
	}

	double SonarFilter::getDanger(double raw)
	{
		double val = filter(raw);
		if ( val == 0 ) return -1.0;
		return 1.0 / ( val * val );
	}
}

