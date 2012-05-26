namespace sonar
{
	class KalmanFilter
	{
	public:
		/** Create a new Kalman filter.
		 * @param pv The process variance of the filter (how fast the world
		 * changes)
		 * @param mv The measurement variance of the filter (how noisy the
		 * measurements are)
		 **/
		KalmanFilter(double pv = 0.01, double mv = 0.01);

		void setPV(double pv);
		void setMV(double mv);

		/** Estimate the true value given a new observation.
		 * @param observation The latest measurement
		 * @return The estimate of the true value
		 **/
		double getEstimate(double observation);

	private:
		double pv, mv;
		double estimate, error, gain;
	};


	class SonarFilter
	{
	public:
		SonarFilter(
				double min = 0.05, 
				double max = 3.0, 
				double pv = 0.01, 
				double mv = 0.01);
		double filter(double raw);
		double getDanger(double raw);
	private:
		double min, max, prev;
		KalmanFilter kf;
	};
}
