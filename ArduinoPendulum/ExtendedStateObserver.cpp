#include "ExtendedStateObserver.h"


		ExtendedStateObserver::ExtendedStateObserver(bool linear)
		{
			this->linear = linear;
		}

		double* ExtendedStateObserver::ObserveState(double samplingPeriod, double u, double b0, double processVariable)
		{
			if (linear)
			{
				//Linear gains
				gain1 = 1;
				gain2 = 1 / (3 * samplingPeriod);
				gain3 = 2 / (pow(8, 2) * pow(samplingPeriod, 2));
			}
			else
			{
				//Nonlinear gains
				gain1 = 1;
				gain2 = 1 / (2 * pow(samplingPeriod, 0.5));
				gain3 = 2 / (pow(5, 2) * pow(samplingPeriod, 1.2));
			}

			double e, fe, fe1;

			//e = z[0] − y
			//fe = fal(e, 0.5, δ)
			//fe1 = fal(e, 0.25, δ)

			e = z[0] - processVariable; //pv = y
			fe = NonLinearFunction(e, 0.5, samplingPeriod); //3rd parameter as sampling period as shown in
			fe1 = NonLinearFunction(e, 0.25, samplingPeriod); //From PID to Active Disturbance Rejection Control by Jingqing Han

			//z[0](k + 1) = z[0](k) + h ∗ (z[1](k) − β01 ∗ e)
			//z[1](k + 1) = z[1](k) + h ∗ (z[2](k) − β02 ∗ fe) + b0 * u
			//z[2](k + 1) = z[2](k) + h ∗ (−β03 ∗ fe1)

			z[0] = z[0] + (samplingPeriod * z[1]) - (gain1 * e);
			z[1] = z[1] + (samplingPeriod * (z[2] + (b0 * u))) - (gain2 * fe);
			z[2] = z[2] - (gain3 * fe1);

			return z;
		}

		double ExtendedStateObserver::NonLinearFunction(double eta, double alpha, double delta)
		{
			//e / (δ ^ α − 1),                         | e | <= δ
			//pow(abs(e), α) * sign(e),                | e | >= δ

			if (abs(eta) <= delta)
			{
				return eta / (pow(delta, 1 - alpha));
			}
			else
			{
				return pow(abs(eta), alpha) * sign(eta);
			}
		}
