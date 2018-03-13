
#include "ADRC_PD.h"


		ADRC_PD::ADRC_PD(double amplificationCoefficient, double dampingCoefficient, double plantCoefficient, double precisionModifier, double kp, double ki, double kd)
		{
			this->amplificationCoefficient = amplificationCoefficient;
			this->dampingCoefficient = dampingCoefficient;
			this->plantCoefficient = plantCoefficient;
			this->precisionModifier = precisionModifier;

			PID pid = PID(kp, ki, kd);
			ExtendedStateObserver eso = ExtendedStateObserver(false);
			NonlinearCombiner nlc = NonlinearCombiner(amplificationCoefficient, dampingCoefficient);

			dateTime = now();
		}

		double ADRC_PD::Calculate(double setpoint, double processVariable)
		{
			samplingPeriod = (now() - dateTime)/1000;

			if (samplingPeriod > 0)
			{
				precisionCoefficient = samplingPeriod * precisionModifier;

				double pdValue = pid->Calculate(setpoint, processVariable, samplingPeriod);

				double pd[2] = {pdValue, previousPD};
        ExtendedStateObserver eso2 = ExtendedStateObserver(false);
				*eso_pass = *eso2.ObserveState(samplingPeriod, output, plantCoefficient, processVariable); //double u, double y, double b0
        NonlinearCombiner nlc = NonlinearCombiner(amplificationCoefficient, dampingCoefficient);
				output = nlc.Combine(pd, plantCoefficient, eso_pass, precisionCoefficient);

				previousPD = pdValue;
				dateTime = now();
			}

			return output;
		}

