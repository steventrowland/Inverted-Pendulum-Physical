#pragma once
#include "Time-master/TimeLib.h"
#include "PID.h"
#include "ExtendedStateObserver.h"
#include "NonlinearCombiner.h"


		class ADRC_PD
		{
		private:
			PID *pid;

			double dateTime;
      double eso_pass[3];
			double amplificationCoefficient = 0;
			double dampingCoefficient = 0;
			double precisionCoefficient = 0; //0.2
			double samplingPeriod = 0; //0.05
			double plantCoefficient = 0; //b0 approximation
			double precisionModifier = 0;
			double previousPD = 0;
			double output = 0;

			/// <summary>
			/// ADRC implementation utilizing a PD controller in place of a tracking differentiator.
			/// </summary>
			/// <param name="amplificationCoefficient">R</param>
			/// <param name="dampingCoefficient">C</param>
			/// <param name="plantCoefficient">B</param>
			/// <param name="precisionModifier">H0</param>
			/// <param name="kp">P Gain</param>
			/// <param name="kd">D Gain</param>
			/// <param name="maxOutput">Constrained maximum output</param>
		public:

			ADRC_PD(double amplificationCoefficient, double dampingCoefficient, double plantCoefficient, double precisionModifier, double kp, double ki, double kd);

			/// <summary>
			/// Calculates the output given the target value and actual value.
			/// </summary>
			/// <param name="setpoint">Target</param>
			/// <param name="processVariable">Actual</param>
			/// <returns></returns>
			double Calculate(double setpoint, double processVariable);
		};
	

