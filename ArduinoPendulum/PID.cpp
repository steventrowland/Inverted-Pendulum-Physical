#include "PID.h"

        /// <summary>
        /// Initializes the PID.
        /// </summary>
        /// <param name="kp">Proportional gain</param>
        /// <param name="ki">Integral gain</param>
        /// <param name="kd">Derivative gain</param>
        /// <param name="maxOutput">Maximum output for constraint</param>
        PID::PID(double kp, double ki, double kd)
        {
            this->kp = kp;
            this->ki = ki;
            this->kd = kd;
        }

        /// <summary>
        /// Calculates the setpoint with a custom sampling period.
        /// </summary>
        /// <param name="setpoint">Target</param>
        /// <param name="processVariable">Actual</param>
        /// <param name="samplingPeriod">Period between calculations</param>
        /// <returns>Returns output of PID</returns>
        double PID::Calculate(double setpoint, double processVariable, double samplingPeriod)
        {
            double POut, IOut, DOut;

            if (samplingPeriod > 0)
            {
                error = setpoint - processVariable;

                POut = kp * error;

                integral += error * samplingPeriod;
                IOut = ki * integral;

                DOut = kd * ((error - previousError) / samplingPeriod);

                output = POut + IOut + DOut;
                
                previousError = error;
            }

            return output;
        }
