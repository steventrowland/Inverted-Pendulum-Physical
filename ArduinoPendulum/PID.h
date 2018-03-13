/*
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
*/

    class PID
    {
        private:
          double kp;
          double ki;
          double kd;
          double integral;
          double error;
          double previousError;
          double output;
          long useTime;

        /// <summary>
        /// Initializes the PID.
        /// </summary>
        /// <param name="kp">Proportional gain</param>
        /// <param name="ki">Integral gain</param>
        /// <param name="kd">Derivative gain</param>
        /// <param name="maxOutput">Maximum output for constraint</param>
        public:
          PID(double kp, double ki, double kd);
        
        /// <summary>
        /// Calculates the setpoint with a custom sampling period.
        /// </summary>
        /// <param name="setpoint">Target</param>
        /// <param name="processVariable">Actual</param>
        /// <param name="samplingPeriod">Period between calculations</param>
        /// <returns>Returns output of PID</returns>
          double Calculate(double setpoint, double processVariable, double samplingPeriod);
    };
