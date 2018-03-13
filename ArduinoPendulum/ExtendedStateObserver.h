#pragma once
#include <math.h>
#include <stdlib.h>

		//ESO
		class ExtendedStateObserver
		{
       int sign(double val) 
      {
        return (0 < val) - (val < 0);
      }

    
			/*
			    //INPUTS: b0/u, y
			    //OUTPUTS: Z1, Z2, Z3
			    // An augmented variable Formula is introduced to a 2-order system. 
			    // Using z1, z2, and z3 to estimate x1, x2, and x3, respectively,
			    // an extended state observer is designed as:
			    // http://ieeexplore.ieee.org/xpls/icp.jsp?arnumber=7334962#references
			    
			    //Total Disturbance Estimation and Rejection via ESO
			    
			    //where β01, β02, and β03 are observer gains
	
			    e = z1 − y
			    fe = fal(e, 0.5, δ)
			    fe1 = fal(e, 0.25, δ)
	
			    z1(k + 1) = z1(k) + h ∗ (z2(k) − β01 ∗ e)
			    z2(k + 1) = z2(k) + h ∗ (z3(k) − β02 ∗ fe) + b0 * u
			    z3(k + 1) = z3(k) + h ∗ (−β03 ∗ fe1)
	
			    fal(e, α, δ) = {
			        e / (δ * α − 1),                         |e| <= δ
			        pow(abs(e), α) * sign(e),                |e| >  δ
			        
			*/
		private:
			double z[3] = {0, 0, 0}; //z[0] Maintains previous state //double z[3]
			double gain1 = 0; //State
			double gain2 = 0;
			double gain3 = 0;
			bool linear = false;


			/// <summary>
			/// 
			/// </summary>
			/// <param name="samplingPeriod">Sampling frequency</param>
			/// <param name="linear">Determines whether using linear or non linear gains</param>
		public:
			ExtendedStateObserver(bool linear);

			/// <summary>
			/// 
			/// </summary>
			/// <param name="samplingPeriod"></param>
			/// <param name="u"></param>
			/// <param name="b0"></param>
			/// <param name="processVariable"></param>
			/// <returns></returns>
			double* ObserveState(double samplingPeriod, double u, double b0, double processVariable);

			/// <summary>
			/// 
			/// </summary>
			/// <param name="eta"></param>
			/// <param name="alpha"></param>
			/// <param name="delta"></param>
			/// <returns></returns>
		private:
			double NonLinearFunction(double eta, double alpha, double delta); //fal
		};
