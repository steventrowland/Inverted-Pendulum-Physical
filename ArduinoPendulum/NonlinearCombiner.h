#pragma once

#include "AbstractMath.h"


		//NLC
		class NonlinearCombiner : public AbstractMath
		{
			/*
			    //INPUTS: e1, e2
			    //OUTPUTS: u0
	
			    // Nonlinear combination of errors and differential errors
			    // where c, r, and h1 are positive parameters
	
			    e1 = v1 − z1
			    e2 = v2 − z2
	
			    u0 = −fhan(e1, c * e2, r, h1)
	
			    // Disturbance rejection
			    // where b0 is a positive parameter
	
			    u = u0 − z3 / b0
	
			    
			*/

		private:
			double amplificationCoefficient = 0; //r
			double dampingCoefficient = 0; //c

			/// <summary>
			/// 
			/// </summary>
			/// <param name="amplificationCoefficient">Corresponds to the limit of acceleration.</param>
			/// <param name="dampingCoefficient">Damping coefficient to be adjusted in the neighborhood of unity.</param>
		public:
			NonlinearCombiner(double amplificationCoefficient, double dampingCoefficient);

			/// <summary>
			/// 
			/// </summary>
			/// <param name="trajectory">v1,v2</param>
			/// <param name="b0">Estimate of coefficient b within +-50%</param>
			/// <param name="eso">Extended State Observer</param>
			/// <returns></returns>
			double Combine(double* td, double b0, double* eso, double precisionCoefficient);
		};
