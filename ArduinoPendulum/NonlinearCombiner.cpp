 #include "NonlinearCombiner.h"


		NonlinearCombiner::NonlinearCombiner(double amplificationCoefficient, double dampingCoefficient)
		{
			this->amplificationCoefficient = amplificationCoefficient;
			this->dampingCoefficient = dampingCoefficient;
		}

		double NonlinearCombiner::Combine(double* td, double b0, double* eso, double precisionCoefficient)
		{
			double e1, e2, u0;

			e1 = (td[0]) - (eso[0]);
			e2 = (td[1]) - (eso[1]);

			u0 = -SetPointJumpPrevention(e1, dampingCoefficient * e2, amplificationCoefficient, precisionCoefficient);

			//Contains disturbance rejection
			return (u0 + (eso[2])) / b0; // b0 must be positive
		}
