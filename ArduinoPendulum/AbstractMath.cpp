#include "AbstractMath.h"

		double AbstractMath::SetPointJumpPrevention(double target, double targetDerivative, double r0, double h)
		{
			double d, a, a0, a1, a2, y, sy, sa;

			d = pow(r0, 2) * h;
			a0 = h * targetDerivative;
			y = target + a0;

			a1 = sqrt(d * (d + 8 * abs(y)));
			a2 = a0 + sign(y) * (a1 - d) / 2;
			sy = (sign(y + d) - sign(y - d)) / 2; //returns 1, or -1

			a = (a0 + y - a2) * sy + a2;
			sa = (sign(a + d) - sign(a - d)) / 2; //returns 1, or -1

			return -r0 * ((a / d) - sign(a)) * sa - r0 * sign(a);
		}
    int sign(int value)
    { 
      return ((value>=0)||(value<0)); 
    }
    double sign(double value) 
    { 
      return ((value>=0)||(value<0)); 
    }
    float sign(float value)
    { 
      return ((value>=0)||(value<0)); 
    }
