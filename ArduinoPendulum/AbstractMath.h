#pragma once
#include <math.h>
#include <stdlib.h>

		class AbstractMath
		{

       int sign(double val) 
      {
        return (0 < val) - (val < 0);
      }
			/*
			    // fhan function is denoted below
			    d = r * h
			    d0 = h * d
			    y = x1 + h * x2
	
			    a0 = sqrt(d2 + 8 * r * abs(y))
	
			    a = {
			        x2 + (a0−d) / 2 * sign(y),  if abs(y) > d0
			        xx2 + yh,                   if abs(y) <= d0
	
			    fhan = {
			        −r * sign(a),               if abs(a) > d
			        -r * a / d,                 if abs(a) <= d
	
			*/

			//
		public:
			double SetPointJumpPrevention(double target, double targetDerivative, double r0, double h); //Get actual name of function,  setpoint jump prevention
		};
