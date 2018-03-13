   #include "ADRC_PD.h"
  //THESE VALUES MAY CHANGE DEPENDING ON IMPLEMENTATION
  int theMotor = 3;
  int potPin = 2;
  
  const float springConst = 365.2;/* N/m */   //hang an object of known mass by the spring, measure displacement. Divide weight by displacement
  //Spring length = 69.4mm  moves 13.6mm w/ 505g
  const float radius = .122;       // radius from point of rotation to attachment point of springs (m)
  const float springRate = .001325;  //Spring stretches 1.325 mm per degree offset
  const float angle = 140;      //Angle between springs and pendulum (in degrees)
  const float pendulumMass = .05; //kg
  const int vertPot = 710;
  const int vertMotor = 73;
  const int motorMaxRPM = 154;

  ADRC_PD ADRC = ADRC_PD(50.0, 200.0, 4.0, 10.0, 10.0, 0.0, 12.5);
                      //(acceleration, damping, plant, precision, initialP, initialI, initialD);
// Rollie's PID constants    50,       200,     4,     10,        10,       0,        12.5, 




  //THESE PARAMETERS REQUIRE NO ADJUSTMENTS
  int potAngle;
  int encAngle;
  int motorAngle;    //This is 0 only if the system is starting at target position.
  //Motor Range is 145 degrees
  int previousSpeed = 0; //previousSpeed = motorSpeed; (at end of loop())
  int motorSpeed;
  int t;
  int last_t;
  

void setup() {
  // put your setup code here, to run once:
  pinMode(theMotor, OUTPUT);
  int potReading = analogRead(potPin);
  t = millis();
  last_t = t;
  
/*
  if (potReading == vertPot)
  {
    motorAngle = 0;
  }
  else if (potReading >= vertPot)
  {
    //MUST RUN CENTER FINDING CODE
  }
  else
  {
    //MUST RUN CENTER FINDING CODE
  }
*/
  motorAngle = 0;
  delayMicroseconds(1000);
}
/******************************************************************************************************/
void loop() {

  last_t = t;
  t = millis();
  motorAngle = getMotorAngle();
  
  /*  //REMOVED BECAUSE ENCODER MAY NOT HAVE A HIGH ENOUGH DEGREE OF ACCURACY
   * int encReading = encoder.readEncoder();
   *            //Unsure if this is a native function, but if not it will be written/included later
   *            //For now, enc will be treated as an angle readout from the encoder.
   *
   */
  //encAngle = vertEnc - encReading;
  
  int potReading = analogRead(potPin);  
  potAngle = (vertPot - potReading)/2.84;  // (value)/2.84 = angle in degrees

  float springForce = getSpringForce();
  
  float forceOut = ADRC.Calculate(vertPot, potAngle);  //Placeholder function for actual ADRC functionality

  
  
  int motorSpeed = previousSpeed + (forceOut - springForce)/pendulumMass*(t - last_t); // F/m = a -> a*t = dv -> dv + v = new v
  
  
  analogWrite(theMotor, motorSpeed);
  previousSpeed = motorSpeed;
}


float getSpringForce()
{
  float distance = springRate*(potAngle - motorAngle);  //Spring stretches 1.325 mm per degree offset
  float spring = -springConst * distance * sin(angle*PI/180);
  return spring;
}

int getMotorAngle() //returns the new estimated motor angle in degrees
{
  float delta = (previousSpeed/255 * motorMaxRPM) * (t-last_t)/360000;  //Degrees per millisecond
  int nu = motorAngle + delta;
  return nu;
}
