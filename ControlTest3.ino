/* Control Test
 *  Tune controller gains
 */

//======Advisor======
int eStop, // global flag to halt robot
  cmdPrcntMtrVltage; // %, duty cycle

volatile bool newPulse = false,
  validPulse;

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoders======
const byte esPin = 3; // Mega: 2 or 3, Leonardo: 3 or 7

const byte pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

volatile int angPos,
  prevAngPos,
  setPos;

volatile long pulseCount;

//======Motor Driver======
const byte mSigPin = 9,
  mEnablePin = 6;

//======Mobile Platform======
byte wheelDiam = 64; // [mm]

//======Circle======
float piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

//======Controller======
int sensorsTmrCtr,
  maxOutVal,
  pubVelRate,
  posDeadZone,
  bounceTime,
  minStaticDutyCycle,
  minStaticMtrCmd,
  minKineticDutyCycle,
  minKineticMtrCmd;

long prevPulseTime;

double minLinearRes;

volatile bool decelerating = false,
  foundTarget = false;

volatile double kp, ki, kd;

volatile int topAngVel, // [deg/s]
  setAngVel, // [deg/s]
  cmdAngVel, // [deg/s]
  cmdPrcntMtrVltge,
  pubMtrCmd,
  sign,
  //setPrcntMtrVltge,
  pulses,
  prevPulses,
  prevPosErr,
  posErr;

volatile long instance,
  sample,
  prevSample;
  
void setup() 
{
  initSystem();

  initBehaviors();

  initSensorsTimer(); // CHANGE to scheduler(); ADD: publish rotational (and later translational) angVel
}

int initSystem()
{
  initNode("ControlTest3");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");

  displayCommands();

  Serial.println("Time\tInst\tSample\tPulses\tSetPos\tActPos\tPosErr\tSetAngV\tCmdAngV\tCmdPMV\tMtrCmd");
}

void displayCommands()
{
  Serial.println("======User Commands======");
  Serial.println("s: stop moving (i.e. emergency stop robot)");
  Serial.println("tx: turn x deg");
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPin), encoderCallback, CHANGE);
}

void initPublishers()
{
  /* Start Motor Channel */
  pinMode(mEnablePin, OUTPUT);
  pinMode(mSigPin, OUTPUT);
}

void initBehaviors()
{
  initVars();

  setParams();
}

void initVars()
{
  //===Encoder===
  pulseCount = 0;

  prevAngPos = 0;
  angPos = 0;
  setPos = 0;
  prevPosErr = 0;
  posErr = 0;

  //===Controller===
  topAngVel = 360; // [deg/s]
  setAngVel = topAngVel; // [deg/s]
  cmdAngVel = 0; // [deg/s]

  // ADD for vel ctrl: setPrcntMtrVltge = 0; // %, duty cycle, proportional to percent of time motor turned on
  cmdPrcntMtrVltge = 0;
  pubMtrCmd = 0;
  
  sign = 1;

  prevSample = 0;
  sample = 0;
  pulses = 0;
  prevPulses = 0;

  instance = 0;

  prevPulseTime = 0; // [ms]
  bounceTime = 50; // [ms]
  validPulse = true;
}

void setParams()
{
  double setKp = 0.1875,
    setKi = 0.000,
    setKd = 0.200;

  setPIDGains(setKp, setKi, setKd);
    
  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;

  minLinearRes = (int) round( piApprox * wheelDiam / pulsesPerRev ); // r_{min} [mm/pulse]

  eStop = 1;

  posDeadZone = 9; // [deg], CHANGE: tune based on resolution

  minStaticDutyCycle = 35; // [%] for M2 Decrementing by 1 from 35, 35% for M2 incrementing from 0 by 1; min duty cycle determined by OpenLoopControl.ino
  minStaticMtrCmd = (int) round( 255 * minStaticDutyCycle / 100.0 );
  minKineticDutyCycle = 14; // [%] for M2 Decrementing by 1 from 35, 35% for M2 incrementing from 0 by 1; min duty cycle determined by OpenLoopControl.ino
  minKineticMtrCmd = (int) round( 255 * minKineticDutyCycle / 100.0 );
}

void setPIDGains(float pg, float ig, float dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;

//  Serial.print("Controller ");
//  Serial.print(cid);
//  Serial.print(" got [kp ki kd]: [");
  
  kp = pg;

  ki = ig;

  kd = dg;
  
//  Serial.print(kp, 3);
//  Serial.print(" ");
//  Serial.print(ki, 3);
//  Serial.print(" ");
//  Serial.print(kd, 3);

  //Serial.println("]");
}

void initSensorsTimer()
{
  noInterrupts();           // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;
  sensorsTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  
  TCNT1 = sensorsTmrCtr;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
  interrupts();             // enable all interrupts
}

void loop() 
{
  unsigned long curTime = millis();

  if(decelerating || pulses == 0 || prevPulses == 0)
    bounceTime = 50; // [ms]
  else
    bounceTime = 35; // [ms]
    
  if(curTime - prevPulseTime > bounceTime) // if the noise is gone
  {
    validPulse = true;
    
    if(newPulse)
    {
      //Serial.println("===New Valid Pulse Detected===");

      prevPulseTime = curTime; // record the time of the first pulse in bounce group
      
      newPulse = false;
    }
  }

//  if(decelerating && pulses == 0 && prevPulses == 0) // wait for 2 readings of 0 vel before confirming at rest
//  {
//    Serial.print("===Change Direction to ");
//    if(sign == 1)
//      Serial.print("CW");
//    else
//      Serial.print("CCW");
//    Serial.println("===");
//    
//    sign = -sign; // change direction
//    
//    decelerating = false;
//  }
    
//  if(abs(pulses) > 0)
//  {
    Serial.print(curTime);
    Serial.print("\t");
    Serial.print(instance);
    Serial.print("\t");
    Serial.print(sample);
    Serial.print("\t");
    Serial.print(pulses);
    Serial.print("\t");
    Serial.print(setPos);
    Serial.print("\t");
    Serial.print(angPos);
    Serial.print("\t");
    Serial.print(posErr);
    Serial.print("\t");
    Serial.print(setAngVel);
    Serial.print("\t");
    Serial.print(cmdAngVel);
    Serial.print("\t");
    Serial.print(cmdPrcntMtrVltge);
    Serial.print("\t");
    Serial.println(pubMtrCmd);
  //}
}

//======Interrupt Service Routines======
void encoderCallback()
{
  if(validPulse)
  {
    if(decelerating && pulses == 0 && prevPulses == 0) // wait for 2 readings of 0 vel before confirming at rest
    {
      sign = -sign; // change direction
      
      decelerating = false;
    }
    
    if(sign == 1)
      pulseCount++;
    else
      pulseCount--;

    validPulse = false;
  }

  newPulse = true;
}

/* Run Sensor Loop at x (maybe 10-20) Hz, 
 * interrupt service routine - tick every 0.1 s (10 Hz)
 */
ISR(TIMER1_OVF_vect) // sensors loop!
{
  TCNT1 = sensorsTmrCtr; // set timer

  readUserInput(); // read: get requested tan and rot velocities. in this case, received by uio, which is a separate task(?)
  
  userTask();

  mtrCmd();

  instance++;
}

void readUserInput()
{
  if(stringComplete)
  {
    //Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "s")
    {
      //Serial.println("stop");

      eStop = 1;
    }
    else if(inputString.substring(0,1) == "t") // given in m/s, convert to mm/s, so operations can be done with integer
    { 
      int setPosChange = inputString.substring(1, inputString.length()).toInt(); // get string after 't'
      
      setPos = angPos + setPosChange; // [deg]
    
//      Serial.print("turn ");
//      Serial.print(setPosChange);
//      Serial.print(" deg to ");
//      Serial.print(setPos);
//      Serial.println(" deg\n");

      eStop = 0;
    }

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

void userTask()
{
  readPosition();
  locateTarget();

  if(abs(posErr) > posDeadZone)
  {
    foundTarget = false;
    
//    Serial.print(abs(posErr));
//    Serial.println(" deg > posDeadZone");
//    Serial.print("===Rotate to ");
//    Serial.print(setPos);
//    Serial.println(" deg===");

    if(decelerating && pulses != 0) // if mtr not fully at rest yet
      cmdAngVel = 0; // ensure mtr does not move until 0 vel detected after decelerating
    else if(decelerating && pulses == 0 && prevPulses != 0)
      cmdAngVel = 0; // ensure mtr does not move until 0 vel detected after decelerating
    else
      cmdAngVel = setAngVel; // may only work for indep mode
  }
  else // found target
  {
    foundTarget = true;
    
    cmdAngVel = 0;
    cmdPrcntMtrVltge = 0; // may not need to explicitly set to 0 if cmdAngVel auto-changes this quickly enough 
    pubMtrCmd = 0; // may not need to explicitly set to 0 if cmdAngVel auto-changes this quickly enough 
  }
}

void readPosition()
{
  // Read analog input (e.g. calc rot vel):
  observeSample();

  angPos = (int) round( minAngularRes * ( sample % pulsesPerRev ) ); // [deg]

//    Serial.print("angPos[");
//    Serial.print(i);
//    Serial.print("] = ( minAngularRes * samples[");
//    Serial.print(i);
//    Serial.print("] ) % 360 = ( ");
//    Serial.print(minAngularRes);
//    Serial.print(" deg/pulse * ");
//    Serial.print(samples[i]);
//    Serial.print(" pulses) % 360 = ");
//    Serial.print(angPos[i]);
//    Serial.println(" deg");
}
  
// get displacement of each wheel to get this instant's
// linear and angular displacements
/* Read and zero pulses.
 * Copy and accumulate counts from pulses
 * to the ang. vel. variable and
 * then reset pulses to zero.
 */
void observeSample()
{
  //Serial.println("======Observe Sample======");
  sample = pulseCount;
//  Serial.print("samples[");
//  Serial.print(i);
//  Serial.print("] = pulseCounts[");
//  Serial.print(i);
//  Serial.print("] = ");
//  Serial.print(samples[i]);
//  Serial.println(" pulses);

//  Serial.print("samples[0]: ");
//  Serial.println(samples[0]);

  prevPulses = pulses;
  pulses = sample - prevSample; // P pulses
  
  //angVel = minAngularRes * pulses * sign * pubVelRate; // [deg/s], converted from pulses
//    Serial.print("angVels[");
//    Serial.print(i);
//    Serial.print("] = minAngularRes * pulses[");
//    Serial.print(i);
//    Serial.print("] * signs[");
//    Serial.print(i);
//    Serial.print("] * pubVelRate = ");
//    Serial.print(minAngularRes);
//    Serial.print(" deg/pulse * ");
//    Serial.print(pulses[i]);
//    Serial.print(" pulses * ");
//    Serial.print(signs[i]);
//    Serial.print(" * ");
//    Serial.print(pubVelRate);
//    Serial.print(" Hz = ");
//    Serial.print(angVels[i]);
//    Serial.println(" deg/s");
  
  prevSample = sample;
}

void locateTarget()
{
  long err, dAngPos, P, I, D;

//  Serial.print("======Locate Target (");
//  Serial.print(setPos);
//  Serial.println(" deg)======");

  computePosErr();

  err = (long) round( posErr / degsPerRad * 256 ); // [rad]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
//  Serial.print("errs[");
//  Serial.print(i);
//  Serial.print("] = -posErrs[");
//  Serial.print(i);
//  Serial.print("] / degsPerRad = ");
//  Serial.print(-posErrs[i]);
//  Serial.print(" deg / ");
//  Serial.print(degsPerRad);
//  Serial.print(" deg/rad = ");
//  Serial.print(errs[i] / 256.0);
//  Serial.println(" rad");
  
  P = (long) round( kp * err * pubVelRate ); // P(t_k) = K(by_{sp}(t_k) - y(t_k))
//  Serial.print("P[");
//  Serial.print(i);
//  Serial.print("] = kp[0][");
//  Serial.print(i);
//  Serial.print("] * errs[");
//  Serial.print(i);
//  Serial.print("] * pubVelRate = ");
//  Serial.print(kp[0][i]);
//  Serial.print(" rad/s * ");
//  Serial.print(errs[i] / 256.0);
//  Serial.print(" rad * ");
//  Serial.print(pubVelRate);
//  Serial.print(" Hz = ");
//  Serial.print(P[i] / 256.0);
//  Serial.println(" rad/s");

//    if(abs(errs[i]) < threshIntegral)
//    {
//      I[i] += (long) round( ki * errs[i] );
//
//      I[i] = clip(I[i], maxOutVal, -maxOutVal);
//    }
//    else
//      I[i] = 0;
//
  dAngPos = (long) round( ( angPos - prevAngPos ) / degsPerRad * 256 );
//  Serial.print("dAngPos[");
//  Serial.print(i);
//  Serial.print("] = ( angPos[");
//  Serial.print(i);
//  Serial.print("] - prevAngPos[");
//  Serial.print(i);
//  Serial.print("] ) / degsPerRad = ( ");
//  Serial.print(angPos[i]);
//  Serial.print(" deg - ");
//  Serial.print(prevAngPos[i]);
//  Serial.print(" deg ) / ");
//  Serial.print(degsPerRad);
//  Serial.print(" deg/rad = ");
//  Serial.print(dAngPos[i] / 256.0);
//  Serial.println(" rad");
  
  D = (long) round( kd * dAngPos * pubVelRate ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
//  Serial.print("D[");
//  Serial.print(i);
//  Serial.print("] = kd[0][");
//  Serial.print(i);
//  Serial.print("] * dAngPos[");
//  Serial.print(i);
//  Serial.print("] * pubVelRate = ");
//  Serial.print(kd[0][i]);
//  Serial.print(" rad * ");
//  Serial.print(dAngPos[i] / 256.0);
//  Serial.print(" rad * ");
//  Serial.print(pubVelRate);
//  Serial.print(" Hz = ");
//  Serial.print(D[i] / 256.0);
//  Serial.println(" rad/s");

  // ADD: if vel control, output setAngVel to vel controller
  setAngVel = (int) round( ( P + D ) * degsPerRad  / 256.0 ); // [deg/s], + I[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
//  Serial.print("setAngVels[");
//  Serial.print(i);
//  Serial.print("] = ( P[");
//  Serial.print(i);
//  Serial.print("] + D[");
//  Serial.print(i);
//  Serial.print("] ) * degsPerRad / 256.0 = ( ");
//  Serial.print(P[i] / 256.0);
//  Serial.print(" rad/s + ");
//  Serial.print(D[i] / 256.0);
//  Serial.print(" rad/s ) * ");
//  Serial.print(degsPerRad);
//  Serial.print(" deg/rad = ");
//  Serial.print(setAngVels[i]);
//  Serial.println(" deg/s");
  
  prevAngPos = angPos; // maintain history of previous measured rotVel

  setAngVel = clip(setAngVel, topAngVel, -topAngVel); // [deg/s]
//  Serial.print("setAngVels[");
//  Serial.print(i);
//  Serial.print("] = ");
//  Serial.print(setAngVels[i]);
//  Serial.println(" deg/s");
}

void computePosErr()
{
  //Serial.println("===Compute Position Error===");

  int pe1, pe2, // pos errors
    b = 1; // set point weight
    
  float H = 1.0; // / (1 + timeConst); // sensor

  pe1 = (int) round( b * H * setPos - angPos );
//  Serial.print("pe1 = angPos[1] - setPos[1] = ");
//  Serial.print(angPos[1]);
//  Serial.print(" deg - ");
//  Serial.print(setPos[1]);
//  Serial.print(" deg = ");
//  Serial.print(pe1);
//  Serial.println(" deg");

  //Serial.print("pe2 = pe1 ");
  if(pe1 > 0)
  {
    pe2 = pe1 - 360; // [deg]
    //Serial.print("- ");
  }
  else
  {
    pe2 = pe1 + 360;
    //Serial.print("+ ");
  }

//  Serial.print("360 deg = ");
//  Serial.print(pe2);
//  Serial.println(" deg");

  posErr = abs(pe1) > abs(pe2) ? pe2 : pe1;
//  if(abs(pe1) > abs(pe2))
//    posErr = pe2;
//  else
//    posErr = pe1;

//  Serial.print("posErrs[1] = ");
//  Serial.print(posErrs[1]);
//  Serial.println(" deg");

  if(pulses !=0 && prevPulses != 0) // check that mtr is moving before checking if decelerating
  {
    // assume decelerating if passed target:
    if(prevPosErr <= 0 && posErr > 0) 
    {
      sign = -1; // continue in same direc until at rest
      
      decelerating = true;
    }
    else if(prevPosErr >= 0 && posErr < 0)
    {
      sign = 1; // continue in same direc until at rest
      
      decelerating = true;
    }
  }

  prevPosErr = posErr;
}

void mtrCmd()
{  
  if(eStop) emergencyStop();
  
  else 
  {
    // if only pos ctrl:
    cmdPrcntMtrVltge = map(cmdAngVel, -topAngVel, topAngVel, -100, 100);
  
    modulatePulseWidth(cmdPrcntMtrVltge);
  }
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
int clip(int a, int maximum, int minimum)
{
  //Serial.print("Computed val: ");
  //Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  //Serial.print(", Clipped val: ");
  //Serial.println(a);

  return a;
}

void emergencyStop()
{
  digitalWrite(mEnablePin, LOW);
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidth(int signedVal) // take signed value, b/t -100 and 100
{
  setSpeedometerSign(signedVal); // set sign variable used by speedometer code

  setHBridgeDirectionBit(signedVal);
  
  pubMtrCmd = getPWMValueFromEntryTable(abs(signedVal)); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  
  analogWrite(mEnablePin, pubMtrCmd); // generate variable pulse-width output
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(!decelerating)
  {
    // assume sign unchanged if signedVal=cmdPrcntMtrVltge=0
    if(signedVal < 0) // {motor direction of rotation} = backward
      sign = -1;
    else if(signedVal > 0)
      sign = 1; // {motor direction of rotation} = {forward | resting}
    //else
      //Serial.println("Invalid command issued to PWM code!\n");
  }
//  Serial.print("M");
//  Serial.print(mid + 1);
//  Serial.print(" speedometer sign: ");
//  Serial.println(signs[mid]);
}

void setHBridgeDirectionBit(int signedVal)
{
  // definitely works for indep mode but may need to change for diff mode
  if(signedVal < 0) // {motor direction of rotation} = CW
    digitalWrite(mSigPin, HIGH); // M1: CW = LOW; M2: CW = HIGH
  else if(signedVal >= 0) // {motor direction of rotation} = {CCW | resting}
    digitalWrite(mSigPin, LOW);
  //else
    //Serial.println("Invalid command issued to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int magnitude)
{
  if(magnitude == 0)
    return magnitude;
  else if(pulses == 0 || prevPulses == 0)
    return map(magnitude, 0, 100, minStaticMtrCmd, 255);
  else
    return map(magnitude, 0, 100, minKineticMtrCmd, 255); // cruise outputs
}
