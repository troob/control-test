/*
 * Control Test
 * User can choose differential or independent drive.
 * If independent, command to turn wheel to position, speed, or acceleration.
 * If differential, command to Drive or Turn to given position (w/ speedLimit)
 * Command to Drive or Turn at given velocity or accel.
 */

#include <math.h>

//======Advisor======
//===arbitration struct===
typedef struct layer LAYER; // C struct for subsumption task output

struct layer
{
  volatile int cmd, // assertion command
    arg, // assertion argument
    flag; // subsumption flag (instead of layer state?)
};

LAYER user,
  halt; // default

const int job1Size = 2;

LAYER *job1[job1Size] = { &user, &halt };

LAYER *thisLayer = &halt;

LAYER **job;

int jobSize, // number of tasks in priority list
  arbitrate; // global flag to enable subsumption

volatile byte userEnable,
  setPosActive,
  setVelActive,
  haltBot;

volatile byte mtrEnable[] = { 0, 0 };

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoders======
const byte esPins[] = 
{
  3, // encoder signal 1 pin
  7 // encoder signal 2 pin
};

const byte numEncoders = 2,
  pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

// values change in callback methods:
volatile int velPulseCounts[numEncoders],
  angPos[numEncoders], // [deg]
  prevAngPos[numEncoders], // [deg]
  setPos[numEncoders], // [deg]
  angVels[numEncoders], // [deg/(1/pubVelRate)s]
  prevAngVels[numEncoders];
  
volatile long pulseCounts[numEncoders],
  setVels[numEncoders]; // [deg/(1/pubVelRate)s]
  
//======Motor Driver======
const byte mSigPins[] = { 8, 9 },
  mEnablePins[] = { 5, 6 };
  
//======Mobile Platform (DFRobot Turtle)======
int wheelDiam = 64, // [mm]
  botDiam = 138; // [mm] (i.e. wheel base or distance between wheel centers
  
//======Circle======
float piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

//======Controller======
const int numMtrs = 2;

int sensorsTmrCtr,
  maxOutVal,
  pubVelRate = 10, // [Hz]
  minLinearRes, // [mm/pulse]
  posDeadZone; // [deg]

volatile double kp[numMtrs], ki[numMtrs], kd[numMtrs];

volatile int topVel,
  setVel;

volatile int pulses[numMtrs],
  pubMtrCmds[numMtrs],
  signs[numMtrs],
  posErrs[numMtrs];
  
volatile long samples[numMtrs],
  prevSamples[numMtrs],
  mtrOut[numMtrs],
  instance, // loop count
  prevInstance;

void setup() 
{
  initSystem();

  initBehaviors();

  initSensorsTimer(); 
}

int initSystem()
{
  initNode("ControlTest");

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
}

void displayCommands()
{
  Serial.println("======User Commands======");
  Serial.println("s: stop moving (i.e. halt robot)");
  Serial.println("mx: alternate motor x status (enable/disable)");
  Serial.println("apx: set absolute position to x deg");
  Serial.println("rpx: set relative position to x deg");
  Serial.println("avx: set absolute velocity to x deg/s");
  Serial.println("rvx: set relative velocity to x deg/s");
  Serial.println("kpx: set proportional gain(s) of enabled controller(s) to x units");
  Serial.println("kix: set integral gain(s) of enabled controller(s) to x units");
  Serial.println("kdx: set derivative gain(s) of enabled controller(s) to x units");
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPins[0]), encoder1Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(esPins[1]), encoder2Callback, CHANGE);
}

void initPublishers()
{
  /* Start Motor Channels */
  for(int i=0; i < numMtrs; i++)
  {
    pinMode(mEnablePins[i], OUTPUT);
    pinMode(mSigPins[i], OUTPUT);
  }
}

void initBehaviors()
{
  initVars();

  setParams();
  
  initJob1(); // set job1 as active job at startup time
}

void initVars()
{
  for(int i=0; i < numMtrs; i++)
  {
    velPulseCounts[i] = 0;
    
    pulseCounts[i] = 0; // left and right distances
  
    pubMtrCmds[i] = 0;
  
    signs[i] = 1;
  
    mtrOut[i] = 0;
  
    angPos[i] = 0;

    setPos[i] = 0;
  
    prevAngPos[i] = 0;

    angVels[i] = 0;

    prevAngVels[i] = 0;

    setVels[i] = 0;

    posErrs[i] = 0;

    samples[i] = 0;

    prevSamples[i] = 0;

    pulses[i] = 0;
  }

  topVel = 36; // [deg/(1/pubVelRate)s]
  setVel = topVel; // [deg/(1/pubVelRate)s]

  instance = 0;
  prevInstance = 0;
}

void setParams()
{
  double setKp[][] = {
      { 5.000, 5.000 }, // pos->vel
      { 0.000, 0.000 }, // vel->pwm
    },
    setKi[][] = {
      { 0.000, 0.000 },
      { 0.000, 0.000 },
    },
    setKd[][] = {
      { 0.300, 0.300 };
      { 0.000, 0.000 },
    };
  
  for(int i=0; i < numMtrs; i++)
  {
    String id = String(i+1);
    String controller = "m" + id;

    setPIDGains(setKp[0][i], setKi[0][i], setKd[0][i], controller); // pos
    setPIDGains(setKp[1][i], setKi[1][i], setKd[1][i], controller); // vel
  }

  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;
  
  minLinearRes = (int) round( piApprox * wheelDiam / pulsesPerRev ); // r_{min} [mm/pulse]

  userEnable = 0;
  
  setPosActive = 0;

  setVelActive = 0;

  arbitrate = 1;

  haltBot = 1;

  posDeadZone = 18; // [deg], CHANGE: tune based on resolution
}

void setPIDGains(float pg, float ig, float dg, String cid)
{
  if(pg < 0 || ig < 0 || dg < 0) return;

  Serial.print("Controller ");
  Serial.print(cid);
  Serial.print(" got [kp ki kd]: [");
  if(cid == "m1")
  {
    kp[0] = pg;

    ki[0] = ig;

    kd[0] = dg;
    
    Serial.print(kp[0], 3);
    Serial.print(" ");
    Serial.print(ki[0], 3);
    Serial.print(" ");
    Serial.print(kd[0], 3);
  }
  else if(cid == "m2")
  {
    kp[1] = pg;

    ki[1] = ig;

    kd[1] = dg;
    
    Serial.print(kp[1], 3);
    Serial.print(" ");
    Serial.print(ki[1], 3);
    Serial.print(" ");
    Serial.print(kd[1], 3);
  }
//  else if(cid == "heading")
//  {
//    
//  }
//  else if(cid == "displacement")
//  {
//    
//  }

  Serial.println("]");
}

int initJob1() // make job1 the active job
{
  job = &job1[0]; // global job priority list pointer

  jobSize = job1Size; // no. tasks in job1 list

  return 0;
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

void loop() {}

//======Interrupt Service Routines======
void encoder1Callback()
{
  velPulseCounts[0]++;
  
  if(signs[0] == 1)
    pulseCounts[0]++;
  else
    pulseCounts[0]--;

//  Serial.print("Pulse Count ");
//  Serial.print(1);
//  Serial.print(": ");
//  Serial.print(pulseCounts[0]);
//  Serial.println("\n");
}

void encoder2Callback()
{
  velPulseCounts[1]++;
  
  if(signs[1] == 1)
    pulseCounts[1]++;
  else
    pulseCounts[1]--;

//  Serial.print("Pulse Count ");
//  Serial.print(2);
//  Serial.print(": ");
//  Serial.print(pulseCounts[1]);
//  Serial.println("\n");
}

ISR(TIMER1_OVF_vect)
{
  readUserInput();

  observeSample();

  speedometer();

  userTask();

  arbitrator();
  
  prevInstance = instance;

  instance++;
}

void readUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "s")
    {
      Serial.println("stop");

      userEnable = 0;

      haltBot = 1;
    }
    else if(inputString.substring(0,1) == "m")
    {
      int mid = inputString.substring(1,inputString.length()).toInt();

      if(mid > 0 && mid <= numMtrs)
      {
        mtrEnable[mid-1] = !mtrEnable[mid-1];
        Serial.print("M");
        Serial.print(mid);
        Serial.print(": ");
        Serial.println(mtrEnable[mid-1]);
      }
      else
        Serial.println("Invalid motor ID");
    }
    else if(inputString.substring(0,2) == "ap") // absolute position, given in deg
    {
      Serial.print("Set Pos (deg): ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          setPos[i] = inputString.substring(2, inputString.length()).toInt() % 360; // get string after 'ap'
        
        Serial.print(setPos[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      userEnable = 1;

      setPosActive = 1;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "rp") // relative position, given in deg
    {
      Serial.print("Set Pos (deg): ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          setPos[i] = ( angPos[i] + inputString.substring(2, inputString.length()).toInt() ) % 360;
        
        Serial.print(setPos[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      userEnable = 1;

      setPosActive = 1;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "av") // absolute velocity, given in deg/s
    {
      Serial.print("Set Vel (deg/s): ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          setVels[i] = (int) round( inputString.substring(2, inputString.length()).toInt() / (float) pubVelRate ); // [deg/(1/pubVelRate)s]
        
        Serial.print(setVels[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      userEnable = 1;

      setVelActive = 1;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "rv") // relative velocity, given in deg/s
    {
      Serial.print("Set Vel (deg/s): ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          setVels[i] = angVels[i] + (int) round( inputString.substring(2, inputString.length()).toInt() / (float) pubVelRate ); // [deg/(1/pubVelRate)s]
        
        Serial.print(setVels[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      userEnable = 1;

      setVelActive = 1;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "kp")
    {
      Serial.print("kp: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          kp[i] = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kp'
        
        Serial.print(kp[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }
    else if(inputString.substring(0,2) == "ki")
    {
      Serial.print("ki: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          ki[i] = inputString.substring(2, inputString.length()).toFloat(); // get string after 'ki'
        
        Serial.print(ki[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }
    else if(inputString.substring(0,2) == "kd")
    {
      Serial.print("kd: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          kd[i] = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kd'
        
        Serial.print(kd[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void displayPIDGains()
{
  Serial.println("\tk_p\tk_i\tk_d");
  for(int i=0; i < numMtrs; i++)
  {
    Serial.print("m");
    Serial.print(i+1);
    Serial.print(" pos\t");
    Serial.print(kp[0][i], 3);
    Serial.print("\t");
    Serial.print(ki[0][i], 3);
    Serial.print("\t");
    Serial.println(kd[0][i], 3);
    Serial.print("m");
    Serial.print(i+1);
    Serial.print(" vel\t");
    Serial.print(kp[1][i], 3);
    Serial.print("\t");
    Serial.print(ki[1][i], 3);
    Serial.print("\t");
    Serial.println(kd[1][i], 3);
  }
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

// get displacement of each wheel to get this instant's
// linear and angular displacements
void observeSample()
{
  int i;

  for(i=0; i < numMtrs; i++)
    samples[i] = pulseCounts[i];

//  Serial.print("samples[0]: ");
//  Serial.println(samples[0]);

  for(i=0; i < numMtrs; i++)
    pulses[i] = samples[i] - prevSamples[i]; // P pulses
  
  for(i=0; i < numMtrs; i++)
    prevSamples[i] = samples[i];
}

/* Read and zero velPulseCounts.
 * Copy and accumulate counts from velPulseCounts
 * to the rot. vel. variable and
 * then reset velPulseCounts to zero.
 */
void speedometer()
{
  Serial.println("======Read Velocity======");
  for(int i=0; i < numMtrs; i++)
  {
    if(mtrEnable[i])
    {
      angVels[i] = minAngularRes * velPulseCounts[i] * signs[i]; // [deg/(1/pubVelRate)s], copy and accumulate counts from velPulseCount to rotVel
      
      velPulseCounts[i] = 0; // reset velPulseCount to zero
    
      Serial.print(angVels[i]);
      Serial.print(" deg/");
      Serial.print(1.0 / pubVelRate);
      Serial.println("s");
    }
  }
}

void userTask()
{
  extern LAYER user;

  readPosition();

  if(userEnable)
  {
    if(setPosActive == 0)
      setVels[1] = 0;
    else
    {
      locateTarget();
      
      if(abs(posErrs[1]) > posDeadZone)
      {
        Serial.print(abs(posErrs[1]));
        Serial.println(" deg > posDeadZone");
        Serial.print("===Rotate to ");
        Serial.print(setPos[1]);
        Serial.println(" deg===");
        
        if(posErrs[1] > 0)
          if(posErrs[1] > 180)
            setVels[1] = -abs(setVels[1]); // CCW
          else
            setVels[1] = abs(setVels[1]); // CW
        else
          if(posErrs[1] < -180)
            setVels[1] = abs(setVels[1]); // CW
          else
            setVels[1] = -abs(setVels[1]); // CCW
      }
      else // found target
      {
        haltBot = 1; // needed b/c PID control shift down to 0 takes too long b/c not tuned
  
        setPosActive = 0; // signal that target is acquired
      }
    }
  }
}

void readPosition()
{
  Serial.println("======Read Position======");

  for(int i=0; i < numMtrs; i++)
  {
    if(mtrEnable[i])
    {
      angPos[i] = (int) round( minAngularRes * samples[i] ) % 360; // [deg]

      Serial.print("angPos[");
      Serial.print(i);
      Serial.print("]= ( minAngularRes * samples[");
      Serial.print(i);
      Serial.print("] ) % 360 = ( ");
      Serial.print(minAngularRes);
      Serial.print(" deg/pulse * ");
      Serial.print(samples[i]);
      Serial.print(" pulses) % 360 = ");
      Serial.print(angPos[i]);
      Serial.println(" deg");
    }
  }
  
  //Serial.println();
}

void locateTarget()
{
  int pe1, pe2, // heading errors
    b = 1; // set point weight
  
  Serial.print("======Locate Target (");
  Serial.print(setPos[1]);
  Serial.println(" deg)======");

  pe1 = angPos[1] - b * setPos[1];
  Serial.print("pe1 = angPos[1] - setPos[1] = ");
  Serial.print(angPos[1]);
  Serial.print(" deg - ");
  Serial.print(setPos[1]);
  Serial.print(" deg = ");
  Serial.print(pe1);
  Serial.println(" deg");

  Serial.print("pe2 = pe1 ");
  if(pe1 > 0)
  {
    pe2 = pe1 - 360; // [deg]
    Serial.print("- ");
  }
  else
  {
    pe2 = pe1 + 360;
    Serial.print("+ ");
  }

  Serial.print("360 deg = ");
  Serial.print(pe2);
  Serial.println(" deg");

  if(abs(pe1) > abs(pe2))
    posErrs[1] = pe2;
  else
    posErrs[1] = pe1;

  Serial.print("posErrs[1] = ");
  Serial.print(posErrs[1]);
  Serial.println(" deg");
}

void arbitrator()
{
  if(haltBot)
    stopMoving();
  else
    computeControlSignals(); // PID
}

void stopMoving()
{
  for(int i=0; i < numMtrs; i++)
    digitalWrite(mEnablePins[i], LOW);
}

void computeControlSignals()
{
  int aOutputs[numMtrs];
  
  displayPIDGains();
  
  controlPos();

  controlVel();

  // Set analog outputs:
  for(int i=0; i < 2; i++)
    aOutputs[i] = (int) round( mtrOut[i] / 256.0 );
  
  modulatePulseWidths(aOutputs); // Note: divide by 256 and earlier multiply by 256 b/c earlier operation in fixed point integer
}

/* Basic behavior: generate error signal 
 * based on difference b/t measured rotVel and
 * requested rotVel for the wheel.
 * Use of error signal: Increase or decrease speed of motor
 * to force measured to equal requested rotVel
 * Input to PID controller: Requested rotVel "vel,"
 * which is input rotVel expressed as encoder pulses
 * per 1/pubVelRate second.
 */
void controlPos()
{
  long errs[numMtrs],
    P[numMtrs],
    I[numMtrs],
    D[numMtrs];
  
  int b = 1; // set point weight

  for(int i=0; i < numMtrs; i++)
  {
    if(mtrEnable[i])
    {
      Serial.print("setPos[");
      Serial.print(i);
      Serial.print("] (deg): ");
      Serial.print(setPos[i]);
      Serial.print(", angPos[");
      Serial.print(i);
      Serial.print("] (deg): ");
      Serial.println(angPos[i]);
      
      errs[i] = (long) round( -posErrs[i] / degsPerRad * 256 ); // [rad]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
      Serial.print("errs[");
      Serial.print(i);
      Serial.print("] = -posErrs[");
      Serial.print(i);
      Serial.print("] / degsPerRad * 256 = ");
      Serial.print(-posErrs[i]);
      Serial.print(" deg / ");
      Serial.print(degsPerRad);
      Serial.print(" deg/rad * 256 = ");
      Serial.print( -posErrs[i] / degsPerRad );
      Serial.print(" * 256 = ");
      Serial.print(errs[i]);
      Serial.println(" rad*256");
      
      P[i] = (long) round( kp[i] * errs[i] ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))
  
  //    if(abs(errs[i]) < threshIntegral)
  //    {
  //      I[i] += (long) round( ki * errs[i] );
  //
  //      I[i] = clip(I[i], maxOutVal, -maxOutVal);
  //    }
  //    else
  //      I[i] = 0;
  //
      D[i] = (long) round( kd[i] * ( ( angPos[i] - prevAngPos[i] ) / degsPerRad * 256 ) ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
  
      setVels[i] = (long) round( ( P[i] + D[i] ) * degsPerRad / 256.0 ); // + I[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
  
      Serial.print("setVels[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(setVels[i]);
      
      prevAngPos[i] = angPos[i]; // maintain history of previous measured rotVel
  
      setVels[i] = clip(setVels[i], topVel, -topVel); // [deg/(1/pubVelRate)s]
    }
  }
}

/* Basic behavior: generate error signal 
 * based on difference b/t measured rotVel and
 * requested rotVel for the wheel.
 * Use of error signal: Increase or decrease speed of motor
 * to force measured to equal requested rotVel
 * Input to PID controller: Requested rotVel "vel,"
 * which is input rotVel expressed as encoder pulses
 * per 1/pubVelRate second.
 */
void controlVel()
{
  long errs[numMtrs],
    P[numMtrs],
    I[numMtrs],
    D[numMtrs];
  
  int b = 1; // set point weight or filter

  for(int i=0; i < numMtrs; i++)
  {
    Serial.print("setVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(setVels[i]);
    Serial.print(", angVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(angVels[i]);
    
    errs[i] = (long) round( ( b * setVels[i] - angVels[i] ) / degsPerRad * 256 ); // [rad/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
    
    P[i] = (long) round( kp[i] * errs[i] ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))

//    if(abs(errs[i]) < threshIntegral)
//    {
//      I[i] += (int) round( ki * errs[i] );
//
//      I[i] = clip(I[i], maxOutVal, -maxOutVal);
//    }
//    else
//      I[i] = 0;

    D[i] = (long) round( kd[i] * ( ( angVels[i] - prevAngVels[i] ) / degsPerRad * 256 ) ); // large when amount of change requested by PID controller is large, and small as signal approaches zero

    mtrOut[i] += P[i] + D[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel

    Serial.print("mtrOut[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(mtrOut[i]);
    
    prevAngVels[i] = angVels[i]; // maintain history of previous measured rotVel

    mtrOut[i] = clip(mtrOut[i], maxOutVal, -maxOutVal); // accumulator
  }
}

/* motorOut must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
long clip(long a, int maximum, int minimum)
{ 
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  return a;
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
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  int i;
  
  for(i=0; i < numMtrs; i++)
  {
    setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code

    setHBridgeDirectionBit(i, signedVals[i]);
  
    pubMtrCmds[i] = getPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  }
  
  for(i=0; i < numMtrs; i++)
    analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(signedVal < 0) // {motor direction of rotation} = backward
    signs[mid] = -1;
  else if(signedVal >= 0)
    signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");

//  Serial.print("M");
//  Serial.print(mid + 1);
//  Serial.print(" speedometer sign: ");
//  Serial.println(signs[mid]);
}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  if(signedVal < 0) // {motor direction of rotation} = backward
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {forward | resting}
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int mid, int magnitude)
{
  Serial.print("magnitude: ");
  Serial.println(magnitude);
  
  return map(magnitude, 0, 100, 60, 255); // cruise outputs
}
