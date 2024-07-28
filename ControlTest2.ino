/* Warning: Does not work 
 *  b/c serial communication is blocked
 *  for unknown reason.
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
  diffDrive,
  eStop;

volatile byte mtrEnable[] = { 0, 0 };

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoders======
// Leonardo
//const byte esPins[] = 
//{
//  3, // encoder signal 1 pin
//  7 // encoder signal 2 pin
//};

// Mega
const byte esPins[] = 
{
  2, // encoder signal 1 pin
  3 // encoder signal 2 pin
};

const byte numEncoders = 2,
  pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

// values change in callback methods:
volatile int velPulseCounts[numEncoders],
  angPos[numEncoders], // [deg]
  prevAngPos[numEncoders], // [deg]
  setPos[numEncoders], // [deg]
  angVels[numEncoders], // [deg/s]
  prevAngVels[numEncoders];
  
volatile long pulseCounts[numEncoders],
  setAngVels[numEncoders]; // [deg/s]
  
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
const int numMtrs = 2,
  numCtrlrs = 2;

int sensorsTmrCtr,
  maxOutVal,
  pubVelRate = 10, // [Hz]
  minLinearRes, // [mm/pulse]
  posDeadZone; // [deg]

float timeConst; // [s]

volatile double kp[numCtrlrs][numMtrs], ki[numCtrlrs][numMtrs], kd[numCtrlrs][numMtrs];

volatile int topAngVel,
  setVel;

volatile int pulses[numMtrs],
  pubMtrCmds[numMtrs],
  signs[numMtrs],
  posErrs[numMtrs],
  botVel;
  
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
  initNode("ControlTest2");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(115200);

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
  Serial.println("pkpx: set proportional gain(s) of enabled controller(s) to x units");
  Serial.println("pkix: set integral gain(s) of enabled controller(s) to x units");
  Serial.println("pkdx: set derivative gain(s) of enabled controller(s) to x units");
  Serial.println("vkpx: set proportional gain(s) of enabled controller(s) to x units");
  Serial.println("vkix: set integral gain(s) of enabled controller(s) to x units");
  Serial.println("vkdx: set derivative gain(s) of enabled controller(s) to x units");
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  //pinMode(esPins[0], INPUT_PULLUP);
  //pinMode(esPins[1], INPUT_PULLUP);

  attachInterrupt(0, encoder1Callback, CHANGE);
  attachInterrupt(5, encoder2Callback, CHANGE);
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
  botVel = 0;
  topAngVel = 360; // [deg/s]

  // only concerned w/ m2 for now
  setAngVels[0] = 0; // [deg/s]
  setAngVels[1] = topAngVel; // [deg/s]
  
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

    posErrs[i] = 0;

    samples[i] = 0;

    prevSamples[i] = 0;

    pulses[i] = 0;
  }

  instance = 0;
  prevInstance = 0;
}

void setParams()
{
  double setKp[][numMtrs] = {
      { 1.000, 1.000 }, // pos->vel
      { 1.000, 1.000 } // vel->pwm
    },
    setKi[][numMtrs] = {
      { 0.000, 0.000 },
      { 0.000, 0.000 }
    },
    setKd[][numMtrs] = {
      { 0.000, 0.000 },
      { 0.000, 0.000 }
    };
  
  for(int i=0; i < numMtrs; i++)
  {
    String id = String(i+1),
      posCtrl = "m" + id + "pos",
      velCtrl = "m" + id + "vel";

    setPIDGains(setKp[0][i], setKi[0][i], setKd[0][i], posCtrl); // pos
    setPIDGains(setKp[1][i], setKi[1][i], setKd[1][i], velCtrl); // vel
  }

  timeConst = -0.6;

  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;
  
  minLinearRes = (int) round( piApprox * wheelDiam / pulsesPerRev ); // r_{min} [mm/pulse]

  userEnable = 0;
  
  setPosActive = 0;

  setVelActive = 0;

  arbitrate = 1;

  diffDrive = 0;

  eStop = 1;

  posDeadZone = 18; // [deg], CHANGE: tune based on resolution
}

void setPIDGains(float pg, float ig, float dg, String cid)
{
  if(pg < 0 || ig < 0 || dg < 0) return;

  Serial.print("Controller ");
  Serial.print(cid);
  Serial.print(" got [kp ki kd]: [");
  if(cid == "m1pos")
  {
    kp[0][0] = pg;

    ki[0][0] = ig;

    kd[0][0] = dg;
    
    Serial.print(kp[0][0], 3);
    Serial.print(" ");
    Serial.print(ki[0][0], 3);
    Serial.print(" ");
    Serial.print(kd[0][0], 3);
  }
  else if(cid == "m1vel")
  {
    kp[1][0] = pg;

    ki[1][0] = ig;

    kd[1][0] = dg;
    
    Serial.print(kp[1][0], 3);
    Serial.print(" ");
    Serial.print(ki[1][0], 3);
    Serial.print(" ");
    Serial.print(kd[1][0], 3);
  }
  else if(cid == "m2pos")
  {
    kp[0][1] = pg;

    ki[0][1] = ig;

    kd[0][1] = dg;
    
    Serial.print(kp[0][1], 3);
    Serial.print(" ");
    Serial.print(ki[0][1], 3);
    Serial.print(" ");
    Serial.print(kd[0][1], 3);
  }
  else if(cid == "m2vel")
  {
    kp[1][1] = pg;

    ki[1][1] = ig;

    kd[1][1] = dg;
    
    Serial.print(kp[1][1], 3);
    Serial.print(" ");
    Serial.print(ki[1][1], 3);
    Serial.print(" ");
    Serial.print(kd[1][1], 3);
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
//  velPulseCounts[0]++;
//  Serial.print("Vel Pulse Count ");
//  Serial.print(1);
//  Serial.print(": ");
//  Serial.print(velPulseCounts[0]);
//  Serial.println("\n");
  
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
//  velPulseCounts[1]++;
//  Serial.print("Vel Pulse Count ");
//  Serial.print(2);
//  Serial.print(": ");
//  Serial.print(velPulseCounts[1]);
//  Serial.println("\n");
  
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

      eStop = 1;
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

      eStop = 0;
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

      eStop = 0;
    }
    else if(inputString.substring(0,2) == "av") // absolute velocity, given in deg/s
    {
      Serial.print("Set Ang Vel (deg/s): ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          setAngVels[i] = inputString.substring(2, inputString.length()).toInt(); // [deg/s]
        
        Serial.print(setAngVels[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      userEnable = 1;

      setVelActive = 1;

      eStop = 0;
    }
    else if(inputString.substring(0,2) == "rv") // relative velocity, given in deg/s
    {
      Serial.print("Set Ang Vel (deg/s): ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          setAngVels[i] = angVels[i] + inputString.substring(2, inputString.length()).toInt(); // [deg/s]
        
        Serial.print(setAngVels[i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      userEnable = 1;

      setVelActive = 1;

      eStop = 0;
    }
    else if(inputString.substring(0,3) == "pkp") // [rad/s]
    {
      Serial.print("kp: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          kp[0][i] = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kp'
        
        Serial.print(kp[0][i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }
    else if(inputString.substring(0,3) == "pki") // [rad/s^2]
    {
      Serial.print("ki: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          ki[0][i] = inputString.substring(3, inputString.length()).toFloat(); // get string after 'ki'
        
        Serial.print(ki[0][i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }
    else if(inputString.substring(0,3) == "pkd") // [rad]
    {
      Serial.print("kd: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          kd[0][i] = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kd'
        
        Serial.print(kd[0][i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }
    else if(inputString.substring(0,3) == "vkp") // [%]
    {
      Serial.print("kp: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          kp[1][i] = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kp'
        
        Serial.print(kp[1][i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }
    else if(inputString.substring(0,3) == "vki") // [%/s]
    {
      Serial.print("ki: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          ki[1][i] = inputString.substring(3, inputString.length()).toFloat(); // get string after 'ki'
        
        Serial.print(ki[1][i]);
        Serial.print(" ");
      }
      Serial.println("\n");

      displayPIDGains();
    }
    else if(inputString.substring(0,3) == "vkd") // [%.s]
    {
      Serial.print("kd: ");
      for(int i=0; i < numMtrs; i++)
      {
        if(mtrEnable[i])
          kd[1][i] = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kd'
        
        Serial.print(kd[1][i]);
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
/* Read and zero pulses.
 * Copy and accumulate counts from pulses
 * to the ang. vel. variable and
 * then reset pulses to zero.
 */
void observeSample()
{
  int i;

  Serial.println("======Observe Sample======");
  for(i=0; i < numMtrs; i++)
  {
    samples[i] = pulseCounts[i];
    Serial.print("samples[");
    Serial.print(i);
    Serial.print("] = pulseCounts[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(samples[i]);
    Serial.println(" pulses");

    angPos[i] = (int) round( minAngularRes * samples[i] ) % 360; // [deg]

    Serial.print("angPos[");
    Serial.print(i);
    Serial.print("] = ( minAngularRes * samples[");
    Serial.print(i);
    Serial.print("] ) % 360 = ( ");
    Serial.print(minAngularRes);
    Serial.print(" deg/pulse * ");
    Serial.print(samples[i]);
    Serial.print(" pulses) % 360 = ");
    Serial.print(angPos[i]);
    Serial.println(" deg");
  }

//  Serial.print("samples[0]: ");
//  Serial.println(samples[0]);

  for(i=0; i < numMtrs; i++)
  {
    pulses[i] = samples[i] - prevSamples[i]; // P pulses
  
    angVels[i] = minAngularRes * pulses[i] * signs[i] * pubVelRate; // [deg/s], converted from pulses
    Serial.print("angVels[");
    Serial.print(i);
    Serial.print("] = minAngularRes * pulses[");
    Serial.print(i);
    Serial.print("] * signs[");
    Serial.print(i);
    Serial.print("] * pubVelRate = ");
    Serial.print(minAngularRes);
    Serial.print(" deg/pulse * ");
    Serial.print(pulses[i]);
    Serial.print(" pulses * ");
    Serial.print(signs[i]);
    Serial.print(" * ");
    Serial.print(pubVelRate);
    Serial.print(" Hz = ");
    Serial.print(angVels[i]);
    Serial.println(" deg/s");
  }
  
  for(i=0; i < numMtrs; i++)
    prevSamples[i] = samples[i];
}

/* If independent drive mode:
 *  cmd = set wheel 1 angular velocity
 *  arg = set wheel 2 angular velocity
 */
void userTask()
{
  extern LAYER user;
  
  Serial.print("======User Task-");

  if(userEnable)
  {
    Serial.print("ENABLED");
    Serial.println("======");

    if(diffDrive)
    {
      Serial.println("===Differential Drive===");
      
//      user.cmd = setFwdVel;
//
//      user.arg = setYawVel;
    }
    else
    {
      // only concerned w/ m2 for now
      Serial.println("===Independent Drive===");

      user.cmd = 0; 
      
      if(setPosActive == 0)
      {
        Serial.println("Set Position-INACTIVE");
        user.arg = 0;
      }
      else
      {
        Serial.println("Set Position-ACTIVE");
        locateTarget();

        if(abs(posErrs[1]) > posDeadZone)
        {
          Serial.print(abs(posErrs[1]));
          Serial.println(" deg > posDeadZone");
          Serial.print("===Rotate to ");
          Serial.print(setPos[1]);
          Serial.println(" deg===");

          // for m2, pos rot is CCW b/c +x is right and +y is up
          if(posErrs[1] > 0)
          {
            if(posErrs[1] > 180)
              user.arg = setAngVels[1]; // CCW, if setAngVel>0
            else
              user.arg = -setAngVels[1]; // CW, if setAngVel>0
          }
          else
          {
            if(posErrs[1] < -180)
              user.arg = -setAngVels[1]; // CW, if setAngVel>0
            else
              user.arg = setAngVels[1]; // CCW, if setAngVel>0
          }
        }
        else // found target
        {
          eStop = 1; // needed b/c PID control shift down to 0 takes too long b/c not tuned
    
          setPosActive = 0; // signal that target is acquired
        }
      }
    }
  }
  else
  {
    Serial.print("DISABLED");
    Serial.println("======");

    user.cmd = 0;
    user.arg = 0;
  }

  user.flag = true;
}

void locateTarget()
{
  long errs[numMtrs],
    P[numMtrs],
    dAngPos[numMtrs],
    I[numMtrs],
    D[numMtrs];
    
  Serial.print("======Locate Target (");
  Serial.print(setPos[1]);
  Serial.println(" deg)======");
  
  computePosErr();

  for(int i=0; i < numMtrs; i++)
  {
    if(mtrEnable[i])
    {
//      Serial.print("setPos[");
//      Serial.print(i);
//      Serial.print("] (deg): ");
//      Serial.print(setPos[i]);
//      Serial.print(", angPos[");
//      Serial.print(i);
//      Serial.print("] (deg): ");
//      Serial.println(angPos[i]);
      
      errs[i] = (long) round( -posErrs[i] / degsPerRad * 256 ); // [rad]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
      Serial.print("errs[");
      Serial.print(i);
      Serial.print("] = -posErrs[");
      Serial.print(i);
      Serial.print("] / degsPerRad = ");
      Serial.print(-posErrs[i]);
      Serial.print(" deg / ");
      Serial.print(degsPerRad);
      Serial.print(" deg/rad = ");
      Serial.print(errs[i] / 256.0);
      Serial.println(" rad");
      
      P[i] = (long) round( kp[0][i] * errs[i] * pubVelRate ); // P(t_k) = K(by_{sp}(t_k) - y(t_k))
      Serial.print("P[");
      Serial.print(i);
      Serial.print("] = kp[0][");
      Serial.print(i);
      Serial.print("] * errs[");
      Serial.print(i);
      Serial.print("] * pubVelRate = ");
      Serial.print(kp[0][i]);
      Serial.print(" rad/s * ");
      Serial.print(errs[i] / 256.0);
      Serial.print(" rad * ");
      Serial.print(pubVelRate);
      Serial.print(" Hz = ");
      Serial.print(P[i] / 256.0);
      Serial.println(" rad/s");
  
  //    if(abs(errs[i]) < threshIntegral)
  //    {
  //      I[i] += (long) round( ki * errs[i] );
  //
  //      I[i] = clip(I[i], maxOutVal, -maxOutVal);
  //    }
  //    else
  //      I[i] = 0;
  //
      dAngPos[i] = (long) round( ( angPos[i] - prevAngPos[i] ) / degsPerRad * 256 );
      Serial.print("dAngPos[");
      Serial.print(i);
      Serial.print("] = ( angPos[");
      Serial.print(i);
      Serial.print("] - prevAngPos[");
      Serial.print(i);
      Serial.print("] ) / degsPerRad = ( ");
      Serial.print(angPos[i]);
      Serial.print(" deg - ");
      Serial.print(prevAngPos[i]);
      Serial.print(" deg ) / ");
      Serial.print(degsPerRad);
      Serial.print(" deg/rad = ");
      Serial.print(dAngPos[i] / 256.0);
      Serial.println(" rad");
      
      D[i] = (long) round( kd[0][i] * dAngPos[i] * pubVelRate ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
      Serial.print("D[");
      Serial.print(i);
      Serial.print("] = kd[0][");
      Serial.print(i);
      Serial.print("] * dAngPos[");
      Serial.print(i);
      Serial.print("] * pubVelRate = ");
      Serial.print(kd[0][i]);
      Serial.print(" rad * ");
      Serial.print(dAngPos[i] / 256.0);
      Serial.print(" rad * ");
      Serial.print(pubVelRate);
      Serial.print(" Hz = ");
      Serial.print(D[i] / 256.0);
      Serial.println(" rad/s");
  
      setAngVels[i] = (long) round( ( P[i] + D[i] ) * degsPerRad / 256.0 ); // + I[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
      Serial.print("setAngVels[");
      Serial.print(i);
      Serial.print("] = ( P[");
      Serial.print(i);
      Serial.print("] + D[");
      Serial.print(i);
      Serial.print("] ) * degsPerRad / 256.0 = ( ");
      Serial.print(P[i] / 256.0);
      Serial.print(" rad/s + ");
      Serial.print(D[i] / 256.0);
      Serial.print(" rad/s ) * ");
      Serial.print(degsPerRad);
      Serial.print(" deg/rad = ");
      Serial.print(setAngVels[i]);
      Serial.println(" deg/s");
      
      prevAngPos[i] = angPos[i]; // maintain history of previous measured rotVel
  
      setAngVels[i] = clip(setAngVels[i], topAngVel, -topAngVel); // [deg/s]
      Serial.print("setAngVels[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.print(setAngVels[i]);
      Serial.println(" deg/s");
    }
  }
}

void computePosErr()
{
  Serial.println("===Compute Position Error===");

  int pe1, pe2, // pos errors
    b = 1; // set point weight
    
  float H = 1;// / (1 + timeConst); // sensor

  pe1 = (int) round( angPos[1] - b * H * setPos[1] );
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
  int i = 0;
  
  if(arbitrate)
  {
    for(i=0; i < jobSize - 1; i++)// step through tasks
      if(job[i]->flag) break; // subsume

    thisLayer = job[i]; // global output winner
  }
  
  mtrCmd(thisLayer); // send command to motors; execute, given pointer to winning layer
}

void mtrCmd(LAYER *l)
{
  int aOutputs[numMtrs];

  Serial.print("======Motor Command-");
  
  // set wheel vels based on drive mode
  if(diffDrive)
  {
    Serial.print("Differential");

    botVel = l->cmd;

    setAngVels[0] = l->cmd + l->arg;
    setAngVels[0] = clip(setAngVels[0], topAngVel, -topAngVel); // don't overflow +/- 100% full speed

    setAngVels[1] = l->cmd - l->arg;
    setAngVels[1] = clip(setAngVels[1], topAngVel, -topAngVel); // don't overflow +/- 100% full speed
  }
  else
  {
    Serial.print("Independent");
    
    for(int i=0; i < numMtrs; i++)
    {
      if(i == 0)
        setAngVels[i] = l->cmd; // [deg/0.1s]
      else if(i == 1)
        setAngVels[i] = l->arg; // [deg/0.1s]
        
      setAngVels[i] = clip(setAngVels[i], topAngVel, -topAngVel); // don't overflow +/- 100% full speed
    }
  }
  
  Serial.println(" Drive======");

  displaySetAngVels();
  
  if(eStop) emergencyStop();
  
  else 
  {
    computeMtrCtrlSignals(); // PID

    // Set analog outputs:
    for(int i=0; i < numMtrs; i++)
      aOutputs[i] = (int) round( mtrOut[i] / 256.0 );
    
    modulatePulseWidths(aOutputs); // Note: divide by 256 and earlier multiply by 256 b/c earlier operation in fixed point integer
  }
}

void displaySetAngVels()
{
  Serial.print("\nSet Ang Vels (deg/s): ");
  for(int i=0; i < numMtrs; i++)
  {
    Serial.print(setAngVels[i]);
    Serial.print(" ");
  }
  Serial.println("\n");
}

void emergencyStop()
{
  Serial.println("======Emergency Stop======");
  
  for(int i=0; i < numMtrs; i++)
    digitalWrite(mEnablePins[i], LOW);
}

void computeMtrCtrlSignals()
{
  long errs[numMtrs],
    P[numMtrs],
    dAngVels[numMtrs],
    I[numMtrs],
    D[numMtrs];
  
  int b = 1; // set point weight or filter

  Serial.println("===Compute Motor Control Signals===");

  for(int i=0; i < numMtrs; i++)
  {
//    Serial.print("setAngVels[");
//    Serial.print(i);
//    Serial.print("]: ");
//    Serial.print(setAngVels[i]);
//    Serial.print(", angVels[");
//    Serial.print(i);
//    Serial.print("]: ");
//    Serial.println(angVels[i]);
    
    errs[i] = (long) round( ( b * setAngVels[i] - angVels[i] ) / degsPerRad * 256 ); // [rad/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
    Serial.print("errs[");
    Serial.print(i);
    Serial.print("] = ( b * setAngVels[");
    Serial.print(i);
    Serial.print("] - angVels[");
    Serial.print(i);
    Serial.print("] ) / degsPerRad = ( ");
    Serial.print(b);
    Serial.print(" * ");
    Serial.print(setAngVels[i]);
    Serial.print(" deg/s - ");
    Serial.print(angVels[i]);
    Serial.print(" deg/s ) / ");
    Serial.print(degsPerRad);
    Serial.print(" deg/rad = ");
    Serial.print(b * setAngVels[i] - angVels[i]);
    Serial.print(" deg/s / ");
    Serial.print(degsPerRad);
    Serial.print(" deg/rad = ");
    Serial.print(errs[i] / 256.0);
    Serial.println(" rad/s");
    
    P[i] = (long) round( kp[1][i] * errs[i] ); // P(t_k) = K(by_{sp}(t_k) - y(t_k))
    Serial.print("P[");
    Serial.print(i);
    Serial.print("] = kp[1][");
    Serial.print(i);
    Serial.print("] * errs[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(kp[0][i]);
    Serial.print(" % * ");
    Serial.print(errs[i] / 256.0);
    Serial.print(" rad/s = ");
    Serial.print(P[i] / 256.0);
    Serial.println(" %/s");
    
//    if(abs(errs[i]) < threshIntegral)
//    {
//      I[i] += (int) round( ki * errs[i] );
//
//      I[i] = clip(I[i], maxOutVal, -maxOutVal);
//    }
//    else
//      I[i] = 0;

    dAngVels[i] = (long) round( ( angVels[i] - prevAngVels[i] ) / degsPerRad * 256 );
    Serial.print("dAngVels[");
    Serial.print(i);
    Serial.print("] = ( angVels[");
    Serial.print(i);
    Serial.print("] - prevAngVels[");
    Serial.print(i);
    Serial.print("] ) / degsPerRad = ( ");
    Serial.print(angVels[i]);
    Serial.print(" deg/s - ");
    Serial.print(prevAngVels[i]);
    Serial.print(" deg/s ) / ");
    Serial.print(degsPerRad);
    Serial.print(" deg/rad = ");
    Serial.print(dAngVels[i] / 256.0);
    Serial.println(" rad/s");
    
    D[i] = (long) round( kd[1][i] * dAngVels[i] ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
    Serial.print("D[");
    Serial.print(i);
    Serial.print("] = kd[1][");
    Serial.print(i);
    Serial.print("] * dAngVels[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(kd[1][i]);
    Serial.print(" %.s * ");
    Serial.print(dAngVels[i]);
    Serial.print(" rad/s / 1 s = ");
    Serial.print(D[i] / 256.0);
    Serial.println(" %/s");

    Serial.print("mtrOut[");
    Serial.print(i);
    Serial.print("] = prevMtrOut[");
    Serial.print(i);
    Serial.print("] + P[");
    Serial.print(i);
    Serial.print("] + D[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print((int) round( mtrOut[i] / 256.0 ));
    Serial.print(" %/s + ");
     
    mtrOut[i] += P[i] + D[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel

    Serial.print(P[i] / 256.0);
    Serial.print(" %/s + ");
    Serial.print(D[i] / 256.0);
    Serial.print(" %/s = ");
    Serial.print((int) round( mtrOut[i] / 256.0 ));
    Serial.println(" %/s");
    
    prevAngVels[i] = angVels[i]; // maintain history of previous measured rotVel

    mtrOut[i] = clip(mtrOut[i], maxOutVal, -maxOutVal); // accumulator
    Serial.print("mtrOut[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print((int) round( mtrOut[i] / 256.0 ));
    Serial.println(" %/s");
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
    if(mtrEnable[i])
    {
      setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code
  
      setHBridgeDirectionBit(i, signedVals[i]);
    
      pubMtrCmds[i] = getPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
    }
  }
  
  for(i=0; i < numMtrs; i++)
  {
    if(mtrEnable[i])
      analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
  }
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
  if(signedVal < 0) 
    signs[mid] = -1; // {motor direction of rotation} = backward
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
    if(diffDrive)
    {
      if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
      else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
    }
    else
    {
      if(mid == 0) digitalWrite(mSigPins[mid], LOW);
      else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
    }
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {forward | resting}
  { 
    if(diffDrive)
    {
      if(mid == 0) digitalWrite(mSigPins[mid], LOW);
      else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
    }
    else
    {
      if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
      else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
    }
  }
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int mid, int magnitude)
{
  Serial.print("magnitude: ");
  Serial.println(magnitude);

  return map(magnitude, 0, 100, 0, 255);
  //return map(magnitude, 0, 100, 60, 255); // cruise outputs
}
