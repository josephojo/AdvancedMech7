
#include <Servo.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*
   NOTE:
        xxx_L represents a variable specifically for the LEFT motor of the robot (Looking forward with it)
        xxx_R represents a variable specifically for the RIGHT motor of the robot (Looking forward with it)
*/

/**********************************************************
   --------------------- Definitions ----------------------
 **********************************************************/

// -------------------- PIN Definitions --------------------

// Pins to set rotation direction
#define IN1_PIN_L 2
#define IN2_PIN_L 3
#define IN3_PIN_R 5 //4
#define IN4_PIN_R 6 //5

// Enable Pins: For "Enabling" the motors to rotate
#define EN_PIN_L 4 //6
#define EN_PIN_R 7

// Pins to recevie Encoder signals
#define ENCOD1_PIN_L 25
#define ENCOD2_PIN_L 26
#define ENCOD3_PIN_R 27
#define ENCOD4_PIN_R 28

//IR Sensor Definitions
#define IR_Servo_Reading 36    //IR Servo reading
#define IR_Left_Reading 22    //IR Left reading
#define IR_Front_Reading 17    //IR Front reading
#define IR_Right_Reading 39    //IR Right reading


#define MAX_DISTANCE 20.0
#define MIN_DISTANCE 2.0
#define NUM_CONDITIONS 37


// -------------------- Variable Definitions --------------------

// #################### Encoder Keeping ####################
volatile long ECount_L = 0;
volatile long ECount_R = 0;
long prev_ECount_L = 0;
long prev_ECount_R = 0;

// #################### Vehicle Parameters ####################
const float dia_L = 32.0;
const float dia_R = 32.0;
const float wheelBase = 87.5;
const int clicksPerRev = 2940;
const float cal_Factor_L = 0.0;
const float cal_Factor_R = 0.0;

// #################### Miner Location ####################
struct Miner {
  double xPos;
  double yPos;
};

// #################### Vehicle Information  ####################
struct Vehicle {
  double prev_xPos = 0.0;
  double prev_yPos = 0.0;
  double curr_xPos = 0.0;
  double curr_yPos = 0.0;
  double initial_xPos = 0.0;
  double initial_yPos = 0.0;
  double curr_Pos = 0.0;
  double prev_Pos = 0.0;
  double prev_Orien;
  double curr_Orien;
  double initial_Orien = 0.0;
  double curr_LinVel;
  double curr_Acc;
  double turn_Radius;
  double del_Ang;
  double del_Dist;
};

Vehicle robo; // Declares and instance of vehicle

// #################### Wheel Information  ####################
struct Wheel {
  volatile long ECount = 0;
  long prev_ECount = 0;

  double prev_LinDist = 0.0;
  double prev_LinVel = 0.0;
  double curr_LinDist;
  double curr_LinVel;
  double del_Dist = 0.0;

  double curr_AngVel;
};

Wheel whl_L;
Wheel whl_R;


// #################### Time Keeping ####################
double start_Time;
struct Timer {
  double curr_Time;
  double prev_Time;
  double dt() {
    return curr_Time - prev_Time;
  }

  double timeInSecs()
  {
    return ((millis() / 1000.0) - start_Time);
  }
};

Timer tim_L;
Timer tim_R;


// #################### IR Sensor Variables ####################
Servo IR_Servo;
Servo Arm_Servo;
Servo Claw_Servo;
int pos = 0;                            // variable to store the servo position

float Distance_IRServo = 0.0;
float Distance_IRLeft = 0.0;
float Distance_IRFront = 0.0;
float Distance_IRRight = 0.0;

// #################### PID ####################
struct PID {
  double Ki;
  double Kp;
  double Kd;
  double integral = 0.0;
  double derivative = 0.0;
  double lastError = 0.0;
  double error = 0.0;
  double pid;
};

PID IR_L_PID; // PID for the left IR
PID IR_R_PID;
PID E_L_PID;
PID E_R_PID;
PID TURN_PID;

// #################### MISC ####################
const double pi = 3.14159265359;
double waitTimer[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
uint8_t targetPWM = 70; // Baseline pwm for main motors
double targetSpeed = 2.1; // 1.7; // Baseline rotational speed for main motors (in rad/sec)
const double wall_Length = 17.8; // distance from pillar to pillar in cms
uint8_t counter = 1;
bool detour = false;
bool turn = false;
float targetDist_Side = 11.0; //11.15;
float targetDist_Front = 5.0; //8.0
double prev_Dist = 0.0 ;
double prev_Ang = 0.0;

double prevvy = 0.0;

int caseStep [][NUM_CONDITIONS] = 
{
{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,0,0,120,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,97,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,0,0,120,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,150,0,0,0,0,0,0,0,0,0,0},
{1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,0,0,120,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,1,0,0,0,425,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,150,0,0,0,0,0,0,0,0,0,0},
{1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,150,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,150,0,0,0,0,0,0,0,0,0,0},
{1,0,0,0,0,1,0,0,0,230,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,70,0,0,0 },
{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0  },
{0,1,0,0,0,1,0,0,0,230,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,0,0,120,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,0,0,120,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,150,0,0,0,0,0,0,0,0,0,0},
{1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0  },
{1,0,0,0,1,0,0,0,0,120,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0  },
{0,0,1,0,0,0,0,0,0,0,90,0,0,0,0,0,0,0,0,0 },
{1,0,0,0,0,1,0,0,0,230,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0  }
};

// ************************* End of Definitions *********************************


void setup()
{
  Serial.begin(115200);
  // set all the motor control pins to outputs
  pinMode(EN_PIN_L, OUTPUT);
  pinMode(EN_PIN_R, OUTPUT);
  pinMode(IN1_PIN_L, OUTPUT);
  pinMode(IN2_PIN_L, OUTPUT);
  pinMode(IN3_PIN_R, OUTPUT);
  pinMode(IN4_PIN_R, OUTPUT);
  pinMode(ENCOD1_PIN_L, INPUT);
  pinMode(ENCOD2_PIN_L, INPUT);
  pinMode(ENCOD3_PIN_R, INPUT);
  pinMode(ENCOD4_PIN_R, INPUT);

  // IR servo control set up
  IR_Servo.attach(38);                                            // servo to Digital pin 38
  IR_Servo.write(0);                                            // set servo to 0
  Arm_Servo.attach(37);                                            // servo to Digital pin 38
  Arm_Servo.write(0);                                            // set servo to 0
  Claw_Servo.attach(40);                                            // servo to Digital pin 38
  Claw_Servo.write(0);

  pinMode(IR_Servo_Reading, INPUT);
  pinMode(IR_Left_Reading, INPUT);
  pinMode(IR_Front_Reading, INPUT);
  pinMode(IR_Right_Reading, INPUT);


  // Setup Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCOD1_PIN_L), Encod_ISR_L, RISING); // interrrupt 1 is data ready
  attachInterrupt(digitalPinToInterrupt(ENCOD3_PIN_R), Encod_ISR_R, RISING); // interrrupt 1 is data ready RISING

  E_L_PID.Kp = 0.35; //0.8;
  E_L_PID.Ki = 0.1; //0.2
  E_L_PID.Kd = 0.3; //0.7;

  E_R_PID.Kp = 0.4; //0.4;
  E_R_PID.Ki = 0.1; //0.1;
  E_R_PID.Kd = 0.3; //0.3;

  IR_L_PID.Kp = 15.0; //5.0;
  IR_L_PID.Ki = 0.000005; //0.3;
  IR_L_PID.Kd = 5.0;

  IR_R_PID.Kp = 15.0; //5.0;
  IR_R_PID.Ki = 0.000005; //0.3;
  IR_R_PID.Kd = 5.0;

  TURN_PID.Kp = 0.3; //5.0;
  TURN_PID.Ki = 0.005; //0.1;
  TURN_PID.Kd = 0.5; //0.2;

  robo.prev_Orien = robo.initial_Orien;
  robo.prev_xPos = robo.initial_xPos;
  robo.prev_yPos = robo.initial_yPos;

  delay(5000);
  //counter = 14;

  start_Time = millis() / 1000;
}

int i = 0;

void loop()
{

  /* TESTING CODE
     Don't remove!!!!!!
     - -----------------------------------------------------------------------------------------------
    if ((millis() - waitTimer[0]) > 200)
    {
      //IR_Sensor();

      //Forward(U_PID.pid);
      //Leftward(E_L_PID.pid, E_R_PID.pid);
      //Forward(E_L_PID.pid, E_R_PID.pid);



        //Serial.print ("time in secs : "); //whl_L.curr_AngVel : ");
        // Serial.print (millis()/1000.0); //whl_L.curr_AngVel); //robo.curr_LinVel);
        // Serial.print ("\t");
        //    Serial.print ("targetSpeed : "); //whl_L.curr_AngVel : ");
        //    Serial.print (targetSpeed); //whl_L.curr_AngVel); //robo.curr_LinVel);
        //    Serial.print ("\t");
        //    Serial.print ("whl_L.curr_AngVel : "); //whl_L.curr_AngVel : ");
        //    Serial.print (whl_L.curr_AngVel);
        //    Serial.print ("\t");
        //    Serial.print ("whl_R.curr_AngVel : "); //whl_L.curr_AngVel : ");
        //    Serial.println (whl_R.curr_AngVel);


        waitTimer[0] = millis();
      }

      //  if (robo.curr_xPos > 200 - 0.7)
      //  {
      //    robo_Halt();
      //    Serial.print("robo.curr_Orien : ");
      //    Serial.println(rad2Deg(robo.curr_Orien));
      //    while (1) {}
      //  }

      //   if (rad2Deg(robo.curr_Orien) > 80) //zrobo.curr_xPos > 400)//  //curr_Pos
      //   {
      //      waitInSecs(0.01);
      //      Forward(E_L_PID.pid, E_R_PID.pid);
      //      Serial.print("robo.curr_xPos : ");
      //      Serial.print(robo.curr_xPos);
      //      Serial.print("\t");
      //      if (robo.curr_xPos > 50)
      //     {
      //       robo_Halt();
      //      Serial.print("robo.curr_Orien : ");
      //      Serial.println(rad2Deg(robo.curr_Orien));
      //      while(1){}
      //     }
      //
      //   }
      - -----------------------------------------------------------------------------------------------
  */
   
  //detour = true;
  if ((millis() - waitTimer[2]) > 50)
  { 
    Serial.print("Counter: ");
    Serial.println(counter);
    if (detour == false && counter < NUM_CONDITIONS)
    {
      // --------- Robot Movement --------
      if (caseStep[counter][0] == 1) // If asked to go forward
      {
        if (caseStep[counter][7] == 1) { //If asked to stop when left IR doesn't see a wall

          GoForward_IR_L(caseStep[counter][5], caseStep[counter][6], caseStep[counter][4]);

        } else if (caseStep[counter][8] == 1) { //If asked to stop when right IR doesn't see a wall

          GoForward_IR_R(caseStep[counter][5], caseStep[counter][6], caseStep[counter][4]);

        } else if (caseStep[counter][11] == 1) { //If asked to stop when front IR sees a wall

          GoForward_IR_F(caseStep[counter][5], caseStep[counter][6], caseStep[counter][4]);

        } else if (caseStep[counter][9] > 0) { //If asked to stop after a distance

          GoForward_Dist(caseStep[counter][5], caseStep[counter][6], caseStep[counter][4], caseStep[counter][9]);
          //Serial.println(caseStep[counter][9]);


        } else {
          Serial.println("Go Forward - No Condition to stop was given!!");
        }

      }
      else if (caseStep[counter][1] == 1) // If asked to go Backward
      {
        GoBackward_Dist(caseStep[counter][5], caseStep[counter][6], caseStep[counter][4], (double)caseStep[counter][9]);
      }
      else if (caseStep[counter][2] == 1) // If asked to Turn right
      {
        TurnRight_Ang(caseStep[counter][10]);
        //Turn_Ang(caseStep[counter][10]);
      }
      else if (caseStep[counter][3] == 1) // If asked to Turn Left
      {
        TurnLeft_Ang(-caseStep[counter][10]);
        //Turn_Ang(-caseStep[counter][10]);
      }
      else {
        caseStep[counter][19] = 1;
      }


      // ---------- Arm Movement -------
      if (caseStep[counter][12] == 1) // If asked to Move Arm
      {
        MoveArm_Ang(caseStep[counter][16]);
        
      } else if (caseStep[counter][13] == 1) // If asked to Reset Arm Location
      {
        MoveArm_Ang(0);
      } else {
        caseStep[counter][20] = 1;
      }

      if (caseStep[counter][15] == 1) // If asked to Grip
      {

      } else if (caseStep[counter][16] == 1) // If asked to Release Grip
      {

      } else {
        caseStep[counter][18] = 1;
      }

      if (caseStep[counter][16] == 1) // If asked to Scan
      {
        
      }

      if (caseStep[counter][18] == 1 && caseStep[counter][19] == 1 && caseStep[counter][20] == 1) {
        counter++;
      }
    }

    if (detour == true)
    {
      //Leftward(0);
      //TurnLeft_Ang(-90);
      if (counter <= 1) {
        //Turn_Ang(270);
        //      Serial.print("Prev Angle : "); Serial.print("\t");Serial.println(rad2Deg(prev_Ang));
        //      Serial.print("Curr Angle : "); Serial.print("\t");Serial.println(rad2Deg(robo.curr_Orien));
      }
      //Forward(-IR_L_PID.pid);
      //Forward(IR_R_PID.pid);
      //Rightward(E_L_PID.pid, E_R_PID.pid);
      Leftward(E_L_PID.pid, E_R_PID.pid);
      //Forward(E_L_PID.pid, E_R_PID.pid);
      //Serial.println("In Detour");
    }

    waitTimer[2] = millis();
  }

  // Updates all variables for calculations every 50 ms
  if ((millis() - waitTimer[1]) > 25)
  {
    cli();
    long EL = ECount_L;
    long ER = ECount_R;
    sei();

    int delta_L = EL - prev_ECount_L;
    int delta_R = ER - prev_ECount_R;

    prev_ECount_L = EL;
    prev_ECount_R = ER;

    // Get the current speed of the motors
    whl_L.curr_AngVel = whl_AngVel_L(delta_L);
    whl_R.curr_AngVel = whl_AngVel_R(delta_R);

//    Serial.print(whl_R.curr_AngVel);
//    Serial.print("\t");
//    Serial.print(whl_L.curr_AngVel);
//    Serial.print("\t");
//    Serial.print("Distance_IRL: ");
//  Serial.print(Distance_IRLeft);
    
    // Update change in wheel distances
    whl_L.del_Dist = whlDeltaD_L(delta_L);
    whl_R.del_Dist = whlDeltaD_R(delta_R);

    // Calculate the PID
    IR_L_PID.error = IR_L_Error();
//    Serial.print("\t");
//    Serial.print("IR_L_PID.error: ");
//  Serial.println(IR_L_PID.error);
    IR_L_PID.integral = IR_L_PID.integral + IR_L_PID.error;
    IR_L_PID.derivative = IR_L_PID.error - IR_L_PID.lastError;
    IR_L_PID.pid = (IR_L_PID.Kp * IR_L_PID.error) + (IR_L_PID.Ki * IR_L_PID.integral) + (IR_L_PID.Kd * IR_L_PID.derivative);
    IR_L_PID.lastError = IR_L_PID.error;

    IR_R_PID.error = IR_R_Error();
    IR_R_PID.integral = IR_R_PID.integral + IR_R_PID.error;
    IR_R_PID.derivative = IR_R_PID.error - IR_R_PID.lastError;
    IR_R_PID.pid = (IR_R_PID.Kp * IR_R_PID.error) + (IR_R_PID.Ki * IR_R_PID.integral) + (IR_R_PID.Kd * IR_R_PID.derivative);
    IR_R_PID.lastError = IR_R_PID.error;

    E_L_PID.error = encodError(targetSpeed, whl_L);
    E_L_PID.integral = E_L_PID.integral + E_L_PID.error;
    E_L_PID.derivative = E_L_PID.error - E_L_PID.lastError;
    E_L_PID.pid = (E_L_PID.Kp * E_L_PID.error) + (E_L_PID.Ki * E_L_PID.integral) + (E_L_PID.Kd * E_L_PID.derivative);
    E_L_PID.lastError = E_L_PID.error;

    E_R_PID.error = encodError(targetSpeed, whl_R);
    E_R_PID.integral = E_R_PID.integral + E_R_PID.error;
    E_R_PID.derivative = E_R_PID.error - E_R_PID.lastError;
    E_R_PID.pid = (E_R_PID.Kp * E_R_PID.error) + (E_R_PID.Ki * E_R_PID.integral) + (E_R_PID.Kd * E_R_PID.derivative);
    E_R_PID.lastError = E_R_PID.error;

    //    if(turn == true)
    //    {
    //TURN_PID.error = turnError()
    //Serial.print("TURN_PID.pid: ");Serial.println(TURN_PID.pid);

    TURN_PID.integral = TURN_PID.integral + TURN_PID.error;
    TURN_PID.derivative = TURN_PID.error - TURN_PID.lastError;
    TURN_PID.pid = (TURN_PID.Kp * TURN_PID.error) + (TURN_PID.Ki * TURN_PID.integral) + (TURN_PID.Kd * TURN_PID.derivative);
    TURN_PID.lastError = TURN_PID.error;
    //}

    Distance_IRFront = IRFront();
    Distance_IRServo = IRServo();

    // Update Delta Distance
    robo.del_Dist = roboDel_Dist();

    // Update Location of Robot
    robo.curr_yPos = roboDist_Y();
    robo.curr_xPos = roboDist_X();

    robo.curr_Pos = roboDist();

    // Update Delta Angle
    robo.del_Ang = roboDel_Ang();

    // Update Robot Orientation
    robo.curr_Orien = roboAngle();

    prevvy = robo.curr_Orien;
    //    Serial.print("robo.prev_Orien : "); //whl_L.curr_AngVel : ");
    //    Serial.println (rad2Deg(prevvy));
    robo.prev_Pos = robo.curr_Pos;
    robo.prev_xPos = robo.curr_xPos;
    robo.prev_yPos = robo.curr_yPos;
    //Serial.println(robo.curr_Pos);

    waitTimer[1] = millis();
  }
}

/**********************************************************
  --------------------- FUNCTIONS ----------------------
 **********************************************************/

// -------------------- ISRs --------------------
void Encod_ISR_L()
{
  if (digitalRead(ENCOD2_PIN_L) == LOW)
  {
    /** and counts down when move from R to L **/
    ECount_L--;
  }
  else
  {
    /** Encoder counts up when move from L to R **/
    ECount_L++;
  }
  //   Serial.print("ECount_L : ");
  //   Serial.println(ECount_L);
}

void Encod_ISR_R()
{
  if (digitalRead(ENCOD4_PIN_R) == LOW)
  {
    /** and counts down when move from R to L **/
    ECount_R--;
  }
  else
  {
    /** Encoder counts up when move from L to R **/
    ECount_R++;
  }
  //   Serial.print("ECount_R : ");
  //   Serial.println(ECount_R);
}

// -------------------- Motion Control Functions --------------------

// Robot Forward function for the IRs
void Forward(double pidResult)
{
  digitalWrite(IN1_PIN_L, LOW);
  digitalWrite(IN2_PIN_L, HIGH);
  analogWrite(EN_PIN_L, (RpS2pwm_L(targetSpeed) + pidResult)); //-13 // + 30

  digitalWrite(IN3_PIN_R, LOW);
  digitalWrite(IN4_PIN_R, HIGH);
  analogWrite(EN_PIN_R, (RpS2pwm_R(targetSpeed) - pidResult));
}

// Robot Forward function for the Encoders
void Forward(double E_pid_L, double E_pid_R)
{
  digitalWrite(IN1_PIN_L, LOW);
  digitalWrite(IN2_PIN_L, HIGH);
  analogWrite(EN_PIN_L, (RpS2pwm_L(targetSpeed - E_pid_L)));

  digitalWrite(IN3_PIN_R, LOW);
  digitalWrite(IN4_PIN_R, HIGH);
  analogWrite(EN_PIN_R, (RpS2pwm_R(targetSpeed - E_pid_R)));
}

// Robot Backward function for the IRs
void Backward(double pidResult)
{
  digitalWrite(IN1_PIN_L, HIGH);
  digitalWrite(IN2_PIN_L, LOW);
  analogWrite(EN_PIN_L, (RpS2pwm_L(targetSpeed) + pidResult)); //-13

  digitalWrite(IN3_PIN_R, HIGH);
  digitalWrite(IN4_PIN_R, LOW);
  analogWrite(EN_PIN_R, (RpS2pwm_R(targetSpeed) - pidResult));
}

// Robot Backward function for the Encoders
void Backward(double E_pid_L, double E_pid_R)
{
  digitalWrite(IN1_PIN_L, HIGH);
  digitalWrite(IN2_PIN_L, LOW);
  analogWrite(EN_PIN_L, (RpS2pwm_L(targetSpeed - E_pid_L)));

  digitalWrite(IN3_PIN_R, HIGH);
  digitalWrite(IN4_PIN_R, LOW);
  analogWrite(EN_PIN_R, (RpS2pwm_R(targetSpeed - E_pid_R)));
}

void Leftward(double E_pid_L, double E_pid_R)
{
  digitalWrite(IN1_PIN_L, HIGH);
  digitalWrite(IN2_PIN_L, LOW);
  analogWrite(EN_PIN_L, RpS2pwm_L(targetSpeed - E_pid_L)); // - 14);

  digitalWrite(IN3_PIN_R, LOW);
  digitalWrite(IN4_PIN_R, HIGH);
  analogWrite(EN_PIN_R, RpS2pwm_R(targetSpeed - E_pid_R));
}

void Rightward(double E_pid_L, double E_pid_R)
{
  digitalWrite(IN1_PIN_L, LOW);
  digitalWrite(IN2_PIN_L, HIGH);
  analogWrite(EN_PIN_L, RpS2pwm_L(targetSpeed - E_pid_L)); // - 14);

  digitalWrite(IN3_PIN_R, HIGH);
  digitalWrite(IN4_PIN_R, LOW);
  analogWrite(EN_PIN_R, RpS2pwm_R(targetSpeed - E_pid_R));
}

void Leftward(double pidResult)
{
  digitalWrite(IN1_PIN_L, HIGH);
  digitalWrite(IN2_PIN_L, LOW);
  analogWrite(EN_PIN_L, RpS2pwm_L(targetSpeed) - pidResult);
  //Serial.println(RpS2pwm_L(targetSpeed));
  digitalWrite(IN3_PIN_R, LOW);
  digitalWrite(IN4_PIN_R, HIGH);
  analogWrite(EN_PIN_R, RpS2pwm_R(targetSpeed) - pidResult);
}

void Rightward(double pidResult)
{
  digitalWrite(IN1_PIN_L, LOW);
  digitalWrite(IN2_PIN_L, HIGH);
  analogWrite(EN_PIN_L, RpS2pwm_L(targetSpeed) + pidResult);

  digitalWrite(IN3_PIN_R, HIGH);
  digitalWrite(IN4_PIN_R, LOW);
  analogWrite(EN_PIN_R, RpS2pwm_R(targetSpeed) + pidResult);
}

void robo_Halt()
{
  digitalWrite(IN1_PIN_L, LOW);
  digitalWrite(IN2_PIN_L, LOW);
  digitalWrite(IN3_PIN_R, LOW);
  digitalWrite(IN4_PIN_R, LOW);
}

// ################## High Level Robot Control #############

// Go Forward until Dist is reached - uses the selected PID
void GoForward_Dist(uint8_t PID_L_IR, uint8_t PID_R_IR, uint8_t PID_encod, double dist) { // dist used to be 75 for centering
  if (PID_L_IR == 1) {
    Forward(-IR_L_PID.pid);
    if (abs(robo.curr_Pos - prev_Dist) > dist) //x
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_R_IR == 1) {
    Forward(IR_R_PID.pid);
    if (abs(robo.curr_Pos - prev_Dist) > dist)//x
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_encod == 1) {
    Forward(E_L_PID.pid, E_R_PID.pid);
    //Serial.println("Heyyy");
    if (abs(robo.curr_Pos - prev_Dist) > dist)//x
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  }
}

// Go Forward until Front IR sensor detects a wall - uses the selected PID
void GoForward_IR_F(uint8_t PID_L_IR, uint8_t PID_R_IR, uint8_t PID_encod) {
  if (PID_L_IR == 1) {
    Forward(-IR_L_PID.pid);
    if (Distance_IRFront <= targetDist_Front)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_R_IR == 1) {
    Forward(IR_R_PID.pid);
    if (Distance_IRFront <= targetDist_Front)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_encod == 1) {
    Forward(E_L_PID.pid, E_R_PID.pid);
    if (Distance_IRFront <= targetDist_Front)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  }
}

// Go Forward until Left IR sensor doesn't detect a wall - uses the selected PID
void GoForward_IR_L(uint8_t PID_L_IR, uint8_t PID_R_IR, uint8_t PID_encod) {
  if (PID_L_IR == 1) {
    Forward(-IR_L_PID.pid);
    if (Distance_IRLeft >= MAX_DISTANCE)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_R_IR == 1) {
    Forward(IR_R_PID.pid);
    if (Distance_IRLeft >= MAX_DISTANCE)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_encod == 1) {
    Forward(E_L_PID.pid, E_R_PID.pid);
    if (Distance_IRLeft >= MAX_DISTANCE)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  }
}

// Go Forward until Right IR sensor doesn't detect a wall - uses the selected PID
void GoForward_IR_R(uint8_t PID_L_IR, uint8_t PID_R_IR, uint8_t PID_encod) {
  if (PID_L_IR == 1) {
    Forward(-IR_L_PID.pid);
    if (Distance_IRRight >= 20.0)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_R_IR == 1) {
    Forward(IR_R_PID.pid);
    if (Distance_IRRight >= 20.0)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_encod == 1) {
    Forward(E_L_PID.pid, E_R_PID.pid);
    if (Distance_IRRight >= 20.0)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  }
}

// Go Backward until Dist is reached - uses the selected PID
void GoBackward_Dist(uint8_t PID_L_IR, uint8_t PID_R_IR, uint8_t PID_encod, double dist) {
  if (PID_L_IR == 1) {
    Backward(-IR_L_PID.pid);
    if (abs(robo.curr_Pos - prev_Dist) > dist)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_R_IR == 1) {
    Backward(IR_R_PID.pid);
    if (abs(robo.curr_Pos - prev_Dist) > dist)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  } else if (PID_encod == 1) {
    Backward(E_L_PID.pid, E_R_PID.pid);
    if (abs(robo.curr_Pos - prev_Dist) > dist)
    {
      robo_Halt();
      prev_Dist = robo.curr_Pos; //x
      prev_Ang = robo.curr_Orien;
      caseStep[counter][19] = 1;
    }
  }
}

// Turn Right until Angle is reached
void TurnRight_Ang(int ang) { // Angle used to be 90 - 10
  Rightward(E_L_PID.pid, E_R_PID.pid);
  if (rad2Deg(robo.curr_Orien - prev_Ang) > ang - 10)
  {
    robo_Halt();
    prev_Dist = robo.curr_Pos;
    prev_Ang = robo.curr_Orien;
    caseStep[counter][19] = 1;
  }
}

// Turn Left until Angle is reached
void TurnLeft_Ang(int ang) { // Angle used to be -90 + 7
  Leftward(E_L_PID.pid, E_R_PID.pid);
  Serial.println("LEFT");
  if (rad2Deg(robo.curr_Orien - prev_Ang) < ang + 12)
  {
    robo_Halt();
    prev_Dist = robo.curr_Pos;
    prev_Ang = robo.curr_Orien;
    caseStep[counter][19] = 1;
  }
}

// Turn until Angle is reached
void Turn_Ang(int ang) {
  double tempSpeed = targetSpeed;
  targetSpeed = 1.0;
  TURN_PID.error = turnError(ang);
  //  Serial.println("In TurnLeft");
  //  Serial.print("\t");
  Serial.print("TURN_PID.Error: "); Serial.println(TURN_PID.error);
  Serial.print("TURN_PID.Pid: "); Serial.println(TURN_PID.pid);

  if (TURN_PID.error < -2)
  {
    Leftward(TURN_PID.pid);
    //Serial.println("ello");
  } else if (TURN_PID.error > 2)
  {
    Rightward(TURN_PID.pid);
  } else
  {
    robo_Halt();
    TURN_PID.error = 0.0;
    TURN_PID.integral = 0.0;
    TURN_PID.derivative = 0.0;
    targetSpeed = tempSpeed;
    prev_Dist = robo.curr_Pos;
    prev_Ang = robo.curr_Orien;
    //counter++;
    caseStep[counter][19] = 1;
  }
}

// <-------------- End of High Level Robot Control ############


// ################## High Level Manipulator Control -------------->

// Move Arm until Angle is reached

void MoveArm_Ang(int ang) {
  Arm_Servo.write(ang);
  delay(500);
}

void MoveClaw_Ang(int ang) {
  Claw_Servo.write(ang);
  delay(500);
}

// <--------------- End of High Level Manipulator Control ############

//-------------------------IR Sensors-------------------------
double IR_Servo_Scan() {

  for (pos = 0; pos <= 40; pos += 1) {          // goes from 0 degrees to 60 degrees
    IR_Servo.write(pos);                           // tell servo to go to position in variable 'pos'
    if (pos == 0 || pos == 10 || pos == 20 || pos == 30 || pos == 40) {
      float Dis_IRServo = IRServo();
      Serial.print("Servo IR Distance: ");
      Serial.println(Dis_IRServo);
      return Dis_IRServo;
    }
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (pos = 40; pos >= 0; pos -= 1) {                      // goes from 60 degrees to 0 degrees
    IR_Servo.write(pos);                                     // tell servo to go to position in variable 'pos'
    if (pos == 0 || pos == 10 || pos == 20 || pos == 30 || pos == 40) {
      float Dis_IRServo = IRServo();
      Serial.print("Servo IR Distance: ");
      Serial.println(Dis_IRServo);
      return Dis_IRServo;
    }
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

float IRFront() {
  double IR_Front_Value;
  analogReadResolution(10);
  analogReadAveraging(100);
  IR_Front_Value = analogRead(IR_Front_Reading); //take reading from sensor

  float Raw_Voltage_IRFront = IR_Front_Value * 0.00322265625; //convert analog reading to voltage (5V/1024bit=0.0048828125)(3.3V/1024bit =0.00322265625)
  float Dis_IRFront = -29.642 * Raw_Voltage_IRFront + 71.236;

  Serial.print("Front IR Distance: ");
  Serial.println(Distance_IRFront);

  return Dis_IRFront;
}

float IRRight() {

  double IR_Right_Value;
  analogReadResolution(10);
  analogReadAveraging(100);
  IR_Right_Value = analogRead(IR_Right_Reading); //take reading from sensor

  float Raw_Voltage_IRRight = IR_Right_Value * 0.00322265625; //convert analog reading to voltage (5V/1024bit=0.0048828125)(3.3V/1024bit =0.00322265625)
  float Dis_IRRight = -29.642 * Raw_Voltage_IRRight + 69.236;

//  Serial.print("Right IR Distance: ");
//  Serial.println(Dis_IRRight);
  return Dis_IRRight;
}

float IRLeft() {

  double IR_Left_Value;
  analogReadResolution(10);
  analogReadAveraging(100);
  IR_Left_Value = analogRead(IR_Left_Reading); //take reading from sensor

  float Raw_Voltage_IRLeft = IR_Left_Value * 0.00322265625; //convert analog reading to voltage (5V/1024bit=0.0048828125)(3.3V/1024bit =0.00322265625)
  float Dis_IRLeft = -29.642 * Raw_Voltage_IRLeft + 71.236;

  Serial.print("Left IR Distance: ");
  Serial.println(Dis_IRLeft);

  return Dis_IRLeft;
}

float IRServo() {

  double IR_Servo_Value;
  analogReadResolution(10);
  analogReadAveraging(100);
  IR_Servo_Value = analogRead(IR_Servo_Reading); //take reading from sensor

  float Raw_Voltage_IRServo = IR_Servo_Value * 0.00322265625; //convert analog reading to voltage (5V/1024bit=0.0048828125)(3.3V/1024bit =0.00322265625)
  float Dis_IRServo = -29.642 * Raw_Voltage_IRServo + 69.236;

//  Serial.print("Servo IR Distance: ");
//  Serial.println(Dis_IRServo);
  return Dis_IRServo;
  
}
// -------------------- Robot Localization Functions --------------------

// -------------------- PID Functions --------------------

// Solves for the difference in the rotational speed of the wheel compared to a set value (in rad/sec)
double encodError(double setVal, Wheel W) {
  return abs(W.curr_AngVel) - setVal;
}

// Solves for the difference between the left IR sensor and 8cm from left wall
double IR_L_Error() {
  float distanceGood_L = 0.0;

  Distance_IRLeft = IRLeft();

  if (Distance_IRLeft < MAX_DISTANCE && Distance_IRLeft > MIN_DISTANCE) {
    distanceGood_L = Distance_IRLeft;
  }
  else if (Distance_IRLeft > MAX_DISTANCE) {
    distanceGood_L = MAX_DISTANCE;
    Distance_IRLeft = MAX_DISTANCE;
  }
  else {
    Distance_IRLeft = MIN_DISTANCE;
    distanceGood_L = MIN_DISTANCE;
  }
  //  Serial.print("distanceGood_L: ");
  //  Serial.print(distanceGood_L);
  //  Serial.print("\t");
  return distanceGood_L - (targetDist_Side); // + 6
}


// Solves for the difference between the right IR sensor and 8cm from right wall
double IR_R_Error() {
  float distanceGood_R = 0.0;

  Distance_IRRight = IRRight();

  if (Distance_IRRight < MAX_DISTANCE && Distance_IRRight > MIN_DISTANCE) {
    distanceGood_R = Distance_IRRight;
  }
  else if (Distance_IRRight > MAX_DISTANCE) {
    distanceGood_R = MAX_DISTANCE;
    Distance_IRRight = MAX_DISTANCE;
  }
  else {
    Distance_IRRight = MAX_DISTANCE;
    distanceGood_R = MAX_DISTANCE;
  }
  //  Serial.print("distanceGood_L: ");
  //  Serial.print(distanceGood_L);
  //  Serial.print("\t");
  return distanceGood_R - targetDist_Side;
}

// Solves for the difference in the angle of the robot compared to a set value (in deg)
double turnError(double setVal) {
  //Serial.println(
  return setVal - (rad2Deg(robo.curr_Orien - prev_Ang));
}

// -------------------- Formulas --------------------

// #################### Wheel related Formulas ####################

// Left Wheel Linear Distance in mm
double whl_Dist_L()
{
  return distPerClick(dia_L) * ECount_L;
}

// Riight Wheel Linear Distance in mm
double whl_Dist_R()
{
  return distPerClick(dia_R) * ECount_R;
}

double whlDeltaD_L(int del)
{
  double ans = distPerClick(dia_L) * (del);

  return ans;
}

double whlDeltaD_R(int del)
{
  double ans = distPerClick(dia_L) * (del);

  return ans;
}


//returns wheel linear velocity in mm/sec
double whl_LinVel(Wheel w, Timer tim)
{
  tim.curr_Time = tim.timeInSecs();
  w.curr_LinVel = (w.curr_LinDist - w.prev_LinDist) / tim.dt();
  tim.prev_Time = tim.curr_Time;
  w.prev_LinDist = w.curr_LinDist;

  return w.curr_LinVel;
}

double whl_AngVel_L(int del)
{
  tim_L.curr_Time = tim_L.timeInSecs();

  double dt = tim_L.dt();
  double angVel = ((del / dt) / clicksPerRev ) * 2 * pi;

  tim_L.prev_Time = tim_L.curr_Time;

  return angVel;
}

double whl_AngVel_R(int del)
{
  tim_R.curr_Time = tim_R.timeInSecs();
  double dt = tim_R.dt();
  double angVel = ((del / dt) / clicksPerRev ) * 2 * pi;

  tim_R.prev_Time = tim_R.curr_Time;
  return angVel;
}


// #################### Robot Formulas ####################

double roboDist_Y()
{
  return (robo.del_Dist * sin((robo.del_Ang / 2) + prevvy)) + robo.prev_yPos; //robo.initial_yPos - (robo.turn_Radius * (cos(robo.curr_Orien) - cos(robo.initial_Orien)));
}

double roboDist_X()
{
  return (robo.del_Dist * cos((robo.del_Ang / 2) + prevvy)) + robo.prev_xPos; // robo.initial_xPos + (robo.turn_Radius * (sin(robo.curr_Orien) - sin(robo.initial_Orien)));
}

double roboDist()
{
  return robo.del_Dist + robo.prev_Pos; //(whl_L.curr_LinDist + whl_R.curr_LinDist) / 2;
}

double roboDel_Dist()
{
  return (whl_L.del_Dist + whl_R.del_Dist) / 2;
}

double roboVel()
{
  return (whl_R.curr_LinVel + whl_L.curr_LinVel) / 2.0 * sin(robo.curr_Orien);
}

double roboAcc()
{
  return ((whl_R.curr_LinVel + whl_L.curr_LinVel) * (whl_R.curr_LinVel - whl_L.curr_LinVel)) / (2 * wheelBase) * cos(robo.curr_Orien);
}

double distPerClick(double dia)
{
  return 2 * (pi * dia) / clicksPerRev;
}

double roboAngle()
{
  return robo.del_Ang + prevvy; //((whl_R.curr_LinVel - whl_L.curr_LinVel) * tim_L.curr_Time()) / wheelBase + robo.initial_Orien;
}

double roboDel_Ang()
{
  return (whl_L.del_Dist - whl_R.del_Dist) / wheelBase; //((whl_R.curr_LinVel - whl_L.curr_LinVel) * tim_L.curr_Time()) / wheelBase + robo.initial_Orien;
}

double turn_Radius()
{
  return (wheelBase * (whl_R.curr_LinVel + whl_L.curr_LinVel)) / (2 * (whl_R.curr_LinVel - whl_L.curr_LinVel));
}




// #################### General Math Formulas ####################

// Converts radian angle to degrees
double rad2Deg(double radi)
{
  return (180 / pi) * radi;
}

// Converts degrees to radian
double deg2Rad(double degr)
{
  return (pi / 180) * degr;
}

// Converts pwm Value to radian/sec speed for the left motor
double pwm2RpS_L(int val)
{
  return 1.4082 * log(val) - 3.7683;// Formula gotten from approximating with a log function in excel
}

// Converts pwm Value to radian/sec speed for the right motor
double pwm2RpS_R(int val)
{
  //return -6E-05 * (val * val) + 0.0292* val + 0.5616;
  return 1.3757 * log(val) - 3.6931;// Formula gotten from approximating with a log function in excel
}

//Converts radians per second to pwm for the left motor
int RpS2pwm_L(double rps)
{
  return exp((rps + 3.7683) / 1.4082);
}

//Converts radians per second to pwm for the right motor
int RpS2pwm_R(double rps)
{
  // int x1 = (-0.0292 + sqrt((0.0292 * 0.0292) - (4 * -6E-05 * (0.5616 - rps)))) / (2 * -6E-05);
  // int x2 = (-0.0292 - sqrt((0.0292 * 0.0292) - (4 * -6E-05 * (0.5616 - rps)))) / (2 * -6E-05);

  return exp((rps + 3.6931) / 1.3757);
}

void waitInSecs(double period)
{
  long t1 = millis();
  while (((millis() - t1) / 1000.0) < period) {}
}

/**********************************************************
   ------------------- END of FUNCTIONS -------------------
 **********************************************************/
