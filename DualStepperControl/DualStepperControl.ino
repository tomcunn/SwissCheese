//**************************************
//*   Stepper Motor Control
//*
//*   Cloned from the AI project
//*   12/11/2022
//*   Cunningham
//***************************************

#include <avr/io.h>
#include <avr/interrupt.h>

#define PRINT_XY
#define PRINT_JOINTS
#define PRINT_STEPS

//Hardware Assignments
#define  MAX_VELOCITY  90000
#define  MAX_ACCELERATION  2000
#define  ERROR_TOLERANCE 100

#define StepA    A0
#define DirA     A1
#define EnableA  38

#define StepB    46
#define DirB     48
#define EnableB  A8

//Function prototypes
void  SetupTimer1              (void);
void  SetupTimer0              (void);
void  MotionAccelerationControl(struct MovementParms *MotorCurrent, struct MovementParms *MotorDesired);
void  MotionVelocityControl    (struct MovementParms *MotorCurrent, struct MovementParms *MotorDesired);
void  ForwardKinematics        (struct Joint_Distances *JointLength, struct Coordinates *CurrentCoordinates);
void  InverseKinematics        (unsigned int x, unsigned int y, struct Joint_Distances *JointLength);
void  WriteStepSize            (struct MovementParms *Motor);
void  ReadJoystick             (struct Coordinates *lDesiredCoordinates);

static int counter=0;

//394mm is the initial joint length.

#define Bx_distance 470 //18.5 inches
#define Steps_per_mm 27.03f  * 2  //Added the x2 because we changed to a higher frequency get 50% duty cycle. Now we interrupt on the rising and falling edge of the pulse. 

#define MAX_X 450
#define MAX_Y 550

//Create a structure for movement data
struct MovementParms
{
  signed long  Position;          // steps  - Everytime the interrupt is triggered, this occurs
  signed int   Velocity;          // 
  signed int   Acceleration;      //
  unsigned int TimerMatchValue;   //
  bool         Direction;         //
  bool         Hold;              //
};

struct Joint_Distances
{
   float AD;               // mm
   float BD;               // mm
   float AB;               // mm
};




struct Coordinates
{
  float x;
  float y;
};

//Create instance of three structures 
volatile struct MovementParms MotorA_Desired;
volatile struct MovementParms MotorA_Current;
volatile struct MovementParms MotorB_Desired;
volatile struct MovementParms MotorB_Current;


struct MovementParms LinearMotion;
struct Joint_Distances Joints;
struct Coordinates CurrentCoordinates;
struct Coordinates DesiredCoordinates;


//Static Variables that are accessed in interrupts
volatile unsigned int abs_Current_Velocity = 0;
static volatile unsigned int Counter = 0;
static volatile unsigned int StepSpeedA = 0;
static volatile unsigned int StepSpeedB = 0;
static volatile unsigned int tempA = 0;
static volatile unsigned int tempB = 0;
static volatile signed long timestep = 0;
static volatile bool RunTask_10ms = 0;
static volatile bool flopa = 0;
static volatile bool flopb = 0;


//Kinematics (distance in mm)
//  A(0,0)        B(100,0)
//   A          B
//     \       /
//      \     /
//       \   /
//         D
//Given x,y, find the distances AD, BD
void InverseKinematics(unsigned int x, unsigned int y, struct Joint_Distances *JointLength)
{
  float x2 = (float)x * x;
  float y2 = (float)y * y;
  
  JointLength->AD = (float)sqrt(x2+y2);
  JointLength->BD = sqrt(((float)Bx_distance-x)*(Bx_distance-x)+y2);
}


//Given the joint lengths, determine the position x,y
void ForwardKinematics(struct Joint_Distances *JointLength, struct Coordinates *Coord)
{
  float AD2 = JointLength->AD * JointLength->AD;
  float BD2 = JointLength->BD * JointLength->BD;

  float x = (AD2 - BD2 + ((float)Bx_distance * Bx_distance)) / (2 * Bx_distance);
  float y2 = AD2 - (x * x);

  if(y2 < 0) return -1;
    Coord->x = x;
    Coord->y = sqrt(y2);
  return 0;
}


//**************************************
// Timer1 - 16 bit timer Interrupt  
// Motor A
// Setup for ATMEGA2560
//**************************************
ISR(TIMER4_COMPA_vect)  
{   
     //Check the direction pin and determine the position
    if(MotorA_Current.Hold == false)
    {
      if(MotorA_Current.Direction == 1)
      {
          digitalWrite(DirA,1);
          MotorA_Current.Position++;
      }
      else
      {
        digitalWrite(DirA,0);
        MotorA_Current.Position--;
      }
      
      if(flopa)
      {
          digitalWrite(StepA,1);
          flopa = 0;
      }
      else
      {
          digitalWrite(StepA,0);
          flopa=1;
      }
    }
   //tempA =StepSpeedA;
   OCR4A= StepSpeedA;
  
}

//**************************************
// Timer3 - 16 bit timer Interrupt 
// Motor B 
// Setup for ATMEGA2560
//**************************************
ISR(TIMER3_COMPA_vect)  
{   
    if(MotorB_Current.Hold == false)
    {
      //Check the direction pin and determine the position
      if(MotorB_Current.Direction == 1)
      {
         digitalWrite(DirB,0);
         MotorB_Current.Position++;
      }
      else
      {
         digitalWrite(DirB,1);
         MotorB_Current.Position--;
      }


      if(flopb)
      {
         digitalWrite(StepB,1);
         flopb = 0;
      }
      else
      {
         digitalWrite(StepB,0);
         flopb=1;
      }
    }
   //tempB = StepSpeedB;
   OCR3A =  StepSpeedB;
}

//**************************************
// Timer0 - 8 bit timer Interrupt  
// Setup for ATMEGA2560
//**************************************
ISR(TIMER0_COMPA_vect)
{
  //Check to see if the flag was not cleared, this will track 
  //overruns in the 10ms task
  if(RunTask_10ms)
  {
    digitalWrite(LED_BUILTIN,1);
  }
  RunTask_10ms = 1;
}


//*******************************************
// WriteStepSize
//
// Pass in a velocity and update the step size
// global variable which is read by the interrupt
// to generate the stepper pulses. Speed is in 
// the units mm/min. 
//*******************************************
void WriteStepSize(struct MovementParms *Motor)
{
  unsigned int StepSpeed = 0;
  //Set the direction pin based on the sign of the speed
  if(Motor->Velocity > 0)
  {
     Motor->Direction = 1;
  }
  else
  {
     Motor->Direction = 0;
  } 
  // Conversion Factor in One Note
  // 1:1 Prescale = 16,000,000Hz
  // 200*16microsteps = 3200 steps/rev
  // 27.03 steps/mm

  //Check to prevent overflows, StepSpeed is 16 bit
  // 700 mm/sec is VERY slow, 0.25 rev/sec.
  if(((Motor->Velocity) > 700) || ((Motor->Velocity) < -700))
  {
     //This is derived from the table above using a Power trendline
      Motor->TimerMatchValue = (unsigned int)(16000000L/(abs(Motor->Velocity)));
  }
  //This is like setting it to zero speed
  else
  {
   Motor->TimerMatchValue = 65000;
     
  }
}
//*******************************************
//  Configure Timer 1 - 16 bit timer
//  Atmega2560
//  Timer/Counter 1, 3, 4, and 5
//*******************************************
void SetupTimer4()
{
  //Clear the register
  TCCR4A = 0;
  TCCR4B = 0;

  //Set the prescaler
  TCCR4B|=(1<<CS10);// Clock/1

  //Start counting at zero and then interrupt on match
  TCCR4B|=(1<<WGM12); // Set in CTC mode

  //The value of the match.
  OCR4A=0; // Set Output compare to 250;

  TIFR4 = 0xFF;


}

//*********************************************
//  Configure Timer 3
//*********************************************
void SetupTimer3()
{
  //Clear the register
  TCCR3A = 0;
  TCCR3B = 0;

  //Set the prescaler
  TCCR3B|=(1<<CS10);// Clock/1

  //Start counting at zero and then interrupt on match
  TCCR3B|=(1<<WGM12); // Set in CTC mode

  //The value of the match.
  OCR3A=0; // Set Output compare to 250;
 
  TIFR3 = 0xFF;

}

//*********************************************
//  Configure Timer 0
//  Page 115
//*********************************************
void SetupTimer0()
{

  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0  = 0; //initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 155;// = (16*10^6) / (100*1024) - 1 = 155(must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // 1024 prescaler - page 142 of datasheet
  TCCR0B |= (1 << CS00); 
  TCCR0B |= (1 << CS02);  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

//*****************************************
//  Main Setup Function
//
//
//*****************************************
void setup() 
{
  //Disable interrupts
  cli();
  
  //Set the pin modes
  pinMode(StepA,OUTPUT);
  pinMode(DirA,OUTPUT);

  pinMode(StepB,OUTPUT);
  pinMode(DirB,OUTPUT);

  pinMode(EnableA,OUTPUT);
  pinMode(EnableB,OUTPUT);

  pinMode(A3,INPUT);
  pinMode(A4,INPUT);

  digitalWrite(EnableA, LOW);
  digitalWrite(EnableB, LOW);

  MotorA_Current.Hold = true;
  MotorB_Current.Hold = true;

  //Setup Timers
  SetupTimer3();
  SetupTimer4();
  SetupTimer0();
  delay(1000);
  
  Serial.begin(115200);

  Serial.println("This is the setup TCCR1A");
  Serial.println(TCCR1A);

  Serial.println("This is the setup TCCR3A");
  Serial.println(TCCR3A);

  Serial.println("This is the setup TCCR1B");
  Serial.println(TCCR1B);

  Serial.println("This is the setup TCCR3B");
  Serial.println(TCCR3B);

  digitalWrite(DirB,1);
  digitalWrite(DirA,1);

  //Need to start in a known position, this needs to be in steps, not mm
  MotorA_Current.Position = (signed long)394 * Steps_per_mm;
  MotorB_Current.Position = (signed long)394 * Steps_per_mm ;

  DesiredCoordinates.x = 235;
  DesiredCoordinates.y = 316;

  delay(2000);

  //enable interrupts
  sei();
  
}


//********************************************
//  Main Loop
//********************************************
void loop() 
{
  static bool FirstLoop = true;

  if(RunTask_10ms)
  {
    //Determine the x,y desired
    ReadJoystick(&DesiredCoordinates);

    #ifdef PRINT_XY
    Serial.print(DesiredCoordinates.x);
    Serial.print(",");
    Serial.print(DesiredCoordinates.y);
#endif //PRINT_XY

    //Compute the joint lengths, pass in mm
    InverseKinematics(DesiredCoordinates.x,DesiredCoordinates.y ,&Joints);

#ifdef PRINT_JOINTS
    Serial.print(",");
    Serial.print(Joints.AD);
    Serial.print(",");
    Serial.print(Joints.BD);
#endif //PRINT_JOINTS
    
    //Assign the joint length to the corresponding motor system
    MotorA_Desired.Position = (signed long)(Joints.BD * Steps_per_mm);
    MotorB_Desired.Position = (signed long)(Joints.AD * Steps_per_mm);
    
    //Compute the motion profile based on the position error
    MotionPositionControl(&MotorA_Current, &MotorA_Desired);
    MotionPositionControl(&MotorB_Current, &MotorB_Desired);

#ifdef PRINT_STEPS
    Serial.print(",");
    Serial.print(MotorA_Desired.Position);
    Serial.print(",");
    Serial.print(MotorA_Current.Position);
#endif //PRINT_JOINTS

    //Call function to covert the velocity from mm/min to timer value
    WriteStepSize(&MotorA_Current);
    WriteStepSize(&MotorB_Current);

    //The StepSpeed is used by the interrupt
    StepSpeedA = MotorA_Current.TimerMatchValue;
    StepSpeedB = MotorB_Current.TimerMatchValue;

    //Compute the current position that the machine is in.
    ForwardKinematics(&Joints,&CurrentCoordinates);

#ifdef PRINT_XY
    Serial.print(",");
    Serial.print(CurrentCoordinates.x);
    Serial.print(",");
    Serial.println(CurrentCoordinates.y);
#endif //PRINT_XY

    if(FirstLoop)
    {
      //Enable to interrupt on timer 1
      TIMSK3 |= (1<<OCIE3A); 
      ////Enable to interrupt on timer 1
      TIMSK4 |= (1<<OCIE4A);
      // 
      FirstLoop = false;
    }

    //Clear the flag that gets set by the interrupt
    //This was added at end to detect overruns of the 10ms task
    RunTask_10ms = 0;
  }
}


//*********************************************
// MotionPositionControl
//*********************************************
void MotionPositionControl(struct MovementParms *MotorCurrent, struct MovementParms *MotorDesired)
{
  signed long Position_Error;

  //Compute the error 
  //Positive value means the cable is too short, this is in steps
  Position_Error = (MotorDesired->Position) - (MotorCurrent->Position);
 
  //The cable is too short, make it longer
  if(Position_Error > ERROR_TOLERANCE)
  {
     MotorCurrent->Velocity = 1000;
     MotorCurrent->Hold = false;
  }
  //The cable is too long, make it shorter
  else if(Position_Error < -ERROR_TOLERANCE)
  {
     MotorCurrent->Velocity = -1000;
     MotorCurrent->Hold = false;
  }
  //Just the right length
  else
  {
     MotorCurrent->Velocity = 10;
     MotorCurrent->Hold = true;
  }
}

//*********************************************
// MotionVelocityControl
//*********************************************
void MotionVelocityControl(struct MovementParms *MotorCurrent, struct MovementParms *MotorDesired)
{  
  //Moving in the positive direction
  if(MotorCurrent->Velocity >= 0)
  
     //Determine if we are speeding up
     if(MotorCurrent->Velocity < MotorDesired->Velocity)
     {
        //Check to see if we can add the acceleration without overshooting
        if(MotorCurrent->Velocity < (MotorDesired->Velocity - MotorDesired->Acceleration))
        {
           MotorCurrent->Velocity += MotorDesired->Acceleration;
        }
        else
        {
           MotorCurrent->Velocity = MotorDesired->Velocity;
        }
     }
     //We are slowing down 
     else
     {
        //Check for overshoot
        if(MotorCurrent->Velocity > (MotorDesired->Velocity + MotorDesired->Acceleration))
        {
           MotorCurrent->Velocity -= MotorDesired->Acceleration;
        }
        else
        {
           MotorCurrent->Velocity = MotorDesired->Velocity;
        }
     }
  //Moving in the negative direction, all the signs flip   
  else
  {
     //Determine if we are speeding up
     if(MotorCurrent->Velocity > MotorDesired->Velocity)
     {
        //Calculate the timer for the next pulse
        if(MotorCurrent->Velocity > (MotorDesired->Velocity + MotorDesired->Acceleration))
        {
           MotorCurrent->Velocity -= MotorDesired->Acceleration;
        }
        else
        {
           MotorCurrent->Velocity = MotorDesired->Velocity;
        }
     }
     //We are slowing down 
     else
     {
        //Calculate the timer for the next pulse
        if(MotorCurrent->Velocity < (MotorDesired->Velocity - MotorDesired->Acceleration))
        {
           MotorCurrent->Velocity += MotorDesired->Acceleration;
        }
        else
        {
           MotorCurrent->Velocity = MotorDesired->Velocity;
        }
     }
  }
}

//***********************************
//  MotionAccelerationControl
//
//
//***********************************
void MotionAccelerationControl(struct MovementParms *MotorCurrent, struct MovementParms *MotorDesired)
{
   //Acceleration is a signed value
   //If positive check to make sure it is not bigger than max V
   //else check the - max speed
   
   if(MotorDesired->Acceleration >= 0)
   {
      if(MotorCurrent->Velocity < (MAX_VELOCITY + MotorDesired->Acceleration))
      {
         MotorCurrent->Velocity += MotorDesired->Acceleration; 
      }
      else
      {
        MotorCurrent->Velocity = MAX_VELOCITY;
      }
   }
   else
   {
      if(MotorCurrent->Velocity > (-MAX_VELOCITY + MotorDesired->Acceleration))
      {
         MotorCurrent->Velocity += MotorDesired->Acceleration;
      }
      else
      {
         MotorCurrent->Velocity = -MAX_VELOCITY;
      }
   }
}


//*****************
//  Read Joystick
//
//*****************
void ReadJoystick(struct Coordinates *lDesiredCoordinates)
{
  int vert;
  int horz;
  
  vert = analogRead(A3);
  horz = analogRead(A4);
  
  if(vert > 700)
  {
    if(lDesiredCoordinates->x < MAX_X)
    {
       lDesiredCoordinates->x = lDesiredCoordinates->x +1;
    }
  }
  else if(vert<  300)
  {
    if(lDesiredCoordinates->x > 2)
    {
       lDesiredCoordinates->x = lDesiredCoordinates->x -1;
    }
  }

  if(horz > 700)
  {
    if(lDesiredCoordinates->y < MAX_Y)
    {
      lDesiredCoordinates->y = lDesiredCoordinates->y +1;
    }
  }
  else if(horz<  300)
  {
    if(lDesiredCoordinates->y > 2)
    {
      lDesiredCoordinates->y = lDesiredCoordinates->y -1;
    }
  }


}