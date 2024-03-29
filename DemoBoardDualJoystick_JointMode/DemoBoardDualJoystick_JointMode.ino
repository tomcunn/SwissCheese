//**************************************
//*   Dual Joystick Embedded Demo Board
//*   05/25/2023
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

#define SOLENOID 9
#define TOGGLE_SWITCH  44

#define JOYSTICK_RIGHT_Y  A3
#define JOYSTICK_RIGHT_X  A5
#define JOYSTICK_LEFT_Y   A4


//Function prototypes
void  SetupTimer1              (void);
void  SetupTimer0              (void);
void  WriteStepSize            (struct MovementParms *Motor);
void  ReadJoystick             (void);

static int counter=0;

//394mm is the initial joint length.

#define Bx_distance 610 //
//#define Steps_per_mm (27.03f  * 2)  //Added the x2 because we changed to a higher frequency get 50% duty cycle. Now we interrupt on the rising and falling edge of the pulse. 
#define Steps_per_mm 54.06f

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


struct MovementParms LinearMotionX;
struct MovementParms LinearMotionY;
struct Joint_Distances CurrentJoints;
struct Joint_Distances DesiredJoints;
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

  pinMode(JOYSTICK_RIGHT_Y,INPUT);
  pinMode(JOYSTICK_RIGHT_X,INPUT);
  pinMode(JOYSTICK_LEFT_Y,INPUT);
  pinMode(TOGGLE_SWITCH,INPUT_PULLUP);

  pinMode(SOLENOID,OUTPUT);

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
  digitalWrite(SOLENOID,0);

  //Need to start in a known position, this needs to be in steps, not mm
  MotorA_Current.Position = (signed long)394 * Steps_per_mm;
  MotorB_Current.Position = (signed long)394 * Steps_per_mm ;

  CurrentCoordinates.x = 235;
  CurrentCoordinates.y = 316;
  Serial.println("SHOULD ONLY BE RUN ONCE"); //mm

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
    //Determine the velocity that is requested by the joystick
    ReadJoystick();

    //Call function to convert the velocity from mm/min to timer value
    WriteStepSize(&MotorA_Current);
    WriteStepSize(&MotorB_Current);

    //The StepSpeed is used by the interrupt
    StepSpeedA = MotorA_Current.TimerMatchValue;
    StepSpeedB = MotorB_Current.TimerMatchValue;

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



//**********************************************************************
//  Read Joystick
//
//  The goal of this is to change linear velocity and not coordinates
//**********************************************************************
void  ReadJoystick()
{
  int left_joy;
  int right_joy;
  int DEADBAND = 50;
  int CENTERX = 500;
  int CENTERY = 500;
  int GAIN  = 18;  //12

  int toggle_state;


  left_joy = analogRead(JOYSTICK_LEFT_Y);
  right_joy = analogRead(JOYSTICK_RIGHT_Y);
  
  //Create a proportional scheme for motion

  if(left_joy > (CENTERY + DEADBAND))
  {
    MotorB_Current.Velocity = -1*GAIN * (left_joy -(CENTERY + DEADBAND));    //mm/min
  }
  else if(left_joy< (CENTERY - DEADBAND))
  {
    MotorB_Current.Velocity = GAIN * ((CENTERY - DEADBAND) - left_joy); 
  }
  else
  {
    MotorB_Current.Velocity = 0;
  }

  if(right_joy > (CENTERX + DEADBAND))
  {
     MotorA_Current.Velocity = -1 *  GAIN * (right_joy - (CENTERX + DEADBAND));    //mm/min
  }
  else if(right_joy < (CENTERX - DEADBAND))
  {
     MotorA_Current.Velocity =GAIN * ((CENTERX - DEADBAND) - right_joy); 
  }
  else
  {
     MotorA_Current.Velocity = 0;    
  }


    //Turn the Motors off if the velocity command is low enough
  if((MotorA_Current.Velocity >= -10) && (MotorA_Current.Velocity <= 10 ))
  {
    MotorA_Current.Hold = true;
  }
  else
  {
    MotorA_Current.Hold = false;
  }

  if((MotorB_Current.Velocity >= -10) && (MotorB_Current.Velocity <= 10 ))
  {
    MotorB_Current.Hold = true;
  }
  else
  {
    MotorB_Current.Hold = false;
  }

   Serial.print( left_joy);
   Serial.print(",");
   Serial.print( right_joy); 
   Serial.print(",");
   Serial.print(MotorA_Current.Velocity);
   Serial.print(",");
   Serial.print(MotorB_Current.Velocity);

  toggle_state = digitalRead(TOGGLE_SWITCH);
 
  Serial.print(",");
  Serial.println(toggle_state);

  if(toggle_state)
  {
     digitalWrite(SOLENOID,1);
  }
  else
  {
      digitalWrite(SOLENOID,0);
  }
}

