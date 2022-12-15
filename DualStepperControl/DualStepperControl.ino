//**************************************
//*   Stepper Motor Control
//*
//*   Cloned from the AI project
//*   12/11/2022
//*   Cunningham
//***************************************

#include <avr/io.h>
#include <avr/interrupt.h>

//Hardware Assignments
#define  MAX_VELOCITY  90000
#define  MAX_ACCELERATION  2000

#define StepA    A0
#define DirA     A1
#define EnableA  38

#define StepB    46
#define DirB     48
#define EnableB  A8


//Function prototypes
void  SetupTimer1(void);
void  SetupTimer0(void);
void  MotionAccelerationControl(struct MovementParms *MotorCurrent, struct MovementParms *MotorDesired);
void  MotionVelocityControl    (struct MovementParms *MotorCurrent, struct MovementParms *MotorDesired);
void  WriteStepSize(struct MovementParms *Motor);
static int counter=0;

//Create a structure for movement data
struct MovementParms
{
  signed int Position;
  signed int Velocity;
  signed int Acceleration;
  unsigned int TimerMatchValue;
  bool Direction;
};

//Create instance of three structures 
volatile struct MovementParms MotorA_Desired;
volatile struct MovementParms MotorA_Current;
volatile struct MovementParms MotorB_Desired;
volatile struct MovementParms MotorB_Current;


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
     //Check the direction pin and determine the position
     if(MotorB_Current.Direction == 1)
     {
        digitalWrite(DirB,1);
        MotorB_Current.Position++;
     }
     else
     {
       digitalWrite(DirB,0);
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
  digitalWrite(EnableA, LOW);
  digitalWrite(EnableB, LOW);

  MotorA_Current.Position = 0;
  MotorB_Current.Position = 0;

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

    MotorA_Desired.Acceleration = 50;
    MotorB_Desired.Acceleration = 50;

    MotorA_Desired.Velocity = 25000;
    MotorB_Desired.Velocity = 25000;
    
    //Call this function to compute the current velocity,this
    //takes into account the acceleration
    MotionVelocityControl(&MotorA_Current, &MotorA_Desired);
    MotionVelocityControl(&MotorB_Current, &MotorB_Desired);

    //Call function to covert the velocity from mm/min to timer value
    WriteStepSize(&MotorA_Current);
    WriteStepSize(&MotorB_Current);

    StepSpeedA = MotorA_Current.TimerMatchValue;
    StepSpeedB = MotorB_Current.TimerMatchValue;

    Serial.print(MotorA_Current.Position);
    Serial.print(",");
    Serial.println(MotorB_Current.Position);

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
// MotionVelocityControl
//
// Uses all global structures
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