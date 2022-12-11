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
#define  MAX_VELOCITY  30000
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
unsigned int WriteStepSize(struct MovementParms *Motor);

//Create a structure for movement data
struct MovementParms
{
  signed int Position;
  signed int Velocity;
  signed int Acceleration;
  bool Direction;
};

//Create instance of three structures 
volatile struct MovementParms MotorA_Desired;
volatile struct MovementParms MotorA_Current;
volatile struct MovementParms MotorB_Desired;
volatile struct MovementParms MotorB_Current;


//Static Variables that are accessed in interrupts
volatile unsigned int abs_Current_Velocity = 0;
volatile unsigned int Counter = 0;
volatile unsigned int StepSpeedA = 0;
volatile unsigned int StepSpeedB = 0;
volatile signed long timestep = 0;
volatile bool RunTask_10ms = 0;

//**************************************
// Timer1 - 16 bit timer Interrupt  
// Motor A
// Setup for ATMEGA2560
//**************************************
ISR(TIMER1_COMPA_vect)  
{   
   if(StepSpeedA < 64000)
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
  
     //Step once
     digitalWrite(StepA,1);
     digitalWrite(StepA,0);
   }
   //Interrupt occurs when timer matches this value
   //Larger velocity means smaller OCR1A value
   //StepSpeed is calculated using another function and then stored 
   //in a global variable to be accessed by the interrupt routine
   OCR1A = StepSpeedA;
}

//**************************************
// Timer3 - 16 bit timer Interrupt 
// Motor B 
// Setup for ATMEGA2560
//**************************************
ISR(TIMER3_COMPA_vect)  
{   
   if(StepSpeedB < 64000)
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
  
     //Step once
     digitalWrite(StepB,1);
     digitalWrite(StepB,0);
   }
   //Interrupt occurs when timer matches this value
   //Larger velocity means smaller OCR1A value
   //StepSpeed is calculated using another function and then stored 
   //in a global variable to be accessed by the interrupt routine
   OCR3A = StepSpeedB;
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
unsigned int WriteStepSize(struct MovementParms *Motor)
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
  // Conversion Factor
  //  20000 mm      1 min      360 deg     1 rev       step          4,444 step
  // ----------- x -------- x -------- x -------- x ------------ =  ---------
  //     min        60 sec      rev        120 mm     0.225 deg       sec
  //
  //   16,000,000 cycles      1 sec            3,600 cycles
  //   ----------------- x --------------- = -----------
  //       1 sec             4,444 step          
  //  16,000,000/4,444    = 3,600 clock cycles  (20000 mm/min)
  //  16,000,000/2,222    = 7,200 clock cycles  (10000 mm/min)
  //  16,000,000/666      = 24,024 clock cycles  (3000 mm/min)
  
  //For some reason the prescale set to zero causes problems, so 8 is the minimum
  //which means the clock is 16,000,000/8=2,000,000
  //  2,000,000/4,444   = 450 clock cycles  (20000 mm/min)
  //  2,000,000/2,222    = 900 clock cycles  (10000 mm/min)
  //  2,000,000/666      = 3003 clock cycles (3000 mm/min)

  //To compute position it is 
  // Conversion Factor
  //    360 deg     1 rev       step          13.33 steps
  //  -------- x -------- x ------------ =  ---------
  //     rev        120 mm     0.225 deg        mm

  
  
  //Check to prevent overflows, StepSpeed is 16 bit
  if(Motor->Velocity > 700 || Motor->Velocity < -700)
  {
     //This is derived from the table above using a Power trendline
     StepSpeed = (signed int)(9048739L/(abs(Motor->Velocity)));
  }
  //This is like setting it to zero speed
  else
  {
     StepSpeed = 65000;
  }
  return StepSpeed;
}
//*******************************************
//  Configure Timer 1 - 16 bit timer
//  Atmega2560
//  Timer/Counter 1, 3, 4, and 5
//*******************************************
void SetupTimer1()
{

  //Set the prescaler
  TCCR1B|=(1<<CS10);// Clock/1

  //Start counting at zero and then interrupt on match
  TCCR1B|=(1<<WGM12); // Set in CTC mode

  //The value of the match.
  OCR1A=250; // Set Output compare to 250;

  //Enable to interrupt on timer 1
  TIMSK1 |= (1<<OCIE1A); 
}

//*********************************************
//  Configure Timer 3
//*********************************************
void SetupTimer3()
{
  //Set the prescaler
  TCCR3B|=(1<<CS10);// Clock/1

  //Start counting at zero and then interrupt on match
  TCCR3B|=(1<<WGM12); // Set in CTC mode

  //The value of the match.
  OCR3A=250; // Set Output compare to 250;

  //Enable to interrupt on timer 1
  TIMSK3 |= (1<<OCIE3A); 
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
  //Set the pin modes
  pinMode(StepA,OUTPUT);
  pinMode(DirA,OUTPUT);

  pinMode(StepB,OUTPUT);
  pinMode(DirB,OUTPUT);

  delay(50);

  //Disable interrupts
  cli();
  //Setup Timers
  SetupTimer3();
  SetupTimer1();
  SetupTimer0();
  //enable interrupts
  sei();
  Serial.begin(115200);
  
}


//********************************************
//  Main Loop
//********************************************
void loop() 
{

  if(RunTask_10ms)
  {
    MotorA_Desired.Acceleration = 2000;
    MotorB_Desired.Acceleration = 2000;
    
    //Call this function to compute the current velocity,this
    //takes into account the acceleration
    MotionVelocityControl(&MotorA_Current, &MotorA_Desired);
    MotionVelocityControl(&MotorB_Current, &MotorB_Desired);

    //Call function to covert the velocity from mm/min to timer value
    StepSpeedA = WriteStepSize(&MotorA_Current);
    StepSpeedB = WriteStepSize(&MotorB_Current);

    
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