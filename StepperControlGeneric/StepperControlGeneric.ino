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
#define  EncoderChannel_A  2
#define  EncoderChannel_B  3
#define  Step 12
#define  Dir  11
#define  MAX_VELOCITY  30000
#define  MAX_ACCELERATION  2000

//Function prototypes
void  SetupTimer1(void);
void  SetupTimer0(void);

void  WriteStepSize(int Speed);
void  MotionVelocityControl(void);
void  MotionAccelerationControl(void);

//Create a structure for movement data
struct MovementParms
{
  signed int Position;
  signed int Velocity;
  signed int Acceleration;
  bool Direction;
};

//Create instance of three structures 
volatile struct MovementParms Desired;
volatile struct MovementParms Current;


//Static Variables that are accessed in interrupts
volatile unsigned int abs_Current_Velocity = 0;
volatile unsigned int Counter = 0;
volatile unsigned int StepSpeed = 0;
volatile signed long timestep = 0;
volatile bool RunTask_10ms = 0;

//**************************************
//  Interrupt used to generate the stepper 
//  pulses
//  The lower the address the higher is the priority level
//  Vector Address 12 
//**************************************
ISR(TIMER1_COMPA_vect)  
{   
   if(StepSpeed < 64000)
   {
     //Check the direction pin and determine the position
     if(Current.Direction == 1)
     {
        digitalWrite(Dir,1);
        Current.Position++;
     }
     else
     {
       digitalWrite(Dir,0);
       Current.Position--;
     }
  
     //Step once
     digitalWrite(Step,1);
     digitalWrite(Step,0);
   }
   //Interrupt occurs when timer matches this value
   //Larger velocity means smaller OCR1A value
   //StepSpeed is calculated using another function and then stored 
   //in a global variable to be accessed by the interrupt routine
   OCR1A = StepSpeed;
}

//**************************************
//  10 ms Task Rate
//  Creates a periodic Task 
//  Vector Address 15
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
void WriteStepSize(signed int Speed)
{
  //Set the direction pin based on the sign of the speed
  if(Speed > 0)
  {
     Current.Direction = 1;
  }
  else
  {
     Current.Direction = 0;
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
  if(Speed > 700 || Speed < -700)
  {
     //This is derived from the table above using a Power trendline
     StepSpeed = (signed int)(9048739L/(abs(Speed)));
  }
  //This is like setting it to zero speed
  else
  {
     StepSpeed = 65000;
  }
}
//*******************************************
//  Configure Timer 1
//*******************************************

void SetupTimer1()
{
  //TCCRnA/B- Stands for Timer/Counter Control Registers.
  TCCR1A = 0;
  TCCR1B = 0;

  //TCNTn- Stands for Timer/Counter Register.
  TCNT1 = 0; // initialize the counter from 0

  //OCRnA/B- Stands for Output Compare Register.
  OCR1A = 65000; // sets the counter compare value

  //TIMSKn- Stands for Timer/Counter Mask In Registers.
  TCCR1B |= (1<<WGM12); // enable the CTC mode
  TCCR1B |= (1<<CS11); // 1/8 Prescale, 0 prescale was not working

  TIMSK1 |= (1<<OCIE1A); 
}

//*********************************************
//  Configure Timer 0
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
  bool run_program = 0;
  //Set the pin modes
  pinMode(Step,OUTPUT);
  pinMode(Dir,OUTPUT);
 

  delay(50);

  //Disable interrupts
  cli();
  //Setup Timers
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
    Desired.Acceleration = 2000;
    
    //Call this function to compute the current velocity,this
    //takes into account the acceleration
    MotionVelocityControl();

    //Call function to covert the velocity from mm/min to timer value
    WriteStepSize(CurrentCart.Velocity);
    
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
void MotionVelocityControl(void)
{  
  //Moving in the positive direction
  if(Current.Velocity >= 0)
  
     //Determine if we are speeding up
     if(Current.Velocity < Desired.Velocity)
     {
        //Check to see if we can add the acceleration without overshooting
        if(Current.Velocity < (Desired.Velocity - Desired.Acceleration))
        {
           Current.Velocity += Desired.Acceleration;
        }
        else
        {
           Current.Velocity = Desired.Velocity;
        }
     }
     //We are slowing down 
     else
     {
        //Check for overshoot
        if(Current.Velocity > (Desired.Velocity + Desired.Acceleration))
        {
           Current.Velocity -= Desired.Acceleration;
        }
        else
        {
           Current.Velocity = Desired.Velocity;
        }
     }
  //Moving in the negative direction, all the signs flip   
  else
  {
     //Determine if we are speeding up
     if(Current.Velocity > Desired.Velocity)
     {
        //Calculate the timer for the next pulse
        if(Current.Velocity > (Desired.Velocity + Desired.Acceleration))
        {
           Current.Velocity -= Desired.Acceleration;
        }
        else
        {
           Current.Velocity = Desired.Velocity;
        }
     }
     //We are slowing down 
     else
     {
        //Calculate the timer for the next pulse
        if(Current.Velocity < (Desired.Velocity - Desired.Acceleration))
        {
           Current.Velocity += Desired.Acceleration;
        }
        else
        {
           Current.Velocity = Desired.Velocity;
        }
     }
  }
}

//***********************************
//  MotionAccelerationControl
//
//
//***********************************
void MotionAccelerationControl(void)
{
   //Acceleration is a signed value
   //If positive check to make sure it is not bigger than max V
   //else check the - max speed
   
   if(Desired.Acceleration >= 0)
   {
      if(Current.Velocity < (MAX_VELOCITY + Desired.Acceleration))
      {
         Current.Velocity += Desired.Acceleration; 
      }
      else
      {
        Current.Velocity = MAX_VELOCITY;
      }
   }
   else
   {
      if(Current.Velocity > (-MAX_VELOCITY + Desired.Acceleration))
      {
         Current.Velocity += Desired.Acceleration;
      }
      else
      {
         Current.Velocity = -MAX_VELOCITY;
      }
   }
}
