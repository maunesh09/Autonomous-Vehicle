#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <math.h>
#include <MechaQMC5883.h>
#include "HC_SR04.h"

//optional for RF EMERGENCY START STOP
//#include <RH_ASK.h> // Include RadioHead Amplitude Shift Keying Library
//#include <SPI.h> // Include dependant SPI Library

// Create Amplitude Shift Keying Object
//RH_ASK rf_driver;  // RF object for incoming & outgoing signal

QMC5883LCompass compass;
Servo steer_servo;

int isr;
int dist = 0; // maximum distance want to cover.

float headingFinal;
float headinga[50];
float XScaled[50];
float YScaled[50];

int left_intr = 0;
int right_intr = 0;
float radius_of_wheel = 6.8; //Cm  //Measure the radius of your wheel and enter it here in cm
volatile byte rotation; // variale for interrupt fun must be volatile
float timetaken, rpm, rps, dtime;
float v;
int distanceCov, D;
int predistanceCov = 0;
unsigned long pevtime;

float diff_angle=0; // difference between required direction and current direction

int angle=0;

float headingDegrees = 0;    // current direction
float actualdegrees= 96;   // required direction

const int D0 = 3;       //Raspberry pin 21    LSB
const int D1 = 4;       //Raspberry pin 22
const int D2 = 5;       //Raspberry pin 23
const int D3 = 6;       //Raspberry pin 24    MSB

const int M0 = 40;       //Raspberry Pin 25    LSB
const int M1 = 42;       //Raspberry Pin 26    MSB

const int buzzer = 11;
int a,b,c,d,e,f,mode,data;

// LEFT SIDE MOTOR

const int EnableL = 22; // PWM
const int LM_pos = 24;      
const int LM_neg = 26;

 //RIGHT SIDE MOTOR
 
const int EnableR = 28; // PWM
const int RM_pos = 30;      
const int RM_neg = 32;


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600); // Mega to RPi
  Serial1.begin(9600); // For RF Tx Rx
  Serial2.begin(9600); // HC-05 to mega
  HC.HCSetup(8,9); //Trig, Echo
   
  rotation = rpm = pevtime = 0; //Initialize all variable to zero
  rps=0;
  steer_servo.attach(10); //our servo pin
  steer_servo.write(96);
  delay(50);

  compass.init();
   
  attachInterrupt(digitalPinToInterrupt(2), Left_ISR, CHANGE); //Left_ISR is called when left wheel sensor is triggered. On Pin D2 of megha
//    attachInterrupt(digitalPinToInterrupt(3), Right_ISR, CHANGE);//Right_ISR is called when right wheel sensor is triggered. On Pin D3 of megha
// for data 
  pinMode(D0, INPUT_PULLUP); // LSB
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D3, INPUT_PULLUP); // MSB

//for mode selection
  pinMode(M0, OUTPUT); // LSB
  pinMode(M1, OUTPUT); // MSB

  pinMode(EnableL , OUTPUT); // Left Side Motor Enable
  pinMode(LM_pos , OUTPUT);  // Left Side Motor +ve
  pinMode(LM_neg , OUTPUT);  // Left Side Motor -ve
  
  pinMode(EnableR , OUTPUT); // Right Side Motor Enable
  pinMode(RM_pos , OUTPUT);  // Right Side Motor +ve
  pinMode(RM_neg , OUTPUT); // Right Side Motor -ve

  pinMode(buzzer, OUTPUT);

  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(500);
}

/*
void Mode()
{
   e = digitalRead(M0);
   f = digitalRead(M1);
  
   mode = 2*f+e;
}
*/
void Data()
{
   a = digitalRead(D0);
   b = digitalRead(D1);
   c = digitalRead(D2);
   d = digitalRead(D3);

   data = 8*d+4*c+2*b+a;
}

float getAvg1(float array[])
{
  float average = 0;
  for (int i = 0; i < 10; i++) //agv=10
  {
    average += array[i];
  }
  average /= 10; //agv;
  return average;
}

float getAvg2(float array[])
{
  float average = 0;
  for (int i = 0; i < 10; i++){
    average += array[i];
  }
  average /= 10;
  return average;
}

void getDegree()
{
  int x,y,z;
  for (int k = 0; k <10; k++) 
  {
    for (int i = 0; i <10; i++)
    {
      compass.read();
      x = compass.getX();
      y = compass.getY();
      z = compass.getZ();

      for (int i = 0; i <100; i++)
        {
          XScaled[i]=x;
          YScaled[i]=y;
        }
    
      float xFinal = getAvg2(XScaled);
      float yFinal = getAvg2(YScaled);
      float heading = atan2(yFinal, xFinal);

      if(heading < 0)
        {
          heading += 2*PI;
        }
      if(heading > 2*PI)
        {
          heading -= 2*PI;
        }
      float headingDegrees = heading * 180/M_PI; 
  
      headinga[k]=headingDegrees;
    }
    //delay(1);
  headingFinal = getAvg1(headinga);

  Serial.print("Heading: ");
  Serial.println(headingFinal);
  delay(1);  
}
}

char path;

void loop() 
{
  if ( Serial2.available() > 0)
   {
      mode = Serial2.parseInt();
      Serial.println(mode);
      //Mode();
      HC.HCLoop(); // HCSR04 Distance checking.
      if( mode == 0)
      {
        breakapplied();
        Serial.println(" Give me path");
        digitalWrite(M0, LOW);
        digitalWrite(M1, LOW);
      }
    
      if( mode == 1) // IP based mode
      { 
        digitalWrite(M0, HIGH);
        digitalWrite(M1, LOW);
        Serial.println("Mode 1 is Seleced");
        
        Data();
        if(data==0)
          {
            Forward();
            Serial.println("Going Forward");
          }
        else if(data==1)
          {
            Right1();
            Serial.println("Right1");
          }
        else if(data==2)
          {
            Right2();
            Serial.println("Right2");
          }
         
        else if(data==3)
          {
            Right3();
            Serial.println("Right3");
          }
         
        else if(data==4)
          {
            Left1();
            Serial.println("Left1");
          }
        
        else if(data==5)
          {
            Left2();
            Serial.println("Left2");
          }
        
        else if(data==6)
          {
            Left3();
            Serial.println("Left3");
          }
      
        else if (data>6)
          {
            Stop();
            Serial.println("Stop");
          }

        
      }
    // Mode 1 ends here 
      
      if (mode == 2 ) // floorplan based mode
      {
          digitalWrite(M0, LOW);
          digitalWrite(M1, HIGH);
        
          v = radius_of_wheel * rpm * 0.104; //0.033 is the radius of the wheel in meter
          distanceCoMv = (2*3.141*radius_of_wheel) * (left_intr/40); //40 is size of chasis change it accordingly
          D = distanceCov;
          Serial.println(" Mode 2 is selected");
          Serial2.write(" Select Path");
    
         //checkpath();
          if(Serial2.available())
            {
              path = Serial.read();
            }
              
           switch(path)
            {
               case 'a': Serial.println(" path 1 selected");
                         checkpath();
                         breakrelease();
                         /*
                         digitalWrite(LM_pos,HIGH); 
                         digitalWrite(RM_pos,HIGH);
                         digitalWrite(LM_neg,LOW); 
                         digitalWrite(RM_neg,LOW);
                         delay(20);
                         */
          
                         //distanceCov = 0;
                         //distanceCov = (2*3.141*radius_of_wheel) * (left_intr/40);
                         actualdegrees= 96;
          
                         if(distanceCov > 25)
                          {
                            breakapplied();
                          }
                          
                         break;
          
               case 'b': Serial.println(" path 2 selected");
                         checkpath();
                         breakrelease();
                          
                         distanceCov = 0;
                         distanceCov = (2*3.141*radius_of_wheel) * (left_intr/40);
                    
                         if(distanceCov > 15 && distanceCov <20)
                           {
                             actualdegrees = 45;
                             steer_servo.write(45);
                             //delay(10);
                           }
                         if(distanceCov > 20 && distanceCov <25)                    
                           { 
                             actualdegrees = 96;
                             steer_servo.write(96);
                             //delay(10);
                           }         
                         if(distanceCov > 25 && distanceCov < 35 )
                           {
                             actualdegrees = 45;
                             steer_servo.write(45);
                           }
                         if (distanceCov >35)
                           {
                             breakapplied();
                           }                             
                         break;
                    
               case 'c': Serial.println("path 3 selected");
                         checkpath();
                         breakrelease();
                          
                         distanceCov = 0;
                         distanceCov = (2*3.141*radius_of_wheel) * (left_intr/40);
                         
                         if(distanceCov < 15)
                          {
                            actualdegrees = 45;
                            steer_servo.write(45);
                            //delay(10);
                          }
                         if(distanceCov > 20 && distanceCov <25 )                    
                          { 
                            actualdegrees = 96;
                            steer_servo.write(96);
                            //delay(50);        
                          }
                         if(distanceCov > 25 && distanceCov < 30)
                          {
                            actualdegrees = 125;
                            steer_servo.write(125);
                            //delay(10);
                          }  
                         if(distanceCov > 30 && distanceCov <40)
                          {
                            actualdegrees = 125;
                            steer_servo.write(125);
                            // delay(10);
                          }
                         if(distanceCov > 40)
                          {
                            //move_stop();
                            breakapplied();
                          }
                         break;
              }
     }
   }

  else
  {
    Serial.println("Select Mode");
    Serial.println("Select Mode");
    delay(50);
  }
}
void checkpath()
{
  if (HC.ObsDistance < 8)
  {
    //breakapplied
    digitalWrite(LM_pos, LOW);
    digitalWrite(LM_neg, LOW);
    digitalWrite(RM_pos, LOW);
    digitalWrite(RM_neg, LOW);
    steer_servo.write(96);

    Serial.println("Path is not clear");

    for(int m = 0; m<1; m++)
    {
      digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);
      delay(500);
    /*digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);
      delay(500);
    */
    }
  }
   else
   {
    Serial.println("Path is clear");
    digitalWrite(LM_pos, HIGH);
    digitalWrite(LM_neg, LOW);
    digitalWrite(RM_pos, HIGH);
    digitalWrite(RM_neg, LOW);
   } 
    
    delay(100);
  }

void Left_ISR()
{
  left_intr++;
  delay(10);
}

void Right_ISR()
{
  right_intr++; 
  delay(10);
  
  rotation++;
  dtime=millis();
  if(rotation>=40)
  {
    timetaken = millis()-pevtime; //timetaken in millisec 
    rpm=(1000/timetaken)*60;    //formulae to calculate rpm
    pevtime = millis();
    rotation=0;
  }
}


void checkangle()
{
  getDegree();
  diff_angle=actualdegrees-headingFinal;
  Serial.print(diff_angle);

  if(diff_angle <= 2 )
  {
    diff_angle=abs(diff_angle);
    rightangle(diff_angle); 
    //Serial.println("i am in right");
    delay(10);  
  }
  
  else if (diff_angle >= 2 )
  {
    diff_angle=abs(diff_angle);
    leftangle(diff_angle);
    //Serial.println("i am in left");
    delay(10);
  }
}

void rightangle(int angledata)
{
    Serial2.println("Right Turn");
    angledata = angledata/2;
    angledata = angledata*200;
    for (int i =0; i<= angledata; i++) // if this dosn't work then try i = "last known servo angle" instead of 0
     {
       steer_servo.write(angledata);
       delay(10);
     } 
}

void leftangle(int angledata)
{
   Serial2.println("Left Turn");
   angledata = angledata/2;      // "/2"
   angledata = angledata*200;
   for (int i = 0; i <= (angledata); i++) // if this dosn't work then try i = "last known servo angle" instead of 0
     { 
       steer_servo.write(angledata);
      // delay(10);
     }
}


void breakapplied()
{
  digitalWrite(EnableL , LOW); // or analogWrite(EnableL,0);
  digitalWrite(LM_pos , LOW);
  digitalWrite(LM_neg , LOW);
  
  digitalWrite(EnableR , LOW); // or analogWrite(EnableR,0);
  digitalWrite(RM_pos , LOW);
  digitalWrite(RM_neg , LOW);
  //steer_servo.write(96);
  delay(50);
}

void breakrelease()
{
  digitalWrite(EnableL , HIGH); //or analogWrite(EnableL,255);
  digitalWrite(LM_pos , HIGH);
  digitalWrite(LM_neg , LOW);

  digitalWrite(EnableR , HIGH); // or analogWrite(EnableR,255);
  digitalWrite(RM_pos , HIGH);
  digitalWrite(RM_neg , LOW);
  //steer_servo.write(96);
  delay(50);
}

void Forward()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,255);
  
}


void Backward()
{
  digitalWrite(LM_pos, HIGH);
  digitalWrite(LM_neg, LOW);
  analogWrite(EnableL,255);

  digitalWrite(RM_pos, HIGH);
  digitalWrite(RM_neg, LOW);
  analogWrite(EnableR,255);
  
}

void Stop()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,0);
  
}

void Left1()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,160);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,255);
  
}

void Left2()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,90);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,255);
  
}


void Left3()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,50);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,255);
  
}

void Right1()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,160);  
  
}
void Right2()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,90);   
  
}

void Right3()
{
  digitalWrite(LM_pos, LOW);
  digitalWrite(LM_neg, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(RM_pos, LOW);
  digitalWrite(RM_neg, HIGH);
  analogWrite(EnableR,50);   
  
}
