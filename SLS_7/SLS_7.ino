/*              SLS Team 7
  Ahmed Zaki, Mohamed Ashraf, Omar Momtaz
  Coach :Ahmed Tarek Fahmy
*/

#include <QTRSensors.h>

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

#define dir1 30
#define pwm1 3
#define dir2 32
#define pwm2 5

#define Minspeed 80
#define Maxspeed 100

#define turn_speed_max 0
#define turn_speed_med 60
#define turn_speed_min 70

#define turn_speed_Rev 50

int status_num=0;

int total_right=0, total_left=0;

int inverse=0;

unsigned int position;

int i=0;

QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
  
unsigned int sensorValues[NUM_SENSORS], SensorValues_temp[NUM_SENSORS], Arranged_Sensors[NUM_SENSORS];

void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
 
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
}

void loop()
{
  position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, 0);
  
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  
  for (i=0;i<8;i++)
  {
  Serial.print(Arranged_Sensors[i]); 
  Serial.print('\t');
  }

  Arrange_Sensors();
  
Green ();

     // these are the states
      
  if(inverse==1)  //Inverse
  {
      Follow_Max (Arranged_Sensors[7]);
      
      if ((Arranged_Sensors[0]==2) || (Arranged_Sensors[0]==3) || (Arranged_Sensors[0]==4) || (Arranged_Sensors[0]==5))
      inverse=0;
  }

  else
  {
            status_num=Status ();
               
          if(status_num==0)                   //White
        {
        
        }
          else if(status_num==1)             //Line Following
        {
            Follow_Max (Arranged_Sensors[0]);
        }
          else if(status_num==2)              //Crossing
        {
        Follow_Max (Arranged_Sensors[0]);
        }
      
          else if(status_num==3)              //Ninety Right
        {
             Right_Reverse ();
             
        }
      
          else if(status_num==4)              //Ninety Left
        {
            left_Reverse ();
            
        }
        
       Serial.print(status_num);
 
  }
    
 Serial.println();
}


void forward (int sped) //Makes the robot go by moving the two motors forward.
{
  analogWrite(pwm2,sped);
  analogWrite(pwm1,sped);
  
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);

  Serial.print("F ");
  Serial.print(sped);
}

void right (int sped) //Makes the robot go right by stopping right motor and moving the left motor.
{
  analogWrite(pwm2,sped);
  analogWrite(pwm1,Minspeed);
  
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);

  Serial.print("R ");
  Serial.print(sped);
}

void Right_Reverse () //Makes the robot go right by moving right motor backwards and moving the left motor forwards.  
{
  analogWrite(pwm2,turn_speed_Rev);
  analogWrite(pwm1,turn_speed_Rev);
  
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);

  Serial.print("RR ");
}

void left (int sped) //Makes the robot go left by stopping left motor and moving the right motor.
{
  analogWrite(pwm2,Minspeed);
  analogWrite(pwm1,sped);
  
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);

  Serial.print("L ");
  Serial.print(sped);
}

void left_Reverse () //Makes the robot go left by moving left motor backwards and moving the right motor forwards.
{
  analogWrite(pwm2,turn_speed_Rev);
  analogWrite(pwm1,turn_speed_Rev);
  
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,HIGH);

  Serial.print("LR ");
}


void backward (int sped) //Makes the robot go by moving the two motors backward.
{
  analogWrite(pwm2,sped);
  analogWrite(pwm1,sped);
  
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);
   
  Serial.print("B ");
  Serial.print(sped);
}

void stop_pp () //Stops the two motors //Makes the robot go by moving the two motors backward.
{
 int sped=0;
  sped=0;
  analogWrite(pwm2,sped);
  analogWrite(pwm1,sped);
}


void Follow_Max (int max_num ) //Line Following code
{

 if(max_num==0) 
  {
    Right_Reverse ();  
  }
  else if(max_num==1)
  {
    right (turn_speed_med);
  }
  else if(max_num==2) 
  {
    right (turn_speed_min); 
  }
 
  else if(max_num==3) 
  {
    forward (Maxspeed);
  }
  else if(max_num==4) 
  {
     forward (Maxspeed);  
  }
  else if(max_num==5)
  {
     left (turn_speed_min); 
  }
  else if(max_num==6) 
  {
     left (turn_speed_med);  
  }
  else if(max_num==7) 
  {
     left_Reverse ();  
  }
  
}

void Arrange_Sensors() //Arranges the number of the sensors descendingly according to their values
{ 
  int  i,j;
  for (i=0;i<8;i++)
  SensorValues_temp[i]=sensorValues[i];
  
  for (i=0;i<8;i++)
  {
    for (j=0;j<8;j++)
    {
      if(SensorValues_temp[j]>=SensorValues_temp[Arranged_Sensors[i]])
      {
        Arranged_Sensors[i] =j; 
      }
    }
    SensorValues_temp[Arranged_Sensors[i]]=0;
  }
}

int Status () //According to this the robot moves
{
  int diff1=0,diff2=0,diff_max=0,diff3=0,diff4=0,diff_max2=0;
  int total=0;
  
  total_right=sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3];
  total_left=sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7];
  total=total_right+total_left;
 
if (total > 6200)
  return (2);
else if (total > 5200) // Inverse
{
    
        if ((Arranged_Sensors[7]==2) || (Arranged_Sensors[7]==3) || (Arranged_Sensors[7]==4) || (Arranged_Sensors[7]==5))
        {
          inverse =1;
          return (5);
        }
     else 
     {
          if(total_right>total_left)
          return (4);
          else
          return (3);
     }   
}
else
{ 
  if (sensorValues[Arranged_Sensors[0]]<750) 
    return (0);
  else
  {
     diff1=sensorValues[Arranged_Sensors[1]]-sensorValues[Arranged_Sensors[2]];
     diff2=sensorValues[Arranged_Sensors[0]]-sensorValues[Arranged_Sensors[1]];
    
     if(diff1>diff2)
        diff_max=diff1;
     else 
        diff_max=diff2;
     
     if (diff_max>100)
        return (1);
     else 
     {
          if(total_right>total_left)
          return (3);
          else
          return (4);
     }   
  }   
 }
}


void Green () //We are working on the function which detects the green square
{

}

void Obstacle () // We are working on the obstacle function
{

}  


