#include <Servo.h>
#define trig 3
#define echo 4
Servo sg90;
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ena 6
#define enb 11
const byte line[5]={A1,A2,A3,A4,A5};
int sensors=B00000;
int Lspeed=50;
int Rspeed=50;
int Rpid,Lpid,flag=0;;
float PID_value;
int error=0,pre_error=0;
int previous_error=0;
float P=0,I=0,D=0;
void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(ena,OUTPUT);
  pinMode(enb,OUTPUT);
  pinMode(echo,INPUT);
  //sg90.attach(5);

}

void loop() {
  scansensor();
  robotcontrol(IN1,IN2,IN3,IN4,ena,enb);
  obstacleavoid(IN1,IN2,IN3,IN4,ena,enb,14);
  Serial.println("I'm here");
    
}
boolean Sensorread(byte PinNumb)
 {return (!digitalRead(PinNumb));}
void scansensor()
{
  sensors=B00000;
  for(int i=0;i<5;i++)
    {
      int b=4-i;
      sensors=sensors+(Sensorread(line[i])<<b);
    }  
  switch(sensors){
    case B00001:
        error=-4;
        break;
    case B00011:
        error=-3;
        break;
    case B00010:
        error=-2;
        break;
    case B00110:
        error=-1;
        break;
    case B00100:
    case B01110:
        error=0;
        break;
    case B01100:
        error=1;
        break;
    case B01000:
        error=2;
        break;
    case B11000:
        error=3;
        break;
    case B10000:
        error=4;
        break;
    case B11110:   
    case B11100:
        error=100;
        break;
    case B01111:
    case B00111:
        error=101;
        break;
    case B00000:
        error=102;
        break;
    case B11111:
        error=103;
        break;
    }
    Serial.println(error);
}
void PID()
{ 
  float Kp=10,Ki=0.000,Kd=90;
  float Time = 1;
  P = error;
  I +=error*Time;
  D = (error-previous_error)/Time;
  PID_value = Kp*P+Ki*I+Kd*D;
  previous_error = error;
  Serial.println(PID_value); 
}
void motor_control()
{
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  Rpid = constrain(Rspeed + PID_value,0,200); 
  Lpid = constrain(Lspeed - PID_value,0,200);
  analogWrite(ena,Rpid);
  analogWrite(enb,Lpid);
  Serial.print("Rpid: ");
  Serial.println(Rpid);
  Serial.print("Lpid: ");
  Serial.println(Lpid);
}
void motornospeed(byte in1, byte in2,byte direct)
{
  switch(direct)
  {
   case 0:
     digitalWrite(in1,LOW);
     digitalWrite(in2,LOW);
     break;
   case 1:
     digitalWrite(in1,LOW);
     digitalWrite(in2, HIGH);
     break;
   case 2:
     digitalWrite(in1,HIGH);
     digitalWrite(in2,LOW);
     break;
  }
}
void robotspeed(byte in1, byte in2, byte in3, byte in4, byte action)
{ 
  switch(action)
  { 
    case 0://dung
     motornospeed(in1,in2,0);
     motornospeed(in3,in4,0);
     break;
    case 1://tien
     motornospeed(in1,in2,1);
     motornospeed(in3,in4,1);
     break;
    case 2://lui
     motornospeed(in1,in2,2);
     motornospeed(in3,in4,2);
     break;
    case 3://re trai
     motornospeed(in1,in2,1);
     motornospeed(in3,in4,0);
     break;
    case 4: //re phai
     motornospeed(in1,in2,0);
     motornospeed(in3,in4,1);
     break;
    case 5: // tien trai
     motornospeed(in1,in2,1);
     motornospeed(in3,in4,2);
     break;
    case 6:// tien phai
     motornospeed(in1,in2,2);
     motornospeed(in3,in4,1);
     break;
  } 
}
void robotcontrol(byte in1, byte in2, byte in3, byte in4,byte ENA,byte ENB)
{
    if(error==102)
    { 
      robotspeed(IN1,IN2,IN3,IN4,2);
      delay(20);
      pre_error=previous_error;
      do{
        nopid(pre_error);
        scansensor();
      }while(error==102);
    }
    else if(error==101)
    {
      robotspeed(IN1,IN2,IN3,IN4,0);
      delay(100); 
      analogWrite(ena,120);
      analogWrite(enb,120);
      robotspeed(IN1,IN2,IN3,IN4,2);
      delay(200);
      robotspeed(IN1,IN2,IN3,IN4,4);
      delay(400);
    }
    else if(error==100)
    {
      robotspeed(IN1,IN2,IN3,IN4,0);
      delay(100);
      analogWrite(ena,120);
      analogWrite(enb,120);
      robotspeed(IN1,IN2,IN3,IN4,2);
      delay(200);
      robotspeed(IN1,IN2,IN3,IN4,3);
      delay(400);
    }
    else if(error==103)
    {
      robotspeed(IN1,IN2,IN3,IN4,0);
      delay(500);
    }
    else{
        PID();
        motor_control();
      }
}
void nopid(int pre)
{
  switch(pre)
  {
    case 0:
    case 1:
    case -1:
       robotspeed(IN1,IN2,IN3,IN4,1);
       analogWrite(ena,70);
       analogWrite(enb,70);
       break;
    case 2:
       robotspeed(IN1,IN2,IN3,IN4,1);
       analogWrite(ena,100);
       analogWrite(enb,60);
       break;
    case 3:
    case 4:
       robotspeed(IN1,IN2,IN3,IN4,1);
       analogWrite(ena,100);
       analogWrite(enb,20);
       break;
    case -2:
       robotspeed(IN1,IN2,IN3,IN4,1);
       analogWrite(ena,60);
       analogWrite(enb,100);
       break;
    case -3:
    case -4:
       robotspeed(IN1,IN2,IN3,IN4,1);
       analogWrite(ena,20);
       analogWrite(enb,100);      
       break;
    }
}
int measuredistance()
{
  unsigned long t;  
  int d;
  digitalWrite(trig, 0);
  delayMicroseconds(2);
  digitalWrite(trig, 1);
  delayMicroseconds(5);
  digitalWrite(trig, 0);
  t=pulseIn(echo,HIGH);
  d=int(t*0.017);//khoang cach vat can cm;
  return d;
 }

 int obstacleavoid(byte M1, byte M2, byte M3, byte M4, byte spa, byte spb, byte maxdistance)
 {
  int frontd=measuredistance();
   if (frontd>maxdistance)
  {
   return 0;
   }
   else
   {
    robotspeed(M1,M2,M3,M4,2);
    analogWrite(spa,130);
    analogWrite(spb,130);
    delay(300);
    robotspeed(M1,M2,M3,M4,0);
    delay(1000); 
    robotspeed(M1,M2,M3,M4,3);
    analogWrite(spa,100);
    delay(400);
    robotspeed(M1,M2,M3,M4,0);
    delay(20);
    robotspeed(M1,M2,M3,M4,1);
    analogWrite(spa,70);
    analogWrite(spb,70);
    delay(200);
    robotspeed(M1,M2,M3,M4,0);
    delay(20);
    robotspeed(M1,M2,M3,M4,4);
    analogWrite(spb,100);
    delay(500);
    robotspeed(M1,M2,M3,M4,0);
    delay(20);
    robotspeed(M1,M2,M3,M4,1);
    analogWrite(spa,70);
    analogWrite(spb,70);
    delay(500);
    robotspeed(M1,M2,M3,M4,0);
    delay(20);
    robotspeed(M1,M2,M3,M4,4);
    analogWrite(spb,100);
    delay(250);
    robotspeed(M1,M2,M3,M4,0);
    delay(20);
    do{
      robotspeed(M1,M2,M3,M4,1);
      analogWrite(spa,70);
      analogWrite(spb,70);
      scansensor();
      }while(error==102);
    robotspeed(M1,M2,M3,M4,2);
    analogWrite(spa,130);
    analogWrite(spb,130);
    delay(100);
    robotspeed(M1,M2,M3,M4,3);
    analogWrite(spa,100);
    analogWrite(spb,70);
    delay(100);
    //delay(300);
    return 0;
    }
   }
