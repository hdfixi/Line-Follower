#include <QTRSensors.h>
//#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

const uint8_t SensorCount=16;
// ultrason  #include<NewPing.h>

#define trig 12
#define echo 11
//unsigned int d=5; //distance
//sonor
//NewPing sonar(trig,echo, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
//screen
LiquidCrystal_I2C lcd(0x27,16,2);
/*****************************/
uint16_t s[SensorCount];
QTRSensorsRC qtr((unsigned char[]){A0, A1, A2, A3, A4, A5, A6,A7,A8,A9,A10,A11,A12,A13,A14,A15}, SensorCount);
int position;
int n=0,p=0,m=0;
//PID
int right_speed,left_speed;
float kp=0.05,kd=0.10,ki=0,P,D,I;
float PIDvalue,lasterror,error;
int left_base,right_base;
//motors
int rightF=8;
int rightR=9;
int leftF=11;
int leftR=10;
//time
unsigned int current_time,last_time,lunch_time;

//kp=0.07;kd=0.12; high speed 
//kp=0.06;kd=0.11; medium speed 


int led=48;
void setup() {
  
Serial.begin(9600);
 lcd.init();
  lcd.clear();
  
  int tst=0;
  //moteurs
  pinMode(rightF,OUTPUT);
  pinMode(leftF,OUTPUT);
  pinMode(rightR,OUTPUT);
  pinMode(leftR,OUTPUT);
  pinMode(39,OUTPUT);
  pinMode(24,OUTPUT);
  pinMode(22,INPUT);
  pinMode(led,OUTPUT);
  pinMode(3,OUTPUT);
  digitalWrite(3, LOW); 

    pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  //qtr.setEmitterPin(2);
 forward(234567);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
   for (int i = 0 ; i < 120; i++)
  {
    qtr.calibrate();
    tst=1;
    delay(20);
  }
  digitalWrite(LED_BUILTIN, LOW); 
  current_time=millis();
  int x=0;
  while(1){
  x=digitalRead(22);
  Serial.println(x);
    if(x==HIGH){
        lunch_time=millis();
      if(lunch_time-current_time>200)
      break;
    }
  }
  delay(800);
 // forwardf(200);
  last_time=current_time;
//Serial.begin(9600);
}

void loop() {
   //position = qtr.readLineM(s,QTR_EMITTERS_ON,0,false);
  current_time=millis();
   //LCD(sonar.ping_cm());
  
 if(n==0&&((distance()>30&&current_time-last_time>1000)||current_time-last_time>15000)){//15000 security!!!!!!!!!!!!! take care 
   lcd.noBacklight();
   //LCD(distance());
   stp(500);
   forward(200);
   //while(1);
   last_time=current_time;
  n=1;
 }
 
else if(n==1&&s[15]+s[14]+s[13]+s[12]+s[11]+s[10]>=4000&&current_time-last_time>=800){//awil 90°
  stp(200);
  right_fblstou(265);
  stp(100);
  last_time=current_time;
  n=2;
  }
  else if(n==2&&s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]>=8000&&current_time-last_time>=1000){//condensateur
    stp(50);
    forward(250);
    right(50);
      //stp(200);
      last_time=current_time;
    n=3;
  
    }
  else if(n==3&&p==0&&s[9]+s[10]+s[11]+s[12]+s[13]+s[14]+s[15]>=5000&&current_time-last_time>=1000){//XX
      right(280);
      stp(100);
      last_time=current_time;
    p=1;
    }
     else if(n==3&&p==1&&s[10]+s[9]+s[8]+s[7]+s[6]+s[5]<2000&&current_time-last_time>=600){
      leftsafe(170);
      stp(100);
      last_time=current_time;
    p=2;
    }
      else if(n==3&&p==2&&current_time-last_time>=1000){
      stp(100);
      last_time=current_time;
    p=3;
    n=4;
    }


    
    else if(n==4&&s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]>=6000&&current_time-last_time>=1500){//d5lna f cercle
    stp(50);
    left(150);
    stp(100);
      last_time=current_time;
    n=5;
    }
    else if(n==5&&s[5]+s[4]+s[3]+s[2]+s[1]+s[0]>=4000&&current_time-last_time>=800){//5rjna ml cercle
    stp(50);
    left_fblstou(130);
    stp(100);
      last_time=current_time;
    n=6;}
    else if(n==6&&s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]>=7000&&current_time-last_time>=1500){//el coins loul "3fsa"
    stp(150);
    forward(300);
    //digitalWrite(3,HIGH);
    stp(100);
    afsaleft(1300);
    forward(70);
    //digitalWrite(3,LOW);
    stp(200);
    last_time=current_time;
    n=7;
    }
    else if(n==7&&s[5]+s[4]+s[3]+s[2]+s[1]+s[0]>=4000&&current_time-last_time>=2500){// el d5la t3 triangle (!!!!! rak bdlt ek w9t 5000)
    stp(100);
    left(220);
    stp(50);
    forward(50);
  
    stp(100);
    last_time=current_time;
    n=8;
    
    }
    else if(n==8&&m==0&&current_time-last_time>=800){
      m=1;
    }
    else if(n==8&&m==1&&s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]>=7000&&current_time-last_time>=2500){
    stp(150);
    forward(340);
    //digitalWrite(3,HIGH);
    stp(100);
    afsalefts(1200);
    forward(70);
    //digitalWrite(3,LOW);
    stp(100);
    last_time=current_time;
    n=9;
    }  
    else if(n==9&&s[15]+s[14]+s[13]+s[2]+s[1]+s[0]>=4000&&current_time-last_time>=1500){
      stp(50);
      last_time=current_time;
      n=10;
    }
       else if(n==10&&s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]<=2000&&current_time-last_time>=1700){
      forward(250);
      right(24);
      lcd.clear();
      lcd.backlight();
      lcd.print("HUNTER X HUNTER");
      stp(12000);
      lcd.clear();
      lcd.noBacklight();
      forward(250);
      last_time=current_time;
      n=11;
    }
       else if(n==11&&s[9]+s[8]+s[7]+s[6]>=2000&&s[15]+s[14]+s[13]+s[12]+s[2]+s[1]+s[0]<=2000&&current_time-last_time>=1700){
      stp(100);
      last_time=current_time;
      n=12;
    }
    else if(n==12&&s[15]+s[14]+s[13]+s[12]+s[11]+s[10]>=4000&&current_time-last_time>=800){//awil 90°
  stp(200);
  right_fblstou(200);
  stp(100);
  last_time=current_time;
  n=13;
  }
else if(n==13&&s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]>=7000&&current_time-last_time>=1000){
  forward(250);
  stp(100000);
  n=14;
}


    
else if(n==1){
  kp=0.04;kd=0.075;
      left_base=120;
      right_base=120;
      PID_special();
      forwardPID(); 
}
else if(n==2){
  kp=0.03;kd=0.055;
      left_base=80;
      right_base=80;
      PID();
      forwardPID(); 
}

else if(n==3&&p==0){
     kp=0.03;kd=0.06;
      left_base=80;
      right_base=80;
      PID();
      forwardPID(); 
}
else if(n==3&&p==1){
     kp=0.02;kd=0.05;
      left_base=50;
      right_base=50;
      PID_touhami();
      forwardPID(); 
}
else if(n==3&&p==2){
     kp=0.02;kd=0.05;
      left_base=50;
      right_base=50;
      PID_touhami();
      forwardPID(); 
}
else if(n==4){
     kp=0.03;kd=0.055;
      left_base=80;
      right_base=80;
      PID();
      forwardPID(); 
}
else if(n==5){
  kp=0.03;kd=0.05;
      left_base=50;
      right_base=50;
      PID();
      forwardPID(); 
}
else if(n==6){
 kp=0.03;kd=0.055;
      left_base=70;
      right_base=70;
      PID();
      forwardPID(); 
}
else if(n==7){
  kp=0.03;kd=0.055;
      left_base=70;
      right_base=70;
      PID();
      forwardPID(); 
      //digitalWrite(3,HIGH);
}
else if(n==8&&m==0){
  kp=0.03;kd=0.05;
      left_base=70;
      right_base=70;
      PID();
      forwardPID();  
}
else if(n==8&&m==1){
  kp=0.02;kd=0.05;
      left_base=70;
      right_base=70;
      PID_touhamiS();
      forwardPID();  
}
else if(n==18){
  kp=0.01;kd=0.015;
      left_base=50;
      right_base=50;
      PID_touhamiS();
      forwardPID();  
}
else if(n==9){
   kp=0.03;kd=0.055;
      left_base=70;
      right_base=70;
      PID();
      forwardPID(); 
}
else if(n==10){
   kp=0.02;kd=0.045;
      left_base=80;
      right_base=80;
      PID_touhamiN();
      forwardPID(); 
}
else if(n==11){
   kp=0.02;kd=0.045;
      left_base=80;
      right_base=80;
      PID_touhamiN();
      forwardPID(); 

}
else if(n==12){
  kp=0.04;kd=0.075;
      left_base=120;
      right_base=120;
      PID();
      forwardPID();
}
else if(n==13){
  kp=0.02;kd=0.05;
      left_base=50;
      right_base=50;
      PID();
      forwardPID(); 
}




}


void PID(){
  position = qtr.readLine(s);
  error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}

void PID_noir(){
  position = qtr.readLine(s,QTR_EMITTERS_ON,1,true);
  error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_special(){
  bool v=false;
  position = qtr.readLine(s,QTR_EMITTERS_ON,0,v);
  error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_touhami(){
  bool v=true;
  position = qtr.readLineM(s,QTR_EMITTERS_ON,0,v);
  error=position-2500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_touhamiN(){
  bool v=true;
  position = qtr.readLineM(s,QTR_EMITTERS_ON,1,v);
  error=position-2500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_touhamiS(){
  bool v=false;
  position = qtr.readLineM(s,QTR_EMITTERS_ON,0,v);
  error=position-2500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void forwardPID(){
  
  analogWrite(rightF,right_speed);
   analogWrite(leftF,left_speed);
  analogWrite(rightR,0);
  analogWrite(leftR,0);

}
void forward(int x){
  
analogWrite(rightF,250);
   analogWrite(leftF,250);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void forward1(int x){
  
analogWrite(rightF,113);
   analogWrite(leftF,110);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void forwardf(int x){
  
analogWrite(rightF,160);
   analogWrite(leftF,160);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void right(int x){
  
  analogWrite(rightF,150);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void rightsafe(int x){
  
  analogWrite(rightF,100);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void right_fblstou(int x){
  
  analogWrite(rightF,120);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,120);
  delay(x);
}
void right_fblstousafe(int x){
  
  analogWrite(rightF,70);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,70);
  delay(x);
}
void left_fblstou(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,120);
  analogWrite(rightR,120);
  analogWrite(leftR,0);
  delay(x);
}
void left(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,120);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void leftsafe(int x){
  
  analogWrite(rightF,0);
  analogWrite(leftF,100);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void afsaleft(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,80);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void afsalefts(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,100);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void stp(long int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void back(int x){
   analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,100);
  analogWrite(leftR,100);
  delay(x);
}
void stp1(){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
}


void LCD(int x){
  lcd.backlight(); 
  lcd.setCursor(2,0);   
  lcd.print(x);
}

int distance(){
  lcd.backlight();
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
int  duration = pulseIn(echo, HIGH);
 int distanceCm = duration * 0.034 / 2;
  int distanceInch = duration * 0.0133 / 2;
return distanceCm;
}
