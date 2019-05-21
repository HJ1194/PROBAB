#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

Adafruit_LSM303 lsm;
const float Pi = 3.141592;

int left_wheel = 2;
int left_wheel_power = 3;
int right_wheel = 4;
int right_wheel_power = 5;
//-----------------------------------ON_OFF_BLUE---------------------------
#define ON_OFF_BT_RX 12
#define ON_OFF_BT_TX 13

char ON_OFF;
char ON_OFF_W =0;
int BT_COUNTER =0;
SoftwareSerial ON_OFF_BT(ON_OFF_BT_RX, ON_OFF_BT_TX);
//-------------------------------------------------------------------------
//------------------------------------COMPASS------------------------------
LSM303 compass;

void BRING_HUMAN_HEADING();
void BRING_STROLLER_HEADING();
void GET_HEADING();
void TURN_LEFT_STABLE();
void TURN_RIGHT_STABLE();
int HEADING;
int HUMAN_HEADING;
int STROLLER_HEADING;
int MAP_HUMAN_HEADING;

//-------------------------------------------------------------------------
//-----------------------------------SONIC---------------------------------
void SONIC_SETUP();
void DETECT_OBSTACLE();
void TURN_LEFT();
void TURN_RIGHT();
void STOP();
void GO();
void AUTO_SWING_MODE();

const int ECHO_LEFT = 22;
const int TRIG_LEFT = 23;

const int ECHO_MID = 24;
const int TRIG_MID = 25;
 
const int ECHO_RIGHT = 28;
const int TRIG_RIGHT = 29;

const int STOP_DIST = 10;
const int STANDARD_DIST = 30;

float DISTANCE_LEFT, DISTANCE_MID, DISTANCE_RIGHT;
float Duration_LEFT, Duration_MID, Duration_RIGHT; 
//-------------------------------------------------------------------------

void setup (void)
{
    Serial.begin(9600);
    ON_OFF_BT.begin(9600);
    pinMode(left_wheel, OUTPUT);
    pinMode(left_wheel_power ,OUTPUT);
    pinMode(right_wheel,OUTPUT);
    pinMode(right_wheel_power, OUTPUT);
//------------------------------SONIC----------------------------------------
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_LEFT, OUTPUT);
  
    pinMode(ECHO_MID, INPUT);
    pinMode(TRIG_MID, OUTPUT);
  
    pinMode(ECHO_RIGHT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
//----------------------------------------------------------------------------
//-----------------------------SPI--------------------------------------------
    SPI.begin (); // SPI 통신 초기화
    digitalWrite(SS, HIGH); // 슬레이브가 선택되지 않은 상태로 유지
    // 안정적인 전송을 위해 분주비를 높여 전송 속도를 낮춤
    SPI.setClockDivider(SPI_CLOCK_DIV16);
//----------------------------------------------------------------------------
//-------------------------COMPASS--------------------------------------------
    Wire.begin();
    compass.init();
    compass.enableDefault();
      
    compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
    compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
//----------------------------------------------------------------------------
}

void ON();
void OFF();
void MOTOR_COMPASS();
void EMERGENCY_STOP();
void GO_BACK();




//------AUTO_SWING MODE------
char AUTO_SWING_W = 0;
char AUTO_SWING=0;
int AUTO_SWING_COUNTER=0;
void AUTO_SWING_MODE();
void AUTO_SWING_COUNT();


void COUNTER();
//----------------------------

//---------ASIST_MODE---------
void UP();
void DOWN();
void GO_BACK();

void ASIST_MODE_COUNT();
int ASIST_COUNT = 0 ;
int LEVEL_COUNT = 0;
int GO_BACK_COUNT = 0;

//----------------------------
// 
void loop ()
{  
   
   Serial.println("START");
     COUNTER();
     SONIC_SETUP();
     EMERGENCY_STOP();

     //======================OFF=================================
      if(BT_COUNTER == 0)
      {
        OFF();
        ON_OFF = ON_OFF_BT.read();
      }
      //======================ON===============================
      else if(BT_COUNTER == 1 )
      {
        while(BT_COUNTER == 1)
        {
          COUNTER();
          ON();
          
          if(BT_COUNTER != 1)
          {
            break;
          }
        ON_OFF_W = 0;
      }
      }
      //====================AUTO_SWING==================================
      //====================AUTO_SWING_OFF==============================
      if(AUTO_SWING_COUNTER == 0)
      {
        OFF();
      }
      //======================AUTO_SWING_ON=============================
      else if(AUTO_SWING_COUNTER == 1 )
      {
        while(AUTO_SWING_COUNTER == 1)
        {
          COUNTER();
          AUTO_SWING_MODE();
        
          if(AUTO_SWING_COUNTER !=1)
          {
            break;
          }  
       
        }
        
        AUTO_SWING_W = 0;
      }
      if(ASIST_COUNT == 1)
      {
        UP();
      }
      
      if(GO_BACK_COUNT == 1)
      {
        GO_BACK();
      }
   
  }

void COUNTER()
{
   if(ON_OFF_BT.available())
    {
      ON_OFF = ON_OFF_BT.read();
     
      if(ON_OFF == 'a')

       {
            BT_COUNTER++;
            ON_OFF = 0;
           
            if(BT_COUNTER > 1)
            {
              BT_COUNTER = BT_COUNTER - 2;
            }           
       } 
        else;
   
      if(ON_OFF == 'b')
        {
          AUTO_SWING_COUNTER++;
 
           if(AUTO_SWING_COUNTER > 1)
            {
               AUTO_SWING_COUNTER = AUTO_SWING_COUNTER - 2;
            }
        } 
        else;

     if(ON_OFF == 'c')
       {
          LEVEL_COUNT++;
          ASIST_COUNT = 1;
           if(LEVEL_COUNT >5)
          {
            LEVEL_COUNT = LEVEL_COUNT -1;
          }
 
       }
     if(ON_OFF == 'd')
       {
          LEVEL_COUNT--;
          ASIST_COUNT = 1;
          if(LEVEL_COUNT < -5)
          {
           LEVEL_COUNT = LEVEL_COUNT + 1;
          }
       }
       if(ON_OFF == 'f')
       {
        GO_BACK_COUNT++;
 
           if(GO_BACK_COUNT > 1)
            {
               GO_BACK_COUNT = GO_BACK_COUNT - 2;
            }  
        }
   }
}

void ASIST_MODE_COUNT()
{
    if(ON_OFF_BT.available())
    {
      ON_OFF = ON_OFF_BT.read();
      
       if(ON_OFF == 'c')
       {
          LEVEL_COUNT++;
          ASIST_COUNT = 1;
          if(LEVEL_COUNT >5)
          {
            LEVEL_COUNT = LEVEL_COUNT -1;
          }
       }
         if(ON_OFF == 'd')
       {
          LEVEL_COUNT--;
          ASIST_COUNT = 1;
          if(LEVEL_COUNT < -5)
          {
            LEVEL_COUNT = LEVEL_COUNT + 1;
          }
       }
       if(ON_OFF == 'e')
       {
        ASIST_COUNT = 2;
       }
       if(ON_OFF == 'f')
       {
        GO_BACK_COUNT++;
 
           if(GO_BACK_COUNT > 1)
            {
               GO_BACK_COUNT = GO_BACK_COUNT - 2;
            }  
        }
     }
}
void GO_BACK()
{
  while(1)
  {
    digitalWrite(left_wheel,LOW);
    analogWrite(left_wheel_power,255);
    digitalWrite(right_wheel,LOW);
    analogWrite(right_wheel_power,255);
    ASIST_MODE_COUNT();
    if(GO_BACK_COUNT != 1)
    {
      break;
    }
    Serial.println("GOBACK");
  }
}
int DOWN_COUNT = 0;

void UP()
{
  while(1)
  {
    if(LEVEL_COUNT < 6 && LEVEL_COUNT >= 0)
    {
    digitalWrite(left_wheel,HIGH);
    analogWrite(left_wheel_power,120 + 27*LEVEL_COUNT);
    digitalWrite(right_wheel,HIGH);
    analogWrite(right_wheel_power,120 + 27*LEVEL_COUNT);
    ASIST_MODE_COUNT();
    if(ASIST_COUNT == 2)
    {
      LEVEL_COUNT = 0 ;
      break;
    }
     Serial.print("LEVEL_COUNT : ");
     Serial.print(LEVEL_COUNT);
    Serial.println("UP");
    }
    if(LEVEL_COUNT > -6 && LEVEL_COUNT < 0)
    {
      //LEVEL_COUNT = LEVEL_COUNT * -1;
      DOWN_COUNT = LEVEL_COUNT * -1;
      digitalWrite(left_wheel,LOW);
      analogWrite(left_wheel_power,120 + 27*DOWN_COUNT);
      digitalWrite(right_wheel,LOW);
      analogWrite(right_wheel_power,120 + 27*DOWN_COUNT);
      ASIST_MODE_COUNT();
      if(ASIST_COUNT == 2)
      {
        LEVEL_COUNT = 0 ;
        break;
      }
     Serial.print("LEVEL_COUNT : ");
     Serial.print(LEVEL_COUNT);
     Serial.println("DOWN");
  
    }
  } 
}


void AUTO_SWING_COUNT()
{
   if(ON_OFF_BT.available())
    {
      ON_OFF = ON_OFF_BT.read();
                 
      if(ON_OFF == 'b')

        {
          AUTO_SWING_COUNTER++;
  
           if(AUTO_SWING_COUNTER > 1)
            {
               AUTO_SWING_COUNTER = AUTO_SWING_COUNTER - 2;
            }
        } 
        else;
  }
}

void AUTO_SWING_MODE()
{
  Serial.println("AUTO_SWING_MODE_ON");
  for(float i =0; i<360; i++)
  {
    Serial.println("UPUP");

    float reading = sin(i*(PI/180)-(PI/2))*127+127;

    digitalWrite(left_wheel,HIGH);
    analogWrite(left_wheel_power,reading);
    digitalWrite(right_wheel,HIGH);
    analogWrite(right_wheel_power,reading);
    Serial.println(reading);
    AUTO_SWING_COUNT();
      
      if(AUTO_SWING_COUNTER != 1)
      {
        break;
      }
    
    delay(8);
    }
    
   
   for(float i =0; i<=340; i++)
    {
    Serial.println("DOWNDOWN");

    float reading = sin(i*(PI/180)-(PI/2))*127+127;

    digitalWrite(left_wheel,LOW);
    analogWrite(left_wheel_power,reading);
    digitalWrite(right_wheel,LOW);
    analogWrite(right_wheel_power,reading);
    Serial.println(reading);
    AUTO_SWING_COUNT();
       
       if(AUTO_SWING_COUNTER != 1)
        {
          break;
        }
       
    delay(8);
    }
}
void OFF()
{
  digitalWrite(left_wheel,HIGH);
  analogWrite(left_wheel_power,0);
  digitalWrite(right_wheel,HIGH);
  analogWrite(right_wheel_power,0);

  delay(100);
}
void ON()
{ 
    BRING_STROLLER_HEADING();
    MOTOR_COMPASS();
    SONIC_SETUP();
    DETECT_OBSTACLE();
    Serial.print("LEFT    : ");
    Serial.println(DISTANCE_LEFT);  
    Serial.print("MID     : ");
    Serial.println(DISTANCE_MID);
    Serial.print("RIGHT   : ");
    Serial.println(DISTANCE_RIGHT);
    Serial.println("--------------");
    Serial.print("HEADING : ");
    Serial.println(HEADING);

    
    // BRING_HUMAN_HEADING();
     //BRING_STROLLER_HEADING();
     //MOTOR_COMPASS();
     //GET_HEADING();
     //DETECT_OBSTACLE();
     
}

//--------------------------------------------------------SONIC--------------------------------------------------------------------
void EMERGENCY_STOP()
{
   if(DISTANCE_LEFT < STOP_DIST || DISTANCE_MID < STOP_DIST || DISTANCE_RIGHT < STOP_DIST) // 긴급 정지
    {
      STOP();
      delay(1000);
      Serial.println("//////////////////////////////////////STOP/////////////////////////////////////////");
        while(DISTANCE_LEFT < STOP_DIST || DISTANCE_MID < STOP_DIST || DISTANCE_RIGHT < STOP_DIST)
        { 
          SONIC_SETUP();
          STOP();
          Serial.println("///////////////////////////////////STOP/////////////////////////////////////");         
          delay(200);
        }
    }
}
void DETECT_OBSTACLE()
{
  Serial.print("DISTANCE_LEFT = ");
  Serial.println(DISTANCE_LEFT);
  Serial.print("DISTANCE_MID = ");
  Serial.println(DISTANCE_MID);
  Serial.print("DISTANCE_RIGHT = ");
  Serial.println(DISTANCE_RIGHT);

  GO();
    if(DISTANCE_LEFT < STOP_DIST || DISTANCE_MID < STOP_DIST || DISTANCE_RIGHT < STOP_DIST) // 긴급 정지
    {
      STOP();
      delay(1000);
      Serial.println("//////////////////////////////////////STOP/////////////////////////////////////////");
        while(DISTANCE_LEFT < STOP_DIST || DISTANCE_MID < STOP_DIST || DISTANCE_RIGHT < STOP_DIST)
        { 
          SONIC_SETUP();
          STOP();
          Serial.println("///////////////////////////////////STOP/////////////////////////////////////");         
          delay(200);
        }
    }
    else
    {/*
      if(DISTANCE_LEFT > STANDARD_DIST && DISTANCE_MID > STANDARD_DIST && DISTANCE_RIGHT > STANDARD_DIST) // 안 걸렸을 때 
      {
        GO();
        Serial.println("//////////////////////////////////////////GO///////////////////////////////////////");
      }
*/
      
       if(DISTANCE_LEFT > STANDARD_DIST && DISTANCE_MID > STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST) //오 걸렸을 때
      {
        STOP();
        Serial.println("//////////////////////////////////////STOP///////////////////////////////////////////////");
        delay(1000);
        SONIC_SETUP();
         
        while(DISTANCE_LEFT > STANDARD_DIST && DISTANCE_MID > STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST)
        { 
          SONIC_SETUP();
          TURN_LEFT();
          Serial.println("///////////////////////////////////STOP_TURN_LEFT/////////////////////////////////////");         
          delay(200);
        }
      }

      
      else if(DISTANCE_LEFT > STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT > STANDARD_DIST) // 중 걸렸을 때 
      {
        STOP();
        Serial.println("//////////////////////////////////////STOP///////////////////////////////////////////////");
        delay(1000);
        SONIC_SETUP();

        while(DISTANCE_LEFT > STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT > STANDARD_DIST)
        { 
          SONIC_SETUP();
          TURN_LEFT();
          Serial.println("///////////////////////////////////STOP_TURN_LEFT/////////////////////////////////////");         
          delay(200);
        }
      }

      
      else if(DISTANCE_LEFT > STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST) // 오 중 걸렸을 때 
      {
        STOP();
        Serial.println("//////////////////////////////////////STOP///////////////////////////////////////////////");
        delay(1000);
        SONIC_SETUP();
         
        while(DISTANCE_LEFT > STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST)
        { 
          SONIC_SETUP();
          TURN_LEFT();
          Serial.println("///////////////////////////////////STOP_TURN_LEFT/////////////////////////////////////");         
          delay(200);
        }
      }

      
      else if(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID > STANDARD_DIST && DISTANCE_RIGHT > STANDARD_DIST) // 왼 걸렸을 때 
      {
        STOP();
        Serial.println("//////////////////////////////////////STOP///////////////////////////////////////////////");
        delay(1000);
        SONIC_SETUP();
         
        while(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID > STANDARD_DIST && DISTANCE_RIGHT > STANDARD_DIST)
        { 
          SONIC_SETUP();
          TURN_RIGHT();
          Serial.println("///////////////////////////////////STOP_TURN_RIGHT/////////////////////////////////////");         
          delay(200);
        }
      }

      
      else if(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID > STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST) // 왼 오 걸렸을 때 
      {
        STOP();
        Serial.println("///////////////////////////////////STOP/////////////////////////////////////");         
        delay(1000);
        
        while(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID > STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST)
        { 
          SONIC_SETUP();
          STOP();
          Serial.println("///////////////////////////////////STOP/////////////////////////////////////");         
          delay(200);
        }
        
      }
        
      else if(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT > STANDARD_DIST) // 왼 중 걸렸을 때 
      {
        STOP();
        Serial.println("//////////////////////////////////////STOP///////////////////////////////////////////////");
        delay(500);
        SONIC_SETUP();
         
        while(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT > STANDARD_DIST)
        { 
          SONIC_SETUP();
          TURN_RIGHT();
          Serial.println("///////////////////////////////////STOP_TURN_RIGHT/////////////////////////////////////");         
          delay(200);
        }
      }
      
      else if(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST) // 다 걸렸을 때 
      {
        STOP();
        Serial.println("///////////////////////////////////STOP/////////////////////////////////////");         
        delay(1000);

        while(DISTANCE_LEFT < STANDARD_DIST && DISTANCE_MID < STANDARD_DIST && DISTANCE_RIGHT < STANDARD_DIST)
        { 
          SONIC_SETUP();
          STOP();
          Serial.println("///////////////////////////////////STOP/////////////////////////////////////");         
          delay(200);
        }
      }
      
      
    }
}

void GO()
{
  digitalWrite(left_wheel,HIGH);
  analogWrite(left_wheel_power,200);
  digitalWrite(right_wheel,HIGH);
  analogWrite(right_wheel_power,200);  
}

void STOP()
{
  digitalWrite(left_wheel,HIGH);
  analogWrite(left_wheel_power,0);
  digitalWrite(right_wheel,HIGH);
  analogWrite(right_wheel_power,0);
}

void TURN_LEFT_STABLE()
{
  digitalWrite(left_wheel,HIGH);
  analogWrite(left_wheel_power,50);
  digitalWrite(right_wheel,HIGH);
  analogWrite(right_wheel_power,200);  
}

void TURN_RIGHT_STABLE()
{
    digitalWrite(left_wheel,HIGH);
  analogWrite(left_wheel_power,200);
  digitalWrite(right_wheel,HIGH);
  analogWrite(right_wheel_power,50);
}
void TURN_LEFT()
{
  digitalWrite(left_wheel,HIGH);
  analogWrite(left_wheel_power,0);
  digitalWrite(right_wheel,HIGH);
  analogWrite(right_wheel_power,200);  
}

void TURN_RIGHT()
{
    digitalWrite(left_wheel,HIGH);
  analogWrite(left_wheel_power,200);
  digitalWrite(right_wheel,HIGH);
  analogWrite(right_wheel_power,0);
}
void SONIC_SETUP()
{
  digitalWrite(TRIG_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_LEFT, LOW);
  DISTANCE_LEFT = pulseIn(ECHO_LEFT, HIGH);
  
  digitalWrite(TRIG_MID, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_MID, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_MID, LOW);
  DISTANCE_MID = pulseIn(ECHO_MID, HIGH);
  
  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_RIGHT, LOW);
  DISTANCE_RIGHT = pulseIn(ECHO_RIGHT, HIGH);
  
  DISTANCE_LEFT = DISTANCE_LEFT / 29 / 2;
  DISTANCE_MID = DISTANCE_MID / 29 / 2;
  DISTANCE_RIGHT = DISTANCE_RIGHT / 29 / 2;

}

//-----------------------------------------------COMPASS------------------------------------------------

/*void GET_HEADING()
{
   HEADING = MAP_HUMAN_HEADING - STROLLER_HEADING;
   
   if(HEADING > 180);
      HEADING -= 360;
     Serial.println("-------360");
 

   if(HEADING < -180);
      HEADING += 360;
     Serial.println("+++++++360");


   
   Serial.print("HEADING : ");
   Serial.println(HEADING);

    if(-30 < HEADING && HEADING < 30)
    {
      GO();
      Serial.println("///////////////////////////////GO///////////////////////////////////////");
    }
    else if(HEADING <= -30)
    {
      TURN_LEFT();
      Serial.println("///////////////////////////////TURN_LEFT///////////////////////////////////////");
    }
    else if(HEADING >= 30)
    {
      TURN_RIGHT();
      Serial.println("///////////////////////////////TURN_RIGHT///////////////////////////////////////");
    }
}
*/
void BRING_HUMAN_HEADING()
{
      int data = Serial.read(); // 데이터 입력 확인
      digitalWrite(SS, LOW); // 슬레이브를 선택한다.
      HUMAN_HEADING = SPI.transfer(0);
      MAP_HUMAN_HEADING = map(HUMAN_HEADING,0,255,0,360);
      digitalWrite(SS, HIGH); // 슬레이브 선택을 해제한다.
      Serial.print("HUMAN_HEADING : ");
      Serial.println(MAP_HUMAN_HEADING);
    
}
  
void BRING_STROLLER_HEADING()
{
  //---------------------HUMAN_HEADING------------------------------
  int data = Serial.read(); // 데이터 입력 확인
      digitalWrite(SS, LOW); // 슬레이브를 선택한다.
      HUMAN_HEADING = SPI.transfer(0);
      MAP_HUMAN_HEADING = map(HUMAN_HEADING,0,255,0,360);
      digitalWrite(SS, HIGH); // 슬레이브 선택을 해제한다.
  //-----------------------------------------------------------------
  //-------------------STROLLER_HEADING------------------------------
  
  //compass.read();                                   이거 원래 있던거
  //STROLLER_HEADING = compass.heading();             이것도 원래있던거

  lsm.read();
  float STROLLER_HEADING = (atan2((int)lsm.magData.y, (int)lsm.magData.x) * 180) / Pi;
  if(STROLLER_HEADING < 0) {
    STROLLER_HEADING = 360 + STROLLER_HEADING;
  }
  
 //----------------------------------------------------------------------

  Serial.print("HUMAN_HEADING : ");
  Serial.println(MAP_HUMAN_HEADING);
  Serial.print("STROLLER_HEADING : ");
  Serial.println(STROLLER_HEADING);
   
   HEADING = MAP_HUMAN_HEADING - STROLLER_HEADING +5;
   Serial.println(HEADING);
   
   
   while(HEADING < -180)
   HEADING += 360;
   //Serial.println("+++++++360");
   
   while(HEADING > 180)
   HEADING -= 360;
   //Serial.println("-------360");
   
   
  Serial.print("HEADING : ");
  Serial.println(HEADING);


  
}
void MOTOR_COMPASS()
{
  
    if(-30 < HEADING && HEADING < 30)
    {
      GO();
      Serial.println("///////////////GO");
    }
    else if(HEADING <= -30)
    {
      TURN_LEFT_STABLE();
      Serial.println("////////////////TURN_LEFT");
    }
    else if(HEADING >= 30)
    {
      TURN_RIGHT_STABLE();
      Serial.println("////////////////TURN_RIGHT");
    }
}
