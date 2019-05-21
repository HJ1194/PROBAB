#include <SPI.h>
//------------------------------COMPASS-------------------------
#include <Wire.h>
int headingDegrees;
//--------------------------------------------------------------

//--------------------------------BLUE--------------------------

#include <SoftwareSerial.h>
#define BT_RX 6
#define BT_TX 7
int re_data;
int Map_Re_Data;

SoftwareSerial BT_RE(BT_RX, BT_TX);
int MAPPING_HUMAN_DEGREE;

//---------------------------------------------------------------

void setup (void)
{
  
//-------------------------------------SPI-----------------------
// SPI 통신을 위한 핀들의 입출력 설정
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  // 마스터의 전송 속도에 맞추어 통신 속도를 설정한다.
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  // SPI 통신을 위한 레지스터를 설정
  SPCR |= _BV(SPE); // SPI 활성화
  SPCR &= ~_BV(MSTR); // Slave 모드 선택
  SPCR |= _BV(SPIE); // 인터럽트 허용
  Serial.begin(9600);
//----------------------------------------------------------------

//--------------------------------BLUE--------------------------
BT_RE.begin(9600);
//--------------------------------------------------------------

}


ISR (SPI_STC_vect)
{
  SPDR = re_data;
}

void loop (void)

{
  if (BT_RE.available())
 {
    re_data = BT_RE.read();
    Map_Re_Data = map(re_data, 0, 255, 0, 360);
    
  }
  
  Serial.print(" HUMAN_HEADING : ");
  Serial.println(Map_Re_Data);
  delay(300);


}


