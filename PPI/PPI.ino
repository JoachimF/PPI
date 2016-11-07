/*
 *  Affichage sur 7 segments
 *  Controle par encodeur rotatif avec bouton
 *  Gestion de découpe par PPI
 *    Reglage des PPI sauvegardé sur eeprom
 *    Reglage de la durée d'impulsion sauvegardé sur eeprom
 *  Gestion de la puissance par PWM remis à 0 sur reset
 *  Gestion de l'eau
 * 
 *  A faire sauvegarde des profils
 */


#include <TM1637Display.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include <EEPROM.h>

const uint8_t SEG_DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
  };
const uint8_t SEG_ON[] = {
  0,           
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  0            
  };
const uint8_t SEG_OFF[] = {
  0,           
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_A | SEG_F | SEG_G | SEG_E,                   // F
  SEG_A | SEG_F | SEG_G | SEG_E                    // F            
  };  
const uint8_t SEG_LASER[] = {
  SEG_F | SEG_E | SEG_D,                           // L
  SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F,   // A
  SEG_A | SEG_F | SEG_G | SEG_C | SEG_D,           // S
  SEG_G | SEG_E,                                   // r
  };
const uint8_t SEG_PPI[] = {
  SEG_F | SEG_A | SEG_B | SEG_G | SEG_E,            // P
  SEG_F | SEG_A | SEG_B | SEG_G | SEG_E,            // P
  SEG_F | SEG_E,                                    // I
  0
  };  
const uint8_t SEG_POWER[] = {
  SEG_F | SEG_A | SEG_B | SEG_G | SEG_E,            // P
  SEG_C | SEG_D | SEG_E | SEG_G,                    // o
  SEG_F | SEG_B | SEG_D,                            // w
  SEG_G | SEG_E                                     // r
  };
const uint8_t SEG_PULSE[] = {
  SEG_F | SEG_A | SEG_B | SEG_G | SEG_E,            // P
  SEG_E | SEG_D | SEG_C,                            // u
  SEG_F | SEG_E,                                    // l
  SEG_A | SEG_F | SEG_G | SEG_C | SEG_D             // s
  };
const uint8_t SEG_EAU[] = {
  SEG_A | SEG_F | SEG_G | SEG_D | SEG_E,            // E
  SEG_F | SEG_A | SEG_B | SEG_G | SEG_E | SEG_C,    // A
  SEG_E | SEG_D | SEG_C | SEG_F | SEG_B,            // U
  0
  };
#define EE_Pulse 0
#define EE_PPI 2
#define LASERON   LOW
#define LASEROFF  HIGH    
typedef enum {ST_POWER,ST_PPI,ST_PULSE,ST_EAU,ST_MAX} E_States ;
typedef struct profil_values {
  long Power ;
  int pulseMS ;
  float PPI ;
} profile ;

/*
#define ST_POWER 1
#define ST_PPI 2
#define ST_PULSE 3
#define ST_EAU 4
#define ST_MAX 3*/

#define POWERMAX 850
int Encoder_ok = 0 ;
int display_numbers = 0 ;
int display_all = 0 ;
long Power ;
int Power_last = Power = 0 ;
E_States STATE = ST_POWER;
int pulseMS = 300 ; // mS * 20
int pulseMS_last = pulseMS ;
float PPI ;
float un_sur_PPI ;
float PPI_last = PPI = 400;
#define ppiX 1/2540.0      //157.4744*25.4; // conversion of pulses in x to inches
#define ppiY 1/2540.0      //157.76525*25.4; // conversion of pulses in y to inches
int XDir = 0;
int YDir = 0;
int XStep = 0;
int YStep = 0;
int XStepOld = 1;
int YStepOld = 1;
int LsrCmd = 1;
int LsrCmdPrev = 1;
int LsrSet = 1;
int FirstOnState = 0;
float XCnt = 0;
float YCnt = 0;
float XCntPrev = 0;
float YCntPrev = 0;
uint32_t NSteps = 0;
uint32_t NStepsPrev = 0;
int mStopCnt = 0;
int mStopPrev = 0;
int moving = 0 ;
unsigned long timeOld = 0;
float cumDist = 0;
int pulse = 0;
int water_Ok = 1 ;
volatile unsigned int timerTicks = 0;
volatile unsigned int segTicks = 0 ;
volatile unsigned int pulseTick = 0 ;
volatile unsigned int water_watchdog = 0 ;
int16_t last, value;
int16_t pasenx=0;
int16_t paseny=0;
int encoder_value ;
unsigned int water_last ;

ClickEncoder Encoder(A0, A1, A2, 4);
#define CLK 10
#define DIO 16
//#define VCC 14
TM1637Display display(CLK, DIO);

//define the pins to make it more human freindly
#define XStepPin      2
#define YStepPin      3
#define YDirPin       4
#define XDirPin       5
#define PPIOnOffPin   6
#define LsrCmdPin     7
#define PwmPin        9
#define LsrSetPin     15
#define WaterLed      A3
#define WaterCnt      8
#define TestBtn       14


// Do not set LsrCmdPin or LsrSetPin to 13... that pin is tied to an
// led that flashes during writing cycles and will flash your laser
// when you don't want it to.
// Module connection pins (Digital Pins)


void save_int(int data,int adress)
{
  EEPROM.write(adress, data >> 8);
  EEPROM.write(adress+1, data & 0xFF);
}

int read_int(int adress)
{
  int data ;
  data = EEPROM.read(adress) << 8;
  data += EEPROM.read(adress+1);
  return data ;
}

void restore()
{
  pulseMS = read_int(EE_Pulse) ;
  PPI = read_int(EE_PPI) ;
  un_sur_PPI = 1/PPI ;
}

void save_ppi()
{
  save_int(PPI,EE_PPI) ;
}

void save_pulse()
{
  save_int(pulseMS,EE_Pulse) ;
}

void verify_water()
{
  int tmp ;
  tmp = digitalRead(WaterCnt) ;
  if(water_last != tmp)
  {
    water_watchdog = 0 ;
    water_last = tmp ;
    water_Ok = 1 ;
  }
  if(!water_Ok){
    digitalWrite(LsrSetPin,LASEROFF) ;
    STATE = ST_EAU ;
    display.setSegments(SEG_EAU) ;
  }
}

void bouton()
{
  switch (Encoder.getButton())
  {
    case ClickEncoder::Clicked:
/*        if(STATE < ST_MAX)
          STATE++ ;
        else
          STATE = 1 ;*/
        switch (STATE)
        {
          case   ST_POWER:
           display.setSegments(SEG_PPI) ;
           segTicks = 0 ;
           display_numbers = 0 ;
           STATE = ST_PPI ;           
          break;
          case   ST_PPI:
            save_ppi() ;
            display.setSegments(SEG_PULSE) ;
            segTicks = 0 ;
            display_numbers = 0 ;
            STATE = ST_PULSE ;
          break;
          case   ST_PULSE:
            save_pulse() ;
            display.setSegments(SEG_POWER) ;
            segTicks = 0 ;
            display_numbers = 0 ;
            STATE = ST_POWER ;
          break;
        } 
      break;
      case ClickEncoder::DoubleClicked:
          case   ST_EAU:
            display.setSegments(SEG_POWER) ;
            segTicks = 0 ;
            display_numbers = 0 ;
            STATE = ST_POWER ;
          break;
      break;
  }  
}

void lit_encodeur()
{
  encoder_value = Encoder.getValue();
  if(encoder_value)
  {
    switch (STATE)
    {
        case ST_POWER: 
          Power += encoder_value;
          if(Power > 100) Power = 100 ;
          if(Power < 0)  Power = 0 ;
          if(Power != Power_last) {
            Power_last = Power ;
            display_numbers = 1 ;
            Timer1.setPwmDuty(PwmPin, (Power * 85)/10) ;
          }    
        break;
    
        case ST_PPI:
          PPI += encoder_value ;
          if(PPI > 1200) PPI = 1200 ;
          if(PPI < 0)  PPI = 0 ;
          un_sur_PPI = 1/PPI ;
          if(PPI != PPI_last) {
            PPI_last = PPI ;
            display_numbers = 1 ;
          }
        break;

        case ST_PULSE:
          pulseMS += encoder_value*20 ;
          if(pulseMS > 800) pulseMS = 800 ;
          if(pulseMS < 0)  pulseMS = 0 ;
          if(pulseMS != pulseMS_last) {
            pulseMS_last = pulseMS ;
            display_numbers = 1 ;
          }
        break;
     
        case ST_EAU:
          display.setSegments(SEG_EAU);
        break;
     }
  }  
}

void display_data()
{
  if (display_numbers) {
    display_numbers = 0 ;
      switch (STATE){
      case ST_POWER:
          display.showNumberDec(Power,false);
      break;    
      case ST_PPI:
          if (digitalRead(PPIOnOffPin))
            display.showNumberDec(PPI,false);
          else
            display.setSegments(SEG_OFF);
      break;
      case ST_PULSE:
          display.showNumberDec(pulseMS/20,false);
      break;
      case ST_EAU:
          display.setSegments(SEG_EAU);
      break;  
    }
  }  
}

void setup() {
  // set pin modes
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff }; 
  pinMode(XStepPin, INPUT);
  pinMode(XDirPin, INPUT);
  pinMode(YStepPin, INPUT);
  pinMode(YDirPin, INPUT);
  pinMode(LsrCmdPin, INPUT);
  pinMode(PPIOnOffPin, INPUT);
  pinMode(LsrSetPin, OUTPUT);
  digitalWrite(LsrSetPin, HIGH);
  digitalWrite(PPIOnOffPin, HIGH);
  pinMode(PwmPin, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);  
//  pinMode(VCC, OUTPUT);
//  digitalWrite(VCC, HIGH);
  restore() ;
  // mase sure laser is off at startup

/* Mode avec interruption */
/* attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);  */
/* Micro, Leonardo, other 32u4-based 0, 1, 2, 3, 7 */
attachInterrupt(digitalPinToInterrupt(XStepPin), Xstep, RISING); // Pin2
attachInterrupt(digitalPinToInterrupt(YStepPin), Ystep, RISING); // Pin3
attachInterrupt(digitalPinToInterrupt(LsrCmdPin), LaserCommand, CHANGE);
Timer1.initialize(50); // débordement toutes les 50µS
Timer1.attachInterrupt(timerIsr);
Encoder.setAccelerationEnabled(true);
Timer1.pwm(PwmPin,0,50) ; // pwm(char pin, int duty, long microseconds)

display.setBrightness(0x0f);
delay(50);
display.setSegments(SEG_LASER);
delay(2000) ;
}

void loop() {
  verify_water() ;
  if (display_all)
  {
      display_all = 0 ;
      bouton() ;
      lit_encodeur() ;
      display_data() ;
  }
  
/*  if(digitalRead(TestBtn))
  {
      pulse = 1 ;
      pulseTick = 0 ;
      digitalWrite(LsrSetPin,LASERON);
      while(pulse == 1) ;
      digitalWrite(LsrSetPin,LASEROFF);
      delay(1000) ;
  }*/
  
  if (digitalRead(PPIOnOffPin)){
    if (!LsrCmd){
//      if (CheckForMotion()){
        CalcTravel();
        if(FirstOnState){
          cumDist = 0;
          pulse = 1;
          pulseTick = 0 ;
          FirstOnState = 0;
        }
        if (cumDist >= un_sur_PPI) {
          cumDist = 0;
          pulse = 1;
          pulseTick = 0 ;
        }
        if (pulse)
          {
            if(water_Ok)
              digitalWrite(LsrSetPin,LASERON);
          }
          else
          {
            digitalWrite(LsrSetPin,LASEROFF);
          }
//      }else{
//        digitalWrite(LsrSetPin,LASEROFF);
//      }
    }else{
      digitalWrite(LsrSetPin,LASEROFF);
      //have to reset the counts here or after first time run, laser flashes once b/4 moving
      XCnt = 0;
      YCnt = 0;
      }
  }
}
/*
int CheckForMotion() {
/*  if ((XCntPrev == XCnt) && (YCntPrev == YCnt)) {
    mStopCnt ++;
    if (mStopCnt >= 1000){mStopPrev = 0;}
    return mStopPrev;
  }else{
    mStopCnt = 0;
    mStopPrev = 1;
    return 1;
  }*//*
  if(moving)
  {
    mStopCnt = 0;
    mStopPrev = 1;
    moving = 1 ;
    return 1 ;
  }
  else
  {
    mStopCnt ++;
    if (mStopCnt >= 100){mStopPrev = 0;}
    return 1;//mStopPrev;
  }
}*/

void CalcTravel() {
  float Xtmp,Ytmp ;
  if (NSteps >= 4){
    noInterrupts();
    Xtmp = XCnt ;   // critical, time-sensitive code here
    Ytmp = YCnt ;
    XCnt = 0;
    YCnt = 0;
    interrupts();
    cumDist += sqrt(pow(Xtmp*ppiX,2) + pow(Ytmp*ppiY,2));
//    XCntPrev = 0;
//    YCntPrev = 0;
  }
}

void Xstep()
{
  XDir = digitalRead(XDirPin); //Interruption pas en X en provenance de la carte de controle
//  XCntPrev = XCnt;
  XCnt += XDir*2-1;
  NSteps ++;
//  moving = 1 ;
}

void Ystep()
{
  YDir = digitalRead(YDirPin); //Interruptionpas en Y en provenance de la carte de controle
//  YCntPrev = YCnt;
  YCnt += YDir*2-1;
  NSteps ++;
//  moving = 1 ;
}

void LaserCommand(void) //Interruption commande du laser en provenance de la carte de controle
{
  LsrCmd = digitalRead(LsrCmdPin);
  if(!water_Ok) // interdiction d'allumer le laser
    LsrCmd = LASEROFF ;
  if (!digitalRead(PPIOnOffPin))
    digitalWrite(LsrSetPin, LsrCmd);
  if ((LsrCmdPrev == LASEROFF) && (LsrCmd == LASERON)) {FirstOnState = 1 ;}
    LsrCmdPrev = LsrCmd ;
}
void timerIsr(void) // ticks toute les 50µS
{ 
  if (LsrCmd || digitalRead(PPIOnOffPin))
  {
    if (timerTicks >= 20) // 1ms
    {
        Encoder.service();
        timerTicks = 0 ;
        display_all = 1 ;
    }
    if (segTicks >= 50000) // 2.5 secondes
    {
      LsrCmdPrev = 1 ;
      XCnt = 0;
      YCnt = 0;
      XCntPrev = 0;
      YCntPrev = 0;
      NSteps = 0;
      NStepsPrev = 0;
      display_numbers = 1 ;
      segTicks = 0 ;
    }
    timerTicks++;
    segTicks++ ;
  }
  if (water_watchdog >= 4000) // 0.2 seconde
  {
      water_Ok = 0 ;
  }
   
  if (pulseTick >= pulseMS) 
  {
    pulse = 0 ;
    pulseTick = 0 ;
  }
  pulseTick++ ;
  water_watchdog++ ;
}

