//#define MOTO
#define SWEEP
//  #define CAR

#include "arduino.h"
#include "LowPower.h"
#include "PinChangeInterrupt.h"
// #include <avr/wdt.h>
#include "timestamp32bits.h"
#include "Adafruit_FRAM_I2C.h"

#define intPin 8
volatile bool flag=false;
bool received=false;  

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;
unsigned long framWritePosition = 0;
unsigned long framWritePositionDebug = 32000;

#ifndef CARGEO
uint16_t SizeRec = 66;
#endif
#ifdef CARGEO
uint16_t SizeRec = 49;//49
uint16_t sizeSend=163;//163
#endif

int insertCounter = 0;
String fixStatus = " ";
String gpsTime = " ";
String latitude = " ";
String longitude = " ";
String course = " ";
String pdop = " ";
uint32_t payLoadSize = 0;
String used_satellites = " ";
String viewed_satellites = " ";
String lastUnixTime = " ";
char Buffer[10] = {0};
String speed = " ";
String imei = " ";
int fixState = 0;
int gnsState = 0;
uint16_t gnsFailCounter = 0;
unsigned long previousMillisGps = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 60000;
uint16_t gpsFailCounter = 0;
bool started = true;
bool restarted = false;
uint16_t httpActionFail = 0;
uint16_t FirstStartCounter = 0;
uint16_t ReStartCounter=0;
unsigned long t1 = 0; //le temps du dernier point inséré
unsigned long t2 = 0; //le temps du dernier point capté
uint16_t ti = 6; //le temps entre chaque insertion
unsigned long t3 = 0; //le temps du dernier envoie
unsigned long unixTimeInt = 0;
bool wakeUp=true;
bool OkToSend = true;
uint16_t maxTime = 0;
uint16_t Size = 0;
String batLev=" ";
volatile uint16_t wakeUpCounter = 0;
char newC[3]={0};
unsigned long pingingInterval=40000;
//6  45----7points 9secondsSend  ok
//6  41----7points 10secondsSend
//6  39----6points 9secondsSend
//6  50----8points 10secondsSend
//6  36----6points 9secondsSend
//6  34----6points 8secondsSend
//6  29----5points 8.5secondsSent
//6  25----4Points 8secondsSend

int badCharCounter=0;
uint64_t lastSend =0;
uint16_t reps=0;
char* one="1";
char* zero="0";
bool ping=true;
uint8_t noGsmCounter=0;
bool insertFlag=false;
bool httpPostCustom(char custom);
bool badCharChecker(String data);
void IntRoutine(void);
// bool httpPostAll();
void httpPostMaster();
void httpPing();
bool httpPostFromTo(uint16_t p1, uint16_t p2);
void decrementCounter(uint16_t value);
bool turnOnGns(uint16_t waitInterval);
bool getGnsStat();
bool getGpsData();
void sendAtCom(char *AtCom);
void getImei();
uint8_t getGsmStat();
String batteryLevel();
String rssiLevel();
bool gprsOn();
void gprsOff();
bool getGprsState();
void flushSim();
void writeDataFram(char* dataFram);
void writeDataFramDebug(char* dataFram, long p1);
void powerUp(); 
void powerDown();
void blinkLED(int k);
void blinkLEDFast(int k);
void clearMemory(int size);
void clearMemoryDiff(int size, int size1);
void clearMemoryDebug(unsigned long size);
void insertMem();
void incrementCounter();
String complete(String s, int t);
int getCounter();
int getValue(uint16_t position, uint8_t largeur);
void incrementValue(uint16_t position, uint8_t largeur);
bool sendAtFram(long timeout, uint16_t pos1, uint16_t pos2, char* Rep, char* Error, int nbRep);
bool fireHttpAction(long timeout, char* Commande, char* Rep, char* Error);
// void trace(unsigned long unixTime, uint8_t type);
void clearValue();
bool insertGpsData();
void resetSS();
void cfunReset();
void hardResetSS();
int getBatchCounter(uint16_t i);
bool gps();
void powerCycle();
bool powerCheck();
bool gpsCheck(uint16_t waitInterval);
bool gsmCheck(uint16_t waitInterval);
void sendFromFram(uint16_t start,uint16_t length);
void SendConnectPacket();
// int limitToSend =7;
// unsigned long te = 28; //le temps entre les envoies
// //Resultat: 04/06/06/15

// int limitToSend =12;
// unsigned long te = 56; //le temps entre les envoies
// //4/4/3/4/3/5/4/3/5/6/7/16

// int limitToSend =14;
// unsigned long te = 56; //le temps entre les envoies
// //4/4/4/4/4/4/3/4/4/7/16.5

uint16_t httpTimeout=20000;
int limitToSend =14;
unsigned long te = 64; //le temps entre les envoies
//4/4/4/4/4/3/4/4/4/4/4/6/16.5

// int limitToSend =16;
// unsigned long te = 72; //le temps entre les envoies
// //4/4/4/4/3/4/4/4/4/4/4/3/7/17.5

String previousUnixTime="0";
volatile uint16_t iterations=1800; //sleeping time = iterations X 8 Seconds

unsigned long datalength, CheckSum, RLength;
unsigned long topiclength;
unsigned char topic[30];
char str[250];
// char message[]="[{\"P\":\"869170031856769131.669804-8.010563026.06100029.2095160931073400\"},{\"P\":\"869170031856769131.669804-8.010563026.06100029.2095160931073400\"}]";
unsigned char encodedByte;
int X;
unsigned long MQTTProtocolNameLength;
unsigned long MQTTClientIDLength;
unsigned long MQTTUsernameLength;
unsigned long MQTTPasswordLength;

char  MQTTHost[]="velovolt.ddns.net";
char  MQTTPort[]="8081";
char  MQTTClientID[]="ABCDEF";
char  MQTTTopic[]="869170030524574";
char  MQTTProtocolName[]="MQIsdp";
// const char  MQTTProtocolName[10]="MQTT";
const char  MQTTLVL = 0x03;
const char  MQTTFlags= 0xC2;
const unsigned  int MQTTKeepAlive = 660;
char  MQTTUsername[]="admin";
char  MQTTPassword[]="isis";
char  MQTTQOS = 0x00;
char  MQTTPacketID=0x0001;

void setup() {
  pinMode(A0,OUTPUT);
  digitalWrite(A0,HIGH);
  fram.begin();
#ifdef MOTO
  pinMode(3,OUTPUT);  // VIO MOTO
  digitalWrite(3, HIGH);//turn On the module
#endif
#ifdef CAR
  pinMode(A2,OUTPUT); // VIO CAR
  digitalWrite(A2, HIGH);//turn On the module
#endif
#ifdef SWEEP
  pinMode(8, OUTPUT);//VIO
  pinMode(6, OUTPUT);//sim Reset
  digitalWrite(6, HIGH);
  digitalWrite(A0, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(A0,HIGH);
#endif

  pinMode(0, INPUT);//SS RX
  pinMode(1, OUTPUT);//SS TX
  pinMode(A3, INPUT);//sim Power Status

  powerDown();

  powerUp();
  delay(100);
  noGsmCounter = 0;
  gpsFailCounter=0;
  httpActionFail=0;
  for (int i = 0; i < 2; i++)
  {
    if (!gpsCheck(180000)||(!gsmCheck(20000))){
      powerCycle();
      delay(5000);
    }else{i=2;digitalWrite(A0,LOW);
    gprsOn();
    Serial.setTimeout(10000);
    Serial.println("AT+CIPSHUT");
    Serial.findUntil("OK","ERROR");

    Serial.println("AT+CIPMUX=0");
    Serial.findUntil("OK","ERROR");

    Serial.println("AT+CGATT=1");
    Serial.findUntil("OK","ERROR");

    Serial.println("AT+CSTT=\"iamgprs1.ma\",\"\",\"\"");
    Serial.findUntil("OK","ERROR");

    Serial.println("AT+CIICR");
    Serial.findUntil("OK","ERROR");

    Serial.println("AT+CIFSR");
    Serial.findUntil(">","ERROR");

    Serial.println("AT+CIPSTART=\"TCP\",\"velovolt.ddns.net\",\"8081\"");
    Serial.findUntil("CONNECT OK","ERROR");

    SendConnectPacket();
    }    
  }
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin), IntRoutine, RISING);
  // disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
}
void SendConnectPacket(){
  Serial.println("AT+CIPSEND");
  Serial.readStringUntil(">");
  Serial.write(0x10);
  MQTTProtocolNameLength = strlen(MQTTProtocolName);
  MQTTClientIDLength = strlen(MQTTClientID);
  MQTTUsernameLength = strlen(MQTTUsername);
  MQTTPasswordLength = strlen(MQTTPassword);
  datalength = MQTTProtocolNameLength + 2 + 4 + MQTTClientIDLength + 2 + MQTTUsernameLength + 2 + MQTTPasswordLength + 2;
  X = datalength;
  do {
    encodedByte = X % 128;
    X = X / 128;
    if (X > 0) {
      encodedByte |= 128;
    }
    Serial.write(encodedByte);
  }
  while (X > 0);
  Serial.write(MQTTProtocolNameLength >> 8);
  Serial.write(MQTTProtocolNameLength & 0xFF);
  Serial.print(MQTTProtocolName);
  Serial.write(MQTTLVL); // LVL
  Serial.write(MQTTFlags); // Flags
  Serial.write(MQTTKeepAlive >> 8);
  Serial.write(MQTTKeepAlive & 0xFF);
  Serial.write(MQTTClientIDLength >> 8);
  Serial.write(MQTTClientIDLength & 0xFF);
  Serial.print(MQTTClientID);
  Serial.write(MQTTUsernameLength >> 8);
  Serial.write(MQTTUsernameLength & 0xFF);
  Serial.print(MQTTUsername);
  Serial.write(MQTTPasswordLength >> 8);
  Serial.write(MQTTPasswordLength & 0xFF);
  Serial.print(MQTTPassword);
  Serial.write(0x1A);
  if (Serial.findUntil("OK","ERROR")){blinkLED(1);}else{blinkLEDFast(1000);}
}
void loop() {
  blinkLED(1);
  if(getCounter()>380){clearMemory(30999);clearMemoryDebug(32003);for(long i=32080;i<32180;i++){writeDataFramDebug("0",i);}powerCycle();}
  powerCheck();
  gpsCheck(180000);
#ifdef SWEEP
    // if((t2 - t3) >= (te-8)){t3=t2;httpPing();httpPostMaster();}
#endif
#ifdef MOTO
  if (digitalRead(8)){
    if((t2 - t3) >= (te-8)){t3=t2;httpPing();httpPostMaster();}
  }else {
    httpTimeout=20000;httpPing();httpPostMaster();
    httpPostCustom('0');
    httpTimeout=14000;Serial.flush();powerDown();
    Wire.beginTransmission(8);
    Wire.write('f');
    Wire.endTransmission();
    enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
    while (wakeUpCounter <= iterations) {
        LowPower.powerDown(SLEEP_8S,ADC_OFF,BOD_OFF);
          enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
      wakeUpCounter++;
      if (wakeUpCounter==iterations+2){blinkLEDFast(20);}else{blinkLED(1);}
    }
    disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));  
    Wire.beginTransmission(8);                 //Vehicule ignition wakeup
    Wire.write('n');
    Wire.endTransmission();
    wakeUpCounter = 0;gpsFailCounter=0;powerUp();gprsOn();
      httpPostCustom('1');
  }
#endif
#ifdef CAR
  if (digitalRead(8)){
    if((t2 - t3) >= (te-8)){t3=t2;httpPing();httpPostMaster();}
  }else {
    httpTimeout=20000;httpPing();httpPostMaster();
    httpPostCustom('0');
    httpTimeout=14000;Serial.flush();powerDown();
      enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
      while (wakeUpCounter <= iterations) {
        LowPower.powerDown(SLEEP_8S,ADC_OFF,BOD_OFF);
        enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
        wakeUpCounter++;
        if (wakeUpCounter==iterations+2){blinkLEDFast(20);}else{blinkLED(1);}
      }
      disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));  
      wakeUpCounter = 0;gpsFailCounter=0;powerUp();gprsOn();
      httpPostCustom('1');
    }
#endif
}
void httpPostMaster(){
  if ((getCounter() < limitToSend)) {
    if(httpPostFromTo(0,getCounter())){
      clearMemoryDiff(0,getCounter()*SizeRec);
      clearMemoryDebug(32003);  for(long i=32080;i<32180;i++){writeDataFramDebug("0",i);}
    }
  }else{
    uint16_t repetitions=getCounter()/limitToSend;
    for (uint16_t i = 1; i<=repetitions; i++){
      if(getBatchCounter(i)==1){
        if(httpPostFromTo((i-1)*limitToSend,((i)*limitToSend))){writeDataFramDebug("0",(32080+i));}
        gps();
        repetitions=getCounter()/limitToSend;
      }
    }
    bool finiShed=true;
    for (uint8_t i = 1; i <= (getCounter()/limitToSend); i++){if (getBatchCounter(i)==1){finiShed=false;}}
    if(finiShed){
      if((getCounter()%limitToSend)!=0){
        uint16_t reps= getCounter()/limitToSend;         
          if(httpPostFromTo(reps*limitToSend,getCounter())){
            clearMemoryDiff(0,getCounter()*SizeRec); 
            clearMemoryDebug(32003); for(long i=32080;i<32180;i++){writeDataFramDebug("0",i);}
          } 
      }else{
        clearMemoryDiff(0,getCounter()*SizeRec); 
        clearMemoryDebug(32003); for(long i=32080;i<32180;i++){writeDataFramDebug("0",i);}
      }
    }
  }
}
bool httpPostFromTo(uint16_t p1, uint16_t p2) {
  if(!ping){
    bool OkToSend = true;
    if (sendAtFram(3000, 31254, 11, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
      if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
#ifndef CARGEO
        if (sendAtFram(5000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
          Serial.setTimeout(10000);
          flushSim();
          Serial.print("AT+HTTPDATA=");
          delay(100);
          uint16_t Size = ((p2-p1) * (SizeRec + 1)) + ((p2-p1) * 8) - 1 + 2;
          Serial.print(Size);
          Serial.print(",");
          uint32_t maxTime = 30000;
          Serial.println(maxTime);
          Serial.findUntil("DOWNLOAD", "ERROR");
          Serial.print("[");
          for (uint16_t i = p1; i < p2 ; i++)
          {
            for (uint16_t j = SizeRec * i; j < (SizeRec * (i + 1)) ; j++)
            {
              if (j == (i * SizeRec)) {Serial.print("{\"P\":\"");delay(1);}
              uint16_t test = fram.read8(j);
              if ((test==0) ){sprintf(Buffer, "%c", 120); //x
              }else{sprintf(Buffer, "%c", test);}
              Serial.write(Buffer);
              delay(1);
            }
            Serial.print("\"}");
            delay(1);
            if (i < p2 - 1) {Serial.write(",");delay(1);}
          }
          Serial.print("]");
          Serial.findUntil("OK", "OK");
        } else OkToSend = false;
#endif
#ifdef CARGEO
        if (sendAtFram(5000, 31286, 67, "OK", "ERROR", 5)) { //URL GEORED
          Serial.setTimeout(10000);
          flushSim();
          Serial.print("AT+HTTPDATA=");
          delay(100);
          // uint16_t Size = ((p2-p1) * (sizeRec)) + 40 + 140;
          uint16_t Size = ((p2-p1) * (sizeSend)) + 40 + 140;
          Serial.print(Size);
          Serial.print(",");
          uint32_t maxTime = 30000;
          Serial.println(maxTime);
          Serial.findUntil("DOWNLOAD", "ERROR");
/////////////////////////////////////////////////////////////////////
          for(int i=0;i<140;i++){
            uint8_t test = fram.read8(i+31405);
            char Buffer[2] = {0};
            if ((test==0) ){sprintf(Buffer, "%c", 120);}else{sprintf(Buffer, "%c", test);}
            Serial.write(Buffer);
            delay(1);
          }  
/////////////////////////////////////////////////////////////////////          
          for (uint16_t i = p1; i < p2 ; i++)
          {
            sendFromFram(31041,13);           //"<Track Imei=\""                           //13
            sendFromFram(31474,15);           //imei                                       //15
            sendFromFram(31054,26);           //"\" Fc=\"WGS84\" FixPosition=\""           //26
            sendFromFram(SizeRec*(i),1);      //fix                                        //1
            sendFromFram(31080,7);            //"\" Lat=\""                                //7
            sendFromFram(1+SizeRec*(i),10);   //Latitude                                   //10
            sendFromFram(31087,7);            //"\"Lon=\""                                 //7
            sendFromFram(11+SizeRec*(i),11);  //Longitude                                  //11
            sendFromFram(31094,7);            //"\" Vit=\""                                //7
            sendFromFram(22+SizeRec*(i),6);   //Speed                                      //6
            sendFromFram(31101,7);            //"\" Sat=\""                                //7
            sendFromFram(28+SizeRec*(i),2);   //Sat                                        //2
            sendFromFram(31108,7);            //"\" Cap=\""                                //7
            sendFromFram(30+SizeRec*(i),6);   //Course                                     //6
            sendFromFram(31115,16);           //"\" BatteryLevel=\""                       //16
            sendFromFram(36+SizeRec*(i),3);   //BatLev                                     //3
            sendFromFram(31131,6);            //"\" Dh=\""                                 //6
            sendFromFram(39+SizeRec*(i),10);  //Dh                                         //10
            sendFromFram(31137,3);            //"\"/>";                                    //3  
          }
/////////////////////////////////////////////////////////////////////          
          for(int i=0;i<40;i++){
            uint8_t test = fram.read8(i+31545);
            char Buffer[2] = {0};
            if ((test==0)){sprintf(Buffer, "%c", 120);}else{sprintf(Buffer, "%c", test);}
            Serial.write(Buffer);
            delay(1);
          }
/////////////////////////////////////////////////////////////////////
          Serial.findUntil("OK", "OK");
        } else OkToSend = false;
#endif
      } else OkToSend = false;
    } else OkToSend = false;
    if (OkToSend) {
      if (fireHttpAction(httpTimeout, "AT+HTTPACTION=", ",200,", "ERROR")) {
       sendAtFram(5000, 31241, 11, "OK", "ERROR", 5);return true;} 
       else 
       {sendAtFram(5000, 31241, 11, "OK", "ERROR", 5);return false;}
    }else{return false;}
  }else{return false;}
}
void powerCycle(){
  fram.begin();
#ifdef MOTO
  pinMode(3,OUTPUT);  // VIO MOTO
  digitalWrite(3, HIGH);//turn On the module
#endif
#ifdef SWEEP
  pinMode(8,OUTPUT);  // VIO MOTO
  digitalWrite(8, HIGH);//turn On the module
#endif
#ifdef CAR
  pinMode(A2,OUTPUT); // VIO CAR
  digitalWrite(A2, HIGH);//turn On the module
#endif

  pinMode(0, INPUT);//SS RX
  pinMode(1, OUTPUT);//SS TX
  pinMode(A3, INPUT);//sim Power Status

  powerDown();

  powerUp();
  gprsOn();
  delay(100);
  noGsmCounter = 0;
  gpsFailCounter=0;
  httpActionFail=0;
}
void powerDown() {
  if ((analogRead(A3) > 200)) {
    pinMode(5, OUTPUT);//PWR KEY  
    digitalWrite(5, LOW);
    delay(1200);
    pinMode(5, INPUT_PULLUP);// Turn On the module
    delay(400);
  }
}
void powerUp() {
  if ((analogRead(A3) < 200)) {
    pinMode(5, INPUT_PULLUP);// Turn On the module
    delay(1200);
    pinMode(5, OUTPUT);//PWR KEY  
    digitalWrite(5, LOW);
    delay(1500);
    pinMode(5, INPUT_PULLUP);// Turn On the module
  }
      delay(2500);
      Serial.begin(4800);
      delay(200);
      getImei();
      gsmCheck(20000);
      turnOnGns(30000);
      noGsmCounter=0;
      gpsFailCounter=0;
      httpActionFail=0;
}
bool gps(){
  if (!getGpsData()) {
    if (!getGnsStat()) {if (gnsFailCounter == 2) {resetSS();} else {turnOnGns(30000);delay(1000);gnsFailCounter++;}}
      if(restarted){if (ReStartCounter == 10) {resetSS();}else {delay(2000);ReStartCounter++;}
      }else if (started){if (FirstStartCounter == 1) {resetSS();}else{delay(60000);FirstStartCounter++;}
      }else if((!restarted)&&(!started)){if (gpsFailCounter == 10) {resetSS();}else {delay(1000);gpsFailCounter++;}}
      
      if (!getGpsData()) {return false;}else{return true;}

  }else{return true;}
}
bool gpsCheck(uint16_t waitInterval){
  currentMillis = millis();
  previousMillis = millis();
  while((!gps())&&((currentMillis - previousMillis) <= waitInterval)&&(analogRead(A3)>200)){
    currentMillis=millis();delay(1000);
  }
    if (((currentMillis - previousMillis) <= waitInterval)){
    if (analogRead(A3)>200){
      return true;
    }else{
      pinMode(6, OUTPUT);//RST KEY  
      digitalWrite(6, LOW);//PULL RST KEY TO GROUND FOR >105MS
      delay(200);
      pinMode(6, INPUT);// RST BACK TO NORMAL
      delay(3000);
      blinkLEDFast(10);
      powerCycle();
      return false;
    }
  }else{return false;}
}
bool gsmCheck(uint16_t waitInterval){
  currentMillis = millis();
  previousMillis = millis();
  uint8_t gsmStatInt=getGsmStat();
  while((gsmStatInt != 1)&&(gsmStatInt != 5)&&((currentMillis - previousMillis) <= waitInterval)&&(analogRead(A3)>200)){
    gsmStatInt=getGsmStat();currentMillis=millis();delay(1000);
  }
  if (((currentMillis - previousMillis) <= waitInterval)){
    if (analogRead(A3)>200){
      return true;
    }else{
      pinMode(6, OUTPUT);//RST KEY  
      digitalWrite(6, LOW);//PULL RST KEY TO GROUND FOR >105MS
      delay(200);
      pinMode(6, INPUT);// RST BACK TO NORMAL
      delay(3000);
      blinkLEDFast(10);
      powerCycle();
      return false;
    }
  }else{return false;}
}
void sendFromFram(uint16_t start,uint16_t length){
  for (uint16_t a = start; a < start + length; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
     if ((test==0) ){sprintf(Buffer, "%c", 120);}else{sprintf(Buffer, "%c", test);}
    Serial.print(Buffer);
  }
}
void httpPing() {
  if(gsmCheck(25000)){
    bool OkToSend = true;
    if (sendAtFram(3000, 31254, 11, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
      if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
#ifndef CARGEO
        if (sendAtFram(15000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
          Serial.setTimeout(10000);
          flushSim();
          Serial.print("AT+HTTPDATA=");
          delay(100);
          uint16_t Size = 11;
          Serial.print(Size);
          Serial.print(",");
          uint32_t maxTime = 30000;
          Serial.println(maxTime);
          Serial.findUntil("DOWNLOAD", "ERROR");
          Serial.print("[{\"S\":\"");
          Serial.print("1\"}]");
          Serial.findUntil("OK", "OK");
        } else OkToSend = false;
#endif
#ifdef CARGEO
        if (sendAtFram(5000, 31286, 67, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
          Serial.setTimeout(10000);
          flushSim();
          Serial.print("AT+HTTPDATA=");
          delay(100);
          uint8_t Size = 2;
          Serial.print(Size);
          Serial.print(",");
          uint16_t maxTime = 5500;
          Serial.println(maxTime);
          Serial.findUntil("DOWNLOAD", "ERROR");
          Serial.print("vv");
          Serial.findUntil("OK", "OK");
        } else OkToSend = false;
#endif
      } else OkToSend = false;
    } else OkToSend = false;
    if (OkToSend) {
      fireHttpAction(3500, "AT+HTTPACTION=", ",200,", "ERROR");
      sendAtFram(5000, 31241, 11, "OK", "ERROR", 5); //httpterm
    }
  }gps();
}
bool httpPostCustom(char custom) {
  if(!ping){
    bool OkToSend = true;
    if (sendAtFram(3000, 31254, 11, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
      if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
        if (sendAtFram(15000, 31609, 73, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
          Serial.setTimeout(10000);
          flushSim();
          // wdt_enable(WDTO_4S);
          Serial.print("AT+HTTPDATA=");
          delay(100);
          uint16_t Size = 26;
          Serial.print(Size);
          Serial.print(",");
          uint32_t maxTime = 30000;
          Serial.println(maxTime);
          Serial.findUntil("DOWNLOAD", "ERROR");
          Serial.print("[{\"S\":\"");
          Serial.print(imei.c_str());
          Serial.print(custom);
          Serial.print("\"}]");
          Serial.findUntil("OK", "OK");
          // wdt_disable();
        } else OkToSend = false;
      } else OkToSend = false;
    } else OkToSend = false;
    if (OkToSend) {
      if (fireHttpAction(2000, "AT+HTTPACTION=", ",200,", "ERROR")) {return true;} else {return false;}
    }else{return false;}
  }else{return false;}
}
void IntRoutine() {wakeUpCounter = iterations+1;}
void decrementCounter(uint16_t value) {
  int countVal = getCounter();
  countVal -= value;
  writeDataFramDebug(complete(String(countVal), 3).c_str(), 32000);
}
bool turnOnGns(uint16_t waitInterval){
  currentMillis = millis();
  previousMillis = millis();
  while (!getGnsStat()&&((currentMillis - previousMillis) <= waitInterval)&&(analogRead(A3)>200)) {
    sendAtFram(3000, 31000, 12, "OK", "ERROR", 5); //"AT+CGNSPWR=1"
    currentMillis=millis();delay(1000);
  } 
  if (((currentMillis - previousMillis) <= waitInterval)){
    if (analogRead(A3)>200){
      return true;
    }else{
      pinMode(6, OUTPUT);//RST KEY  
      digitalWrite(6, LOW);//PULL RST KEY TO GROUND FOR >105MS
      delay(200);
      pinMode(6, INPUT);// RST BACK TO NORMAL
      delay(3000);
      blinkLEDFast(10);
      powerCycle();
      return false;
    }
  }else{return false;}
}
bool getGnsStat() {
  flushSim();
  // wdt_enable(WDTO_8S);
  Serial.setTimeout(100);
  Serial.println("AT+CGNSPWR?");//"AT+CGNSPWR?"
  if (Serial.findUntil("1", "0")) {/*wdt_disable();*/return true;}else{/*wdt_disable();*/return false;}
}
bool badCharChecker(String data){
      for (int i = 0; i < strlen(data.c_str()); i++){   
          if((!isAlphaNumeric(data.c_str()[i])&&(int(data.c_str()[i])!=45)&&(int(data.c_str()[i])!=46))||((int(data.c_str()[i])==0))||((int(data.c_str()[i])==32))){badCharCounter++;}
        }
        if (badCharCounter>0){badCharCounter=0;return false;}else{badCharCounter=0;return true;}
}
bool getGpsData() {
  fixStatus = gpsTime = latitude = longitude = used_satellites = viewed_satellites = speed = " ";
  Serial.setTimeout(2000);
  flushSim();
  char gpsData[120] = {0};
  Serial.println("AT+CGNSINF");
  Serial.readBytesUntil('O', gpsData, 119);///////////////////////////////////////////////////
  Serial.println("AT+CIPSEND");
  Serial.readStringUntil(">");
  memset(str, 0, sizeof(str));
  topiclength = sprintf((char * ) topic, MQTTTopic);
  // datalength = sprintf((char * ) str, "%s%u", topic, Counter);
  datalength = sprintf((char * ) str, "%s%s", topic, gpsData);
  delay(1000);
  Serial.write(0x30);
  X = datalength + 2;
  do {
    encodedByte = X % 128;
    X = X / 128;
    if (X > 0) {
      encodedByte |= 128;
    }
    Serial.write(encodedByte);
  }
  while (X > 0);
  Serial.write(topiclength >> 8);
  Serial.write(topiclength & 0xFF);
  Serial.print(str);
  Serial.write(0x1A);
  if (Serial.findUntil("OK","ERROR")){blinkLED(2);}else{blinkLEDFast(1000);}



  // String gpsdatastr = String(gpsData);

  // uint8_t ind1 = gpsdatastr.indexOf(',');
  // //  String mode = gpsdatastr.substring(0, ind1);

  // uint8_t ind2 = gpsdatastr.indexOf(',', ind1 + 1);
  // fixStatus = gpsdatastr.substring(ind1 + 1, ind2);
  // fixStatus = fixStatus.substring(0, 1);
  
  // uint8_t ind3 = gpsdatastr.indexOf(',', ind2 + 1);
  // String utctime = gpsdatastr.substring(ind2 + 1, ind3);
  // timestamp32bits stamp = timestamp32bits();

  // unixTimeInt = stamp.timestamp(
  //                 (utctime.substring(2, 4)).toInt(),
  //                 (utctime.substring(4, 6)).toInt(),
  //                 (utctime.substring(6, 8)).toInt(),
  //                 (utctime.substring(8, 10)).toInt(),
  //                 (utctime.substring(10, 12)).toInt(),
  //                 (utctime.substring(12, 14)).toInt());
  // lastUnixTime = String(unixTimeInt);
  // lastUnixTime = lastUnixTime.substring(0, 10);
  // // badCharChecker(lastUnixTime);

  // unsigned long gpsTimeInt = unixTimeInt - 315961182 ; //315964782 - 3600
  // gpsTime = String(gpsTimeInt);
  // t2 = gpsTimeInt;

  // uint8_t ind4 = gpsdatastr.indexOf(',', ind3 + 1);
  // latitude = gpsdatastr.substring(ind3 + 1, ind4);
  // while (strlen(latitude.c_str()) < 10) {
  //   latitude += '0';
  // }

  // uint8_t ind5 = gpsdatastr.indexOf(',', ind4 + 1);
  // longitude = gpsdatastr.substring(ind4 + 1, ind5);
  // while (strlen(longitude.c_str()) < 11) {
  //   longitude += '0';
  // }
  // uint8_t ind6 = gpsdatastr.indexOf(',', ind5 + 1);
  // //  String altitude = gpsdatastr.substring(ind5 + 1, ind6);

  // uint8_t ind7 = gpsdatastr.indexOf(',', ind6 + 1);
  // speed = gpsdatastr.substring(ind6 + 1, ind7);
  // speed = speed.substring(0, 6);
  // while (strlen(speed.c_str()) < 6) {
  //   speed = '0' + speed;
  // }

  // uint8_t ind8 = gpsdatastr.indexOf(',', ind7 + 1);
  // course = gpsdatastr.substring(ind7 + 1, ind8);
  // course = course.substring(0, 6);
  // while (strlen(course.c_str()) < 6) {
  //   course = '0' + course;
  // }

  // uint8_t ind9 = gpsdatastr.indexOf(',', ind8 + 1);
  // //  String fixmode = gpsdatastr.substring(ind8 + 1, ind9);
  // uint8_t ind10 = gpsdatastr.indexOf(',', ind9 + 1);
  // //  String reserved1 = gpsdatastr.substring(ind9 + 1, ind10);
  // uint8_t ind11 = gpsdatastr.indexOf(',', ind10 + 1);
  // //  String HDOP = gpsdatastr.substring(ind10 + 1, ind11);

  // uint8_t ind12 = gpsdatastr.indexOf(',', ind11 + 1);
  // pdop = gpsdatastr.substring(ind11 + 1, ind12);
  // pdop = pdop.substring(0, 4);
  // while (strlen(pdop.c_str()) < 4) {
  //   pdop = '0' + pdop;
  // }

  // uint8_t ind13 = gpsdatastr.indexOf(',', ind12 + 1);
  // //  String VDOP = gpsdatastr.substring(ind12 + 1, ind13);
  // uint8_t ind14 = gpsdatastr.indexOf(',', ind13 + 1);
  // //  String reserved2 = gpsdatastr.substring(ind13 + 1, ind14);
  // uint8_t ind15 = gpsdatastr.indexOf(',', ind14 + 1);
  // String viewed_satellites = gpsdatastr.substring(ind14 + 1, ind15);
  // while (strlen(viewed_satellites.c_str()) < 2) {
  //   viewed_satellites = '0' + viewed_satellites;
  // }
  // uint8_t ind16 = gpsdatastr.indexOf(',', ind15 + 1);

  // used_satellites = gpsdatastr.substring(ind15 + 1, ind16);
  // used_satellites = used_satellites.substring(0, 2);
  // while (strlen(used_satellites.c_str()) < 2) {
  //   used_satellites = '0' + used_satellites;
  // }

  // //  uint8_t ind17 = gpsdatastr.indexOf(',', ind16 + 1);
  // ////  String reserved3 = gpsdatastr.substring(ind16 + 1, ind17);
  // //  uint8_t ind18 = gpsdatastr.indexOf(',', ind17 + 1);
  // ////  String N0max = gpsdatastr.substring(ind17 + 1, ind18);
  // //  uint8_t ind19 = gpsdatastr.indexOf(',', ind18 + 1);
  // ////  String HPA = gpsdatastr.substring(ind18 + 1, ind19);
  // //  uint8_t ind20 = gpsdatastr.indexOf(',', ind19 + 1);
  // ////  String VPA = gpsdatastr.substring(ind19);
  // //////////////////////////////////////////////////////////////////
  // if ((imei.length()!=15)){imei="869170031000000";}
  // if ((fixStatus.length()!=1)){fixStatus="1";}
  // if ((latitude.length()!=10)){latitude="33.5619400";}
  // if ((longitude.length()!=11)){latitude="-7.63555500";}
  // if ((speed.length()!=6)){speed="000.00";}
  // if ((used_satellites.length()!=2)){used_satellites="01";}
  // if ((course.length()!=6)){course="0357.6";}
  // if ((lastUnixTime).length()!=10){lastUnixTime="1600000000";}
  // // wdt_disable();
  // if ((fixStatus.toInt() == 1) && (latitude.toInt() > 20) && (longitude.toInt() < 0)&&(badCharCounter==0)&&(lastUnixTime!=previousUnixTime)) {
  //   previousUnixTime=lastUnixTime;
  //   started = false;
  //   restarted=false;
  //   insertMem();
  //   getImei();
  //   return true;
  // } else {return false;badCharCounter=0;} 
}
// void sendAtCom(char *AtCom) {
//   flushSim();
//   wdt_enable(WDTO_2S);
//   Serial.println(AtCom);
//   String tempGSM = Serial.readString();
//   wdt_disable();
// }
void getImei() {
  flushSim(); 
  // wdt_enable(WDTO_2S);
  Serial.println("AT+GSN");
  String tempGSM = Serial.readString();
  String tempIMEI=tempGSM;
  if (strstr(tempGSM.c_str(), "OK")) {
    imei = strstr(tempIMEI.c_str(), "8");
    imei= imei.substring(0, 15);
  }else{imei="869170031000000";}
  // wdt_disable();
}
uint8_t getGsmStat() {
  flushSim();
  // wdt_enable(WDTO_8S);
  Serial.println("AT+CREG?");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(',');
  String gsmStat = tempGSM.substring(ind1 + 1, ind1 + 2);
  // wdt_disable();
  return gsmStat.toInt();
}
String batteryLevel() {
  flushSim();
  String batteryLevel="0";
  Serial.setTimeout(1000);
  uint16_t waitInterval=2500;
  currentMillis = millis();
  previousMillis = millis();
  while((Serial.println("AT+CBC")<=0)&&((currentMillis - previousMillis) <= waitInterval)){flushSim();currentMillis=millis();delay(1000);/*wdt_reset();*/}
  if ((currentMillis - previousMillis) <= waitInterval)
  {
    String tempGSM = "0";
    tempGSM=Serial.readString();
    int ind1 = tempGSM.indexOf(',');
    int ind2 = tempGSM.indexOf(',', ind1 + 1);
    //String chargeState = tempGSM.substring( 1, ind1 + 1);
    batteryLevel = tempGSM.substring(ind1 + 1, ind2);
    uint16_t vbat = batteryLevel.toInt();
    // if (vbat<=7){lowBat=true;}else{lowBat=false;}
    if (vbat >= 100) {
      sprintf(batteryLevel.c_str(), "%d", vbat);
    }
    else if ((vbat < 100) && (vbat > 9)) {
      sprintf(batteryLevel.c_str(), "0%d", vbat);
    } else if (vbat < 10) {
      char charbat[3] = {0};
      itoa(vbat, charbat, 10);
      sprintf(batteryLevel.c_str(), "00%d", vbat);
    } else {strcpy(batteryLevel.c_str(), "000");}
   return batteryLevel;
  }else {strcpy(batteryLevel.c_str(), "001");return batteryLevel;} 
}
String rssiLevel() {
  flushSim();
  // wdt_enable(WDTO_4S);
  Serial.println("AT+CSQ");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(':');
  int ind2 = tempGSM.indexOf(',');
  String rssiLevel = tempGSM.substring(ind1 + 1, ind2);
  uint16_t n = rssiLevel.toInt();
  int r;
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  int p = abs(r);
  char charRSSI[8] = {0};
  if (p >= 100) {
    sprintf(charRSSI, "-%d", p);
  } else sprintf(charRSSI, "-0%d", p);
  // wdt_disable();
  return String(charRSSI);
}
bool gprsOn() {
  if(gsmCheck(20000)){
    sendAtFram(5000, 31140, 9, "OK", "ERROR", 5); //"AT+CFUN=1"
    sendAtFram(5000, 31184, 29, "OK", "OK", 5); //"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""
    if (sendAtFram(5000, 31213, 12, "OK", "OK", 5)) { //"AT+SAPBR=1,1"
      if (sendAtFram(5000, 31225, 8, "OK", "ERROR", 5)) { //"AT+CIICR"
        if (sendAtFram(5000, 31233, 8, ">", "ERROR", 5)) { //"AT+CIFSR"
          return true;
        } else return false;
      } else return false;
    } else return false;
  }
}
void gprsOff() {
  sendAtFram(5000, 31599, 10, "OK", "ERROR", 5); //"AT+CGATT=0"
}
bool getGprsState() {//very slow!!
  Serial.setTimeout(10000);
  flushSim();
  Serial.write("AT+CGATT?");
  //Serial.println("");
  if (Serial.findUntil("1", "OK")) {
    return true;
  } else return false;
}
void flushSim() {
  // wdt_enable(WDTO_2S);
  uint16_t timeoutlp = 0;
  while (timeoutlp++ < 40) {
    while (Serial.available()) {
      Serial.read();
      timeoutlp = 0;  // If char was received reset the timer
    }
    // wdt_reset();
    delay(1);
  }//wdt_disable();
}
void writeDataFram(char* dataFram) {
  uint8_t dataFramSize = strlen(dataFram);
  for (unsigned long i = framWritePosition; i <= (dataFramSize + framWritePosition); i++)
  {
    delay(1);
    fram.write8(i, dataFram[(i - framWritePosition)]);
    
    uint8_t test1 = fram.read8(i);
    
    if (insertFlag){
      if ((test1==32)||(test1==0)||(test1!=int(dataFram[(i - framWritePosition)]))){
        fram.write8(i, 120);}
      }else if (test1!=int(dataFram[(i - framWritePosition)]))
      {fram.write8(i, 120);}

  } framWritePosition += (dataFramSize) ; /*wdt_disable();*/
}
void writeDataFramDebug(char* dataFram, long p1) {
  // wdt_enable(WDTO_1S);
  for (unsigned long i = p1; i < (p1 + strlen(dataFram)); i++)
  {
    delay(1);
    fram.write8(i, dataFram[(i - p1)]);
    uint8_t test1 = fram.read8(i);
    if (test1!=int(dataFram[(i - p1)])){fram.write8(i, 120);} //x
    // wdt_reset();
  // }wdt_disable();
  }
}
void blinkLED(int k) {
  for (int i = 0; i < k; i++) {
    digitalWrite(A0, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(A0, LOW);    // turn the LED off by making the voltage LOW
    delay(100);
  }
}
void blinkLEDFast(int k) {
  for (int i = 0; i < k; i++) {
    digitalWrite(A0, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(20);                       // wait for a second
    digitalWrite(A0, LOW);    // turn the LED off by making the voltage LOW
    delay(30);
  }
}
void clearMemory(int size) {
  // wdt_enable(WDTO_1S);
  for (uint16_t a = 0; a < size; a++) {
    fram.write8(a, 59);  //;
    // wdt_reset();
  // }wdt_disable();
  }
  framWritePosition = 0;
}
void clearMemoryDiff(int size, int size1) {
  // wdt_enable(WDTO_1S);
  for (uint16_t a = size; a < size1; a++) {
    fram.write8(a, 59); //;
    // wdt_reset();
  }
  // wdt_disable();
  // if(size1==getCounter()){framWritePosition = 0;}
}
void clearMemoryDebug(unsigned long size) {
  // wdt_enable(WDTO_1S);
  for (uint16_t a = 32000; a < size; a++) {
    fram.write8(a, 48); //0
    // wdt_reset();
  }
  // wdt_disable();
}
void insertMem() {
  framWritePosition = getCounter() * SizeRec;
  insertFlag=true;
#ifndef CARGEO 
  char* ourImei=imei.c_str();
  writeDataFram(ourImei);                    //15
#endif
  writeDataFram(fixStatus.c_str());               //1
  writeDataFram(latitude.c_str());                //10
  writeDataFram(longitude.c_str());               //11
  writeDataFram(speed.c_str());                   //6
  writeDataFram(used_satellites.c_str());         //2
  writeDataFram(course.c_str());                  //6
  writeDataFram(batteryLevel().c_str());          //3
#ifndef CARGEO
  writeDataFram(lastUnixTime.c_str());            //10
  writeDataFram("00");
#endif
#ifdef CARGEO
  writeDataFram(gpsTime.c_str());                  //10
#endif


  insertFlag=false;
  // wdt_enable(WDTO_4S);
  // Wire.requestFrom(8, 4);
  // byte lb1; byte hb1; byte lb2; byte hb2;
  // while (Wire.available()){lb1=Wire.read();hb1=Wire.read();lb2=Wire.read();hb2=Wire.read();received=true;}
  //   if(received){
  //     wdt_disable();
  //     char str1[3];sprintf(str1, "%d", word(hb1,lb1));
  //     char str2[3];sprintf(str2, "%d", word(hb2,lb2));
  //     writeDataFram(str1);
  //     writeDataFram(str2);
  //     received=false;
  //   }else {writeDataFram("00");}
  //   wdt_enable(WDTO_1S);
  //   Wire.beginTransmission(8);
  //   Wire.write('r');
  //   Wire.endTransmission();
  //   wdt_disable();
  incrementCounter();
  if(((getCounter()/limitToSend)>=1)&&(getCounter()%limitToSend)==0){writeDataFramDebug("1",(32080+(getCounter()/limitToSend)));}
}
void incrementCounter() {
  int countVal = getCounter();
  countVal++;
  writeDataFramDebug(complete(String(countVal), 3).c_str(), 32000);
}
String complete(String s, int t) {
  // wdt_enable(WDTO_2S);
  while (strlen(s.c_str()) < t) {
    s = '0' + s;
  }
  // wdt_disable();
  return s;
}
int getCounter() {
  // wdt_enable(WDTO_2S);
  String retour = " ";
  for (long i = 32000; i <= 32002; i++) {
    char Buffer[5] = {0};
    uint8_t test = fram.read8(i);
    sprintf(Buffer, "%c", test);
    retour += Buffer;
  }
  // wdt_disable();
  return retour.toInt();
}
int getBatchCounter(uint16_t j) {
  // wdt_enable(WDTO_2S);
  String retour = " ";
  char Buffer[5] = {0};
  uint8_t test = fram.read8(32080+j);
  sprintf(Buffer, "%c", test);
  retour = Buffer;
  // wdt_disable();
  return retour.toInt();
}
int getValue(uint16_t position, uint8_t largeur) {
  // wdt_enable(WDTO_2S);
  String retour = " ";
  for (long i = position; i < position + largeur; i++) {
    char Buffer[2] = {0};
    uint8_t test = fram.read8(i);
    sprintf(Buffer, "%c", test);
    retour += Buffer;
  }
  // wdt_disable();
  return retour.toInt();
}
void incrementValue(uint16_t position, uint8_t largeur) {
  int countVal = getValue(position, largeur);
  countVal++;
  writeDataFramDebug(complete(String(countVal), largeur).c_str(), position);
}
void resetSS() {
  cfunReset();
  turnOnGns(30000);
  gsmCheck(20000);
  gprsOn();
  restarted=true;
  gnsFailCounter = 0;
  gpsFailCounter = 0;
  httpActionFail = 0;
  FirstStartCounter = 0;
  ReStartCounter=0;
}
void hardResetSS() {
  // pinMode(5, OUTPUT);//PWR KEY
  // digitalWrite(5, LOW);
  // delay(2000);
  // pinMode(5, INPUT_PULLUP);
  // delay(100);
  // powerUp();
  // Serial.begin(4800);
  sendAtFram(6000, 31730, 11, "OK", "ERROR", 1);  //CFUN=1,1
  Serial.begin(4800);
  turnOnGns(30000);
  while ((getGsmStat()!=1)&&(getGsmStat() != 5))  {delay(500);}
  gprsOn();
  restarted=true;
  gnsFailCounter = 0;
  gpsFailCounter = 0;
  httpActionFail = 0;
  FirstStartCounter = 0;
  ReStartCounter=0;
}
void cfunReset(){
  sendAtFram(6000, 31741, 9, "OK", "ERROR", 1);  //CFUN=0
  sendAtFram(6000, 31140, 9, "OK", "ERROR", 1);  //CFUN=1
}
bool sendAtFram(long timeout, uint16_t pos1, uint16_t pos2, char* Rep, char* Error, int nbRep) {
  if (analogRead(A3)>200)
  {
    flushSim();
    Serial.setTimeout(timeout);
    for (uint16_t a = pos1; a < pos1 + pos2; a++)
    {
      uint8_t test = fram.read8(a);
      char Buffer[2] = {0};
      if ((test==0) ){sprintf(Buffer, "%c", 120);}else{sprintf(Buffer, "%c", test);}
      Serial.print(Buffer);
    } Serial.println("");
    int compteur = 0;
    while ((!Serial.findUntil(Rep, Error)) && (compteur < nbRep)) {
      flushSim();
      for (uint16_t a = pos1; a < pos1 + pos2; a++)
      {
        uint8_t test = fram.read8(a);
        char Buffer[2] = {0};
        if ((test==0) ){sprintf(Buffer, "%c", 120);}else{sprintf(Buffer, "%c", test);}
        Serial.print(Buffer);
      } Serial.println("");
      compteur++;
      delay(50);
    }
    if (compteur <= nbRep){return true;} else{return false;}
    Serial.setTimeout(1000);
  }else{return false;}
}
bool fireHttpAction(long timeout, char* Commande, char* Rep, char* Error) {
  if (analogRead(A3)>200){
    flushSim();
    Serial.setTimeout(timeout);
    Serial.print(Commande);
    Serial.println(1, DEC);
    if(Serial.findUntil(Rep, Error)){
      ping =false;
    #ifdef SWEEP
    blinkLED(2);
    #endif
      httpActionFail=0;
      return true;
    } else{
    #ifdef SWEEP
    blinkLED(4);
    #endif
      ping = true;
      httpActionFail++;
      return false;
    }
    Serial.setTimeout(1000);
  }else{return false;}
}
void clearValue() {
  if (getValue(32010, 2) > 0 ) {
    clearMemoryDiff(32010, 32080);
  }
}
bool insertGpsData() {
  if (getGpsData()) {gpsFailCounter = 0;insertMem();t1 = t2;return true;
  } else return false;
}
bool powerCheck(){
  if (analogRead(A3)<200){
    pinMode(6, OUTPUT);//RST KEY  
    digitalWrite(6, LOW);//PULL RST KEY TO GROUND FOR >105MS
    delay(200);
    pinMode(6, INPUT);// RST BACK TO NORMAL
    delay(3000);
    blinkLEDFast(10);powerCycle();}
  if (analogRead(A3)>200){return true;}else{return false;}
}