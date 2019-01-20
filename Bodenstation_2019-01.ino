#include <SD.h>
#include <SPI.h>
#include <RH_RF69.h>
#include "config.h"
#include <util/crc16.h>
#include <math.h>

int stepCounter;
int stepsN;
int stepsNC;
int stepsD;
int stepsDC;
int stepsNMax = 275;
int n;

double targetLONG;
double targetLAT;
double targetALT;
double homeLONG;
double homeLAT;
boolean cont = true;

File logfile;

#if DEBUG
#define VPRINT(data) Serial.print(data); logfile.print(data);
#define VPRINTLN(data) Serial.println(data); logfile.println(data);
#else
#define VPRINT(data)
#define VPRINTLN(data)
#endif

//Function Prototypes
uint16_t calcCRC(byte *pbuff, int plen);

// Module initialisieren
// Radio
uint8_t key[] = { AES_KEY };
RH_RF69 rfm69(RADIO_CS, RADIO_INT);


int gps_eval(char *dataset, float *coordinates){
  if(dataset[0] != 'G'){
    return 0; // not a gps dataset
  }
  else{
    float longitude;
    float latitude;
    float altitude;
    char *ptr;
    char delimiter[] = ",:";
    ptr = strtok(dataset, delimiter);
    int counter = 0;
    while(ptr != NULL){
      if(counter == 2){
        coordinates[0] = (float)atof(ptr); // LONG
      }
      else if(counter == 4){
        coordinates[1] = (float)atof(ptr); // LAT
      }
      else if(counter == 8){
        coordinates[2] = (float)atof(ptr); // ALT
      }
      ptr = strtok(NULL, delimiter);
      counter++;
    }
    return 1;
  }

}


void setup() {
  Serial.begin(115200); 
  //Serial.begin(9600); warum zwei Übertragungsraten ?
  // SD init
  VPRINT("Init: SD ");
  /*if (!SD.begin(SD_PIN)) {
    VPRINTLN("[FAILED]"); 
    while (1);
  }
  VPRINTLN("[OK]");*/


  // Radio init
  VPRINT("Init: RFM69 ");
  if (!rfm69.init()) {
    VPRINTLN("[FAILED]");
    while (1);
  }
  rfm69.setFrequency(433.0);
  rfm69.setTxPower(20);
  rfm69.setEncryptionKey(key);
  rfm69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
  VPRINTLN("[OK]"); 

  digitalWrite(directionA, LOW);
  digitalWrite(enableA, LOW);

  digitalWrite(directionB, LOW);
  digitalWrite(enableB, LOW);

  pinMode(stepA, OUTPUT); //step
  pinMode(directionA, OUTPUT); //direction
  pinMode(enableA, OUTPUT); //enable

  pinMode(stepB, OUTPUT); //step
  pinMode(directionB, OUTPUT); //direction
  pinMode(enableB, OUTPUT); //enable

  homeLONG = 42434.39;
  homeLAT  = 173184.93;

  stepsDC = 0;
  stepsNC = 0;
}

void loop() {
  if (rfm69.available()) {
    //Change Alex: 60 Zeichen zu kurz  
	uint8_t buf[120];
    uint8_t len = sizeof(buf);
		
	char msg[120]; //msg - send to rasberry buffer
	uint16_t crc;

    if (rfm69.recv(buf, &len)) {
      /*logfile = SD.open("log.txt", FILE_WRITE);
      logfile.println((char*)buf);
      logfile.println("loop");*/
      Serial.println("loop");
      //logfile.close();

	  //Send msg to Rasberry
	  //calculate CRC-checksum
	  crc = calcCRC(buf, strlen(buf));
	  sprintf(msg, "<m><d>%s</d><CRC:%d></m>", (char*)buf, crc);
	  Serial.println(msg);
    

      float cansat_position[3];
      if(gps_eval((char*)buf, cansat_position)){
        move_antenna(cansat_position[0]*3600, cansat_position[1]*3600, cansat_position[2]);
      }
    } else {
      Serial.println("recv failed");
    }
  }
  //Antennentest
  if (n != 1){

  move_antenna(42388.71, 173168.13, 700);//E
  move_antenna(42441.68, 173178.47, 700); //C
  move_antenna(42486.91, 173155.77, 700); //D
  move_antenna(42396.34, 173189.83, 700); //G
  move_antenna(42491.21, 173189.74, 700); //B
  move_antenna(42460.25, 173213.42, 700); //A
  move_antenna(42431.52, 173215.45, 700); //F
  n=1;
  }
}




void move_antenna(double longitude, double latitude, double altitude) {
  //Drehung
  double deltaLONG = (longitude - homeLONG);
  double deltaLAT = (latitude - homeLAT);
  
  double DDegree = atan(fabs(deltaLONG)/fabs(deltaLAT));
  //Serial.println(DDegree);
  int Dsteps = (int)(1100 * (DDegree / (2 * M_PI)));
 

  if (deltaLONG > 0 && deltaLAT > 0) {
    stepsD = Dsteps; 
  }
  else if (deltaLONG > 0 && deltaLAT < 0) {
    stepsD = 550- Dsteps; 
  }
  else if (deltaLONG < 0 && deltaLAT < 0) {
    stepsD = 550 + Dsteps; //550+Dsteps
  }
  else if (deltaLONG < 0 && deltaLAT > 0) {
    stepsD = 1100 - Dsteps; //825+Dsteps
  }

  if (stepsD != stepsDC) {
    int DeltaD = stepsD - stepsDC;
    //Serial.println(DeltaD);
    if (DeltaD > 0 && DeltaD <= 550) {
      
      for (stepCounter = 0; stepCounter < DeltaD; stepCounter++) {
        
        digitalWrite(directionA, LOW);
        digitalWrite(stepA, HIGH);
        delayMicroseconds(300);
        digitalWrite(stepA, LOW);
        delayMicroseconds(10000);

        if (stepsDC==1099){
          stepsDC=0;
          }
          else{
        stepsDC++;
          }
      }
    } else if (DeltaD < 0 && DeltaD >=-550) {
      //Serial.println(abs(DeltaD));    
      for (stepCounter = 0; stepCounter < abs(DeltaD); stepCounter++) {
        
        digitalWrite(directionA, HIGH);
        digitalWrite(stepA, HIGH);
        delayMicroseconds(300);
        digitalWrite(stepA, LOW);
        delayMicroseconds(10000);
        if (stepsDC==0){
          stepsDC=1099;
          }
          else{
        stepsDC--;
          }

      }
    }
    else if (DeltaD > 550){
      //Serial.println(1100-DeltaD);
      for (stepCounter = 0; stepCounter < 1100-DeltaD; stepCounter++) {
        
        digitalWrite(directionA, HIGH);
        digitalWrite(stepA, HIGH);
        delayMicroseconds(300);
        digitalWrite(stepA, LOW);
        delayMicroseconds(10000);

        if (stepsDC==0){
          stepsDC=1099;
          }
          else{
        stepsDC--;
          }
      }
      }
   else if (DeltaD < -550){
    
    for (stepCounter = 0; stepCounter < abs(1100+DeltaD); stepCounter++) {
        
        digitalWrite(directionA, LOW);
        digitalWrite(stepA, HIGH);
        delayMicroseconds(300);
        digitalWrite(stepA, LOW);
        delayMicroseconds(10000);
        if (stepsDC==1099){
          stepsDC=0;
          }
          else{
        stepsDC++;
          }
    }
  }

  //Neigung
  double distanceLONG = deltaLONG * 30.9;
  double distanceLAT = deltaLAT * 30.9;
  double distance = sqrt(sq(distanceLONG) + sq(distanceLAT));
  int kruemmung = (int)(sqrt(sq((long long)(distance)) + sq((long long)(6371000))) - 6371000);
  double normalHeight = (altitude - kruemmung);
  double NDegree = acos((normalHeight) / (sqrt(sq(distanceLONG) + sq(distanceLAT) + sq(normalHeight))));
  int stepsN = (int)(1100 * (NDegree / (2 * M_PI)));
  //Serial.println(stepsN);

  if (stepsN != stepsNC) {
    int DeltaN = stepsN - stepsNC;
    Serial.println(DeltaN);
    if (DeltaN > 0) {
      for (stepCounter = 0; stepCounter < DeltaN; stepCounter++) {
        
        if (stepsNC <400){
        digitalWrite(directionB, LOW);
        digitalWrite(stepB, HIGH);
        delayMicroseconds(300);
        digitalWrite(stepB, LOW);
        delayMicroseconds(10000);
        stepsNC++;
        }
      }
    }
    else if (DeltaN < 0) {
      for (stepCounter = 0; stepCounter < (abs(DeltaN)); stepCounter++) {
        
        if (stepsNC>0){
        digitalWrite(directionB, HIGH);
        digitalWrite(stepB, HIGH);
        delayMicroseconds(300);
        digitalWrite(stepB, LOW);
        delayMicroseconds(10000);

        stepsNC--;
        }
      }
    }
  }
}
}

uint16_t calcCRC(byte *pbuff, int plen)
{
  uint16_t crc=0; // starting value as you like, must be the same before each calculation
  for (uint16_t i=0;i<plen;i++) // for each character in the string
  {
    // crc = _crc16_update(crc, str[i]); // update the crc value: CRC16
    crc = _crc_xmodem_update(crc, pbuff[i]); // update the crc value: CRC16, XMODEM
  }
  return crc;
}
