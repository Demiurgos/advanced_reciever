
#include "dwam_utils.h"

#include <inttypes.h>
#include <Wire.h>

uint8_t currentSigStr = 0;
unsigned long ulLastSendLevel = 0;

uint8_t _aInitXeoDarr81[223] = {
    0x00, 0x90, 0x41, 0xE0, 0x00, 0xA4, 0x0B, 0x65, 0xE4, 0xAE, 0x22, 0x10, 0x3C, 0x61, 0x06, 0xE4,
    0x02, 0x79, 0x03, 0x65, 0xE4, 0xAE, 0x22, 0x50, 0x42, 0x24, 0x06, 0x65, 0x4A, 0xA3, 0xBC, 0x8C,
    0x14, 0xA9, 0x05, 0x8C, 0x12, 0xA3, 0x07, 0x2E, 0x02, 0x8C, 0x1F, 0x2D, 0x01, 0x3D, 0xBF, 0xA3,
    0x07, 0xAE, 0x22, 0x3D, 0xBF, 0xA3, 0x07, 0xAE, 0x22, 0x3D, 0xBF, 0xA3, 0x07, 0xAE, 0x22, 0x3D,
    0xBF, 0xA3, 0x07, 0xAE, 0x22, 0x3D, 0xBF, 0xA3, 0x07, 0xAE, 0x22, 0x3D, 0xBF, 0xA3, 0x07, 0xAE,
    0x22, 0x3D, 0xBF, 0xA3, 0x07, 0xAE, 0x22, 0x7D, 0x2B, 0x75, 0x9B, 0xAE, 0x22, 0xFD, 0x96, 0x36,
    0x48, 0x9F, 0xF0, 0xAE, 0x16, 0xF7, 0x04, 0xE6, 0x06, 0xE7, 0x26, 0x2F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x4F, 0x42, 0x24, 0x06, 0x65, 0x07, 0x2C, 0x7B, 0x8C, 0x1D, 0xAD, 0x01, 0x3D, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x51, 0x42, 0x24, 0x06, 0x65, 0x07, 0xC7, 0x84, 0x6D, 0x12, 0x3D,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00
};

uint8_t _aInitRF[25] = {
    0xAA, 0x07, 0x55, 0x0E, 0x4B, 0x3E, 0xC8, 0x3C, 0xC8, 0xEC, 0x59, 0x01, 0x95, 0x06, 0xD1, 0x0B, 0x00, 0x00, 0x79, 0xD1, 0x7B, 0xEA, 0x76, 0x38, 0x55
};

uint8_t _aInitMCU[44][2] = {
      {0xC9, 0x01}, {0xCA, 0x02}, {0xCB, 0x01}, {0xC8, 0x25},
      {0x13, 0x7E}, {0x15, 0x01}, {0x16, 0x12}, {0x17, 0x12}, // 0x15 select transmitter
      {0x18, 0x3F}, {0x43, 0x00}, {0x1D, 0x00}, //{0x1E, 0x3F},
      {0x1F, 0x05}, {0x2A, 0xA0}, {0x38, 0xA0}, {0x20, 0x70},
      {0x21, 0x70}, {0x22, 0x07}, {0x23, 0xE4}, {0x24, 0xE4},
      {0x29, 0x60}, {0x42, 0x21}, {0x44, 0xDF}, {0x45, 0xFF},
      {0x4A, 0x3C}, {0x81, 0x07}, {0x82, 0x40}, {0x83, 0x85},
      {0x84, 0x37}, {0x8F, 0xA0}, {0x9F, 0x75}, {0x94, 0x31},
      {0x98, 0x31}, {0xD0, 0x40}, {0xD1, 0x17}, {0xDE, 0x28},
      {0xDF, 0x30}, {0xE4, 0xA3}, {0xE5, 0x0D}, {0xEC, 0x7F},
      {0xED, 0x00}, {0x1A, 0xD2}, {0x8C, 0x86}, {0x19, 0xBA}, // {0x19, 0xBA},
      {0x10, 0x01}
};

uint8_t bufferRec[32];
uint8_t bufferSend[32];


uint8_t DT_AGCD_CanSend = 0;
int DT_AGCD_Current=0;
uint8_t DT_AGCD_Level[5];
uint8_t DT_AGCD_Level_avarage = 0;

void stateUpdate(State *state){
  EEPROM.update(0x0100, state->Transmitter); 
  EEPROM.update(0x0101, state->Input); 
  EEPROM.update(0x0102, state->Volume); 
  EEPROM.update(0x0103, state->Zone); 
}

void stateLoad(State *state){
  //EEPROM.update(0x0100, 1); 
  state->Transmitter = EEPROM.read(0x0100);
  state->Input = EEPROM.read(0x0101);
  state->Volume = EEPROM.read(0x0102);
  state->Zone = EEPROM.read(0x0103); 
  state->Mute = 0;
  state->States = 0;
}

void darrSendChangeTransmitter(uint8_t index){
  
   Serial.println("SendChangeTrassmitter! ");
  for (int i=0; i<10; i++){
    uint16_t a13 = i2c_ReadWord(0x40, 0x13);  
    _delay_ms(10);
    for (int j=0; j<3; j++){     
       uint8_t d1 = i2c_ReadByte(0x40, 0x4B);
       _delay_ms(10);
       Serial.print("    > "+String(a13, HEX)+" | "+String(d1, HEX)+"!");
       i2c_SendThreeBytes(0x40, 0x4D, 0x01, 0x00);
       _delay_ms(1);
       i2c_SendTwoBytes(0x40, 0x4F, 0x00);
       _delay_ms(1);
       i2c_SendTwoBytes(0x40, 0x4B, j);
      // i2c_SendTwoBytes(0x40, 0x4B, 0x83);
       _delay_ms(1);
       darrSendMessage(aMessage);
       _delay_ms(150);
       a13 = i2c_ReadWord(0x40, 0x13);  
       _delay_ms(250);
    }
    _delay_ms(300);
  }
}

uint8_t darrTryRecieve(uint8_t *message){
  uint8_t get4C = i2c_ReadByte(0x40, 0x4C);
  _delay_ms(10);
  if ((get4C & 0x04) == 0x04) return 0;
  
  uint8_t get4D = i2c_ReadByte(0x40, 0x4D);
  _delay_ms(10);
  uint8_t get4E = i2c_ReadByte(0x40, 0x4E);
  _delay_ms(10);        
  i2c_ReadData(0x40, 0x50, 16);
  for (int i=0l; i<16; i++){
    message[i] = bufferRec[i];
  }
  _delay_ms(10);

  return 1;
}


uint8_t darrInitSource(uint8_t source){
  _delay_ms(10);
  if (source == 1){
    i2c_SendTwoBytes(0x40, 0x23, 0x93);
  } else if (source == 2){
    i2c_SendTwoBytes(0x40, 0x23, 0xE4);
  } else {
     i2c_SendTwoBytes(0x40, 0x23, 0x39);
  }
  _delay_ms(10);
  i2c_SendTwoBytes(0x40, 0x22, 0x0F);
  _delay_ms(10);
  i2c_SendTwoBytes(0x40, 0x21, 0xF0);
  
}

uint8_t darrTrySend(Message *message){
  
  uint8_t c1 = i2c_ReadByte(0x40, 0x72);
  _delay_ms(10);
  uint8_t d1 = i2c_ReadByte(0x40, 0x4B);
  _delay_ms(10);
  if ((d1 & 0x08) == 0x00){
    if (message->bSetuped){
           Serial.print("Setuped: ");
           for (int i=0; i<6; i++){
             Serial.print(message->data[i], HEX);
             Serial.print(", ");
           }
           Serial.print(message->bSetuped, HEX);
           Serial.print(", ");
           Serial.print(message->bSended, HEX);
           Serial.println();
    }
    i2c_SendTwoBytes(0x40, 0x4B, 0x83);
    _delay_ms(10);
    
    if (message->bSetuped && !message->bSended){
      darrSendMessage(message->data);
      message->bSended = true;
      darrSendMessage(aMessage);
    } else {
      unsigned long  uCur = millis();
      if ((uCur - ulLastSendLevel) > 500){
        currentSigStr = darrSendAGCD();
        message->bSended = currentSigStr != 0;
      }
    }
  }
}

void prepareMessage(Message *mess, uint8_t Type, uint8_t Value){
  for (int i=0; i<16; i++) aMessage[i] = 0;
  mess->bSended = false;
  mess->bSetuped = false;

  mess->data[0] = 0xA0;
  mess->data[1] = Type;
  mess->data[2] = Value;
  mess->data[3] = currentZone;
  mess->data[4] = 0x7F;
}

bool bBlinkOn = false;
void sysBlink(bool bTransConn = true){
  if (bTransConn == false){
    digitalWrite(8, HIGH);    // RED
    digitalWrite(6, LOW);    // BLUE
    return;
  } else {
    digitalWrite(8, LOW);    // RED
  }
  if (bBlinkOn){
    digitalWrite(6, LOW);    // BLUE
    bBlinkOn = false;
  } else {
    digitalWrite(6, HIGH);    // BLUE
    bBlinkOn = true;
  }
}

void sysShowState(uint8_t curStream, uint8_t curStates){
  if (curStream == 1){
     digitalWrite(9, HIGH);    // BLUE
     digitalWrite(19, LOW);    // BLUE
     digitalWrite(14, LOW);    // BLUE
  } else if (curStream == 2){
     digitalWrite(9, LOW);    // BLUE
     digitalWrite(19, HIGH);    // BLUE
     digitalWrite(14, LOW);    // BLUE
  } else if (curStream == 3){
     digitalWrite(9, LOW);    // BLUE
     digitalWrite(19, LOW);    // BLUE
     digitalWrite(14, HIGH);    // BLUE
  } 
  
  if ((curStates & 0x01) == 0x01){
    digitalWrite(21, HIGH); // GREEN
    digitalWrite(20, LOW);   // RED
  } else {
    digitalWrite(21, LOW); // GREEN
    digitalWrite(20, HIGH);   // RED
  }
  
  if ((curStates & 0x02) == 0x02){
    digitalWrite(18, HIGH); // GREEN
    digitalWrite(15, LOW);   // RED
  } else {
    digitalWrite(18, LOW); // GREEN
    digitalWrite(15, HIGH);   // RED
  }
  
  if ((curStates & 0x04) == 0x04){
    digitalWrite(10, HIGH); // GREEN
    digitalWrite(16, LOW);   // RED
  } else {
    digitalWrite(10, LOW); // GREEN
    digitalWrite(16, HIGH);   // RED
  }
}

void allInit(){
  Wire.begin();
  
  pinMode(DWAM_RESET_PIN, OUTPUT);
  pinMode(6, OUTPUT);  pinMode(7, OUTPUT);  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);  pinMode(21, OUTPUT);  pinMode(20, OUTPUT);
  pinMode(19, OUTPUT);  pinMode(18, OUTPUT);  pinMode(15, OUTPUT);
  pinMode(14, OUTPUT);  pinMode(10, OUTPUT);  pinMode(16, OUTPUT);  
}

uint8_t i2c_TestAddress(uint8_t addr){  
  Wire.beginTransmission(addr);
  return (0== Wire.endTransmission());
}

uint8_t  i2c_ReadByte(uint8_t addr, uint8_t command){
  Wire.beginTransmission(addr); // transmit to device #112
  Wire.write(command);      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting
  
  Wire.requestFrom(addr, 1);   
  return Wire.read(); 
}

uint16_t i2c_ReadWord(uint8_t addr, uint8_t command){
  Wire.beginTransmission(addr); // transmit to device #112
  Wire.write(command);      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting
  
  Wire.requestFrom(addr, 2);  
  uint16_t r1 = Wire.read();
  uint16_t r2 = Wire.read();
  return (r2 << 8) + r1;
}

uint8_t i2c_ReadData(uint8_t addr, uint8_t command, uint8_t uiSize){
  Wire.beginTransmission(addr); 
  Wire.write(command);      
  Wire.endTransmission();      
  
  Wire.requestFrom(addr, uiSize);     
  for (int i=0; i<uiSize; i++){
    bufferRec[i] = Wire.read();
  }

  return uiSize;
}

uint8_t  i2c_SendTwoBytes(uint8_t addr, uint8_t val1, uint8_t val2){
  bufferSend[0] = val1; bufferSend[1] = val2;
  return Wire.writeTo(addr, bufferSend, 2);        // sends five bytes
}
uint8_t  i2c_SendThreeBytes(uint8_t addr, uint8_t val1, uint8_t val2, uint8_t val3){
  bufferSend[0] = val1; bufferSend[1] = val2; bufferSend[2] = val3;
  return Wire.writeTo(addr, bufferSend, 3);        // sends five bytes
}
void    darrReset(){
  digitalWrite(DWAM_RESET_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  _delay_ms(100);
  digitalWrite(DWAM_RESET_PIN, LOW);    // turn the LED off by making the voltage LOW
  _delay_ms(100);
  digitalWrite(DWAM_RESET_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  _delay_ms(100);
}

uint8_t darrInit(){ 

  return Wire.writeTo(0x41, _aInitXeoDarr81, 223); 
}

uint8_t darrInitRF(){
  return Wire.writeTo(0x40, _aInitRF, 25); 
}


uint8_t darrInitDWAM(uint8_t iRFIndex){
  _aInitMCU[5][1] = iRFIndex;

  uint8_t result = 0;
  for (int i=0; i<44; i++){
    result = i2c_SendTwoBytes(0x40, _aInitMCU[i][0], _aInitMCU[i][1]);
    if (result != 0) return result;
    _delay_ms(5);
  }  
  return result;
}

uint8_t darrSendMessage(uint8_t *message){
  uint16_t chk_sum = 0x50;
  bufferSend[0] = 0x50;
  for (int i=0; i<15; i++){
    bufferSend[i+1] = message[i];
    chk_sum = chk_sum + message[i];
  }
  bufferSend[16] = (chk_sum & 0xFF);
/*  Serial.print("  >  Send: ");
  for (int i=0; i<17; i++){
    Serial.print(bufferSend[i], HEX);
    Serial.print(", ");
  }
  Serial.println();*/
  return Wire.writeTo (0x40, bufferSend, 17); 
}


uint8_t darrReadAGCD(){
  DT_AGCD_Level[DT_AGCD_Current]=i2c_ReadByte(0x40, 0x74) & 0x7F;
  DT_AGCD_Current++;
  if (DT_AGCD_Current==5) {
    DT_AGCD_Current=0;
    DT_AGCD_CanSend = 1;
  }
}

void darrCalcAGCDLevel(){
  uint16_t calc = 0;
  uint8_t maxValue = 0;
  for (int i=0; i<5; i++){
    if (maxValue<DT_AGCD_Level[i]) maxValue = DT_AGCD_Level[i];
    calc += DT_AGCD_Level[i];
  }
  calc = calc - maxValue;
  calc = calc / 4; 
  DT_AGCD_Level_avarage = calc;  
}

uint8_t darrSendAGCD(){  
  if (DT_AGCD_CanSend == 0) return 0;

 // i2c_SendTwoBytes(0x40, 0x4B, 0x83);
  darrCalcAGCDLevel();

  uint8_t AGCDMessage[16];

  for (int i=0; i<16; i++) AGCDMessage[i] = 0;
  
  AGCDMessage[0] = 0x5D;
  AGCDMessage[1] = DT_AGCD_Level_avarage; 
  AGCDMessage[2] = DT_AGCD_Level_avarage; 
  AGCDMessage[3] = DT_AGCD_Level_avarage; 
  AGCDMessage[4] = 0x7F;  

  Serial.print(" Signal level:");
  Serial.print(DT_AGCD_Level_avarage);
  Serial.println(".");
  ulLastSendLevel = millis();
  darrSendMessage(AGCDMessage);

  return DT_AGCD_Level_avarage;
}



void dacInit(uint8_t volume){
  i2c_SendTwoBytes(0x4C, 0x82, 0x01);
  _delay_ms(5);
  i2c_SendTwoBytes(0x4C, 0x83, 0x11);
  _delay_ms(5);
  i2c_SendTwoBytes(0x4C, 0xA8, 0x02); // 40 регистр
  _delay_ms(5);
  i2c_SendTwoBytes(0x4C, 0xA2, 0x00); // 34 регистр
  _delay_ms(5);
  dacSetVolume(volume);
  //i2c_SendTwoBytes(0x4C, 0xBE, 0x30);
  //i2c_SendTwoBytes(0x4C, 0xBD, 0x30);  
}

const uint8_t uiVolumes[51] = {
    0xFF, 0xBD, 0xA9, 0x95, 0x7C, 0x77, 0x6B, 0x69, 0x67, 0x65, 0x63, 0x61, 0x5F, 0x5D, 0x5B, 0x59, 
    0x57, 0x55, 0x53, 0x51, 0x4F, 0x4D, 0x4B, 0x49, 0x47, 0x45, 0x43, 0x41, 0x3F, 0x3D, 0x3B, 0x39, 
    0x37, 0x35, 0x33, 0x31, 0x2F, 0x2D, 0x2B, 0x2A, 0x29, 0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22,
    0x21, 0x20, 0x1F
};

void dacSetVolume(uint8_t value){
  i2c_SendTwoBytes(0x4C, 0x82, 0x00);
  _delay_ms(5);
  i2c_SendTwoBytes(0x4C, 0x83, 0x00);
  _delay_ms(5);
  i2c_SendTwoBytes(0x4C, 0xBE, uiVolumes[value]);
  _delay_ms(5);
  i2c_SendTwoBytes(0x4C, 0xBD, uiVolumes[value]);
  _delay_ms(5);
}

void scrSendState(State state){
  bufferSend[0] = 0x01;
  bufferSend[1] = currentSigStr;
  if (state.Mute == 1){
    bufferSend[2] = state.Volume | 0x80;
  } else {
    bufferSend[2] = state.Volume;
  }
  bufferSend[3] = state.Transmitter;
  bufferSend[4] = state.Zone;
  bufferSend[5] = state.Input;
  bufferSend[6] = state.States;
  Wire.writeTo (0x30, bufferSend, 7); 
  _delay_ms(5);
  
}

void dacMute(uint8_t bMute){
  if (bMute == 1){
    i2c_SendTwoBytes(0x4C, 0x82, 0x01);
    _delay_ms(5);
    i2c_SendTwoBytes(0x4C, 0x83, 0x11);
    _delay_ms(5);
  } else {
    i2c_SendTwoBytes(0x4C, 0x82, 0x00);
    _delay_ms(5);
    i2c_SendTwoBytes(0x4C, 0x83, 0x00);
    _delay_ms(5);
  }
}

void scrDataToState(uint8_t *data, State *state){
  state->Transmitter = data[1];
  state->Input =       data[3];
  state->Volume =      data[0] & 0x7F;
  state->Zone =        data[2];
  state->Mute =        0;
  state->States =      0;
  if ((data[0] & 0x80) == 0x80) state->Mute = 1;
}

bool scrGetData(uint8_t *data){
  int iRecieveCount = 0;
  Wire.requestFrom(0x30, 6);
  while(Wire.available()) { 
    if (iRecieveCount==6) {
      while(Wire.available()){
        Wire.read();
      }
      return false;
    }
    data[iRecieveCount] = Wire.read();
    iRecieveCount++;
  }
  if (data[4] == 0xFF) return false;
  
  return (iRecieveCount == 6);
}
