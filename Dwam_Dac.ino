
#include <EEPROM.h>
#include "dwam_utils.h"

Message message;
State curState;

uint8_t iCurrentSetup=0;
uint8_t iCurrentSend=0;

void setup() {
  Serial.begin(115200);
  allInit();
  _delay_ms(100);
  
}

unsigned long ulLastReqLevel = 0;
unsigned long ulLastReqSt13 = 0;
unsigned long ulSendStateScr = 0;
unsigned long ulLastRecieveMessage = 0;
unsigned long ulCurrent;

uint8_t currentStream = 3;
uint8_t currentReciever = 1;
uint8_t currentVolume = 1;
uint8_t currentZone = 1;
uint8_t currentMute = 0;
bool bRestart = false;
bool blinkSys = false;
bool bSendState = false;

uint8_t streamStates[4] = {0, 0, 0, 0};
uint8_t sendStates[4] = {0, 0, 0, 0};

bool bSendMessage = false;
bool bStartSend = false;
uint8_t aMessage[16];

void prepareMessage(uint8_t Type, uint8_t Value){
  for (int i=0; i<16; i++) aMessage[i] = 0;
  bSendMessage = true;

  aMessage[0] = 0xA0;
  aMessage[1] = Type;
  aMessage[2] = Value;
  aMessage[3] = currentZone;
  aMessage[4] = 0x7F;
}

bool ShowState(){
  if (bufferRec[0] != 0xA0) return false;
  if (bufferRec[1] != 0x90) return false;

  sysBlink();

  curState.States = 0x00;

  if (bufferRec[2] == 0x01) curState.States = curState.States | 0x01;
  if (bufferRec[3] == 0x01) curState.States = curState.States | 0x02;
  if (bufferRec[4] == 0x01) curState.States = curState.States | 0x04;

  sysShowState(curState.Input, curState.States);

  uint8_t stateTest = 1 << (curState.Input-1);

  if (bufferRec[curState.Input+1] == 0x01){
    darrInitSource(curState.Input);
    dacSetVolume(curState.Volume);
  }
  return true;
}

void  OnMessage(uint8_t *message){
  if (message[0] != 0xA0) return;
  if (ShowState()) return ;
  if (message[3] != curState.Zone) return ;
  
    if (message[1] == 0x90) ;//return;  // Принять инфу о состоянии входов приемника.
    if (message[3] != currentZone) return ;
    if (message[1] == 0x02){
      curState.Mute = message[2];
      Serial.print("    mute message: "); 
      Serial.println(message[2], HEX);
      dacMute(curState.Mute);
    }
    if ((message[1] == 0x03)||(message[1] == 0x04)){ // громкость.
      dacSetVolume(message[2]);
      curState.Volume = message[2];
      curState.Mute = 0;
      dacMute(currentMute);
      Serial.print("    current volume: "); 
      Serial.println(message[2], HEX);
      stateUpdate(&curState);
      //return;
    }
    if (message[1] == 0x05){ 
      curState.Transmitter = 1;  
      stateUpdate(&curState);
      bRestart = true;
    }
    if (message[1] == 0x06){    
      curState.Transmitter = 2;   
      stateUpdate(&curState);
      bRestart = true;    
    }
    if (message[1] == 0x07){
      curState.Transmitter = 3;  
      stateUpdate(&curState);
      bRestart = true;         
    }
    if (message[1] == 0x08){ 
      curState.Input = 1;  
      curState.States = 0;
      stateUpdate(&curState);
    }
    if (message[1] == 0x09){    
      curState.Input = 2;      
      curState.States = 0;
      stateUpdate(&curState);
    }
    if (message[1] == 0x0A){
      curState.Input = 3;   
      curState.States = 0;
      stateUpdate(&curState);
    }   
}

void SendChangeTrassmitter(){
}

void MainInterrupt(uint16_t a13Value){
  _delay_ms(10);

  if (a13Value == 0x0000) return;
  if ((a13Value & 0x40) == 0x40){
    uint8_t a19 = i2c_ReadByte(0x40, 0x19); 
  }
  if ((a13Value & 0x04) == 0x04){
    message.bSended = false;
  }
  if ((a13Value & 0x08) == 0x08){
    message.bSended = false;
    message.bSetuped = false;
  }
  if ((a13Value & 0x01) == 0x01){
    if (message.bSended == false){
      darrTrySend(&message);
    }
  }
  if ((a13Value & 0x02) == 0x02){
    uint8_t message[16];
    while(darrTryRecieve(message)){
      OnMessage(message);
    }
  }
  if ((a13Value & 0x200) == 0x200){
    uint8_t a19 = i2c_ReadByte(0x40, 0x19); 
  }
}

void loop() {
 // Wire.begin();
  //bool blinkSys = false;
 // while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
 // }
  bRestart = false;

  message.bSetuped = false;
  message.bSended = false;

  stateLoad(&curState);
  Serial.print("R:"+String(curState.Transmitter, HEX)+" S:"+String(curState.Zone, HEX)+" Z:"+String(curState.Zone, HEX)+" V:"+String(curState.Volume, HEX));

  uint8_t uiResult;

  darrReset();
  _delay_ms(10);
  Serial.println("  Init: "+String(darrInit(), HEX));
  _delay_ms(10);
  Serial.println("  Init RF: "+String(darrInitRF(), HEX));
  _delay_ms(10);
  Serial.println("  Init Reciever["+String(curState.Transmitter)+"]: result "+String(darrInitDWAM(curState.Transmitter), HEX));
  _delay_ms(10);
  dacInit(currentVolume);
  Serial.println("  Init DAC. Volume:"+String(curState.Volume, DEC));
  _delay_ms(10);

  ulLastReqLevel = (millis()/100)*100;
  ulLastReqSt13 = (millis()/500)*500+30;
  ulSendStateScr = (millis()/1000)*1000+60;

  sysBlink(false);
  
  while (1){
    ulCurrent = millis();
    if (ulCurrent>ulLastReqLevel) {
      darrReadAGCD();
     _delay_ms(10);
      MainInterrupt(i2c_ReadWord(0x40, 0x13));

      
      while (ulCurrent>=ulLastReqLevel) ulLastReqLevel = ulLastReqLevel+100;
    }
    if (ulCurrent>ulLastReqSt13) {
     bSendState = true;
     _delay_ms(10);

      scrSendState(curState);
      while (ulCurrent>=ulLastReqSt13) ulLastReqSt13 = ulLastReqSt13+500;
    }
    if (ulCurrent>ulSendStateScr) { 
      if ((ulCurrent-ulLastRecieveMessage) >= 1500){
        MainInterrupt(0x0002) ;
      }
      while (ulCurrent>=ulSendStateScr) ulSendStateScr = ulSendStateScr+1000;
    }
    if (bRestart == true) return;
    _delay_ms(1);
  }
}
