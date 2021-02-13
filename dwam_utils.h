#ifndef DWAM_UTILS_H 
#define DWAM_UTILS_H 

#include <inttypes.h>

#define DWAM_RESET_PIN 5


struct Message{
  bool bSended;
  bool bSetuped;
  uint8_t data[16];
};

struct State{  
  uint8_t Transmitter;
  uint8_t Zone;
  uint8_t Input;
  uint8_t Volume;
  uint8_t Mute;
  uint8_t States;
};

void stateUpdate(State *state);
void stateLoad(State *state);

void sysBlink(bool bTransConn = true);

void sysShowState(uint8_t curStream, uint8_t curStates);

void darrSendChangeTransmitter(uint8_t index);
void prepareMessage(Message *mess, uint8_t Type, uint8_t Value);

void allInit();

uint8_t i2c_TestAddress(uint8_t addr);

uint8_t  i2c_ReadByte(uint8_t addr, uint8_t command);
uint16_t i2c_ReadWord(uint8_t addr, uint8_t command);
uint8_t  i2c_ReadData(uint8_t addr, uint8_t command, uint8_t uiSize);
uint8_t  i2c_SendTwoBytes(uint8_t addr, uint8_t val1, uint8_t val2);
uint8_t  i2c_SendThreeBytes(uint8_t addr, uint8_t val1, uint8_t val2, uint8_t val3);
void    darrReset();
uint8_t darrInitRF();
uint8_t darrInit();
uint8_t darrInitDWAM(uint8_t iRFIndex);

uint8_t darrSendMessage(uint8_t *message);

uint8_t darrReadAGCD();
uint8_t darrSendAGCD();

uint8_t darrInitSource(uint8_t source);

uint8_t darrTrySend(Message *message);
uint8_t darrTryRecieve(uint8_t *message);

void dacInit(uint8_t volume);
void dacSetVolume(uint8_t value);
void dacMute(uint8_t bMute);

void scrSendState(State state);
bool scrGetData(uint8_t *data);

extern uint8_t bufferRec[32];

#endif 
