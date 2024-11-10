#ifndef MERCURY236_H
#define MERCURY236_H

#include <Arduino.h>

#define BUFFER              64

#define OK                  0
#define WRONG_CRC           1
#define COMMUNICATION_ERROR 2

#pragma pack(push, 1)

typedef unsigned char uChar;

#pragma pack(pop)

class Mercury236 {
public:
    Mercury236(HardwareSerial &serial, uint8_t deRePin = 2);

    int checkChannel();
    int initConnection();
    int closeConnection();

    int getU(float *U);
    int getI(float *I);
    int getPhasesAngle(float *A);

    int getCosF(float *CosF);
    int getActivePower(float *P);
    int getReactivePower(float *S);

    void setDeRePin(uint8_t deRePin);
    void setTimeOut(uint32_t timeOut);
    void setChTimeOut(uint32_t chTimeOut);
    void setPMAddress(uint8_t pmAddress);

    void setDebugSerial(HardwareSerial &debugSerial = Serial);

private:
    HardwareSerial &serial;
    HardwareSerial *debugSerial = nullptr;

    uint32_t TIME_OUT = 500;
    uint32_t CH_TIME_OUT = 500;
    uint8_t PM_ADDRESS = 0x00;
    uint8_t DE_RE_PIN;
    
    bool DEBUG = false;

    int sendReceive(uChar *bodyBytes, int bodyLen, uChar *responceBuff);
    void writeRequest(uChar *command, int commandLen);
    int readResponce(uChar *responceBuff);
    static uint16_t ModRTU_CRC(const uChar *buf, int len);
    int checkCRC(uChar *buf, int len);

    int getThreeValues(float *values, uChar BWRI, float factor);
    int getFourValues(float *values, uChar BWRI, float factor);

    float convertThreeFloat(unsigned char* b, float factor);
    float convertFourFloat(unsigned char* b, float factor);

    void debugPrint(const String &message); 
    void debugPrint(const uChar *data, int length);
};

#endif
