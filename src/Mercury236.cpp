#include "Mercury236.h"

Mercury236::Mercury236(HardwareSerial &serial, uint8_t deRePin)
    : serial(serial), DE_RE_PIN(deRePin) {
    pinMode(DE_RE_PIN, OUTPUT);
    digitalWrite(DE_RE_PIN, LOW);
}

//===================================================================Настройки соединения================================================================
void Mercury236::setTimeOut(uint32_t timeOut) {
    TIME_OUT = timeOut;
}
void Mercury236::setChTimeOut(uint32_t chTimeOut) {
    CH_TIME_OUT = chTimeOut;
}
void Mercury236::setPMAddress(uint8_t pmAddress) {
    PM_ADDRESS = pmAddress;
}
void Mercury236::setDebugSerial(HardwareSerial &serial) {
    this->debugSerial = &serial;
    DEBUG = true;
}

//===================================================================Установка соединения================================================================
int Mercury236::checkChannel() {
    //Резервируем 2 байта для CRC
    uChar checkConnect[] = {PM_ADDRESS, 0x00, 0xFF, 0xFF};
    uChar buf[BUFFER];

    return sendReceive(checkConnect, sizeof(checkConnect), buf);
}
int Mercury236::initConnection() {
    uChar initConnect[] = {PM_ADDRESS, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF, 0xFF};
    uChar buf[BUFFER];

    return sendReceive(initConnect, sizeof(initConnect), buf);
}

int Mercury236::closeConnection() {
    uChar byeCmd[] = {PM_ADDRESS, 0x02, 0xFF, 0xFF};
    uChar buf[BUFFER];

    return sendReceive(byeCmd, sizeof(byeCmd), buf);
}

//===================================================================Получение мгновенных значений==========================================================
int Mercury236::getU(float *U) {
    return getThreeValues(U, 0x11, 100.0);
}
int Mercury236::getI(float *I) {
    return getThreeValues(I, 0x21, 1000.0);
}
int Mercury236::getPhasesAngle(float *A) {
    return getThreeValues(A, 0x51, 100.0);
}
int Mercury236::getFreq(float *F) {
    return getThreeValues(F, 0x40, 100.0);
}

int Mercury236::getCosF(float *CosF) {
    return getFourValues(CosF, 0x30, 1000.0);
}
int Mercury236::getActivePower(float *P) {
    return getFourValues(P, 0x00, 100.0);
}
int Mercury236::getReactivePower(float *S) {
    return getFourValues(S, 0x08, 100.0);
}

int Mercury236::getThreeValues(float *values, uChar BWRI, float factor) {
    uChar command[] = {PM_ADDRESS, 0x08, 0x16, BWRI, 0xFF, 0xFF};
    uChar buf[BUFFER];
    int state = sendReceive(command, sizeof(command), buf);

    if (state == OK) {
            values[0] = convertThreeFloat(&buf[1], factor);  // p1
            values[1] = convertThreeFloat(&buf[4], factor);  // p2
            values[2] = convertThreeFloat(&buf[7], factor);  // p3
    }

    return state;
}

int Mercury236::getFourValues(float *values, uChar BWRI, float factor) {
    uChar command[] = {PM_ADDRESS, 0x08, 0x16, BWRI, 0xFF, 0xFF};
    uChar buf[BUFFER];
    int state = sendReceive(command, sizeof(command), buf);

    if (state == OK) {
            values[0] = convertThreeFloat(&buf[1], factor);  // p1
            values[1] = convertThreeFloat(&buf[4], factor);  // p2
            values[2] = convertThreeFloat(&buf[7], factor);  // p3
            values[3] = convertThreeFloat(&buf[10], factor); // sum
    }

    return state;
}

float Mercury236::convertThreeFloat(unsigned char* b, float factor) {
    //Два старших разряда старшего байта указывают положение вектора полной мощности и должны маскироваться
    int val = ((b[0] & 0x3F) << 16) | (b[2] << 8) | b[1];
    return val / factor;
}

float Mercury236::convertFourFloat(unsigned char* b, float factor) {
    int val = ((b[1] & 0x3F) << 24) | (b[0] << 16) | (b[3] << 8) | b[2];
    return val / factor;
}

//===================================================================Отправка запроса================================================================

int Mercury236::sendReceive(uChar *command, int commandLen, uChar *responceBuff) {
    uint16_t crc = ModRTU_CRC(command, commandLen - 2);
    command[commandLen - 2] = crc & 0xFF;          // Младший байт CRC
    command[commandLen - 1] = crc >> 8;            // Старший байт CRC

    writeRequest(command, commandLen);

    int len = readResponce(responceBuff);

    if(len > 1){
        return checkCRC(responceBuff, len);
    }

    return COMMUNICATION_ERROR;
}

void Mercury236::writeRequest(uChar *command, int commandLen) {
    debugPrint("Sending: ");
    debugPrint(command, commandLen);

    digitalWrite(DE_RE_PIN, HIGH);
    serial.write(command, commandLen);  
    serial.flush();
    digitalWrite(DE_RE_PIN, LOW);
    delay(TIME_OUT);
}

int Mercury236::readResponce(uChar *responceBuff) {
    int bytesRead = 0;
    unsigned long startTime = millis();

    while (bytesRead < BUFFER) {
        if (serial.available() > 0) {
            responceBuff[bytesRead++] = serial.read();
            startTime = millis();
        } else if (millis() - startTime >= CH_TIME_OUT) {
            break;
        }
    }

    debugPrint("Received: ");
    debugPrint(responceBuff, bytesRead);

    return bytesRead;
}

//===================================================================ModRTU_CRC================================================================
uint16_t Mercury236::ModRTU_CRC(const uChar *buf, int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];         

    for (int i = 8; i != 0; i--) {   
      if ((crc & 0x0001) != 0) {    
        crc >>= 1;             
        crc ^= 0xA001;
      }
      else             
        crc >>= 1;      
    }
  }

  return crc; 
}

int Mercury236::checkCRC(uChar *buf, int len) {
	uint16_t receivedCRC = (buf[len - 1] << 8) | buf[len - 2];
    uint16_t calculatedCRC = ModRTU_CRC(buf, len - 2);

	return (receivedCRC == calculatedCRC) ? OK : WRONG_CRC;
}

//===================================================================Дебаг========================================================================
void Mercury236::debugPrint(const String &message) {
    if (DEBUG) {
        debugSerial->println(message);
    }
}
void Mercury236::debugPrint(const uChar *data, int length) {
    if (DEBUG) {
        for (int i = 0; i < length; ++i) {
            debugSerial->print(data[i], HEX);
            debugSerial->print(" ");
        }
        debugSerial->println();
    }
}

