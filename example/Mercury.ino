#include <Mercury236.h>

#define RS485_RX_PIN 16
#define RS485_TX_PIN 17
#define DE_RE_PIN 2  // Управление направлением передачи/приема для RS485

#define START_TIME(id) unsigned long start_time_##id = micros();
#define END_TIME(id, description) \
  Serial.print(description); \
  Serial.print(" выполнено за: "); \
  Serial.print(micros() - start_time_##id); \
  Serial.println(" микросекунд.");

Mercury236 mercury236(Serial2, DE_RE_PIN);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println("======Настройка соединения");
  //mercury236.setDebugSerial(Serial);  // Включение отладки
  mercury236.setPMAddress(0x7F);
  mercury236.setChTimeOut(10);
  mercury236.setTimeOut(50);
}

String stringError(int code) {
  switch (code) {
    case OK:
      return "OK";
    case WRONG_CRC:
      return "WRONG_CRC";
    case COMMUNICATION_ERROR:
      return "COMMUNICATION_ERROR";
    default:
      return "UNKNOWN_ERROR";
  }
}

void loop() {
  START_TIME(1);
  int connectionResult = mercury236.checkChannel();
  END_TIME(1, "Проверка линии");
  Serial.println("======Проверка линии " + stringError(connectionResult));

  START_TIME(2);
  int connectionResult2 = mercury236.initConnection();
  END_TIME(2, "Открываем соединение");
  Serial.println("======Открываем соединение " + stringError(connectionResult2));

  START_TIME(3);
  float dataU[3];
  int connectionResult3 = mercury236.getU(dataU);
  if (connectionResult3 == OK) {
    Serial.println();
    Serial.print("U.p1: ");
    Serial.println(dataU[0]);
    Serial.print("U.p2: ");
    Serial.println(dataU[1]);
    Serial.print("U.p3: ");
    Serial.println(dataU[2]);
  }
  END_TIME(3, "Получаем напряжение");
  Serial.println("======Получаем напряжение " + stringError(connectionResult3));

  START_TIME(4);
  float dataI[4];
  int connectionResult4 = mercury236.getI(dataI);
  if (connectionResult4 == OK) {
    Serial.println();
    Serial.print("I.p1: ");
    Serial.println(dataI[0]);
    Serial.print("I.p2: ");
    Serial.println(dataI[1]);
    Serial.print("I.p3: ");
    Serial.println(dataI[2]);
  }
  END_TIME(4, "Получаем напряжение");
  Serial.println("======Получаем напряжение " + stringError(connectionResult4));


  START_TIME(5);
  float dataCosF[5];
  int connectionResult5 = mercury236.getCosF(dataCosF);
  if (connectionResult5 == OK) {
    Serial.println();
    Serial.print("CosF.p1: ");
    Serial.println(dataCosF[0]);
    Serial.print("CosF.p2: ");
    Serial.println(dataCosF[1]);
    Serial.print("CosF.p3: ");
    Serial.println(dataCosF[2]);
    Serial.print("CosF.sum: ");
    Serial.println(dataCosF[3]);
  }
  END_TIME(5, "Получаем CosF");
  Serial.println("======Получаем CosF " + stringError(connectionResult5));

  int connectionResult6 = mercury236.closeConnection();
  Serial.println("======Закрываем соединение " + stringError(connectionResult6));


  delay(15000);
}
