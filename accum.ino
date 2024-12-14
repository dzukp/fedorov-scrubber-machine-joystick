// Закоментарить, чтобы отключить печать в ком-порт
#define ENABLE_PRINT


const int numReadings = 10;

int readings[numReadings];  // показания с аналогового входа
int readIndex = 0;          // индекс текущего значения
int total = 0;              // текущее общее количество
int average = 0;            // среднее значение

float Vcc = 5.01;           // напряжение питания ардуино
float Voltagelcd;           // вывод значения с запятой

byte aiAccum = A0;       // аналоговый входной порт
byte doRed = 8;          // красный светодиод
byte doYellow = 9;       // желтый светодиод
byte doGreen = 10;       // зеленый светодиод
byte doAccOk = 13;
byte doPump1 = 6;
byte doPump2 = 7;


class Button {
  public:
    Button (byte pin) {
      _pin = pin;
      pinMode(_pin, INPUT_PULLUP);
    }
    bool clicked() {
      bool btnState = digitalRead(_pin);
      if (!btnState && !_flag && millis() - _tmr >= 100) {
        _flag = true;
        _tmr = millis();
        return true;
      }
      if (!btnState && _flag && millis() - _tmr >= 500) {
        _tmr = millis ();
        return true;
      }
      if (btnState && _flag) {
        _flag = false;
        _tmr = millis();
      }
      return false;
    }
  private:
    byte _pin;
    uint32_t _tmr;
    bool _flag;
};


Button incrBtn(11);  // Кнопка увеличения 
Button decrBtn(12);  // Кнопка уменьшения 
bool incrBtnPressed = false;
bool decrBtnPressed = false;
int pumpSpeed = 0;
byte pumpLeds[] = {2, 3, 4, 5};

void setup() {
  pinMode(aiAccum, INPUT);
  
  pinMode(doRed, OUTPUT);
  pinMode(doYellow, OUTPUT);
  pinMode(doGreen, OUTPUT);
  pinMode(doAccOk, OUTPUT);
  pinMode(doPump1, OUTPUT);
  pinMode(doPump2, OUTPUT);
  
  Serial.begin(115200);

  // инициализируйте все показания равным 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  for (int i = 0; i < sizeof(pumpLeds) / sizeof(pumpLeds[0]); i++) {
    pinMode(pumpLeds[i], OUTPUT);
  }
}


void loop() {
  
  accumProcess();
  pumpProcess();

  delay(10);
}


void accumProcess() {

  // вычтите последнее значение:
  total = total - readings[readIndex];
  // считывание с датчика:
  readings[readIndex] = analogRead(aiAccum);
  // добавьте полученные данные к общему количеству:
  total = total + readings[readIndex];
  // перейдите к следующей позиции в массиве:
  readIndex = readIndex + 1;

  // если мы находимся в конце массива...
  if (readIndex >= numReadings) {
    // ...вернуться к началу:
    readIndex = 0;
  }

  // рассчет среднего значения:
  average = total / numReadings;
  // отправьте его на компьютер в виде цифр ASCII
  print("", average);
      
  // Проверка уровня входного напряжения и включение светодиодов при напряжении 30В 4,4v  0,7mA  36k & 6k2
 
  if (average >= 720) // *******************
  {
    digitalWrite(doRed, LOW);
    digitalWrite(doYellow, LOW);
    digitalWrite(doGreen, HIGH);
    digitalWrite(doAccOk, HIGH);
  }
  else if (average < 720 && average > 650) // ************************
  {
    digitalWrite(doRed, LOW);
    digitalWrite(doYellow, HIGH);
    digitalWrite(doGreen, LOW);
    digitalWrite(doAccOk, HIGH);
  }
  else if (average <= 650) // ***********************
  {
    digitalWrite(doRed, HIGH);
    digitalWrite(doYellow, LOW);
    digitalWrite(doGreen, LOW);
    digitalWrite(doAccOk, LOW);
  }
}


void pumpProcess() {

  if (!incrBtnPressed && incrBtn.clicked() && pumpSpeed < 3) {
    pumpSpeed += 1;
  }

  if (!decrBtnPressed && decrBtn.clicked() && pumpSpeed > 0) {
    pumpSpeed -= 1;
  }

  print("pump speed", pumpSpeed);

  for (int i = 0; i < sizeof(pumpLeds) / sizeof(pumpLeds[0]); i++) {
    digitalWrite(pumpLeds[i], (i == pumpSpeed) ? HIGH : LOW);
  }

  switch (pumpSpeed) {
    case 0:
      digitalWrite(doPump1, LOW);
      digitalWrite(doPump2, LOW);
      break;
    case 1:
      digitalWrite(doPump1, HIGH);
      digitalWrite(doPump2, LOW);
      break;
    case 2:
      digitalWrite(doPump1, LOW);
      digitalWrite(doPump2, HIGH);
      break;
    case 3:
      digitalWrite(doPump1, HIGH);
      digitalWrite(doPump2, HIGH);
      break;
    default:
      digitalWrite(doPump1, LOW);
      digitalWrite(doPump2, LOW);
      print("anomal pump speed");
      break;
  }
}


void print(const char* title, double value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %.2f", title, value);
  Serial.println(output);
#endif
}

void print(const char* title, int value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %d", title, value);
  Serial.println(output);
#endif
}

void print(const char* title, bool value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %d", title, value ? 1 : 0);
  Serial.println(output);
#endif
}

void print(const char* text) {
#ifdef ENABLE_PRINT
  char output[64];
  snprintf(output, sizeof(output), "%s", text);
  Serial.println(output);
#endif
}
