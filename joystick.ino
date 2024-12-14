#include <Arduino.h>
#include <Smooth.h>

#define LOOP_PERIOD 100  //  период выполнения основного цикла мс
#define SMOOTHED_SAMPLE_SIZE 1
#define SMOOTHED_PWD_SAMPLE_PERIOD 100  // период за который измеряется среднее значение ШИМ 
#define PULSE_LEN 1000  // длина импульса
#define PULSE_PERIOD 5000  // период импульсов

#define ENABLE_PRINT  // Закоментарить, чтобы отключить печать в ком-порт

// Пороги работы джойстика
const int frwEdge = 600;
const int bcwEdge = 130;

void print(char* title, double value);
void print(char* title, int value);
void print(char* title, bool value);
void print(char* text);

byte diJoystickSwitch = 2;
byte aiJoystick = A3;
byte aiDr1 = A0;
byte aiDr2 = A1;
byte diAccum = 13;
byte diBrush = 6;
byte diAir = 11;
byte diValve = 12;

byte doForward = 9;
byte doEngine = 10;
byte doForwardPulse = 7;
byte doBackwardPulse = 8;
byte doBrush = 3;
byte doAir = 4;
byte doValve = 5;

Smooth joystick(SMOOTHED_SAMPLE_SIZE);
Smooth dr1(SMOOTHED_PWD_SAMPLE_PERIOD / LOOP_PERIOD);
Smooth dr2(SMOOTHED_PWD_SAMPLE_PERIOD / LOOP_PERIOD);


class Pulse {
  /*
  Класс для генерации импульса
  */
  public:
    Pulse() : _state(0) {}

    void start(int time=PULSE_LEN, int period=PULSE_PERIOD) {
      /*
      ф-ция генерирует импульсы длиной time и периодом period
      */
      switch (_state) {
        case 0:
          _end_time = millis() + time;
          _state = 1;
          break;

        case 1:
          if (millis() > _end_time) {
            _end_time = millis() + period - time;
            _state = 2;
          }
          break;

        case 2:
          if (millis() > _end_time) {
            _state = 0;
          }
          break;

        case -1:
          break;

        default:
          _state = 0;
      }
    }

    void stop() {
      /* ф-ция останавливает генерацию импульсов */
      _state = 0;
    }

    void restart() {
      stop();
      start();
    }

    bool state() { 
      /* Возвращает состояние, true - верхний уровень, false - нижний уровень  */
      return _state == 1; 
    }

  private:
    uint32_t _end_time;
    int _state;
};


class Machine {
  /*
  Класс управления машиной,
  принимает команду
  forward() - вперед
  stop() - стоять
  backward() - назад
  Оценивает состояния DR и управляет драйвером и реле
  */
  public:
    Machine() : _command(0) {}

    void forward() {
      if (_command != 1) {
        _command = 1;
      }
    }

    void backward() {
      if (_command != -1) {
        _command = -1;
      }
    }

    void stop() {
      if (_command != 0) {
        _command = 0;
      }
    }

    void process() {

      print("DR1 is active", _is_dr1_active());
      print("DR2 is active", _is_dr2_active());

      /*
      Выбираем действие исходя из команды
      1 - вперёд
      -1 - назад
      0 - стоп
      */
      switch (_command) {
        
        case 1: _forwardProcess(); break; // вперёд
        case -1: _backwardProcess(); break; // назад
        case 0: _stopProcess(); break; // стоп
        default: print("machine bad command", _command); break;
      }
      // Если драйвер движения вперёд включен, 
      // то хотим скомутировать реле для движения вперёд
      do_frw = _is_dr1_active();      
      // Если драйвер движения назад включен и не хотим скомутировать реле для движения вперёд, 
      // то хотим скомутировать реле для движения назад
      do_bcw = _is_dr2_active() && !do_frw;
    }

  private:
    void _forwardProcess() {
      // Была команда ехать вперёд
      if (!_is_dr2_active() && !_is_dr1_active()) {
        // оба драйвера выключены - шлем импульс вперёд
        k1_frw.start();
        k2_bcw.stop();
        print("frw 1 0");
      }
      else if (_is_dr2_active() && !_is_dr1_active()) {
        // драйвер назад включен, вперёд выключен - шлём импульсы назад и вперёд
        // чтобы выключить драйвер движения назад и включить драйвер вперёд
        k1_frw.start();
        k2_bcw.start();
        print("frw 1 1");
      }
      else if (!_is_dr2_active() && _is_dr1_active()) {
        // драйвер назад выключен, вперёд включен - всё как надо, импульсы не шлём
        k1_frw.stop();
        k2_bcw.stop();
        print("frw 0 0");
      }
      else if(_is_dr2_active() && _is_dr1_active()) {
        // оба драйвера включены - шлём импульс назад, 
        // чтобы выключить драйвер движения назад
        k1_frw.stop();
        k2_bcw.start();
        print("frw 0 1");
      }
    }

    void _backwardProcess() {
      // команда ехать назад
      if (!_is_dr2_active() && !_is_dr1_active()) {
        // оба драйвера выключены - шлем импульс назад
        k1_frw.stop();
        k2_bcw.start();
        print("bcw 0 1");
      }
      else if (_is_dr2_active() && !_is_dr1_active()) {
        // драйвер назад включен, вперёд выключен - всё как надо, импульсы не шлём
        k1_frw.stop();
        k2_bcw.stop();
        print("bcw 0 0");
      }
      else if (!_is_dr2_active() && _is_dr1_active()) {
        // драйвер назад выключен, вперёд включен - шлём импульсы назад и вперёд
        // чтобы включить драйвер движения назад и выключить драйвер вперёд
        k1_frw.start();
        k2_bcw.start();
        print("bcw 1 1");
      }
      else if(_is_dr2_active() && _is_dr1_active()) {
        // оба драйвера включены - шлем импульс вперёд,
        // чтобы выключить драйвер движения вперёд
        k1_frw.start();
        k2_bcw.stop();
        print("bcw 1 0");
      }
    }

    void _stopProcess() {
      // команда стоять
      if (!_is_dr2_active() && !_is_dr1_active()) {
        // оба драйвера выключены - всё как надо, импульсы не шлём
        k1_frw.stop();
        k2_bcw.stop();
        print("stop 0 0");
      }
      else if (_is_dr2_active() && !_is_dr1_active()) {
        // драйвер назад включен, вперёд выключен - шлём импульс назад
        k1_frw.stop();
        k2_bcw.start();
        print("stop 0 1");
      }
      else if (!_is_dr2_active() && _is_dr1_active()) {
        // драйвер вперёд включен, назад выключен - шлём импульс вперёд
        k1_frw.start();
        k2_bcw.stop();
        print("stop 1 0");
      }
      else if(_is_dr2_active() && _is_dr1_active()) {
        // оба драйвера включены - шлем импульс вперёд и назад,
        // чтобы выключить оба драйвера
        k1_frw.start();
        k2_bcw.start();
        print("stop 1 1");
      }
    }

    bool _is_dr1_active() { 
      /*
      Анализиируем состояние драйвера вперёд
      true - включен
      false - выключен
      */
      return dr1.get_avg() < 1; 
    }
    bool _is_dr2_active() { 
      /*
      Анализиируем состояние драйвера назад
      true - включен
      false - выключен
      */
      return dr2.get_avg() < 1; 
    }

    int _command;
    bool _wrong_state;

  public:
    Pulse k1_frw;
    Pulse k2_bcw;

    bool do_frw;
    bool do_bcw;
};


Machine machine;


void setup() {
  pinMode(aiJoystick, INPUT);
  pinMode(diJoystickSwitch, INPUT);
  pinMode(aiDr1, INPUT);
  pinMode(aiDr2, INPUT);
  pinMode(diAccum, INPUT);
  pinMode(diBrush, INPUT);
  pinMode(diAir, INPUT);
  pinMode(diValve, INPUT);

  pinMode(doForward, OUTPUT);
  pinMode(doEngine, OUTPUT);
  pinMode(doForwardPulse, OUTPUT);
  pinMode(doBackwardPulse, OUTPUT);
  pinMode(doBrush, OUTPUT);
  pinMode(doAir, OUTPUT);
  pinMode(doValve, OUTPUT);
  
  Serial.begin(9600);
}


void loop() {

  // запоминаем текущее время, чтобы измерить время цикла
  uint32_t start_loop_time = millis();
  
  joystick.add(analogRead(aiJoystick));
  dr1.add(digitalRead(aiDr1) ? 100 : 0);
  dr2.add(digitalRead(aiDr2) ? 100 : 0);

  bool accum = digitalRead(diAccum);
  bool joystickSwitch = digitalRead(diJoystickSwitch);
  bool brush = digitalRead(diBrush);
  bool air = digitalRead(diAir);
  bool valve = digitalRead(diValve);

  print("-----");
  print("Joyst", joystick.get_avg());
  print("Switch", joystickSwitch);
  print("DR1", dr1.get_avg());
  print("DR2", dr2.get_avg());

  int cmdMove = 0;
  if (joystick.get_avg() > frwEdge && joystickSwitch && accum) {
    // Значение джойстика выше порога, свитч сработал, аккумулятор в норме - команда вперёд
    cmdMove = 1;
  }
  else if (joystick.get_avg() < bcwEdge && joystickSwitch && accum) {
    // Значение джойстика ниже порога, свитч сработал, аккумулятор в норме - команда назад
    cmdMove = -1;
  }

  if (accum) {
    // Аккумулятор в норме
    digitalWrite(doBrush, brush ? HIGH : LOW);
    digitalWrite(doAir, air ? HIGH : LOW);
    digitalWrite(doValve, valve ? HIGH : LOW);

    // Логика работы machine описана в классе Machine
    if (cmdMove == 1) {
      // команада вперёд для объекта machine
      machine.forward();
    }
    else if (cmdMove == -1) {
      // команада назад для объекта machine
      machine.backward();
    }
    else {
      // команада стоп для объекта machine
      machine.stop();
    }

    print("machive cmd", cmdMove);

    // Вызываем главную ф-цию логики машины
    machine.process();

    // получаем из машины команду для драйвера вперёд и передаём её на соотв выход
    digitalWrite(doForwardPulse, machine.k1_frw.state() ? HIGH : LOW);
    // получаем из машины команду для драйвера назад и передаём её на соотв выход
    digitalWrite(doBackwardPulse, machine.k2_bcw.state() ? HIGH : LOW);

    print("DR1 pulse",  machine.k1_frw.state());
    print("DR2 pulse",  machine.k2_bcw.state());
    
    if (machine.do_frw) {
      // коммутируем реле, чтобы подключить драйвер вперёд
      digitalWrite(doForward, HIGH);
      digitalWrite(doEngine, HIGH);
      print("FRW 1 ENG 1");
    }
    else if (machine.do_bcw) {
      // коммутируем реле, чтобы подключить драйвер назад
      digitalWrite(doForward, LOW);
      digitalWrite(doEngine, HIGH);
      print("FRW 0 ENG 1");
    }
    else {
      // коммутируем реле, чтобы отключить оба драйвера
      digitalWrite(doForward, LOW);
      digitalWrite(doEngine, LOW);
      print("FRW 0 ENG 0");
    }

  }
  else {
    // Аккумулятор сел, всё останавливаем
    digitalWrite(doBrush, LOW);
    digitalWrite(doAir, LOW);
    digitalWrite(doValve, LOW);

    machine.stop();
    machine.process();

    digitalWrite(doForwardPulse, LOW);
    digitalWrite(doBackwardPulse, LOW);
    digitalWrite(doForward, LOW);
    digitalWrite(doEngine, LOW);

    print("accum is low");
  }

  // вычисляем время цикла
  uint32_t loop_time = millis() - start_loop_time;
  print("loop_time", loop_time);
  if (LOOP_PERIOD > loop_time) {
    // если время цикла короткое, то делаем паузу
    print("delay", LOOP_PERIOD - loop_time);
    delay(LOOP_PERIOD - loop_time);
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
