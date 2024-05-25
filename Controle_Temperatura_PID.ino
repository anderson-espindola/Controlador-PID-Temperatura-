
// Bibliotecas
#include <LiquidCrystal.h>
#include <PID_v2.h>

// Constantes PWM e PID
#define MIN_PWM 30
#define MAX_PWM 320
#define KP 0.4       // constante proporcional - ela que vai ler a temperatura e calcula o erro que vai ficar atuando o fan para corrigir a temperatura.
#define KI 0.4      //constante intelgral
#define KD 0.05    //constante derivativa

// Declaração de Pinos
int PinFanRPM = A1;     // Fan RPM
int PinFanPWM = 10;     // Fan PWM
int PinTemp = A0;       // Sensor Temperatura
int PinRelay = 13;      // Rele

// Pinos Display LCD no Arduino
//  Ard.- LCD
//  12  - RS
//  11  - E
//  5   - DB4
//  4   - DB5
//  3   - DB6
//  2   - DB7
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);    // Display LCD
// Outros Pinos:
//    VDD - 5V
//    VSS - GND
//    R_W - GND
//    VO - Pino central de um Trimpot, outros pinos do Trimpot no 5V e GND.

// Declaração de Variáveis
unsigned long pulseDuration;
double currentTemp;
double maxTemp, minTemp;
double currentPWM;
double frequency;
double setpoint = 30;  // Setpoint de Temperatura desejada para o sistema
int i;
int currentRPM;
double samples[10];

// Inicialização da variável PID
PID fanPID(&currentTemp, &currentPWM, &setpoint, KP, KI, KD, REVERSE);

/*
 * SETUP
 */
void setup()
{
  // Configuração Timer Arduino para 25KHz do PWM
  TCCR1A = 0;               // undo the configuration done by...
  TCCR1B = 0;               // ...the Arduino core library
  TCNT1  = 0;               // reset timer
  TCCR1A = _BV(COM1A1)      // non-inverted PWM on ch. A
           | _BV(COM1B1)    // same on ch; B
           | _BV(WGM11);    // mode 10: ph. correct PWM, TOP = ICR1
  TCCR1B = _BV(WGM13)       // ditto
           | _BV(CS10);     // prescaler = 1
  ICR1   = 320;             // TOP = 320

  // Configura Pinos
  pinMode(PinFanPWM, OUTPUT);
  pinMode(PinFanRPM, INPUT);
  pinMode(PinRelay, OUTPUT);
  digitalWrite(PinFanRPM, HIGH);

  // Inicia LCD
  lcd.begin(16, 2);

  // Configura PID
  fanPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  fanPID.SetMode(AUTOMATIC);

}


/*
 * LOOP
 */
void loop()
{
  // Faz Leituras de Temperatura e RPM do Fan
  readTemp();
  readPulse();

  // Faz Escrita do Display com informações
  writeLCD();

  // Se Temperatura for maior que 30C, aciona Rele, calcula PID e aciona PWM
  if (currentTemp > 30.0) {
    digitalWrite(PinRelay, LOW);
    fanPID.Compute();
    analogWrite25k(currentPWM);
  } else {                          // Senão, desliga Rele e fica numa histerese via loop while até a Temperatura ultrapassar 35C
    digitalWrite(PinRelay, HIGH);
    writeLCD_FanOFF();
    while (currentTemp < 30) {
      readTemp();
      writeLCD_FanOFF();
      delay(20000);                 // Delay de 20seg
    }
  }
}

/*
***** FUNÇÕES ******
*/

// Função para fazer escrita do PWM no pino de saída através do registro
void analogWrite25k(int value)
{
  OCR1B = value;
}

// Função para fazer leitura do Sensor de Temperatura
void readTemp() {
  // Faz amostragem de 10 leituras e tira média delas
  for (i = 0; i <= 9; i++) {
    samples[i] = analogRead(PinTemp);
    currentTemp = currentTemp + samples[i];
    delay(50);
  }
  currentTemp = currentTemp / 10.0;
  currentTemp = (( 5.0 * currentTemp * 100.0) / 1024.0);    // Converte leitura de Counts para Celcius
}

// Função para fazer leitura do PWM real do Fan
void readPulse() {
  pulseDuration = pulseIn(PinFanRPM, HIGH);  // changed from LOW to HIGH
  frequency = 1000000 / pulseDuration;
  currentRPM = frequency / 2 * 60;
}

// Função para escrever no display dados de Temperatura e PWM
void writeLCD() {
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(currentTemp, 1);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("%PWM:");
  lcd.print((currentPWM / 320) * 100, 0);
  lcd.print(" ");
}

// Função para escrever no display dados de Temperatura e PWM OFF
void writeLCD_FanOFF() {
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(currentTemp, 1);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("FAN OFF ");
}

