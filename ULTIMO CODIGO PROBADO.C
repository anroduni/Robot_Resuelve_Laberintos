#include <Servo.h>
Servo myservo;

// Pines para sensor ultrasónico y control de motores
const int Echo  = A4;
const int Trig  = A5;
#define ENA  5
#define ENB  6
#define IN1  7
#define IN2  8
#define IN3  9
#define IN4 11

// Parámetros de la red neuronal
const int InputNodes   = 3;
const int HiddenNodes  = 4;
const int OutputNodes  = 5;

enum Movimiento { ADELANTE = 0, IZQUIERDA, DERECHA, RETROCEDER, GIRO_COMPLETO };

// Ángulos servo
const int ANGULO_CENTER = 125;
const int ANGULO_LEFT   = 215;
const int ANGULO_RIGHT  = 35;
const int ANGULO_SCAN_STEP = 15;

// Parámetros de movimiento y sensor
const int SCAN_THRESHOLD = 20;
const int DELAY_READ = 40;
const int SPEED_LEFT = 95;
const int SPEED_RIGHT = 102;

// Historial
Movimiento historial[5];
int indexHist = 0;

// Pesos de red neuronal
float HiddenWeights[3][4] = {
  {1.8991509504, -0.4769472541, -0.6483690221, -0.3860916525},
  {-0.2818610915, 4.0106956995, 3.2291858058, -2.8943011047},
  {0.3340650865, -1.4016114422, 1.3580053903, -0.9814159763}
};
float OutputWeights[4][5] = {
  { 1.1360722975,  1.5460239494, 1.6194612260,  1.8819066697,  0.1},
  {-1.5469665068, 1.3951930739, 0.1939382609,  0.3099250414,  0.2},
  {-0.7755982418, 0.9390808626, 2.0862510746, -1.1229484266,  0.2},
  {-1.2357090352, 0.8583930286, 0.7247020799,  0.9762852710,  0.1}
};

double Hidden[HiddenNodes];
double Output[OutputNodes];

void setup() {
  myservo.attach(3);
  myservo.write(ANGULO_CENTER);
  delay(500);

  Serial.begin(9600);
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
  randomSeed(analogRead(0));

  for (int i = 0; i < 5; i++) historial[i] = ADELANTE;
}

void loop() {
  if (obstaculoAdelante()) {
    stop();
    delay(100);

    int bestLeft = 0, bestRight = 0;
    for (int a = ANGULO_CENTER + 15; a <= ANGULO_LEFT; a += ANGULO_SCAN_STEP)
      bestLeft = max(bestLeft, averageDistance(a));

    for (int a = ANGULO_CENTER - 15; a >= ANGULO_RIGHT; a -= ANGULO_SCAN_STEP)
      bestRight = max(bestRight, averageDistance(a));

    double maxDist = max(bestLeft, bestRight);
    double entrada1 = constrain(1.0 - (maxDist / 50.0), 0.0, 1.0);
    double norm_ang = (bestLeft > bestRight) ? 0.8 : -0.8;

    InputToOutput(1.0, entrada1, norm_ang);
    int decision = indiceMayorEvitarRedundancia();
    ejecutarMovimiento((Movimiento)decision);
  } else {
    ejecutarMovimiento(ADELANTE);
  }

  delay(10);
}

void ejecutarMovimiento(Movimiento mov) {
  guardarMovimiento(mov);
  switch (mov) {
    case ADELANTE:
      analogWrite(ENA, SPEED_RIGHT);
      analogWrite(ENB, SPEED_LEFT);
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      break;

    case IZQUIERDA:
      analogWrite(ENA, SPEED_RIGHT);
      analogWrite(ENB, SPEED_LEFT);
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      delay(600); stop(); break;

    case DERECHA:
      analogWrite(ENA, SPEED_RIGHT);
      analogWrite(ENB, SPEED_LEFT);
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      delay(600); stop(); break;

    case RETROCEDER:
      analogWrite(ENA, SPEED_RIGHT);
      analogWrite(ENB, SPEED_LEFT);
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      delay(700); stop(); break;

    case GIRO_COMPLETO:
      analogWrite(ENA, SPEED_RIGHT);
      analogWrite(ENB, SPEED_LEFT);
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      delay(1250); stop(); break;
  }
}

void guardarMovimiento(Movimiento mov) {
  historial[indexHist] = mov;
  indexHist = (indexHist + 1) % 5;
}

int indiceMayorEvitarRedundancia() {
  int mayor = 0;
  double maxVal = -1.0;
  for (int i = 0; i < OutputNodes; i++) {
    if (Output[i] > maxVal && !esRepetido((Movimiento)i)) {
      maxVal = Output[i];
      mayor = i;
    }
  }
  return mayor;
}

bool esRepetido(Movimiento mov) {
  for (int i = 0; i < 5; i++) {
    if (historial[i] == mov) return true;
  }
  return false;
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

bool obstaculoAdelante() {
  for (int offset = -22; offset <= 22; offset += ANGULO_SCAN_STEP) {
    if (averageDistance(ANGULO_CENTER + offset) < SCAN_THRESHOLD)
      return true;
  }
  return false;
}

int averageDistance(int angle) {
  myservo.write(angle);
  delay(60);
  return measureDistanceReliable();
}

int measureDistanceReliable() {
  int minVal = 9999;
  for (int i = 0; i < 4; i++) {
    int d = measureDistance();
    if (d > 0 && d < minVal) minVal = d;
    delay(DELAY_READ);
  }
  return minVal;
}

int measureDistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  long dur = pulseIn(Echo, HIGH);
  return dur / 58;
}

void InputToOutput(double In1, double In2, double In3) {
  double Accum;
  for (int i = 0; i < HiddenNodes; i++) {
    Accum = In1 * HiddenWeights[0][i] + In2 * HiddenWeights[1][i] + In3 * HiddenWeights[2][i];
    Hidden[i] = tanh(Accum);
  }
  for (int i = 0; i < OutputNodes; i++) {
    Accum = 0;
    for (int j = 0; j < HiddenNodes; j++)
      Accum += Hidden[j] * OutputWeights[j][i];
    Output[i] = tanh(Accum);
  }
}

// PWM (Pulse‑Width Modulation – Modulación por Ancho de Pulso)
// NN (Neural Network – Red Neuronal)
// ENA/ENB Son los pines que habilitan (Enable) el controlador de motores L298.