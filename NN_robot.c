#include <Servo.h>  // Librería para controlar servomotor
Servo myservo;      // Objeto servo para mover el sensor

// Pines del sensor ultrasónico y motores 
const int Echo  = A4;   // Pin de eco del HC-SR04
const int Trig  = A5;   // Pin de trigger del HC-SR04
#define ENA  5           // Habilita motor A
#define ENB  6           // Habilita motor B
#define IN1  7           // Motor A dirección 1
#define IN2  8           // Motor A dirección 2
#define IN3  9           // Motor B dirección 1
#define IN4 11           // Motor B dirección 2

/******************************************
 * Configuración de la red neuronal
 ******************************************/
const int InputNodes   = 3;   // bias + 2 entradas: distancia y ángulo
const int HiddenNodes  = 4;
const int OutputNodes  = 4;   // señales para IN1–IN4

double Hidden[HiddenNodes];
double Output[OutputNodes];

float HiddenWeights[3][4] = {
  {1.8991509504, -0.4769472541, -0.6483690221, -0.3860916525},
  {-0.2818610915, 4.0106956995, 3.2291858058, -2.8943011047},
  {0.3340650865, -1.4016114422, 1.3580053903, -0.9814159763}
};
float OutputWeights[4][4] = {
  { 1.1360722975,  1.5460239494,   1.6194612260,  1.8819066697},
  {-1.5469665068, 1.3951930739,   0.1939382609,  0.3099250414},
  {-0.7755982418, 0.9390808626,   2.0862510746,  -1.1229484266},
  {-1.2357090352, 0.8583930286,   0.7247020799,   0.9762852710}
};

const int ANGULO_CENTER = 125;   // mirar al frente (ajustado según calibración)
const int ANGULO_LEFT   = 215;   // girar sensor a la izquierda (125 + 90)
const int ANGULO_RIGHT  = 35;    // girar sensor a la derecha (125 - 90)
const int ANGULO_SCAN_STEP = 45; // paso intermedio al regresar al centro
const int SCAN_THRESHOLD = 20;   // umbral de detección (en cm)

const int DELAY_READ = 50;     // ms entre lecturas del sensor
const int SPEED      = 100;    // velocidad PWM (0-255)

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
}

void loop() {
  // Escaneo frontal con ±10° para ampliar rango
  int dCenter = averageDistance(ANGULO_CENTER);
  int dLeft10 = averageDistance(ANGULO_CENTER + 10);
  int dRight10 = averageDistance(ANGULO_CENTER - 10);

  if (dCenter > SCAN_THRESHOLD && dLeft10 > SCAN_THRESHOLD && dRight10 > SCAN_THRESHOLD) {
    driveWithNN(dCenter, ANGULO_CENTER);
  } else {
    stop();
    delay(100);

    // Escaneo lateral izquierdo completo en T
    int bestLeft = stableDistanceAt(ANGULO_LEFT);
    for (int a = ANGULO_LEFT - ANGULO_SCAN_STEP; a > ANGULO_CENTER; a -= ANGULO_SCAN_STEP) {
      int d = stableDistanceAt(a);
      if (d > bestLeft) bestLeft = d;
    }

    // Escaneo lateral derecho completo en T
    int bestRight = stableDistanceAt(ANGULO_RIGHT);
    for (int a = ANGULO_RIGHT + ANGULO_SCAN_STEP; a < ANGULO_CENTER; a += ANGULO_SCAN_STEP) {
      int d = stableDistanceAt(a);
      if (d > bestRight) bestRight = d;
    }

    if (bestLeft > SCAN_THRESHOLD && bestRight > SCAN_THRESHOLD) {
      // Ambos lados libres, elegir aleatorio
      if (random(2) == 0) giroIzquierda();
      else giroDerecha();
    } else if (bestLeft > SCAN_THRESHOLD) {
      giroIzquierda();
    } else if (bestRight > SCAN_THRESHOLD) {
      giroDerecha();
    } else {
      retroceder();
      delay(500); // retroceso más largo
      turnAround();
    }
  }
  delay(100);
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void retroceder() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(500); // Retroceso antes de giro
  stop();
}

void giroIzquierda() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(600);
  stop();
}

void giroDerecha() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(600);
  stop();
}

void turnAround() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(1250);
  stop();
}

int averageDistance(int angle) {
  myservo.write(angle);
  delay(200);
  int d1 = measureDistance();
  delay(DELAY_READ);
  int d2 = measureDistance();
  return (d1 + d2) / 2;
}

int stableDistanceAt(int angle) {
  return averageDistance(angle);
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

void driveWithNN(int dist_cm, int angle_deg) {
  double entrada1 = ((-2.0 / SCAN_THRESHOLD) * dist_cm) + 1.0;
  double norm_ang = map(angle_deg, ANGULO_LEFT, ANGULO_RIGHT, -100, 100);
  norm_ang = constrain(norm_ang, -100.0, 100.0) / 100.0;

  double In1 = 1.0;
  double In2 = entrada1;
  double In3 = norm_ang;

  InputToOutput(In1, In2, In3);

  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void InputToOutput(double In1, double In2, double In3) {
  double Accum;
  for(int i = 0; i < HiddenNodes; i++) {
    Accum = In1 * HiddenWeights[0][i] + In2 * HiddenWeights[1][i] + In3 * HiddenWeights[2][i];
    Hidden[i] = tanh(Accum);
  }
  for(int i = 0; i < OutputNodes; i++) {
    Accum = 0;
    for(int j = 0; j < HiddenNodes; j++) {
      Accum += Hidden[j] * OutputWeights[j][i];
    }
    Output[i] = tanh(Accum);
  }
}

