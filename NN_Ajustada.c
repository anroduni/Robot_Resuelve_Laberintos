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

// Pesos preentrenados (fijos)
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

const int ANGULO_CENTER = 125;           // Ángulo central del servo (ajustado)
const int ANGULO_LEFT   = ANGULO_CENTER + 90;  // Lado izquierdo
const int ANGULO_RIGHT  = ANGULO_CENTER - 90;  // Lado derecho
const int ANGULO_SCAN_STEP = 45;         // Paso intermedio para escaneo lateral en T
const int SCAN_THRESHOLD = 20;            // Distancia mínima para considerar libre (cm)

const int DELAY_READ = 50;    // Delay entre lecturas sensor
const int SPEED      = 100;   // Velocidad PWM motores (0-255)

void setup() {
  myservo.attach(3);
  myservo.write(ANGULO_CENTER);  // Servo al frente al iniciar
  delay(500);                    // Espera estabilización servo

  Serial.begin(9600);

  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stop();  // Asegurar motores apagados al iniciar
  randomSeed(analogRead(0)); // Inicializar aleatorio para decisiones
}

void loop() {
  // Escaneo frontal ampliado ±10° para detectar obstáculos frontales
  int dCenter = averageDistance(ANGULO_CENTER);
  int dLeft10 = averageDistance(ANGULO_CENTER + 10);
  int dRight10 = averageDistance(ANGULO_CENTER - 10);

  if (dCenter > SCAN_THRESHOLD && dLeft10 > SCAN_THRESHOLD && dRight10 > SCAN_THRESHOLD) {
    // Si todo despejado, mover con NN hacia adelante
    driveWithNN(dCenter, ANGULO_CENTER);
  } else {
    stop();
    delay(100);

    // Escaneo lateral en T a la izquierda
    int bestLeft = stableDistanceAt(ANGULO_LEFT);
    for (int a = ANGULO_LEFT - ANGULO_SCAN_STEP; a > ANGULO_CENTER; a -= ANGULO_SCAN_STEP) {
      int d = stableDistanceAt(a);
      if (d > bestLeft) bestLeft = d;
    }

    // Escaneo lateral en T a la derecha
    int bestRight = stableDistanceAt(ANGULO_RIGHT);
    for (int a = ANGULO_RIGHT + ANGULO_SCAN_STEP; a < ANGULO_CENTER; a += ANGULO_SCAN_STEP) {
      int d = stableDistanceAt(a);
      if (d > bestRight) bestRight = d;
    }

    // Decisión basada en escaneo lateral
    if (bestLeft > SCAN_THRESHOLD && bestRight > SCAN_THRESHOLD) {
      // Ambos lados libres: girar izquierda o derecha aleatoriamente
      if (random(2) == 0) giroIzquierda();
      else giroDerecha();
    } else if (bestLeft > SCAN_THRESHOLD) {
      giroIzquierda();
    } else if (bestRight > SCAN_THRESHOLD) {
      giroDerecha();
    } else {
      // Ambos lados bloqueados: retroceder y girar 180°
      retroceder();
      delay(700); // Retroceso un poco más largo
      turnAround();
    }
  }
  delay(100);  // Tiempo para maniobras
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

// Movimiento con red neuronal (avance y maniobras laterales suaves)
void driveWithNN(int dist_cm, int angle_deg) {
  // Normalización de entrada distancia [-1,1]
  double entrada1 = ((-2.0 / SCAN_THRESHOLD) * dist_cm) + 1.0;
  // Normalización ángulo [-1,1]
  double norm_ang = map(angle_deg, ANGULO_LEFT, ANGULO_RIGHT, -100, 100);
  norm_ang = constrain(norm_ang, -100.0, 100.0) / 100.0;

  double In1 = 1.0;  // bias
  double In2 = entrada1;
  double In3 = norm_ang;

  InputToOutput(In1, In2, In3);

  // PWM para velocidad constante
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);

  // Salidas motor según NN, ajustadas para que ambos motores avancen
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Retroceso con ambos motores hacia atrás
void retroceder() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(700); // Tiempo de retroceso
  stop();
}

// Giro a la izquierda (90° aprox)
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

// Giro a la derecha (90° aprox)
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

// Giro 180° (media vuelta)
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

// Obtiene distancia promedio para reducir ruido
int averageDistance(int angle) {
  myservo.write(angle);
  delay(200); // Tiempo para mover servo y estabilizar
  int d1 = measureDistance();
  delay(DELAY_READ);
  int d2 = measureDistance();
  return (d1 + d2) / 2;
}

// Función auxiliar (puede simplificar llamadas)
int stableDistanceAt(int angle) {
  return averageDistance(angle);
}

// Medición directa del sensor ultrasónico
int measureDistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  long dur = pulseIn(Echo, HIGH);
  return dur / 58;
}

// Propagación hacia adelante en la red neuronal
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