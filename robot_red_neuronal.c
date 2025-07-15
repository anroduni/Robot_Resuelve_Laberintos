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
 * - HiddenWeights y OutputWeights: pesos preentrenados
 ******************************************/
const int InputNodes   = 3;   // bias + 2 entradas: distancia y ángulo
const int HiddenNodes  = 4;
const int OutputNodes  = 4;   // señales para IN1–IN4

// Variables temporales para activaciones
double Hidden[HiddenNodes];
double Output[OutputNodes];

// Matrices de pesos 
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

/******************************************/
// Ángulos para barrido del servo
const int ANGULO_CENTER = 90;   // mirar al frente
const int ANGULO_LEFT   = 150;  // girar sensor a la izquierda
const int ANGULO_RIGHT  = 30;   // girar sensor a la derecha

// Umbral de detección del laberinto (en centímetros)
const int THRESHOLD_DIST = 5;  // <--- editar según prueba en laberinto

double distanciaMaxima = THRESHOLD_DIST;  // normalización de entrada de NN

const int DELAY_READ = 50;    // ms entre lecturas sucesivas para filtrar ruido
const int SPEED      = 100;   // velocidad PWM (0-255)

/************************************************************
 * setup(): se ejecuta al arrancar
 * - Inicializa puertos y posición del servo
 ************************************************************/
void setup() {
  myservo.attach(3);          // Pin 3 controla servo
  Serial.begin(9600);         // Para debug por monitor
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();                     // Asegura motores detenidos
  myservo.write(ANGULO_CENTER);
  delay(500);                 // Tiempo para estabilizar servo
}

/************************************************************
 * loop(): ciclo principal
 * 1) Mide al frente; si libre, avanza con NN
 * 2) Si bloqueado, mide a izquierda y derecha
 * 3) Si todas bloqueadas, gira 180° (turnAround)
 ************************************************************/
void loop() {
  int dF = stableDistanceAt(ANGULO_CENTER);  // distancia filtrada al frente
  if (dF > THRESHOLD_DIST) {
    driveWithNN(dF, ANGULO_CENTER);          // usa NN para moverse
  } else {
     stop();
    int dL = stableDistanceAt(ANGULO_LEFT);  // prueba izquierda
    if (dL > THRESHOLD_DIST) {
      driveWithNN(dL, ANGULO_LEFT);
    } else {
       stop();
      int dR = stableDistanceAt(ANGULO_RIGHT); // prueba derecha
      if (dR > THRESHOLD_DIST) {
        driveWithNN(dR, ANGULO_RIGHT);
      } else {
         stop();
        turnAround();  // bloqueo total: giro en el lugar
      }
    }
  }
  delay(100);  // deja tiempo para la maniobra antes de siguiente ciclo
}

/************************************************************
 * stop(): detiene los dos motores (modo reposo)
 ************************************************************/
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

/************************************************************
 * stableDistanceAt(angle):
 * - Mueve servo al ángulo indicado
 * - Lee distancia dos veces, con DELAY_READ ms entre ellas
 * - Devuelve promedio para reducir lecturas erráticas
 ************************************************************/
int stableDistanceAt(int angle) {
  myservo.write(angle);
  delay(200);                     // espera a posicionar servo
  int d1 = measureDistance();     // primera medición
  delay(DELAY_READ);
  int d2 = measureDistance();     // segunda medición
  return (d1 + d2) / 2;           // promedio
}

/************************************************************
 * measureDistance():
 * - Genera pulso en Trig
 * - Mide duración en Echo
 * - Convierte a cm (usando factor 58)
 * - Retorna valor entero
 ************************************************************/
int measureDistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  long dur = pulseIn(Echo, HIGH); // microsegundos de eco
  int dist = dur / 58;            // cm aproximados
  return dist;
}

/************************************************************
 * driveWithNN(dist_cm, angle_deg):
 * - Normaliza distancia y ángulo
 * - Ejecuta feedforward con NN
 * - Aplica señales de salida a IN1–IN4
 * - PWM fijo en ENA/ENB (SPEED)
 ************************************************************/
void driveWithNN(int dist_cm, int angle_deg) {
  // normalizar cercanía a [-1..1]
  double entrada1 = ((-2.0 / distanciaMaxima) * dist_cm) + 1.0;
  // normalizar ángulo a [-1..1]
  double norm_ang = map(angle_deg, ANGULO_LEFT, ANGULO_RIGHT, -100, 100);
  norm_ang = constrain(norm_ang, -100.0, 100.0) / 100.0;

  // Preparar entradas para NN
  double In1 = 1.0;    // bias
  double In2 = entrada1;
  double In3 = norm_ang;

  InputToOutput(In1, In2, In3);  // calcula Hidden[] y Output[]

  // Convertir activación a señal digital (0 o 1)
  int o1 = round(abs(Output[0]));
  int o2 = round(abs(Output[1]));
  int o3 = round(abs(Output[2]));
  int o4 = round(abs(Output[3]));

  // Enciende motores con PWM constante
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, o1);
  digitalWrite(IN2, o2);
  digitalWrite(IN3, o3);
  digitalWrite(IN4, o4);
}

/************************************************************
 * turnAround(): gira 180° en su propio eje
 * - Un motor hacia adelante, otro hacia atrás
 * - Delay fijo (ajustable) para completar giro
 ************************************************************/
void turnAround() {
  reverseSlightly();
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  // Motor A adelante, Motor B atrás -> giro en sitio
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(300);  // <--- ajustar si no completa 180°
  stop();     // detener después del giro
}

/************************************************************
 * reverseSlightly(): retrocede ligeramente
 * - Ambos motores en dirección inversa
 * - Breve delay para retroceso antes de girar
 ************************************************************/
void reverseSlightly() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);   // Motor A hacia atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Motor B hacia atrás
  digitalWrite(IN4, LOW);
  delay(200);               // Tiempo de retroceso
  stop();                   // Detiene tras retroceder
}

/************************************************************
 * InputToOutput(In1, In2, In3): feedforward de la NN
 * - Capa oculta usa tanh
 * - Capa salida usa tanh
 ************************************************************/
void InputToOutput(double In1, double In2, double In3) {
  double Accum;
  // Cálculo de capa oculta
  for(int i = 0; i < HiddenNodes; i++) {
    Accum = In1 * HiddenWeights[0][i]
          + In2 * HiddenWeights[1][i]
          + In3 * HiddenWeights[2][i];
    Hidden[i] = tanh(Accum);
  }
  // Cálculo de capa de salida
  for(int i = 0; i < OutputNodes; i++) {
    Accum = 0;
    for(int j = 0; j < HiddenNodes; j++) {
      Accum += Hidden[j] * OutputWeights[j][i];
    }
    Output[i] = tanh(Accum);
  }
}
// PWM (Pulse‑Width Modulation – Modulación por Ancho de Pulso)
// NN (Neural Network – Red Neuronal)
// ENA/ENB Son los pines que habilitan (Enable) el controlador de motores L298.