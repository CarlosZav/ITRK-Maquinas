//4207
//4235
//4187
//4210
//4222

// ****4210

// Pines del encoder
const int encoderPinA = 18;
const int encoderPinB = 19;

//Pines para el control del motor
const int pinPWMA = 21;
const int pinPWMB = 25;
const int pinMotorA = 22;
const int pinMotorB = 23;

float gradosActuales = 0;

// Número de pulsos por revolución del encoder
const int PPR =  4210;  // Cambia este valor según tu encoder

// Variables para el contador de pulsos y la velocidad
volatile long pulseCount = 0;
volatile long pulseCountGrades = 0;
unsigned long lastTime = 0;
float velocidadRPM = 0;
int direction = 1;  // 1 para adelante, -1 para atrás

//Valor para saturacion de la señál de repuesta
float velocidadMax = 120.0; //Velocidad Maxima del motor en rpm
float velocidadMin = 20.0; //Velocidad minima a la que el motor se e piez a a mover

//variavles PID
float cv = 0;
float cv1 = 0;
float error = 0;
float error1 = 0;
float error2 = 0;
float kp = 3.26;
float ki = 0.0001;
float kd = 0.31;
float tm = 0.1;
float setPoint = 60; // Pedirselo al usuario en rpm 93.5 max


// Función de interrupción para contar los pulsos del encoder
void IRAM_ATTR encoderISR() {
  // Lee el estado de los pines A y B
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  // Determina la dirección basada en la secuencia de los pulsos
  if (stateA == HIGH) {
    if (stateB == LOW) {
      direction = 1;  // Rotación en sentido horario
    } else {
      direction = -1; // Rotación en sentido antihorario
    }
  } else {
    if (stateB == HIGH) {
      direction = 1;  // Rotación en sentido horario
    } else {
      direction = -1; // Rotación en sentido antihorario
    }
  }

  // Incrementa o decrementa el contador de pulsos según la dirección
  pulseCount += direction;
  pulseCountGrades += direction;
}

void setup() {
  // Inicializa la comunicación serial
  Serial.begin(115200);

  // Configura los pines del encoder como entradas
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  pinMode(pinMotorA, OUTPUT);
  digitalWrite(pinMotorA, HIGH);

  pinMode(pinMotorB, OUTPUT);
  digitalWrite(pinMotorB, HIGH);

  pinMode(pinPWMA, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  analogWrite(pinPWMA, 0);
  analogWrite(pinPWMB, 0);

  // Configura la interrupción en el pin A del encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
}

void loop() {

  calcularGrados();
  calcularVelocidad();
  controlPID();

  //delay(100);

}

void calcularVelocidad(){
  // Obtén el tiempo actual
  unsigned long currentTime = millis();

  // Calcula la velocidad cada 100 ms
  if (currentTime - lastTime >= 100) {
    // Desactiva las interrupciones temporalmente para evitar inconsistencias
    noInterrupts();
    long pulses = labs(pulseCount);
    pulseCount = 0;
    interrupts();

    // Calcula la velocidad en RPM
    velocidadRPM = (pulses * 60000.0) / (PPR * (currentTime - lastTime));

    // Actualiza el tiempo de la última medición
    lastTime = currentTime;

    // Muestra la velocidad en RPM por el monitor serial
    Serial.print("Velocidad: ");
    Serial.print(velocidadRPM);
    Serial.print(" RPM, Dirección: ");
    if (direction == 1) {
      Serial.println("Adelante");
    } else {
      Serial.println("Atrás");
    }
    Serial.print("SetPoint = ");
    Serial.println(setPoint);
    Serial.print("Velocidad = ");
    Serial.println(velocidadRPM);
    Serial.print("Valor de cv = ");
    Serial.println(cv);
    Serial.print("error");
    Serial.println(error);
  }
}

void calcularGrados(){
  gradosActuales = (pulseCountGrades * 360) / PPR;
  /*Serial.print("Pulsos: ");
  Serial.println(pulseCountGrades);
  Serial.print("Grados: ");
  Serial.println(gradosActuales);
  */
}

void controlPID(){
  error = setPoint - velocidadRPM;

  //Ecuacion control PID discreto
  cv = cv1 + (kp + kd / tm) * error + (-kp+ki*tm - 2*kd/tm) * error1 + (kd/tm)*error2;
  error2 = error1;
  error1 = error;

  if (cv > velocidadMax){
    cv = velocidadMax;
  } else if (cv < velocidadMin){
    cv = velocidadMin;
  }

  cv1 = cv;

  analogWrite(pinPWMA, (cv*255) / 120);
  // ledcWriteChannel(canalPWM1, (cv*255) / 120);    // PWM en GPIO 5
  //ledcWriteChannel(canalPWM2, 0);    // PWM en GPIO 5
  analogWrite(pinPWMB, 0);

  /*Serial.print("valor de pwm");
  Serial.println((cv*255) / 120);*/

  /*
  Serial.print("SetPoint = ");
  Serial.println(setPoint);
  Serial.print("Velocidad = ");
  Serial.println(velocidadRPM);
  Serial.print("Valor de cv = ");
  Serial.println(cv);*/
}