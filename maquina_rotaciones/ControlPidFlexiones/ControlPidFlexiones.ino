// Asignación de pines
const int pinA = 5; // Canal A del encoder (GPIO 34)
const int pinB = 4; // Canal B del encoder (GPIO 35)

const pinPWM = 2;

// Constantes del controlador PID
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

int PPR = 900;

// Variables del controlador
double velocidadActual, valorPwmCalculado, Setpoint = 40;  // Setpoint = valor objetivo en rpms
double error, lastError, cumError, rateError;
unsigned long currentTime, previousTime;
double elapsedTime;

//Variables para el conteo con encoder
volatile int contadorPulsos = 0; // Cuenta de los pulsos
volatile int ultimoEstadoA = 0; // Último estado del pin A

const int controlMotor1 = 6;
const int controlMotor2 = 7;

const long tiempoCalculoVelocidad = 100;
unsigned long previousMillis = 0;
unsigned long currentTime = 0;



void IRAM_ATTR encoderISR() {
  int estadoA = digitalRead(pinA);
  int estadoB = digitalRead(pinB);

  // Determinar dirección según el cambio de estados
  if (estadoA != ultimoEstadoA) {
    if (estadoA == estadoB) {
      contadorPulsos++; // Girando en sentido horario
    } else {
      contadorPulsos--; // Girando en sentido antihorario
    }
  }
  ultimoEstadoA = estadoA; // Actualizar el estado previo

}



void setup() {

  //Seteo de pines de contro de giro del motor
  pinMode(controlMotor1, OUTPUT);
  digitalWrite(pin_valvulaA, LOW);

  pinMode(controlMotor2, OUTPUT);
  digitalWrite(pin_valvulaB, LOW);

  // Configurar pines del encoder como entradas
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  // Leer estado inicial del canal A
  ultimoEstadoA = digitalRead(pinA);

  // Configuración del pin para el control de PWM
  pinMode(pinPWM, OUTPUT);

  // Configurar interrupción para el canal A
  attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, CHANGE);

  Serial.begin(9600);

}

void loop() {

  // Realizar el calculo de la velocidad actual
  calcularVelocidadActual();

  // Llama a la función PID para calcular la salida
  valorPwmCalculado = computePID(velocidadActual);

  // Mapea la salida del PID a un rango de PWM (0-255)
  int pwmValue = map(Output, 0, 100, 0, 255);

  // Aplica el valor PWM al motor (0-255)
  analogWrite(PIN_OUTPUT, pwmValue);

  // Imprime los valores para depuración
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("\tInput: ");
  Serial.print(Input);
  Serial.print("\tPID Output: ");
  Serial.print(Output);
  Serial.print("\tPWM: ");
  Serial.println(pwmValue);

  delay(100);  // Retraso para evitar lecturas rápidas
}

double computePID(double inp) {
  currentTime = millis();                          // Obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime); // Calcular el tiempo transcurrido

  error = Setpoint - Input;                        // Calcular el error
  cumError += error * elapsedTime;                 // Calcular la integral del error
  rateError = (error - lastError) / elapsedTime;   // Calcular la derivada del error

  double output = Kp * error + Ki * cumError + Kd * rateError;  // Salida del PID

  // Limitar la salida del PID al rango 0-100%
  output = constrain(output, 0, 250);

  lastError = error;                               // Almacenar el error anterior
  previousTime = currentTime;                      // Almacenar el tiempo anterior

  return output;
}

void calcularVelocidadActual(){
  unsigned long currentMillis = millis();  // Obtener el tiempo actual

  // Calcular la velocidad cada segundo
  if (currentMillis - previousMillis >= tiempoCalculoVelocidad) {
    previousMillis = currentMillis;  // Actualizar el tiempo de referencia

    // Calcular la velocidad en RPM (Revoluciones por minuto)
    velocidadActual = (contadorPulsos / PPR) * (60000.0 / tiempoCalculoVelocidad);  // Dividir entre 2 si el encoder tiene 2 pulsos por revolución

    // Mostrar la velocidad en el monitor serial
    Serial.print("Velocidad del motor: ");
    Serial.print(velocidadActual);
    Serial.println(" RPM");

    contadorPulsos = 0;  // Resetear el contador de pulsos para el siguiente cálculo
  }
}
