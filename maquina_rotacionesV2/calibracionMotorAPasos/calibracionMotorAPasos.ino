#define PUL 18  // Pin de pulsos en ESP32
#define DIR 19  // Pin de dirección
#define ENA 5   // Pin de habilitación (opcional)


//Variables para ROTACIONES
const int pasosPorCiclo = 1600;
unsigned long tiempoAnterior = 0;
unsigned long intervaloPaso = 0.0; // Microsegundos, ajusta para cambiar velocidad 312.5 para 1 rps
bool estadoPulso = LOW;
int contadorPasos = 0;
bool direccion = HIGH;
unsigned long tiempoInicioDireccion = 0;
int prevPos = 1;
unsigned long RPM = 60;
int numeroRotaciones = 60;
int conteoRotaciones = 0;
int numeroRotacionesCambio = 10;
int conteoRotacionesCambio = 0;

//Variables para FLEXIONES
//reutilizar pasosPorCiclo
//reutilizar tiempoAnterior
unsigned long intervaloPasoFlex = 0.0;
//reutilizar estadoPulso
//Reutilizar contadorPasos
//reutilizar direccion
//reutilizar tiempoInicioDireccion
int prevPosFlex = 0;
unsigned long setVelocidad = 100;
int anguloA = 90;
int anguloB = 90;
int flexionesSecadoras = 100; // establecido
int conteoFlexiones = 0;
float pasosAnguloA = 0.0;
float pasosAnguloB = 0.0;
float pasosCicloFlex = 0.0;

void setup() {

  delay (2000);
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(ENA, OUTPUT);
    
    digitalWrite(ENA, LOW); // Habilitar el driver
    digitalWrite(DIR, direccion);
    tiempoInicioDireccion = micros();
    
    calcularIntervaloPaso();
    calcularIntervaloPasoFlexiones();


  Serial.begin(115200);
}

void loop() {

  //controlRotaciones();

  controlFlexiones();

}

void calcularIntervaloPaso(){
  intervaloPaso = ((60000000) / (RPM * pasosPorCiclo)) / 2;
}

void controlRotaciones(){

  unsigned long tiempoActual = micros();

  if ((conteoRotaciones < numeroRotaciones) && (numeroRotaciones != 0)){

    if ((prevPos == 1) && (tiempoActual - tiempoAnterior >= intervaloPaso)){

      tiempoAnterior = tiempoActual;
      estadoPulso = !estadoPulso;
      digitalWrite(PUL, estadoPulso);

      if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
        contadorPasos++;
      }

      if(contadorPasos >= pasosPorCiclo){
        //prevPos = 2;
        //direccion = !direccion;
        //digitalWrite(DIR, direccion);
        conteoRotaciones ++;
        contadorPasos = 0;
      }

      if(conteoRotaciones - conteoRotacionesCambio >= numeroRotacionesCambio){
        prevPos = 2;
        direccion = !direccion;
        digitalWrite(DIR, direccion);
        conteoRotacionesCambio = conteoRotaciones;
      }

    } else if ((prevPos == 2 ) && (tiempoActual - tiempoAnterior >= intervaloPaso)){

      tiempoAnterior = tiempoActual;
      estadoPulso = !estadoPulso;
      digitalWrite(PUL, estadoPulso);

      if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
        contadorPasos++;
      }

      if(contadorPasos >= pasosPorCiclo ){
        //prevPos = 1;
        //direccion = !direccion;
        //digitalWrite(DIR, direccion);
        conteoRotaciones ++;
        contadorPasos = 0;
      }

      if(conteoRotaciones - conteoRotacionesCambio>= numeroRotacionesCambio){
        prevPos = 1;
        direccion = !direccion;
        digitalWrite(DIR, direccion);
        conteoRotacionesCambio = conteoRotaciones;
      }
    }
  }

  else{
    numeroRotaciones = 0;
    conteoRotaciones = 0;
  }
  
}

void calcularIntervaloPasoFlexiones(){

  pasosAnguloA = (anguloA*pasosPorCiclo)/360; //Equivalente a pasos por revolcuiones = pasosPorCiclo
  Serial.print("Pasos Angulo A = ");
  Serial.println(pasosAnguloA);
 
  pasosAnguloB = (anguloB*pasosPorCiclo)/360;
   Serial.print("Pasos Angulo B = ");
  Serial.println(pasosAnguloB);

  pasosCicloFlex = pasosAnguloA + pasosAnguloB; 
   Serial.print("Pasos Total = ");
  Serial.println(pasosCicloFlex);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

  intervaloPasoFlex = 60000000 / ( 2 * setVelocidad * pasosCicloFlex );

   Serial.print("Intervalo en uS ");
  Serial.println(intervaloPasoFlex);
}

void controlFlexiones(){

  unsigned long tiempoActual = micros();

  if ((conteoFlexiones < flexionesSecadoras) && (flexionesSecadoras != 0)){

    if ((prevPosFlex == 0) && (tiempoActual - tiempoAnterior >= intervaloPasoFlex)){

      tiempoAnterior = tiempoActual;
      estadoPulso = !estadoPulso;
      digitalWrite(PUL, estadoPulso);

      if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
        contadorPasos++;
      }

      if(contadorPasos >= pasosAnguloA){
        prevPosFlex = 2;
        direccion = !direccion;
        digitalWrite(DIR, direccion);
        conteoFlexiones ++;
        contadorPasos = 0;
      }

    } else if ((prevPosFlex == 1) && (tiempoActual - tiempoAnterior >= intervaloPasoFlex)){

      tiempoAnterior = tiempoActual;
      estadoPulso = !estadoPulso;
      digitalWrite(PUL, estadoPulso);

      if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
        contadorPasos++;
      }

      if(contadorPasos >= pasosCicloFlex){
        prevPosFlex = 2;
        direccion = !direccion;
        digitalWrite(DIR, direccion);
        conteoFlexiones ++;
        contadorPasos = 0;
      }

    } else if ((prevPosFlex == 2 ) && (tiempoActual - tiempoAnterior >= intervaloPasoFlex)){

      tiempoAnterior = tiempoActual;
      estadoPulso = !estadoPulso;
      digitalWrite(PUL, estadoPulso);

      if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
        contadorPasos++;
      }

      if(contadorPasos >= pasosCicloFlex ){
        prevPos = 1;
        direccion = !direccion;
        digitalWrite(DIR, direccion);
        conteoFlexiones ++;
        contadorPasos = 0;
      }
    }
  }
  else{
    flexionesSecadoras = 0;
    conteoFlexiones = 0;

  }
}