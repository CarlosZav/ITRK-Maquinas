// period of pulse accumulation and serial output, milliseconds
const int inputPin = 2;
const int pinSSR = 4;
const int MainPeriod = 100;
long previousMillis = 0; // will store last time of the cycle end

volatile unsigned long previousMicros =0;
volatile unsigned long duration =0; // accumulates pulse width
volatile unsigned int pulseCount =0;
unsigned int pulseCountCopy =0;
unsigned long durationTotal = 0;
unsigned long periodo = 0;
unsigned long controMicros = 0;

// interrupt handler
void freqCounterCallback() {
  pulseCount++;
  if (pulseCount > 0){
  unsigned long currentMicros = micros();
  duration += currentMicros - previousMicros;
  previousMicros = currentMicros;
  }
}

void calculatePeriod(){
  durationTotal += duration;
  periodo = float(durationTotal) / ((float)pulseCount - 1);
  Serial.print("Tiempo por semiPeriodo");
  Serial.print(periodo);
  Serial.println(" uS"); 
}

void setup(){
  Serial.begin(9600); 
  pinMode(inputPin, INPUT);
  pinMode(pinSSR, OUTPUT);
  digitalWrite(pinSSR, HIGH);
  attachInterrupt(digitalPinToInterrupt(inputPin), freqCounterCallback, RISING);
}

void loop(){

  calculatePeriod();

  unsigned long conteoMicros = micros();

  Serial.println(conteoMicros);

  if (conteoMicros >= 60000000){ 

    Serial.println("PASADOOOO");

    if(pulseCountCopy != pulseCount){
      if ((conteoMicros - 60000000) >= (periodo/4) ){   
        digitalWrite(pinSSR, LOW);
      }
    }
  } else{
    pulseCountCopy = pulseCount;
    Serial.print("pulso 1: ");
    Serial.println(pulseCountCopy);
    Serial.print("pulso 2: ");
    Serial.println(pulseCount);

  }
}