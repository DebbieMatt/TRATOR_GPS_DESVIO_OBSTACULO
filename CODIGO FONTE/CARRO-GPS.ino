#include "BluetoothSerial.h"  
#include <ESP32Servo.h>
#include <TinyGPS++.h>

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17

#define GPS_BAUD 9600

#define pin1 25 
#define pin2 33 
#define pin3 32 
#define pin4 34 

BluetoothSerial SerialBT; 
char comando;  

static const int servoPin = 13;

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

void gps_mode() {
  // This sketch displays information every time a new sentence is correctly encoded.
  unsigned long start = millis();

  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LONG: "); 
      Serial.println(gps.location.lng(), 6);
      Serial.print("SPEED (km/h) = "); 
      Serial.println(gps.speed.kmph()); 
      Serial.print("ALT (min)= "); 
      Serial.println(gps.altitude.meters());
      Serial.print("HDOP = "); 
      Serial.println(gps.hdop.value() / 100.0); 
      Serial.print("Satellites = "); 
      Serial.println(gps.satellites.value()); 
      Serial.print("Time in UTC: ");
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
      Serial.println("");
    }
  }
}

Servo servo1;

const int PINO_TRIG = 4; // Pino D4 conectado ao TRIG do HC-SR04
const int PINO_ECHO = 2; // Pino D2 conectado ao ECHO do HC-SR047
//INCLUSÃO DO LED
const int PINO_LED = 5;  // Pino D5 conectado ao LED

void sensor_sonico() {
  digitalWrite(PINO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PINO_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_TRIG, LOW);
  
  long duracao = pulseIn(PINO_ECHO, HIGH); // Mede o tempo de resposta do ECHO
  float distancia = (duracao * 0.0343) / 2;// Calcula a distância usando a velocidade do som (aproximadamente 343 m/s)
  Serial.print("Distância: ");
  Serial.print(distancia);
  Serial.println(" cm");
  
  if (distancia <= 10) // quando estiver menor ou igual a 10 cm enviará um alerta. ** Valor editável para a distância preferir.
  {
    digitalWrite(PINO_LED, HIGH); // Acende o LED se a distância for menor ou igual a 10 cm
  } else {
    digitalWrite(PINO_LED, LOW);  // Desliga o LED caso contrário
  }
  
  delay(1000); // Aguarda 1 segundo antes de fazer a próxima leitura
}

void setup() {   
    Serial.begin(9600);
    servo1.attach(servoPin);     
    
    //btSerial.begin(9600);   
    SerialBT.begin("Carrinho");    
    
    pinMode(pin1, OUTPUT);   
    pinMode(pin2, OUTPUT);   
    pinMode(pin3, OUTPUT);   
    pinMode(pin4, OUTPUT);     
    
    Serial.println("Fim Setup");   
    delay(2000); 

    Serial.begin(9600); // Inicializa a comunicação serial
    pinMode(PINO_TRIG, OUTPUT); // Configura o pino TRIG como saída
    pinMode(PINO_ECHO, INPUT);  // Configura o pino ECHO como entrada
    pinMode(PINO_LED, OUTPUT);  // Configura o pino LED como saída

  // Serial Monitor
  Serial.begin(115200);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");
}  

void loop() {   

   for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(10);
    sensor_sonico();
    gps_mode();
  }

  for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(10);
    sensor_sonico();
    gps_mode();
  } 
    
    if (SerialBT.available()) {     
        comando = SerialBT.read();   
    }    
        
    switch (comando) {     
        case 'F': { //move frente         
            digitalWrite(pin1, HIGH);         
            digitalWrite(pin2, LOW);           
            digitalWrite(pin3, HIGH);         
            digitalWrite(pin4, LOW);         
            break;       
        }     
        case 'I': {//frente direita           
            digitalWrite(pin1, HIGH);         
            digitalWrite(pin2, LOW);  
            digitalWrite(pin3, LOW);         
            digitalWrite(pin4, HIGH);         
            break;      
        }     
        case 'G': {//frente esquerda          
            digitalWrite(pin1, LOW);         
            digitalWrite(pin2, HIGH);           
            digitalWrite(pin3, HIGH);         
            digitalWrite(pin4, LOW);         
            break;       
        }     
        case 'R': {//direita           
            digitalWrite(pin1, HIGH);         
            digitalWrite(pin2, LOW);           
            digitalWrite(pin3, LOW);         
            digitalWrite(pin4, LOW);         
            break;       
        }     
        case 'L': {//esquerda           
            digitalWrite(pin1, LOW);         
            digitalWrite(pin2, LOW);           
            digitalWrite(pin3, HIGH);         
            digitalWrite(pin4, LOW);         
            break;       
        }     
        case 'B': {// ré           
            digitalWrite(pin1, LOW);         
            digitalWrite(pin2, HIGH);           
            digitalWrite(pin3, LOW);         
            digitalWrite(pin4, HIGH);         
            break;       
        }     
        case 'H': {// ré esquerda           
            digitalWrite(pin1, LOW); 
            digitalWrite(pin2, LOW);           
            digitalWrite(pin3, LOW);         
            digitalWrite(pin4, HIGH);         
            break;       
        }     
        case 'J': {//ré direita         
            digitalWrite(pin1, LOW);         
            digitalWrite(pin2, HIGH);           
            digitalWrite(pin3, LOW);         
            digitalWrite(pin4, LOW);         
            break;       
        }     
        default: {         
            digitalWrite(pin1, LOW);         
            digitalWrite(pin2, LOW);           
            digitalWrite(pin3, LOW);         
            digitalWrite(pin4, LOW);         
            break;       
        }   
    } 
}