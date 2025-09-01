#include "BluetoothSerial.h"  
#include <ESP32Servo.h>
#include <TinyGPS++.h>

// Verificar se o ESP32 suporta Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Verificar se é uma placa ESP32
#ifndef ESP32
#error This code is designed for ESP32 only!
#endif

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// Motor control pins - Ponte H
// Motores da DIREITA conectados no OUT1 e OUT2 da ponte H
#define MOTOR_RIGHT_PIN1 25  // Controla OUT1 da ponte H (motores direita)
#define MOTOR_RIGHT_PIN2 33  // Controla OUT2 da ponte H (motores direita)
// Motores da ESQUERDA conectados no OUT3 e OUT4 da ponte H  
#define MOTOR_LEFT_PIN1 32   // Controla OUT3 da ponte H (motores esquerda)
#define MOTOR_LEFT_PIN2 34   // Controla OUT4 da ponte H (motores esquerda)

// Sensor pins
#define PINO_TRIG 4
#define PINO_ECHO 2
#define PINO_LED 5
#define SERVO_PIN 13

// Timing constants
#define GPS_UPDATE_INTERVAL 100    // ms
#define SENSOR_UPDATE_INTERVAL 50  // ms
#define SERVO_DELAY 15            // ms
#define DISTANCE_THRESHOLD 10.0    // cm

BluetoothSerial SerialBT; 
char comando = 'S'; // Default stop command

Servo servo1;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Timing variables
unsigned long lastGpsUpdate = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastServoUpdate = 0;

// Servo control variables
int currentServoPos = 0;
int servoDirection = 1; // 1 for forward, -1 for backward
bool servoActive = true;

// GPS data cache
struct GPSData {
  bool hasValidLocation = false;
  double lat = 0.0;
  double lng = 0.0;
  double speed = 0.0;
  double altitude = 0.0;
  uint32_t hdop = 0;
  uint32_t satellites = 0;
} gpsData;

// Motor control states
enum MotorState {
  FORWARD,
  BACKWARD,
  STOP
};

void setup() {   
    Serial.begin(115200);
    
    // Initialize servo
    servo1.attach(SERVO_PIN);
    servo1.write(0);
    
    // Initialize Bluetooth
    if (!SerialBT.begin("Carrinho_4Motores")) {
        Serial.println("ERRO: Falha ao inicializar Bluetooth!");
        Serial.println("Verifique se esta é uma placa ESP32 compatível.");
    } else {
        Serial.println("Bluetooth inicializado: Carrinho_4Motores");
    }    
    
    // Initialize motor control pins
    pinMode(MOTOR_RIGHT_PIN1, OUTPUT);   
    pinMode(MOTOR_RIGHT_PIN2, OUTPUT);   
    pinMode(MOTOR_LEFT_PIN1, OUTPUT);   
    pinMode(MOTOR_LEFT_PIN2, OUTPUT);   
    
    // Initialize sensor pins
    pinMode(PINO_TRIG, OUTPUT);
    pinMode(PINO_ECHO, INPUT);
    pinMode(PINO_LED, OUTPUT);
    
    // Initialize GPS
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    
    // Stop all motors initially
    stopAllMotors();
    
    Serial.println("=== CARRINHO 4 MOTORES INICIADO ===");
    Serial.println("Motores Direita: OUT1-OUT2 | Motores Esquerda: OUT3-OUT4");
    Serial.println("Comandos: F=Frente, B=Ré, L=Esquerda, R=Direita, S=Parar");
}  

void loop() {
    unsigned long currentTime = millis();
    
    // Handle Bluetooth commands (highest priority)
    handleBluetoothCommands();
    
    // Update servo position (non-blocking)
    updateServo(currentTime);
    
    // Update sensor reading (time-based)
    if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
        updateSonicSensor();
        lastSensorUpdate = currentTime;
    }
    
    // Update GPS data (time-based)
    if (currentTime - lastGpsUpdate >= GPS_UPDATE_INTERVAL) {
        updateGPS();
        lastGpsUpdate = currentTime;
    }
}

void handleBluetoothCommands() {
    if (SerialBT.available()) {     
        comando = SerialBT.read();   
        executeMotorCommand(comando);
        
        // Debug: mostrar comando recebido
        Serial.print("Comando: ");
        Serial.println(comando);
    }
}

void executeMotorCommand(char cmd) {
    switch (cmd) {     
        case 'F': // Mover para frente
            moveForward();
            Serial.println("Movimento: FRENTE");
            break;
            
        case 'B': // Mover para trás
            moveBackward();
            Serial.println("Movimento: RÉ");
            break;
            
        case 'L': // Virar à esquerda (motores direita para frente)
            turnLeft();
            Serial.println("Movimento: ESQUERDA");
            break;
            
        case 'R': // Virar à direita (motores esquerda para frente)
            turnRight();
            Serial.println("Movimento: DIREITA");
            break;
            
        case 'G': // Curva suave à esquerda (frente + esquerda)
            curveLeft();
            Serial.println("Movimento: CURVA ESQUERDA");
            break;
            
        case 'I': // Curva suave à direita (frente + direita)
            curveRight();
            Serial.println("Movimento: CURVA DIREITA");
            break;
            
        case 'H': // Ré com curva à esquerda
            reverseLeft();
            Serial.println("Movimento: RÉ ESQUERDA");
            break;
            
        case 'J': // Ré com curva à direita
            reverseRight();
            Serial.println("Movimento: RÉ DIREITA");
            break;
            
        case 'S': // Parar
        default:
            stopAllMotors();
            Serial.println("Movimento: PARADO");
            break;
    } 
}

// === FUNÇÕES DE CONTROLE DOS MOTORES ===

void moveForward() {
    // Ambos os lados para frente
    setRightMotors(FORWARD);
    setLeftMotors(FORWARD);
}

void moveBackward() {
    // Ambos os lados para trás
    setRightMotors(BACKWARD);
    setLeftMotors(BACKWARD);
}

void turnLeft() {
    // Motores direita para frente, esquerda parados ou para trás
    setRightMotors(FORWARD);
    setLeftMotors(BACKWARD);
}

void turnRight() {
    // Motores esquerda para frente, direita parados ou para trás
    setRightMotors(BACKWARD);
    setLeftMotors(FORWARD);
}

void curveLeft() {
    // Curva suave: direita rápido, esquerda lento/parado
    setRightMotors(FORWARD);
    setLeftMotors(STOP);
}

void curveRight() {
    // Curva suave: esquerda rápido, direita lento/parado
    setRightMotors(STOP);
    setLeftMotors(FORWARD);
}

void reverseLeft() {
    // Ré com curva à esquerda
    setRightMotors(BACKWARD);
    setLeftMotors(STOP);
}

void reverseRight() {
    // Ré com curva à direita
    setRightMotors(STOP);
    setLeftMotors(BACKWARD);
}

void stopAllMotors() {
    setRightMotors(STOP);
    setLeftMotors(STOP);
}

// === FUNÇÕES DE CONTROLE INDIVIDUAL DOS LADOS ===

void setRightMotors(MotorState state) {
    // Controla motores da DIREITA (OUT1 e OUT2 da ponte H)
    switch (state) {
        case FORWARD:
            digitalWrite(MOTOR_RIGHT_PIN1, HIGH);
            digitalWrite(MOTOR_RIGHT_PIN2, LOW);
            break;
        case BACKWARD:
            digitalWrite(MOTOR_RIGHT_PIN1, LOW);
            digitalWrite(MOTOR_RIGHT_PIN2, HIGH);
            break;
        case STOP:
        default:
            digitalWrite(MOTOR_RIGHT_PIN1, LOW);
            digitalWrite(MOTOR_RIGHT_PIN2, LOW);
            break;
    }
}

void setLeftMotors(MotorState state) {
    // Controla motores da ESQUERDA (OUT3 e OUT4 da ponte H)
    switch (state) {
        case FORWARD:
            digitalWrite(MOTOR_LEFT_PIN1, HIGH);
            digitalWrite(MOTOR_LEFT_PIN2, LOW);
            break;
        case BACKWARD:
            digitalWrite(MOTOR_LEFT_PIN1, LOW);
            digitalWrite(MOTOR_LEFT_PIN2, HIGH);
            break;
        case STOP:
        default:
            digitalWrite(MOTOR_LEFT_PIN1, LOW);
            digitalWrite(MOTOR_LEFT_PIN2, LOW);
            break;
    }
}

// === FUNÇÕES DO SERVO ===

void updateServo(unsigned long currentTime) {
    if (!servoActive || (currentTime - lastServoUpdate < SERVO_DELAY)) {
        return;
    }
    
    currentServoPos += servoDirection;
    
    // Change direction at limits
    if (currentServoPos >= 180) {
        currentServoPos = 180;
        servoDirection = -1;
    } else if (currentServoPos <= 0) {
        currentServoPos = 0;
        servoDirection = 1;
    }
    
    servo1.write(currentServoPos);
    lastServoUpdate = currentTime;
}

// === FUNÇÕES DO SENSOR ULTRASSÔNICO ===

void updateSonicSensor() {
    // Trigger pulse
    digitalWrite(PINO_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PINO_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINO_TRIG, LOW);
    
    // Measure echo duration with timeout
    long duracao = pulseIn(PINO_ECHO, HIGH, 30000); // 30ms timeout
    
    if (duracao == 0) {
        // Timeout occurred, sensor may be disconnected
        return;
    }
    
    float distancia = (duracao * 0.0343) / 2.0;
    
    // Only print distance occasionally to reduce serial overhead
    static uint8_t printCounter = 0;
    if (++printCounter >= 20) { // Print every 20th reading
        Serial.print("Distância: ");
        Serial.print(distancia, 1);
        Serial.println(" cm");
        printCounter = 0;
    }
    
    // Update LED based on distance
    digitalWrite(PINO_LED, (distancia <= DISTANCE_THRESHOLD) ? HIGH : LOW);
    
    // Auto-stop if obstacle detected (opcional)
    /*
    if (distancia <= DISTANCE_THRESHOLD && comando == 'F') {
        stopAllMotors();
        Serial.println("OBSTÁCULO DETECTADO - PARANDO!");
    }
    */
}

// === FUNÇÕES DO GPS ===

void updateGPS() {
    // Process available GPS data
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isUpdated()) {
                // Update cached GPS data
                gpsData.hasValidLocation = true;
                gpsData.lat = gps.location.lat();
                gpsData.lng = gps.location.lng();
                gpsData.speed = gps.speed.kmph();
                gpsData.altitude = gps.altitude.meters();
                gpsData.hdop = gps.hdop.value();
                gpsData.satellites = gps.satellites.value();
                
                // Print GPS data less frequently to reduce serial overhead
                static uint8_t gpsCounter = 0;
                if (++gpsCounter >= 30) { // Print every 30th update (~3 seconds)
                    printGPSData();
                    gpsCounter = 0;
                }
            }
        }
    }
}

void printGPSData() {
    if (!gpsData.hasValidLocation) return;
    
    Serial.println("=== GPS DATA ===");
    Serial.print("LAT: "); Serial.print(gpsData.lat, 6);
    Serial.print(" | LNG: "); Serial.print(gpsData.lng, 6);
    Serial.print(" | VEL: "); Serial.print(gpsData.speed, 1); Serial.print(" km/h");
    Serial.print(" | SAT: "); Serial.print(gpsData.satellites);
    Serial.print(" | HDOP: "); Serial.println(gpsData.hdop / 100.0, 1);
}

// === FUNÇÕES UTILITÁRIAS ===

void setServoActive(bool active) {
    servoActive = active;
    if (!active) {
        servo1.write(90); // Position servo to center when inactive
    }
}

GPSData getGPSData() {
    return gpsData;
}

// Função para teste dos motores (chamada via Bluetooth com comando 'T')
void testMotors() {
    Serial.println("=== TESTE DOS MOTORES ===");
    
    Serial.println("Testando motores DIREITA...");
    setRightMotors(FORWARD);
    delay(1000);
    setRightMotors(STOP);
    delay(500);
    
    Serial.println("Testando motores ESQUERDA...");
    setLeftMotors(FORWARD);
    delay(1000);
    setLeftMotors(STOP);
    delay(500);
    
    Serial.println("Teste concluído!");
}
