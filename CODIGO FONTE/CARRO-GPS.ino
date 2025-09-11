#include "BluetoothSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Verificar se o ESP32 suporta Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Motor control pins - Ponte H
#define MOTOR_FRONT_PIN1 25  // OUT1 da ponte H (motores dianteiros)
#define MOTOR_FRONT_PIN2 33  // OUT2 da ponte H (motores dianteiros)
#define MOTOR_REAR_PIN1 32   // OUT3 da ponte H (motores traseiros)
#define MOTOR_REAR_PIN2 35   // OUT4 da ponte H (motores traseiros) - Mudado de 34 para 35

// PINS PARA ENABLE DA PONTE H (MUITO IMPORTANTE!)
#define MOTOR_FRONT_ENABLE 26  // Enable para motores dianteiros
#define MOTOR_REAR_ENABLE 27   // Enable para motores traseiros

// Pinos do cartão SD
#define SD_CS 5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK 18

// Pinos GPS
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// Pinos Sensor Ultrassônico e LED
#define PINO_TRIG 4  // Pino D4 conectado ao TRIG do HC-SR04
#define PINO_ECHO 2  // Pino D2 conectado ao ECHO do HC-SR04
#define PINO_LED 13  // Pino do LED indicador

// Objetos globais
BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;
File logFile;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Waypoints pré-definidos (latitude, longitude) 
const int NUM_WAYPOINTS = 5; 
const double WAYPOINTS[NUM_WAYPOINTS][2] = {   // Estacionamento Direito UFMT
  {-56.06222333,-15.60980333},  // Waypoint 1   
  {-56.06226167,-15.60986}, // Waypoint 2   
  {-56.06228833,-15.609885}, // Waypoint 3   
  {-56.06257333,-15.60962667}, // Waypoint 4 
  {-56.06226667,-15.60980167}  // Waypoint 5
};

// Configurações de navegação
const double WAYPOINT_TOLERANCE = 0.0001;  // Tolerância para considerar waypoint alcançado (aprox. 10 metros)
const double TURN_THRESHOLD = 0.5;         // Threshold em radianos para decidir virar
const int MAX_TURN_TIME = 3000;           // Tempo máximo para virar (ms)
const int WAYPOINT_TIMEOUT = 60000;       // Timeout para alcançar waypoint (ms)

// Variáveis de controle
char comando = 'S';
bool motorsEnabled = true;
bool debugMode = true;
bool mpuAvailable = false;
bool sdCardAvailable = false;
bool loggingEnabled = false;
bool gpsAvailable = false;

// Variáveis de navegação autônoma
bool autonomousMode = false;
int currentWaypoint = 0;
bool waypointsPassed[NUM_WAYPOINTS] = {false}; // Array para marcar waypoints visitados
unsigned long waypointStartTime = 0;
unsigned long lastNavigationUpdate = 0;
double lastValidLat = 0.0;
double lastValidLng = 0.0;

// Estrutura para dados dos sensores
struct SensorData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temperature;
    float distance;
};

SensorData currentSensor = {0};

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

// Timing variables
unsigned long lastCommand = 0;
unsigned long lastSensorRead = 0;
unsigned long lastLogWrite = 0;
unsigned long lastGpsUpdate = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastDistanceRead = 0;

// Timing constants
const int GPS_UPDATE_INTERVAL = 1000;      // ms
const int SENSOR_UPDATE_INTERVAL = 100;    // ms
const int DISTANCE_UPDATE_INTERVAL = 250;  // ms
const int NAVIGATION_UPDATE_INTERVAL = 500; // ms
const float DISTANCE_THRESHOLD = 15.0;     // cm

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== CARRINHO COMPLETO ESP32 V1.0 ===");
    Serial.println("Iniciando diagnóstico do sistema...\n");
    
    // Initialize motor pins
    setupMotors();
    
    // Initialize Bluetooth
    setupBluetooth();
    
    // Initialize sensor pins
    setupSensors();
    
    // Initialize GPS
    setupGPS();
    
    // Initialize MPU6050
    setupMPU();
    
    // Initialize SD Card
    setupSDCard();
    
    // Teste inicial
    Serial.println("\n=== TESTE INICIAL DOS MOTORES ===");
    testAllMotors();
    
    if (mpuAvailable) {
        Serial.println("\n=== TESTE INICIAL DOS SENSORES ===");
        readSensors();
    }
    
    // Aguarda GPS válido antes de iniciar navegação
    Serial.println("\n=== AGUARDANDO SINAL GPS VÁLIDO ===");
    waitForValidGPS();
    
    // Inicia navegação autônoma automaticamente após os testes
    Serial.println("\n🚀 INICIANDO NAVEGAÇÃO AUTÔNOMA");
    startAutonomousNavigation();
    
    printCommands();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Handle commands from both Bluetooth and Serial
    handleCommands();
    
    // Update ultrasonic sensor
    if (currentTime - lastDistanceRead >= DISTANCE_UPDATE_INTERVAL) {
        updateSonicSensor();
        lastDistanceRead = currentTime;
    }
    
    // Update MPU6050 sensors
    if (mpuAvailable && currentTime - lastSensorRead >= SENSOR_UPDATE_INTERVAL) {
        updateSensorData();
        lastSensorRead = currentTime;
    }
    
    // Update GPS data
    if (gpsAvailable && currentTime - lastGpsUpdate >= GPS_UPDATE_INTERVAL) {
        updateGPS();
        lastGpsUpdate = currentTime;
    }
    
    // Navigation logic
    if (autonomousMode && currentTime - lastNavigationUpdate >= NAVIGATION_UPDATE_INTERVAL) {
        updateNavigation();
        lastNavigationUpdate = currentTime;
    }
    
    // Log data periodically
    if (loggingEnabled && currentTime - lastLogWrite >= 2000) {
        logDataToSD();
        lastLogWrite = currentTime;
    }
    
    // Safety: Stop motors after 5 seconds without command (only in manual mode)
    if (!autonomousMode && currentTime - lastCommand > 5000 && comando != 'S') {
        stopAllMotors();
        comando = 'S';
        Serial.println("⚠️ TIMEOUT: Motores parados por segurança");
    }
    
    delay(10); // Small delay for stability
}

// === NAVIGATION FUNCTIONS ===

void waitForValidGPS() {
    Serial.println("📍 Aguardando sinal GPS válido...");
    
    unsigned long startTime = millis();
    while (!gpsData.hasValidLocation) {
        if (millis() - startTime > 60000) { // 60 segundos timeout
            Serial.println("⚠️ Timeout GPS - continuando sem GPS");
            return;
        }
        
        updateGPS();
        if (gpsData.hasValidLocation) {
            Serial.println("✓ GPS válido obtido!");
            lastValidLat = gpsData.lat;
            lastValidLng = gpsData.lng;
            break;
        }
        
        delay(1000);
        Serial.print(".");
    }
    Serial.println();
}

void startAutonomousNavigation() {
    if (!gpsData.hasValidLocation) {
        Serial.println("⚠️ GPS inválido - navegação autônoma desabilitada");
        return;
    }
    
    autonomousMode = true;
    currentWaypoint = 0;
    waypointStartTime = millis();
    
    // Reset waypoints status
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        waypointsPassed[i] = false;
    }
    
    Serial.println("🎯 NAVEGAÇÃO AUTÔNOMA INICIADA");
    Serial.printf("   Destino: Waypoint %d (%.6f, %.6f)\n", 
        currentWaypoint + 1, 
        WAYPOINTS[currentWaypoint][1], 
        WAYPOINTS[currentWaypoint][0]);
    
    logNavigationEvent("NAV_START", currentWaypoint);
}

void stopAutonomousNavigation() {
    autonomousMode = false;
    stopAllMotors();
    
    Serial.println("🛑 NAVEGAÇÃO AUTÔNOMA PARADA");
    
    // Exibe relatório final
    printNavigationReport();
    logNavigationEvent("NAV_STOP", currentWaypoint);
}

void updateNavigation() {
    if (!autonomousMode || !gpsData.hasValidLocation) {
        return;
    }
    
    // Verifica timeout do waypoint
    if (millis() - waypointStartTime > WAYPOINT_TIMEOUT) {
        Serial.printf("⚠️ TIMEOUT - Waypoint %d não alcançado\n", currentWaypoint + 1);
        nextWaypoint();
        return;
    }
    
    // Calcula distância e direção para o waypoint atual
    double targetLat = WAYPOINTS[currentWaypoint][1];
    double targetLng = WAYPOINTS[currentWaypoint][0];
    
    double distance = calculateDistance(gpsData.lat, gpsData.lng, targetLat, targetLng);
    double bearing = calculateBearing(gpsData.lat, gpsData.lng, targetLat, targetLng);
    
    if (debugMode) {
        Serial.printf("🧭 WP%d: Dist=%.2fm, Bear=%.1f°\n", 
            currentWaypoint + 1, distance * 111000, bearing * 180 / PI);
    }
    
    // Verifica se waypoint foi alcançado
    if (distance < WAYPOINT_TOLERANCE) {
        waypointReached();
        return;
    }
    
    // Verifica obstáculo
    if (currentSensor.distance > 0 && currentSensor.distance < DISTANCE_THRESHOLD) {
        handleObstacle();
        return;
    }
    
    // Navegação baseada no bearing
    navigateToTarget(bearing);
}

double calculateDistance(double lat1, double lng1, double lat2, double lng2) {
    // Fórmula de Haversine simplificada para distâncias pequenas
    double dlat = lat2 - lat1;
    double dlng = lng2 - lng1;
    
    return sqrt(dlat * dlat + dlng * dlng);
}

double calculateBearing(double lat1, double lng1, double lat2, double lng2) {
    double dlng = lng2 - lng1;
    double dlat = lat2 - lat1;
    
    return atan2(dlng, dlat);
}

void navigateToTarget(double targetBearing) {
    // Bearing atual do veículo (usando giroscópio ou GPS)
    double currentBearing = getCurrentBearing();
    
    // Diferença angular
    double bearingDiff = targetBearing - currentBearing;
    
    // Normaliza para [-PI, PI]
    while (bearingDiff > PI) bearingDiff -= 2 * PI;
    while (bearingDiff < -PI) bearingDiff += 2 * PI;
    
    if (abs(bearingDiff) < TURN_THRESHOLD) {
        // Movimento para frente
        moveForwardAutonomous();
    } else if (bearingDiff > 0) {
        // Vira à direita
        turnRight();
        comando = 'R';
    } else {
        // Vira à esquerda  
        turnLeft();
        comando = 'L';
    }
}

double getCurrentBearing() {
    // Usa giroscópio Z ou diferença de posições GPS
    if (mpuAvailable) {
        // Integração simples do giroscópio (método básico)
        static double integratedBearing = 0.0;
        integratedBearing += currentSensor.gyroZ * (NAVIGATION_UPDATE_INTERVAL / 1000.0);
        return integratedBearing;
    } else {
        // Usa mudança de posição GPS
        static double lastLat = 0.0, lastLng = 0.0;
        
        if (lastLat != 0.0 && lastLng != 0.0) {
            return calculateBearing(lastLat, lastLng, gpsData.lat, gpsData.lng);
        }
        
        lastLat = gpsData.lat;
        lastLng = gpsData.lng;
        return 0.0;
    }
}

void moveForwardAutonomous() {
    if (!motorsEnabled) return;
    
    Serial.println("⬆️ NAV: Movendo para frente");
    setFrontMotors(LOW, HIGH);
    setRearMotors(LOW, HIGH);
    comando = 'F';
}

void waypointReached() {
    waypointsPassed[currentWaypoint] = true;
    
    Serial.printf("🎯 WAYPOINT %d ALCANÇADO!\n", currentWaypoint + 1);
    Serial.printf("   Posição: %.6f, %.6f\n", gpsData.lat, gpsData.lng);
    
    logNavigationEvent("WAYPOINT_REACHED", currentWaypoint);
    
    // Para momentaneamente
    stopAllMotors();
    delay(2000); // Pausa de 2 segundos
    
    nextWaypoint();
}

void nextWaypoint() {
    currentWaypoint++;
    
    if (currentWaypoint >= NUM_WAYPOINTS) {
        // Todos os waypoints foram processados
        Serial.println("🏁 TODOS OS WAYPOINTS PROCESSADOS!");
        stopAutonomousNavigation();
        return;
    }
    
    waypointStartTime = millis();
    Serial.printf("🎯 Próximo destino: Waypoint %d (%.6f, %.6f)\n", 
        currentWaypoint + 1,
        WAYPOINTS[currentWaypoint][1], 
        WAYPOINTS[currentWaypoint][0]);
        
    logNavigationEvent("NEXT_WAYPOINT", currentWaypoint);
}

void handleObstacle() {
    static unsigned long obstacleStartTime = 0;
    static bool obstacleDetected = false;
    
    if (!obstacleDetected) {
        obstacleDetected = true;
        obstacleStartTime = millis();
        Serial.println("🚧 OBSTÁCULO DETECTADO - Parando");
        stopAllMotors();
        logNavigationEvent("OBSTACLE_DETECTED", currentWaypoint);
    }
    
    // Estratégia simples: espera 3 segundos e tenta desviar
    if (millis() - obstacleStartTime > 3000) {
        Serial.println("🔄 Tentando desviar...");
        
        // Gira à direita por 2 segundos
        turnRight();
        delay(2000);
        
        // Move para frente por 3 segundos
        moveForwardAutonomous();
        delay(3000);
        
        // Gira à esquerda para retomar direção
        turnLeft();
        delay(2000);
        
        obstacleDetected = false;
        logNavigationEvent("OBSTACLE_AVOIDED", currentWaypoint);
    }
}

void printNavigationReport() {
    Serial.println("\n📊 === RELATÓRIO DE NAVEGAÇÃO ===");
    Serial.printf("Waypoints processados: %d de %d\n", currentWaypoint, NUM_WAYPOINTS);
    
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        Serial.printf("  Waypoint %d: %s\n", i + 1, 
            waypointsPassed[i] ? "✓ PASSOU" : "✗ NÃO PASSOU");
    }
    
    int passedCount = 0;
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        if (waypointsPassed[i]) passedCount++;
    }
    
    Serial.printf("Taxa de sucesso: %.1f%%\n", (float)passedCount / NUM_WAYPOINTS * 100);
    Serial.println("================================\n");
}

void logNavigationEvent(String event, int waypointIndex) {
    if (!sdCardAvailable || !loggingEnabled) return;
    
    String filename = "/nav_log.txt";
    File navLog = SD.open(filename, FILE_APPEND);
    
    if (navLog) {
        String logEntry = String(millis()) + "," + event + ",WP" + String(waypointIndex + 1) + ",";
        logEntry += String(gpsData.lat, 6) + "," + String(gpsData.lng, 6);
        logEntry += "," + String(currentSensor.distance, 1);
        
        navLog.println(logEntry);
        navLog.close();
    }
}

// === SETUP FUNCTIONS ===

void setupMotors() {
    pinMode(MOTOR_FRONT_PIN1, OUTPUT);
    pinMode(MOTOR_FRONT_PIN2, OUTPUT);
    pinMode(MOTOR_REAR_PIN1, OUTPUT);
    pinMode(MOTOR_REAR_PIN2, OUTPUT);
    pinMode(MOTOR_FRONT_ENABLE, OUTPUT);
    pinMode(MOTOR_REAR_ENABLE, OUTPUT);
    
    // Enable H-bridge
    digitalWrite(MOTOR_FRONT_ENABLE, HIGH);
    digitalWrite(MOTOR_REAR_ENABLE, HIGH);
    
    // Start with motors stopped
    stopAllMotors();
    
    Serial.println("✓ Motores configurados e ponte H ativada");
}

void setupBluetooth() {
    if (!SerialBT.begin("Carrinho_ESP32")) {
        Serial.println("✗ ERRO: Bluetooth não inicializado!");
    } else {
        Serial.println("✓ Bluetooth iniciado: 'Carrinho_ESP32'");
    }
}

void setupSensors() {
    pinMode(PINO_TRIG, OUTPUT);
    pinMode(PINO_ECHO, INPUT);
    pinMode(PINO_LED, OUTPUT);
    digitalWrite(PINO_LED, LOW);
    Serial.println("✓ Sensor ultrassônico configurado");
}

void setupGPS() {
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    
    // Check if GPS is responding
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {
        if (gpsSerial.available()) {
            gpsAvailable = true;
            break;
        }
    }
    
    if (gpsAvailable) {
        Serial.println("✓ GPS inicializado");
    } else {
        Serial.println("✗ GPS não detectado (verifique conexões)");
    }
}

void setupMPU() {
    if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpuAvailable = true;
        
        Serial.println("✓ MPU6050 encontrado e configurado");
        Serial.println("  Calibrando MPU6050...");
        delay(1000);
        updateSensorData();
        Serial.println("  ✓ MPU6050 calibrado");
    } else {
        Serial.println("✗ MPU6050 não encontrado");
        mpuAvailable = false;
    }
}

void setupSDCard() {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (SD.begin(SD_CS)) {
        sdCardAvailable = true;
        Serial.println("✓ Cartão SD inicializado");
        
        // Create log file
        String filename = "/log_" + String(millis()/1000) + ".csv";
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            logFile.println("timestamp,command,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,temp,distance,lat,lng,speed");
            logFile.close();
            Serial.println("  ✓ Arquivo de log criado: " + filename);
        }
        
        // Create navigation log file
        File navLog = SD.open("/nav_log.txt", FILE_WRITE);
        if (navLog) {
            navLog.println("timestamp,event,waypoint,lat,lng,distance");
            navLog.close();
            Serial.println("  ✓ Arquivo de navegação criado");
        }
    } else {
        Serial.println("✗ Cartão SD não encontrado");
        sdCardAvailable = false;
    }
}

void printCommands() {
    Serial.println("\n=== SISTEMA PRONTO ===");
    Serial.println("\n📱 COMANDOS DISPONÍVEIS:");
    Serial.println("┌─────────────────────────────────────┐");
    Serial.println("│ MOVIMENTO:                          │");
    Serial.println("│  F - Frente    B - Ré               │");
    Serial.println("│  L - Esquerda  R - Direita          │");
    Serial.println("│  S - Parar     N - Neutro           │");
    Serial.println("├─────────────────────────────────────┤");
    Serial.println("│ NAVEGAÇÃO:                          │");
    Serial.println("│  I - Iniciar navegação autônoma     │");
    Serial.println("│  K - Parar navegação autônoma       │");
    Serial.println("│  W - Status waypoints               │");
    Serial.println("│  Q - Relatório de navegação         │");
    Serial.println("├─────────────────────────────────────┤");
    Serial.println("│ SISTEMA:                            │");
    Serial.println("│  T - Teste motores                  │");
    Serial.println("│  D - Debug ON/OFF                   │");
    Serial.println("│  E - Enable motores ON/OFF          │");
    Serial.println("├─────────────────────────────────────┤");
    Serial.println("│ SENSORES:                           │");
    Serial.println("│  M - Status MPU6050                 │");
    Serial.println("│  A - Ler todos sensores             │");
    Serial.println("│  G - Status GPS                     │");
    Serial.println("│  U - Ler distância ultrassônica     │");
    Serial.println("├─────────────────────────────────────┤");
    Serial.println("│ ARMAZENAMENTO:                      │");
    Serial.println("│  C - Status SD Card                 │");
    Serial.println("│  O - Toggle logging ON/OFF          │");
    Serial.println("├─────────────────────────────────────┤");
    Serial.println("│ DIAGNÓSTICO:                        │");
    Serial.println("│  P - Estado dos pinos               │");
    Serial.println("│  V - Status completo do sistema     │");
    Serial.println("└─────────────────────────────────────┘");
    Serial.println("\nEnvie comandos via Bluetooth ou Serial");
    Serial.println("====================================\n");
}

// === COMMAND HANDLING ===

void handleCommands() {
    char newCommand = 0;
    
    if (SerialBT.available()) {
        newCommand = SerialBT.read();
        Serial.print("📱 BT: ");
    } else if (Serial.available()) {
        newCommand = Serial.read();
        Serial.print("💻 Serial: ");
    }
    
    if (newCommand != 0 && newCommand != '\n' && newCommand != '\r') {
        lastCommand = millis();
        processCommand(newCommand);
    }
}

void processCommand(char cmd) {
    Serial.print("Comando '");
    Serial.print(cmd);
    Serial.println("' recebido");
    
    comando = cmd;
    
    if (loggingEnabled) {
        logCommand(String(cmd));
    }
    
    switch (toupper(cmd)) {
        // Movement commands (only work in manual mode)
        case 'F': 
            if (!autonomousMode) moveForward(); 
            else Serial.println("⚠️ Modo autônomo ativo!");
            break;
        case 'B': 
            if (!autonomousMode) moveBackward(); 
            else Serial.println("⚠️ Modo autônomo ativo!");
            break;
        case 'L': 
            if (!autonomousMode) turnLeft(); 
            else Serial.println("⚠️ Modo autônomo ativo!");
            break;
        case 'R': 
            if (!autonomousMode) turnRight(); 
            else Serial.println("⚠️ Modo autônomo ativo!");
            break;
        case 'S': 
            if (!autonomousMode) stopAllMotors(); 
            else Serial.println("⚠️ Modo autônomo ativo!");
            break;
        case 'N': 
            if (!autonomousMode) setNeutral(); 
            else Serial.println("⚠️ Modo autônomo ativo!");
            break;
        
        // Navigation commands
        case 'I': startAutonomousNavigation(); break;
        case 'K': stopAutonomousNavigation(); break;
        case 'W': printWaypointStatus(); break;
        case 'Q': printNavigationReport(); break;
        
        // System commands
        case 'T': testAllMotors(); break;
        case 'D': toggleDebug(); break;
        case 'E': toggleMotors(); break;
        
        // Sensor commands
        case 'M': mpuStatus(); break;
        case 'A': readAllSensors(); break;
        case 'G': gpsStatus(); break;
        case 'U': readUltrasonic(); break;
        
        // Storage commands
        case 'C': sdStatus(); break;
        case 'O': toggleLogging(); break;
        
        // Diagnostic commands
        case 'P': testPins(); break;
        case 'V': systemStatus(); break;
        
        default:
            Serial.println("✗ Comando inválido! Use 'V' para ver comandos.");
            break;
    }
}

void printWaypointStatus() {
    Serial.println("\n🎯 === STATUS DOS WAYPOINTS ===");
    Serial.printf("Modo autônomo: %s\n", autonomousMode ? "ATIVO" : "INATIVO");
    Serial.printf("Waypoint atual: %d de %d\n", currentWaypoint + 1, NUM_WAYPOINTS);
    
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        Serial.printf("  WP%d (%.6f, %.6f): %s\n", i + 1,
            WAYPOINTS[i][1], WAYPOINTS[i][0],
            waypointsPassed[i] ? "✓ PASSOU" : 
            (i == currentWaypoint && autonomousMode) ? "🎯 ATUAL" : "⏳ PENDENTE");
    }
}

// === MOVEMENT FUNCTIONS ===

void setFrontMotors(int pin1State, int pin2State) {
    digitalWrite(MOTOR_FRONT_PIN1, pin1State);
    digitalWrite(MOTOR_FRONT_PIN2, pin2State);
    
    if (debugMode) {
        Serial.print("  🔧 Front: P1=");
        Serial.print(pin1State);
        Serial.print(" P2=");
        Serial.println(pin2State);
    }
}

void setRearMotors(int pin1State, int pin2State) {
    digitalWrite(MOTOR_REAR_PIN1, pin1State);
    digitalWrite(MOTOR_REAR_PIN2, pin2State);
    
    if (debugMode) {
        Serial.print("  🔧 Rear: P1=");
        Serial.print(pin1State);
        Serial.print(" P2=");
        Serial.println(pin2State);
    }
}

void moveForward() {
    if (!motorsEnabled) {
        Serial.println("⚠️ Motores desabilitados!");
        return;
    }
    
    // Check for obstacles
    if (currentSensor.distance > 0 && currentSensor.distance < DISTANCE_THRESHOLD) {
        Serial.println("⚠️ Obstáculo detectado! Parando...");
        stopAllMotors();
        return;
    }
    
    Serial.println("⬆️ MOVENDO PARA FRENTE");
    setFrontMotors(LOW, HIGH);
    setRearMotors(LOW, HIGH);
}

void moveBackward() {
    if (!motorsEnabled) {
        Serial.println("⚠️ Motores desabilitados!");
        return;
    }
    
    Serial.println("⬇️ MOVENDO PARA TRÁS");
    setFrontMotors(HIGH, LOW);
    setRearMotors(HIGH, LOW);
}

void turnLeft() {
    if (!motorsEnabled) {
        Serial.println("⚠️ Motores desabilitados!");
        return;
    }
    
    Serial.println("⬅️ VIRANDO ESQUERDA");
    setFrontMotors(LOW, HIGH);
    setRearMotors(HIGH, LOW);
}

void turnRight() {
    if (!motorsEnabled) {
        Serial.println("⚠️ Motores desabilitados!");
        return;
    }
    
    Serial.println("➡️ VIRANDO DIREITA");
    setFrontMotors(HIGH, LOW);
    setRearMotors(LOW, HIGH);
}

void stopAllMotors() {
    Serial.println("⏹️ PARANDO MOTORES");
    setFrontMotors(LOW, LOW);
    setRearMotors(LOW, LOW);
    comando = 'S';
}

void setNeutral() {
    Serial.println("🔄 PONTO MORTO");
    setFrontMotors(LOW, LOW);
    setRearMotors(LOW, LOW);
}

// === SENSOR FUNCTIONS ===

void updateSonicSensor() {
    digitalWrite(PINO_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PINO_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINO_TRIG, LOW);
    
    long duracao = pulseIn(PINO_ECHO, HIGH, 30000);
    
    if (duracao > 0) {
        currentSensor.distance = (duracao * 0.0343) / 2.0;
        
        // Update LED
        digitalWrite(PINO_LED, (currentSensor.distance <= DISTANCE_THRESHOLD) ? HIGH : LOW);
        
        // Auto-stop if obstacle detected while moving forward (only in manual mode)
        if (!autonomousMode && currentSensor.distance <= DISTANCE_THRESHOLD && comando == 'F') {
            stopAllMotors();
            Serial.println("🛑 OBSTÁCULO! Distância: " + String(currentSensor.distance) + " cm");
        }
    }
}

void readUltrasonic() {
    updateSonicSensor();
    Serial.print("📏 Distância: ");
    Serial.print(currentSensor.distance, 1);
    Serial.println(" cm");
    
    if (currentSensor.distance <= DISTANCE_THRESHOLD) {
        Serial.println("⚠️ OBSTÁCULO PRÓXIMO!");
    }
}

void updateSensorData() {
    if (!mpuAvailable) return;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    currentSensor.accelX = a.acceleration.x;
    currentSensor.accelY = a.acceleration.y;
    currentSensor.accelZ = a.acceleration.z;
    currentSensor.gyroX = g.gyro.x;
    currentSensor.gyroY = g.gyro.y;
    currentSensor.gyroZ = g.gyro.z;
    currentSensor.temperature = temp.temperature;
}

void readSensors() {
    if (!mpuAvailable) {
        Serial.println("⚠️ MPU6050 não disponível!");
        return;
    }
    
    updateSensorData();
    
    Serial.println("\n📊 === LEITURA MPU6050 ===");
    Serial.println("🏃 ACELERÔMETRO (m/s²):");
    Serial.printf("  X: %.2f  Y: %.2f  Z: %.2f\n", 
        currentSensor.accelX, currentSensor.accelY, currentSensor.accelZ);
    
    Serial.println("🌀 GIROSCÓPIO (rad/s):");
    Serial.printf("  X: %.2f  Y: %.2f  Z: %.2f\n",
        currentSensor.gyroX, currentSensor.gyroY, currentSensor.gyroZ);
    
    Serial.printf("🌡️ TEMPERATURA: %.1f°C\n", currentSensor.temperature);
    
    float totalAccel = sqrt(pow(currentSensor.accelX, 2) + 
                           pow(currentSensor.accelY, 2) + 
                           pow(currentSensor.accelZ, 2));
    Serial.printf("📈 Aceleração total: %.2f m/s²\n", totalAccel);
}

void readAllSensors() {
    Serial.println("\n=== LEITURA COMPLETA DOS SENSORES ===");
    
    if (mpuAvailable) {
        readSensors();
    }
    
    readUltrasonic();
    
    if (gpsAvailable && gpsData.hasValidLocation) {
        printGPSData();
    } else {
        Serial.println("📍 GPS: Sem sinal");
    }
}

// === GPS FUNCTIONS ===

void updateGPS() {
    if (!gpsAvailable) return;
    
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isUpdated()) {
                gpsData.hasValidLocation = true;
                gpsData.lat = gps.location.lat();
                gpsData.lng = gps.location.lng();
                gpsData.speed = gps.speed.kmph();
                gpsData.altitude = gps.altitude.meters();
                gpsData.hdop = gps.hdop.value();
                gpsData.satellites = gps.satellites.value();
                
                // Atualiza última posição válida
                lastValidLat = gpsData.lat;
                lastValidLng = gpsData.lng;
            }
        }
    }
}

void printGPSData() {
    if (!gpsData.hasValidLocation) {
        Serial.println("📍 GPS: Aguardando sinal...");
        return;
    }
    
    Serial.println("📍 === DADOS GPS ===");
    Serial.printf("  Lat: %.6f | Lng: %.6f\n", gpsData.lat, gpsData.lng);
    Serial.printf("  Velocidade: %.1f km/h | Alt: %.1f m\n", gpsData.speed, gpsData.altitude);
    Serial.printf("  Satélites: %d | HDOP: %.1f\n", gpsData.satellites, gpsData.hdop / 100.0);
    
    if (autonomousMode) {
        double targetLat = WAYPOINTS[currentWaypoint][1];
        double targetLng = WAYPOINTS[currentWaypoint][0];
        double distance = calculateDistance(gpsData.lat, gpsData.lng, targetLat, targetLng);
        Serial.printf("  Distância até WP%d: %.1f m\n", currentWaypoint + 1, distance * 111000);
    }
}

void gpsStatus() {
    Serial.println("\n🛰️ === STATUS GPS ===");
    Serial.print("GPS disponível: ");
    Serial.println(gpsAvailable ? "✓ SIM" : "✗ NÃO");
    
    if (gpsAvailable) {
        Serial.print("Sinal válido: ");
        Serial.println(gpsData.hasValidLocation ? "✓ SIM" : "✗ NÃO");
        
        if (gpsData.hasValidLocation) {
            printGPSData();
        } else {
            Serial.printf("Última posição válida: %.6f, %.6f\n", lastValidLat, lastValidLng);
        }
    }
}

// === SD CARD FUNCTIONS ===

void logCommand(String command) {
    if (!sdCardAvailable || !loggingEnabled) return;
    
    String filename = "/log_" + String(millis()/100000) + ".csv";
    logFile = SD.open(filename, FILE_APPEND);
    
    if (logFile) {
        String logEntry = String(millis()) + "," + command + ",";
        logEntry += String(currentSensor.accelX, 2) + ",";
        logEntry += String(currentSensor.accelY, 2) + ",";
        logEntry += String(currentSensor.accelZ, 2) + ",";
        logEntry += String(currentSensor.gyroX, 2) + ",";
        logEntry += String(currentSensor.gyroY, 2) + ",";
        logEntry += String(currentSensor.gyroZ, 2) + ",";
        logEntry += String(currentSensor.temperature, 1) + ",";
        logEntry += String(currentSensor.distance, 1) + ",";
        logEntry += String(gpsData.lat, 6) + ",";
        logEntry += String(gpsData.lng, 6) + ",";
        logEntry += String(gpsData.speed, 1);
        
        logFile.println(logEntry);
        logFile.close();
    }
}

void logDataToSD() {
    logCommand("AUTO_LOG");
}

void sdStatus() {
    Serial.println("\n💾 === STATUS SD CARD ===");
    Serial.print("SD Card disponível: ");
    Serial.println(sdCardAvailable ? "✓ SIM" : "✗ NÃO");
    
    if (sdCardAvailable) {
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
        
        Serial.printf("  Tamanho: %llu MB\n", cardSize);
        Serial.printf("  Usado: %llu MB\n", usedBytes);
        Serial.printf("  Livre: %llu MB\n", cardSize - usedBytes);
        
        // Status dos arquivos de log
        if (SD.exists("/nav_log.txt")) {
            File navLog = SD.open("/nav_log.txt");
            if (navLog) {
                Serial.printf("  Log navegação: %d bytes\n", navLog.size());
                navLog.close();
            }
        }
    }
}

// === SYSTEM FUNCTIONS ===

void toggleDebug() {
    debugMode = !debugMode;
    Serial.print("🔧 Modo Debug: ");
    Serial.println(debugMode ? "ATIVADO" : "DESATIVADO");
}

void toggleMotors() {
    motorsEnabled = !motorsEnabled;
    digitalWrite(MOTOR_FRONT_ENABLE, motorsEnabled ? HIGH : LOW);
    digitalWrite(MOTOR_REAR_ENABLE, motorsEnabled ? HIGH : LOW);
    
    Serial.print("⚙️ Motores: ");
    Serial.println(motorsEnabled ? "HABILITADOS" : "DESABILITADOS");
    
    if (!motorsEnabled) {
        stopAllMotors();
        if (autonomousMode) {
            Serial.println("⚠️ Navegação autônoma pausada - motores desabilitados");
        }
    }
}

void toggleLogging() {
    loggingEnabled = !loggingEnabled;
    Serial.print("📝 Logging: ");
    Serial.println(loggingEnabled ? "ATIVADO" : "DESATIVADO");
    
    if (loggingEnabled && !sdCardAvailable) {
        Serial.println("⚠️ SD Card não disponível!");
        loggingEnabled = false;
    }
}

void mpuStatus() {
    Serial.println("\n🔍 === STATUS MPU6050 ===");
    Serial.print("MPU6050 disponível: ");
    Serial.println(mpuAvailable ? "✓ SIM" : "✗ NÃO");
    
    if (mpuAvailable) {
        Serial.println("Configurações:");
        Serial.println("  Range: ±2G / ±250°/s");
        Serial.println("  Filtro: 21Hz");
        Serial.println("  I2C: SDA=21, SCL=22");
        readSensors();
    }
}

void testPins() {
    Serial.println("\n🔌 === TESTE DE PINOS ===");
    
    Serial.println("MOTORES:");
    Serial.printf("  Front Enable (26): %s\n", digitalRead(MOTOR_FRONT_ENABLE) ? "HIGH" : "LOW");
    Serial.printf("  Rear Enable (27): %s\n", digitalRead(MOTOR_REAR_ENABLE) ? "HIGH" : "LOW");
    Serial.printf("  Front P1 (25): %s\n", digitalRead(MOTOR_FRONT_PIN1) ? "HIGH" : "LOW");
    Serial.printf("  Front P2 (33): %s\n", digitalRead(MOTOR_FRONT_PIN2) ? "HIGH" : "LOW");
    Serial.printf("  Rear P1 (32): %s\n", digitalRead(MOTOR_REAR_PIN1) ? "HIGH" : "LOW");
    Serial.printf("  Rear P2 (35): %s\n", digitalRead(MOTOR_REAR_PIN2) ? "HIGH" : "LOW");
    
    Serial.println("SENSORES:");
    Serial.printf("  LED (13): %s\n", digitalRead(PINO_LED) ? "HIGH" : "LOW");
    Serial.printf("  Trig (4): %s\n", digitalRead(PINO_TRIG) ? "HIGH" : "LOW");
}

void systemStatus() {
    Serial.println("\n=== STATUS COMPLETO DO SISTEMA ===");
    Serial.println("┌─────────────────────────────────┐");
    Serial.printf("│ 🔋 Uptime: %lu segundos        │\n", millis() / 1000);
    Serial.printf("│ 📱 Bluetooth: %s              │\n", SerialBT.hasClient() ? "Conectado  " : "Aguardando ");
    Serial.printf("│ ⚙️  Motores: %s              │\n", motorsEnabled ? "Habilitados" : "Desabilitado");
    Serial.printf("│ 🧭 MPU6050: %s               │\n", mpuAvailable ? "OK         " : "Não encontr");
    Serial.printf("│ 📍 GPS: %s                   │\n", gpsAvailable ? "OK         " : "Não encontr");
    Serial.printf("│ 💾 SD Card: %s               │\n", sdCardAvailable ? "OK         " : "Não encontr");
    Serial.printf("│ 📝 Logging: %s               │\n", loggingEnabled ? "Ativo      " : "Inativo    ");
    Serial.printf("│ 🔧 Debug: %s                 │\n", debugMode ? "ON         " : "OFF        ");
    Serial.printf("│ 📏 Distância: %.1f cm        │\n", currentSensor.distance);
    Serial.printf("│ 🚗 Comando atual: %c         │\n", comando);
    Serial.printf("│ 🎯 Modo autônomo: %s         │\n", autonomousMode ? "ATIVO      " : "INATIVO    ");
    if (autonomousMode) {
        Serial.printf("│ 📍 Waypoint atual: %d/%d       │\n", currentWaypoint + 1, NUM_WAYPOINTS);
    }
    Serial.println("└─────────────────────────────────┘");
    
    if (autonomousMode) {
        printWaypointStatus();
    }
}

void testAllMotors() {
    if (!motorsEnabled) {
        Serial.println("⚠️ Habilitando motores para teste...");
        motorsEnabled = true;
        digitalWrite(MOTOR_FRONT_ENABLE, HIGH);
        digitalWrite(MOTOR_REAR_ENABLE, HIGH);
    }
    
    // Para navegação autônoma durante o teste
    bool wasAutonomous = autonomousMode;
    if (autonomousMode) {
        Serial.println("⚠️ Pausando navegação autônoma para teste...");
        autonomousMode = false;
    }
    
    Serial.println("🧪 Iniciando teste de motores...");
    
    Serial.println("1. Teste FRENTE (2s)...");
    moveForward();
    delay(2000);
    
    Serial.println("2. Teste RÉ (2s)...");
    moveBackward();
    delay(2000);
    
    Serial.println("3. Teste ESQUERDA (1.5s)...");
    turnLeft();
    delay(1500);
    
    Serial.println("4. Teste DIREITA (1.5s)...");
    turnRight();
    delay(1500);
    
    Serial.println("5. PARANDO...");
    stopAllMotors();
    
    // Restaura navegação autônoma se estava ativa
    if (wasAutonomous) {
        Serial.println("🔄 Retomando navegação autônoma...");
        autonomousMode = true;
    }
    
    Serial.println("✅ Teste concluído!");
}