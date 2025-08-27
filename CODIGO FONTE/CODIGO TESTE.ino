#include <Arduino.h>
#include <TinyGPS++.h>        // Biblioteca para processamento de dados GPS
#include <HardwareSerial.h>   // Para comunicação serial com GPS
#include <BluetoothSerial.h>  // Para comunicação Bluetooth

// Definições dos pinos do ESP32
const int MOTOR_DIR1 = 12;    // Pino para motor direito
const int MOTOR_DIR2 = 13;    // Pino para motor direito
const int MOTOR_ESQ1 = 14;    // Pino para motor esquerdo
const int MOTOR_ESQ2 = 15;    // Pino para motor esquerdo
const int SENSOR_PROX = 34;   // Pino analógico para sensor de proximidade (ADC1)
const int LED_INDICADOR = 2;  // LED interno do ESP32
const int BOTAO_EMERGENCIA = 4; // Pino para botão de emergência

// Configuração do GPS - usando UART2 do ESP32
#define RXD2 16
#define TXD2 17
HardwareSerial SerialGPS(2);

// Configuração Bluetooth
BluetoothSerial SerialBT;

// Objeto GPS
TinyGPSPlus gps;

// Constantes de configuração
const int DISTANCIA_PARADA = 30;     // Distância em cm para parar
const int VELOCIDADE_NORMAL = 180;   // Velocidade normal (0-255) - PWM do ESP32
const int VELOCIDADE_LENTA = 120;    // Velocidade reduzida perto do waypoint
const int RAIO_ATINGIDO = 5;         // Raio em metros para considerar waypoint atingido
const int FREQUENCIA_PWM = 5000;     // Frequência do PWM (5kHz)
const int RESOLUCAO_PWM = 8;         // Resolução do PWM (8 bits = 0-255)

// Waypoints pré-definidos (latitude, longitude)
const int NUM_WAYPOINTS = 3;
const double WAYPOINTS[NUM_WAYPOINTS][2] = {
  {-23.5505, -46.6333},  // Waypoint 1
  {-23.5630, -46.6540},  // Waypoint 2
  {-23.5489, -46.6388}   // Waypoint 3
};

// Canais PWM do ESP32
const int CANAL_DIR1 = 0;
const int CANAL_DIR2 = 1;
const int CANAL_ESQ1 = 2;
const int CANAL_ESQ2 = 3;

// Estrutura para estado do sistema
struct EstadoSistema {
  int waypointAtual;
  bool emMovimento;
  bool obstaculoDetectado;
  bool emergencia;
  double latitude;
  double longitude;
  bool gpsValido;
  int satellites;
  float precisao;
};

EstadoSistema estado = {0, false, false, false, 0.0, 0.0, false, 0, 0.0};

// Protótipos de funções
void inicializarSistema();
void inicializarPWM();
void verificarEntradas();
void processarGPS();
void navegarParaWaypoint();
void calcularNavegacao();
void moverFrente(int velocidade = VELOCIDADE_NORMAL);
void moverTras(int velocidade = VELOCIDADE_NORMAL);
void parar();
void virarDireita(int intensidade = 50);
void virarEsquerda(int intensidade = 50);
int lerSensorProximidade();
bool verificarObstaculo();
double calcularDistancia(double lat1, double lon1, double lat2, double lon2);
bool waypointAtingido(double lat, double lon, int waypointIndex);
void tratarEmergencia();
void tarefaComunicacao(void *parameter);
void enviarDadosBluetooth();

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin("ESP32_Carro"); // Nome do dispositivo Bluetooth
  
  inicializarSistema();
  inicializarPWM();
  
  // Criar tarefa para comunicação (usando o segundo núcleo)
  xTaskCreatePinnedToCore(
    tarefaComunicacao,   // Função da tarefa
    "TarefaComunicacao", // Nome da tarefa
    10000,               // Tamanho do stack
    NULL,                // Parâmetro
    1,                   // Prioridade
    NULL,                // Task handle
    0                    // Núcleo (0 ou 1)
  );
  
  Serial.println("Sistema de navegação ESP32 iniciado");
  SerialBT.println("Sistema de navegação ESP32 iniciado");
  Serial.print("Navegando para Waypoint ");
  Serial.println(estado.waypointAtual + 1);
}

void loop() {
  // Verificar entradas (sensor, botão de emergência)
  verificarEntradas();
  
  // Processar dados GPS
  processarGPS();
  
  // Verificar se há emergência
  if (estado.emergencia) {
    tratarEmergencia();
    return;
  }
  
  // Navegar apenas se GPS for válido e não houver obstáculos
  if (estado.gpsValido && !estado.obstaculoDetectado) {
    navegarParaWaypoint();
  } else if (estado.obstaculoDetectado) {
    parar();
    Serial.println("Obstáculo detectado! Aguardando...");
    SerialBT.println("Obstáculo detectado! Aguardando...");
    delay(1000);
  }
  
  delay(50);
}

void inicializarSistema() {
  // Configurar pinos
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  pinMode(MOTOR_ESQ1, OUTPUT);
  pinMode(MOTOR_ESQ2, OUTPUT);
  pinMode(LED_INDICADOR, OUTPUT);
  pinMode(SENSOR_PROX, INPUT);
  pinMode(BOTAO_EMERGENCIA, INPUT_PULLUP);
  
  // Estado inicial
  estado.waypointAtual = 0;
  estado.emMovimento = false;
  estado.obstaculoDetectado = false;
  estado.emergencia = false;
  digitalWrite(LED_INDICADOR, LOW);
  
  parar(); // Garantir que está parado
}

void inicializarPWM() {
  // Configurar PWM para os motores
  ledcSetup(CANAL_DIR1, FREQUENCIA_PWM, RESOLUCAO_PWM);
  ledcSetup(CANAL_DIR2, FREQUENCIA_PWM, RESOLUCAO_PWM);
  ledcSetup(CANAL_ESQ1, FREQUENCIA_PWM, RESOLUCAO_PWM);
  ledcSetup(CANAL_ESQ2, FREQUENCIA_PWM, RESOLUCAO_PWM);
  
  ledcAttachPin(MOTOR_DIR1, CANAL_DIR1);
  ledcAttachPin(MOTOR_DIR2, CANAL_DIR2);
  ledcAttachPin(MOTOR_ESQ1, CANAL_ESQ1);
  ledcAttachPin(MOTOR_ESQ2, CANAL_ESQ2);
}

void verificarEntradas() {
  // Verificar sensor de proximidade
  estado.obstaculoDetectado = verificarObstaculo();
  
  // Verificar botão de emergência
  estado.emergencia = (digitalRead(BOTAO_EMERGENCIA) == LOW);
  
  // Acionar LED se obstáculo detectado ou emergência
  digitalWrite(LED_INDICADOR, estado.obstaculoDetectado || estado.emergencia);
}

void processarGPS() {
  // Processar dados do GPS
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        estado.latitude = gps.location.lat();
        estado.longitude = gps.location.lng();
        estado.satellites = gps.satellites.value();
        estado.precisao = gps.hdop.value();
        estado.gpsValido = true;
        
        // Exibir posição atual periodicamente
        static unsigned long ultimaExibicao = 0;
        if (millis() - ultimaExibicao > 5000) {
          ultimaExibicao = millis();
          Serial.print("Posição: ");
          Serial.print(estado.latitude, 6);
          Serial.print(", ");
          Serial.print(estado.longitude, 6);
          Serial.print(" | Satélites: ");
          Serial.print(estado.satellites);
          Serial.print(" | Precisão: ");
          Serial.println(estado.precisao);
        }
      } else {
        estado.gpsValido = false;
        Serial.println("Aguardando sinal GPS...");
      }
    }
  }
  
  // Se não receber dados GPS por 2 segundos, considerar inválido
  static unsigned long ultimaLeituraGPS = 0;
  if (millis() - ultimaLeituraGPS > 2000) {
    estado.gpsValido = false;
    ultimaLeituraGPS = millis();
  }
}

void navegarParaWaypoint() {
  // Verificar se waypoint atual foi atingido
  if (waypointAtingido(estado.latitude, estado.longitude, estado.waypointAtual)) {
    Serial.print("Waypoint ");
    Serial.print(estado.waypointAtual + 1);
    Serial.println(" atingido!");
    SerialBT.print("Waypoint ");
    SerialBT.print(estado.waypointAtual + 1);
    SerialBT.println(" atingido!");
    
    // Avançar para o próximo waypoint
    estado.waypointAtual++;
    
    if (estado.waypointAtual >= NUM_WAYPOINTS) {
      Serial.println("Todos os waypoints foram atingidos! Parando...");
      SerialBT.println("Todos os waypoints foram atingidos! Parando...");
      parar();
      estado.emMovimento = false;
      while(true) { 
        delay(1000); 
        // Piscar LED indicando conclusão
        digitalWrite(LED_INDICADOR, !digitalRead(LED_INDICADOR));
      } // Fim da missão
    } else {
      Serial.print("Navegando para Waypoint ");
      Serial.println(estado.waypointAtual + 1);
      SerialBT.print("Navegando para Waypoint ");
      SerialBT.println(estado.waypointAtual + 1);
    }
  }
  
  // Calcular direção para o waypoint
  calcularNavegacao();
}

void calcularNavegacao() {
  if (!estado.gpsValido) {
    parar();
    return;
  }
  
  double destinoLat = WAYPOINTS[estado.waypointAtual][0];
  double destinoLon = WAYPOINTS[estado.waypointAtual][1];
  
  // Calcular distância para o waypoint
  double distancia = calcularDistancia(estado.latitude, estado.longitude, 
                                      destinoLat, destinoLon);
  
  // Reduzir velocidade quando estiver próximo do waypoint
  int velocidade = (distancia < 15.0) ? VELOCIDADE_LENTA : VELOCIDADE_NORMAL;
  
  // Cálculo de direção baseado em coordenadas
  double deltaLon = destinoLon - estado.longitude;
  double deltaLat = destinoLat - estado.latitude;
  
  if (distancia > RAIO_ATINGIDO) {
    // Determinar direção baseado na diferença de coordenadas
    if (abs(deltaLon) > abs(deltaLat)) {
      if (deltaLon > 0.0001) {  // Waypoint está a leste
        virarDireita();
      } else if (deltaLon < -0.0001) {  // Waypoint está a oeste
        virarEsquerda();
      } else {
        moverFrente(velocidade);
      }
    } else {
      if (deltaLat > 0.0001) {  // Waypoint está ao norte
        moverFrente(velocidade);
      } else if (deltaLat < -0.0001) {  // Waypoint está ao sul
        moverTras(velocidade);
      } else {
        moverFrente(velocidade);
      }
    }
    
    estado.emMovimento = true;
    
    // Debug a cada 2 segundos
    static unsigned long ultimoDebug = 0;
    if (millis() - ultimoDebug > 2000) {
      ultimoDebug = millis();
      Serial.print("Distância para waypoint: ");
      Serial.print(distancia);
      Serial.println(" metros");
    }
  }
}

// Funções de controle do motor com PWM do ESP32
void moverFrente(int velocidade) {
  ledcWrite(CANAL_DIR1, velocidade);
  ledcWrite(CANAL_DIR2, 0);
  ledcWrite(CANAL_ESQ1, velocidade);
  ledcWrite(CANAL_ESQ2, 0);
}

void moverTras(int velocidade) {
  ledcWrite(CANAL_DIR1, 0);
  ledcWrite(CANAL_DIR2, velocidade);
  ledcWrite(CANAL_ESQ1, 0);
  ledcWrite(CANAL_ESQ2, velocidade);
}

void parar() {
  ledcWrite(CANAL_DIR1, 0);
  ledcWrite(CANAL_DIR2, 0);
  ledcWrite(CANAL_ESQ1, 0);
  ledcWrite(CANAL_ESQ2, 0);
  estado.emMovimento = false;
}

void virarDireita(int intensidade) {
  int reducao = (VELOCIDADE_NORMAL * intensidade) / 100;
  ledcWrite(CANAL_DIR1, VELOCIDADE_NORMAL - reducao);
  ledcWrite(CANAL_DIR2, 0);
  ledcWrite(CANAL_ESQ1, VELOCIDADE_NORMAL);
  ledcWrite(CANAL_ESQ2, 0);
}

void virarEsquerda(int intensidade) {
  int reducao = (VELOCIDADE_NORMAL * intensidade) / 100;
  ledcWrite(CANAL_DIR1, VELOCIDADE_NORMAL);
  ledcWrite(CANAL_DIR2, 0);
  ledcWrite(CANAL_ESQ1, VELOCIDADE_NORMAL - reducao);
  ledcWrite(CANAL_ESQ2, 0);
}

// Funções do sensor de proximidade
int lerSensorProximidade() {
  int valorAnalogico = analogRead(SENSOR_PROX);
  // Conversão do valor analógico para distância em centímetros
  // Ajustar conforme o sensor específico utilizado
  int distancia = (6787 / (valorAnalogico - 3)) - 4;
  
  return distancia;
}

bool verificarObstaculo() {
  int distancia = lerSensorProximidade();
  if (distancia <= DISTANCIA_PARADA && distancia > 0) {
    Serial.print("Obstáculo a ");
    Serial.print(distancia);
    Serial.println(" cm");
    return true;
  }
  return false;
}

// Cálculo de distância entre coordenadas GPS (fórmula de Haversine)
double calcularDistancia(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000; // Raio da Terra em metros
  double phi1 = lat1 * DEG_TO_RAD;
  double phi2 = lat2 * DEG_TO_RAD;
  double deltaPhi = (lat2 - lat1) * DEG_TO_RAD;
  double deltaLambda = (lon2 - lon1) * DEG_TO_RAD;
  
  double a = sin(deltaPhi/2) * sin(deltaPhi/2) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda/2) * sin(deltaLambda/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return R * c;
}

bool waypointAtingido(double lat, double lon, int waypointIndex) {
  double destinoLat = WAYPOINTS[waypointIndex][0];
  double destinoLon = WAYPOINTS[waypointIndex][1];
  double distancia = calcularDistancia(lat, lon, destinoLat, destinoLon);
  
  return (distancia <= RAIO_ATINGIDO);
}

void tratarEmergencia() {
  Serial.println("EMERGÊNCIA! Parando imediatamente!");
  SerialBT.println("EMERGÊNCIA! Parando imediatamente!");
  parar();
  digitalWrite(LED_INDICADOR, HIGH);
  
  // Piscar LED rapidamente durante emergência
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_INDICADOR, !digitalRead(LED_INDICADOR));
    delay(200);
  }
  
  // Aguardar reset do botão de emergência
  while (digitalRead(BOTAO_EMERGENCIA) == LOW) {
    digitalWrite(LED_INDICADOR, !digitalRead(LED_INDICADOR));
    delay(200);
  }
  
  estado.emergencia = false;
  digitalWrite(LED_INDICADOR, LOW);
  Serial.println("Emergência resolvida. Retomando operação.");
  SerialBT.println("Emergência resolvida. Retomando operação.");
}

// Tarefa que roda no segundo núcleo para comunicação
void tarefaComunicacao(void *parameter) {
  while (true) {
    enviarDadosBluetooth();
    delay(1000); // Enviar dados a cada segundo
  }
}

void enviarDadosBluetooth() {
  if (SerialBT.hasClient()) {
    SerialBT.print("WP:");
    SerialBT.print(estado.waypointAtual + 1);
    SerialBT.print("/");
    SerialBT.print(NUM_WAYPOINTS);
    SerialBT.print("|Lat:");
    SerialBT.print(estado.latitude, 6);
    SerialBT.print("|Lon:");
    SerialBT.print(estado.longitude, 6);
    SerialBT.print("|Sat:");
    SerialBT.print(estado.satellites);
    SerialBT.print("|Mov:");
    SerialBT.print(estado.emMovimento ? "SIM" : "NAO");
    SerialBT.print("|Obs:");
    SerialBT.println(estado.obstaculoDetectado ? "SIM" : "NAO");
  }
}
