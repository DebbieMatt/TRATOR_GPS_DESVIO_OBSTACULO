📋 ESTRUTURA COMPLETA DE LIGAÇÕES - ESP32

ALIMENTAÇÃO (BASE)

```
ESP32 GND → Protoboard GND
ESP32 3.3V → Protoboard 3.3V (para sensores GPS e MPU)
ESP32 VIN → Para sensor Ultrassônico e cartão SD
Bateria 7.4V -> Ponte H L298n
```

1. PONTE H L298N (CONTROLE DE MOTORES DC)

```
// Motores Dianteiros
OUT1 (Motor Front) → GPIO 25
OUT2 (Motor Front) → GPIO 33
ENA (Enable Front) → GPIO 26

// Motores Traseiros  
OUT3 (Motor Rear) → GPIO 32
OUT4 (Motor Rear) → GPIO 35
ENB (Enable Rear) → GPIO 27
```

2. SENSOR ULTRASSÔNICO (HC-SR04)

```
TRIG → GPIO 4
ECHO → GPIO 2
VCC  → 5V
GND  → GND
```

3. GPS NEO-8M

```
GPS TX → ESP32 RX2 (GPIO 16)
GPS RX → ESP32 TX2 (GPIO 17)
GPS VCC → 3.3V
GPS GND → GND
```

4. MPU-6050 (ACELERÔMETRO/GIROSCÓPIO)

```
SCL → GPIO 22 (SCL)
SDA → GPIO 21 (SDA)
VCC → 3.3V
GND → GND
```

5. MÓDULO CARTÃO SD

```
CS   → GPIO 5
MOSI → GPIO 23
MISO → GPIO 19  
SCK  → GPIO 18
VCC  → 5V
GND  → GND
```

📊 RESUMO DE PINAGEM ESP32

Componente Pinos ESP32 Utilizados Função
Ponte H L298N 25, 33, 26, 32, 35, 27 Controle de motores
Sensor Ultrassônico 4, 2 Detecção obstáculos
GPS NEO-8M 16, 17 Navegação
MPU-6050 22, 21 Orientação
Cartão SD 5, 23, 19, 18 Log de dados

Esta configuração mantém todos os componentes essenciais para um carro autônomo com GPS e desvio de obstáculos! 🚗💨
