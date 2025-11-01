# Trator Autônomo com Navegação por GPS e Detecção de Obstáculos em Tempo Real

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.md)
[![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D?logo=arduino)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/ESP32-Platform-red)](https://www.espressif.com/)

## 📋 Sobre o Projeto

Este projeto foi desenvolvido como Trabalho de Conclusão de Curso (TCC) e consiste em um protótipo de **trator autônomo** equipado com sistema de navegação por GPS e detecção de obstáculos em tempo real. O objetivo é criar uma solução de agricultura de precisão que permite a automação de tarefas agrícolas, aumentando a eficiência e reduzindo custos operacionais.

### 🎯 Objetivos

- Implementar navegação autônoma utilizando coordenadas GPS
- Detectar e desviar de obstáculos em tempo real
- Registrar dados de trajeto e operações em cartão microSD
- Permitir controle e monitoramento via Bluetooth
- Demonstrar viabilidade de automação agrícola de baixo custo

## ✨ Funcionalidades

- 🛰️ **Navegação GPS**: Sistema de posicionamento global para trajetórias precisas
- 🚧 **Detecção de Obstáculos**: Sensores ultrassônicos para evitar colisões
- 📊 **Registro de Dados**: Armazenamento de coordenadas e eventos em cartão SD
- 📱 **Conectividade Bluetooth**: Comunicação sem fio para monitoramento e controle
- ⚙️ **Estabilização**: Sensor MPU6050 para controle de inclinação e orientação
- 🔋 **Controle de Motores**: Driver L298N para acionamento dos motores DC

## 🛠️ Componentes Utilizados

### Hardware Principal
- **ESP32**: Microcontrolador principal
- **NEO-6M GPS Module**: Módulo de GPS
- **HC-SR04**: Sensor ultrassônico de distância
- **MPU6050**: Sensor giroscópio e acelerômetro
- **L298N**: Driver para motores DC
- **Módulo MicroSD**: Armazenamento de dados
- **Módulo Bluetooth**: Comunicação sem fio

## 📚 Bibliotecas e Dependências

Certifique-se de instalar as versões específicas das bibliotecas para garantir a compatibilidade:

| Biblioteca | Versão | Repositório |
|------------|--------|-------------|
| TinyGPS++ | 3.3.0 | [TinyGPS](https://github.com/mikalhart/TinyGPSPlus) |
| Adafruit_MPU6050 | 2.2.6 | [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050) |
| Adafruit_Sensor | 1.1.15 | [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor) |
| Wire | 4.1.0 | [Wire Library](https://github.com/PaulStoffregen/Wire) |
| BluetoothSerial | 1.1.0 | [Bluetooth Serial](https://github.com/espressif/arduino-esp32) |
| FS | 2.1.3 | [File System](https://github.com/espressif/arduino-esp32) |
| SD | 1.3.0 | [SD Library](https://github.com/arduino-libraries/SD) |
| SPI | 3.0.0 | [SPI Library](https://github.com/PaulStoffregen/SPI) |

## 🚀 Instalação e Configuração

### Pré-requisitos

1. **Arduino IDE** - [Download](https://www.arduino.cc/en/software)
2. **Visual Studio** (opcional) - [Download](https://visualstudio.microsoft.com/pt-br/)
3. **Java Runtime Environment** - [Download](https://www.java.com/pt-BR/download/)
4. **Drivers ESP32** - Disponíveis na IDE do Arduino

### Passos de Instalação

1. Clone este repositório:
```bash
git clone https://github.com/seu-usuario/trator-autonomo-gps.git
cd trator-autonomo-gps
```

2. Instale o suporte para ESP32 na Arduino IDE:
   - Abra Arduino IDE → Preferências
   - Adicione a URL: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Vá em Ferramentas → Placa → Gerenciador de Placas
   - Busque por "ESP32" e instale

3. Instale as bibliotecas necessárias:
   - Abra Arduino IDE → Sketch → Incluir Biblioteca → Gerenciar Bibliotecas
   - Instale cada biblioteca listada na tabela acima com suas respectivas versões

4. Abra o arquivo principal do projeto na Arduino IDE

5. Selecione a placa ESP32 em Ferramentas → Placa

6. Configure a porta serial correta em Ferramentas → Porta

7. Compile e faça o upload do código

## 📐 Esquemáticos e Diagramas

O projeto inclui esquemáticos desenvolvidos em diferentes ferramentas:

- **Circuito.io**: [Visualizar Esquemático](https://www.circuito.io/app?components=513,11028,13959,360217,975601,1671987,7654321)
- **EasyEDA**: [Ver PDF](https://github.com/DebbieMatt/TRATOR_GPS_DESVIO_OBSTACULO/blob/acbd88e0e9cd93e092cadc461c549b807c64c717/Schematic_carro-gps_2025-09-10.pdf)
- **Cirkit Designer IDE**: [Visualizar simulação]()
- **Proteus**: [Simulação do circuito](https://github.com/DebbieMatt/TRATOR_GPS_DESVIO_OBSTACULO/blob/0fc588080093bac535e943cc864872d5c4b39cab/Carro_GPS.pdsprj)

## 📖 Referências e Inspirações

Este projeto foi desenvolvido com base em diversos tutoriais e projetos relacionados:

- [ESP32 with NEO-6M GPS Module (Arduino IDE)](https://randomnerdtutorials.com/esp32-neo-6m-gps-module-arduino/)
- [HC-SR04 com ESP32 - Curso ESP32 básico](https://portal.vidadesilicio.com.br/hc-sr04-com-esp32/)
- [How To Make A DIY Arduino Obstacle Avoiding Car](https://youtu.be/1n_KjpMfVT0)
- [ESP32 RC Car With Robotic Arm](https://www.hackster.io/pius4109/esp32-rc-car-with-robotic-arm-92a909)
- [Projeto Arduino GPS 6M](https://www.usinainfo.com.br/blog/projeto-arduino-gps-6m-registrando-localizacao/)
- [ESP32: Guide for MicroSD Card Module](https://randomnerdtutorials.com/esp32-microsd-card-arduino/)
- [Interface L298N DC Motor Driver](https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/)

## 🔧 Estrutura do Projeto

```
trator-autonomo-gps/
├── src/
│   ├── main.cpp              # Código principal
│   ├── gps_navigation.h      # Módulo de navegação GPS
│   ├── obstacle_detection.h  # Módulo de detecção de obstáculos
│   └── motor_control.h       # Módulo de controle de motores
├── schematics/
│   ├── circuito.io/
│   ├── easyeda/
│   └── proteus/
├── docs/
│   ├── manual_usuario.pdf
│   └── relatorio_tcc.pdf
├── LICENSE.md
└── README.md
```

## 🎓 Trabalho Acadêmico

Este projeto foi desenvolvido como Trabalho de Conclusão de Curso (TCC) em [Nome do Curso] na [Nome da Instituição].

**Orientador(a)**: [Nome do Orientador]  
**Período**: [Ano/Semestre]

## 👥 Autora e Colaboradora

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/maria01eduarda" title="Autora Principal">
        <img src="https://avatars.githubusercontent.com/u/100963109?v=4" width="120px;" alt="Maria Eduarda"/><br>
        <sub>
          <b>Maria Eduarda</b><br>
          <i>Autora Principal</i><br>
          Desenvolvimento do projeto e implementação
        </sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/DebbieMatt" title="Colaboradora">
        <img src="https://avatars.githubusercontent.com/u/112919058?v=4" width="120px;" alt="Débora Mateus"/><br>
        <sub>
          <b>Débora Mateus</b><br>
          <i>Colaboradora</i><br>
          Esquemáticos e documentação
        </sub>
      </a>
    </td>
  </tr>
</table>

## 🤝 Como Contribuir

Contribuições são bem-vindas! Se você deseja contribuir com este projeto:

1. Faça um Fork do projeto
2. Crie uma branch para sua feature (`git checkout -b feature/MinhaFeature`)
3. Commit suas mudanças (`git commit -m 'Adiciona MinhaFeature'`)
4. Push para a branch (`git push origin feature/MinhaFeature`)
5. Abra um Pull Request

## 📝 Licença

Este projeto está sob a licença MIT. Veja o arquivo [LICENSE.md](LICENSE.md) para mais detalhes.

## 📞 Contato

Para dúvidas ou sugestões sobre o projeto:

- **Maria Eduarda**: [GitHub](https://github.com/maria01eduarda)
- **Débora Mateus**: [GitHub](https://github.com/DebbieMatt)

---

<div align="center">
  <p>Desenvolvido com 💚 para inovação agrícola</p>
  <p>⭐ Se este projeto foi útil, considere deixar uma estrela!</p>
</div>
