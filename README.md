# bengala_tech
Protótipo para Bengala assistiva com ESP32, 4× HC-SR04, OLED SSD1306 e telemetria MQTT (obstáculos e desníveis).

## Hardware
- ESP32
- 4× HC-SR04
- OLED SSD1306 128x64

**Pinagem:**
- OLED: SDA=21, SCL=22, VCC=3V3, GND
- HC-SR04: L(TRIG25/ECHO34), F(TRIG26/ECHO35), R(TRIG27/ECHO33), Down(TRIG14/ECHO32)

## Firmware
- Arduino IDE 2.x (ou PlatformIO)
- Libs: PubSubClient, Adafruit SSD1306, Adafruit GFX, Adafruit BusIO

## MQTT
- Broker: `test.mosquitto.org:1883` (teste)
Obs: Se por algum motivo o Broker não funcionar, altere no código e no APP para os seguintes Brokers:
broker.hivemq.com, broker.emqx.io,mqtt.eclipseprojects.io, test.mosquitto.org
- Tópicos:
  - `cane/<ID>/status` (retain/LWT)
  - `cane/<ID>/telemetry` (JSON)
  - `cane/<ID>/telemetry/L|F|R` (`n|m|f|x`)
  - `cane/<ID>/telemetry/down` (cm)
  - `cane/<ID>/event/#` (hazard/step)

## Estrutura
firmware/ # código Arduino
images/ # fotos e prints
hardware/ # .fzz/.fzpz do Fritzing
dashboards/ # configurações do app MQTT

## Simulação
- Wokwi: https://wokwi.com/projects/442647087003864065
