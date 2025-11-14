/***** ESP32 + 4x HC-SR04 + SSD1306 + MQTT (JSON + tópicos split) *****/
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ======= Wi-Fi / MQTT ======= */
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* MQTT_HOST = "broker.hivemq.com"; 
// Caso não conecte, altere o MQTT_HOST para um desses abaixo:
/*broker.hivemq.com*/
/*broker.emqx.io*/
/*mqtt.eclipseprojects.io*/
/*test.mosquitto.org*/
const uint16_t MQTT_PORT = 1883;
const char* DEVICE_ID = "cane-esp32-bengala-tech"; // <-- mude para um ID único

WiFiClient net;
PubSubClient mqtt(net);
String tp(const char* leaf){ String s="cane/"; s+=DEVICE_ID; s+="/"; s+=leaf; return s; }

/* ======= OLED I2C ======= */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET   -1
#define OLED_ADDR    0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ======= COMEÇO DO MAPA NA TELA OLED ======= */
void renderMapTiny(Adafruit_SSD1306 &display, long L, long F, long R, long D){
  // helpers locais
  auto arrowsFor = [](long cm)->uint8_t { return (cm>300)?3 : (cm>=150)?2 : 1; };
  auto triR=[&](int x,int y,int s){ display.fillTriangle(x,y, x-s,y-s, x-s,y+s, SSD1306_WHITE); };
  auto triL=[&](int x,int y,int s){ display.fillTriangle(x,y, x+s,y-s, x+s,y+s, SSD1306_WHITE); };
  auto triU=[&](int x,int y,int s){ display.fillTriangle(x,y, x-s,y+s, x+s,y+s, SSD1306_WHITE); };
  auto triD=[&](int x,int y,int s){ display.fillTriangle(x,y, x-s,y-s, x+s,y-s, SSD1306_WHITE); };
  auto textW=[&](const String& s){ int16_t x1,y1; uint16_t w,h; display.getTextBounds(s,0,0,&x1,&y1,&w,&h); return (int)w; };

  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // título
  const char* ttl = "Bengala Tech";
  int tw = textW(ttl);
  display.setCursor((128 - tw)/2, 0);
  display.print(ttl);
  display.drawLine(2, 10, 126, 10, SSD1306_WHITE);

  // quadro grande
  const int boxX = 8;
  const int boxY = 14;
  const int boxW = 62;
  const int boxH = 46;
  display.drawRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);

  // centro = quadrado pequeno
  const int cx = boxX + boxW/2;
  const int cy = boxY + boxH/2;
  const int userSize = 4;
  display.fillRect(cx - userSize/2, cy - userSize/2, userSize, userSize, SSD1306_WHITE);

  // parâmetros das setas
  const int triS = 3;
  const int edge = 4;
  const int gapH = 7;   // distância do centro p/ 1ª seta horizontal (E/D)
  const int gapV = 3;  // distância do centro p/ 1ª seta vertical   (Fr/Bx)

  // ---- DIREITA ----
  {
    int a = cx + userSize/2 + gapH;
    int b = boxX + boxW - edge;
    int p1 = a + (b - a) * 1 / 4;
    int p2 = a + (b - a) * 2 / 4;
    int p3 = a + (b - a) * 3 / 4;
    uint8_t n = arrowsFor(R);
    if (n >= 1) triR(p1, cy, triS);
    if (n >= 2) triR(p2, cy, triS);
    if (n >= 3) triR(p3, cy, triS);
  }

  // ---- ESQUERDA ----
  {
    int a = boxX + edge;
    int b = cx - userSize/2 - gapH;
    int p1 = a + (b - a) * 1 / 4;   // perto da borda
    int p2 = a + (b - a) * 2 / 4;
    int p3 = a + (b - a) * 3 / 4;   // perto do centro
    uint8_t n = arrowsFor(L);
    if (n >= 1) triL(p3, cy, triS); // começa perto do centro
    if (n >= 2) triL(p2, cy, triS);
    if (n >= 3) triL(p1, cy, triS);
  }

  // ---- CIMA (FRENTE) ----
  {
    int a = boxY + edge;
    int b = cy - userSize/2 - gapV;
    int p1 = a + (b - a) * 1 / 4;   // perto da borda
    int p2 = a + (b - a) * 2 / 4;
    int p3 = a + (b - a) * 3 / 4;   // perto do centro
    uint8_t n = arrowsFor(F);
    if (n >= 1) triU(cx, p3, triS);
    if (n >= 2) triU(cx, p2, triS);
    if (n >= 3) triU(cx, p1, triS);
  }

  // ---- BAIXO (CHÃO) ----
  {
    int a = cy + userSize/2 + gapV;
    int b = boxY + boxH - edge;
    int p1 = a + (b - a) * 1 / 4;   // perto do centro
    int p2 = a + (b - a) * 2 / 4;
    int p3 = a + (b - a) * 3 / 4;   // perto da borda
    uint8_t n = arrowsFor(D);
    if (n >= 1) triD(cx, p1, triS);
    if (n >= 2) triD(cx, p2, triS);
    if (n >= 3) triD(cx, p3, triS);
  }

  // textos à direita
  const int colRight = boxX + boxW + 6;
  const int baseY = 16;   // primeira linha
  const int step  = 12;   // distância entre linhas

  display.setCursor(colRight, baseY + step*0); display.print("Fr:"); display.print((int)F);
  display.setCursor(colRight, baseY + step*1); display.print("Dr:"); display.print((int)R);
  display.setCursor(colRight, baseY + step*2); display.print("Bx:"); display.print((int)D);
  display.setCursor(colRight, baseY + step*3); display.print("Es:"); display.print((int)L);

  display.display();
}

/* ======= FIM NOVO MAPA ======= */

/* ======= Pinos ======= */
#define I2C_SDA 21
#define I2C_SCL 22
struct USPins { uint8_t trig, echo; const char* name; };
USPins S_left  = {25, 34, "left"};
USPins S_front = {26, 35, "front"};
USPins S_right = {27, 33, "right"}; // usamos 33
USPins S_down  = {14, 32, "down"};  // TRIG 14, ECHO 32

/* ======= Leitura rápida ======= */
#define FAST_MODE 1
#if FAST_MODE
  const unsigned long PULSE_TIMEOUT_US = 24000;
  const uint8_t  INTER_SENSOR_DELAY_MS = 6;
  const uint8_t  FRAME_DELAY_MS        = 0;
  const int      SNAP_DELTA_CM         = 20;
  const uint8_t  EMA_NUM               = 3; 
  const uint8_t  EMA_DEN               = 4;
#else
  const unsigned long PULSE_TIMEOUT_US = 30000;
  const uint8_t  INTER_SENSOR_DELAY_MS = 30;
  const uint8_t  FRAME_DELAY_MS        = 60;
  const int      SNAP_DELTA_CM         = 9999;
  const uint8_t  EMA_NUM               = 7;
  const uint8_t  EMA_DEN               = 8;
#endif

/* ======= Bandas ======= */
#define BAND1_MAX_CM 150
#define BAND2_MAX_CM 300
#define MAX_ECHO_CM  400

/* ======= Estado ======= */
long dL=0,dF=0,dR=0,dD=0;   // brutas
long sL=0,sF=0,sR=0,sD=0;   // suavizadas
int  lastBandL=0,lastBandF=0,lastBandR=0;
long lastDown=0;

/* ======= Utils ======= */
int band3(long cm){ if(cm<0) return 0; if(cm<=BAND1_MAX_CM) return 1; if(cm<=BAND2_MAX_CM) return 2; return 3; }
long readUltrasonicCM(uint8_t trig,uint8_t echo){
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10);
  digitalWrite(trig,LOW);
  long dur = pulseIn(echo,HIGH,PULSE_TIMEOUT_US);
  if(dur==0) return -1;
  long cm = dur/58;
  if(cm<0) cm=-1;
  if(cm>MAX_ECHO_CM) cm=MAX_ECHO_CM;
  return cm;
}
long smoothFast(long prev,long cur){
  if(cur<0) return (prev*EMA_NUM)/EMA_DEN;
  if(prev==0) return cur;
  long delta = labs(cur-prev);
  if(delta>=SNAP_DELTA_CM) return cur;
  return (prev*EMA_NUM + cur)/EMA_DEN;
}

/* ======= MQTT: helpers ======= */
void ensureWiFi(){
  if(WiFi.status()==WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status()!=WL_CONNECTED) delay(250);
}
void publishStatusOnline(){ mqtt.publish(tp("status").c_str(),"online",true); } 
void publishState(){
  char buf[128];
  snprintf(buf,sizeof(buf), "{\"fw\":\"1.0.0\",\"rssi_dbm\":%d}", WiFi.RSSI());
  mqtt.publish(tp("state").c_str(), buf, true);
}
void publishHeartbeat(){ mqtt.publish(tp("heartbeat").c_str(), "1"); }
void ensureMQTT(){
  while(!mqtt.connected()){
    if(mqtt.connect(DEVICE_ID, tp("status").c_str(), 1, true, "offline")){
      publishStatusOnline();
      publishState();
      
    } else delay(1000);
  }
}
void publishTelemetry(){
  char buf[96];
  auto bchar=[](long v){int b=band3(v); return b==1?'n':(b==2?'m':'f');};
  snprintf(buf,sizeof(buf), "{\"L\":\"%c\",\"F\":\"%c\",\"R\":\"%c\",\"down\":%ld}",
           bchar(sL), bchar(sF), bchar(sR), sD<0?0:sD);
  mqtt.publish(tp("telemetry").c_str(), buf);
}
/* ======= AUX: converte cm -> 'n'/'m'/'f'/'x' ======= */
char bandChar(long v){
  if (v < 0) return 'x';          // inválido/sem leitura
  int b = band3(v);               // 1=near, 2=mid, 3=far
  if (b == 1) return 'n';
  if (b == 2) return 'm';
  return 'f';
}
/* >>> TÓPICOS "SPLIT" PARA O APP (Gauge/Multi-State) <<< */
void publishTelemetrySplit() {
  char s[2] = {0, 0};
  s[0] = bandChar(sL); mqtt.publish(tp("telemetry/L").c_str(), s);
  s[0] = bandChar(sF); mqtt.publish(tp("telemetry/F").c_str(), s);
  s[0] = bandChar(sR); mqtt.publish(tp("telemetry/R").c_str(), s);
  char buf[12];
  snprintf(buf, sizeof(buf), "%ld", (sD < 0 ? 0 : sD));
  mqtt.publish(tp("telemetry/down").c_str(), buf);
}
void publishHazard(const char* dir,const char* type,long dist_cm){
  char buf[96];
  snprintf(buf,sizeof(buf), "{\"dir\":\"%s\",\"type\":\"%s\",\"dist_cm\":%ld}",
           dir,type, dist_cm<0?0:dist_cm);
  mqtt.publish(tp("event/hazard").c_str(), buf);
}
void publishStep(long down_cm,long delta_cm){
  char buf[96];
  const char* kind = (delta_cm>=0) ? "down" : "up";
  snprintf(buf,sizeof(buf), "{\"kind\":\"%s\",\"down_cm\":%ld,\"delta_cm\":%ld}",
           kind, down_cm<0?0:down_cm, delta_cm);
  mqtt.publish(tp("event/step").c_str(), buf);
}

/* ======= Setup / Loop ======= */
void setup(){
  // I2C + OLED
  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay(); display.display();

  // Sensores
  pinMode(S_left.trig,OUTPUT);  pinMode(S_left.echo,INPUT);
  pinMode(S_front.trig,OUTPUT); pinMode(S_front.echo,INPUT);
  pinMode(S_right.trig,OUTPUT); pinMode(S_right.echo,INPUT);
  pinMode(S_down.trig,OUTPUT);  pinMode(S_down.echo,INPUT);
  digitalWrite(S_left.trig,LOW); digitalWrite(S_front.trig,LOW);
  digitalWrite(S_right.trig,LOW); digitalWrite(S_down.trig,LOW);

  // Wi-Fi + MQTT
  ensureWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  ensureMQTT();

  // Splash
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.print("Inicializando...");
  display.display(); delay(250);
}

unsigned long tRead=0, tTelem=0, tHB=0;
const unsigned long READ_MS=110;      // ciclo de sensores
const unsigned long TELEMETRY_MS=1000;
const unsigned long HEARTBEAT_MS=30000;

void loop(){
  ensureWiFi();
  ensureMQTT();
  mqtt.loop();

  unsigned long now = millis();

  // Leitura periódica dos sensores
  if (now - tRead >= READ_MS) {
    tRead = now;

    // Leituras sequenciais (reduz crosstalk entre HC-SR04)
    dL = readUltrasonicCM(S_left.trig,  S_left.echo);  delay(INTER_SENSOR_DELAY_MS);
    dF = readUltrasonicCM(S_front.trig, S_front.echo); delay(INTER_SENSOR_DELAY_MS);
    dR = readUltrasonicCM(S_right.trig, S_right.echo); delay(INTER_SENSOR_DELAY_MS);
    dD = readUltrasonicCM(S_down.trig,  S_down.echo);  delay(INTER_SENSOR_DELAY_MS);

    // Suavização (EMA rápida)
    sL = smoothFast(sL, dL);
    sF = smoothFast(sF, dF);
    sR = smoothFast(sR, dR);
    sD = smoothFast(sD, dD);

    // --- Display: mini-mapa + textos estilo do mock ---
    renderMapTiny(display, sL, sF, sR, sD);

    // Eventos quando entrar em "near" (banda 1)
    int bL = band3(sL), bF = band3(sF), bR = band3(sR);
    if (bL == 1 && lastBandL != 1) publishHazard("left",  "obstacle", sL);
    if (bF == 1 && lastBandF != 1) publishHazard("front", "obstacle", sF);
    if (bR == 1 && lastBandR != 1) publishHazard("right", "obstacle", sR);
    lastBandL = bL; lastBandF = bF; lastBandR = bR;

    // Degrau / desnível detectado por variação do sensor de chão
    long deltaDown = sD - lastDown;
    if (labs(deltaDown) >= 15) {                      // limiar de 15 cm
      publishStep(sD < 0 ? 0 : sD, deltaDown);        // MQTT: event/step
      if (sD >= 0) lastDown = sD;
    }
  }

  // Telemetria (JSON agregado + tópicos split p/ dashboards)
  if (now - tTelem >= TELEMETRY_MS) {
    tTelem = now;
    publishTelemetry();
    publishTelemetrySplit();
  }

  // Heartbeat periódico
  if (now - tHB >= HEARTBEAT_MS) {
    tHB = now;
    publishHeartbeat();
  }

  if (FRAME_DELAY_MS) delay(FRAME_DELAY_MS);
}