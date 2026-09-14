// ============================================================
//  PiZaff DogTrack - Palmare GPS-LoRa
//  Copyright (C) 2026 Pietro Zaffarano
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see <https://www.gnu.org/licenses/>.
// ============================================================

// ============================================================
//  PALMARE GPS-LoRa  v1.5  (multi-cane, stile Garmin)
//  ESP32-S3 + SX1262 + NEO-M8N + QMC5883L + Sharp Memory 1.3"
// ============================================================

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <QMC5883LCompass.h>
#include <Preferences.h>
#define USE_BLE 0
#if USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

static const int8_t PIN_LORA_NSS  = 8;
static const int8_t PIN_LORA_SCK  = 9;
static const int8_t PIN_LORA_MOSI = 10;
static const int8_t PIN_LORA_MISO = 11;
static const int8_t PIN_LORA_RST  = 12;
static const int8_t PIN_LORA_BUSY = 13;
static const int8_t PIN_LORA_DIO1 = 14;
// ============================================================
//  PIN — differenze V3 / V4 (BOARD_V4 definito in platformio.ini)
//
//  Sulla V4 tre pin che usavamo hanno una funzione DEDICATA a bordo:
//   GPIO 7  = VFEM_Ctrl   -> controllo dell'amplificatore RF (il PA che
//                            porta l'uscita a 28 dBm). Usarlo come
//                            ingresso pulsante interferirebbe con la
//                            trasmissione ad alta potenza.
//   GPIO 34 = VGNSS_Ctrl  -> alimentazione del connettore GNSS
//   GPIO 21 = OLED_RST    -> reset dell'OLED integrato
//  Su V4 vengono quindi spostati su GPIO liberi (4, 16, 15).
//
//  Corretti e invariati su entrambe: LoRa 8-14, VBAT_Read=1,
//  ADC_Ctrl=37, Vext_Ctrl=36 (verificati sul pin layout ufficiale).
//
//  NOTA: 38/39/40/41/42 sulla V4 sono instradati anche al connettore
//  GNSS (SH1.25-8pin). Restano usabili come GPIO normali finche' quel
//  connettore non viene usato — che e' il nostro caso, il GPS lo
//  colleghiamo a filo.
// ============================================================
static const int8_t PIN_GPS_RX = 33;
#ifdef BOARD_V4
static const int8_t PIN_GPS_TX = 16;   // V4: 34 e' VGNSS_Ctrl
#else
static const int8_t PIN_GPS_TX = 34;
#endif
static const int8_t PIN_SDA = 41;
static const int8_t PIN_SCL = 42;
static const int8_t PIN_LCD_SCK = 38;
static const int8_t PIN_LCD_DI  = 39;
static const int8_t PIN_LCD_CS  = 40;
#ifdef BOARD_V4
static const int8_t PIN_LCD_DC  = 15;  // V4: 21 e' OLED_RST integrato
#else
static const int8_t PIN_LCD_DC  = 21;
#endif
static const int8_t PIN_LCD_RST = 47;
// GPIO5 confermato difettoso su questa scheda (test con ponticello e scambio
// dei fili: il segnale non arriva mai su GPIO5, sempre e solo su GPIO6,
// indipendentemente da quale filo dell'encoder ci sia collegato) - canale A
// spostato su GPIO2, verificato libero/funzionante.
#ifdef BOARD_V4
// V4: GPIO2 serve come CSD per l'amplificatore RF esterno (vedi PIN_PA_*
// sotto) - il canale A dell'encoder si sposta su GPIO18, libero su questa
// scheda. NON ANCORA VERIFICATO fisicamente: stesso test fatto per GPIO5
// (ponticello/scambio fili) va rifatto qui prima di fidarsi al 100%.
static const int8_t PIN_ENC_A  = 18;
#else
static const int8_t PIN_ENC_A  = 2;
#endif
static const int8_t PIN_ENC_B  = 6;
#ifdef BOARD_V4
static const int8_t PIN_ENC_SW = 4;    // V4: 7 e' VFEM_Ctrl (ampli RF!)
#else
static const int8_t PIN_ENC_SW = 7;
#endif
// GPIO 4 e 7 sono entrambi RTC-capable: l'accensione da deep sleep
// funziona su tutte e due le schede.
static const int8_t PIN_VBAT_ADC = 1;
static const int8_t PIN_ADC_CTRL = 37;
static const int8_t PIN_VEXT     = 36;

// Amplificatore RF esterno (solo V4): va acceso esplicitamente via software,
// altrimenti resta in "bypass mode" a potenza ridotta - stesso discorso gia'
// fatto e verificato sul collare. Scritta sulla scheda: "V4.3" -> front-end
// KCT8103L (non GC1109/V4.2). GPIO7=VFEM (alimentazione, sempre HIGH),
// GPIO2=CSD (chip enable, sempre HIGH, richiede lo spostamento di PIN_ENC_A
// sopra), GPIO5=CTX (LNA in RX / PA in TX, gestito da RadioLib con
// setRfSwitchPins - vedi applyFemFixV4()).
// GPIO5 e' lo stesso pin segnato "difettoso" mesi fa testando l'encoder
// (mai spostato oltre GPIO2 prima d'ora): non era guasto, era gia'
// collegato al front-end.
#ifdef BOARD_V4
static const int8_t PIN_PA_VFEM = 7;
static const int8_t PIN_PA_CSD  = 2;
static const int8_t PIN_PA_CTX  = 5;
#endif

static const float    LORA_FREQ    = 869.525f;
static const float    LORA_BW      = 125.0f;
static const uint8_t  LORA_SF      = 8;
static const uint8_t  LORA_CR      = 5;
static const uint8_t  LORA_SYNC    = 0xA7;
// Tensione TCXO passata a radio.begin(): stesso discorso e stessa fonte del
// collare (vedi commento gemello li') - 1.8V su V4, default RadioLib 1.6V
// su V3 (nessuna conferma per la V3, si resta al comportamento di sempre).
#ifdef BOARD_V4
static const float LORA_TCXO_V = 1.8f;
#else
static const float LORA_TCXO_V = 1.6f;
#endif
// POTENZA TX — impostata al MASSIMO erogabile dalla scheda.
//
// 22 = limite tecnico del chip SX1262 (RadioLib rifiuta valori superiori
// e radio.begin() fallisce). Sulla Heltec V4 l'amplificatore esterno
// porta l'uscita della scheda a ~28 dBm, il suo massimo dichiarato.
//
// COSA COMPORTA (ERP = uscita_scheda + guadagno_antenna - 2.15):
//   V4 + antenna 10 dBi  -> ~35.9 dBm ERP   (limite EU 27)
//   V4 + antenna FPC 2.5 -> ~28.4 dBm ERP   (limite EU 27)
// Per restare nei limiti EU: 12-13 con la 10 dBi, 19-20 con la FPC.
//
// ASSORBIMENTO: a piena potenza il PA della V4 tira parecchia corrente
// durante l'impulso TX. Se in trasmissione vedi riavvii casuali, e' un
// calo di tensione (batteria scarica, connettore/cavi sottili): scendi
// di qualche dBm o controlla l'alimentazione.
static const int8_t   LORA_PWR_DBM = 22;
static const uint16_t LORA_PREAMB  = 12;
static const uint16_t PKT_MAGIC    = 0xCA9E;

#pragma pack(push, 1)
struct CollarPacket {
  uint16_t magic;
  uint8_t  version;
  uint8_t  flags;      // bit0 fix, bit1 mov, bit2 batt scarica, bit3 FERMA, bit4 in carica
  uint32_t collar_id;
  int32_t  lat_e7;
  int32_t  lon_e7;
  uint16_t alt_m;
  uint16_t speed_x10;
  uint16_t course_deg;
  uint8_t  sats;
  uint8_t  hdop_x10;
  uint8_t  batt_pct;
  uint16_t batt_mv;
  uint16_t seq;
};
#pragma pack(pop)

// v2: aggiunto il campo p1 (parametro) e i comandi CMD_SET_SF/CMD_SET_PWR.
// Struct e versione DEVONO restare identiche byte-per-byte sul collare
// (vedi CmdPacket in collare/src/main.cpp).
static const uint16_t CMD_MAGIC = 0xACDC;
enum : uint8_t {
  CMD_ACK = 1, CMD_STANDBY = 2, CMD_WAKE = 3,
  CMD_SET_SF  = 4,   // p1 = nuovo SF (7-12), broadcast (collar_id=0)
  CMD_SET_PWR = 5,   // p1 = nuova potenza TX del collare in dBm, mirato
};
#pragma pack(push, 1)
struct CmdPacket {
  uint16_t magic;
  uint8_t  version;    // 2
  uint8_t  cmd;
  uint32_t collar_id;
  uint16_t seq;
  uint8_t  p1;         // parametro, dipende da cmd (0 se non usato)
};
#pragma pack(pop)

static const char* BLE_NAME         = "PalmareCane";
static const char* BLE_SERVICE_UUID = "7a0b1000-0001-4c01-8000-000000000001";
static const char* BLE_CHAR_UUID    = "7a0b1001-0001-4c01-8000-000000000001";

static const int MAX_DOGS     = 6;
static const int NAME_LEN     = 9;
static const int MAX_UNPAIRED = 6;

// Punti per cane: la Heltec V3 NON ha PSRAM (ESP32-S3FN8), la V4 ne ha 2MB.
// Rilevata a runtime: con PSRAM buffer generoso, senza PSRAM buffer ridotto
// in RAM interna (6 cani x 300 pt x 20 byte = 36 KB, sostenibile).
// Lo storico COMPLETO resta comunque su LittleFS (tracce GPX): questo
// buffer serve solo a disegnare la scia sulla mappa live.
static const int TRAIL_POINTS_PSRAM = 5000;
static const int TRAIL_POINTS_NOPSRAM = 300;
static int MAX_TRAIL_POINTS = TRAIL_POINTS_NOPSRAM;   // deciso in initTrails()
static bool trailUsesPsram = false;

#pragma pack(push, 1)
struct GpsPoint {
  int32_t  lat_e7;
  int32_t  lon_e7;
  uint16_t speed_x10;
  uint16_t alt_m;
  uint32_t timestamp;
  uint8_t  sats;
  uint8_t  batt_pct;
  uint16_t rssi;
};
#pragma pack(pop)

struct DogTrail {
  GpsPoint* points = nullptr;
  int idx = 0;
  int count = 0;
  uint32_t lastAddMs = 0;
};

static DogTrail trails[MAX_DOGS];

static bool initTrails();
static void addTrailPoint(int dogIdx, const CollarPacket &pkt, int16_t rssi);
static void clearTrail(int dogIdx);
static int getTrailCount(int dogIdx);
static GpsPoint* getTrailPoint(int dogIdx, int pointIdx);
static uint32_t gpsUnix();   // prototipo: usata da addTrailPoint, definita piu' sotto

SPIClass  loraSPI(HSPI);
SX1262    radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, loraSPI);
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, PIN_LCD_DI, PIN_LCD_SCK,
                          PIN_LCD_DC, PIN_LCD_RST, PIN_LCD_CS);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
QMC5883LCompass compass;
Preferences prefs;

#if USE_BLE
BLEServer*         bleServer = nullptr;
BLECharacteristic* bleChar   = nullptr;
#endif
volatile bool bleConnected   = false;

#define BLACK SSD1306_BLACK
#define WHITE SSD1306_WHITE

struct PairedDog {
  uint32_t id;
  char     name[NAME_LEN + 1];
};

struct DogLive {
  bool     everSeen = false;
  bool     fix = false;
  bool     moving = false;
  bool     lowBatt = false;
  bool     charging = false;
  bool     ferma = false;
  bool     fermaPrev = false;
  double   lat = 0, lon = 0;
  uint16_t alt = 0;
  float    speedKmh = 0;
  uint16_t course = 0;
  uint8_t  sats = 0;
  uint8_t  hdop_x10 = 0;   // qualita' del fix (HDOP*10); 0 = mai ricevuto
  uint8_t  battPct = 0;
  uint16_t battMv = 0;
  uint16_t seq = 0;
  int16_t  rssi = 0;
  float    snr = 0;
  uint32_t lastRxMs = 0;
};

static PairedDog dogs[MAX_DOGS];
static DogLive   live[MAX_DOGS];
static int       dogCount = 0;
static int       activeDog = 0;
static uint8_t   pendingCmd[MAX_DOGS] = {0};
static uint8_t   pendingP1[MAX_DOGS] = {0};   // parametro per pendingCmd (es. nuova potenza CMD_SET_PWR)

static bool initTrails() {
  trailUsesPsram = psramFound();
  MAX_TRAIL_POINTS = trailUsesPsram ? TRAIL_POINTS_PSRAM : TRAIL_POINTS_NOPSRAM;
  size_t bytes = (size_t)MAX_TRAIL_POINTS * sizeof(GpsPoint);

  Serial.printf("[TRAIL] PSRAM: %s -> %d punti/cane\n",
                trailUsesPsram ? "presente" : "assente (uso RAM interna)",
                MAX_TRAIL_POINTS);

  for (int i = 0; i < MAX_DOGS; i++) {
    trails[i].points = trailUsesPsram ? (GpsPoint*)ps_malloc(bytes)
                                      : (GpsPoint*)malloc(bytes);
    if (trails[i].points == nullptr) {
      // Fallimento reale: libera quel che ho preso e disattiva la funzione
      // in modo pulito invece di lasciare puntatori a meta'
      Serial.printf("[TRAIL] allocazione fallita al cane %d: trail disattivato\n", i);
      for (int k = 0; k < i; k++) { free(trails[k].points); trails[k].points = nullptr; }
      return false;
    }
    memset(trails[i].points, 0, bytes);
    trails[i].idx = 0;
    trails[i].count = 0;
    trails[i].lastAddMs = 0;
  }
  Serial.printf("[TRAIL] Buffer allocati: %d cani x %d punti = %.1f KB\n",
                MAX_DOGS, MAX_TRAIL_POINTS, (MAX_DOGS * bytes) / 1024.0f);
  return true;
}

static void addTrailPoint(int dogIdx, const CollarPacket &pkt, int16_t rssi) {
  if (dogIdx < 0 || dogIdx >= MAX_DOGS) return;
  if (trails[dogIdx].points == nullptr) return;
  if (millis() - trails[dogIdx].lastAddMs < 1000) return;
  trails[dogIdx].lastAddMs = millis();
  int pos = trails[dogIdx].idx;
  GpsPoint &pt = trails[dogIdx].points[pos];
  pt.lat_e7 = pkt.lat_e7;
  pt.lon_e7 = pkt.lon_e7;
  pt.speed_x10 = pkt.speed_x10;
  pt.alt_m = pkt.alt_m;
  pt.timestamp = gpsUnix();      // ora reale dal GPS, non secondi-da-boot
  pt.sats = pkt.sats;
  pt.batt_pct = pkt.batt_pct;
  pt.rssi = (uint16_t)rssi;
  trails[dogIdx].idx++;
  if (trails[dogIdx].idx >= MAX_TRAIL_POINTS) trails[dogIdx].idx = 0;
  if (trails[dogIdx].count < MAX_TRAIL_POINTS) trails[dogIdx].count++;
}

static void clearTrail(int dogIdx) {
  if (dogIdx < 0 || dogIdx >= MAX_DOGS) return;
  if (trails[dogIdx].points) {
    memset(trails[dogIdx].points, 0, MAX_TRAIL_POINTS * sizeof(GpsPoint));
    trails[dogIdx].idx = 0;
    trails[dogIdx].count = 0;
    trails[dogIdx].lastAddMs = 0;
  }
}

static int getTrailCount(int dogIdx) {
  if (dogIdx < 0 || dogIdx >= MAX_DOGS) return 0;
  return trails[dogIdx].count;
}

static GpsPoint* getTrailPoint(int dogIdx, int pointIdx) {
  if (dogIdx < 0 || dogIdx >= MAX_DOGS) return nullptr;
  if (pointIdx < 0 || pointIdx >= trails[dogIdx].count) return nullptr;
  int realIdx = (trails[dogIdx].idx - trails[dogIdx].count + pointIdx + MAX_TRAIL_POINTS) % MAX_TRAIL_POINTS;
  return &trails[dogIdx].points[realIdx];
}

struct UnpairedSeen {
  uint32_t id = 0;
  int16_t  rssi = 0;
  uint32_t lastMs = 0;
};
static UnpairedSeen unpaired[MAX_UNPAIRED];

struct Settings {
  bool  metric = true;
  float declination = 3.5f;
  int   calX = 0, calY = 0, calZ = 0;
  float calScaleX = 1.0f, calScaleY = 1.0f, calScaleZ = 1.0f;
  bool  calibrated = false;
  uint8_t sf  = LORA_SF;        // spreading factor attivo (persistito, vedi menu "SF:")
  int8_t  pwr = LORA_PWR_DBM;   // potenza TX del PALMARE stesso (indipendente dal collare)
} cfg;

enum Screen {
  SCR_HOME, SCR_MENU,
  SCR_PAIR_LIST,
  SCR_NAME_EDIT,
  SCR_DOG_LIST,
  SCR_DOG_ACTIONS,
  SCR_CONFIRM_DELETE,
  SCR_WIFI, SCR_TRACKS, SCR_TRACKS_DEL, SCR_INFO, SCR_CONFIRM_RESET,
  SCR_ADVANCED, SCR_ADVANCED_INFO,  // SF/potenza palmare: sottomenu a parte per non saturare il menu principale
  SCR_SF_PICKER,  // scelta diretta dell'SF (ruota per scegliere, click per confermare)
  SCR_COMPASS     // heading live (si aggiorna muovendosi), click = calibra
};
volatile Screen screen = SCR_HOME;

static const char* MENU_ITEMS[] = {
  "< HOME", "Associa", "Cani",
  "Mappa", "Bussola", "Unita:",
  "Tracce", "Info", "Reset", "Avanzate"
};
static const int MENU_N = 10;
static int menuIdx = 0;
static int advIdx = 0;   // selezione nel sottomenu "Avanzate" (SF/potenza palmare)
static int sfPickIdx = 0;   // selezione nel picker SF (0="< indietro", 1..6=SF 7..12)

static int      pairSelIdx = 0;
static uint32_t pairTargetId = 0;
static int      editDogIdx = -1;
static int      dogListIdx = 0;
static int      dogActionIdx = 0;
// Quando si cambia cane attivo in Home, mostra "x/y" al posto della quota
// per 3s (poi drawHome() torna da sola all'altitudine - vedi li').
static uint32_t homeFractionShowUntil = 0;
// saveDogs() scrive su NVS (flash): chiamarla ad ogni singolo scatto della
// rotella mentre si sfogliano i cani in Home la farebbe scattare ad ogni
// tick, con conseguente scatto/lag percepibile ruotando in fretta. Si segna
// solo che c'e' un salvataggio pending e lo si scarica dopo una breve pausa
// (vedi loop()), piu' comunque sempre allo spegnimento in powerOff().
static bool     dogsSaveDirty = false;
static uint32_t dogsSaveDirtyMs = 0;

static const char CHARSET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
static const int  CHARSET_N = sizeof(CHARSET) - 1;
static char editName[NAME_LEN + 1];
static int  editPos = 0;
static int  editChar = 1;

volatile int32_t encDelta = 0;
volatile bool btnPressed = false;
volatile bool btnLongPressed = false;
volatile uint32_t btnDownMs = 0;
volatile bool loraPacketFlag = false;
static void handleLoraPacket();   // prototipo: usata dagli handler /gpx e /api/trail piu' sotto

// ---- Encoder con tabella Gray ----
// Decoder a 4 stati (quadratura completa): molto piu' robusto del semplice
// "confronta A e B quando cambia A" di prima, che su alcuni moduli KY-040
// perde colpi o sbaglia direzione. Interrompe su ENTRAMBI i pin (A e B),
// non solo su A, e usa gli ultimi 2+2 bit (stato precedente + stato
// attuale) come indice nella tabella: le transizioni impossibili/rimbalzi
// valgono 0, quelle valide +-1.
//
// UN CLIC FISICO = 4 TRANSIZIONI ELETTRICHE (un giro completo di Gray:
// 00->01->11->10->00), quindi encDelta si muove a scatti di 4 per ogni
// click, non di 1 - il consumo in loop() qui sotto e' stato adattato di
// conseguenza (vedi ENC_STEP_UNIT).
volatile uint8_t encPrevAB = 0;
static const int8_t ENC_TAB[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};
void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t curAB = (a << 1) | b;
  uint8_t idx = (encPrevAB << 2) | curAB;
  encDelta += ENC_TAB[idx];
  encPrevAB = curAB;
}

void IRAM_ATTR buttonISR() {
  if (digitalRead(PIN_ENC_SW) == LOW) {
    btnDownMs = millis();
  } else {
    uint32_t held = millis() - btnDownMs;
    if (held > 30 && held < 700)  btnPressed = true;
    else if (held >= 700)         btnLongPressed = true;
  }
}

void IRAM_ATTR loraISR() { loraPacketFlag = true; }

#if USE_BLE
class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    s->getAdvertising()->start();
  }
};
#endif

static double haversineM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1), dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}
static float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1))*sin(radians(lat2)) -
             sin(radians(lat1))*cos(radians(lat2))*cos(dLon);
  float b = degrees(atan2(y, x));
  if (b < 0) b += 360.0f;
  return b;
}

static uint16_t readBatteryMv() {
  digitalWrite(PIN_ADC_CTRL, LOW); delay(8);
  // 12 campioni invece di 4, media troncata (si scartano il piu' alto e il
  // piu' basso prima di fare la media): una singola lettura rumorosa - es.
  // per un impulso di corrente durante una trasmissione LoRa proprio in
  // quel momento - pesava per intero sul risultato prima. Cosi' un paio di
  // campioni anomali non spostano la media, senza dover ritarare il
  // rapporto del partitore (che resta quello verificato col multimetro).
  const int N = 12;
  uint16_t samples[N];
  for (int i = 0; i < N; i++) { samples[i] = analogReadMilliVolts(PIN_VBAT_ADC); delay(2); }
  digitalWrite(PIN_ADC_CTRL, HIGH);
  for (int i = 1; i < N; i++) {
    uint16_t key = samples[i]; int j = i - 1;
    while (j >= 0 && samples[j] > key) { samples[j + 1] = samples[j]; j--; }
    samples[j + 1] = key;
  }
  uint32_t raw = 0;
  for (int i = 1; i < N - 1; i++) raw += samples[i];
  // Stesso rapporto usato sul collare (5.87, da multimetro 3.56V contro
  // rawMv=606 letto sul collare) applicato anche qui per scelta esplicita,
  // non riverificato con una misura separata sul palmare.
  return (uint16_t)((raw / (N - 2)) * 5.87f);
}
static uint8_t battPercent(uint16_t mv) {
  // Una LiPo 1S non si scarica in modo lineare: la tensione resta quasi
  // piatta per la maggior parte della carica e poi scende/sale ripida
  // vicino ai due estremi. Estremi corretti a 4.20V=100% (tensione massima
  // reale di carica di una LiPo 1S, non 3.90V come tarato inizialmente) e
  // 3.30V=0% (soglia di sicurezza per non scaricarla troppo).
  struct Pt { uint16_t mv; uint8_t pct; };
  static const Pt curve[] = {
    {3300, 0}, {3500, 10}, {3600, 20}, {3700, 40},
    {3800, 60}, {3900, 75}, {4000, 85}, {4100, 95}, {4200, 100},
  };
  const int n = sizeof(curve) / sizeof(curve[0]);
  if (mv <= curve[0].mv) return curve[0].pct;
  if (mv >= curve[n - 1].mv) return curve[n - 1].pct;
  for (int i = 1; i < n; i++) {
    if (mv <= curve[i].mv) {
      const Pt &a = curve[i - 1], &b = curve[i];
      return (uint8_t)(a.pct + (float)(mv - a.mv) * (b.pct - a.pct) / (b.mv - a.mv));
    }
  }
  return 100;
}
static float readHeadingDeg() {
  compass.read();
  // Verificato con una sequenza di ~30 campioni durante una rotazione
  // reale (palmare in piano, corpo che gira su se stesso): atan2(Y,X)
  // disegna un cerchio pulito e monotono lungo tutto il giro, mentre
  // atan2(Z,X) resta chiuso in una fascia di ~130 gradi (Z non e' nel
  // piano orizzontale in questa posizione d'uso) - un primo test rapido a
  // mano aveva suggerito il contrario ed era sbagliato.
  float h = atan2(compass.getY(), compass.getX()) * 180.0f / PI;
  // Il verso di rotazione era giusto ma il nord risultava all'esatto
  // opposto (180 gradi fissi) - verificato confrontando con una bussola
  // vera: la convenzione di segno degli assi del sensore montato e'
  // invertita rispetto a quella attesa dalla formula standard.
  h += 180.0f;
  h += cfg.declination;
  if (h < 0) h += 360.0f;
  if (h >= 360.0f) h -= 360.0f;
  return h;
}

// ---------------- OROLOGIO (ora italiana, auto da GPS) -------------------
// Il GPS manda l'ora in UTC: per l'Italia serve sommare 1h (CET, inverno) o
// 2h (CEST, ora legale) a seconda del periodo. Regola UE: l'ora legale va
// dall'ultima domenica di marzo (01:00 UTC) all'ultima domenica di ottobre
// (01:00 UTC) - stessa regola in tutta la UE, calcolata qui invece di usare
// una data fissa cosi' resta corretta anno dopo anno senza ritoccare nulla.
static int dowSakamoto(int y, int m, int d) {  // 0=domenica
  static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
  if (m < 3) y -= 1;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}
static int lastSundayOfMonth(int y, int m, int lastDay) {
  return lastDay - dowSakamoto(y, m, lastDay);
}
static bool isDstEU(int year, int month, int day, int hourUtc) {
  if (month < 3 || month > 10) return false;
  if (month > 3 && month < 10) return true;
  int marLastSun = lastSundayOfMonth(year, 3, 31);
  if (month == 3) return (day > marLastSun) || (day == marLastSun && hourUtc >= 1);
  int octLastSun = lastSundayOfMonth(year, 10, 31);
  return (day < octLastSun) || (day == octLastSun && hourUtc < 1);
}
// hh=-1 se il GPS non ha ancora un orario valido.
static void italianLocalTime(int &hh, int &mm) {
  if (!gps.time.isValid() || !gps.date.isValid() || gps.date.year() < 2020) {
    hh = -1; mm = -1;
    return;
  }
  int y = gps.date.year(), mo = gps.date.month(), d = gps.date.day();
  int h = gps.time.hour();
  h += isDstEU(y, mo, d, h) ? 2 : 1;
  if (h >= 24) h -= 24;   // solo per l'ora mostrata, la data non serve qui
  hh = h; mm = gps.time.minute();
}

static int rssiBars(int16_t rssi) {
  if (rssi > -95)  return 4;
  if (rssi > -105) return 3;
  if (rssi > -113) return 2;
  if (rssi > -120) return 1;
  return 0;
}
static void drawSignalBars(int x, int y, int bars) {
  for (int i = 0; i < 4; i++) {
    int h = 3 + i * 2;
    int yy = y + 9 - h;
    if (i < bars) display.fillRect(x + i * 4, yy, 3, h, SSD1306_WHITE);
    else          display.drawRect(x + i * 4, yy, 3, h, SSD1306_WHITE);
  }
}

static uint8_t palmareBattPct = 100;
static uint16_t palmareBattMv = 4000;
// Rilevamento carica: nessun pin STAT/CHG dedicato, e nessuna soglia fissa
// funziona (vedi commento gemello nel collare) - una batteria quasi piena e
// scollegata riposa gia' sopra 4V, e per "surface charge" ci resta un po'
// anche subito dopo lo stacco. Serve la TENDENZA: ACCENDI se la tensione
// sale in modo sostenuto su una finestra di qualche minuto, SPEGNI appena
// scende chiaramente dal massimo visto (lo stacco e' un crollo netto, non
// una discesa lenta, perche' la scheda torna a scaricare la batteria da
// sola senza piu' il contributo del caricatore).
static uint16_t palmareBattPeakMv = 0;
static uint16_t palmareBattRefMv = 0;
static uint32_t palmareBattRefMs = 0;
static bool palmareCharging = false;
static void refreshPalmareBatt() {
  static uint32_t lastMs = 0;
  const uint32_t BATT_TREND_WINDOW_MS = 180000;  // 3 minuti
  if (millis() - lastMs >= 30000 || lastMs == 0) {
    lastMs = millis();
    palmareBattMv = readBatteryMv();
    if (palmareBattMv > palmareBattPeakMv) palmareBattPeakMv = palmareBattMv;
    if (palmareBattRefMs == 0) { palmareBattRefMv = palmareBattMv; palmareBattRefMs = millis(); }
    if (millis() - palmareBattRefMs >= BATT_TREND_WINDOW_MS) {
      if (!palmareCharging && (int32_t)palmareBattMv - (int32_t)palmareBattRefMv >= 12) palmareCharging = true;
      palmareBattRefMv = palmareBattMv;
      palmareBattRefMs = millis();
    }
    if (palmareCharging && (int32_t)palmareBattPeakMv - (int32_t)palmareBattMv >= 20) {
      palmareCharging = false;
      palmareBattPeakMv = palmareBattMv;
      palmareBattRefMv = palmareBattMv;
      palmareBattRefMs = millis();
    }
    palmareBattPct = battPercent(palmareBattMv);
  }
}

#define USE_BUZZER 1
// GPIO48 non e' esposto sugli header di questa scheda (assente dallo schema
// pinout) - quasi certamente riservato al LED RGB integrato sui moduli
// ESP32-S3, non un pin generico per periferiche esterne. Spostato su
// GPIO21 (confermato raggiungibile sulla scheda fisica, a differenza di
// 17/18 provati prima). ATTENZIONE: su V3 (non V4) GPIO21 e' gia' usato
// per PIN_LCD_DC - se in futuro serve il buzzer anche su una V3, va
// scelto un pin diverso solo per quel build.
static const int8_t PIN_BUZZER = 21;
static void beep(int freq, int ms) {
#if USE_BUZZER
  // noTone() difensivo PRIMA di ogni nuovo tono (tranne il primissimo in
  // assoluto): nelle sequenze multi-tono i log confermano che il codice ci
  // prova ma non suona, mentre un tono isolato funziona sempre - sintomo
  // tipico di un secondo tone() che fallisce se il canale LEDC non viene
  // fermato esplicitamente prima. MA chiamare noTone() PRIMA che tone() sia
  // mai stato chiamato una volta rompe il driver ("LEDC is not initialized",
  // visto nei log con beepPowerOn() - il primissimo beep dell'intero
  // programma): il canale va inizializzato da un tone() vero prima di poter
  // essere fermato. Quindi noTone() solo dalla seconda chiamata in poi.
  static bool toneEverStarted = false;
  if (toneEverStarted) noTone(PIN_BUZZER);
  tone(PIN_BUZZER, freq, ms);
  toneEverStarted = true;
#endif
}
// tone() dovrebbe fermarsi da solo dopo "ms", ma su alcune build del core
// ESP32 l'ultimo tono di una sequenza puo' restare acceso a oltranza (timer
// di stop che non parte se la CPU e' occupata altrove nel momento giusto) -
// da qui il "beep fisso" segnalato. Fermata esplicita a fine sequenza,
// dopo aver aspettato la durata prevista dell'ultimo tono: elimina il
// rischio indipendentemente dal comportamento del core.
static void beepStop() {
#if USE_BUZZER
  noTone(PIN_BUZZER);
#endif
}
static void beepLinkLost()  { beep(2200, 120); delay(150); beep(1800, 120); delay(150); beep(1400, 200); delay(200); beepStop(); }
static void beepLinkBack()  { beep(1400, 90);  delay(110); beep(2200, 90); delay(90); beepStop(); }
static void beepLowBatt()   { beep(1000, 60);  delay(90);  beep(1000, 60); delay(90); beep(1000, 60); delay(60); beepStop(); }
static void beepFerma()     { beep(1500, 350); delay(430); beep(1500, 350); delay(350); beepStop(); }
// GPS del palmare (diverso dal LoRa col collare): toni piu' bassi apposta,
// cosi' si distinguono ad orecchio da beepLinkLost/beepLinkBack.
static void beepGpsLost()   { beep(900, 150);  delay(180); beep(600, 250); delay(250); beepStop(); }
static void beepGpsBack()   { beep(600, 90);   delay(110); beep(900, 90);  delay(90);  beepStop(); }
static void beepPowerOn()   { beep(1200, 90);  delay(110); beep(2000, 130); delay(130); beepStop(); }
static void beepPowerOff()  { beep(2000, 90);  delay(110); beep(1200, 160); delay(160); beepStop(); }

static void drawSimple(const char* line1, const char* line2);
static void wakeBurst(uint32_t id) {
  drawSimple("Sveglio...", "Trasmetto\nper 4 sec\nal collare");
  CmdPacket c;
  c.magic = CMD_MAGIC; c.version = 2; c.cmd = CMD_WAKE;
  c.collar_id = id; c.seq = 0; c.p1 = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {
    c.seq++;
    radio.transmit((uint8_t*)&c, sizeof(c));
    delay(60);
  }
  loraPacketFlag = false;
  radio.startReceive();
  drawSimple("Fatto", "Se il collare\nera in standby\nfa 1 beep e\nriparte.");
  delay(1800);
}

// Cambia SF su TUTTI i collari appaiati - deve combaciare col palmare o il
// collegamento si rompe del tutto, e il collare non ha schermo per
// rimediare. Trasmette CMD_SET_SF in broadcast (collar_id=0) al VECCHIO SF
// per 40s (copre anche il ciclo piu' lento di un collare fermo, 30s),
// contando quanti collari distinti rispondono con la conferma (CMD_ACK,
// ancora al vecchio SF - i collari passano al nuovo SF SOLO dopo aver
// confermato). Ritorna quanti hanno confermato: il chiamante decide se
// vale la pena passare anche lui al nuovo SF.
static int sfBurst(uint8_t newSF) {
  char l2[48];
  snprintf(l2, sizeof(l2), "Trasmetto\nfino a 40s\nSF %u->%u", cfg.sf, newSF);
  drawSimple("Cambio SF...", l2);
  CmdPacket c;
  c.magic = CMD_MAGIC; c.version = 2; c.cmd = CMD_SET_SF;
  c.collar_id = 0; c.seq = 0; c.p1 = newSF;
  uint32_t confirmedIds[MAX_DOGS] = {0};
  int confirmedN = 0;
  uint32_t t0 = millis(), lastTxMs = 0;
  loraPacketFlag = false;
  radio.startReceive();
  while (millis() - t0 < 40000) {
    esp_task_wdt_reset();
    if (millis() - lastTxMs >= 1000) {
      lastTxMs = millis();
      c.seq++;
      radio.standby();
      radio.transmit((uint8_t*)&c, sizeof(c));
      radio.startReceive();
    }
    if (loraPacketFlag) {
      loraPacketFlag = false;
      if (radio.getPacketLength() == sizeof(CmdPacket)) {
        CmdPacket ack;
        if (radio.readData((uint8_t*)&ack, sizeof(ack)) == RADIOLIB_ERR_NONE &&
            ack.magic == CMD_MAGIC && ack.version == 2 &&
            ack.cmd == CMD_ACK && ack.p1 == newSF) {
          bool known = false;
          for (int i = 0; i < confirmedN; i++) if (confirmedIds[i] == ack.collar_id) known = true;
          if (!known && confirmedN < MAX_DOGS) confirmedIds[confirmedN++] = ack.collar_id;
        }
      }
      radio.startReceive();
    }
    delay(2);
  }
  loraPacketFlag = false;
  radio.startReceive();
  return confirmedN;
}

static bool     fsOk = false;
static bool     sessOpen = false;
static uint32_t sessStartUnix = 0;
static char     sessPath[24] = {0};
// Vero quando la sessione e' stata aperta senza un orario valido dal GPS del
// palmare (sessStartUnix=0 scritto nell'header): il collare aggancia spesso
// PRIMA del palmare, quindi la primissima sessione della giornata nasce
// spesso "senza data" anche se il fix del palmare arriva 2 secondi dopo.
// trackFixupStartTime() riscrive i 4 byte dell'header non appena l'ora
// diventa valida, cosi' la sessione non resta "senza data" per sempre.
static bool     sessStartUnixPending = false;

#pragma pack(push, 1)
struct TrkPt { uint8_t dog; uint32_t ts; int32_t lat; int32_t lon; };
#pragma pack(pop)
static TrkPt    trkBuf[48];
static uint8_t  trkBufN = 0;
static int32_t  trkLastLat[MAX_DOGS], trkLastLon[MAX_DOGS];
static uint32_t trkLastFlushMs = 0;

static uint32_t gpsUnix() {
  if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < 2020) return 0;
  int y = gps.date.year(), m = gps.date.month(), d = gps.date.day();
  y -= (m <= 2);
  int era = y / 400;
  unsigned yoe = y - era * 400;
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097L + (long)doe - 719468L;
  return (uint32_t)days * 86400UL +
         gps.time.hour() * 3600UL + gps.time.minute() * 60UL + gps.time.second();
}

static void trackEnsureSpace() {
  if (!fsOk) return;
  int guard = 0;
  while (LittleFS.totalBytes() - LittleFS.usedBytes() < 96 * 1024 && guard++ < 20) {
    File dir = LittleFS.open("/t");
    if (!dir) return;
    String oldest = "";
    File f;
    while ((f = dir.openNextFile())) {
      String n = String("/t/") + f.name();
      f.close();
      if (n == String(sessPath)) continue;
      if (oldest == "" || n < oldest) oldest = n;
    }
    dir.close();
    if (oldest == "") return;
    LittleFS.remove(oldest);
  }
}

static void trackOpenSession() {
  if (sessOpen || !fsOk) return;
  prefs.begin("palmare", false);
  uint32_t n = prefs.getUInt("sessN", 0) + 1;
  prefs.putUInt("sessN", n);
  prefs.end();
  snprintf(sessPath, sizeof(sessPath), "/t/s%05lu.trk", (unsigned long)n);
  trackEnsureSpace();
  File f = LittleFS.open(sessPath, "w");
  if (!f) return;
  uint16_t magic = 0x54AB; uint8_t ver = 1;
  sessStartUnix = gpsUnix();
  sessStartUnixPending = (sessStartUnix == 0);
  f.write((uint8_t*)&magic, 2);
  f.write(&ver, 1);
  f.write((uint8_t*)&sessStartUnix, 4);
  uint8_t nd = (uint8_t)dogCount;
  f.write(&nd, 1);
  for (int i = 0; i < dogCount; i++) {
    f.write((uint8_t*)&dogs[i].id, 4);
    char nm[10] = {0};
    strncpy(nm, dogs[i].name, 9);
    f.write((uint8_t*)nm, 10);
  }
  f.close();
  sessOpen = true;
}

// Chiamata da loop(): se la sessione e' nata senza orario valido (vedi sopra),
// appena il GPS del palmare ha un fix riscrive solo i 4 byte del timestamp
// nell'header (offset 3: dopo magic(2)+ver(1)), senza toccare il resto del
// file. No-op immediato se non c'e' nulla in sospeso.
static void trackFixupStartTime() {
  if (!sessStartUnixPending || !sessOpen || !fsOk) return;
  uint32_t now = gpsUnix();
  if (now == 0) return;
  File f = LittleFS.open(sessPath, "r+");
  if (!f) return;
  f.seek(3);
  f.write((uint8_t*)&now, 4);
  f.close();
  sessStartUnix = now;
  sessStartUnixPending = false;
}

static void trackFlush() {
  if (!fsOk || !sessOpen || trkBufN == 0) return;
  trackEnsureSpace();
  File f = LittleFS.open(sessPath, "a");
  if (f) { f.write((uint8_t*)trkBuf, trkBufN * sizeof(TrkPt)); f.close(); }
  trkBufN = 0;
}

// L'intestazione di una sessione (elenco cani + nd) e' scritta una volta sola
// da trackOpenSession(). Se il roster cambia (associ/elimini un cane) mentre
// una sessione e' gia' aperta, gli indici usati da trackLog() non corrispondono
// piu' a quelli congelati nell'header -> punti persi o attribuiti al cane
// sbagliato nell'export GPX. Si chiude la sessione corrente (flush compreso):
// la prossima trackLog() ne apre una nuova con un header aggiornato.
static void trackCloseSession() {
  trackFlush();
  sessOpen = false;
  sessPath[0] = 0;
  sessStartUnixPending = false;
}

static void trackLog(int dogIdx, double lat, double lon) {
  if (!fsOk || dogIdx < 0 || dogIdx >= MAX_DOGS) return;
  int32_t la = (int32_t)(lat * 1e7), lo = (int32_t)(lon * 1e7);
  if (la == trkLastLat[dogIdx] && lo == trkLastLon[dogIdx]) return;
  trkLastLat[dogIdx] = la; trkLastLon[dogIdx] = lo;
  if (!sessOpen) trackOpenSession();
  if (!sessOpen) return;
  if (trkBufN >= (uint8_t)(sizeof(trkBuf) / sizeof(trkBuf[0]))) trackFlush();
  TrkPt &p = trkBuf[trkBufN++];
  p.dog = dogIdx; p.ts = gpsUnix(); p.lat = la; p.lon = lo;
}

static void trackDeleteAll() {
  trkBufN = 0; sessOpen = false; sessPath[0] = 0;
  File dir = LittleFS.open("/t");
  if (!dir) return;
  String toDel[40]; int nDel = 0;
  File f;
  while ((f = dir.openNextFile()) && nDel < 40) {
    toDel[nDel++] = String("/t/") + f.name();
    f.close();
  }
  dir.close();
  for (int i = 0; i < nDel; i++) LittleFS.remove(toDel[i]);
}

static void trackStats(int &nFiles, int &usedPct) {
  nFiles = 0; usedPct = 0;
  if (!fsOk) return;
  File dir = LittleFS.open("/t");
  if (dir) {
    File f;
    while ((f = dir.openNextFile())) { nFiles++; f.close(); }
    dir.close();
  }
  if (LittleFS.totalBytes())
    usedPct = (int)(LittleFS.usedBytes() * 100 / LittleFS.totalBytes());
}

static WebServer  webServer(80);
static bool       wifiApOn = false;
static const char* AP_SSID = "Pi_zaff_sys_DogTrack";
static const char* AP_PASS = "pizaff123";
// L'hotspot non si spegneva mai da solo: lo accendi per guardare la mappa,
// chiudi il telefono e resta acceso a consumare finche' non torni nel menu
// ad occhio. Si spegne da solo dopo WIFI_AUTO_OFF_MS senza nessun client
// associato (il timer riparte ad ogni connessione, quindi resta acceso
// tutto il tempo che serve mentre lo stai davvero usando).
static const uint32_t WIFI_AUTO_OFF_MS = 5UL * 60UL * 1000UL;
static uint32_t   wifiIdleSinceMs = 0;

static const char MAP_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="it"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Pi_Zaff DogTrack v1.5</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{--bg:#101418;--fg:#e8edf2;--acc:#3ba55d;--warn:#e5a53a;--bad:#d9534f}
*{margin:0;padding:0;box-sizing:border-box}
body{background:var(--bg);color:var(--fg);font-family:system-ui,sans-serif;height:100dvh;display:flex;flex-direction:column;overflow:hidden}
#top{padding:8px 12px;display:flex;align-items:center;gap:8px;background:#171d24;flex-wrap:wrap}
#top h1{font-size:15px;font-weight:600;flex:1;min-width:80px}
#st{font-size:12px;color:var(--acc)}
#mapSelect{background:#2a3542;color:var(--fg);border:1px solid #3a4552;border-radius:4px;padding:4px 6px;font-size:11px}
#settingsBtn,#trailBtn{background:#2a3542;color:var(--fg);border:0;border-radius:6px;padding:5px 9px;font-size:12px}
#map,#radar{flex:1;min-height:0}
#radar{display:none;position:relative}
#radar canvas{width:100%;height:100%;display:block}
#cards{display:flex;gap:8px;overflow-x:auto;padding:8px 12px;background:#171d24}
.card{min-width:130px;background:#1f2833;border-radius:10px;padding:8px 10px;flex-shrink:0;border:2px solid transparent;-webkit-tap-highlight-color:transparent}
.card.act{border-color:var(--acc);background:#24352c}
.card .nm{font-weight:700;font-size:14px;display:flex;align-items:center;gap:6px}
.card .dist{font-size:20px;font-weight:800;margin:2px 0}
.card .sub{font-size:11px;color:#9fb0c0}
.arrow{display:inline-block;width:14px;height:14px}
.old{color:var(--warn)}.lost{color:var(--bad);animation:bl 1s infinite}
.ferma{color:var(--bad);font-weight:700;animation:bl 1s infinite}
@keyframes bl{50%{opacity:.25}}
.bz{width:10px;height:10px;border-radius:50%;background:var(--acc);display:inline-block}
.bz.mv{background:var(--warn)}
.sig{display:inline-flex;align-items:flex-end;gap:1px;margin-left:auto;height:13px}
.sig i{width:3px;background:#3a4756;border-radius:1px;display:block}
.sig i.on{background:var(--acc)}
.infoBtn{width:16px;height:16px;border-radius:50%;background:#2a3542;color:#9fb0c0;font-size:10px;font-weight:700;display:flex;align-items:center;justify-content:center;flex-shrink:0;border:0;cursor:pointer;font-family:inherit;padding:0}
.gps{display:inline-flex;align-items:center;gap:4px;font-size:11px;padding:1px 6px 1px 4px;border-radius:8px}
.gps svg{flex-shrink:0}
.gps.gok{background:#1c3b26;color:var(--acc)}
.gps.gok svg{fill:var(--acc);stroke:var(--acc)}
.gps.gpart{background:#3a2e18;color:var(--warn)}
.gps.gpart svg{fill:var(--warn);stroke:var(--warn)}
.gps.gbad{background:#3a2020;color:var(--bad)}
.gps.gbad svg{fill:var(--bad);stroke:var(--bad)}
.batt{display:inline-flex;align-items:center;gap:3px;font-size:11px}
.batt svg{flex-shrink:0}
.batt.bok{color:var(--acc)}
.batt.bwarn{color:var(--warn)}
.batt.bbad{color:var(--bad)}
.batt svg rect{fill:currentColor}
.batt svg rect:first-child{fill:none;stroke:currentColor;stroke-width:1}
#trk{background:#171d24;padding:6px 12px;font-size:13px}
#trk summary{cursor:pointer;color:#9fb0c0;padding:4px 0}
.trow{display:flex;align-items:center;gap:8px;padding:6px 0;border-top:1px solid #232d38}
.trow .ti{flex:1}.trow .td{font-size:11px;color:#9fb0c0}
.trow a,.trow button{background:#2a3542;color:#e8edf2;border:0;border-radius:6px;padding:5px 10px;font-size:12px;text-decoration:none}
.trow button.del{background:#5a2b2b}
#modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.8);z-index:1000;align-items:center;justify-content:center}
#modal.show{display:flex}
#modalContent{background:#1f2833;border-radius:12px;padding:20px;width:88%;max-width:340px;max-height:80vh;overflow-y:auto}
#modalContent h2{margin-bottom:14px;color:var(--acc);font-size:16px}
.settingRow{display:flex;align-items:center;justify-content:space-between;padding:10px 0;border-bottom:1px solid #2a3542;gap:10px}
.settingRow:last-child{border-bottom:none}
.settingRow label{font-size:12px;color:#9fb0c0}
.settingRow input,.settingRow select{background:#2a3542;border:1px solid #3a4552;color:var(--fg);border-radius:4px;padding:5px 6px;font-size:12px;width:110px}
#calibBtn{width:auto;background:var(--warn);color:#000;font-weight:600;border:0;border-radius:6px;padding:6px 10px;font-size:12px}
#calibStatus{font-size:11px;color:var(--acc);margin-top:6px;min-height:14px}
#modalBtns{display:flex;gap:10px;margin-top:15px;justify-content:flex-end}
#modalBtns button{background:var(--acc);color:#fff;border:0;border-radius:6px;padding:8px 15px;font-size:12px}
#modalBtns button.cancel{background:#2a3542;color:var(--fg)}
.me-arrow{width:22px;height:22px;transform-origin:50% 50%}
#infoModal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.8);z-index:1000;align-items:center;justify-content:center}
#infoModal.show{display:flex}
#infoModalContent{background:#1f2833;border-radius:12px;padding:20px;width:88%;max-width:340px;max-height:80vh;overflow-y:auto}
#infoModalContent h2{margin-bottom:14px;color:var(--acc);font-size:16px}
.infoRow{display:flex;justify-content:space-between;padding:7px 0;border-bottom:1px solid #2a3542;font-size:13px;gap:10px}
.infoRow:last-child{border-bottom:none}
.infoRow span:first-child{color:#9fb0c0}
.infoRow span:last-child{font-weight:600;font-variant-numeric:tabular-nums;text-align:right}
#infoModalClose{width:100%;margin-top:15px;background:var(--acc);color:#fff;border:0;border-radius:6px;padding:9px;font-size:13px}
</style></head><body>
<div id="top">
  <h1>Pi_Zaff DogTrack <small style="font-size:10px;font-weight:400;color:#9fb0c0">v1.5</small></h1>
  <span id="st">connessione...</span>
  <select id="mapSelect">
    <option value="osm">Mappa</option>
    <option value="sat">Satellite</option>
    <option value="topo">Topo</option>
  </select>
  <button id="trailBtn">Trail OFF</button>
  <button id="settingsBtn">⚙</button>
</div>
<div id="map"></div><div id="radar"><canvas id="cv"></canvas></div>
<div id="cards"></div>
<details id="trk"><summary>Tracce salvate</summary><div id="trklist">...</div></details>

<div id="modal">
  <div id="modalContent">
    <h2>Impostazioni</h2>
    <div class="settingRow">
      <label>Unità di misura</label>
      <select id="setMetric">
        <option value="1">Metrico (km)</option>
        <option value="0">Imperiale (mi)</option>
      </select>
    </div>
    <div class="settingRow">
      <label>Declinazione (°)</label>
      <input type="number" id="setDecl" step="0.1" min="-180" max="180">
    </div>
    <div class="settingRow">
      <label>Bussola</label>
      <button id="calibBtn">Calibra</button>
    </div>
    <div id="calibStatus"></div>
    <div id="modalBtns">
      <button class="cancel" id="modalCancel">Annulla</button>
      <button id="modalSave">Salva</button>
    </div>
  </div>
</div>
<div id="infoModal">
  <div id="infoModalContent">
    <h2 id="infoModalTitle">Info</h2>
    <div id="infoModalBody"></div>
    <button id="infoModalClose">Chiudi</button>
  </div>
</div>
<script>
let map=null,leafletOk=false,me=null,meArrowIcon=null,dogMk={},dogTr={},dogTrail={},first=true,showTrail=false,curLayer=null;
const TILE_LAYERS={
 osm:{url:'https://tile.openstreetmap.org/{z}/{x}/{y}.png',opts:{maxZoom:19,attribution:'OSM'}},
 sat:{url:'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',opts:{maxZoom:19,attribution:'Esri'}},
 topo:{url:'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',opts:{maxZoom:17,subdomains:'abc',attribution:'OpenTopoMap'}}
};
function tryLeaflet(){
 let s=document.createElement('script');s.src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
 s.onload=()=>{leafletOk=true;initMap()};s.onerror=()=>useRadar();document.head.appendChild(s);
 // Timeout di sicurezza, non una scadenza vera: se il caricamento va a
 // buon fine anche DOPO questi secondi (es. da cache locale ma con un
 // po' di latenza), s.onload sopra chiama comunque initMap() piu' tardi -
 // che ora si prende cura lei di ripristinare la mappa vera anche se nel
 // frattempo useRadar() era gia' scattato, invece di restare bloccati sul
 // radar per sempre con la mappa vera creata ma invisibile dietro di esso.
 setTimeout(()=>{if(!leafletOk)useRadar()},12000);}
function makeLayer(name){
 let cfg=TILE_LAYERS[name]||TILE_LAYERS.osm;
 return L.tileLayer(cfg.url,cfg.opts);
}
function initMap(){
 // Ripristina la mappa vera anche se useRadar() era gia' scattato nel
 // frattempo (vedi commento in tryLeaflet) - va fatto PRIMA di creare la
 // mappa Leaflet, che calcola le dimensioni del contenitore #map solo se
 // e' gia' visibile in quel momento.
 document.getElementById('map').style.display='block';
 document.getElementById('radar').style.display='none';
 map=L.map('map',{zoomControl:false}).setView([41.1,16.8],14);
 map.invalidateSize();
 curLayer=makeLayer('osm');
 let fails=0;curLayer.on('tileerror',()=>{if(++fails>6&&first)useRadar()});
 curLayer.on('tileload',()=>{fails=0});curLayer.addTo(map);
 meArrowIcon=L.divIcon({
  className:'',
  html:'<svg class="me-arrow" viewBox="0 0 24 24"><circle cx="12" cy="12" r="9" fill="#2b8cff" fill-opacity="0.35"/><path d="M12 3 L18 18 L12 14 L6 18 Z" fill="#2b8cff" stroke="#fff" stroke-width="1"/></svg>',
  iconSize:[22,22],iconAnchor:[11,11]
 });
}
function switchLayer(name){
 if(!map)return;
 if(curLayer)map.removeLayer(curLayer);
 curLayer=makeLayer(name);
 curLayer.addTo(map);
}
function useRadar(){document.getElementById('map').style.display='none';
 document.getElementById('radar').style.display='block';map=null;}
let D={dogs:[],me:null,act:0,metric:true};
function fD(m){return D.metric?(m<1000?Math.round(m)+' m':(m/1000).toFixed(2)+' km')
 :(m*3.281<5280?Math.round(m*3.281)+' ft':(m*3.281/5280).toFixed(2)+' mi');}
function sigBars(rssi){
 let n = rssi>-95?4 : rssi>-105?3 : rssi>-113?2 : rssi>-120?1 : 0;
 let out='<span class="sig">';
 for(let i=0;i<4;i++) out+=`<i class="${i<n?'on':''}" style="height:${4+i*3}px"></i>`;
 return out+'</span>';
}
// Icona satellite (SVG minimale: corpo + 2 pannelli), usata nel badge GPS
// qui sotto - sull'OLED (drawSatIcon in main.cpp lato firmware) e' una
// versione a soli pixel per lo spazio ridotto, qui possiamo permetterci
// una forma vera.
const SAT_SVG = '<svg viewBox="0 0 20 20" width="11" height="11"><rect x="8" y="8" width="4" height="4" rx="1"/><rect x="1" y="6" width="5" height="8" rx="1" fill="none" stroke-width="1.4"/><rect x="14" y="6" width="5" height="8" rx="1" fill="none" stroke-width="1.4"/><line x1="6" y1="9" x2="8" y2="9" stroke-width="1.4"/><line x1="12" y1="9" x2="14" y2="9" stroke-width="1.4"/></svg>';
function gpsBadge(d){
 // "veritiero": distingue fix valido, "vedo N satelliti ma niente fix
 // ancora" e "collare mai sentito" - non solo un si/no.
 let cls = d.fix ? 'gok' : (d.sats>0 ? 'gpart' : 'gbad');
 let txt = d.fix ? `${d.sats} sat` : (d.sats>0 ? `${d.sats} sat, no fix` : 'no GPS');
 return `<span class="gps ${cls}">${SAT_SVG}${txt}</span>`;
}
// Icona batteria (corpo + polo + livello) per il badge batteria, stesso
// stile grafico del gpsBadge/SAT_SVG qui sopra.
function battBadge(pct){
 let cls = pct<15?'bbad':pct<40?'bwarn':'bok';
 let fillW = Math.max(1, Math.round(Math.min(100,Math.max(0,pct))/100*10));
 let svg = `<svg viewBox="0 0 16 10" width="16" height="10">`+
   `<rect x="0.5" y="0.5" width="12" height="9" rx="1.5"/>`+
   `<rect x="13" y="3" width="1.5" height="4" rx="0.5"/>`+
   `<rect x="2" y="2" width="${fillW}" height="6" rx="0.5"/></svg>`;
 return `<span class="batt ${cls}">${svg}${Math.round(pct)}%</span>`;
}
// Stessi 3 stati della Home sul palmare (in movimento/in ferma/a riposo),
// dalle stesse flag mv/ferma del collare - non un'invenzione della web GUI.
function moveTxt(d){
 if(d.ferma) return '<span class="ferma">IN FERMA!</span>';
 if(d.mv) return d.spd>1?d.spd.toFixed(0)+' km/h':'In movimento';
 return 'A riposo';
}
function card(d,i){
 let ageTxt=d.age<8?'ora':d.age<60?d.age+'s fa':'PERSO '+Math.floor(d.age/60)+'m';
 let cls=d.age<15?'':d.age<60?'old':'lost';
 return `<div class="card ${i==D.act?'act':''}" data-idx="${i}">
  <div class="nm"><span class="bz ${d.mv?'mv':''}"></span>${d.n}
   <svg class="arrow" viewBox="0 0 20 20" style="transform:rotate(${d.b}deg)">
    <path d="M10 1 L16 17 L10 13 L4 17 Z" fill="#3ba55d"/></svg>
   ${sigBars(d.rssi)}
   <button class="infoBtn" onclick="event.stopPropagation();showDogInfo(${i})">i</button></div>
  <div class="dist">${d.d>=0?fD(d.d):'--'}</div>
  <div class="sub">${moveTxt(d)} · ${d.rssi}dBm</div>
  <div class="sub">${gpsBadge(d)} ${battBadge(d.bt)}</div>
  <div class="sub ${cls}">${ageTxt}</div></div>`;}

// Cambia cane attivo: aggiorna subito in locale (card evidenziata al volo)
// e avvisa il palmare via POST cosi' resta sincronizzato anche sull'OLED
function selectDog(i){
 if(i===D.act)return;
 D.act=i;
 document.getElementById('cards').innerHTML=D.dogs.map(card).join('');
 fetch('/data',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({act:i})}).catch(()=>{});
 if(showTrail){
  // rimuovi le trail degli altri cani, mostra solo quella del cane ora attivo
  Object.keys(dogTrail).forEach(name=>{
   if(map&&dogTrail[name]){map.removeLayer(dogTrail[name]);}
   delete dogTrail[name];
  });
  loadTrail(true);   // forzata: il cane e' cambiato, la traccia va rifatta subito
 }
}
document.getElementById('cards').addEventListener('click',e=>{
 let card=e.target.closest('.card');
 if(card)selectDog(parseInt(card.dataset.idx));
});

let lastTrailFetch=0;
function loadTrail(force){
 if(!showTrail||!map||D.act<0||D.act>=D.dogs.length)return;
 // Throttle: la traccia storica cambia lentamente, riscaricarla ogni 2s
 // (come faceva prima) e' inutile e pesante per il palmare.
 const now=Date.now();
 if(!force && now-lastTrailFetch<15000)return;
 lastTrailFetch=now;
 let dogName=D.dogs[D.act].n;
 fetch(`/api/trail?dog=${D.act}`).then(r=>r.json()).then(j=>{
  if(!j.points||j.points.length<1)return;
  let pts=j.points.map(p=>[p.lat,p.lon]);
  if(!dogTrail[dogName]){
   dogTrail[dogName]=L.polyline(pts,{color:'#808080',weight:2,opacity:.5,dashArray:'5,5'}).addTo(map);
  }else{
   dogTrail[dogName].setLatLngs(pts);
  }
 }).catch(()=>{});}

function upd(){
 fetch('/data').then(r=>r.json()).then(j=>{
  D=j;document.getElementById('st').textContent='● live';
  document.getElementById('st').style.color='#3ba55d';
  document.getElementById('cards').innerHTML=j.dogs.map(card).join('');
  if(map){
   if(j.me){
    if(!me){me=L.marker([j.me.lat,j.me.lon],{icon:meArrowIcon}).addTo(map).bindPopup('Tu');}
    else me.setLatLng([j.me.lat,j.me.lon]);
    let hd=j.me.head||0;
    let el=me.getElement();
    if(el){let svg=el.querySelector('.me-arrow');if(svg)svg.style.transform='rotate('+hd+'deg)';}
   }
   j.dogs.forEach((d,i)=>{
    if(d.lat==0&&d.lon==0)return;
    if(!dogMk[d.n]){
     dogMk[d.n]=L.marker([d.lat,d.lon]).addTo(map).bindTooltip(d.n,{permanent:true,direction:'top',offset:[-15,-8]});
     dogTr[d.n]=L.polyline([],{color:i==0?'#3ba55d':'#e5a53a',weight:3,opacity:.7}).addTo(map);
    }else dogMk[d.n].setLatLng([d.lat,d.lon]);
    let tr=dogTr[d.n];tr.addLatLng([d.lat,d.lon]);
    if(tr.getLatLngs().length>500)tr.setLatLngs(tr.getLatLngs().slice(-500));
   });
   if(first&&(j.me||j.dogs.some(d=>d.lat!=0))){
    let pts=[];if(j.me)pts.push([j.me.lat,j.me.lon]);
    j.dogs.forEach(d=>{if(d.lat!=0)pts.push([d.lat,d.lon])});
    if(pts.length){map.fitBounds(pts,{padding:[40,40],maxZoom:16});first=false;}
   }
   if(showTrail)loadTrail();
  } else drawRadar();
 }).catch(()=>{let s=document.getElementById('st');s.textContent='● offline';s.style.color='#d9534f';});
}
function drawRadar(){
 let cv=document.getElementById('cv'),ctx=cv.getContext('2d');
 let w=cv.width=cv.clientWidth*devicePixelRatio,h=cv.height=cv.clientHeight*devicePixelRatio;
 let cx=w/2,cy=h/2,R=Math.min(w,h)*.42;ctx.clearRect(0,0,w,h);
 let maxD=100;D.dogs.forEach(d=>{if(d.d>maxD)maxD=d.d});maxD*=1.15;
 ctx.strokeStyle='#2a3542';ctx.fillStyle='#9fb0c0';
 ctx.font=(12*devicePixelRatio)+'px sans-serif';ctx.textAlign='center';
 [.33,.66,1].forEach(f=>{ctx.beginPath();ctx.arc(cx,cy,R*f,0,7);ctx.stroke();
  ctx.fillText(fD(maxD*f),cx,cy-R*f-4*devicePixelRatio);});
 ctx.fillStyle='#e8edf2';ctx.font='bold '+(14*devicePixelRatio)+'px sans-serif';
 ctx.fillText('N',cx,cy-R-18*devicePixelRatio);
 let hd=(D.me&&D.me.head!=null)?D.me.head:0;
 ctx.save();ctx.translate(cx,cy);ctx.rotate(hd*Math.PI/180);
 ctx.fillStyle='#2b8cff';ctx.beginPath();
 ctx.moveTo(0,-14*devicePixelRatio);ctx.lineTo(9*devicePixelRatio,10*devicePixelRatio);
 ctx.lineTo(-9*devicePixelRatio,10*devicePixelRatio);ctx.closePath();ctx.fill();ctx.restore();
 // Il triangolo blu al centro sei tu (il palmare) - senza etichetta non
 // era chiaro, veniva scambiato per un semplice mirino della bussola.
 ctx.fillStyle='#2b8cff';ctx.font='bold '+(11*devicePixelRatio)+'px sans-serif';
 ctx.fillText('TU',cx,cy+22*devicePixelRatio);
 D.dogs.forEach((d,i)=>{if(d.d<0)return;
  let a=(d.b-90)*Math.PI/180,r=R*Math.min(d.d/maxD,1);
  let x=cx+Math.cos(a)*r,y=cy+Math.sin(a)*r;
  ctx.fillStyle=i==D.act?'#3ba55d':'#e5a53a';
  ctx.beginPath();ctx.arc(x,y,8*devicePixelRatio,0,7);ctx.fill();
  ctx.fillStyle='#e8edf2';ctx.font=(11*devicePixelRatio)+'px sans-serif';
  ctx.fillText(d.n,x,y-11*devicePixelRatio);});
}
function fmtSt(st){if(!st)return 'sessione senza data';
 let d=new Date(st*1000);return d.toLocaleDateString('it-IT',{day:'2-digit',month:'2-digit'})+' '+d.toLocaleTimeString('it-IT',{hour:'2-digit',minute:'2-digit'});}
function loadTracks(){
 fetch('/tracks').then(r=>r.json()).then(l=>{
  l.sort((a,b)=>b.f.localeCompare(a.f));
  document.getElementById('trklist').innerHTML=l.length?l.map(t=>
   `<div class="trow"><div class="ti"><div>${fmtSt(t.st)}</div>
    <div class="td">${t.dogs.join(', ')} · ${t.pts} punti</div></div>
    <a href="/gpx?f=${t.f}" download>GPX</a>
    <button class="del" onclick="delTrk('${t.f}')">X</button></div>`).join('')
   :'<div class="td" style="padding:8px 0">nessuna traccia</div>';
 }).catch(()=>{});}
function delTrk(f){if(!confirm('Eliminare questa traccia?'))return;
 fetch('/deltrack?f='+f).then(()=>loadTracks());}
document.getElementById('trk').addEventListener('toggle',e=>{if(e.target.open)loadTracks()});

document.getElementById('mapSelect').addEventListener('change',e=>switchLayer(e.target.value));

document.getElementById('trailBtn').addEventListener('click',()=>{
 showTrail=!showTrail;
 let btn=document.getElementById('trailBtn');
 btn.textContent=showTrail?'Trail ON':'Trail OFF';
 btn.style.background=showTrail?'#3ba55d':'#2a3542';
 if(showTrail)loadTrail(true);
 else if(map){
  Object.keys(dogTrail).forEach(name=>{map.removeLayer(dogTrail[name]);});
  dogTrail={};
 }
});

// ---- Impostazioni: apri precaricando i valori attuali dal palmare ----
function openModal(){
 fetch('/getsettings').then(r=>r.json()).then(s=>{
  document.getElementById('setMetric').value=s.metric?'1':'0';
  document.getElementById('setDecl').value=s.decl;
  document.getElementById('calibStatus').textContent='';
  document.getElementById('modal').classList.add('show');
 }).catch(()=>{document.getElementById('modal').classList.add('show');});
}
function closeModal(){document.getElementById('modal').classList.remove('show');}
function saveSettingsWeb(){
 let metric=document.getElementById('setMetric').value==='1';
 let decl=parseFloat(document.getElementById('setDecl').value)||0;
 fetch('/settings',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({metric:metric,decl:decl})})
 .then(()=>{D.metric=metric;closeModal();})
 .catch(()=>{closeModal();});
}
function calibrateCompassWeb(){
 let st=document.getElementById('calibStatus');
 st.textContent='Calibrazione avviata: ruota il palmare a 8 in aria per 15s...';
 fetch('/calibrate',{method:'POST'}).then(()=>{
  let left=17;
  let iv=setInterval(()=>{
   left--;
   st.textContent=left>0?`Calibrazione in corso... ${left}s`:'Fatto! Puoi chiudere.';
   if(left<=0)clearInterval(iv);
  },1000);
 }).catch(()=>{st.textContent='Errore, riprova.';});
}
document.getElementById('settingsBtn').addEventListener('click',openModal);
document.getElementById('modalCancel').addEventListener('click',closeModal);
document.getElementById('modalSave').addEventListener('click',saveSettingsWeb);
document.getElementById('calibBtn').addEventListener('click',calibrateCompassWeb);
document.getElementById('modal').addEventListener('click',e=>{if(e.target.id==='modal')closeModal();});

// ---- Info cane: stessi dati della schermata "Info" sul palmare -------
function fmtCoord(lat,lon){
 if(lat===0&&lon===0) return 'n/d';
 return Math.abs(lat).toFixed(6)+(lat>=0?'N':'S')+' '+Math.abs(lon).toFixed(6)+(lon>=0?'E':'W');
}
function showDogInfo(i){
 let d=D.dogs[i];
 if(!d)return;
 let seen = d.rssi>-999;   // stesso sentinel usato dal palmare per "mai sentito"
 let hasPos = d.llat!==0||d.llon!==0;
 document.getElementById('infoModalTitle').textContent=d.n;
 document.getElementById('infoModalBody').innerHTML=
  `<div class="infoRow"><span>Spreading factor</span><span>SF${D.sf}</span></div>
   <div class="infoRow"><span>RSSI</span><span>${seen?d.rssi+' dBm':'n/d'}</span></div>
   <div class="infoRow"><span>SNR</span><span>${seen?d.snr.toFixed(1):'n/d'}</span></div>
   <div class="infoRow"><span>Bussola palmare</span><span>${D.cal?'calibrata':'da calibrare'}</span></div>
   <div class="infoRow"><span>Batteria palmare</span><span>${D.pbat}%</span></div>
   <div class="infoRow"><span>Batteria collare</span><span>${seen?d.bt+'%':'n/d'}</span></div>
   <div class="infoRow"><span>Ultima posizione</span><span>${fmtCoord(d.llat,d.llon)}</span></div>
   <div class="infoRow"><span>Altitudine</span><span>${hasPos?d.alt+' m':'n/d'}</span></div>
   <div class="infoRow"><span>HDOP</span><span>${hasPos?d.hdop.toFixed(1):'n/d'}</span></div>`;
 document.getElementById('infoModal').classList.add('show');
}
function closeInfoModal(){document.getElementById('infoModal').classList.remove('show');}
document.getElementById('infoModalClose').addEventListener('click',closeInfoModal);
document.getElementById('infoModal').addEventListener('click',e=>{if(e.target.id==='infoModal')closeInfoModal();});

tryLeaflet();upd();setInterval(upd,2000);
</script></body></html>)HTML";

static void buildDataJson(String &out) {
  out.reserve(1408);
  out = "{";
  bool myFix = gps.location.isValid() && gps.location.age() < 5000;
  if (myFix) {
    char b[96];
    snprintf(b, sizeof(b), "\"me\":{\"lat\":%.7f,\"lon\":%.7f,\"head\":%.0f},",
             gps.location.lat(), gps.location.lng(), readHeadingDeg());
    out += b;
  } else out += "\"me\":null,";
  // sf/pbat/cal: dati "di sistema" (non per-cane) che servono al popup
  // Info della web GUI - stessa roba che mostra la schermata Info sul palmare.
  char hdr[112];
  snprintf(hdr, sizeof(hdr), "\"act\":%d,\"metric\":%s,\"sf\":%u,\"pbat\":%u,\"cal\":%s,\"dogs\":[",
           activeDog, cfg.metric ? "true" : "false", cfg.sf, palmareBattPct,
           cfg.calibrated ? "true" : "false");
  out += hdr;
  for (int i = 0; i < dogCount; i++) {
    DogLive &d = live[i];
    double dist = -1; float brg = 0;
    if (myFix && d.everSeen && d.fix) {
      dist = haversineM(gps.location.lat(), gps.location.lng(), d.lat, d.lon);
      brg  = bearingDeg(gps.location.lat(), gps.location.lng(), d.lat, d.lon);
    }
    uint32_t ageS = d.everSeen ? (millis() - d.lastRxMs) / 1000 : 99999;
    char b[336];
    snprintf(b, sizeof(b),
      "%s{\"n\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,\"d\":%.0f,\"b\":%.0f,"
      "\"spd\":%.1f,\"bt\":%u,\"mv\":%s,\"ferma\":%s,\"age\":%lu,\"rssi\":%d,"
      "\"fix\":%s,\"sats\":%u,\"snr\":%.1f,\"hdop\":%.1f,\"alt\":%u,"
      "\"llat\":%.7f,\"llon\":%.7f}",
      i ? "," : "", dogs[i].name,
      (d.everSeen && d.fix) ? d.lat : 0.0, (d.everSeen && d.fix) ? d.lon : 0.0,
      dist, brg, d.speedKmh, d.battPct, d.moving ? "true" : "false",
      d.ferma ? "true" : "false",
      (unsigned long)ageS, d.everSeen ? d.rssi : -999,
      (d.everSeen && d.fix) ? "true" : "false", d.everSeen ? d.sats : 0,
      d.everSeen ? d.snr : 0.0f, d.hdop_x10 / 10.0f, d.alt,
      // llat/llon: SEMPRE l'ultimo fix buono (indipendente dal fix del
      // pacchetto piu' recente), come "Pos:" nella schermata Info del
      // palmare - "lat"/"lon" sopra restano invece legate al fix attuale
      // (servono alla mappa, che non deve mostrare un marker su una
      // posizione ormai vecchia). "alt" segue la stessa logica di llat/llon.
      d.lat, d.lon);
    out += b;
  }
  out += "]}";
}

static volatile bool remoteCalibRequested = false;
static void saveSettings();   // prototipo: usata dentro wifiStart() piu' sotto
static void saveDogs();       // prototipo: idem

static void wifiStart() {
  if (wifiApOn) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  // Niente DNSServer che risponde a "*": era quello a far scattare il
  // popup automatico del captive portal sul telefono (il sistema operativo
  // rileva il DNS che dirotta ogni dominio e lo interpreta come rete "da
  // sbloccare"). Cosi' invece il telefono si connette senza popup, e la
  // mappa si apre solo se l'utente va lui stesso su 192.168.4.1.
  webServer.on("/", []() {
    webServer.send_P(200, "text/html", MAP_HTML);
  });
  webServer.on("/tracks", []() {
    String j = "[";
    bool first = true;
    File dir = LittleFS.open("/t");
    if (dir) {
      File f;
      while ((f = dir.openNextFile())) {
        uint16_t mg = 0; uint8_t ver = 0, nd = 0; uint32_t st = 0;
        f.read((uint8_t*)&mg, 2); f.read(&ver, 1);
        f.read((uint8_t*)&st, 4); f.read(&nd, 1);
        String names = "";
        for (int i = 0; i < nd && i < MAX_DOGS; i++) {
          uint32_t id; char nm[11] = {0};
          f.read((uint8_t*)&id, 4); f.read((uint8_t*)nm, 10);
          if (i) names += ",";
          names += "\"" + String(nm) + "\"";
        }
        size_t hdr = 8 + (size_t)nd * 14;
        size_t pts = (mg == 0x54AB && f.size() > hdr) ? (f.size() - hdr) / sizeof(TrkPt) : 0;
        if (!first) j += ",";
        first = false;
        j += "{\"f\":\"" + String(f.name()) + "\",\"st\":" + String(st) +
             ",\"pts\":" + String(pts) + ",\"dogs\":[" + names + "]}";
        f.close();
      }
      dir.close();
    }
    j += "]";
    webServer.send(200, "application/json", j);
  });
  webServer.on("/gpx", []() {
    trackFlush();
    String fn = "/t/" + webServer.arg("f");
    File f = LittleFS.open(fn, "r");
    if (!f) { webServer.send(404, "text/plain", "not found"); return; }
    uint16_t mg; uint8_t ver, nd = 0; uint32_t st;
    f.read((uint8_t*)&mg, 2); f.read(&ver, 1);
    f.read((uint8_t*)&st, 4); f.read(&nd, 1);
    char names[MAX_DOGS][11] = {{0}};
    for (int i = 0; i < nd && i < MAX_DOGS; i++) {
      uint32_t id; f.read((uint8_t*)&id, 4); f.read((uint8_t*)names[i], 10);
    }
    size_t hdrEnd = f.position();
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.sendHeader("Content-Disposition", "attachment; filename=" + webServer.arg("f") + ".gpx");
    webServer.send(200, "application/gpx+xml", "");
    webServer.sendContent("<?xml version=\"1.0\"?><gpx version=\"1.1\" creator=\"PiZaff-DogTrack\" xmlns=\"http://www.topografix.com/GPX/1/1\">");
    for (int dg = 0; dg < nd && dg < MAX_DOGS; dg++) {
      webServer.sendContent("<trk><name>" + String(names[dg]) + "</name><trkseg>");
      f.seek(hdrEnd);
      TrkPt p;
      String chunk = "";
      while (f.read((uint8_t*)&p, sizeof(p)) == sizeof(p)) {
        if (p.dog != dg) continue;
        char row[128];
        if (p.ts) {
          time_t t = (time_t)p.ts;
          struct tm tmv;
          gmtime_r(&t, &tmv);
          snprintf(row, sizeof(row),
            "<trkpt lat=\"%.7f\" lon=\"%.7f\"><time>%04d-%02d-%02dT%02d:%02d:%02dZ</time></trkpt>",
            p.lat / 1e7, p.lon / 1e7,
            tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
            tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
        } else {
          snprintf(row, sizeof(row),
            "<trkpt lat=\"%.7f\" lon=\"%.7f\"/>", p.lat / 1e7, p.lon / 1e7);
        }
        chunk += row;
        if (chunk.length() > 1400) {
          webServer.sendContent(chunk); chunk = "";
          // Una traccia grande puo' richiedere molti giri di questo while:
          // senza questo controllo il collare rischierebbe di non ricevere
          // l'ACK entro la finestra di 300ms perche' loop() resta bloccato
          // qui dentro. Si serve un pacchetto in arrivo ogni poche migliaia
          // di punti invece di aspettare che tutto il file sia stato inviato.
          if (loraPacketFlag) { loraPacketFlag = false; handleLoraPacket(); }
        }
      }
      if (chunk.length()) webServer.sendContent(chunk);
      webServer.sendContent("</trkseg></trk>");
    }
    webServer.sendContent("</gpx>");
    webServer.sendContent("");
    f.close();
  });
  webServer.on("/api/trail", []() {
    // ATTENZIONE: costruire tutto il JSON in una String era un bug grave.
    // 5000 punti x ~84 caratteri = ~420 KB in RAM, mentre l'ESP32-S3 con
    // WiFi+WebServer attivi ne ha ~150-200 KB liberi (e String alloca in
    // RAM INTERNA, non in PSRAM): allocazione fallita o crash sicuro.
    // Ora la risposta viene inviata a blocchi (chunked), come il GPX.
    int dogIdx = webServer.arg("dog").toInt();
    if (dogIdx < 0 || dogIdx >= MAX_DOGS) {
      webServer.send(400, "application/json", "{\"error\":\"invalid dog\"}");
      return;
    }
    int cnt = getTrailCount(dogIdx);

    // Decimazione: per disegnare una scia su mappa 800 punti bastano e
    // avanzano. Oltre, si manda 1 punto ogni N senza perdere la forma.
    const int MAX_OUT = 800;
    int stepN = (cnt > MAX_OUT) ? ((cnt + MAX_OUT - 1) / MAX_OUT) : 1;

    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "application/json", "");
    char head[96];
    snprintf(head, sizeof(head), "{\"dog\":%d,\"count\":%d,\"step\":%d,\"points\":[",
             dogIdx, cnt, stepN);
    webServer.sendContent(head);

    String chunk;
    chunk.reserve(1600);
    bool firstPt = true;
    for (int i = 0; i < cnt; i += stepN) {
      GpsPoint *pt = getTrailPoint(dogIdx, i);
      if (!pt) break;
      char row[128];
      snprintf(row, sizeof(row),
        "%s{\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"alt\":%d,\"ts\":%lu,\"rssi\":%d}",
        firstPt ? "" : ",",
        pt->lat_e7 / 10000000.0, pt->lon_e7 / 10000000.0,
        pt->speed_x10 / 10.0, (int)pt->alt_m,
        (unsigned long)pt->timestamp, (int)(int16_t)pt->rssi);
      firstPt = false;
      chunk += row;
      if (chunk.length() > 1400) {
        webServer.sendContent(chunk); chunk = "";
        // Stesso motivo del /gpx qui sopra: con 800 punti da inviare non
        // vogliamo far aspettare il collare oltre la sua finestra di ACK.
        if (loraPacketFlag) { loraPacketFlag = false; handleLoraPacket(); }
      }
    }
    if (chunk.length()) webServer.sendContent(chunk);
    webServer.sendContent("]}");
    webServer.sendContent("");     // chiude la risposta chunked
  });
  webServer.on("/getsettings", []() {
    char b[96];
    snprintf(b, sizeof(b), "{\"metric\":%s,\"decl\":%.2f}",
             cfg.metric ? "true" : "false", cfg.declination);
    webServer.send(200, "application/json", b);
  });
  webServer.on("/settings", []() {
    if (webServer.method() == HTTP_POST) {
      String body = webServer.arg("plain");
      if (body.indexOf("metric") != -1) {
        cfg.metric = body.indexOf("\"metric\":true") != -1;
        int declStart = body.indexOf("\"decl\":");
        if (declStart > 0) {
          declStart += 7;
          // Il limite -180/+180 nella pagina e' solo un suggerimento del
          // browser, non blocca una richiesta anomala: readHeadingDeg() fa
          // un solo giro di correzione e si aspetta un valore gia' in
          // questo range, altrimenti la bussola punterebbe nella direzione
          // sbagliata finche' non reimposti la declinazione.
          cfg.declination = constrain(atof(body.substring(declStart).c_str()), -180.0f, 180.0f);
        }
        saveSettings();
        webServer.send(200, "application/json", "{\"ok\":true}");
        return;
      }
    }
    webServer.send(400, "application/json", "{\"error\":\"invalid\"}");
  });
  webServer.on("/calibrate", []() {
    remoteCalibRequested = true;
    webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"Ruota il palmare a 8 per 15 secondi\"}");
  });
  webServer.on("/data", []() {
    if (webServer.method() == HTTP_POST) {
      String body = webServer.arg("plain");
      int actIdx = 0;
      if (body.indexOf("\"act\":") != -1) {
        int pos = body.indexOf("\"act\":");
        if (pos > 0) {
          pos += 6;
          actIdx = atoi(body.substring(pos).c_str());
        }
      }
      if (actIdx >= 0 && actIdx < dogCount) { activeDog = actIdx; saveDogs(); }
      webServer.send(200, "application/json", "{\"ok\":true}");
      return;
    }
    String j; buildDataJson(j);
    webServer.send(200, "application/json", j);
  });
  webServer.on("/deltrack", []() {
    String fn = "/t/" + webServer.arg("f");
    if (fn == String(sessPath)) { trkBufN = 0; sessOpen = false; sessPath[0] = 0; }
    bool ok = LittleFS.remove(fn);
    webServer.send(ok ? 200 : 404, "text/plain", ok ? "ok" : "no");
  });
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "http://192.168.4.1/", true);
    webServer.send(302, "text/plain", "");
  });
  webServer.begin();
  wifiApOn = true;
  wifiIdleSinceMs = millis();
}

static void wifiStop() {
  if (!wifiApOn) return;
  webServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiApOn = false;
}

static void loadAll() {
  prefs.begin("palmare", true);
  cfg.metric      = prefs.getBool("metric", true);
  cfg.declination = prefs.getFloat("decl", 3.5f);
  cfg.calX = prefs.getInt("calx", 0);
  cfg.calY = prefs.getInt("caly", 0);
  cfg.calZ = prefs.getInt("calz", 0);
  cfg.calScaleX = prefs.getFloat("calsx", 1.0f);
  cfg.calScaleY = prefs.getFloat("calsy", 1.0f);
  cfg.calScaleZ = prefs.getFloat("calsz", 1.0f);
  cfg.calibrated = prefs.getBool("caldone", false);
  // Scarta una calibrazione salvata corrotta (offset fuori dal range
  // possibile per un sensore a 16 bit, +-32768) - puo' capitare da una
  // vecchia calibrazione fatta con una rotazione incompleta, vedi
  // runCalibration().
  if (cfg.calibrated && (abs(cfg.calX) > 32768 || abs(cfg.calY) > 32768 || abs(cfg.calZ) > 32768)) {
    cfg.calibrated = false;
    cfg.calX = cfg.calY = cfg.calZ = 0;
    cfg.calScaleX = cfg.calScaleY = cfg.calScaleZ = 1.0f;
  }
  cfg.sf  = prefs.getUChar("sf", LORA_SF);
  cfg.pwr = (int8_t)prefs.getUChar("pwr", (uint8_t)LORA_PWR_DBM);
  if (cfg.sf < 7 || cfg.sf > 12) cfg.sf = LORA_SF;
  dogCount = prefs.getInt("dogN", 0);
  if (dogCount > MAX_DOGS) dogCount = 0;
  if (dogCount > 0) {
    size_t need = sizeof(PairedDog) * dogCount;
    if (prefs.getBytesLength("dogs") == need)
      prefs.getBytes("dogs", dogs, need);
    else dogCount = 0;
  }
  activeDog = constrain(prefs.getInt("activeDog", 0), 0, max(0, dogCount - 1));
  prefs.end();
  if (cfg.calibrated) {
    compass.setCalibrationOffsets(cfg.calX, cfg.calY, cfg.calZ);
    compass.setCalibrationScales(cfg.calScaleX, cfg.calScaleY, cfg.calScaleZ);
  }
}

static void saveSettings() {
  prefs.begin("palmare", false);
  prefs.putBool("metric", cfg.metric);
  prefs.putFloat("decl", cfg.declination);
  prefs.putInt("calx", cfg.calX);
  prefs.putInt("caly", cfg.calY);
  prefs.putInt("calz", cfg.calZ);
  prefs.putFloat("calsx", cfg.calScaleX);
  prefs.putFloat("calsy", cfg.calScaleY);
  prefs.putFloat("calsz", cfg.calScaleZ);
  prefs.putBool("caldone", cfg.calibrated);
  prefs.putUChar("sf", cfg.sf);
  prefs.putUChar("pwr", (uint8_t)cfg.pwr);
  prefs.end();
}

static void saveDogs() {
  prefs.begin("palmare", false);
  prefs.putInt("dogN", dogCount);
  prefs.putBytes("dogs", dogs, sizeof(PairedDog) * dogCount);
  prefs.putInt("activeDog", activeDog);
  prefs.end();
}

static void factoryReset() {
  prefs.begin("palmare", false);
  prefs.clear();
  prefs.end();
  ESP.restart();
}

static int findDogById(uint32_t id) {
  for (int i = 0; i < dogCount; i++) if (dogs[i].id == id) return i;
  return -1;
}

static void noteUnpaired(uint32_t id, int16_t rssi) {
  for (int i = 0; i < MAX_UNPAIRED; i++) {
    if (unpaired[i].id == id) { unpaired[i].rssi = rssi; unpaired[i].lastMs = millis(); return; }
  }
  int oldest = 0;
  for (int i = 0; i < MAX_UNPAIRED; i++) {
    if (unpaired[i].id == 0) { oldest = i; break; }
    if (unpaired[i].lastMs < unpaired[oldest].lastMs) oldest = i;
  }
  unpaired[oldest].id = id;
  unpaired[oldest].rssi = rssi;
  unpaired[oldest].lastMs = millis();
}

static int unpairedFreshCount() {
  int n = 0;
  for (int i = 0; i < MAX_UNPAIRED; i++)
    if (unpaired[i].id != 0 && millis() - unpaired[i].lastMs < 60000) n++;
  return n;
}

static UnpairedSeen* unpairedFresh(int idx) {
  int n = 0;
  for (int i = 0; i < MAX_UNPAIRED; i++) {
    if (unpaired[i].id != 0 && millis() - unpaired[i].lastMs < 60000) {
      if (n == idx) return &unpaired[i];
      n++;
    }
  }
  return nullptr;
}

#if !USE_BLE
static void bleInit() {}
static void bleNotifyDog(int, double, float) {}
#else
static void bleInit() {
  BLEDevice::init(BLE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new SrvCB());
  BLEService* svc = bleServer->createService(BLE_SERVICE_UUID);
  bleChar = svc->createCharacteristic(BLE_CHAR_UUID,
              BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  bleChar->addDescriptor(new BLE2902());
  svc->start();
  bleServer->getAdvertising()->addServiceUUID(BLE_SERVICE_UUID);
  bleServer->getAdvertising()->start();
}

static void bleNotifyDog(int idx, double dist, float brg) {
  if (!bleConnected || !bleChar) return;
  char json[256];
  snprintf(json, sizeof(json),
    "{\"id\":\"%08X\",\"name\":\"%s\",\"lat\":%.7f,\"lon\":%.7f,"
    "\"dist\":%.0f,\"brg\":%.0f,\"spd\":%.1f,\"battC\":%u,\"lowBatt\":%d,"
    "\"sats\":%u,\"mov\":%d,\"rssi\":%d,\"seq\":%u,\"active\":%d}",
    dogs[idx].id, dogs[idx].name, live[idx].lat, live[idx].lon,
    dist, brg, live[idx].speedKmh, live[idx].battPct,
    live[idx].lowBatt ? 1 : 0, live[idx].sats,
    live[idx].moving ? 1 : 0, live[idx].rssi, live[idx].seq,
    idx == activeDog ? 1 : 0);
  bleChar->setValue((uint8_t*)json, strlen(json));
  bleChar->notify();
}
#endif

static void handleLoraPacket() {
  // Un CmdPacket (11 byte) qui fuori da sfBurst() e' solo la conferma di
  // un cambio SF arrivata in ritardo (la finestra di sfBurst() e' gia'
  // chiusa): non c'e' niente da fare con essa a questo punto, si scarta -
  // sicuramente NON va letta come un CollarPacket (29 byte, layout diverso).
  if (radio.getPacketLength() == sizeof(CmdPacket)) {
    CmdPacket junk;
    radio.readData((uint8_t*)&junk, sizeof(junk));
    radio.startReceive();
    return;
  }
  CollarPacket pkt;
  int st = radio.readData((uint8_t*)&pkt, sizeof(pkt));
  int16_t rssi = (int16_t)radio.getRSSI();
  float   snr  = radio.getSNR();
  radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) return;
  if (pkt.magic != PKT_MAGIC || pkt.version != 4) return;

  int idx = findDogById(pkt.collar_id);
  if (idx < 0) {
    noteUnpaired(pkt.collar_id, rssi);
    return;
  }

  DogLive &d = live[idx];
  d.everSeen = true;
  d.fix     = pkt.flags & 0x01;
  d.moving  = pkt.flags & 0x02;
  d.lowBatt = pkt.flags & 0x04;
  d.ferma   = pkt.flags & 0x08;
  d.charging = pkt.flags & 0x10;
  bool fermaJustStarted = (d.ferma && !d.fermaPrev);
  d.fermaPrev = d.ferma;
  if (d.fix) {
    d.lat = pkt.lat_e7 / 1e7; d.lon = pkt.lon_e7 / 1e7; d.alt = pkt.alt_m;
    d.speedKmh = pkt.speed_x10 / 10.0f;
    d.course   = pkt.course_deg;
    d.hdop_x10 = pkt.hdop_x10;   // il collare lo calcola solo con un fix valido
  }
  d.sats = pkt.sats; d.battPct = pkt.batt_pct; d.battMv = pkt.batt_mv;
  d.seq = pkt.seq; d.rssi = rssi; d.snr = snr; d.lastRxMs = millis();
  if (d.fix) trackLog(idx, d.lat, d.lon);
  if (d.fix) addTrailPoint(idx, pkt, rssi);

  CmdPacket c;
  c.magic = CMD_MAGIC; c.version = 2;
  c.cmd = pendingCmd[idx] ? pendingCmd[idx] : CMD_ACK;
  c.collar_id = dogs[idx].id; c.seq = (uint16_t)millis();
  c.p1 = pendingP1[idx];
  delay(15);
  radio.transmit((uint8_t*)&c, sizeof(c));
  if (pendingCmd[idx] == CMD_STANDBY || pendingCmd[idx] == CMD_SET_PWR) { pendingCmd[idx] = 0; pendingP1[idx] = 0; }
  loraPacketFlag = false;
  radio.startReceive();

  if (fermaJustStarted) beepFerma();

  if (gps.location.isValid() && d.fix) {
    double dist = haversineM(gps.location.lat(), gps.location.lng(), d.lat, d.lon);
    float  brg  = bearingDeg(gps.location.lat(), gps.location.lng(), d.lat, d.lon);
    bleNotifyDog(idx, dist, brg);
  }
}

static void drawCompassRing(int cx, int cy, int ringR, float myHeading) {
  const char* labels[4] = {"N", "E", "S", "O"};
  const int   labelDeg[4] = {0, 90, 180, 270};
  for (int i = 0; i < 4; i++) {
    float rel = radians(labelDeg[i] - myHeading - 90.0f);
    // +6 (non +8): con l'anello piu' vicino al bordo dello schermo, la
    // lettera che ruota in cima deve restare sotto la riga divisoria senza
    // sforare - vedi il conto dei margini in drawHome().
    int lx = cx + (int)((ringR + 6) * cos(rel));
    int ly = cy + (int)((ringR + 6) * sin(rel));
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(labels[i], 0, 0, &x1, &y1, &w, &h);
    display.setCursor(lx - w / 2, ly - h / 2);
    display.print(labels[i]);
  }
  display.drawCircle(cx, cy, ringR, SSD1306_WHITE);
}

static void drawArrow(int cx, int cy, int r, float angleDeg) {
  float a = radians(angleDeg - 90.0f);
  int tipX = cx + (int)(cos(a) * r), tipY = cy + (int)(sin(a) * r);
  float aL = a + radians(140.0f), aR = a - radians(140.0f);
  int lX = cx + (int)(cos(aL) * r * 0.6f), lY = cy + (int)(sin(aL) * r * 0.6f);
  int rX = cx + (int)(cos(aR) * r * 0.6f), rY = cy + (int)(sin(aR) * r * 0.6f);
  int bX = cx - (int)(cos(a) * r * 0.25f), bY = cy - (int)(sin(a) * r * 0.25f);
  display.fillTriangle(tipX, tipY, lX, lY, bX, bY, SSD1306_WHITE);
  display.fillTriangle(tipX, tipY, rX, rY, bX, bY, SSD1306_WHITE);
}

static void drawBattIcon(int x, int y, uint8_t pct) {
  display.drawRect(x, y, 14, 6, SSD1306_WHITE);
  display.fillRect(x + 14, y + 1, 1, 4, SSD1306_WHITE);
  display.fillRect(x + 1, y + 1, map(pct, 0, 100, 0, 11), 4, SSD1306_WHITE);
}
// Fulmine minimale (stesso ingombro della lettera C/P che sostituisce
// quando la batteria e' in carica: zero spazio extra da trovare nella riga
// gia' piena in alto - niente pin STAT/CHG sulla scheda, la carica si
// deduce dalla tendenza della tensione, vedi handleLoraPacket()/refreshPalmareBatt()).
static void drawChargeIcon(int x, int y) {
  display.drawLine(x + 3, y,     x + 1, y + 4, SSD1306_WHITE);
  display.drawLine(x + 1, y + 4, x + 3, y + 4, SSD1306_WHITE);
  display.drawLine(x + 3, y + 4, x,     y + 7, SSD1306_WHITE);
}

// Montagnetta minimale (7x8px): un triangolo pieno, accanto alla quota in
// Home. Sostituisce la vecchia iconcina satellite (tolta: ridondante con
// le lettere "C"/"P" delle icone di link qui accanto, e serviva lo spazio
// per l'altitudine).
static void drawMountainIcon(int x, int y) {
  display.fillTriangle(x, y + 8, x + 2, y + 1, x + 4, y + 8, SSD1306_WHITE);
}

// Orologio minimale (7x9px): cerchio + due lancette fisse (non indicano
// l'ora reale, sono solo il simbolo "orologio" accanto al testo HH:MM).
static void drawClockIcon(int x, int y) {
  display.drawCircle(x + 3, y + 4, 3, SSD1306_WHITE);
  display.drawLine(x + 3, y + 4, x + 3, y + 2, SSD1306_WHITE);
  display.drawLine(x + 3, y + 4, x + 5, y + 4, SSD1306_WHITE);
}

// Iconcina telefono (disegnata in NERO dentro al box 9x9 gia' riempito di
// bianco da drawLinkIcon): sostituisce la lettera "P" quando un telefono e'
// connesso al WiFi del palmare, cosi' si vede a colpo d'occhio che qualcuno
// sta guardando la mappa invece del semplice stato GPS del palmare stesso.
static void drawPhoneIcon(int x, int y) {
  display.fillRoundRect(x + 2, y + 1, 4, 7, 1, SSD1306_BLACK);
  display.drawPixel(x + 3, y + 6, SSD1306_WHITE);
}

// Icona di stato "link GPS" con lettera parametrica (C=collare, P=palmare):
// state 2=fix valido (icona piena), 1=dati ma senza fix (contorno fisso),
// 0=nessun dato mai ricevuto (contorno lampeggiante se blink=true, altrimenti
// invisibile). Sostituisce le due funzioni separate drawGpsIcon/drawWifiIcon
// che qui in Home mostravano solo UNA delle due fonti (o GPS o WiFi): ora si
// vedono entrambe (GPS del collare E del palmare) sempre.
static void drawLinkIcon(int x, int y, char letter, int state, bool blink) {
  if (state == 2) {
    display.fillRoundRect(x, y, 9, 9, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(x + 1, y + 1); display.print(letter);
    display.setTextColor(SSD1306_WHITE);
  } else if (state == 1) {
    display.drawRoundRect(x, y, 9, 9, 2, SSD1306_WHITE);
    display.setCursor(x + 1, y + 1); display.print(letter);
  } else if (blink) {
    display.drawRoundRect(x, y, 9, 9, 2, SSD1306_WHITE);
    display.setCursor(x + 1, y + 1); display.print(letter);
  }
}

static void formatDist(double m, char* out, size_t n) {
  if (cfg.metric) {
    if (m < 1000) snprintf(out, n, "%.0f m", m);
    else          snprintf(out, n, "%.2f km", m / 1000.0);
  } else {
    double ft = m * 3.28084;
    if (ft < 5280) snprintf(out, n, "%.0f ft", ft);
    else           snprintf(out, n, "%.2f mi", ft / 5280.0);
  }
}

static const char* cardinal(float deg) {
  static const char* C[] = {"N","NE","E","SE","S","SW","W","NW"};
  return C[(int)((deg + 22.5f) / 45.0f) & 7];
}

// Schermata bussola: mostra l'heading live (si aggiorna da solo ruotando il
// palmare, richiamata di continuo dal ciclo di redraw normale come
// drawHome()) cosi' si vede a colpo d'occhio se il sensore risponde davvero
// - stesso sensore ping-ato all'avvio in setup(), qui pero' e' un test
// pratico: se i gradi non cambiano muovendo il palmare, qualcosa non va.
static void drawCompassScreen() {
  float heading = readHeadingDeg();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("Bussola");
  display.drawFastHLine(0, 9, 64, SSD1306_WHITE);

  char degBuf[8];
  snprintf(degBuf, sizeof(degBuf), "%.0f%c", heading, (char)247);  // 247 = simbolo gradi nel font di default
  display.setTextSize(2);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(degBuf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((64 - w) / 2, 30);
  display.print(degBuf);

  display.setTextSize(1);
  const char* c = cardinal(heading);
  int16_t cx1, cy1; uint16_t cw, ch;
  display.getTextBounds(c, 0, 0, &cx1, &cy1, &cw, &ch);
  display.setCursor((64 - cw) / 2, 52);
  display.print(c);

  display.setCursor(0, 90);
  display.print(cfg.calibrated ? "Calibrata" : "Non cal.");
  display.setCursor(0, 108);
  display.print("Click:cal.");
  display.display();
}

static void drawHome() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (dogCount == 0) {
    display.setCursor(0, 30); display.print("Nessun");
    display.setCursor(0, 40); display.print("cane.");
    display.setCursor(0, 58); display.print("Menu ->");
    display.setCursor(0, 68); display.print("Associa");
    display.display();
    return;
  }

  DogLive &d = live[activeDog];
  uint32_t age = d.everSeen ? (millis() - d.lastRxMs) : 0;
  bool blinkPhase = (millis() / 400) % 2;

  bool myFix = gps.location.isValid() && gps.location.age() < 5000;
  int gpsState = myFix ? 2 : (gps.passedChecksum() > 0 ? 1 : 0);
  // Stato GPS del COLLARE (non del palmare): 0 = mai sentito questo collare,
  // 1 = pacchetti ricevuti ma senza fix al momento, 2 = fix valido adesso.
  int dogGpsState = !d.everSeen ? 0 : (d.fix ? 2 : 1);

  // ---- Layout Home: riga superiore riorganizzata --------------------------
  // Il nome si e' spostato sotto il numero di distanza (vedi piu' sotto):
  // qui in cima ora c'e' la quota a sinistra e i link GPS a destra. Appena
  // cambi cane (loop(), case SCR_HOME) per 3s TUTTA questa riga si svuota e
  // al centro appare solo "x/y" - quota e icone tornano da sole dopo.
  bool showFraction = dogCount > 1 && millis() < homeFractionShowUntil;
  if (showFraction) {
    char fracBuf[8];
    snprintf(fracBuf, sizeof(fracBuf), "%d/%d", activeDog + 1, dogCount);
    int16_t fx1, fy1; uint16_t fw, fh;
    display.getTextBounds(fracBuf, 0, 0, &fx1, &fy1, &fw, &fh);
    display.setCursor((64 - fw) / 2, 0);
    display.print(fracBuf);
  } else {
    // Non c'e' spazio per quota+orologio insieme sulla stessa riga (l'ora
    // "23:59" da sola serve gia' 30px, la quota+icone GPS ne occupano
    // 47 dei 64 disponibili): si alternano nello stesso posto, 5s la
    // quota e 3s l'orologio, a ciclo continuo.
    if ((millis() % 8000) >= 5000) {
      int hh, mm;
      italianLocalTime(hh, mm);
      drawClockIcon(0, 0);
      display.setCursor(9, 0);
      if (hh >= 0) display.printf("%02d:%02d", hh, mm);
      else         display.print("--:--");
    } else {
      drawMountainIcon(0, 0);
      display.setCursor(6, 0);
      // Come Pos:/HDOP nella schermata Info: resta l'ultima quota buona
      // anche se il collare adesso non ha fix.
      if (d.everSeen && (d.lat != 0 || d.lon != 0)) display.printf("%u", d.alt);
      else                                          display.print("--");
    }
    drawLinkIcon(45, 0, 'C', dogGpsState, blinkPhase);
    if (WiFi.softAPgetStationNum() > 0) {
      display.fillRoundRect(55, 0, 9, 9, 2, SSD1306_WHITE);
      drawPhoneIcon(55, 0);
    } else {
      drawLinkIcon(55, 0, 'P', gpsState, blinkPhase);
    }
  }

  bool linkStale = d.everSeen && age >= 60000;
  if (d.everSeen && age < 60000) drawSignalBars(0, 10, rssiBars(d.rssi));
  else if (blinkPhase)           drawSignalBars(0, 10, 0);
  // Lampeggia anche con batteria scarica O segnale perso (non solo batteria
  // scarica come prima): se il collegamento e' saltato, la percentuale
  // mostrata e' l'ultima nota, non quella vera in questo momento - deve
  // essere visivamente chiaro che quel dato potrebbe non essere aggiornato.
  if (!((d.lowBatt || linkStale) && blinkPhase)) {
    if (d.charging) drawChargeIcon(16, 12);
    else { display.setCursor(16, 12); display.print("C"); }
    drawBattIcon(22, 12, d.everSeen ? d.battPct : 0);
  }
  if (!(palmareBattPct < 15 && blinkPhase)) {
    if (palmareCharging) drawChargeIcon(38, 12);
    else { display.setCursor(38, 12); display.print("P"); }
    drawBattIcon(44, 12, palmareBattPct);
  }
  display.drawFastHLine(0, 21, 64, SSD1306_WHITE);
  bool dogFix = d.everSeen && d.fix;

  // Nome centrato, fino a 6 caratteri (era 3-4 quando stava in alto stretto
  // tra quota e icone): usato sia sotto la bussola che nel fallback qui
  // sotto, cosi' sai sempre quale cane stai guardando in ogni schermata.
  char nameBuf[7];
  snprintf(nameBuf, sizeof(nameBuf), "%.6s", dogs[activeDog].name);
  int16_t nx1, ny1; uint16_t nw, nh;
  display.getTextBounds(nameBuf, 0, 0, &nx1, &ny1, &nw, &nh);

  if (myFix && dogFix) {
    double dist = haversineM(gps.location.lat(), gps.location.lng(), d.lat, d.lon);
    float brg   = bearingDeg(gps.location.lat(), gps.location.lng(), d.lat, d.lon);
    float myHeading = readHeadingDeg();
    float rel   = brg - myHeading;
    if (rel < 0) rel += 360.0f;

    // BUG reale (non visibile "a occhio" sui margini dell'anello, che sono
    // giusti): a testTextSize(2) un carattere e' alto 14px, quindi la
    // distanza a y=79 occupava le righe 79-92 - e il nome, subito sotto a
    // y=90, ci finiva sopra per 3 righe (79-92 vs 90-96). Spostato tutto il
    // blocco (anello, freccia, distanza, nome, movimento, direzione, eta')
    // di qualche pixel piu' in alto per far posto: ora la distanza (75,
    // size2) occupa le righe 75-88, il nome parte a 92 - 3px di margine
    // pulito, zero sovrapposizioni. Margine sopra l'anello (verso la riga
    // divisoria a y=21) resta comunque positivo (1px, invece di 2px prima)
    // e quello sotto "agg. ora" (verso il bordo schermo a y=128) pure (2px,
    // invece di 4px prima): piu' stretti ma nessuno dei due si tocca.
    drawCompassRing(32, 48, 16, myHeading);
    drawArrow(32, 48, 12, rel);

    char buf[10];
    if (cfg.metric) {
      if (dist < 1000) snprintf(buf, sizeof(buf), "%.0fm", dist);
      else             snprintf(buf, sizeof(buf), "%.1fk", dist / 1000.0);
    } else {
      double ft = dist * 3.28084;
      if (ft < 5280)  snprintf(buf, sizeof(buf), "%.0fft", ft);
      else            snprintf(buf, sizeof(buf), "%.1fmi", ft / 5280.0);
    }
    display.setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((64 - w) / 2, 75);
    display.print(buf);

    display.setTextSize(1);
    display.setCursor((64 - nw) / 2, 92);
    display.print(nameBuf);

    display.setCursor(0, 101);
    // 3 stati (movimento/ferma/riposo), SOLO qui sotto la bussola: sono
    // veritieri solo quando c'e' un fix pieno (palmare+collare) da mostrare
    // insieme ad essi - fuori da qui non compaiono piu'.
    if (d.ferma) {
      if (blinkPhase) display.print("IN FERMA!");
    } else if (d.moving) {
      if (d.speedKmh >= 1.0f) display.printf("%.0fkm/h", d.speedKmh);
      else                    display.print("Movimento");
    } else {
      display.print("A riposo");
    }
    display.setCursor(0, 110);
    // Tolti i satelliti da questa riga (es. "NE45 SAT12"): su schermo
    // stretto da 64px andava a capo e si accavallava con "agg. ora" sotto.
    display.printf("%s%d%c", cardinal(brg), (int)brg, (char)247);
    display.setCursor(0, 119);
    if (age < 8000)        display.print("agg. ora");
    else if (age < 60000)  display.printf("agg. %lus", age / 1000);
    else if (blinkPhase)   display.print("PERSO!");
  } else {
    display.setCursor(0, 44);
    if (gpsState == 0)      { display.print("GPS: nes-"); display.setCursor(0, 54); display.print("sun dato"); }
    else if (!myFix)        { display.print("GPS in"); display.setCursor(0, 54); display.print("fix..."); }
    else if (!d.everSeen)   { display.print("Attendo"); display.setCursor(0, 54); display.print("segnale"); }
    else                    { display.print("Collare"); display.setCursor(0, 54); display.print("no fix"); }
    display.setCursor((64 - nw) / 2, 70);
    display.print(nameBuf);
  }

  display.display();
}

static void drawList(const char* title, int n, int sel,
                     void (*rowText)(int, char*, size_t), const char* footer) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); display.print(title);
  display.drawFastHLine(0, 9, 64, SSD1306_WHITE);
  char row[24];
  int maxRows = footer ? 9 : 11;
  // Finestra scorrevole centrata sulla selezione: prima non c'era, le righe
  // oltre maxRows non venivano MAI disegnate ne' erano raggiungibili anche
  // se sel poteva arrivare li' (bug latente, mai emerso finche' n<=maxRows -
  // ma con menu piu' lunghi le voci in fondo sarebbero rimaste invisibili).
  int first = 0;
  if (n > maxRows) {
    first = sel - maxRows / 2;
    if (first < 0) first = 0;
    if (first > n - maxRows) first = n - maxRows;
  }
  for (int i = first; i < n && i < first + maxRows; i++) {
    int y = 12 + (i - first) * 10;
    if (i == sel) display.fillRect(0, y - 1, 64, 9, SSD1306_WHITE);
    display.setTextColor(i == sel ? SSD1306_BLACK : SSD1306_WHITE);
    rowText(i, row, sizeof(row));
    row[10] = '\0';
    display.setCursor(1, y); display.print(row);
  }
  display.setTextColor(SSD1306_WHITE);
  if (footer) { display.setCursor(0, 120); display.print(footer); }
  display.display();
}

static void menuRow(int i, char* out, size_t n) {
  if (i == 5) snprintf(out, n, "%s%s", MENU_ITEMS[i], cfg.metric ? "m" : "ft");
  else        snprintf(out, n, "%s", MENU_ITEMS[i]);
}
// Sottomenu "Avanzate": riga 0 = "< indietro" (stesso schema di
// pairRow/dogRow), poi SF e potenza del palmare, poi una riga di aiuto -
// spostate qui dal menu principale per non saturarlo (era arrivato a 12
// voci). Le etichette con valore devono stare sotto i 10 caratteri
// mostrati da drawList() (row[10]='\0'): "Potenza:22" e' il piu' lungo
// possibile (10 esatti, es. "Potenza:100" sforerebbe se mai si arrivasse
// a 3 cifre - ma il massimo e' 22, quindi sempre 2 cifre, sempre 10).
static void advRow(int i, char* out, size_t n) {
  if (i == 0)      snprintf(out, n, "< indietro");
  else if (i == 1) snprintf(out, n, "SF:%u", cfg.sf);
  else if (i == 2) snprintf(out, n, "Potenza:%d", cfg.pwr);
  else             snprintf(out, n, "? Che sono?");
}
// Picker SF: riga 0 = "< indietro" (stesso schema di advRow/dogRow), righe
// 1-6 = SF 7..12. Segna quello attivo cosi' si vede subito da dove si parte.
static void sfPickRow(int i, char* out, size_t n) {
  if (i == 0) { snprintf(out, n, "< indietro"); return; }
  uint8_t sf = (uint8_t)(6 + i);   // i=1 -> SF7 ... i=6 -> SF12
  snprintf(out, n, "SF %u%s", sf, sf == cfg.sf ? " *" : "");
}
// Riga 0 = "< indietro" (si esce ruotando fin li' e cliccando, mai col
// click lungo): stesso schema di MENU_ITEMS[0]="< HOME". Le voci vere
// partono da indice 1, quindi vanno shiftate di -1 verso unpairedFresh().
static void pairRow(int i, char* out, size_t n) {
  if (i == 0) { snprintf(out, n, "< indietro"); return; }
  UnpairedSeen* u = unpairedFresh(i - 1);
  if (u) snprintf(out, n, "%08X", u->id);
  else   snprintf(out, n, "-");
}
// Stesso schema di pairRow(): riga 0 = "< indietro", cani veri da indice 1.
static void dogRow(int i, char* out, size_t n) {
  if (i == 0) { snprintf(out, n, "< indietro"); return; }
  snprintf(out, n, "%s", dogs[i - 1].name);
}
// dogPwrGuess: "migliore stima" della potenza TX del collare - il palmare
// non riceve mai la potenza attuale nella telemetria (solo batt/gps/ecc),
// quindi qui si tiene traccia solo dell'ultimo valore INVIATO, non di uno
// confermato letto dal collare. Se il comando si perde per strada, resta
// comunque quello di prima sul collare (nessun rischio, vedi CMD_SET_PWR).
static int8_t dogPwrGuess[MAX_DOGS] = {22, 22, 22, 22, 22, 22};
static void actionRow(int i, char* out, size_t n) {
  static const char* A[] = {"Rinomina", "Standby", "Sveglia", "Potenza", "Elimina", "Annulla"};
  if (i == 3) snprintf(out, n, "Pot:%ddBm", dogPwrGuess[dogListIdx]);
  else        snprintf(out, n, "%s", A[i]);
}

static void drawNameEditor() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(editDogIdx < 0 ? "Nome:" : "Rinomina:");
  display.drawFastHLine(0, 9, 64, SSD1306_WHITE);

  display.setCursor(1, 14);
  for (int i = 0; i < editPos && i < NAME_LEN; i++) display.print(editName[i]);
  display.print("_");

  display.setTextSize(3);
  display.fillRect(20, 36, 24, 28, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(24, 39);
  display.print(CHARSET[editChar]);

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 76);  display.print("Ruota:");
  display.setCursor(0, 86);  display.print("lettera");
  display.setCursor(0, 100); display.print("Click: ok");
  display.setCursor(0, 114); display.print("Lungo:fine");
  display.display();
}

static void drawSimple(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 4); display.print(line1);
  display.drawFastHLine(0, 14, 64, SSD1306_WHITE);
  display.setCursor(0, 20); display.print(line2);
  display.display();
}

static void drawBootLogo() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.drawRect(2, 20, 60, 62, WHITE);
  display.drawRect(4, 22, 56, 58, WHITE);
  display.setTextSize(1);
  display.setCursor(12, 36); display.print("Pi_Zaff");
  display.setCursor(15, 52); display.print("System");
  display.setCursor(8, 96);  display.print("DogTrack");
  display.setCursor(24, 108); display.print("v1.5");
  display.display();
}

// Spegne (e TIENE spento durante il deep sleep vero) tutto cio' che puo'
// restare ad assorbire corrente - va chiamata da OGNI punto che sta per
// entrare in deep sleep. Scrivere LOW un istante prima di
// esp_deep_sleep_start() SENZA gpio_hold_en() non basta: un pin non tenuto
// torna floating (non "diventa LOW"), e se il chip dall'altra parte ha un
// pull-up interno sul suo ingresso di enable (comune sui front-end RF),
// floating puo' essere letto come ancora HIGH - il PA (o il GPS/bussola su
// Vext) resterebbe acceso per tutta la notte. Stesso meccanismo e stessa
// soluzione gia' usata per PIN_LORA_NSS nello standby del collare.
// gpio_hold_en() richiede solo un GPIO "output-capable" (verificato
// nell'header driver/gpio.h di ESP-IDF) - non e' ristretto ai soli pin RTC
// 0-21, quindi funziona anche su GPIO36 (Vext).
//
// Il reset della radio (RST tenuto LOW) sostituisce l'affidarsi a
// radio.sleep() chiamato caso per caso - confirmPowerOn() puo' arrivare
// qui prima ancora che radio.begin() sia mai stato chiamato in questo
// avvio: in quel caso l'SX1262 resterebbe nel suo stato di post-reset
// (~1.5mA) per tutto il sonno. Tenerla in reset hardware funziona sempre,
// a prescindere da che comandi SPI siano gia' stati mandati.
static void powerDownPeripherals() {
  digitalWrite(PIN_VEXT, HIGH);              // GPS/bussola off
  gpio_hold_en((gpio_num_t)PIN_VEXT);
  digitalWrite(PIN_ADC_CTRL, HIGH);          // partitore batteria scollegato
  gpio_hold_en((gpio_num_t)PIN_ADC_CTRL);
  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_LORA_RST, LOW);           // radio in reset hardware
  gpio_hold_en((gpio_num_t)PIN_LORA_RST);
  // Pull-up RTC esplicito sul pulsante encoder - costa zero, toglie ogni
  // dubbio sul pull-up "normale" configurato da INPUT_PULLUP in setup()
  // (che potrebbe non sopravvivere al deep sleep a seconda del wakeup
  // source usato).
  rtc_gpio_pullup_en((gpio_num_t)PIN_ENC_SW);
  rtc_gpio_pulldown_dis((gpio_num_t)PIN_ENC_SW);
#ifdef BOARD_V4
  digitalWrite(PIN_PA_CSD,  LOW);
  digitalWrite(PIN_PA_VFEM, LOW);
  gpio_hold_en((gpio_num_t)PIN_PA_CSD);
  gpio_hold_en((gpio_num_t)PIN_PA_VFEM);
#endif
  gpio_deep_sleep_hold_en();
}

static void powerOff() {
  beepPowerOff();
  // Non aspettare il debounce di 1500ms in loop(): se il cane attivo e'
  // cambiato appena prima di spegnere, salvalo subito qui.
  if (dogsSaveDirty) { dogsSaveDirty = false; saveDogs(); }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(2, 50); display.print("Spengo...");
  display.display();
  delay(900);
  display.clearDisplay();
  display.display();
  trackFlush();
  radio.sleep();
  wifiStop();
  digitalWrite(PIN_VEXT, HIGH);
#if USE_BLE
  btStop();
#endif
  while (digitalRead(PIN_ENC_SW) == LOW) { esp_task_wdt_reset(); delay(50); }
  delay(100);

  // Se il caricatore e' collegato, niente vero deep sleep: mostra una
  // schermata "in carica" che si aggiorna ogni ~2s (nessun pin STAT/CHG
  // dedicato, quindi refreshPalmareBatt() aggiorna palmareCharging dalla
  // tendenza della tensione ogni 30s, come sempre). Quando il caricatore
  // viene staccato la tensione smette di salire, palmareCharging torna
  // falso da solo e si esce dal ciclo per il vero spegnimento. Tenendo il
  // pulsante 1s si forza comunque lo spegnimento anche mentre e' in carica.
  while (palmareCharging) {
    esp_task_wdt_reset();
    refreshPalmareBatt();
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(10, 40); display.print("In carica");
    drawChargeIcon(26, 55);
    char pctBuf[8];
    snprintf(pctBuf, sizeof(pctBuf), "%u%%", palmareBattPct);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(pctBuf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((64 - w) / 2, 70); display.print(pctBuf);
    display.display();

    bool forceOff = false;
    uint32_t t0 = millis();
    while (millis() - t0 < 2000) {
      esp_task_wdt_reset();
      if (digitalRead(PIN_ENC_SW) == LOW) {
        uint32_t hs = millis();
        while (digitalRead(PIN_ENC_SW) == LOW) {
          esp_task_wdt_reset();
          if (millis() - hs >= 1000) { forceOff = true; break; }
          delay(20);
        }
        if (forceOff) break;
      }
      delay(30);
    }
    if (forceOff) { palmareCharging = false; break; }
  }

  display.clearDisplay();
  display.display();
  powerDownPeripherals();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_ENC_SW, 0);
  esp_deep_sleep_start();
}

static void confirmPowerOn() {
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) return;
  uint32_t t0 = millis();
  while (digitalRead(PIN_ENC_SW) == LOW) {
    if (millis() - t0 >= 2000) return;
    delay(20);
  }
  powerDownPeripherals();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_ENC_SW, 0);
  esp_deep_sleep_start();
}

static void runCalibration() {
  drawSimple("Bussola", "Ruota a 8\nin aria\nper 15s...");
  delay(1200);
  // Calibrazione fatta a mano (non con compass.calibrate() della libreria)
  // per poter controllare il range min/max misurato su ciascun asse prima
  // di accettarlo: successo che una rotazione troppo limitata (es. solo
  // un giro piatto senza inclinare) desse un cerchio vero centrato lontano
  // dall'origine - la bussola sembrava "bloccata" su pochi gradi anche
  // ruotando per intero, perche' visto dall'origine un cerchio piccolo e
  // lontano copre solo un piccolo angolo (effetto parallasse), non perche'
  // ci fosse un magnete/interferenza vicino.
  compass.clearCalibration();
  int32_t xMin = 32767, xMax = -32768, yMin = 32767, yMax = -32768, zMin = 32767, zMax = -32768;
  uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    compass.read();
    int x = compass.getX(), y = compass.getY(), z = compass.getZ();
    if (x < xMin) xMin = x; if (x > xMax) xMax = x;
    if (y < yMin) yMin = y; if (y > yMax) yMax = y;
    if (z < zMin) zMin = z; if (z > zMax) zMax = z;
  }
  int32_t xRange = xMax - xMin, yRange = yMax - yMin, zRange = zMax - zMin;
  // Sotto questa soglia il movimento non ha coperto abbastanza quell'asse
  // per un offset affidabile (vedi commento sopra).
  const int32_t MIN_RANGE = 600;
  if (xRange < MIN_RANGE || yRange < MIN_RANGE || zRange < MIN_RANGE) {
    drawSimple("Riprova", "Movimento\ntroppo\nlimitato.");
    delay(2000);
    return;
  }
  cfg.calX = (xMin + xMax) / 2;
  cfg.calY = (yMin + yMax) / 2;
  cfg.calZ = (zMin + zMax) / 2;
  float xAvgDelta = xRange / 2.0f, yAvgDelta = yRange / 2.0f, zAvgDelta = zRange / 2.0f;
  float avgDelta = (xAvgDelta + yAvgDelta + zAvgDelta) / 3.0f;
  cfg.calScaleX = avgDelta / xAvgDelta;
  cfg.calScaleY = avgDelta / yAvgDelta;
  cfg.calScaleZ = avgDelta / zAvgDelta;
  cfg.calibrated = true;
  compass.setCalibrationOffsets(cfg.calX, cfg.calY, cfg.calZ);
  compass.setCalibrationScales(cfg.calScaleX, cfg.calScaleY, cfg.calScaleZ);
  saveSettings();
  drawSimple("Fatto!", "Salvata.");
  delay(1500);
}

static void nameEditorStart(int dogIdx, uint32_t newId) {
  editDogIdx  = dogIdx;
  pairTargetId = newId;
  memset(editName, 0, sizeof(editName));
  if (dogIdx >= 0) {
    strncpy(editName, dogs[dogIdx].name, NAME_LEN);
  }
  editPos = 0;
  editChar = 1;
  screen = SCR_NAME_EDIT;
}

static void nameEditorFinish() {
  editName[editPos] = '\0';
  for (int i = editPos - 1; i >= 0 && editName[i] == ' '; i--) editName[i] = '\0';
  if (strlen(editName) == 0) { screen = SCR_MENU; return; }

  if (editDogIdx >= 0) {
    strncpy(dogs[editDogIdx].name, editName, NAME_LEN);
    dogs[editDogIdx].name[NAME_LEN] = '\0';
  } else {
    if (dogCount >= MAX_DOGS) { screen = SCR_MENU; return; }
    dogs[dogCount].id = pairTargetId;
    strncpy(dogs[dogCount].name, editName, NAME_LEN);
    dogs[dogCount].name[NAME_LEN] = '\0';
    live[dogCount] = DogLive();
    activeDog = dogCount;
    dogCount++;
    for (int i = 0; i < MAX_UNPAIRED; i++)
      if (unpaired[i].id == pairTargetId) unpaired[i].id = 0;
    // Il roster e' cambiato: se una sessione traccia e' gia' aperta, il suo
    // header (scritto alla prima posizione GPS) non conosce questo cane.
    // Chiudila: la prossima traccia loggata ne apre una nuova con header corretto.
    trackCloseSession();
  }
  saveDogs();
  drawSimple("Salvato!", editName);
  delay(1200);
  screen = SCR_HOME;
}

static void deleteDog(int idx) {
  for (int i = idx; i < dogCount - 1; i++) {
    dogs[i] = dogs[i + 1];
    live[i] = live[i + 1];
    pendingCmd[i] = pendingCmd[i + 1];
    // pendingP1[]/dogPwrGuess[] sono paia con pendingCmd[]/lo slot del
    // cane (vedi dichiarazioni): traslarli separatamente lasciava un
    // valore vecchio/di un altro cane nello slot subentrato, mandando la
    // potenza TX sbagliata al prossimo CMD_SET_PWR in coda (bug trovato in
    // revisione).
    pendingP1[i] = pendingP1[i + 1];
    dogPwrGuess[i] = dogPwrGuess[i + 1];
    trkLastLat[i] = trkLastLat[i + 1];
    trkLastLon[i] = trkLastLon[i + 1];
  }
  pendingCmd[dogCount - 1] = 0;
  pendingP1[dogCount - 1] = 0;
  dogPwrGuess[dogCount - 1] = 22;
  // trails[] e' legato allo SLOT dell'array, non all'identita' del cane: dopo
  // lo shift qui sopra ogni cane da idx in poi ha cambiato slot, quindi la sua
  // vecchia scia in RAM apparterrebbe al cane sbagliato. Azzero tutti gli slot
  // coinvolti (idx..vecchio ultimo indice) cosi' la mappa riparte pulita
  // invece di mischiare il percorso del cane eliminato con quello subentrato.
  for (int i = idx; i < dogCount; i++) clearTrail(i);
  dogCount--;
  if (activeDog >= dogCount) activeDog = max(0, dogCount - 1);
  saveDogs();
  // Stesso motivo del roster-change in nameEditorFinish(): l'header della
  // sessione traccia eventualmente aperta non riflette piu' gli indici attuali.
  trackCloseSession();
}

#ifdef BOARD_V4
// Fix per il FEM KCT8103L della V4.3, stesso ragionamento e stessa fonte del
// collare (vedi commento gemello li'):
//  - CTX (GPIO5) va gestito da RadioLib in automatico solo durante il TX
//    (HIGH), non tenuto fisso: da fermo (LOW) il front-end resta in
//    modalita' LNA, che secondo il datasheet KCT8103L da' 21dB di guadagno
//    in piu' in ricezione - il vero buco di portata inseguito per giorni.
//  - patch al registro 0x8B5 del chip (bit0=1) per la sensibilita' RX,
//    indicata da un ingegnere Heltec. Richiede RADIOLIB_LOW_LEVEL=1 nei
//    build_flags. Il freeze del palmare visto durante i test NON era questa
//    riga: era un GND scollegato durante lo spostamento del filo
//    dell'encoder da GPIO2 a GPIO18. La patch e' innocua (timeout di
//    sicurezza di 1s nel codice RadioLib che la applica).
static void applyFemFixV4() {
  radio.setRfSwitchPins(RADIOLIB_NC, PIN_PA_CTX);
  uint8_t r = 0;
  radio.readRegister(0x8B5, &r, 1);
  r |= 0x01;
  radio.writeRegister(0x8B5, &r, 1);
  // begin() imposta il limite di corrente del PA interno a soli 60mA - con
  // il PA esterno acceso il chip puo' autolimitarsi in TX prima di arrivare
  // alla potenza richiesta. 140 e' il massimo consentito, stesso valore
  // usato dal riferimento MeshCore per questa scheda.
  radio.setCurrentLimit(140.0);
}
#endif
// Guadagno RX aumentato (sezione 9.6 del datasheet SX126x) - a differenza
// della patch sopra e' una feature ufficiale del chip, non "undocumented".
// Costa un filo di corrente in piu' in ricezione, non in trasmissione.
static void applyRxBoostedGain() {
  radio.setRxBoostedGainMode(true);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  // Rilascia l'hold lasciato da powerDownPeripherals() prima di dormire -
  // senza questo, i digitalWrite() qui sotto (e quello su VEXT/ADC_CTRL
  // piu' in basso in setup(), e il reset radio dentro radio.begin()) non
  // avrebbero effetto: il pin resterebbe agganciato al valore di prima
  // del sonno (stesso motivo per cui PIN_LORA_NSS viene rilasciato a
  // inizio setup() sul collare).
  gpio_hold_dis((gpio_num_t)PIN_VEXT);
  gpio_hold_dis((gpio_num_t)PIN_ADC_CTRL);
  gpio_hold_dis((gpio_num_t)PIN_LORA_RST);
  gpio_deep_sleep_hold_dis();
#ifdef BOARD_V4
  gpio_hold_dis((gpio_num_t)PIN_PA_VFEM);
  gpio_hold_dis((gpio_num_t)PIN_PA_CSD);
  // Accendi il PA esterno PRIMA di qualunque radio.begin() piu' sotto.
  // GPIO5 (CTX) NON va qui: lo gestisce RadioLib da solo, vedi applyFemFixV4().
  pinMode(PIN_PA_VFEM, OUTPUT); digitalWrite(PIN_PA_VFEM, HIGH);
  pinMode(PIN_PA_CSD,  OUTPUT); digitalWrite(PIN_PA_CSD,  HIGH);
#endif
  confirmPowerOn();
  beepPowerOn();

  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n[PALMARE] boot v1.5"));

  // Watchdog hardware: se il firmware si blocca per qualunque motivo (un
  // caso limite in RadioLib, un blocco su LittleFS...) il dispositivo si
  // riavvia da solo invece di restare piantato in giro attaccato al cane
  // per ore senza che nessuno se ne accorga. 20s di margine: la calibrazione
  // bussola (runCalibration()) blocca da sola per 10s filati, quindi il
  // timeout deve stare comodamente sopra.
  esp_task_wdt_init(20, true);
  esp_task_wdt_add(NULL);

  pinMode(PIN_VEXT, OUTPUT);     digitalWrite(PIN_VEXT, LOW);
  pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(PIN_ENC_A,  encoderISR, CHANGE);
  attachInterrupt(PIN_ENC_B,  encoderISR, CHANGE);
  attachInterrupt(PIN_ENC_SW, buttonISR, CHANGE);

  display.begin(SSD1306_EXTERNALVCC, 0x00, false, false);
  display.clearDisplay();
  display.setRotation(1);
  drawBootLogo();
  uint32_t logoT0 = millis();

  // Il modulo GPS montato (NEO-M9N) parla a 38400 baud, non 9600 come i
  // moduli precedenti (M8N) - confermato con una scansione diagnostica:
  // a 38400 il 96% dei byte erano testo leggibile con '$' (inizio sentenza
  // NMEA) visto piu' volte, unica velocita' con quel profilo.
  gpsSerial.begin(38400, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Wire.begin(PIN_SDA, PIN_SCL);
  // Il modulo montato risponde su I2C a 0x2C, non 0x0D (default QMC5883L) -
  // la scritta sul modulo dice "HMC5883L" ma quell'indirizzo (0x1E) non
  // risponde affatto sul bus (verificato con scansione completa): quasi
  // certamente un clone con indirizzo diverso dall'originale, comune sulle
  // schede economiche. setADDR() DEVE stare prima di init(): init() scrive
  // gia' dei registri di configurazione, se l'indirizzo fosse ancora quello
  // sbagliato in quel momento la configurazione andrebbe persa.
  compass.setADDR(0x2C);
  compass.init();
  loadAll();
  fsOk = LittleFS.begin(true);
  if (fsOk && !LittleFS.exists("/t")) LittleFS.mkdir("/t");
  initTrails();
  for (int i = 0; i < MAX_DOGS; i++) { trkLastLat[i] = INT32_MIN; trkLastLon[i] = INT32_MIN; }

  loraSPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  int st = radio.begin(LORA_FREQ, LORA_BW, cfg.sf, LORA_CR, LORA_SYNC, cfg.pwr, LORA_PREAMB, LORA_TCXO_V, false);
  if (st != RADIOLIB_ERR_NONE) {
    // Prima era un `while(true) delay(1000)` cieco: se la radio non risponde
    // (contatto intermittente, modulo lento a svegliarsi) il palmare restava
    // acceso e sordo per sempre, spegnibile solo staccando la batteria. Ora
    // resta comunque leggibile il pulsante (tieni premuto = spegni, come nel
    // loop normale) e ogni 3s si riprova l'init - se il contatto si
    // ristabilisce, si continua da qui senza bisogno di spegnere e riaccendere.
    drawSimple("ERR LoRa", "Radio non\ntrovata!\n\nRiprovo...\n\nTieni per\nspegnere");
    uint32_t holdStart = 0, lastRetryMs = millis();
    while (st != RADIOLIB_ERR_NONE) {
      esp_task_wdt_reset();
      if (digitalRead(PIN_ENC_SW) == LOW) {
        if (holdStart == 0) holdStart = millis();
        if (millis() - holdStart >= 1500) powerOff();
      } else {
        holdStart = 0;
      }
      if (millis() - lastRetryMs >= 3000) {
        lastRetryMs = millis();
        // Stesso SF/potenza del primo tentativo (cfg.sf/cfg.pwr, non le
        // costanti di default): un utente che aveva alzato l'SF per
        // portata restava altrimenti risincronizzato silenziosamente su
        // SF8 dopo un retry, sordo a tutti i collari rimasti sull'SF vero
        // (bug trovato in revisione, mai riprodotto ma reale).
        st = radio.begin(LORA_FREQ, LORA_BW, cfg.sf, LORA_CR, LORA_SYNC, cfg.pwr, LORA_PREAMB, LORA_TCXO_V, false);
      }
      delay(20);
    }
  }
  radio.setCRC(true);
  radio.setDio2AsRfSwitch(true);
  applyRxBoostedGain();
#ifdef BOARD_V4
  applyFemFixV4();
#endif
  radio.setPacketReceivedAction(loraISR);
  radio.startReceive();

  bleInit();
  while (millis() - logoT0 < 2000) delay(20);
  Serial.println(F("[PALMARE] pronto"));
}

void loop() {
  esp_task_wdt_reset();
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (loraPacketFlag) { loraPacketFlag = false; handleLoraPacket(); }
  refreshPalmareBatt();
  if (wifiApOn) {
    webServer.handleClient();
    if (palmareBattPct < 15) {
      // Batteria scarica: il WiFi e' uno dei consumi maggiori, e in questo
      // momento la priorita' e' non perdere il collegamento LoRa col
      // collare - si spegne subito, indipendentemente da client connessi.
      wifiStop();
      if (screen == SCR_WIFI) {
        drawSimple("WiFi OFF", "Batteria\nscarica.");
        delay(1500);
        screen = SCR_MENU;
      }
    } else if (WiFi.softAPgetStationNum() > 0) {
      wifiIdleSinceMs = millis();
    } else if (millis() - wifiIdleSinceMs >= WIFI_AUTO_OFF_MS) {
      wifiStop();
      // Avvisa solo se sei li' a guardare proprio la schermata WiFi -
      // altrove non interrompe quello che stai facendo, si spegne e basta.
      if (screen == SCR_WIFI) {
        drawSimple("WiFi OFF", "Nessun\ntelefono\nda 5 min.");
        delay(1500);
        screen = SCR_MENU;
      }
    }
  }
  if (millis() - trkLastFlushMs >= 30000) { trkLastFlushMs = millis(); trackFlush(); }
  trackFixupStartTime();
  if (dogsSaveDirty && millis() - dogsSaveDirtyMs >= 1500) { dogsSaveDirty = false; saveDogs(); }

  if (remoteCalibRequested) { remoteCalibRequested = false; runCalibration(); }

  // Per-cane (non un singolo static confrontato ogni volta col cane
  // attivo): con un solo static, cambiare cane con la rotella confrontava
  // lo stato del NUOVO cane attivo contro il flag lasciato dal VECCHIO,
  // facendo scattare un beep di link perso/tornato "falso" ad ogni cambio
  // (bug trovato in revisione).
  static bool wasLinked[MAX_DOGS] = {false};
  static bool lowBattWarned[MAX_DOGS] = {false};
  static uint32_t lastLinkLostBeepMs[MAX_DOGS] = {0};
  if (dogCount > 0) {
    DogLive &d = live[activeDog];
    bool linked = d.everSeen && (millis() - d.lastRxMs) < 60000;
    if (linked) {
      if (!wasLinked[activeDog]) beepLinkBack();
    } else if (d.everSeen) {
      if (wasLinked[activeDog]) {
        // Appena perso il link in questo giro di loop(): primo avviso subito,
        // non aspettare un altro minuto (bug corretto - prima
        // lastLinkLostBeepMs veniva aggiornato ad ogni giro finche'
        // collegato, quindi al momento della perdita il timer risultava
        // "appena azzerato" e il primo beep scattava un minuto DOPO il
        // previsto, cioe' 2 minuti dall'ultimo pacchetto invece di 1).
        beepLinkLost();
        lastLinkLostBeepMs[activeDog] = millis();
      } else if (millis() - lastLinkLostBeepMs[activeDog] >= 60000) {
        // Gia' perso da prima: si ripete ogni 60s finche' il collegamento
        // non torna - il palmare e' quello che tieni in mano, un
        // promemoria che si ripete finche' non risolvi ha piu' senso qui
        // che sul collare, dove invece serve a ritrovare il cane a orecchio.
        beepLinkLost();
        lastLinkLostBeepMs[activeDog] = millis();
      }
    }
    wasLinked[activeDog] = linked;
    if (d.lowBatt && !lowBattWarned[activeDog]) { beepLowBatt(); lowBattWarned[activeDog] = true; }
    if (!d.lowBatt) lowBattWarned[activeDog] = false;
  }

  // GPS del PALMARE stesso (diverso dal LoRa col collare): un beep quando
  // si perde/torna il fix, stessa logica edge-triggered del link LoRa ma
  // senza ripetizione (il fix GPS puo' sfarfallare molto piu' spesso di un
  // link radio, un allarme che si ripete ogni minuto sarebbe fastidioso).
  static bool wasGpsFix = false;
  bool myFixNow = gps.location.isValid() && gps.location.age() < 5000;
  if (myFixNow && !wasGpsFix) beepGpsBack();
  if (!myFixNow && wasGpsFix) beepGpsLost();
  wasGpsFix = myFixNow;

  // Batteria del PALMARE stesso (non del collare): prima c'era solo l'icona
  // che lampeggia in Home, che pero' non si vede se non hai lo schermo
  // sott'occhio in quel momento - un beep si sente comunque.
  static bool palmareLowBattWarned = false;
  bool palmareLowBatt = palmareBattPct < 15;
  if (palmareLowBatt && !palmareLowBattWarned) { beepLowBatt(); palmareLowBattWarned = true; }
  if (!palmareLowBatt) palmareLowBattWarned = false;

  static uint32_t holdStart = 0;
  if (digitalRead(PIN_ENC_SW) == LOW) {
    if (holdStart == 0) holdStart = millis();
    uint32_t held = millis() - holdStart;
    if (held > 1500) {
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(2, 40); display.print("Tieni per");
      display.setCursor(2, 50); display.print("spegnere");
      int w = map(min(held, (uint32_t)5000), 1500, 5000, 0, 60);
      display.drawRect(2, 66, 60, 10, WHITE);
      display.fillRect(2, 66, w, 10, WHITE);
      display.display();
    }
    if (held >= 5000) { holdStart = 0; powerOff(); }
    delay(20);
    return;
  } else {
    holdStart = 0;
  }

  int32_t d = 0;
  noInterrupts(); d = encDelta; interrupts();
  bool click = btnPressed;     btnPressed = false;
  bool longClick = btnLongPressed; btnLongPressed = false;
  // La tabella di Gray in encoderISR() accumula 4 unita' per ogni click
  // fisico (4 transizioni elettriche per detent): un click completo NON
  // arriva mai a "d" prima di raggiungere +-4, quindi si aspetta quella
  // soglia invece di +-1 - altrimenti ogni click muoverebbe il menu di 4
  // voci invece di 1. Sotto la soglia (rotazione a meta' strada, poi
  // magari invertita prima di completare il click) resta in coda: e'
  // corretto che non conti finche' il click non si conclude davvero.
  static const int32_t ENC_STEP_UNIT = 4;
  int step = (d >= ENC_STEP_UNIT) ? 1 : (d <= -ENC_STEP_UNIT ? -1 : 0);
  // Consuma un solo click per giro di loop() (tutta la logica sotto e'
  // scritta per uno scatto alla volta, es. gli indici con wraparound tipo
  // "(idx + step + n) % n"): se pero' si azzerasse encDelta per intero come
  // prima, ruotando in fretta la rotella gli scatti in eccesso andrebbero
  // persi silenziosamente e il menu sembrerebbe "mangiare" input. Qui invece
  // si scala solo il click appena consumato (4 unita'), il resto resta in
  // coda e viene smaltito uno per volta nei prossimi giri di loop() (che
  // gira molto piu' veloce di quanto un dito possa girare la rotella).
  if (step != 0) { noInterrupts(); encDelta -= step * ENC_STEP_UNIT; interrupts(); }
  // Click breve ad ogni scatto della rotella: feedback tattile/sonoro,
  // utile in uso reale quanto durante lo sviluppo (non era solo debug).
  if (step != 0) beep(2500, 30);

  switch (screen) {

    case SCR_HOME:
      if (step != 0 && dogCount > 1) {
        activeDog = (activeDog + step + dogCount) % dogCount;
        dogsSaveDirty = true; dogsSaveDirtyMs = millis();
        homeFractionShowUntil = millis() + 3000;
      }
      if (click) { screen = SCR_MENU; menuIdx = 0; }
      break;

    case SCR_MENU:
      if (step) menuIdx = constrain(menuIdx + step, 0, MENU_N - 1);
      if (click) {
        switch (menuIdx) {
          case 0: screen = SCR_HOME; break;
          case 1: pairSelIdx = 0; screen = SCR_PAIR_LIST; break;
          case 2:
            if (dogCount == 0) { drawSimple("Nessuno", "Associa\nprima un\ncollare."); delay(1200); }
            else { dogListIdx = 0; screen = SCR_DOG_LIST; }
            break;
          case 3:
            if (palmareBattPct < 15) { drawSimple("Batteria", "scarica!\n\nWiFi non\ndisponibile."); delay(1500); }
            else { wifiStart(); screen = SCR_WIFI; }
            break;
          case 4: screen = SCR_COMPASS; break;
          case 5: cfg.metric = !cfg.metric; saveSettings(); break;
          case 6: screen = SCR_TRACKS; break;
          case 7: screen = SCR_INFO; break;
          case 8: screen = SCR_CONFIRM_RESET; break;
          case 9: advIdx = 0; screen = SCR_ADVANCED; break;
        }
      }
      break;

    case SCR_ADVANCED:
      // Stesso schema di navigazione di PAIR_LIST/DOG_LIST: riga 0 =
      // "< indietro", si esce ruotando fin li' e cliccando.
      if (step) advIdx = constrain(advIdx + step, 0, 3);
      if (click) {
        if (advIdx == 0) { screen = SCR_MENU; break; }
        if (advIdx == 1) {
          // SF deve combaciare su TUTTI gli apparati o il collegamento si
          // rompe - vedi sfBurst(): trasmette il cambio in broadcast al
          // vecchio SF e passa al nuovo SOLO se almeno un collare conferma.
          if (dogCount == 0) {
            drawSimple("SF", "Nessun\ncollare\nassociato.");
            delay(1200);
            break;
          }
          // Apre il picker gia' posizionato sull'SF attivo (riga 0 = "<
          // indietro", righe 1-6 = SF 7..12) invece di ciclare di 1 a ogni
          // click - si sceglie il valore ruotando, si conferma cliccando.
          sfPickIdx = (int)cfg.sf - 6;
          screen = SCR_SF_PICKER;
        } else if (advIdx == 2) {
          // Potenza del PALMARE stesso: indipendente dal collare, nessun
          // rischio di disallineamento - si applica subito.
          static const int8_t PWR_STEPS[] = {10, 14, 17, 20, 22};
          int idx2 = 0;
          for (int i = 0; i < 5; i++) if (PWR_STEPS[i] == cfg.pwr) idx2 = i;
          cfg.pwr = PWR_STEPS[(idx2 + 1) % 5];
          radio.standby();
          radio.setOutputPower(cfg.pwr);
          radio.startReceive();
          saveSettings();
        } else {
          screen = SCR_ADVANCED_INFO;
        }
      }
      break;

    case SCR_ADVANCED_INFO:
      // Schermata di sola lettura: qualunque rotazione esce, come
      // BLE/INFO - vedi quel case per lo stesso schema.
      if (step) screen = SCR_ADVANCED;
      break;

    case SCR_SF_PICKER:
      // Stesso schema di navigazione di PAIR_LIST/DOG_LIST: riga 0 =
      // "< indietro", righe 1-6 = SF 7..12. Ruota per scegliere, click per
      // confermare quel valore preciso (non piu' un ciclo di 1 in 1).
      if (step) sfPickIdx = constrain(sfPickIdx + step, 0, 6);
      if (click) {
        if (sfPickIdx == 0) { screen = SCR_ADVANCED; break; }
        uint8_t newSF = (uint8_t)(6 + sfPickIdx);
        if (newSF == cfg.sf) { screen = SCR_ADVANCED; break; }  // gia' quello attivo
        int confirmed = sfBurst(newSF);
        if (confirmed == 0) {
          drawSimple("SF invariato", "Nessuna\nrisposta.\nSF non\ncambiato.");
          delay(1800);
        } else {
          radio.standby();
          radio.setSpreadingFactor(newSF);
          cfg.sf = newSF;
          saveSettings();
          char l2[48];
          snprintf(l2, sizeof(l2), "%d/%d collari\nconfermati.\nOra SF %u.", confirmed, dogCount, newSF);
          drawSimple("SF cambiato", l2);
          delay(2000);
          radio.startReceive();
        }
        screen = SCR_ADVANCED;
      }
      break;

    case SCR_PAIR_LIST: {
      int n = unpairedFreshCount();
      if (n == 0) {
        // Niente da elencare: esci con una semplice rotazione, non serve
        // una riga "< indietro" dedicata.
        if (step) screen = SCR_MENU;
      } else {
        int rows = n + 1;   // +1 per "< indietro" in cima
        if (step) pairSelIdx = constrain(pairSelIdx + step, 0, rows - 1);
        if (click) {
          if (pairSelIdx == 0) { screen = SCR_MENU; break; }
          UnpairedSeen* u = unpairedFresh(pairSelIdx - 1);
          if (u) {
            if (dogCount >= MAX_DOGS) { drawSimple("Limite!", "Max 6 cani\nEliminane\nuno prima"); delay(1500); }
            else nameEditorStart(-1, u->id);
          }
        }
      }
      break;
    }

    case SCR_NAME_EDIT:
      if (step) {
        editChar += step;
        if (editChar < 0) editChar = CHARSET_N - 1;
        if (editChar >= CHARSET_N) editChar = 0;
      }
      if (click) {
        if (editPos < NAME_LEN) {
          editName[editPos] = CHARSET[editChar];
          editPos++;
          editChar = 1;
          if (editPos >= NAME_LEN) nameEditorFinish();
        }
      }
      if (longClick) nameEditorFinish();
      break;

    case SCR_DOG_LIST: {
      int rows = dogCount + 1;   // +1 per "< indietro" in cima
      if (step) dogListIdx = constrain(dogListIdx + step, 0, rows - 1);
      if (click) {
        if (dogListIdx == 0) { screen = SCR_MENU; break; }
        dogListIdx -= 1;    // normalizza: da qui in poi e' l'indice reale del cane
        dogActionIdx = 0;
        screen = SCR_DOG_ACTIONS;
      }
      break;
    }

    case SCR_DOG_ACTIONS:
      if (step) dogActionIdx = constrain(dogActionIdx + step, 0, 5);
      if (click) {
        if (dogActionIdx == 0) nameEditorStart(dogListIdx, 0);
        else if (dogActionIdx == 1) {
          pendingCmd[dogListIdx] = CMD_STANDBY;
          drawSimple("Inviato", "Standby al\nprossimo\npacchetto\ndel collare.\n\n3 beep =\nspento.");
          delay(2000);
          dogListIdx += 1; screen = SCR_DOG_LIST;   // ri-aggiungi l'offset di "< indietro"
        }
        else if (dogActionIdx == 2) {
          wakeBurst(dogs[dogListIdx].id);
          dogListIdx += 1; screen = SCR_DOG_LIST;
        }
        else if (dogActionIdx == 3) {
          // Potenza del COLLARE: indipendente dal palmare, nessun rischio
          // di disallineamento - resta su questa schermata cosi' si puo'
          // cliccare piu' volte per ciclare, come "Unita:" nel menu.
          static const int8_t PWR_STEPS[] = {10, 14, 17, 20, 22};
          int idx2 = 0;
          for (int i = 0; i < 5; i++) if (PWR_STEPS[i] == dogPwrGuess[dogListIdx]) idx2 = i;
          dogPwrGuess[dogListIdx] = PWR_STEPS[(idx2 + 1) % 5];
          pendingCmd[dogListIdx] = CMD_SET_PWR;
          pendingP1[dogListIdx] = (uint8_t)dogPwrGuess[dogListIdx];
        }
        else if (dogActionIdx == 4) screen = SCR_CONFIRM_DELETE;
        else { dogListIdx += 1; screen = SCR_DOG_LIST; }   // Annulla
      }
      break;

    case SCR_CONFIRM_DELETE:
      if (step) screen = SCR_DOG_ACTIONS;
      if (click) {
        deleteDog(dogListIdx);
        drawSimple("Eliminato", "");
        delay(1000);
        screen = dogCount > 0 ? SCR_DOG_LIST : SCR_MENU;
        dogListIdx = 0;
      }
      break;

    case SCR_INFO:
      // Schermate di sola lettura: la rotella non serve ad altro qui,
      // quindi qualunque rotazione esce - niente click/click lungo.
      if (step) screen = SCR_MENU;
      break;

    case SCR_WIFI:
      // Stesso schema delle conferme: ruota = esci (WiFi resta acceso),
      // click = azione "in piu'" (qui: esci E spegni il WiFi).
      if (step) screen = SCR_MENU;
      if (click) { wifiStop(); screen = SCR_MENU; }
      break;

    case SCR_TRACKS:
      // Idem: ruota = esci, click = azione "in piu'" (vai a conferma cancellazione).
      if (step) screen = SCR_MENU;
      if (click) screen = SCR_TRACKS_DEL;
      break;

    case SCR_TRACKS_DEL:
      if (step) screen = SCR_TRACKS;
      if (click) {
        trackDeleteAll();
        drawSimple("Fatto", "Tracce\ncancellate");
        delay(1200);
        screen = SCR_TRACKS;
      }
      break;

    case SCR_CONFIRM_RESET:
      if (step) screen = SCR_MENU;
      if (click) factoryReset();
      break;

    case SCR_COMPASS:
      // Schermata "live": nessuna lista da scorrere, ruotare esce (come
      // BLE/ADVANCED_INFO), il click avvia la calibrazione - resta su
      // questa schermata anche dopo (runCalibration() e' bloccante e
      // ritorna qui), cosi' si vede subito l'heading con la nuova taratura.
      if (step) { screen = SCR_MENU; break; }
      if (click) runCalibration();
      break;
  }

  static uint32_t lastDraw = 0;
  uint32_t interval = (screen == SCR_HOME || screen == SCR_COMPASS) ? 100 : 150;
  if (millis() - lastDraw >= interval) {
    lastDraw = millis();
    switch (screen) {
      case SCR_HOME:      drawHome(); break;
      case SCR_COMPASS:   drawCompassScreen(); break;
      case SCR_MENU:      drawList("MENU", MENU_N, menuIdx, menuRow, nullptr); break;
      case SCR_PAIR_LIST: {
        int n = unpairedFreshCount();
        if (n == 0) drawSimple("Associa", "Accendi il\ncollare e\nattendi...\n\nRuota=esci");
        else drawList("In aria", n + 1, pairSelIdx, pairRow, nullptr);
        break;
      }
      case SCR_NAME_EDIT: drawNameEditor(); break;
      case SCR_DOG_LIST:  drawList("Cani", dogCount + 1, dogListIdx, dogRow, nullptr); break;
      case SCR_DOG_ACTIONS: drawList(dogs[dogListIdx].name, 6, dogActionIdx, actionRow, nullptr); break;
      case SCR_ADVANCED: drawList("Avanzate", 4, advIdx, advRow, nullptr); break;
      case SCR_SF_PICKER: drawList("Scegli SF", 7, sfPickIdx, sfPickRow, nullptr); break;
      case SCR_ADVANCED_INFO:
        drawSimple("Info radio",
          "SF: quanto\nlontano\narriva il\nsegnale.\n\nPotenza:\nforza del\nsegnale\ndel tuo\npalmare.\n\nRuota=esci");
        break;
      case SCR_CONFIRM_DELETE: {
        char l2[48];
        snprintf(l2, sizeof(l2), "Eliminare\n%.8s?\n\nClick=SI\nRuota=no", dogs[dogListIdx].name);
        drawSimple("Conferma", l2);
        break;
      }
      case SCR_WIFI: {
        char l2[96];
        snprintf(l2, sizeof(l2),
                 "%s\n%s\n\nIP:\n192.168.4.1\n\nTel:%d\n\nRuota=esci\nClick=OFF",
                 AP_SSID, AP_PASS, WiFi.softAPgetStationNum());
        drawSimple("Mappa WiFi", l2);
        break;
      }
      case SCR_TRACKS: {
        int nF, uPct; trackStats(nF, uPct);
        char l2[80];
        snprintf(l2, sizeof(l2),
                 "Sessioni:\n%d\n\nSpazio:\n%d%%\n\nRuota=esci\nClick=canc.",
                 nF, uPct);
        drawSimple("Tracce", l2);
        break;
      }
      case SCR_TRACKS_DEL:
        drawSimple("CANC.TUTTE", "Click=SI\nRuota=no");
        break;
      case SCR_INFO: {
        DogLive &dl = live[activeDog];
        // dl.lat/lon vengono aggiornati SOLO quando arriva un pacchetto con
        // fix valido (vedi handleLoraPacket): restano quindi sempre le
        // ultime coordinate buone anche se il collare ora e' senza fix o
        // fuori portata - esattamente "l'ultima posizione nota".
        // HDOP: dilution of precision, quanto sono ben distribuiti in cielo
        // i satelliti usati per calcolare quella posizione (piu' basso =
        // meglio; sotto 2 ottimo, sopra 10 scarso). Diverso dal NUMERO di
        // satelliti: pochi ma ben sparsi battono tanti ma ammassati.
        char coordBuf[64];
        if (dl.everSeen && (dl.lat != 0 || dl.lon != 0)) {
          snprintf(coordBuf, sizeof(coordBuf), "Pos:\n%c%.6f\n%c%.6f\nAlt:%um\nHDOP:%.1f",
                   dl.lat >= 0 ? 'N' : 'S', fabs(dl.lat),
                   dl.lon >= 0 ? 'E' : 'W', fabs(dl.lon),
                   dl.alt, dl.hdop_x10 / 10.0f);
        } else {
          snprintf(coordBuf, sizeof(coordBuf), "Pos: n/d");
        }
        char l2[192];
        snprintf(l2, sizeof(l2),
                 "SF%u\nRSSI:%d\nSNR:%.1f\nCal:%s C:%d\nBP:%u%%\nBC:%u%%\n%s\nRuota=esci",
                 cfg.sf, dl.rssi, dl.snr,
                 cfg.calibrated ? "OK" : "NO", dogCount,
                 palmareBattPct, dl.battPct, coordBuf);
        drawSimple("Info", l2);
        break;
      }
      case SCR_CONFIRM_RESET:
        drawSimple("RESET?", "Click=SI\nRuota=no");
        break;
    }
  }

  delay(5);
}
