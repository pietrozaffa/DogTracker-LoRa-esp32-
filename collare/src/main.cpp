// ============================================================
//  PiZaff DogTrack - Collare GPS-LoRa
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
//  COLLARE GPS-LoRa  v1.5
//  ESP32-S3 + SX1262 (Heltec V3 pinout) + NEO-M8N + MPU6050
//
//  Funzioni:
//   - Fix GPS continuo (UART1)
//   - TX LoRa adattivo: cane in movimento = invio rapido,
//     cane fermo = invio lento (risparmio batteria)
//   - Lettura tensione batteria (partitore on-board Heltec V3)
//   - Pacchetto binario compatto con CRC gestito da RadioLib
// ============================================================

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Preferences.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

// Modalita' di sleep memorizzata in RTC (sopravvive al deep sleep).
// DEVE stare qui, PRIMA di goToSleep()/confirmPowerOnCollare() che la
// usano piu' sotto - una variabile globale va dichiarata prima del
// primo punto in cui viene referenziata nel file, altrimenti il
// compilatore da' errore "was not declared in this scope".
RTC_DATA_ATTR static uint8_t rtcSleepMode = 0;   // 0=spento tot., 1=standby radio
// Vero se confirmPowerOnCollare() ha rilevato il gesto di reset di
// fabbrica (magnete tenuto 10s all'accensione): il reset vero e proprio
// (currentSF/currentPwr, che dipendono da costanti dichiarate piu' sotto
// nel file) viene applicato in setup() appena sono disponibili.
static bool factoryResetPending = false;

// Reed switch: dichiarato QUI perche' goToSleep()/confirmPowerOnCollare()
// piu' sotto lo usano. Prima il numero di pin era scritto a mano in 5
// punti: spostando il pin sulla V4 quei punti sarebbero rimasti al 7
// (VFEM_Ctrl) e lo spegnimento avrebbe smesso di funzionare.
#ifdef BOARD_V4
static const int8_t PIN_REED = 4;      // V4: GPIO 7 e' VFEM_Ctrl (ampli RF!)
#else
static const int8_t PIN_REED = 7;
#endif

// Amplificatore RF esterno (solo V4): a differenza della V3, va acceso
// esplicitamente via software - senza questi pin resta in modalita' ridotta.
// Scritta sulla scheda: "V4.3" -> front-end KCT8103L (non GC1109/V4.2, che
// useremmo se leggesse V4.2 - i due chip hanno una mappatura pin diversa,
// vedi note di Meshtastic per heltec_v4). Confermato anche a posteriori:
// GPIO5 "difettoso" trovato mesi fa testando l'encoder non era un pin
// guasto, era gia' collegato al CTX di questo front-end.
//   GPIO7 = VFEM (alimentazione front-end, sempre HIGH)
//   GPIO2 = CSD  (chip enable, sempre HIGH)
//   GPIO5 = CTX  (seleziona LNA in ricezione / PA in trasmissione - va
//                 gestito da RadioLib con setRfSwitchPins(), vedi
//                 applyFemFixV4(): HIGH solo durante il burst di TX,
//                 LOW il resto del tempo per avere il guadagno LNA in RX)
// Nessun conflitto con altri pin usati su questa scheda (verificato sopra).
#ifdef BOARD_V4
static const int8_t PIN_PA_VFEM = 7;
static const int8_t PIN_PA_CSD  = 2;
static const int8_t PIN_PA_CTX  = 5;
#endif
// Buzzer: dichiarato QUI (invece che piu' sotto con gli altri pin) per lo
// stesso motivo del reed - confirmPowerOnCollare() ora lo usa per i 5 beep
// di conferma del reset di fabbrica.
// GPIO48 non e' esposto sugli header di questa scheda (assente dallo schema
// pinout) - quasi certamente riservato al LED RGB integrato sui moduli
// ESP32-S3, non un pin generico per periferiche esterne. Spostato su
// GPIO21 (confermato raggiungibile sulla scheda fisica): nessun altro uso
// su questa scheda (il collare non ha display).
static const int8_t PIN_BUZZER = 21;
static void beepN(int n, int freq = 2000, int durMs = 120, int gapMs = 130);  // prototipo

// Vext (alimentazione GPS/MPU6050, LOW = ON): dichiarato QUI (invece che
// piu' sotto con ADC_CTRL) per lo stesso motivo del reed - serve dentro
// powerDownPeripherals()/goToSleep() piu' in basso.
static const int8_t PIN_VEXT = 36;
// ADC_CTRL (partitore batteria, HIGH = scollegato) e LORA_RST (reset del
// chip radio): dichiarati QUI per lo stesso motivo, servono a
// powerDownPeripherals() piu' sotto - i loro "gemelli" (VBAT_ADC, e gli
// altri pin SPI della radio) restano dichiarati piu' in basso.
static const int8_t PIN_ADC_CTRL = 37;
static const int8_t PIN_LORA_RST = 12;

// ---------------- ACCENSIONE / SPEGNIMENTO (reed) ------------
// Spegne (e TIENE spento durante il deep sleep vero) tutto cio' che puo'
// restare ad assorbire corrente. Va chiamata da OGNI punto che sta per
// entrare in deep sleep. Scrivere LOW un istante prima di
// esp_deep_sleep_start() SENZA gpio_hold_en() non basta: un pin non tenuto
// torna floating (non "diventa LOW"), e se il chip dall'altra parte ha un
// pull-up interno sul suo ingresso di enable (comune sui front-end RF),
// floating puo' essere letto come ancora HIGH. gpio_hold_en() richiede solo
// un GPIO "output-capable" (verificato nell'header driver/gpio.h di
// ESP-IDF) - non e' ristretto ai soli pin RTC 0-21, funziona anche su
// GPIO36/37 (Vext/ADC_CTRL).
//
// Il reset della radio (RST tenuto LOW) sostituisce l'affidarsi a
// radio.sleep() chiamato caso per caso - due punti del codice arrivano al
// deep sleep senza mai aver chiamato radio.sleep() (magnete tolto durante
// il reset di fabbrica, e il retry-loop se la radio non si inizializza):
// in quei casi l'SX1262 resterebbe in STDBY_RC post-POR (~1.5mA) per
// tutto il sonno. Tenerlo in reset hardware funziona sempre, a prescindere
// da che comandi SPI siano gia' stati mandati.
// resetRadio=true SOLO per lo spegnimento vero (goToSleep()): tiene
// l'SX1262 in reset hardware perche' li' non deve rispondere piu' a
// nessuno. false per lo standby (enterStandby()/resumeStandbySleep()): la
// radio li' deve restare accesa e in ascolto (startReceiveDutyCycleAuto())
// per il risveglio via burst del palmare - resettarla in quel momento
// (bug reale trovato in revisione: prima veniva resettata SEMPRE, anche
// qui, subito prima o dopo aver ordinato l'ascolto autonomo, mandando quel
// comando SPI a un chip in reset) rompeva silenziosamente lo standby: il
// collare restava risvegliabile solo col magnete finche' non si staccava
// e riattaccava la batteria.
static void powerDownPeripherals(bool resetRadio) {
  digitalWrite(PIN_VEXT, HIGH);              // GPS/MPU6050 off
  gpio_hold_en((gpio_num_t)PIN_VEXT);
  digitalWrite(PIN_ADC_CTRL, HIGH);          // partitore batteria scollegato
  gpio_hold_en((gpio_num_t)PIN_ADC_CTRL);
  if (resetRadio) {
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_RST, LOW);         // radio in reset hardware
    gpio_hold_en((gpio_num_t)PIN_LORA_RST);
  }
  // Pull-up RTC esplicito sul reed: necessario per l'EXT1 usato dallo
  // standby (documentato: i pull-up normali non sopravvivono se le
  // periferiche RTC sono spente), probabilmente ridondante per l'EXT0 del
  // vero spegnimento (che invece tiene le periferiche RTC accese da solo) -
  // ma costa zero metterlo comunque su entrambi i casi.
  rtc_gpio_pullup_en((gpio_num_t)PIN_REED);
  rtc_gpio_pulldown_dis((gpio_num_t)PIN_REED);
#ifdef BOARD_V4
  digitalWrite(PIN_PA_CSD,  LOW);
  digitalWrite(PIN_PA_VFEM, LOW);
  gpio_hold_en((gpio_num_t)PIN_PA_CSD);
  gpio_hold_en((gpio_num_t)PIN_PA_VFEM);
#endif
  gpio_deep_sleep_hold_en();
}

// Da spento: magnete tenuto 2s sul case = accensione.
// Da acceso: magnete tenuto 3s = spegnimento (deep sleep).
// Il tempo minimo evita toggle accidentali (es. collare vicino a metallo).
static void goToSleep() {
  powerDownPeripherals(true);
  rtcSleepMode = 0;                          // spento totale: sveglia SOLO a magnete
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_REED, 0);
  esp_deep_sleep_start();
}

static void confirmPowerOnCollare() {
  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  if (wc != ESP_SLEEP_WAKEUP_EXT0 && wc != ESP_SLEEP_WAKEUP_EXT1) return; // primo avvio
  uint32_t t0 = millis();
  while (digitalRead(PIN_REED) == LOW) {
    if (millis() - t0 >= 2000) {
      // Acceso. Se il magnete resta appoggiato altri 8s (10s totali dalla
      // pressione), e' il gesto di reset di fabbrica dei parametri radio
      // (SF/potenza) - l'unica via di recupero fisica su un dispositivo
      // senza schermo, per il caso limite in cui il ciclo SF automatico
      // da solo non basti. Il reset vero (currentSF/saveSF, dichiarati
      // piu' sotto) viene applicato in setup(); qui segnalo solo che va
      // fatto, e do il feedback sonoro subito mentre il magnete e' ancora
      // li' cosi' l'utente sa che e' scattato.
      pinMode(PIN_BUZZER, OUTPUT);
      bool didReset = false;
      while (digitalRead(PIN_REED) == LOW) {
        if (!didReset && millis() - t0 >= 10000) {
          didReset = true;
          factoryResetPending = true;
          beepN(5, 2000, 100, 100);
        }
        delay(50);
      }
      return;
    }
    delay(20);
  }
  goToSleep();   // magnete tolto troppo presto: rispegni
}

// ---------------- PIN (Heltec WiFi LoRa 32 V3 / cloni) -------
// LoRa SX1262 (PIN_LORA_RST dichiarato piu' in alto, vedi commento li' -
// serve a powerDownPeripherals())
static const int8_t PIN_LORA_NSS  = 8;
static const int8_t PIN_LORA_SCK  = 9;
static const int8_t PIN_LORA_MOSI = 10;
static const int8_t PIN_LORA_MISO = 11;
static const int8_t PIN_LORA_BUSY = 13;
static const int8_t PIN_LORA_DIO1 = 14;

// GPS NEO-M8N su UART1 (pin liberi sull'header V3)
// Su V4 il GPIO 34 e' VGNSS_Ctrl (funzione dedicata a bordo): il TX
// del GPS viene spostato su GPIO 16, libero. Vedi platformio.ini.
static const int8_t PIN_GPS_RX = 33;   // ESP32 RX  <- GPS TX
#ifdef BOARD_V4
static const int8_t PIN_GPS_TX = 16;   // V4: 34 e' VGNSS_Ctrl
#else
static const int8_t PIN_GPS_TX = 34;   // ESP32 TX  -> GPS RX
#endif

// MPU6050 su I2C secondario (l'I2C 17/18 e' occupato dall'OLED)
static const int8_t PIN_SDA = 41;
static const int8_t PIN_SCL = 42;

// Buzzer passivo (feedback acustico accensione/spegnimento):
// + su GPIO48, - su GND (PIN_BUZZER dichiarato piu' in alto, vedi commento
// li'). Sostituisce il LED di stato: col collare dentro il case sigillato
// e coperto dal pelo, un beep si sente, un LED non si vede.

// Reed switch magnetico (accensione/spegnimento senza fori nel case):
// un capo a GPIO7, l'altro a GND. Magnete avvicinato = contatto chiuso.
// Batteria (Heltec V3: ADC su GPIO1, abilitazione partitore su GPIO37)
static const int8_t PIN_VBAT_ADC  = 1;
// PIN_ADC_CTRL e PIN_VEXT (alimentazione periferiche esterne, LOW = ON su
// V3) sono dichiarati piu' in alto, vedi commento li'.

// ---------------- PARAMETRI RADIO ----------------------------
static const float    LORA_FREQ    = 869.525f;  // MHz, banda EU 10% duty
static const float    LORA_BW      = 125.0f;    // kHz
static const uint8_t  LORA_SF      = 8;         // compromesso portata/refresh
static const uint8_t  LORA_CR      = 5;         // 4/5
static const uint8_t  LORA_SYNC    = 0xA7;      // rete privata
// Tensione TCXO passata a radio.begin(): su V4 1.8V e' il valore indicato
// dal file di configurazione Meshtastic per heltec_v4 per il suo TCXO. Sulla
// V3 NON abbiamo questa conferma - resta al default RadioLib (1.6V, quello
// gia' usato finora, quindi zero rischio di regressione li'). Un TCXO
// alimentato alla tensione sbagliata puo' non stabilizzarsi bene, con
// perdita di precisione in frequenza.
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

// ID rete: il palmare scarta pacchetti con magic diverso
static const uint16_t PKT_MAGIC = 0xCA9E;

// ---------------- INTERVALLI TX ------------------------------
static const uint32_t TX_FAST_MS = 2500;    // cane in movimento
static const uint32_t TX_SLOW_MS = 30000;   // cane fermo
static const uint32_t MOTION_HOLD_MS = 15000; // resta "fast" per 15s dall'ultimo movimento
static const float    MOTION_G_THRESH = 0.25f; // delta-g per trigger movimento

// ---------------- PACCHETTO ----------------------------------
#pragma pack(push, 1)
struct CollarPacket {
  uint16_t magic;      // PKT_MAGIC
  uint8_t  version;    // 4
  uint8_t  flags;      // bit0 fix, bit1 mov, bit2 batt scarica, bit3 FERMA, bit4 in carica
  uint32_t collar_id;  // ID univoco derivato dal MAC eFuse del chip
  int32_t  lat_e7;     // gradi * 1e7
  int32_t  lon_e7;     // gradi * 1e7
  uint16_t alt_m;      // metri (0 se no fix)
  uint16_t speed_x10;  // velocita' cane km/h * 10
  uint16_t course_deg; // rotta GPS 0-359 (direzione in cui corre il cane)
  uint8_t  sats;       // satelliti in uso
  uint8_t  hdop_x10;   // HDOP * 10 (cap 255)
  uint8_t  batt_pct;   // 0-100
  uint16_t batt_mv;    // tensione mV
  uint16_t seq;        // contatore pacchetti
};
#pragma pack(pop)

// ---------------- COMANDI DAL PALMARE -------------------------
// v2: aggiunto il campo p1 (parametro) e i comandi CMD_SET_SF/CMD_SET_PWR.
// Struct e versione DEVONO restare identiche byte-per-byte su collare e
// palmare (vedi CmdPacket nel palmare) - se cambi un campo qui, cambialo
// anche li'.
static const uint16_t CMD_MAGIC = 0xACDC;
enum : uint8_t {
  CMD_ACK = 1, CMD_STANDBY = 2, CMD_WAKE = 3,
  CMD_SET_SF  = 4,   // p1 = nuovo SF (7-12), broadcast (collar_id=0)
  CMD_SET_PWR = 5,   // p1 = nuova potenza TX del collare in dBm, mirato
};
#pragma pack(push, 1)
struct CmdPacket {
  uint16_t magic;      // CMD_MAGIC
  uint8_t  version;    // 2
  uint8_t  cmd;        // CMD_*
  uint32_t collar_id;  // destinatario (0 = broadcast)
  uint16_t seq;
  uint8_t  p1;         // parametro, dipende da cmd (0 se non usato)
};
#pragma pack(pop)

// ---------------- OGGETTI GLOBALI ----------------------------
SPIClass  loraSPI(HSPI);
SX1262    radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, loraSPI);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
MPU6050   mpu;

static uint32_t collarId = 0;   // derivato dal MAC, univoco per chip
static uint16_t txSeq = 0;
static uint32_t lastTxMs = 0;
static uint32_t lastMotionMs = 0;
static bool     mpuOk = false;
static float    lastAccMag = 1.0f;

// Link col palmare (via ACK nella finestra RX dopo ogni TX)
static uint32_t lastAckMs = 0;
static bool     everAcked = false;
static uint32_t lastLinkBeepMs = 0;
static bool     wokeByRadio = false;    // sveglia da standby via palmare

// ---------------- SF/POTENZA REGOLABILI (persistenti) ---------
// Prima erano costanti fisse: SF deve combaciare col palmare o il
// collegamento si rompe del tutto, quindi ogni cambio passa da qui - MAI
// applicato direttamente senza il meccanismo di prova/rollback qui sotto.
// Persistiti su NVS (non c'era prima sul collare: solo RTC_DATA_ATTR, che
// sopravvive allo standby ma non a uno stacco batteria) cosi' un valore
// confermato resta valido anche dopo un cambio batteria.
Preferences radioPrefs;
static uint8_t  currentSF  = LORA_SF;
static int8_t   currentPwr = LORA_PWR_DBM;
// Cambio SF "in prova": 0 = nessuno in corso. Se non si riceve nessuna
// risposta valida dal palmare entro SF_TRIAL_MS dall'applicazione, si torna
// da soli al vecchio SF - il collare non puo' restare orfano in mezzo al
// bosco per un comando che non e' mai arrivato a destinazione.
static uint8_t  sfTrialOld = 0;
static uint32_t sfTrialStartMs = 0;
static const uint32_t SF_TRIAL_MS = 60000;
// Rete di sicurezza: se non sento il palmare da diversi minuti (batteria
// tolta e SF tornato al default, comando CMD_SET_SF perso mentre ero fuori
// portata, ecc.), provo in sequenza gli altri SF finche' non torno in
// contatto - cosi' un palmare rimasto "avanti" mi ritrova da solo, senza
// bisogno di intervento manuale. 0 = nessun ciclo di recupero in corso.
static uint8_t  sfRecoveryCandidate = 0;
static uint32_t sfRecoveryStepMs = 0;
static const uint32_t SF_LOST_MS = 5UL * 60UL * 1000UL;
static const uint32_t SF_RECOVERY_STEP_MS = 20000;

static void loadRadioCfg() {
  radioPrefs.begin("collare", true);
  currentSF  = radioPrefs.getUChar("sf", LORA_SF);
  currentPwr = (int8_t)radioPrefs.getUChar("pwr", (uint8_t)LORA_PWR_DBM);
  radioPrefs.end();
  if (currentSF < 7 || currentSF > 12) currentSF = LORA_SF;
}
static void saveSF(uint8_t sf) {
  radioPrefs.begin("collare", false);
  radioPrefs.putUChar("sf", sf);
  radioPrefs.end();
}
static void savePwr(int8_t pwr) {
  radioPrefs.begin("collare", false);
  radioPrefs.putUChar("pwr", (uint8_t)pwr);
  radioPrefs.end();
}

// Finestra di ascolto ACK: deve coprire il tempo di trasmissione del
// CmdPacket di risposta del palmare (11 byte, BW125/CR4-5/preambolo 12),
// che cresce esponenzialmente con SF - a 300ms fissi (il valore di prima,
// giusto per SF7/8) un SF10 non farebbe MAI in tempo a rientrare nella
// finestra: il collare non riceverebbe piu' l'ACK pur essendo collegato.
// Valori = airtime calcolato (formula Semtech AN1200.13) * 1.5 di margine
// per jitter/drift, + 50ms di margine fisso per il turnaround software,
// arrotondati: SF7 45->150, SF8 91->300 (invariato), SF9 161->400,
// SF10 322->650, SF11 643->1200, SF12 1286->2200.
static uint32_t ackWindowMs(uint8_t sf) {
  switch (sf) {
    case 7:  return 150;
    case 8:  return 300;
    case 9:  return 400;
    case 10: return 650;
    case 11: return 1200;
    default: return 2200;   // 12
  }
}

// Duty cycle: la banda EU 869.525MHz e' limitata al 10% di tempo in
// trasmissione. TX_FAST_MS (2.5s, usato col cane in movimento) va benissimo
// a SF7/8, ma a SF9 il pacchetto GPS (29 byte) dura gia' ~243ms - quasi il
// 10% di 2.5s, zero margine - e da SF10 in su lo sfora chiaramente se il
// cane corre a lungo senza fermarsi (tutt'altro che raro). Questa funzione
// alza il pavimento dell'intervallo "veloce" con l'SF, tenendo il duty
// cycle sotto l'8% (margine di sicurezza, non il 10% esatto) - calcolato
// come airtime del pacchetto (formula Semtech AN1200.13) / 0.08, arrotondato
// per eccesso. A SF7/8 il valore e' sotto i 2.5s normali quindi non cambia
// nulla (si prende sempre il piu' grande tra questo e TX_FAST_MS); il
// refresh rallenta automaticamente solo agli SF alti, dove serve davvero.
static uint32_t minFastIntervalMs(uint8_t sf) {
  switch (sf) {
    case 7:  return 900;
    case 8:  return 1700;
    case 9:  return 3100;
    case 10: return 5600;
    case 11: return 12200;
    default: return 22300;   // 12
  }
}

// Stato FERMA (cane in punta: era attivo, ora immobile)
static bool     inFerma = false;
static bool     wasActive = false;
static uint32_t stillSinceMs = 0;
static const uint32_t FERMA_STILL_MS = 8000;    // immobilita' minima per FERMA
static const uint32_t FERMA_MAX_MS   = 180000;  // dopo 3 min = riposo, non ferma

volatile bool loraRxFlag = false;
void IRAM_ATTR collarLoraISR() { loraRxFlag = true; }

// ---------------- FINESTRA COMANDI ----------------------------
// Dopo ogni TX il collare ascolta brevemente: il palmare risponde
// con ACK (link ok) o un comando (es. STANDBY).
static bool waitCmd(uint32_t windowMs, CmdPacket &out) {
  loraRxFlag = false;
  radio.startReceive();
  uint32_t t0 = millis();
  while (millis() - t0 < windowMs) {
    if (loraRxFlag) {
      loraRxFlag = false;
      int st = radio.readData((uint8_t*)&out, sizeof(out));
      if (st == RADIOLIB_ERR_NONE && out.magic == CMD_MAGIC && out.version == 2 &&
          (out.collar_id == collarId || out.collar_id == 0)) {
        radio.standby();
        return true;
      }
      radio.startReceive();      // pacchetto non per me: continua ad ascoltare
    }
    delay(2);
  }
  radio.standby();
  return false;
}

// Standby "spento da palmare": ESP in deep sleep, SX1262 in sniffing
// autonomo a bassissimo consumo. Si risveglia col burst radio del
// palmare (DIO1) oppure col magnete (reed). Il magnete tenuto invece
// spegne del tutto (rtcSleepMode=0: sveglia SOLO a magnete).
static void beepStandby();   // scala discendente: vado in standby

static void enterStandby() {
  Serial.println(F("[COLLARE] standby radio (sveglia da palmare o magnete)"));
  beepStandby();
  radio.standby();
  // Qui la radio resta ad ascoltare (sniffing) per svegliarsi dal palmare -
  // senza il FEM lo fa con la sola sensibilita' nativa dell'SX1262 (senza
  // LNA), non piu' sordo di prima che aggiungessimo il fix, solo senza il
  // bonus extra durante lo standby.
  powerDownPeripherals(false);
  rtcSleepMode = 1;
  radio.startReceiveDutyCycleAuto();         // sniffing preambolo autonomo
  // In deep sleep i GPIO non tenuti fluttuano: NSS deve restare ALTO
  // o l'SX1262 vede SPI spuria e lo sniffing si pianta.
  gpio_hold_en((gpio_num_t)PIN_LORA_NSS);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_LORA_DIO1, 1);          // radio
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_REED, ESP_EXT1_WAKEUP_ALL_LOW); // magnete
  esp_deep_sleep_start();
}

// Torna a dormire in standby SENZA rifare tutta enterStandby() (niente
// beep, niente radio.standby()/startReceiveDutyCycleAuto() - il chip resta
// gia' cosi' com'era, non lo si tocca): serve solo per i due casi in
// setup() in cui ci si sveglia da standby ma si scopre che non era il
// gesto giusto (falso allarme radio, o magnete tolto troppo presto) e si
// deve semplicemente rimettere gli stessi hold/wakeup di prima.
//
// BUG CORRETTO: prima non chiamava powerDownPeripherals() - il PA esterno,
// riacceso incondizionatamente a inizio setup() (vedi commento li'), tornava
// a dormire ancora acceso ogni volta che si passava da qui, vanificando lo
// scopo dello standby a basso consumo. Capita in due casi reali: falso
// allarme radio durante lo standby, o magnete toccato troppo poco.
static void resumeStandbySleep() {
  powerDownPeripherals(false);
  gpio_hold_en((gpio_num_t)PIN_LORA_NSS);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_LORA_DIO1, 1);
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_REED, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_deep_sleep_start();
}

// ---------------- BATTERIA -----------------------------------
// Heltec V3: VBAT -> partitore 390k/100k -> GPIO1, abilitato da GPIO37 LOW
static uint16_t readBatteryMv() {
  digitalWrite(PIN_ADC_CTRL, LOW);
  delay(10);
  // 12 campioni invece di 8, media troncata (si scartano il piu' alto e il
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
  raw /= (N - 2);
  // Rapporto ritarato su hardware reale: la vecchia taratura (5.06, da
  // multimetro 4.06V contro raw=803) sottostimava la tensione di circa il
  // 16% - riverificato con multimetro 3.56V contro rawMv=606 letto qui ->
  // 3560/606 = 5.87.
  return (uint16_t)(raw * 5.87f);
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

// ---------------- MOVIMENTO ----------------------------------
// CORRETTO in v2.2: campionamento a intervallo fisso (100ms).
// Prima veniva letto ogni ciclo (~20ms): il delta tra letture cosi'
// ravvicinate era troppo piccolo e il trigger risultava insensibile.
static bool isMoving() {
  if (!mpuOk) return true;  // senza IMU restiamo sempre in fast (fail-safe)
  static uint32_t lastSampleMs = 0;
  if (millis() - lastSampleMs >= 100) {
    lastSampleMs = millis();
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float gx = ax / 16384.0f, gy = ay / 16384.0f, gz = az / 16384.0f;
    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    float delta = fabsf(mag - lastAccMag);
    lastAccMag = mag;
    if (delta > MOTION_G_THRESH) lastMotionMs = millis();
  }
  return (millis() - lastMotionMs) < MOTION_HOLD_MS;
}

// ---------------- BUZZER DI STATO -----------------------------
// 1 beep  = accensione avvenuta
// 3 beep  = spegnimento in corso
static void beepN(int n, int freq, int durMs, int gapMs) {
  for (int i = 0; i < n; i++) {
    tone(PIN_BUZZER, freq, durMs);
    delay(durMs + gapMs);
  }
}

// Standby via radio: scala DISCENDENTE (alto->basso, "vado a dormire")
static void beepStandby() {
  tone(PIN_BUZZER, 2200, 140); delay(200);
  tone(PIN_BUZZER, 1600, 140); delay(200);
  tone(PIN_BUZZER, 1000, 260); delay(300);
}
// Sveglia via radio: scala ASCENDENTE (basso->alto, "mi sveglio")
static void beepWake() {
  tone(PIN_BUZZER, 1000, 140); delay(200);
  tone(PIN_BUZZER, 2000, 220); delay(260);
}

#ifdef BOARD_V4
// Fix per il FEM KCT8103L della V4.3 (vedi nota sui pin PIN_PA_* piu' sopra
// per la scoperta che GPIO5 = CTX, non un GPIO libero):
//  - CTX (GPIO5) va lasciato gestire in automatico da RadioLib solo durante
//    il burst di TX (setRfSwitchPins: HIGH in TX, LOW il resto del tempo) -
//    da fermo (LOW) il front-end resta in modalita' LNA, che secondo il
//    datasheet KCT8103L da' 21dB di guadagno in piu' in ricezione. Tenuto
//    fisso HIGH (come lo lasciavamo prima, senza saperlo) il ricevitore
//    restava sempre in bypass, senza quel guadagno - il vero buco di
//    portata che inseguivamo.
//  - patch al registro 0x8B5 del chip (bit0=1): indicata da un ingegnere
//    Heltec per la sensibilita' RX sui front-end esterni V4, misurata (in
//    un altro progetto) come miglioramento dal 55% al 25% di pacchetti
//    persi. Richiede RADIOLIB_LOW_LEVEL=1 nei build_flags (altrimenti
//    readRegister/writeRegister restano protected e non compila). Un
//    freeze del palmare era stato inizialmente attribuito a questa riga,
//    ma la causa reale era un GND scollegato durante lo spostamento del
//    filo dell'encoder - la patch e' innocua (il codice RadioLib che la
//    applica ha un timeout di sicurezza di 1s, non puo' bloccare per
//    sempre).
static void applyFemFixV4() {
  radio.setRfSwitchPins(RADIOLIB_NC, PIN_PA_CTX);
  uint8_t r = 0;
  radio.readRegister(0x8B5, &r, 1);
  r |= 0x01;
  radio.writeRegister(0x8B5, &r, 1);
  // begin() imposta il limite di corrente del PA interno a soli 60mA - con
  // il PA esterno acceso il chip puo' autolimitarsi in TX prima di arrivare
  // alla potenza richiesta. 140 e' il massimo consentito (vedi commento
  // setCurrentLimit in RadioLib) ed e' il valore usato dal riferimento
  // MeshCore per questa stessa scheda.
  radio.setCurrentLimit(140.0);
}
#endif
// Guadagno RX aumentato (sezione 9.6 del datasheet SX126x) - a differenza
// della patch sopra e' una feature ufficiale del chip, non "undocumented".
// Costa un filo di corrente in piu' in ricezione, non in trasmissione.
static void applyRxBoostedGain() {
  radio.setRxBoostedGainMode(true);
}

// ---------------- SETUP --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(PIN_REED, INPUT_PULLUP);
  // Rilascia l'hold lasciato da powerDownPeripherals() prima di dormire -
  // senza questo i digitalWrite() sotto (e quello su VEXT/ADC_CTRL piu' in
  // basso in setup(), e il reset radio dentro radio.begin()) non
  // avrebbero effetto: il pin resterebbe agganciato al valore di prima
  // del sonno.
  gpio_hold_dis((gpio_num_t)PIN_VEXT);
  gpio_hold_dis((gpio_num_t)PIN_ADC_CTRL);
  gpio_hold_dis((gpio_num_t)PIN_LORA_RST);
#ifdef BOARD_V4
  gpio_hold_dis((gpio_num_t)PIN_PA_VFEM);
  gpio_hold_dis((gpio_num_t)PIN_PA_CSD);
  // Accendi il PA esterno PRIMA di qualunque radio.begin() qui sotto
  // (compreso quello nel ramo "sveglia da standby" poco piu' in basso).
  // GPIO5 (CTX) NON va qui: lo gestisce RadioLib da solo, vedi applyFemFixV4().
  pinMode(PIN_PA_VFEM, OUTPUT); digitalWrite(PIN_PA_VFEM, HIGH);
  pinMode(PIN_PA_CSD,  OUTPUT); digitalWrite(PIN_PA_CSD,  HIGH);
#endif
  loadRadioCfg();   // SF/potenza persistiti: servono PRIMA di ogni radio.begin() qui sotto

  gpio_hold_dis((gpio_num_t)PIN_LORA_NSS);   // rilascia l'hold del deep sleep
  gpio_deep_sleep_hold_dis();

  // --- Sveglia dallo STANDBY via radio (DIO1)? Verifica il burst ---
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0 && rtcSleepMode == 1) {
    loraSPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
    int st = radio.begin(LORA_FREQ, LORA_BW, currentSF, LORA_CR, LORA_SYNC, currentPwr, LORA_PREAMB, LORA_TCXO_V, false);
    if (st == RADIOLIB_ERR_NONE) {
      radio.setCRC(true);
      radio.setDio2AsRfSwitch(true);
      applyRxBoostedGain();
#ifdef BOARD_V4
      applyFemFixV4();
#endif
      radio.setPacketReceivedAction(collarLoraISR);
      uint64_t mac = ESP.getEfuseMac();
      collarId = (uint32_t)(mac ^ (mac >> 32));
      CmdPacket c;
      // il palmare trasmette il burst per ~4s: la finestra scala con SF
      // (a SF alti il singolo pacchetto dura di piu', vedi ackWindowMs()).
      if (!(waitCmd(max((uint32_t)1800, ackWindowMs(currentSF) + 500), c) && c.cmd == CMD_WAKE)) {
        // falso allarme (altro traffico LoRa): torna a dormire in standby.
        // (prima qui mancava il gpio_hold_en() rifatto da resumeStandbySleep()
        // - rilasciato a inizio setup() e mai piu' riarmato, il che rischiava
        // di far vedere all'SX1262 SPI spuria durante lo sniffing successivo)
        radio.startReceiveDutyCycleAuto();
        resumeStandbySleep();
      }
      wokeByRadio = true;   // burst valido: prosegue il boot completo
    }
  } else if (rtcSleepMode == 1) {
    // Sveglia da STANDBY ma non via radio (quindi via magnete, unica altra
    // sorgente armata): qui il magnete serve per spegnere del tutto, non
    // per far ripartire il collare - la sveglia "normale" da standby arriva
    // gia' in automatico dal burst radio del palmare (ramo sopra). Stesso
    // gesto/durata dello spegnimento da acceso (3s) cosi' il magnete si
    // comporta sempre allo stesso modo: "spegni", non "accendi e poi
    // rispegni a parte".
    uint32_t t0 = millis();
    while (digitalRead(PIN_REED) == LOW) {
      if (millis() - t0 >= 3000) {
        pinMode(PIN_BUZZER, OUTPUT);
        beepN(3);
        // NIENTE radio.sleep() qui (bug trovato in revisione): in questo
        // ramo (sveglia da standby via magnete, non via radio) la radio
        // non e' mai stata re-inizializzata in questo risveglio - ne'
        // loraSPI.begin() ne' radio.begin() sono mai stati chiamati - una
        // radio.sleep() qui manda un comando SPI a bus/chip non
        // configurati in questo boot, rischiando di bloccarsi e non
        // arrivare mai al vero spegnimento sotto. goToSleep() gestisce
        // gia' la radio in modo robusto (reset hardware via RST, non SPI -
        // vedi commento su powerDownPeripherals()), quindi qui non serve.
        digitalWrite(PIN_VEXT, HIGH);
        while (digitalRead(PIN_REED) == LOW) delay(50);
        delay(100);
        goToSleep();
      }
      delay(20);
    }
    resumeStandbySleep();   // magnete tolto troppo presto: torna in standby, non spegnere e non accendere
  } else {
    confirmPowerOnCollare();     // accensione a magnete (2s, o reset di fabbrica a 10s)
  }
  rtcSleepMode = 0;

  if (factoryResetPending) {
    factoryResetPending = false;
    currentSF = LORA_SF; currentPwr = LORA_PWR_DBM;
    saveSF(currentSF); savePwr(currentPwr);
    sfTrialOld = 0; sfRecoveryCandidate = 0;
  }

  Serial.begin(115200);
  delay(300);
  uint64_t mac = ESP.getEfuseMac();
  collarId = (uint32_t)(mac ^ (mac >> 32));   // 32 bit univoci dal MAC
  Serial.printf("\n[COLLARE] boot - ID: %08X\n", collarId);

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);          // accendi Vext (alimenta GPS se collegato a Vext)
  pinMode(PIN_ADC_CTRL, OUTPUT);
  digitalWrite(PIN_ADC_CTRL, HIGH);
  pinMode(PIN_BUZZER, OUTPUT);
  if (wokeByRadio) beepWake();          // scala su: svegliato dal palmare
  else             beepN(1);            // 1 beep: acceso a magnete

  // GPS - il modulo montato (NEO-M9N) parla a 38400 baud, non 9600 come i
  // moduli precedenti (M8N). Confermato sul palmare con una scansione
  // diagnostica (stesso modulo, stessa velocita' su entrambe le schede).
  gpsSerial.begin(38400, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // IMU
  Wire.begin(PIN_SDA, PIN_SCL);
  mpu.initialize();
  mpuOk = mpu.testConnection();
  Serial.printf("[COLLARE] MPU6050: %s\n", mpuOk ? "OK" : "NON TROVATO (resto in fast TX)");
  if (mpuOk) {
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);
  }

  // LoRa
  // Se il risveglio e' arrivato dal burst radio del palmare, radio.begin()
  // e' gia' stato chiamato con successo nel ramo "Sveglia dallo STANDBY via
  // radio" qui sopra (wokeByRadio=true solo li', a burst confermato) -
  // rifarlo qui ripeterebbe da capo la calibrazione interna dell'SX1262
  // (immagine/DC-DC) per niente, ad ogni singolo risveglio da standby
  // (trovato in revisione: sprecava tempo e corrente esattamente sul
  // percorso che il design a basso consumo dovrebbe minimizzare).
  int st = RADIOLIB_ERR_NONE;
  if (!wokeByRadio) {
    loraSPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
    st = radio.begin(LORA_FREQ, LORA_BW, currentSF, LORA_CR, LORA_SYNC, currentPwr, LORA_PREAMB, LORA_TCXO_V, false);
  }
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[COLLARE] LoRa init FALLITO, code %d\n", st);
    // Prima era un `while(true) delay(1000)` cieco: un collare senza radio
    // restava acceso e sordo per sempre, spegnibile solo staccando la
    // batteria (stesso bug gia' risolto sul palmare). Qui riprova ogni 3s
    // e nel frattempo continua a leggere il magnete (3s tenuto = spegni),
    // cosi' resta comunque una via d'uscita anche in questo stato.
    uint32_t reedStart = 0, lastRetryMs = millis();
    while (st != RADIOLIB_ERR_NONE) {
      if (digitalRead(PIN_REED) == LOW) {
        if (reedStart == 0) reedStart = millis();
        if (millis() - reedStart >= 3000) {
          beepN(3);
          digitalWrite(PIN_VEXT, HIGH);
          while (digitalRead(PIN_REED) == LOW) delay(50);
          delay(100);
          goToSleep();
        }
      } else {
        reedStart = 0;
      }
      if (millis() - lastRetryMs >= 3000) {
        lastRetryMs = millis();
        st = radio.begin(LORA_FREQ, LORA_BW, currentSF, LORA_CR, LORA_SYNC, currentPwr, LORA_PREAMB, LORA_TCXO_V, false);
      }
      delay(20);
    }
  }
  radio.setCRC(true);
  radio.setDio2AsRfSwitch(true);      // richiesto dalle board Heltec V3 / cloni
  applyRxBoostedGain();
#ifdef BOARD_V4
  applyFemFixV4();
#endif
  radio.setPacketReceivedAction(collarLoraISR);
  // SF e potenza persistono su NVS e possono essere stati abbassati da un
  // comando CMD_SET_PWR/CMD_SET_SF passato (anche per sbaglio, dal menu
  // "Cani -> azioni collare" sul palmare) - senza questo log resterebbe
  // invisibile, il collare non lo segnalava mai prima.
  Serial.printf("[COLLARE] LoRa OK - SF=%u pwr=%ddBm\n", currentSF, currentPwr);

  lastMotionMs = millis();            // parte in modalita' fast
}

// ---------------- LOOP ---------------------------------------
void loop() {
  // ---- SPEGNIMENTO: magnete tenuto sul case per 3 secondi -----
  static uint32_t reedStart = 0;
  if (digitalRead(PIN_REED) == LOW) {
    if (reedStart == 0) reedStart = millis();
    if (millis() - reedStart >= 3000) {
      Serial.println(F("[COLLARE] spegnimento (magnete)"));
      beepN(3);                                // 3 beep: mi sto spegnendo
      radio.sleep();
      digitalWrite(PIN_VEXT, HIGH);            // spegni GPS
      while (digitalRead(PIN_REED) == LOW) delay(50);  // attendi rimozione magnete
      delay(100);
      goToSleep();
    }
  } else {
    reedStart = 0;
  }

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  bool moving = isMoving();
  bool fixNow = gps.location.isValid() && gps.location.age() < 5000;

  // ---- Rilevamento FERMA: era in azione, ora immobile -----------
  bool txNow = false;
  float spd = (fixNow && gps.speed.isValid()) ? gps.speed.kmph() : 0;
  if (moving) {
    wasActive = true;
    stillSinceMs = 0;
    if (inFerma) { inFerma = false; txNow = true; }   // ferma finita: avvisa subito
  } else {
    if (stillSinceMs == 0) stillSinceMs = millis();
    if (wasActive && !inFerma &&
        millis() - stillSinceMs >= FERMA_STILL_MS && spd < 1.0f) {
      inFerma = true; txNow = true;                    // FERMA! avvisa subito
    }
    if (inFerma && millis() - stillSinceMs >= FERMA_MAX_MS) {
      inFerma = false; wasActive = false; txNow = true; // 3+ min = riposo
    }
  }

  // ---- Beep link perso col palmare (2 beep ogni 120s) ------------
  if (everAcked && millis() - lastAckMs > 60000 &&
      millis() - lastLinkBeepMs > 120000) {
    lastLinkBeepMs = millis();
    beepN(2, 1600, 150, 180);
  }

  // Batteria: controlla ogni 60s (evita letture ADC continue)
  static uint32_t lastBattMs = 0;
  static uint16_t battMv = 3900;
  // Rilevamento carica: nessun pin STAT/CHG dedicato. Una soglia fissa NON
  // funziona - una batteria quasi piena e SCOLLEGATA riposa gia' sopra 4V, e
  // subito dopo lo stacco ci resta ancora un po' (effetto "surface charge"
  // della cella al litio) prima di scendere: qualunque soglia fissa prima o
  // poi la becca come falsa carica. L'unico segnale affidabile e' la
  // TENDENZA: in carica la tensione sale in modo continuo nei minuti; da
  // scollegata, sotto il carico normale della scheda (radio/GPS/display),
  // crolla subito di decine di mV perche' non c'e' piu' corrente del
  // caricatore a compensare l'assorbimento. Quindi: ACCENDI se sale in modo
  // sostenuto su una finestra di qualche minuto, SPEGNI appena scende
  // chiaramente dal massimo visto (niente bisogno di aspettare una discesa
  // "lenta": lo stacco e' un evento netto, non graduale).
  static uint16_t battPeakMv = 0;
  static uint16_t battRefMv = 0;
  static uint32_t battRefMs = 0;
  static bool charging = false;
  const uint32_t BATT_TREND_WINDOW_MS = 180000;  // 3 minuti
  if (millis() - lastBattMs >= 60000 || lastBattMs == 0) {
    lastBattMs = millis();
    battMv = readBatteryMv();
    if (battMv > battPeakMv) battPeakMv = battMv;
    if (battRefMs == 0) { battRefMv = battMv; battRefMs = millis(); }
    if (millis() - battRefMs >= BATT_TREND_WINDOW_MS) {
      if (!charging && (int32_t)battMv - (int32_t)battRefMv >= 12) charging = true;
      battRefMv = battMv;
      battRefMs = millis();
    }
    if (charging && (int32_t)battPeakMv - (int32_t)battMv >= 30) {
      charging = false;
      battPeakMv = battMv;
      battRefMv = battMv;
      battRefMs = millis();
    }
  }
  bool lowBatt = battPercent(battMv) < 15;

  // Link perso (stessa soglia dei 60s usata sopra per il beep): anche da
  // fermo passa all'intervallo veloce, per ritrovare il link prima invece
  // di aspettare fino a 30s tra un tentativo e l'altro. Solo per 15 minuti
  // pero': se il link resta perso piu' a lungo (fuori portata per un bel
  // po', non un buco momentaneo) si torna al ritmo lento normale, per non
  // consumare la batteria trasmettendo veloce a vuoto per ore.
  const uint32_t LINK_LOST_FAST_WINDOW_MS = 15UL * 60000UL;
  uint32_t sinceAckMs = millis() - lastAckMs;
  bool linkLost = everAcked && sinceAckMs > 60000 &&
                  sinceAckMs <= 60000 + LINK_LOST_FAST_WINDOW_MS;
  // Con batteria scarica raddoppia gli intervalli: meglio un refresh
  // piu' lento che un collare morto a meta' battuta.
  uint32_t interval = (moving || linkLost) ? max(TX_FAST_MS, minFastIntervalMs(currentSF)) : TX_SLOW_MS;
  if (lowBatt) interval *= 2;

  if (txNow || millis() - lastTxMs >= interval) {
    lastTxMs = millis();

    CollarPacket pkt = {};
    pkt.magic     = PKT_MAGIC;
    pkt.version   = 4;
    pkt.collar_id = collarId;
    pkt.seq       = txSeq++;

    if (fixNow)  pkt.flags |= 0x01;
    if (moving)  pkt.flags |= 0x02;
    if (lowBatt) pkt.flags |= 0x04;
    if (inFerma) pkt.flags |= 0x08;
    if (charging) pkt.flags |= 0x10;

    // Satelliti "in uso" (campo del GGA): TinyGPSPlus lo aggiorna appena
    // arriva una sentenza valida, indipendentemente dal fix. Lo mandiamo
    // sempre, non solo con fixNow, cosi' sul palmare/web si vede "vedo 3
    // satelliti ma non ancora abbastanza per un fix" invece di un secco 0
    // indistinguibile da "GPS spento/non risponde".
    pkt.sats = (uint8_t)gps.satellites.value();

    if (fixNow) {
      pkt.lat_e7 = (int32_t)(gps.location.lat() * 1e7);
      pkt.lon_e7 = (int32_t)(gps.location.lng() * 1e7);
      pkt.alt_m  = gps.altitude.isValid() ? (uint16_t)constrain(gps.altitude.meters(), 0, 65535) : 0;
      pkt.hdop_x10 = (uint8_t)constrain(gps.hdop.hdop() * 10.0, 0, 255);
      if (gps.speed.isValid())
        pkt.speed_x10 = (uint16_t)constrain(gps.speed.kmph() * 10.0, 0, 65535);
      if (gps.course.isValid())
        pkt.course_deg = (uint16_t)gps.course.deg() % 360;
    }

    pkt.batt_mv  = battMv;
    pkt.batt_pct = battPercent(battMv);

    int st = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("[TX] seq=%u fix=%d mov=%d ferma=%d v=%.1f sats=%u batt=%u%% -> %s\n",
                  pkt.seq, fixNow, moving, inFerma, pkt.speed_x10 / 10.0f,
                  pkt.sats, pkt.batt_pct, st == RADIOLIB_ERR_NONE ? "OK" : "ERR");

    // ---- finestra comandi: il palmare risponde con ACK o comando --
    // La finestra scala con SF (vedi ackWindowMs()): a SF alti il pacchetto
    // di risposta del palmare dura di piu', una finestra fissa lo perderebbe.
    CmdPacket cmd;
    if (waitCmd(ackWindowMs(currentSF), cmd)) {
      lastAckMs = millis(); everAcked = true;
      if (sfRecoveryCandidate != 0) {
        // Ero nel ciclo di recupero SF e ho appena ricevuto risposta: questo
        // e' l'SF giusto, lo salvo e smetto di ciclare.
        saveSF(currentSF);
        sfRecoveryCandidate = 0;
      }
      if (cmd.cmd == CMD_STANDBY) {
        enterStandby();   // spento da palmare
      } else if (cmd.cmd == CMD_SET_PWR) {
        // Potenza: indipendente per apparato, nessun rischio di
        // disallineamento - si applica subito, niente prova/rollback.
        currentPwr = (int8_t)cmd.p1;
        radio.setOutputPower(currentPwr);
        savePwr(currentPwr);
      } else if (cmd.cmd == CMD_SET_SF && cmd.p1 >= 7 && cmd.p1 <= 12 &&
                 cmd.p1 != currentSF && sfTrialOld == 0) {
        // SF: deve combaciare sui due apparati o il collegamento si rompe.
        // Confermo SUBITO al vecchio SF (il palmare aspetta questo prima
        // di passare lui stesso al nuovo SF), POI passo al nuovo SF - mai
        // il contrario, altrimenti la conferma non arriverebbe mai.
        CmdPacket ack;
        ack.magic = CMD_MAGIC; ack.version = 2; ack.cmd = CMD_ACK;
        ack.collar_id = collarId; ack.seq = 0; ack.p1 = cmd.p1;
        delay(15);
        radio.transmit((uint8_t*)&ack, sizeof(ack));
        sfTrialOld = currentSF;
        currentSF = cmd.p1;
        radio.setSpreadingFactor(currentSF);
        sfTrialStartMs = millis();
      }
    }

    // Cambio SF in prova: se il palmare torna a farsi sentire (qualsiasi
    // scambio valido) DOPO il passaggio al nuovo SF, e' la conferma che
    // anche lui e' passato al nuovo SF - si salva su NVS e la prova finisce.
    // Se entro SF_TRIAL_MS non arriva nulla, si torna da soli al vecchio SF:
    // nessuno resta orfano per un comando che non e' arrivato a destinazione.
    if (sfTrialOld != 0) {
      if (everAcked && lastAckMs >= sfTrialStartMs) {
        saveSF(currentSF);
        sfTrialOld = 0;
      } else if (millis() - sfTrialStartMs >= SF_TRIAL_MS) {
        currentSF = sfTrialOld;
        radio.setSpreadingFactor(currentSF);
        sfTrialOld = 0;
      }
    }

    // Ciclo di recupero SF: nessun contatto col palmare da SF_LOST_MS e
    // nessun cambio SF gia' in corso -> provo un altro SF ogni
    // SF_RECOVERY_STEP_MS, in sequenza 7..12, finche' non rispondono.
    bool lostLong = everAcked ? (millis() - lastAckMs > SF_LOST_MS)
                               : (millis() > SF_LOST_MS);
    if (lostLong && sfTrialOld == 0) {
      if (sfRecoveryCandidate == 0 || millis() - sfRecoveryStepMs >= SF_RECOVERY_STEP_MS) {
        currentSF = (currentSF >= 12) ? 7 : currentSF + 1;
        sfRecoveryCandidate = currentSF;
        radio.setSpreadingFactor(currentSF);
        sfRecoveryStepMs = millis();
      }
    } else if (sfRecoveryCandidate != 0 && !lostLong) {
      sfRecoveryCandidate = 0;   // tornato in contatto per altra via
    }
  }

  delay(20);
}
