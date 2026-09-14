# PiZaff DogTrack — firmware collare + palmare

Sistema di localizzazione per cani da caccia/ricerca, stile Garmin Astro, tutto
autocostruito su schede **Heltec WiFi LoRa 32 (V3 / V4)**.

- **Collare** — modulo GPS + LoRa che il cane porta al collo. Legge la
  posizione, la trasmette via radio, dorme il più possibile per durare a
  batteria.
- **Palmare** — ricevitore con display OLED, encoder e bussola. Mostra fino a
  **6 cani** contemporaneamente su schermata "freccia + distanza", tiene la
  traccia di ognuno e la può esportare in GPX da una mini pagina web.

Comunicazione **LoRa punto-punto a 869 MHz** (banda EU, rete privata), nessun
server, nessuna SIM, nessuna copertura cellulare richiesta.

---

## Contenuto del repository

| Percorso | Cosa contiene |
|---|---|
| `collare/` | Progetto PlatformIO del collare (`src/main.cpp`, ~1000 righe) |
| `palmare/` | Progetto PlatformIO del palmare (`src/main.cpp`, ~3100 righe) + tabelle partizioni |
| `palmare dogtrk.stl` / `.3mf` | Case del palmare pronto per la stampa 3D |
| `PiZaff_DogTrack_Manuale_A5 (2).pdf` | Manuale utente (formato A5) |
| `PiZaff_DogTrack_Libretto (1).pdf` | Libretto di istruzioni |
| `PiZaff_DogTracker_Schema (3).html` | Schema dei collegamenti (apribile nel browser) |
| `demo_gui.html` / `demo_oled.html` | Anteprima dell'interfaccia palmare, da aprire nel browser |

---

## Hardware

### Collare
- Heltec WiFi LoRa 32 **V3** (ESP32-S3, SX1262, 8 MB flash) **oppure V4**
  (16 MB flash, PA RF esterno ~28 dBm)
- GPS **u-blox NEO-M8N** (UART)
- Accelerometro **MPU6050** (I²C) — rileva se il cane si muove
- **Reed switch** + magnete — accensione/spegnimento senza pulsanti né fori
- Buzzer (feedback sonoro su accensione / reset)
- Batteria LiPo 1S

### Palmare
- Heltec WiFi LoRa 32 **V3** oppure **V4** (la V4 ha 2 MB di PSRAM: buffer
  traccia da 300 → 5000 punti per cane)
- GPS **u-blox NEO-M8N** (UART)
- Bussola **QMC5883L** (I²C) — anche quella integrata nei moduli "M8N + compass"
- Display **OLED 128×64** SSD1309/SSD1306 (SPI), es. Waveshare 2.42"
- **Encoder rotativo KY-040** con pulsante — unico controllo
- Batteria LiPo 1S

I pinout completi (con le differenze V3/V4) sono commentati in testa ai
rispettivi `src/main.cpp`. Lo schema visuale è in
`PiZaff_DogTracker_Schema (3).html`.

> **Nota V4:** su V3 e V4 alcuni GPIO cambiano funzione (sulla V4 sono
> riservati all'amplificatore RF, al connettore GNSS e al reset OLED
> integrati). Il firmware se ne occupa da solo tramite il flag `BOARD_V4`;
> basta scegliere l'ambiente di build giusto.

---

## Compilazione e flash

### Con PlatformIO (consigliato)

Installa l'estensione **PlatformIO** in VS Code, poi apri **la cartella
`collare/` o `palmare/`** (non la root del repo — sono due progetti separati).

Collare:

```bash
cd collare
pio run -e collare_v4 -t upload   # Heltec V4 (16 MB, PA esterno)
pio run -e collare_v3 -t upload   # Heltec V3 (8 MB)
```

Palmare:

```bash
cd palmare
pio run -e palmare_v4 -t upload   # Heltec V4 — usa partitions_pizaff_16mb.csv, PSRAM QSPI
pio run -e palmare_v3 -t upload   # Heltec V3 — usa partitions_pizaff.csv
```

Monitor seriale: `pio device monitor` (115200 baud). In VS Code l'ambiente si
sceglie dalla barra di stato in basso.

Le librerie (RadioLib, TinyGPSPlus, MPU6050 / Adafruit SSD1306+GFX,
QMC5883LCompass) vengono scaricate in automatico da `platformio.ini`.

### Con Arduino IDE

Il file `.csv` delle partizioni **non viene letto**. Per il palmare imposta a
mano:

- **Tools → Partition Scheme →** `Huge APP (3MB No OTA/1MB SPIFFS)`
- solo V4: **Tools → PSRAM →** `QSPI PSRAM`
- **Tools → Filesystem →** LittleFS

Il collare sta nella partizione di default, nessuna impostazione particolare.

---

## Primo utilizzo

### Collare (nessun pulsante, si usa il magnete)

| Gesto | Effetto |
|---|---|
| Magnete appoggiato **2 s** da spento | **Accensione** (5 beep) |
| Magnete appoggiato **3 s** da acceso | **Spegnimento** (deep sleep, sveglia solo a magnete) |
| Magnete tenuto **10 s** all'accensione | **Reset di fabbrica** dei parametri radio (SF/potenza) |

Il tempo minimo evita accensioni/spegnimenti accidentali (collare vicino a
oggetti metallici). All'accensione il collare cerca il fix GPS e comincia a
trasmettere; il LED/buzzer danno conferma.

### Palmare (solo l'encoder: ruota per muoverti, click per confermare)

1. **Accendi** premendo il pulsante dell'encoder.
2. Vai su **Menu → Associa**: il palmare elenca gli ID dei collari che sente in
   giro. Seleziona quello del tuo cane e dagli un nome (max 8 caratteri).
   Ripeti per ogni cane (fino a 6).
3. Torna alla **Home**: per ogni cane vedi freccia (direzione relativa alla tua
   bussola), distanza, stato fix, batteria del collare.
4. **Menu → Bussola**: calibrazione bussola (ruota il palmare su sé stesso).
5. **Menu → Mappa**: attiva il WiFi per la mappa live nel browser (vedi sotto).
6. **Menu → Tracce**: elenco delle tracce registrate, cancellazione, export GPX.
7. **Menu → Unità**: metri / piedi.
8. **Menu → Avanzate**: cambio Spreading Factor e potenza TX (vedi "Come
   funziona").

Spegnimento: voce di menu dedicata / click lungo dalla Home (deep sleep,
sveglia col pulsante encoder).

---

## Come funziona

### Il collegamento radio

- **869.525 MHz**, banda 125 kHz, CR 4/5, sync word privata `0xA7`.
- **SF (Spreading Factor) 8** di default — compromesso tra portata e frequenza
  di aggiornamento. Modificabile da palmare (SF 7–12): SF alto = più portata ma
  refresh più lento e più aria occupata.
- Pacchetto **binario compatto** (`CollarPacket`, ~30 byte) con CRC gestito da
  RadioLib: magic, ID collare, lat/lon (1e-7°), quota, velocità, rotta,
  satelliti, HDOP, batteria %, mV, contatore di sequenza + flag
  (fix / in movimento / batteria scarica / fermo / in carica).
- Il palmare risponde con `CmdPacket` (`ACK`, `STANDBY`, `WAKE`,
  `SET_SF`, `SET_PWR`).

### TX adattivo (autonomia del collare)

L'MPU6050 dice se il cane si muove:

| Stato | Intervallo TX |
|---|---|
| In movimento (o entro 15 s dall'ultimo movimento) | **2.5 s** |
| Fermo | **30 s** |

Cane fermo = poche trasmissioni = batteria che dura. Fra un invio e l'altro,
quando può, il collare va in sleep.

### Standby e sveglia da remoto

Dal palmare (**Cani → Standby**) si mette un collare in standby: la radio resta
in ascolto a bassissimo consumo e il collare si risveglia solo quando il
palmare gli manda `WAKE` (**Cani → Sveglia**). Utile per il cane a riposo tra
una battuta e l'altra senza doverlo maneggiare.

### Auto-recupero SF

Se il palmare perde il contatto, prova a ri-sincronizzarsi mandando `SET_SF` in
broadcast sui vari SF finché il collare non risponde. Il reset di fabbrica col
magnete (10 s) è la via di recupero manuale sul collare, che non ha schermo.

### Potenza TX e limiti EU

Sulla **V4** l'amplificatore esterno porta l'uscita a ~28 dBm. L'ERP dipende
dall'antenna:

| Config | ERP stimato | Limite EU (27 dBm ERP) |
|---|---|---|
| V4 + antenna 10 dBi | ~35.9 dBm | usa **12–13 dBm** di TX |
| V4 + antenna FPC 2.5 dBi | ~28.4 dBm | usa **19–20 dBm** di TX |

`LORA_PWR_DBM` nel `main.cpp` del collare (e la voce **Avanzate → Potenza** sul
palmare) regolano questo valore. **Rispetta la normativa del tuo paese.**

> A piena potenza il PA della V4 assorbe parecchia corrente durante l'impulso
> TX: se vedi riavvii casuali in trasmissione è un calo di tensione (batteria
> scarica o cavi/connettori troppo sottili) — scendi di qualche dBm.

### Mappa e tracce (WiFi del palmare)

Con **Menu → Mappa** il palmare accende un access point:

- SSID **`Pi_zaff_sys_DogTrack`**, password **`pizaff123`**
- apri **http://192.168.4.1** nel browser

La pagina mostra la posizione live dei cani e la scia. Le tracce complete sono
salvate su **LittleFS** e scaricabili in **GPX** da `/tracks`. Il buffer in RAM
serve solo a disegnare la scia sulla mappa live (300 punti/cane su V3, 5000 su
V4 grazie alla PSRAM); lo storico su file non ha questo limite.

---

## Struttura dei sorgenti

Entrambi i `main.cpp` hanno in testa un commento lungo che spiega pinout,
differenze V3/V4, e le scelte non ovvie (gestione PA RF esterno, `gpio_hold_en`
prima del deep sleep, perché serve la tabella partizioni custom sul palmare,
ecc.). È la prima cosa da leggere prima di mettere mano al codice.

---

## Licenza

Copyright (C) 2026 Pietro Zaffarano.

Questo progetto è distribuito con licenza **GNU GPL v3.0**: puoi usarlo,
studiarlo, modificarlo e ridistribuirlo, ma ogni derivato che distribuisci
deve restare open source con la stessa licenza — non può essere chiuso in un
prodotto proprietario né essere fatto passare per opera di qualcun altro.
Testo completo in [LICENSE](LICENSE).
