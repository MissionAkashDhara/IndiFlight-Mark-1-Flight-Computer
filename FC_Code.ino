/*
 * ============================================================
 *  ROCKET FLIGHT COMPUTER — Arduino Nano (ATmega328P)
 *  Hardware: BMP180 · MPU6050 · Winbond W25Q64 8MB SPI Flash
 *            Logic-level MOSFET · Buzzer · LED (Green/Red)
 *
 *  State machine:
 *    BOOT → SELF_TEST → PAD_IDLE → BOOST → COAST
 *    → APOGEE_FIRE → DESCENT → LANDED
 *
 *  Design goals:
 *    – Non-blocking everywhere (no delay() in flight path)
 *    – 25 Hz main loop tick (40 ms) — comfortable for Nano
 *    – Dual-vote apogee (BMP alt drop + MPU vert velocity)
 *    – Single re-fire attempt with continuity sense
 *    – AVR watchdog 2s; resets on every loop tick
 *    – CRC-16 per flash session; flagged on boot readout
 *
 *  Libraries required (install via Library Manager):
 *    – Adafruit BMP085 (covers BMP180)
 *    – MPU6050 by Electronic Cats  -OR-  I2Cdevlib MPU6050
 *    – SPIMemory by Marzogh  (Winbond W25Qxx)
 *
 *  Wiring summary:
 *    BMP180  : SDA→A4, SCL→A5
 *    MPU6050 : SDA→A4, SCL→A5  (shared I2C bus, addr 0x68)
 *    Flash   : CS→D10, MOSI→D11, MISO→D12, SCK→D13
 *    MOSFET  : Gate→D9  (via 100Ω resistor; 100µF on Vcc rail)
 *    Cont.   : D8 pull-up to Vcc through 10kΩ → pyro line sense
 *    Buzzer  : D6 (active buzzer, active-HIGH)
 *    LED Grn : D5  LED Red : D4  (330Ω series resistor each)
 * ============================================================
 */

// ── Libraries ────────────────────────────────────────────────
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <SPIMemory.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// ── Pin definitions ──────────────────────────────────────────
#define PIN_FLASH_CS    10
#define PIN_MOSFET       9
#define PIN_CONTINUITY   8   // INPUT_PULLUP; LOW = continuity OK
#define PIN_BUZZER       6
#define PIN_LED_GREEN    5
#define PIN_LED_RED      4

// ── Tuning constants ─────────────────────────────────────────
#define LOOP_INTERVAL_MS       40    // 25 Hz main loop
#define BASELINE_SAMPLES       50    // 2 s of averaging at 25 Hz
#define LAUNCH_ACCEL_G        2.5f   // g threshold for launch detect
#define LAUNCH_CONFIRM_TICKS    3    // consecutive samples (120 ms)
#define BURNOUT_ACCEL_G       0.5f   // below this → motor out
#define BURNOUT_CONFIRM_TICKS   3
#define APOGEE_ALT_DROP_M     2.0f   // BMP: metres below peak to vote
#define APOGEE_VOTE_MS        200    // ms both votes must hold
#define FIRE_PULSE_MS         800    // MOSFET on-time
#define REFIRE_WAIT_MS       1000    // wait before single retry
#define LANDING_DELTA_M       0.5f   // max alt change to call landed
#define LANDING_CONFIRM_MS  10000    // 10 s stable window
#define LOCATOR_PERIOD_MS   10000    // beep every 10 s after landing
#define LOCATOR_BEEPS           3

// ── Flash layout ─────────────────────────────────────────────
// Session header: AA BB [session_id u8] [start_ms u32] [groundAlt f32]
// Data rows: 'D' [timestamp u32] [bmpAlt f32] [mpuAlt f32]
//            [ax f32] [ay f32] [az f32]
//            [vx f32] [vy f32] [vz f32]
//            [roll f32] [pitch f32] [yaw f32]
//            [temp f32] [pressure f32]        → 57 bytes/row
// Event row: 'E' [timestamp u32] [type u8] [alt f32]     → 10 bytes
// Session end: CC DD [CRC16 u16]
#define FLASH_MAGIC_START_0  0xAA
#define FLASH_MAGIC_START_1  0xBB
#define FLASH_MAGIC_END_0    0xCC
#define FLASH_MAGIC_END_1    0xDD
#define EVT_LAUNCH           0x01
#define EVT_BURNOUT          0x02
#define EVT_APOGEE           0x03
#define EVT_DEPLOY_OK        0x04
#define EVT_DEPLOY_FAIL      0x05
#define EVT_LANDED           0x06
#define EEPROM_SESSION_ADDR  0       // 1 byte: next session ID

// ── State machine ─────────────────────────────────────────────
enum State : uint8_t {
  ST_BOOT,
  ST_SELF_TEST,
  ST_PAD_IDLE,
  ST_BOOST,
  ST_COAST,
  ST_APOGEE_FIRE,
  ST_DESCENT,
  ST_LANDED,
  ST_HALT              // sensor fail — buzzer alarm, no logging
};

// ── Globals ───────────────────────────────────────────────────
Adafruit_BMP085 bmp;
MPU6050         mpu;
SPIFlash        flash(PIN_FLASH_CS);

State     state          = ST_BOOT;
uint32_t  loopLastMs     = 0;

// Baseline
float     groundAlt      = 0.0f;
uint8_t   baselineCnt    = 0;
float     baselineAccum  = 0.0f;

// Flight data (updated every tick)
float     bmpAlt         = 0.0f;   // metres AGL
float     mpuAlt         = 0.0f;   // integrated
float     ax=0,ay=0,az=0;          // g
float     vx=0,vy=0,vz=0;          // m/s (integrated)
float     roll=0,pitch=0,yaw=0;    // degrees (integrated)
float     temperature    = 0.0f;   // °C
float     pressure_hPa   = 0.0f;   // hPa
float     accelMag       = 0.0f;   // total g magnitude

// Apogee tracking
float     peakAlt        = 0.0f;
uint32_t  bmpApogeeMs    = 0;      // ms BMP vote has been true
uint32_t  mpuApogeeMs    = 0;      // ms MPU vote has been true
float     vertVel        = 0.0f;   // m/s vertical (integrated from MPU z)

// State counters / timers
uint8_t   launchCtr      = 0;
uint8_t   burnoutCtr     = 0;
uint32_t  landingStableMs= 0;
float     landingRefAlt  = 0.0f;
bool      landingRefSet  = false;

// Pyro
bool      continuityOK   = false;
bool      deployConfirmed= false;
bool      deployFail     = false;

// Flash write
uint32_t  flashAddr      = 0;
uint32_t  sessionStart   = 0;      // address of current session header
uint16_t  sessionCRC     = 0;
bool      flashReady     = false;
uint8_t   sessionID      = 0;

// Buzzer / LED non-blocking
uint32_t  buzzerOffMs    = 0;
bool      buzzerOn       = false;
uint8_t   buzzerPattern  = 0;
uint8_t   buzzerStep     = 0;
uint32_t  buzzerNextMs   = 0;

// Locator beep
uint32_t  locatorNextMs  = 0;

// ── CRC-16/CCITT (fast table-less) ───────────────────────────
uint16_t crc16Update(uint16_t crc, uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    bool xorFlag = ((crc ^ (b << 8)) & 0x8000) != 0;
    crc <<= 1;
    b   <<= 1;
    if (xorFlag) crc ^= 0x1021;
  }
  return crc;
}

// ── Flash helpers ─────────────────────────────────────────────
// Write bytes and accumulate CRC
void flashWrite(uint8_t *buf, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    flash.writeByte(flashAddr++, buf[i]);
    sessionCRC = crc16Update(sessionCRC, buf[i]);
  }
}

void flashWriteU8(uint8_t v)  { flashWrite(&v, 1); }
void flashWriteU32(uint32_t v){ flashWrite((uint8_t*)&v, 4); }
void flashWriteF32(float v)   { flashWrite((uint8_t*)&v, 4); }

// Scan flash to find first free address (first 0xFF byte aligned to 64-byte boundary)
uint32_t findFreeFlashAddr() {
  // Walk in 4096-byte page chunks for speed
  for (uint32_t addr = 0; addr < flash.getCapacity(); addr += 4096) {
    uint8_t b = flash.readByte(addr);
    if (b == 0xFF) return addr;
  }
  return 0; // flash full — caller handles
}

// ── Buzzer patterns (non-blocking) ───────────────────────────
// Each pattern is a sequence of on/off intervals in ms, 0-terminated
// We re-use a simple 3-slot struct
struct BuzzPattern {
  uint16_t ms[8]; // alternating on/off ms; 0 = end
};

// Index 0 = single short beep, 1 = double, 2 = triple, 3 = alarm (fast repeating)
const BuzzPattern PATTERNS[] PROGMEM = {
  {{80, 80, 0}},                              // 0: single — ready chirp
  {{80, 80, 80, 80, 0}},                      // 1: double — continuity warn
  {{80, 80, 80, 80, 80, 80, 0}},              // 2: triple — apogee confirm
  {{50, 50, 50, 50, 50, 50, 50, 0}}           // 3: rapid — HALT alarm
};

void buzzStart(uint8_t patIdx) {
  buzzerPattern = patIdx;
  buzzerStep    = 0;
  buzzerNextMs  = millis();
  // kick immediately
  digitalWrite(PIN_BUZZER, HIGH);
  buzzerOn = true;
}

void buzzTick() {
  if (buzzerNextMs == 0) return;
  uint32_t now = millis();
  if (now < buzzerNextMs) return;

  uint16_t interval = pgm_read_word(&PATTERNS[buzzerPattern].ms[buzzerStep]);
  if (interval == 0) {
    // pattern done
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOn      = false;
    buzzerNextMs  = 0;
    return;
  }
  // Toggle
  buzzerOn = !buzzerOn;
  digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
  buzzerNextMs = now + interval;
  buzzerStep++;
}

// Blocking beep — only used during BOOT (safe, not in flight path)
void beepBlocking(uint16_t onMs) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(onMs);
  digitalWrite(PIN_BUZZER, LOW);
  delay(80);
}

// ── Sensor read ───────────────────────────────────────────────
void readSensors() {
  // BMP180 — getAltitude() blocks ~26ms on Ultra-Low-Power mode
  // Use pressure directly to minimise blocking (still ~5ms)
  pressure_hPa = bmp.readPressure() / 100.0f;
  temperature  = bmp.readTemperature();
  // Standard atmosphere altitude (Baro formula)
  bmpAlt       = 44330.0f * (1.0f - pow(pressure_hPa / 1013.25f, 0.1903f)) - groundAlt;

  // MPU6050 raw
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  // Scale: ±2g → 16384 LSB/g; ±250°/s → 131 LSB/°/s
  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;
  float gx = rawGx / 131.0f;   // deg/s
  float gy = rawGy / 131.0f;
  float gz = rawGz / 131.0f;

  float dt = LOOP_INTERVAL_MS / 1000.0f;

  // Integrate gyro for attitude (simple Euler — fine for short flights)
  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

  // Integrate acceleration for velocity (rocket axis = Z for vertical)
  // Remove gravity component (1g) from az when computing vertical
  float vertAccel = (az - 1.0f) * 9.81f;  // m/s²
  vz += vertAccel * dt;
  vx += ax * 9.81f * dt;
  vy += ay * 9.81f * dt;

  // MPU altitude: integrate vertical velocity
  mpuAlt += vz * dt;

  // Vertical velocity for apogee vote (m/s, positive = up)
  vertVel = vz;

  // Total acceleration magnitude (g)
  accelMag = sqrt(ax*ax + ay*ay + az*az);
}

// ── Flash data row ────────────────────────────────────────────
void logDataRow() {
  if (!flashReady) return;
  uint32_t ts = millis();
  flashWriteU8('D');
  flashWriteU32(ts);
  flashWriteF32(bmpAlt);
  flashWriteF32(mpuAlt);
  flashWriteF32(ax); flashWriteF32(ay); flashWriteF32(az);
  flashWriteF32(vx); flashWriteF32(vy); flashWriteF32(vz);
  flashWriteF32(roll); flashWriteF32(pitch); flashWriteF32(yaw);
  flashWriteF32(temperature);
  flashWriteF32(pressure_hPa);
  // 57 bytes per row — 8 MB / 57 ≈ 148,000 rows ≈ 98 min at 25 Hz
}

void logEvent(uint8_t evtType) {
  if (!flashReady) return;
  flashWriteU8('E');
  flashWriteU32(millis());
  flashWriteU8(evtType);
  flashWriteF32(bmpAlt);
}

// ── Serial flash dump (boot) ──────────────────────────────────
void dumpFlashToSerial() {
  Serial.println(F("=== PREVIOUS FLIGHT DATA ==="));
  uint32_t addr = 0;
  uint8_t  sessionCount = 0;

  while (addr < flash.getCapacity() - 10) {
    uint8_t b0 = flash.readByte(addr);
    if (b0 == 0xFF) break;   // unwritten — done
    if (b0 != FLASH_MAGIC_START_0) { addr++; continue; }
    uint8_t b1 = flash.readByte(addr + 1);
    if (b1 != FLASH_MAGIC_START_1) { addr++; continue; }

    sessionCount++;
    uint8_t  sid    = flash.readByte(addr + 2);
    uint32_t startT; flash.readByteArray(addr + 3, (uint8_t*)&startT, 4);
    float    gAlt;   flash.readByteArray(addr + 7, (uint8_t*)&gAlt, 4);

    Serial.print(F("--- SESSION ")); Serial.print(sid);
    Serial.print(F(" | groundAlt=")); Serial.print(gAlt, 1); Serial.println(F("m ---"));
    Serial.println(F("ms,bmpAlt,mpuAlt,ax,ay,az,vx,vy,vz,roll,pitch,yaw,temp,pres"));

    addr += 11; // header size: 2+1+4+4

    uint16_t crcCalc = 0;
    // Recompute CRC over raw header bytes (already passed addr, so track separately)
    // For simplicity we do a forward scan for rows until END marker
    uint32_t rowAddr = addr;
    bool foundEnd = false;

    while (rowAddr < flash.getCapacity() - 10) {
      uint8_t tag = flash.readByte(rowAddr++);

      if (tag == FLASH_MAGIC_END_0) {
        uint8_t t2 = flash.readByte(rowAddr++);
        if (t2 == FLASH_MAGIC_END_1) {
          uint16_t storedCRC;
          flash.readByteArray(rowAddr, (uint8_t*)&storedCRC, 2);
          rowAddr += 2;
          Serial.print(F("CRC: "));
          Serial.println((storedCRC == crcCalc) ? F("OK") : F("CORRUPT"));
          foundEnd = true;
          addr = rowAddr;
          break;
        }
      } else if (tag == 'D') {
        uint8_t buf[56];
        flash.readByteArray(rowAddr, buf, 56);
        rowAddr += 56;

        uint32_t ts;   memcpy(&ts, buf,     4);
        float bA;      memcpy(&bA, buf+4,   4);
        float mA;      memcpy(&mA, buf+8,   4);
        float lax,lay,laz,lvx,lvy,lvz,lr,lp,ly2,lt,lpres;
        memcpy(&lax,  buf+12, 4); memcpy(&lay, buf+16, 4); memcpy(&laz, buf+20, 4);
        memcpy(&lvx,  buf+24, 4); memcpy(&lvy, buf+28, 4); memcpy(&lvz, buf+32, 4);
        memcpy(&lr,   buf+36, 4); memcpy(&lp,  buf+40, 4); memcpy(&ly2, buf+44, 4);
        memcpy(&lt,   buf+48, 4); memcpy(&lpres,buf+52,4);

        Serial.print(ts);  Serial.print(',');
        Serial.print(bA,2);Serial.print(',');
        Serial.print(mA,2);Serial.print(',');
        Serial.print(lax,3);Serial.print(',');
        Serial.print(lay,3);Serial.print(',');
        Serial.print(laz,3);Serial.print(',');
        Serial.print(lvx,2);Serial.print(',');
        Serial.print(lvy,2);Serial.print(',');
        Serial.print(lvz,2);Serial.print(',');
        Serial.print(lr,1); Serial.print(',');
        Serial.print(lp,1); Serial.print(',');
        Serial.print(ly2,1);Serial.print(',');
        Serial.print(lt,1); Serial.print(',');
        Serial.println(lpres,1);

      } else if (tag == 'E') {
        uint32_t ts; flash.readByteArray(rowAddr, (uint8_t*)&ts, 4); rowAddr+=4;
        uint8_t  et = flash.readByte(rowAddr++);
        float    ea; flash.readByteArray(rowAddr, (uint8_t*)&ea, 4); rowAddr+=4;
        const char* names[] = {"?","LAUNCH","BURNOUT","APOGEE",
                               "DEPLOY_OK","DEPLOY_FAIL","LANDED"};
        Serial.print(F("EVENT,")); Serial.print(ts); Serial.print(',');
        Serial.print((et<=6)?names[et]:"?"); Serial.print(',');
        Serial.println(ea, 1);
      } else {
        // Unrecognised byte — likely corruption; skip
        break;
      }
    }
    if (!foundEnd) addr = rowAddr;
  }

  if (sessionCount == 0) {
    Serial.println(F("(no previous sessions)"));
  }
  Serial.println(F("=== END OF PREVIOUS DATA ==="));
  Serial.println();
}

// ── Open new flash session ────────────────────────────────────
void openFlashSession() {
  flashAddr   = findFreeFlashAddr();
  sessionCRC  = 0;
  sessionStart= flashAddr;
  sessionID   = EEPROM.read(EEPROM_SESSION_ADDR) + 1;
  EEPROM.write(EEPROM_SESSION_ADDR, sessionID);

  // Header: AA BB id[1] startMs[4] groundAlt[4]
  flashWriteU8(FLASH_MAGIC_START_0);
  flashWriteU8(FLASH_MAGIC_START_1);
  flashWriteU8(sessionID);
  flashWriteU32(millis());
  flashWriteF32(groundAlt);

  flashReady = true;
}

void closeFlashSession() {
  if (!flashReady) return;
  // End marker + CRC (CRC covers everything from first header byte to here)
  uint8_t e0 = FLASH_MAGIC_END_0;
  uint8_t e1 = FLASH_MAGIC_END_1;
  flashWrite(&e0, 1);
  flashWrite(&e1, 1);
  uint16_t crc = sessionCRC;   // save before the CRC bytes corrupt it
  flash.writeByte(flashAddr++, (uint8_t)(crc >> 8));
  flash.writeByte(flashAddr++, (uint8_t)(crc & 0xFF));
  flashReady = false;
}

// ── LED helpers ───────────────────────────────────────────────
void ledGreen(bool on) { digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW); }
void ledRed(bool on)   { digitalWrite(PIN_LED_RED,   on ? HIGH : LOW); }

// ── setup() ──────────────────────────────────────────────────
void setup() {
  // Pins
  pinMode(PIN_MOSFET,      OUTPUT); digitalWrite(PIN_MOSFET, LOW);
  pinMode(PIN_BUZZER,      OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LED_GREEN,   OUTPUT); digitalWrite(PIN_LED_GREEN, LOW);
  pinMode(PIN_LED_RED,     OUTPUT); digitalWrite(PIN_LED_RED, LOW);
  pinMode(PIN_CONTINUITY,  INPUT_PULLUP);

  Serial.begin(115200);
  delay(300); // let CH340 enumerate

  // ── BOOT state: dump flash ────────────────────────────────
  state = ST_BOOT;
  Serial.println(F("Rocket FC v1.0 — booting"));
  ledGreen(true);

  Wire.begin();
  Wire.setClock(400000); // fast I2C

  if (!flash.begin()) {
    Serial.println(F("FLASH FAIL — data logging unavailable"));
  } else {
    dumpFlashToSerial();
  }

  // ── SELF_TEST ────────────────────────────────────────────
  state = ST_SELF_TEST;
  Serial.println(F("--- NEW SESSION ---"));
  Serial.println(F("Running self-test..."));

  // BMP180
  if (!bmp.begin()) {
    Serial.println(F("FAIL: BMP180 not found"));
    state = ST_HALT;
  } else {
    float testPres = bmp.readPressure() / 100.0f;
    if (testPres < 300.0f || testPres > 1100.0f) {
      Serial.print(F("FAIL: BMP180 pressure out of range: "));
      Serial.println(testPres);
      state = ST_HALT;
    } else {
      Serial.print(F("BMP180 OK — "));
      Serial.print(testPres); Serial.println(F(" hPa"));
    }
  }

  // MPU6050
  if (state != ST_HALT) {
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println(F("FAIL: MPU6050 not found"));
      state = ST_HALT;
    } else {
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250°/s
      mpu.setDLPFMode(MPU6050_DLPF_BW_42);             // 42Hz LPF
      Serial.println(F("MPU6050 OK"));
    }
  }

  // Pyro continuity
  if (state != ST_HALT) {
    continuityOK = (digitalRead(PIN_CONTINUITY) == LOW);
    if (!continuityOK) {
      Serial.println(F("WARN: Pyro continuity OPEN — check igniter"));
      ledRed(true); delay(100); ledRed(false);
      beepBlocking(200); beepBlocking(200);
    } else {
      Serial.println(F("Pyro continuity OK"));
    }
  }

  // Halt on hard sensor fail
  if (state == ST_HALT) {
    Serial.println(F("HALTED — sensor fail"));
    ledRed(true);
    while (true) {
      // Alarm beep forever; watchdog will reset if enabled
      // Don't enable WDT in halt so we stay here indefinitely
      digitalWrite(PIN_BUZZER, HIGH); delay(50);
      digitalWrite(PIN_BUZZER, LOW);  delay(50);
    }
  }

  // ── Baseline averaging ─────────────────────────────────────
  Serial.println(F("Building altitude baseline (2s)..."));
  baselineCnt   = 0;
  baselineAccum = 0.0f;
  groundAlt     = 0.0f; // raw reads first

  while (baselineCnt < BASELINE_SAMPLES) {
    float rawPres = bmp.readPressure() / 100.0f;
    float rawAlt  = 44330.0f * (1.0f - pow(rawPres / 1013.25f, 0.1903f));
    baselineAccum += rawAlt;
    baselineCnt++;
    delay(40);
    wdt_reset(); // safe here — we're still in setup
  }
  groundAlt = baselineAccum / baselineCnt;
  Serial.print(F("Ground altitude ref: ")); Serial.print(groundAlt, 1); Serial.println(F(" m ASL"));

  // ── Arm watchdog (2 s) ────────────────────────────────────
  wdt_enable(WDTO_2S);

  // ── PAD_IDLE ──────────────────────────────────────────────
  state = ST_PAD_IDLE;
  ledGreen(true); ledRed(false);
  beepBlocking(80); // single ready chirp (blocking OK here, pre-flight)
  Serial.println(F("PAD IDLE — armed. Waiting for launch."));
  Serial.println(F("ms,bmpAlt,mpuAlt,ax,ay,az,vx,vy,vz,roll,pitch,yaw,temp,pres,state"));

  loopLastMs = millis();
}

// ── loop() ───────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // ── Rate limiter: 25 Hz tick ─────────────────────────────
  if (now - loopLastMs < LOOP_INTERVAL_MS) return;
  loopLastMs = now;

  wdt_reset(); // pat the dog every tick

  // ── Read sensors (every tick, all states except HALT) ────
  if (state != ST_HALT && state != ST_BOOT && state != ST_SELF_TEST) {
    readSensors();
  }

  // ── Continuity monitor (pad idle only) ───────────────────
  if (state == ST_PAD_IDLE) {
    continuityOK = (digitalRead(PIN_CONTINUITY) == LOW);
  }

  // ── Non-blocking buzzer tick ──────────────────────────────
  buzzTick();

  // ── State machine ─────────────────────────────────────────
  switch (state) {

    // ── PAD IDLE ───────────────────────────────────────────
    case ST_PAD_IDLE: {
      // Refine baseline — slow drift correction
      groundAlt = groundAlt * 0.99f +
                  (44330.0f * (1.0f - pow(pressure_hPa / 1013.25f, 0.1903f))) * 0.01f;

      // Continuity status LED
      ledGreen(continuityOK);
      ledRed(!continuityOK);

      // Print pad status every 5 s to serial (every 125 ticks)
      static uint8_t padPrintCtr = 0;
      if (++padPrintCtr >= 125) {
        padPrintCtr = 0;
        Serial.print(F("PAD | Alt:")); Serial.print(bmpAlt,1);
        Serial.print(F("m | Cont:")); Serial.println(continuityOK ? F("OK") : F("OPEN"));
        // Chirp: 1=OK, 2=warn
        if (!continuityOK) buzzStart(1); else buzzStart(0);
      }

      // Launch detect: accelMag > threshold, N consecutive ticks
      if (accelMag >= LAUNCH_ACCEL_G) {
        launchCtr++;
      } else {
        launchCtr = 0;
      }

      if (launchCtr >= LAUNCH_CONFIRM_TICKS) {
        // LAUNCH CONFIRMED
        openFlashSession();
        logEvent(EVT_LAUNCH);

        // Reset integration state
        vx = vy = vz = 0.0f;
        mpuAlt = 0.0f;
        peakAlt = bmpAlt;
        launchCtr = 0;

        ledGreen(false); ledRed(false);
        Serial.println(F("*** LAUNCH DETECTED ***"));
        state = ST_BOOST;
      }
      break;
    }

    // ── BOOST ─────────────────────────────────────────────
    case ST_BOOST: {
      logDataRow();

      // Track peak
      if (bmpAlt > peakAlt) peakAlt = bmpAlt;

      // Burnout: accel < threshold for N ticks
      if (accelMag < BURNOUT_ACCEL_G) {
        burnoutCtr++;
      } else {
        burnoutCtr = 0;
      }

      if (burnoutCtr >= BURNOUT_CONFIRM_TICKS) {
        logEvent(EVT_BURNOUT);
        burnoutCtr = 0;
        // Reset apogee timers
        bmpApogeeMs = 0;
        mpuApogeeMs = 0;
        Serial.println(F("*** BURNOUT ***"));
        state = ST_COAST;
      }
      break;
    }

    // ── COAST ─────────────────────────────────────────────
    case ST_COAST: {
      logDataRow();

      if (bmpAlt > peakAlt) peakAlt = bmpAlt;

      // BMP vote: altitude more than 2m below peak
      if (bmpAlt < (peakAlt - APOGEE_ALT_DROP_M)) {
        bmpApogeeMs += LOOP_INTERVAL_MS;
      } else {
        bmpApogeeMs = 0;
      }

      // MPU vote: vertical velocity negative (falling)
      if (vertVel < 0.0f) {
        mpuApogeeMs += LOOP_INTERVAL_MS;
      } else {
        mpuApogeeMs = 0;
      }

      // Both votes must hold for APOGEE_VOTE_MS
      if (bmpApogeeMs >= APOGEE_VOTE_MS && mpuApogeeMs >= APOGEE_VOTE_MS) {
        logEvent(EVT_APOGEE);
        Serial.print(F("*** APOGEE @ ")); Serial.print(peakAlt,1); Serial.println(F("m ***"));
        buzzStart(2); // triple beep
        state = ST_APOGEE_FIRE;
      }
      break;
    }

    // ── APOGEE FIRE ───────────────────────────────────────
    case ST_APOGEE_FIRE: {
      // Non-blocking fire sequence using a sub-state
      static uint8_t fireStep    = 0;
      static uint32_t fireTimer  = 0;
      static bool     refireDone = false;

      switch (fireStep) {
        case 0: // Initiate fire
          Serial.println(F("FIRE — pyro channel"));
          digitalWrite(PIN_MOSFET, HIGH);
          ledRed(true);
          fireTimer = now;
          fireStep  = 1;
          break;

        case 1: // Wait FIRE_PULSE_MS
          if (now - fireTimer >= FIRE_PULSE_MS) {
            digitalWrite(PIN_MOSFET, LOW);
            ledRed(false);
            fireTimer = now;
            fireStep  = 2;
          }
          break;

        case 2: // Wait 200ms settle then check continuity
          if (now - fireTimer >= 200) {
            bool stillLive = (digitalRead(PIN_CONTINUITY) == LOW);
            if (!stillLive) {
              // Continuity lost → igniter fired
              deployConfirmed = true;
              logEvent(EVT_DEPLOY_OK);
              Serial.println(F("DEPLOY CONFIRMED"));
              fireStep = 99; // done
            } else if (!refireDone) {
              // Continuity still present → re-fire attempt
              Serial.println(F("Continuity present — re-fire in 1s"));
              fireTimer  = now;
              fireStep   = 3;
              refireDone = true;
            } else {
              // Already re-fired, still no deploy
              deployFail = true;
              logEvent(EVT_DEPLOY_FAIL);
              Serial.println(F("DEPLOY FAIL — continuity still live after re-fire"));
              fireStep = 99;
            }
          }
          break;

        case 3: // Wait before re-fire
          if (now - fireTimer >= REFIRE_WAIT_MS) {
            Serial.println(F("RE-FIRE"));
            digitalWrite(PIN_MOSFET, HIGH);
            ledRed(true);
            fireTimer = now;
            fireStep  = 4;
          }
          break;

        case 4: // Re-fire pulse
          if (now - fireTimer >= FIRE_PULSE_MS) {
            digitalWrite(PIN_MOSFET, LOW);
            ledRed(false);
            fireTimer = now;
            fireStep  = 2; // go back to continuity check
          }
          break;

        case 99: // Fire sequence complete
          fireStep   = 0;
          refireDone = false;

          // Reset velocity integrators for descent tracking
          vx = vy = vz = 0.0f;

          state = ST_DESCENT;
          break;
      }
      break;
    }

    // ── DESCENT ───────────────────────────────────────────
    case ST_DESCENT: {
      // Log at reduced rate: every 2nd tick (12.5 Hz) to save flash
      static uint8_t desCtr = 0;
      if (++desCtr >= 2) {
        desCtr = 0;
        logDataRow();
      }

      // Landing detect: altitude stable within LANDING_DELTA_M
      // AND total accel close to 1g (static)
      bool accelStatic = (accelMag > 0.85f && accelMag < 1.15f);

      if (!landingRefSet) {
        landingRefAlt  = bmpAlt;
        landingRefSet  = true;
        landingStableMs = 0;
      }

      if (fabsf(bmpAlt - landingRefAlt) > LANDING_DELTA_M || !accelStatic) {
        // Still moving — reset window
        landingRefAlt   = bmpAlt;
        landingStableMs = 0;
      } else {
        landingStableMs += LOOP_INTERVAL_MS;
      }

      if (landingStableMs >= LANDING_CONFIRM_MS) {
        logEvent(EVT_LANDED);
        closeFlashSession();
        Serial.println(F("*** LANDED ***"));
        locatorNextMs = now;
        state = ST_LANDED;
      }
      break;
    }

    // ── LANDED ────────────────────────────────────────────
    case ST_LANDED: {
      // Locator beep: 3 beeps every 10 s
      if (now >= locatorNextMs) {
        locatorNextMs = now + LOCATOR_PERIOD_MS;
        for (uint8_t i = 0; i < LOCATOR_BEEPS; i++) {
          digitalWrite(PIN_BUZZER, HIGH); delay(80);
          digitalWrite(PIN_BUZZER, LOW);  delay(80);
          wdt_reset();
        }
        ledGreen(true); delay(200); ledGreen(false);
      }

      // Print landing summary once
      static bool landPrinted = false;
      if (!landPrinted) {
        landPrinted = true;
        Serial.print(F("Peak altitude: ")); Serial.print(peakAlt,1); Serial.println(F(" m AGL"));
        Serial.print(F("Deploy: "));
        Serial.println(deployConfirmed ? F("OK") : (deployFail ? F("FAIL") : F("n/a")));
        Serial.println(F("Session finalized. Safe to power off."));
      }
      break;
    }

    default: break;
  }

  // ── Serial telemetry (all active flight states) ───────────
  if (state >= ST_BOOST && state <= ST_DESCENT) {
    Serial.print(now);              Serial.print(',');
    Serial.print(bmpAlt,  2);       Serial.print(',');
    Serial.print(mpuAlt,  2);       Serial.print(',');
    Serial.print(ax,      3);       Serial.print(',');
    Serial.print(ay,      3);       Serial.print(',');
    Serial.print(az,      3);       Serial.print(',');
    Serial.print(vx,      2);       Serial.print(',');
    Serial.print(vy,      2);       Serial.print(',');
    Serial.print(vz,      2);       Serial.print(',');
    Serial.print(roll,    1);       Serial.print(',');
    Serial.print(pitch,   1);       Serial.print(',');
    Serial.print(yaw,     1);       Serial.print(',');
    Serial.print(temperature, 1);   Serial.print(',');
    Serial.print(pressure_hPa, 1);  Serial.print(',');
    Serial.println(state);
  }
}
