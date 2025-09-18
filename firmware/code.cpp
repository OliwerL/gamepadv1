#include <NimBLEDevice.h>

// ====== PINY (Twoje) ======
#define PIN_LEFT_X   0
#define PIN_LEFT_Y   1
#define PIN_RIGHT_X  2
#define PIN_RIGHT_Y  3
#define PIN_UP       4
#define PIN_DOWN     5
#define PIN_LEFT     6
#define PIN_RIGHT    7
#define PIN_A        8
#define PIN_B        10
#define PIN_X        18   // przenieś, jeśli użyjesz natywnego USB
#define PIN_Y        19

// ====== BLE NUS ======
#define NUS_SERVICE  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // PC -> ESP
#define NUS_TX_CHAR  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // ESP -> PC

// ====== PARAMETRY OSI ======
#define SAMPLE_HZ      100           // częstotliwość wysyłania
#define DEADZONE_FRAC  0.06f         // ~6% martwej strefy
#define LPF_ALPHA      0.30f         // filtr wygładzający (0..1), 0=bez zmian
#define BOOT_CENTER_MS 800           // czas uśredniania środka przy starcie

// ====== ZMIENNE ======
NimBLECharacteristic* txChar;

struct AxisCal {
  int32_t center = 2048;   // środek
  int32_t minv   = 1600;   // obserwowane min
  int32_t maxv   = 2500;   // obserwowane max
  int32_t filt   = 0;      // stan przefiltrowany w skali -32768..32767
} cal[4]; // lx,ly,rx,ry

// czytanie przycisku (pullup): 1 = wciśnięty
static inline int btn(int pin){ return !digitalRead(pin); }

// szybkie mapowanie do int16 z deadzone i filtrem
int16_t map_axis(int raw, AxisCal &c) {
  // aktualizuj min/max „uczące się”
  if (raw < c.minv) c.minv = raw;
  if (raw > c.maxv) c.maxv = raw;
  int32_t span_pos = c.maxv - c.center;
  int32_t span_neg = c.center - c.minv;
  int32_t span = max(span_pos, span_neg);
  if (span < 200) span = 200; // minimalny sensowny zakres

  // przesunięcie względem środka
  float x = (float)(raw - c.center) / (float)span; // ~ -1..+1

  // martwa strefa
  float dz = DEADZONE_FRAC;
  if (x > -dz && x < dz) x = 0.f;
  else if (x >= dz) x = (x - dz) / (1.f - dz);
  else if (x <= -dz) x = (x + dz) / (1.f - dz);

  // skala do int16
  int32_t y = (int32_t)(x * 32767.f);
  // filtr IIR
  c.filt = (int32_t)((1.f - LPF_ALPHA) * c.filt + LPF_ALPHA * y);
  return (int16_t)c.filt;
}

// autokalibracja środka (przez ~BOOT_CENTER_MS na starcie)
void auto_center_boot() {
  uint32_t t0 = millis();
  uint32_t n = 0;
  uint64_t acc[4] = {0,0,0,0};
  while (millis() - t0 < BOOT_CENTER_MS) {
    acc[0] += analogRead(PIN_LEFT_X);
    acc[1] += analogRead(PIN_LEFT_Y);
    acc[2] += analogRead(PIN_RIGHT_X);
    acc[3] += analogRead(PIN_RIGHT_Y);
    n++;
    delay(2);
  }
  for (int i=0;i<4;i++) {
    int32_t c0 = (int32_t)(acc[i] / max<uint32_t>(n,1));
    cal[i].center = c0;
    cal[i].minv   = c0 - 300; // początkowy zapas
    cal[i].maxv   = c0 + 300;
    cal[i].filt   = 0;
  }
}

// „twardy” recal środka na aktualną pozycję (wywoływany z komendy BLE)
void recalibrate_now() {
  cal[0].center = analogRead(PIN_LEFT_X);
  cal[1].center = analogRead(PIN_LEFT_Y);
  cal[2].center = analogRead(PIN_RIGHT_X);
  cal[3].center = analogRead(PIN_RIGHT_Y);
  for (int i=0;i<4;i++) {
    cal[i].minv = cal[i].center - 300;
    cal[i].maxv = cal[i].center + 300;
    cal[i].filt = 0;
  }
}

// RX callback (opcjonalne komendy z PC: {"cmd":"cal"})
class RxCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string s = c->getValue();
    if (s.find("\"cal\"") != std::string::npos) {
      recalibrate_now();
    }
  }
};

void setup() {
  // ADC na 3.3 V
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LEFT_X,  ADC_11db);
  analogSetPinAttenuation(PIN_LEFT_Y,  ADC_11db);
  analogSetPinAttenuation(PIN_RIGHT_X, ADC_11db);
  analogSetPinAttenuation(PIN_RIGHT_Y, ADC_11db);

  // przyciski
  pinMode(PIN_UP,    INPUT_PULLUP);
  pinMode(PIN_DOWN,  INPUT_PULLUP);
  pinMode(PIN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_A,     INPUT_PULLUP);
  pinMode(PIN_B,     INPUT_PULLUP);
  pinMode(PIN_X,     INPUT_PULLUP);
  pinMode(PIN_Y,     INPUT_PULLUP);

  // autokalibracja środka przy starcie
  auto_center_boot();

  // BLE
  NimBLEDevice::init("ESP32C3_PAD");
  auto server  = NimBLEDevice::createServer();
  auto service = server->createService(NUS_SERVICE);
  txChar = service->createCharacteristic(NUS_TX_CHAR, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  auto rxChar = service->createCharacteristic(NUS_RX_CHAR, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxChar->setCallbacks(new RxCb());
  service->start();

  auto adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  NimBLEDevice::startAdvertising();
}

void loop() {
  // surowe
  int raw_lx = analogRead(PIN_LEFT_X);
  int raw_ly = analogRead(PIN_LEFT_Y);
  int raw_rx = analogRead(PIN_RIGHT_X);
  int raw_ry = analogRead(PIN_RIGHT_Y);

  // mapowanie -> int16
  int16_t lx = map_axis(raw_lx, cal[0]);
  int16_t ly = map_axis(raw_ly, cal[1]);
  int16_t rx = map_axis(raw_rx, cal[2]);
  int16_t ry = map_axis(raw_ry, cal[3]);

  // przyciski -> bitmask
  uint16_t k = 0;
  k |= btn(PIN_UP)    << 0;
  k |= btn(PIN_DOWN)  << 1;
  k |= btn(PIN_LEFT)  << 2;
  k |= btn(PIN_RIGHT) << 3;
  k |= btn(PIN_A)     << 4;
  k |= btn(PIN_B)     << 5;
  k |= btn(PIN_X)     << 6;
  k |= btn(PIN_Y)     << 7;

  // JSON (int16; wygodny do gier/emulatorów)
  char msg[120];
  int n = snprintf(msg, sizeof(msg),
                   "{\"lx\":%d,\"ly\":%d,\"rx\":%d,\"ry\":%d,\"k\":%u}",
                   (int)lx,(int)ly,(int)rx,(int)ry,k);

  txChar->setValue((uint8_t*)msg, n);
  txChar->notify();

  delay(1000 / SAMPLE_HZ);
}
