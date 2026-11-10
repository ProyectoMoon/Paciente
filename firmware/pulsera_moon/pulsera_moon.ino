// =====================================================================
//  MOON – Pulsera (ESP32-C3 Super Mini + MAX3010x + MPU6050)
//  Versión optimizada: menos consumo de batería y loop más liviano.
//
//  Lo que se manda a la web NO cambia: "spo2,bpm,temp,g" cada 500 ms
//  por BLE (mismos UUID), así que la página sigue funcionando igual.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- Configuración ----------------
// 1 = imprime diagnóstico por el monitor serie. Ponelo en 0 cuando la
// pulsera ande con batería: imprimir cada 500 ms gasta CPU y energía.
#define DEBUG 1

// Estos UUID tienen que ser EXACTAMENTE los mismos que en la página web
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_DEVICE_NAME         "MOON"

#define CPU_MHZ 80                 // 80 MHz alcanza para todo (BLE incluido) y gasta mucho menos que 160
#define BPM_OFFSET 45              // cuántos BPM mide de más el sensor (se resta)
#define BLE_INTERVALO_MS 500       // la web cuenta muestras de caída con este ritmo: no lo subas
#define G_INTERVALO_MS 10          // lectura del acelerómetro (~100 por segundo)
#define TEMP_CADA_CICLOS 30        // la temperatura casi no cambia: se lee cada ~30 s (bloquea ~30 ms)
#define CICLOS_INVALIDOS_MAX 6     // ciclos (~1 s) sin lectura válida antes de borrar el último valor
// ---- Arranque limpio (evita las lecturas malas del principio) ----
// IR por debajo de esto = no hay dedo (o está mal apoyado). Con el dedo bien
// puesto el IR suele andar entre 80000 y 250000; sin dedo, unos pocos miles.
#define IR_DEDO_MIN 50000
// Al apoyar el dedo, la señal tarda unos segundos en asentarse: se descartan
// las muestras de este tiempo antes de empezar a medir.
#define ESTABILIZAR_MS 2000
// Brillo automático del LED: mientras se estabiliza, se sube o baja la luz
// hasta que el IR quede en este rango. Si la luz es mucha el sensor se
// satura (~262000) y si es poca la señal es débil: en los dos casos da
// "señal no válida".
#define IR_OBJETIVO_MIN 90000
#define IR_OBJETIVO_MAX 200000
// Lecturas válidas y parecidas entre sí que hacen falta para mostrar el
// primer valor (antes era 1: mostraba la primera, que suele ser ruido).
#define MIN_LECTURAS_MOSTRAR 2
#define MAX_DIF_BPM  35
#define MAX_DIF_SPO2 6
#define USAR_MPU 1
// 1 = giroscopio prendido (se lee y se imprime en el diagnóstico).
// 0 = giroscopio apagado: ahorra ~3,5 mA. Hoy la web solo usa la
//     aceleración (g), así que podés apagarlo si no lo necesitás.
#define USAR_GIROSCOPIO 1

#define SDA_PIN 8
#define SCL_PIN 9

#if DEBUG
  #define LOG(...)   Serial.printf(__VA_ARGS__)
#else
  #define LOG(...)   do {} while (0)
#endif

// ---------------- BLE ----------------
BLECharacteristic *pVitalsChar = nullptr;
volatile bool bleConectado = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConectado = true; LOG("BLE: cliente conectado\n"); }
  void onDisconnect(BLEServer *) override {
    bleConectado = false;
    LOG("BLE: cliente desconectado, vuelvo a anunciarme\n");
    BLEDevice::startAdvertising();
  }
};

void iniciarBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  pVitalsChar = pService->createCharacteristic(
      BLE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pVitalsChar->addDescriptor(new BLE2902());
  pVitalsChar->setValue("nan,nan,0.0,1.00");
  pService->start();

  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  // Anunciarse cada ~200-300 ms en vez del valor por defecto (mucho más
  // seguido): gasta menos batería mientras espera, y la pulsera sigue
  // apareciendo en la lista casi al instante. Unidades de 0,625 ms.
  pAdv->setMinInterval(320);   // 200 ms
  pAdv->setMaxInterval(480);   // 300 ms
  BLEDevice::startAdvertising();
  LOG("BLE listo, anunciando como '" BLE_DEVICE_NAME "'\n");
}

// Manda "spo2,bpm,temp,g". Sin lectura válida manda "nan,nan,temp,g":
// la web ignora los signos vitales pero sigue procesando la aceleración.
void enviarVitalesBLE(int spo2Val, int bpmVal, float tempVal, float gVal) {
  if (!bleConectado) return;   // sin nadie conectado no se arma ni se manda nada
  char msg[40];
  int len = (spo2Val > 0 && bpmVal > 0)
    ? snprintf(msg, sizeof(msg), "%d,%d,%.1f,%.2f", spo2Val, bpmVal, tempVal, gVal)
    : snprintf(msg, sizeof(msg), "nan,nan,%.1f,%.2f", tempVal, gVal);
  pVitalsChar->setValue((uint8_t *)msg, len);
  pVitalsChar->notify();
  LOG("BLE -> %s\n", msg);
}

// ---------------- MAX3010x ----------------
MAX30105 particleSensor;

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
const int32_t bufferLength = BUFFER_SIZE;
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

#define NUM_READINGS 3
#define MUESTRAS_NUEVAS 25
bool primerLlenado = false;
int32_t bpmReadings[NUM_READINGS];
int32_t spo2Readings[NUM_READINGS];
int readingIndex = 0;
int lecturasGuardadas = 0;

int ultimoSpo2 = 0;
int ultimoBpm = 0;
int ciclosInvalidos = 0;
float ultimaTemp = 0.0f;
uint32_t ciclosDesdeTemp = TEMP_CADA_CICLOS;   // fuerza una lectura en el primer ciclo

uint8_t brilloLed = 100;     // brillo actual del LED (0-255), se ajusta solo

// Pico de aceleración (en g) desde el último envío. 1.0 = quieto.
float gReportado = 1.0f;

// ---------------- MPU6050 ----------------
#if USAR_MPU
#define MPU6050_ADDR 0x68
#define ACCEL_SENSITIVITY 4096.0f   // rango ±8g

void mpuEscribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void iniciarMPU() {
  mpuEscribir(0x6B, 0x00);   // PWR_MGMT_1: despertar
#if USAR_GIROSCOPIO
  mpuEscribir(0x6C, 0x00);   // PWR_MGMT_2: acelerómetro y giroscopio prendidos
  LOG("MPU6050 listo (±8g, giroscopio prendido).\n");
#else
  mpuEscribir(0x6C, 0x07);   // PWR_MGMT_2: apaga el giroscopio -> ahorra ~3,5 mA
  LOG("MPU6050 listo (±8g, giroscopio apagado).\n");
#endif
  mpuEscribir(0x1C, 0x10);   // ACCEL_CONFIG: ±8g
}

#if USAR_GIROSCOPIO
#define GYRO_SENSITIVITY 131.0f   // rango ±250 °/s

// Lee el giroscopio en °/s
void leerGiroscopio(float &gx, float &gy, float &gz) {
  gx = gy = gz = 0;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((uint16_t)MPU6050_ADDR, (uint8_t)6, true) != 6) return;
  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  gx = x / GYRO_SENSITIVITY;
  gy = y / GYRO_SENSITIVITY;
  gz = z / GYRO_SENSITIVITY;
}
#endif

// Aceleración total en g (solo acelerómetro: 6 bytes)
float leerG() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return 1.0f;
  if (Wire.requestFrom((uint16_t)MPU6050_ADDR, (uint8_t)6, true) != 6) return 1.0f;
  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  // Se hace la cuenta en enteros y se escala una sola vez al final
  float m2 = (float)x * x + (float)y * y + (float)z * z;
  return sqrtf(m2) * (1.0f / ACCEL_SENSITIVITY);
}

void actualizarG() {
  static uint32_t ultimo = 0;
  uint32_t ahora = millis();
  if (ahora - ultimo < G_INTERVALO_MS) return;
  ultimo = ahora;
  float g = leerG();
  if (fabsf(g - 1.0f) > fabsf(gReportado - 1.0f)) gReportado = g;
}
#else
void actualizarG() {}
#endif

void enviarPeriodico() {
  static uint32_t ultimoEnvio = 0;
  uint32_t ahora = millis();
  if (ahora - ultimoEnvio < BLE_INTERVALO_MS) return;
  ultimoEnvio = ahora;
  enviarVitalesBLE(ultimoSpo2, ultimoBpm, ultimaTemp, gReportado);
  gReportado = 1.0f;
}

inline void tareasRapidas() {
  actualizarG();
  enviarPeriodico();
}

// Promedio de las lecturas guardadas
void promedios(long &bpmProm, long &spo2Prom) {
  long sB = 0, sS = 0;
  for (int i = 0; i < lecturasGuardadas; i++) { sB += bpmReadings[i]; sS += spo2Readings[i]; }
  bpmProm = sB / lecturasGuardadas;
  spo2Prom = sS / lecturasGuardadas;
}

void setup() {
  setCpuFrequencyMhz(CPU_MHZ);

#if DEBUG
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 2000) delay(10);   // con batería no hay USB: no esperar de más
#endif
  LOG("Arrancando...\n");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(50);

  // I2C rápido (400 kHz): los dos sensores lo soportan y cada lectura
  // ocupa el bus 4 veces menos tiempo.
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    while (1) {
      LOG("No se encontró el MAX3010x. Revisá el cableado.\n");
      delay(1000);
    }
  }
  LOG("Sensor de pulso detectado.\n");

  byte ledBrightness = brilloLed;   // valor inicial: después se ajusta solo
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

#if USAR_MPU
  iniciarMPU();
#endif

  iniciarBLE();
}

// ---- Detección de dedo ----
bool dedoListo = false;      // true = dedo apoyado y señal ya estabilizada
uint32_t dedoDesde = 0;      // cuándo se apoyó el dedo (0 = no hay dedo)

// Borra todo lo medido: se llama cuando se saca el dedo
void reiniciarMedicion() {
  dedoListo = false;
  dedoDesde = 0;
  primerLlenado = false;
  ultimoSpo2 = 0;
  ultimoBpm = 0;
  lecturasGuardadas = 0;
  readingIndex = 0;
  ciclosInvalidos = 0;
}

// Espera la próxima muestra del sensor (sin dejar de leer g ni de mandar
// por BLE). Devuelve false si el sensor no responde.
bool tomarMuestra(uint32_t &red, uint32_t &ir) {
  uint32_t t0 = millis();
  while (!particleSensor.available()) {
    tareasRapidas();
    particleSensor.check();
    if (millis() - t0 > 2000) {
      LOG("El sensor de pulso no entrega muestras. Revisá cables y alimentación.\n");
      return false;
    }
    delay(1);   // cede la CPU: deja dormir al procesador entre muestras
  }
  red = particleSensor.getRed();
  ir  = particleSensor.getIR();
  particleSensor.nextSample();
  return true;
}

void loop() {
  uint32_t red, ir;

  // ---- 1) Esperar a que haya dedo y que la señal se asiente ----
  if (!dedoListo) {
    if (!tomarMuestra(red, ir)) return;
    if (ir < IR_DEDO_MIN) {          // todavía no hay dedo
      dedoDesde = 0;
      return;
    }
    if (dedoDesde == 0) {
      dedoDesde = millis();
      LOG("Dedo detectado, estabilizando la señal...\n");
    }
    // Ajusta el brillo mientras se estabiliza (cada ~200 ms, 8 muestras)
    static uint8_t cuenta = 0;
    if (++cuenta >= 8) {
      cuenta = 0;
      uint8_t antes = brilloLed;
      int nuevo = brilloLed;
      if (ir > IR_OBJETIVO_MAX)      nuevo -= (ir > 250000 ? 20 : 5);
      else if (ir < IR_OBJETIVO_MIN) nuevo += 5;
      if (nuevo < 10)  nuevo = 10;
      if (nuevo > 255) nuevo = 255;
      brilloLed = (uint8_t)nuevo;
      if (brilloLed != antes) {
        particleSensor.setPulseAmplitudeRed(brilloLed);
        particleSensor.setPulseAmplitudeIR(brilloLed);
        dedoDesde = millis();   // con brillo nuevo, la señal tiene que volver a asentarse
        LOG("IR:%lu -> brillo LED %d\n", (unsigned long)ir, brilloLed);
      }
    }
    if (millis() - dedoDesde < ESTABILIZAR_MS) return;   // descarta estas muestras
    dedoListo = true;
    primerLlenado = false;
    LOG("Señal estable, empiezo a medir.\n");
  }

  int inicio = 0;

  if (!primerLlenado) {
    LOG("Llenando buffer inicial...\n");
  } else {
    // Corre las 75 muestras más nuevas al principio de una sola vez
    const size_t quedan = (bufferLength - MUESTRAS_NUEVAS) * sizeof(uint32_t);
    memmove(redBuffer, redBuffer + MUESTRAS_NUEVAS, quedan);
    memmove(irBuffer,  irBuffer  + MUESTRAS_NUEVAS, quedan);
    inicio = bufferLength - MUESTRAS_NUEVAS;
  }

  // ---- Tomar muestras nuevas ----
  for (int i = inicio; i < bufferLength; i++) {
    if (!tomarMuestra(red, ir)) { primerLlenado = false; return; }
    if (ir < IR_DEDO_MIN) {
      // Se sacó el dedo: no se mezclan muestras sin dedo con las buenas
      LOG("Dedo retirado, se borra la medición.\n");
      reiniciarMedicion();
      return;
    }
    redBuffer[i] = red;
    irBuffer[i]  = ir;
  }
  primerLlenado = true;

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                         &spo2, &validSPO2, &heartRate, &validHeartRate);
  tareasRapidas();   // el algoritmo tarda un poco: no perder un pico de caída

  LOG("IR:%lu  spo2:%d(v%d)  hr:%d(v%d)\n",
      (unsigned long)irBuffer[bufferLength - 1],
      (int)spo2, validSPO2, (int)heartRate, validHeartRate);

  // La lectura de temperatura bloquea ~30 ms; se hace cada TEMP_CADA_CICLOS
  if (++ciclosDesdeTemp >= TEMP_CADA_CICLOS) {
    ciclosDesdeTemp = 0;
    ultimaTemp = particleSensor.readTemperature();
    tareasRapidas();
  }

  // ---- Procesar datos ----
  if (validHeartRate && validSPO2 && heartRate > 40 && heartRate < 180 && spo2 >= 70 && spo2 <= 100) {
    bool coherente = true;
    if (lecturasGuardadas > 0) {
      long pB, pS;
      promedios(pB, pS);
      coherente = (labs(heartRate - pB) <= MAX_DIF_BPM && labs(spo2 - pS) <= MAX_DIF_SPO2);
    }

    if (!coherente && lecturasGuardadas >= NUM_READINGS) {
      LOG("Lectura inconsistente con las anteriores (ruido), la descarto\n");
    } else {
      if (!coherente) { lecturasGuardadas = 0; readingIndex = 0; }

      ciclosInvalidos = 0;
      bpmReadings[readingIndex] = heartRate;
      spo2Readings[readingIndex] = spo2;
      readingIndex = (readingIndex + 1) % NUM_READINGS;
      if (lecturasGuardadas < NUM_READINGS) lecturasGuardadas++;

      if (lecturasGuardadas >= MIN_LECTURAS_MOSTRAR) {
        long pB, pS;
        promedios(pB, pS);
        int bpmProm = pB - BPM_OFFSET;
        if (bpmProm < 30) bpmProm = 30;
        ultimoSpo2 = pS;
        ultimoBpm = bpmProm;
        LOG("BPM promedio: %d  SpO2 promedio: %d%%  Temp: %.1f C\n", bpmProm, (int)pS, ultimaTemp);
      } else {
        LOG("Confirmando lectura...\n");
      }
    }
  } else {
    LOG("Señal no válida - mantené el dedo quieto y firme\n");
    if (++ciclosInvalidos >= CICLOS_INVALIDOS_MAX) {
      ultimoSpo2 = 0;
      ultimoBpm = 0;
      lecturasGuardadas = 0;
      readingIndex = 0;
    }
  }

#if USAR_MPU && USAR_GIROSCOPIO
  float gx, gy, gz;
  leerGiroscopio(gx, gy, gz);
  LOG("Gyro X:%.2f Y:%.2f Z:%.2f deg/s\n", gx, gy, gz);
#endif

  LOG("--------------------------------------------------\n");
}
