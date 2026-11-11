// =====================================================================
//  MOON – Pulsera (ESP32-C3 Super Mini + MAX30102 + MPU6050)
//  Versión MUÑECA: mide pulso y SpO2 apoyando el sensor en la muñeca.
//
//  En la muñeca la señal del pulso es ~10 veces más débil que en el dedo,
//  así que esta versión:
//   - pone los LED (rojo e infrarrojo) al MÁXIMO y con el pulso de luz
//     más largo, para que llegue la mayor cantidad de luz posible;
//   - usa un rango del ADC más amplio para que esa luz no lo sature;
//   - NO usa el algoritmo de Maxim (spo2_algorithm.h), que en la muñeca
//     casi siempre da "no válido". En su lugar filtra la señal y detecta
//     cada latido uno por uno;
//   - con movimiento (siempre hay, es una pulsera): resta el movimiento
//     que mide el MPU6050 de la señal del pulso (filtro adaptativo) y
//     sigue el pulso por frecuencia, ignorando el ritmo del brazo.
//     El SpO2 se mide con la muñeca quieta y se mantiene mientras tanto.
//
//  Lo que se manda a la web NO cambia: "spo2,bpm,temp,g" cada 500 ms
//  por BLE (mismos UUID), así que la página sigue funcionando igual.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "MAX30105.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- Configuración ----------------
// 1 = imprime diagnóstico por el monitor serie. Ponelo en 0 cuando la
// pulsera ande con batería: imprimir gasta CPU y energía.
#define DEBUG 1

// Estos UUID tienen que ser EXACTAMENTE los mismos que en la página web
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_DEVICE_NAME         "MOON"

#define CPU_MHZ 80                 // 80 MHz alcanza para todo (BLE incluido) y gasta menos que 160
#define BLE_INTERVALO_MS 500       // la web cuenta muestras de caída con este ritmo: no lo subas
#define G_INTERVALO_MS 10          // lectura del acelerómetro (~100 por segundo)
#define TEMP_INTERVALO_MS 30000    // la temperatura casi no cambia: se lee cada 30 s

// ---- Sensor de pulso (MAX30102) ----
#define LED_BRILLO_MAX 255         // LED al máximo (~50 mA por pulso)
#define LED_BRILLO_MIN 64          // nunca baja de acá (solo baja si se satura)
#define SENSOR_SAMPLE_RATE 200     // muestras por segundo del sensor...
#define SENSOR_PROMEDIO 4          // ...promediadas de a 4 = 50 muestras/s limpias
#define FS_NOMINAL 50.0f           // = SENSOR_SAMPLE_RATE / SENSOR_PROMEDIO
#define SENSOR_ANCHO_PULSO 411     // µs: el pulso de luz más largo = más señal
#define SENSOR_RANGO_ADC 16384     // rango más amplio: con LED al máximo no se satura
#define IR_SATURADO 250000         // el ADC llega a 262143: arriba de esto está saturado

// Contacto con la piel: con el sensor apoyado en la muñeca el IR "de fondo"
// suele estar entre 30000 y 200000; en el aire, unos pocos miles.
// Mirá el valor "IR:" en el monitor serie y ajustalo si hace falta.
#define IR_CONTACTO_MIN 20000
#define ESTABILIZAR_MS 1500        // espera al apoyar el sensor para que la señal se asiente

// ---- Detección de latidos ----
#define BPM_MIN 40
#define BPM_MAX 180
#define LATIDOS_GUARDADOS 8        // se usa la mediana de los últimos 8 latidos
#define LATIDOS_PARA_MOSTRAR 4     // latidos coherentes antes de mostrar el primer valor
#define TOLERANCIA_LATIDO 0.30f    // un latido nuevo puede diferir hasta 30% de la mediana
#define SIN_LATIDO_MS 6000         // sin un latido bueno en este tiempo se reinicia el detector

// Movimiento: si la aceleración se aparta de 1 g más que esto, se considera
// que la muñeca se está moviendo.
#define MOVIMIENTO_G 0.12f
#define MOVIMIENTO_PAUSA_MS 1000

// ---- Medición con movimiento (es una pulsera: siempre hay movimiento) ----
// 1) Cancelación de movimiento: un filtro adaptativo (NLMS) aprende cuánto
//    del movimiento que mide el MPU6050 se "mete" en la señal del pulso y
//    lo resta.
// 2) Pulso por frecuencia: cada segundo se mira qué ritmos hay en los
//    últimos 8 s de señal. Se descarta el ritmo del brazo (el que también
//    ve el acelerómetro) y se sigue el ritmo del corazón de un segundo al
//    siguiente.
// 3) El SpO2 solo se actualiza con la muñeca quieta (como los relojes):
//    con movimiento no es confiable. Mientras tanto se mantiene el último.
#define NLMS_TAPS 6                // memoria del filtro (muestras por eje)
#define NLMS_MU 0.03f              // velocidad de aprendizaje
#define ESPECTRO_SEG 8             // segundos de señal que se analizan
#define ESPECTRO_FS 25             // la señal se reduce a 25 muestras/s para analizar
#define HR_SALTO_MAX 15            // BPM que puede cambiar el pulso de un segundo a otro
#define PULSO_RETENER_MS 10000     // sin una lectura confiable, el último pulso dura esto
#define SPO2_RETENER_MS 120000     // el último SpO2 (medido quieto) dura esto
#define SIN_CONTACTO_MUESTRAS 25   // ~0,5 s seguidos sin piel para dar por perdido el contacto

#define USAR_MPU 1
// 1 = giroscopio prendido (se lee y se imprime en el diagnóstico).
// 0 = giroscopio apagado: ahorra ~3,5 mA. Hoy la web solo usa la
//     aceleración (g), así que podés apagarlo si no lo necesitás.
#define USAR_GIROSCOPIO 1

#define SDA_PIN 8
#define SCL_PIN 9

#if DEBUG
  // Solo imprime si hay una compu escuchando: con batería (sin USB) imprimir
  // puede trabar el programa y hacer perder muestras del sensor.
  #define LOG(...)   do { if (Serial) Serial.printf(__VA_ARGS__); } while (0)
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
  pAdv->setMinInterval(320);   // 200 ms (unidades de 0,625 ms)
  pAdv->setMaxInterval(480);   // 300 ms
  BLEDevice::startAdvertising();
  LOG("BLE listo, anunciando como '" BLE_DEVICE_NAME "'\n");
}

// Manda "spo2,bpm,temp,g". Sin lectura válida manda "nan,nan,temp,g":
// la web ignora los signos vitales pero sigue procesando la aceleración.
void enviarVitalesBLE(int spo2Val, int bpmVal, float tempVal, float gVal) {
  if (!bleConectado) return;
  char msg[40];
  int len = (spo2Val > 0 && bpmVal > 0)
    ? snprintf(msg, sizeof(msg), "%d,%d,%.1f,%.2f", spo2Val, bpmVal, tempVal, gVal)
    : snprintf(msg, sizeof(msg), "nan,nan,%.1f,%.2f", tempVal, gVal);
  pVitalsChar->setValue((uint8_t *)msg, len);
  pVitalsChar->notify();
}

// ---------------- Estado general ----------------
MAX30105 particleSensor;

int ultimoSpo2 = 0;          // 0 = sin lectura válida
int ultimoBpm = 0;
float ultimaTemp = 0.0f;
float gReportado = 1.0f;     // pico de aceleración desde el último envío (1.0 = quieto)
float accX = 0, accY = 0, accZ = 1;   // última aceleración por eje (en g)
uint32_t ultimoMovimiento = 0;
bool pulsoOk = false;        // ¿se encontró el MAX30102?
bool mpuOk = false;          // ¿se encontró el MPU6050?

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
  // WHO_AM_I (registro 0x75) tiene que responder 0x68: así sabemos que está conectado
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);
  uint8_t quien = 0;
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint16_t)MPU6050_ADDR, (uint8_t)1, true) == 1) quien = Wire.read();
  mpuOk = (quien == 0x68 || quien == 0x70 || quien == 0x72 || quien == 0x98);   // 0x68 original; otros = clones compatibles
  if (!mpuOk) {
    LOG("ATENCIÓN: no se encontró el MPU6050 (respondió 0x%02X). Sin él no hay detección de caídas ni cancelación de movimiento. Revisá el cableado.\n", quien);
    return;
  }
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
  accX = x * (1.0f / ACCEL_SENSITIVITY);   // se guardan los ejes para cancelar el movimiento
  accY = y * (1.0f / ACCEL_SENSITIVITY);
  accZ = z * (1.0f / ACCEL_SENSITIVITY);
  return sqrtf(accX * accX + accY * accY + accZ * accZ);
}

void actualizarG() {
  static uint32_t ultimo = 0;
  uint32_t ahora = millis();
  if (ahora - ultimo < G_INTERVALO_MS) return;
  ultimo = ahora;
  if (!mpuOk) return;
  float g = leerG();
  float desvio = fabsf(g - 1.0f);
  if (desvio > fabsf(gReportado - 1.0f)) gReportado = g;
  if (desvio > MOVIMIENTO_G) ultimoMovimiento = ahora;   // la muñeca se está moviendo
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

// =====================================================================
//  PROCESAMIENTO DE LA SEÑAL DEL PULSO
// =====================================================================

// Filtro pasa-bajos de 2º orden (Butterworth). Deja pasar el pulso
// (hasta ~4 Hz = 240 bpm) y saca el ruido rápido.
struct Biquad {
  float b0, b1, b2, a1, a2, z1 = 0, z2 = 0;
  void pasaBajos(float fc, float fs) {
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float c = cosf(w0), al = sinf(w0) / (2.0f * 0.7071f);
    float a0 = 1.0f + al;
    b0 = (1.0f - c) / 2.0f / a0;  b1 = (1.0f - c) / a0;  b2 = b0;
    a1 = -2.0f * c / a0;          a2 = (1.0f - al) / a0;
  }
  void pasaAltos(float fc, float fs) {
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float c = cosf(w0), al = sinf(w0) / (2.0f * 0.7071f);
    float a0 = 1.0f + al;
    b0 = (1.0f + c) / 2.0f / a0;  b1 = -(1.0f + c) / a0;  b2 = b0;
    a1 = -2.0f * c / a0;          a2 = (1.0f - al) / a0;
  }
  float paso(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void reiniciar() { z1 = z2 = 0; }
};

// Un canal de luz (rojo o infrarrojo): separa la parte fija (DC, la piel y
// el hueso) de la parte que late (AC, la sangre).
// Ruido blanco: la 2ª diferencia tiene 6 veces su varianza, y el filtro de
// 4 Hz deja pasar ~18% (4,44 Hz de ancho efectivo sobre 25 Hz). 0,178/6:
#define RUIDO_FACTOR 0.0296f
// Un latido se usa para SpO2 solo si al menos el 50% de su energía es pulso
#define CALIDAD_MIN 0.5f

struct Canal {
  float dc = 0;
  Biquad filtro;
  float x1 = 0, x2 = 0;                // dos muestras anteriores (para medir el ruido)
  float energia = 0, ruido = 0;        // acumulados dentro del latido actual
  float procesar(uint32_t x) {
    if (dc == 0) { dc = x; x1 = x2 = x; }
    // Ruido: la "segunda diferencia" casi no tiene pulso (que es lento) y sí
    // todo el ruido rápido. Se pasa a "ruido que queda después del filtro".
    float d = x - 2.0f * x1 + x2;
    x2 = x1; x1 = x;
    ruido += d * d * RUIDO_FACTOR;
    dc += (x - dc) * (1.0f / 64.0f);    // promedio lento (~1,3 s)
    float ac = filtro.paso(x - dc);
    energia += ac * ac;
    return ac;
  }
  // Energía del pulso con el ruido restado (0 si el ruido tapa al pulso)
  float energiaLimpia() { return energia - ruido; }
  float calidad() { return energia > 0 ? 1.0f - ruido / energia : 0; }
  void nuevoLatido() { energia = 0; ruido = 0; }
  void reiniciar() { dc = 0; filtro.reiniciar(); nuevoLatido(); }
};

Canal canalIR, canalRojo;

uint8_t brilloLed = LED_BRILLO_MAX;
float fsReal = FS_NOMINAL;          // muestras/s medidas de verdad (los módulos clon varían)

// Historial de latidos
float intervalos[LATIDOS_GUARDADOS];   // ms entre latidos
float energiaRojo[LATIDOS_GUARDADOS];  // energía del pulso rojo de cada latido (para SpO2)
float energiaIR[LATIDOS_GUARDADOS];    // energía del pulso IR de cada latido
int nLatidos = 0, idxLatido = 0;
int nR = 0, idxR = 0;                  // latidos limpios guardados para el SpO2
int rechazosSeguidos = 0;
uint32_t ultimoLatidoBueno = 0;
int bpmLatidos = 0;                    // pulso medido latido a latido (muñeca quieta)
uint32_t spo2Ms = 0;                   // cuándo se midió el último SpO2

// Estado del detector
uint32_t nMuestra = 0, muestraUltimoPico = 0;
bool hayPicoPrevio = false;         // false = todavía no hubo un pico desde el reinicio
float s1 = 0, s2 = 0;               // las dos muestras anteriores de la señal
float envolvente = 0;               // amplitud típica del pulso (para el umbral)
bool hayContacto = false;
uint32_t contactoDesde = 0;
int muestrasSaturadas = 0;

float mediana(const float *v, int n) {
  float t[LATIDOS_GUARDADOS];
  memcpy(t, v, n * sizeof(float));
  for (int i = 1; i < n; i++) {            // ordenamiento por inserción (n <= 8)
    float k = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return (n % 2) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

// Reinicia el detector latido a latido (no borra lo que se muestra)
void borrarLatidos() {
  nLatidos = 0; idxLatido = 0; rechazosSeguidos = 0;
  nR = 0; idxR = 0;
  bpmLatidos = 0;
}
// Borra lo que se muestra (por ejemplo, al sacar la pulsera)
void borrarSalida() {
  ultimoSpo2 = 0; ultimoBpm = 0; spo2Ms = 0;
}

void reiniciarMovimiento();   // está más abajo

void reiniciarSenal() {
  canalIR.reiniciar(); canalRojo.reiniciar();
  s1 = s2 = 0; envolvente = 0;
  muestraUltimoPico = nMuestra;
  hayPicoPrevio = false;
  borrarLatidos();
  borrarSalida();
  reiniciarMovimiento();
}

void cambiarBrillo(uint8_t b) {
  brilloLed = b;
  particleSensor.setPulseAmplitudeRed(b);
  particleSensor.setPulseAmplitudeIR(b);
  contactoDesde = millis();   // con brillo nuevo la señal tiene que volver a asentarse
  reiniciarSenal();
  LOG("IR saturado -> brillo LED baja a %d\n", b);
}

// =====================================================================
//  CANCELACIÓN DE MOVIMIENTO Y PULSO POR FRECUENCIA
// =====================================================================

// Pasa-banda 0,5-4 Hz (30-240 bpm): deja solo la zona donde está el pulso
struct PasaBanda {
  Biquad pa, pb;
  void configurar(float fs) { pa.pasaAltos(0.5f, fs); pb.pasaBajos(4.0f, fs); }
  float paso(float x) { return pb.paso(pa.paso(x)); }
  void reiniciar() { pa.reiniciar(); pb.reiniciar(); }
};
PasaBanda bpPulso, bpAx, bpAy, bpAz;

// Filtro adaptativo NLMS. Referencias: los 3 ejes y los 3 ejes al cuadrado
// (el movimiento no "ensucia" la señal de forma perfectamente proporcional:
// los cuadrados capturan esa parte, que crea ritmos al doble del brazo).
#define NLMS_REFS 6
#define NLMS_N (NLMS_REFS * NLMS_TAPS)
float nlmsX[NLMS_N];
float nlmsW[NLMS_N];
float mediaCuad[3] = {0, 0, 0};

float cancelarMovimiento(float pulso, float ax, float ay, float az) {
  float refs[NLMS_REFS] = { ax, ay, az, ax * ax, ay * ay, az * az };
  for (int e = 0; e < 3; e++) {            // a los cuadrados se les saca el promedio
    mediaCuad[e] += (refs[3 + e] - mediaCuad[e]) * 0.02f;
    refs[3 + e] -= mediaCuad[e];
  }
  // corre la memoria y pone las muestras nuevas de cada referencia al principio
  for (int e = 0; e < NLMS_REFS; e++) {
    float *v = nlmsX + e * NLMS_TAPS;
    memmove(v + 1, v, (NLMS_TAPS - 1) * sizeof(float));
    v[0] = refs[e];
  }
  float y = 0, potencia = 0;
  for (int i = 0; i < NLMS_N; i++) { y += nlmsW[i] * nlmsX[i]; potencia += nlmsX[i] * nlmsX[i]; }
  float limpio = pulso - y;              // lo que queda = pulso sin el movimiento
  if (millis() - ultimoMovimiento < 2000) {
    // Aprende solo cuando hay movimiento: quieto, el acelerómetro solo tiene
    // ruido y el filtro aprendería cualquier cosa (y después, al moverse,
    // restaría de más).
    float paso = NLMS_MU * limpio / (potencia + 1e-2f);
    for (int i = 0; i < NLMS_N; i++) nlmsW[i] += paso * nlmsX[i];
  } else {
    for (int i = 0; i < NLMS_N; i++) nlmsW[i] *= 0.999f;   // olvida de a poco
  }
  return limpio;
}

// Historial reducido a 25 muestras/s para el análisis por frecuencia
#define ESPECTRO_N (ESPECTRO_SEG * ESPECTRO_FS)
float histPulso[ESPECTRO_N];
float histAcc[3][ESPECTRO_N];
int histIdx = 0, histLlenas = 0;
float accumP = 0, accumA[3] = {0, 0, 0};
int accumN = 0;

// Rango de búsqueda: 42 a 180 bpm, de a 0,05 Hz (3 bpm), con interpolación
#define F_MIN 0.70f
#define F_PASO 0.05f
#define N_FREC 47

// Potencia de la señal v a la frecuencia f (algoritmo de Goertzel)
float potenciaEn(const float *v, float f) {
  float w = 2.0f * (float)M_PI * f / ESPECTRO_FS;
  float coef = 2.0f * cosf(w), q1 = 0, q2 = 0;
  for (int i = 0; i < ESPECTRO_N; i++) {
    float q0 = coef * q1 - q2 + v[i];
    q2 = q1; q1 = q0;
  }
  return q1 * q1 + q2 * q2 - coef * q1 * q2;
}
void leerSensor();     // están más abajo
void actualizarG();

uint32_t generacion = 0;         // cambia cada vez que se reinicia la señal
float bpmSeguido = 0;            // pulso que se viene siguiendo por frecuencia
uint32_t bpmSeguidoMs = 0;       // cuándo se confirmó por última vez
float confianzaEspectro = 0;     // qué tan claro es el pico (para el diagnóstico)

// Copia de la ventana en orden (así el sensor puede seguir leyéndose
// mientras se analiza, sin mezclar datos)
static float copiaPulso[ESPECTRO_N];
static float copiaAcc[3][ESPECTRO_N];

// ¿f está cerca del ritmo del brazo o de sus "ecos" (mitad, doble, triple)?
// Margen de 0,15 Hz: con 8 s de señal, cada ritmo se "desparrama" ~0,125 Hz.
bool cercaDelBrazo(float f, float fBrazo, float m = 0.15f) {
  return fabsf(f - fBrazo) <= m || fabsf(f - 2 * fBrazo) <= m ||
         fabsf(f - 3 * fBrazo) <= m || fabsf(f - 0.5f * fBrazo) <= m;
}
float candidatoBpm = 0;   // para engancharse con movimiento hacen falta
int candidatoVeces = 0;   // 3 análisis seguidos que coincidan

// Se llama una vez por segundo con la ventana llena
void analizarFrecuencias(bool moviendose) {
  if (histLlenas < ESPECTRO_N) return;
  const uint32_t gen = generacion;   // si durante el análisis se saca la pulsera, se descarta
  for (int i = 0; i < ESPECTRO_N; i++) {
    int j = (histIdx + i) % ESPECTRO_N;
    copiaPulso[i] = histPulso[j];
    for (int e = 0; e < 3; e++) copiaAcc[e][i] = histAcc[e][j];
  }
  static float P[N_FREC], A[N_FREC], Porig[N_FREC];
  float maxA = 1e-12f, sumaP = 0, fBrazo = 0;
  for (int k = 0; k < N_FREC; k++) {
    float f = F_MIN + k * F_PASO;
    P[k] = potenciaEn(copiaPulso, f);
    Porig[k] = P[k];
    A[k] = 0;
    if (moviendose) for (int e = 0; e < 3; e++) A[k] += potenciaEn(copiaAcc[e], f);
    if (A[k] > maxA) { maxA = A[k]; fBrazo = f; }
    // la ESP32-C3 calcula los decimales lento: entre frecuencia y frecuencia
    // se siguen leyendo los sensores para no perder muestras
    leerSensor();
    actualizarG();
    if (gen != generacion) return;   // se reinició la señal (sin contacto, saturación...)
  }
  // Se "apagan" las frecuencias del brazo (y su doble y su mitad)
  for (int k = 0; k < N_FREC; k++) {
    float f = F_MIN + k * F_PASO;
    if (moviendose) {
      P[k] /= (1.0f + 10.0f * A[k] / maxA);
      if (cercaDelBrazo(f, fBrazo)) P[k] *= 0.15f;
    }
    sumaP += P[k];
  }
  // Si ya veníamos siguiendo un pulso, se busca cerca de ese valor
  int kIni = 0, kFin = N_FREC - 1;
  if (bpmSeguido > 0) {
    float fPrev = bpmSeguido / 60.0f, ancho = HR_SALTO_MAX / 60.0f;
    kIni = (int)ceilf((fPrev - ancho - F_MIN) / F_PASO);
    kFin = (int)floorf((fPrev + ancho - F_MIN) / F_PASO);
    if (kIni < 0) kIni = 0;
    if (kFin > N_FREC - 1) kFin = N_FREC - 1;
  }
  int kMax = kIni;
  for (int k = kIni; k <= kFin; k++) if (P[k] > P[kMax]) kMax = k;
  confianzaEspectro = P[kMax] / (sumaP / N_FREC + 1e-12f);
  // Sin un pulso previo y con movimiento, se exige mucha más claridad y que no
  // sea un ritmo del brazo: mejor no mostrar nada que un valor equivocado.
  float fMax = F_MIN + kMax * F_PASO;
  // Si el pulso que veníamos siguiendo (o el que se encontró) cae sobre el
  // ritmo del brazo, no se pueden separar: se mantiene el último valor bueno
  // en vez de correrlo hacia un valor equivocado.
  if (moviendose && bpmSeguido > 0 &&
      (cercaDelBrazo(bpmSeguido / 60.0f, fBrazo, 0.2f) || cercaDelBrazo(fMax, fBrazo, 0.2f))) return;
  if (bpmSeguido == 0) {
    // Para engancharse por primera vez (sin un pulso previo):
    if (confianzaEspectro < (moviendose ? 6.0f : 4.0f)) return;
    if (moviendose && (cercaDelBrazo(fMax, fBrazo) || fabsf(fMax - fBrazo) <= 0.25f)) return;
    // ¿Es un "eco" al doble de otro ritmo? (la onda del pulso y el movimiento
    // tienen ecos al doble de su ritmo). Si a la mitad hay fuerza, o la mitad
    // coincide con el brazo, no se arriesga.
    int kMitad = (int)lroundf((fMax / 2 - F_MIN) / F_PASO);
    if (kMitad >= 0 && Porig[kMitad] > 0.25f * Porig[kMax]) return;
    if (moviendose && kMitad >= 0 && fabsf(fMax / 2 - fBrazo) <= 0.15f) return;
    // tiene que repetirse varias veces seguidas (±6 bpm) antes de creerle
    float bpmCand = fMax * 60.0f;
    if (fabsf(bpmCand - candidatoBpm) <= 6.0f) candidatoVeces++;
    else { candidatoBpm = bpmCand; candidatoVeces = 1; }
    if (candidatoVeces < (moviendose ? 3 : 2)) return;
  }
  candidatoVeces = 0;
  if (confianzaEspectro < 2.5f) return;          // pico poco claro: no se actualiza
  // interpolación parabólica para afinar entre dos frecuencias
  float d = 0;
  if (kMax > 0 && kMax < N_FREC - 1) {
    float a = P[kMax - 1], b = P[kMax], c = P[kMax + 1];
    float den = a - 2 * b + c;
    if (den != 0) d = 0.5f * (a - c) / den;
    if (d > 0.5f) d = 0.5f;
    if (d < -0.5f) d = -0.5f;
  }
  // la ventana se tomó con la frecuencia nominal; se corrige con la real
  float bpm = (F_MIN + (kMax + d) * F_PASO) * 60.0f * (fsReal / FS_NOMINAL);
  if (bpm < BPM_MIN || bpm > BPM_MAX) return;
  bpmSeguido = (bpmSeguido == 0) ? bpm : 0.7f * bpmSeguido + 0.3f * bpm;
  bpmSeguidoMs = millis();
}

void reiniciarMovimiento() {
  generacion++;
  bpPulso.reiniciar(); bpAx.reiniciar(); bpAy.reiniciar(); bpAz.reiniciar();
  memset(nlmsX, 0, sizeof(nlmsX)); memset(nlmsW, 0, sizeof(nlmsW));
  mediaCuad[0] = mediaCuad[1] = mediaCuad[2] = 0;
  histIdx = 0; histLlenas = 0; accumP = 0; accumN = 0;
  accumA[0] = accumA[1] = accumA[2] = 0;
  bpmSeguido = 0; bpmSeguidoMs = 0;
  candidatoBpm = 0; candidatoVeces = 0;
}

// Se llama cuando el detector encuentra un pico (un latido)
void registrarLatido(uint32_t muestraPico) {
  float intervaloMs = (muestraPico - muestraUltimoPico) * 1000.0f / fsReal;
  bool hubo = hayPicoPrevio;
  hayPicoPrevio = true;
  muestraUltimoPico = muestraPico;

  // Relación rojo/IR de este latido (base del cálculo de SpO2)
  // R = (pulso rojo / fondo rojo) / (pulso IR / fondo IR), con el ruido
  // restado de cada uno para que no agrande la medida.
  // Se guardan las energías de cada latido y el SpO2 se calcula con la suma
  // de varios latidos: así el ruido de uno se compensa con el de otro.
  // Solo se filtra por la calidad del IR (la señal más fuerte).
  float eR = canalRojo.energiaLimpia() / (canalRojo.dc * canalRojo.dc);
  float eI = canalIR.energiaLimpia() / (canalIR.dc * canalIR.dc);
  bool limpio = canalIR.calidad() >= CALIDAD_MIN && eI > 0;
  canalIR.nuevoLatido(); canalRojo.nuevoLatido();

  if (!hubo) return;
  if (millis() - ultimoMovimiento < MOVIMIENTO_PAUSA_MS) return;   // se movió: no cuenta
  if (intervaloMs < 60000.0f / BPM_MAX || intervaloMs > 60000.0f / BPM_MIN) return;

  // ¿Se parece a los latidos anteriores?
  if (nLatidos >= 3) {
    float m = mediana(intervalos, nLatidos);
    if (fabsf(intervaloMs - m) > m * TOLERANCIA_LATIDO) {
      // Muchos rechazos seguidos = cambió el ritmo de verdad: se empieza de nuevo
      if (++rechazosSeguidos >= 4) borrarLatidos();
      return;
    }
  }
  rechazosSeguidos = 0;

  intervalos[idxLatido] = intervaloMs;
  idxLatido = (idxLatido + 1) % LATIDOS_GUARDADOS;
  if (nLatidos < LATIDOS_GUARDADOS) nLatidos++;
  // Para el SpO2 solo se guardan los latidos limpios (el pulso sirve igual)
  if (limpio) {
    energiaRojo[idxR] = eR;
    energiaIR[idxR] = eI;
    idxR = (idxR + 1) % LATIDOS_GUARDADOS;
    if (nR < LATIDOS_GUARDADOS) nR++;
  }
  ultimoLatidoBueno = millis();

  if (nLatidos >= LATIDOS_PARA_MOSTRAR) {
    bpmLatidos = (int)lroundf(60000.0f / mediana(intervalos, nLatidos));
  }
  // Curva de calibración de Maxim para el MAX30102. En la muñeca es
  // aproximada: comparala con un oxímetro de dedo y ajustá si hace falta.
  if (bpmLatidos && nR >= 3) {
    float sR = 0, sI = 0;
    for (int i = 0; i < nR; i++) { sR += energiaRojo[i]; sI += energiaIR[i]; }
    if (sR <= 0 || sI <= 0) return;
    float r = sqrtf(sR / sI);
    if (r < 0.2f || r > 3.0f) return;   // valor imposible: ruido
    float spo2 = -45.060f * r * r + 30.354f * r + 94.845f;
    if (spo2 > 100) spo2 = 100;
    // Suavizado: el valor mostrado se mueve de a poco y no salta con cada latido
    static float spo2Suave = 0;
    spo2Suave = (ultimoSpo2 == 0) ? spo2 : 0.75f * spo2Suave + 0.25f * spo2;
    // Valores muy bajos se mandan igual (con piso en 50) para que la web
    // pueda dar la alarma: nunca se esconde una lectura crítica.
    ultimoSpo2 = (int)lroundf(spo2Suave < 50 ? 50 : spo2Suave);
    spo2Ms = millis();
  }
}

// Decide qué pulso mostrar (se llama una vez por segundo)
void actualizarSalida(bool moviendose) {
  if (!hayContacto) return;          // sin la pulsera puesta no se muestra nada
  uint32_t ahora = millis();
  if (!moviendose && bpmLatidos) {
    // Quieto: el pulso latido a latido es el más preciso. Además "enseña"
    // al seguimiento por frecuencia dónde está el pulso.
    ultimoBpm = bpmLatidos;
    bpmSeguido = bpmLatidos;
    bpmSeguidoMs = ahora;
  } else if (bpmSeguido > 0 && ahora - bpmSeguidoMs < PULSO_RETENER_MS) {
    // Con movimiento: el pulso que se sigue por frecuencia
    ultimoBpm = (int)lroundf(bpmSeguido);
  } else if (ultimoBpm && ahora - bpmSeguidoMs >= PULSO_RETENER_MS && ahora - ultimoLatidoBueno >= PULSO_RETENER_MS) {
    ultimoBpm = 0;                     // hace mucho que no hay una lectura confiable
    bpmSeguido = 0;
  }
  if (ultimoSpo2 && ahora - spo2Ms > SPO2_RETENER_MS) ultimoSpo2 = 0;
}

// Procesa UNA muestra del sensor (se llama 50 veces por segundo)
void procesarMuestra(uint32_t rojo, uint32_t ir) {
  nMuestra++;

  // ---- Contacto con la piel ----
  // Con movimiento el IR puede bajar un instante: recién se da por perdido
  // el contacto si pasa ~0,5 s seguido.
  static int muestrasSinPiel = 0;
  if (ir < IR_CONTACTO_MIN) {
    if (++muestrasSinPiel < SIN_CONTACTO_MUESTRAS && hayContacto) return;
    if (hayContacto) { LOG("Sensor sin contacto con la piel\n"); reiniciarSenal(); }
    hayContacto = false;
    return;
  }
  muestrasSinPiel = 0;
  if (!hayContacto) {
    hayContacto = true;
    contactoDesde = millis();
    if (brilloLed != LED_BRILLO_MAX) {   // si antes se había bajado, vuelve al máximo
      brilloLed = LED_BRILLO_MAX;
      particleSensor.setPulseAmplitudeRed(brilloLed);
      particleSensor.setPulseAmplitudeIR(brilloLed);
    }
    reiniciarSenal();
    LOG("Contacto detectado, estabilizando...\n");
  }

  // ---- LED al máximo, salvo que se sature ----
  if (ir > IR_SATURADO || rojo > IR_SATURADO) {
    if (++muestrasSaturadas >= 10 && brilloLed > LED_BRILLO_MIN) {
      muestrasSaturadas = 0;
      cambiarBrillo(brilloLed - 32 < LED_BRILLO_MIN ? LED_BRILLO_MIN : brilloLed - 32);
    } else if (muestrasSaturadas % 250 == 0) {   // cada ~5 s
      LOG("ATENCIÓN: el sensor sigue saturado aun con el LED al mínimo (entra mucha luz de afuera?)\n");
    }
    return;
  }
  muestrasSaturadas = 0;

  canalIR.procesar(ir);
  canalRojo.procesar(rojo);

  // Señal del pulso (invertida: en la muñeca más sangre = MENOS luz) y
  // movimiento, los dos filtrados igual, y el movimiento restado.
  float pulso = bpPulso.paso(-((float)ir - canalIR.dc));
  float ax = bpAx.paso(accX), ay = bpAy.paso(accY), az = bpAz.paso(accZ);
  float s = cancelarMovimiento(pulso, ax, ay, az);

  // Historial a 25 muestras/s (promedio de a 2) para el análisis por frecuencia
  accumP += s; accumA[0] += ax; accumA[1] += ay; accumA[2] += az;
  if (++accumN >= (int)(FS_NOMINAL / ESPECTRO_FS)) {
    histPulso[histIdx] = accumP / accumN;
    for (int e = 0; e < 3; e++) { histAcc[e][histIdx] = accumA[e] / accumN; accumA[e] = 0; }
    histIdx = (histIdx + 1) % ESPECTRO_N;
    if (histLlenas < ESPECTRO_N) histLlenas++;
    accumP = 0; accumN = 0;
  }

  if (millis() - contactoDesde < ESTABILIZAR_MS) return;   // todavía asentándose

  // Umbral adaptable: 40% de la amplitud típica (que baja de a poco)
  envolvente *= 0.995f;
  if (fabsf(s) > envolvente) envolvente = fabsf(s);
  float umbral = 0.4f * envolvente;

  // ¿La muestra anterior fue un pico? (más alta que sus dos vecinas)
  uint32_t refractario = (uint32_t)(fsReal * 60.0f / BPM_MAX);
  if (s1 > s2 && s1 >= s && s1 > umbral && (nMuestra - 1 - muestraUltimoPico) > refractario) {
    registrarLatido(nMuestra - 1);
  }
  s2 = s1; s1 = s;
}

// Lee todas las muestras nuevas del sensor, en orden
void leerSensor() {
  if (!pulsoOk) return;
  particleSensor.check();
  while (particleSensor.available()) {
    procesarMuestra(particleSensor.getFIFORed(), particleSensor.getFIFOIR());
    particleSensor.nextSample();
  }
}

// Mide cuántas muestras por segundo entrega el sensor de verdad, para que
// los BPM salgan bien aunque el módulo no sea exacto (por esto antes había
// que restarle 45 BPM a mano).
bool reiniciarMedidaFs = true;   // se pone en true cuando el sensor (re)arranca

void medirFrecuencia() {
  static uint32_t t0 = 0, n0 = 0;
  static bool primeraVez = true;
  uint32_t ahora = millis();
  if (!pulsoOk) return;            // sin sensor no hay muestras que contar
  if (reiniciarMedidaFs) { reiniciarMedidaFs = false; t0 = ahora; n0 = nMuestra; primeraVez = true; return; }
  // La primera medición se hace rápido (2 s) y se usa directo; después se
  // va afinando cada 5 s.
  if (ahora - t0 < (primeraVez ? 2000u : 5000u)) return;
  float medida = (nMuestra - n0) * 1000.0f / (ahora - t0);
  if (medida > FS_NOMINAL * 0.5f && medida < FS_NOMINAL * 2.0f) {
    fsReal = primeraVez ? medida : 0.7f * fsReal + 0.3f * medida;
    if (primeraVez) LOG("El sensor entrega %.1f muestras/s\n", medida);
    primeraVez = false;
  }
  t0 = ahora; n0 = nMuestra;
}

bool iniciarSensorPulso() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    LOG("No se encontró el MAX30102. Revisá el cableado (sigo buscándolo; las caídas funcionan igual).\n");
    pulsoOk = false;
    return false;
  }
  // ledMode 2 = rojo + infrarrojo
  particleSensor.setup(LED_BRILLO_MAX, SENSOR_PROMEDIO, 2, SENSOR_SAMPLE_RATE,
                       SENSOR_ANCHO_PULSO, SENSOR_RANGO_ADC);
  brilloLed = LED_BRILLO_MAX;
  LOG("MAX30102 listo: LED al máximo (%d), %d muestras/s.\n", LED_BRILLO_MAX, (int)FS_NOMINAL);
  pulsoOk = true;
  particleSensor.clearFIFO();     // se descartan muestras viejas del arranque
  reiniciarMedidaFs = true;       // y se vuelve a medir la velocidad del sensor
  return true;
}

void setup() {
  setCpuFrequencyMhz(CPU_MHZ);

#if DEBUG
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 2000) delay(10);
#endif
  LOG("Arrancando...\n");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(50);

  // Si el MAX30102 no aparece, la pulsera NO se queda trabada: sigue con
  // Bluetooth y detección de caídas, y vuelve a buscarlo cada 5 s.
  iniciarSensorPulso();

  canalIR.filtro.pasaBajos(4.0f, FS_NOMINAL);
  canalRojo.filtro.pasaBajos(4.0f, FS_NOMINAL);
  bpPulso.configurar(FS_NOMINAL);
  bpAx.configurar(FS_NOMINAL); bpAy.configurar(FS_NOMINAL); bpAz.configurar(FS_NOMINAL);

#if USAR_MPU
  iniciarMPU();
#endif

  iniciarBLE();
}

void loop() {
  // Si el MAX30102 no estaba, se lo vuelve a buscar cada 5 s
  static uint32_t ultimoIntento = 0;
  if (!pulsoOk && millis() - ultimoIntento > 5000) {
    ultimoIntento = millis();
    iniciarSensorPulso();
  }

  actualizarG();
  leerSensor();
  medirFrecuencia();
  enviarPeriodico();

  uint32_t ahora = millis();

  // Si hace rato que no hay un latido limpio, se reinicia el detector
  // latido a latido (el pulso sigue por frecuencia mientras tanto)
  if (bpmLatidos && ahora - ultimoLatidoBueno > SIN_LATIDO_MS) borrarLatidos();

  // Una vez por segundo: análisis por frecuencia y qué valor mostrar
  static uint32_t ultimoAnalisis = 0;
  if (hayContacto && ahora - ultimoAnalisis >= 1000) {
    ultimoAnalisis = ahora;
    bool moviendose = (ahora - ultimoMovimiento < 2000);
    analizarFrecuencias(moviendose);
    actualizarSalida(moviendose);
  }

  // Temperatura (bloquea ~30 ms, por eso se lee poco)
  static uint32_t ultimaLecturaTemp = 0;
  if (pulsoOk && (ultimaLecturaTemp == 0 || ahora - ultimaLecturaTemp > TEMP_INTERVALO_MS)) {
    ultimaLecturaTemp = ahora;
    ultimaTemp = particleSensor.readTemperature();
    leerSensor();   // se leen enseguida las muestras que llegaron mientras tanto
  }

#if DEBUG
  static uint32_t ultimoLog = 0;
  if (ahora - ultimoLog >= 1000) {
    ultimoLog = ahora;
    float pi = canalIR.dc > 0 ? 100.0f * envolvente / canalIR.dc : 0;
    LOG("IR:%lu  LED:%d  fs:%.1f  señal:%.3f%%  latidos:%d  frec:%.0f(conf %.1f)  %s  BPM:%d  SpO2:%d\n",
        (unsigned long)canalIR.dc, brilloLed, fsReal, pi, nLatidos, bpmSeguido, confianzaEspectro,
        (ahora - ultimoMovimiento < 2000) ? "MOVIMIENTO" : "quieto",
        ultimoBpm, ultimoSpo2);
  #if USAR_MPU && USAR_GIROSCOPIO
    if (mpuOk) {
      float gx, gy, gz;
      leerGiroscopio(gx, gy, gz);
      LOG("Gyro X:%.2f Y:%.2f Z:%.2f deg/s\n", gx, gy, gz);
    }
  #endif
  }
#endif

  delay(2);   // cede la CPU (el FIFO del sensor guarda las muestras mientras tanto)
}
