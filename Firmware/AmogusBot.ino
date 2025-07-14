#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SPIFFS.h>
#include <FS.h>
#include <ContinuousStepper.h>
#include <LittleFS.h>
#include <ESP_I2S.h>

#define PIN_DIR 13
#define PIN_STEP 14
#define PIN_DIR_2 12
#define PIN_STEP_2 27

#define SDA_PIN 4
#define SCL_PIN 16
#define FORWARD 1
#define REVERSE 0

#define I2S_LRC 25
#define I2S_BCLK 33
#define I2S_DIN 26

i2s_data_bit_width_t bps = I2S_DATA_BIT_WIDTH_16BIT;
i2s_mode_t mode = I2S_MODE_STD;
i2s_slot_mode_t slot = I2S_SLOT_MODE_MONO;

float Kp = 80.0;
float Ki = 0.5;
float Kd = 0.1;

float angleSetpoint = 0;
float integral = 0, prevError = 0;
float dFiltered = 0;
unsigned long prevTime = 0;

bool stopRequested = false;

File wavFile;
I2SClass i2s;

// Creates an instance
ContinuousStepper<StepperDriver> stepper;
ContinuousStepper<StepperDriver> stepper2;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

bool playWAV(const char* path);

void audioTask(void *param) {
  const char* path = (const char*)param;
  playWAV(path);
  vTaskDelete(NULL);
}

void playAudioNonBlocking(const char* path) {
  // create a 2 KB stack task at priority 1
  xTaskCreatePinnedToCore(
    audioTask,
    "audio",
    2048,
    (void*)path,
    1,
    NULL,
    0  // run on core 0
  );
}

float readAngle() {
  sensors_event_t event;
  accel.getEvent(&event);
  int16_t x = event.acceleration.x;
  int16_t y = event.acceleration.y;
  int16_t z = event.acceleration.z;
  float xf = x / 256.0, yf = y / 256.0, zf = z / 256.0;
  return atan2(-xf, sqrt(yf * yf + zf * zf)) * 180.0 / PI;
}

uint32_t readLE32(File &f) {
  uint8_t b[4];
  f.read(b,4);
  return uint32_t(b[0]) | (uint32_t(b[1])<<8) | (uint32_t(b[2])<<16) | (uint32_t(b[3])<<24);
}

void stopWAV() {
  stopRequested = true;
}

bool playWAV(const char* path) {
  stopRequested = false;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) { Serial.println("open failed"); return false; }

  // 1) Check RIFF/WAVE
  char tag[4];
  f.read((uint8_t*)tag,4);
  if (memcmp(tag,"RIFF",4)) { Serial.println("no RIFF"); f.close(); return false; }
  f.seek(8);                // skip riffSize + "WAVE"
  f.read((uint8_t*)tag,4);
  if (memcmp(tag,"WAVE",4)) { Serial.println("no WAVE"); f.close(); return false; }

  // 2) Walk chunks until "fmt " then until "data"
  uint16_t audioFormat=0, channels=0;
  uint32_t sampleRate=0, bitsPerSample=0, dataSize=0;

  while (f.available()) {
    f.read((uint8_t*)tag,4);
    uint32_t chunkSize = readLE32(f);

    if (memcmp(tag,"fmt ",4)==0) {
      // parse fmt chunk
      audioFormat   = f.read() | (f.read()<<8);
      channels      = f.read() | (f.read()<<8);
      sampleRate    = readLE32(f);
      f.seek(f.position()+6);                // skip byteRate (4) + blockAlign (2)
      bitsPerSample = f.read() | (f.read()<<8);
      // skip any extra fmt bytes
      if (chunkSize > 16) f.seek(f.position() + (chunkSize - 16));
    }
    else if (memcmp(tag,"data",4)==0) {
      dataSize = chunkSize;
      break;
    }
    else {
      // skip over unknown chunk
      f.seek(f.position() + chunkSize);
    }
  }

  if (!dataSize) {
    Serial.println("no data chunk");
    f.close();
    return false;
  }

  // 3) init I2S
  if (!audioFormat || audioFormat != 1) {
    Serial.println("unsupported audio format");
    f.close();
    return false;
  }

  // 4) stream in small buffers
  const size_t BUF_SZ = 256;
  uint8_t buf[BUF_SZ];
  uint32_t remaining = dataSize;

   while (remaining && !stopRequested) {
    size_t toRead = remaining > BUF_SZ ? BUF_SZ : remaining;
    size_t got = f.read(buf, toRead);
    if (!got) break;
    size_t written;
    i2s.write(buf, got);
    remaining -= got;
  }

  f.close();
  return !stopRequested;
}

  f.close();
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  while (!Serial);
  Serial.println("Ready!");

  stepper.begin(PIN_STEP, PIN_DIR);
  stepper2.begin(PIN_STEP_2, PIN_DIR_2);
  while (!accel.begin()) {
    Serial.println("No ADXL345 detected. Check wiring!");
  }
  Serial.println("ADXL345 initialized.");

  i2s.setPins(I2S_BCLK, I2S_LRC, I2S_DIN);

  if (!i2s.begin(mode, 7350, bps, slot)) {
    Serial.println("Failed to initialize I2S!");
    return;
  }

  if (!LittleFS.begin(true)) {
    Serial.println("error with littlefs");
    return;
  }
  Serial.println("LittleFS started");

  prevTime = millis();
}
long counter = 0;
int speed1 = 0;
int speed2 = 0;
void loop() {
counter += 1;
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;
  if (dt <= 0.0) dt = 1e-3;
/*
  float angle = readAngle();
  float error = angleSetpoint - angle;
  integral += error * dt;
  integral = constrain(integral, -100.0, 100.0);
  float rawD = (error - prevError) / dt;
  const float alpha = 0.1;
  dFiltered = alpha * rawD + (1 - alpha) * dFiltered;
  prevError = error;

  float output = Kp * error + Ki * integral + Kd * dFiltered;
  output = constrain(output, -2000, 2000);
  Serial.print(angle); Serial.print("°  PID: "); Serial.println(output);
*/
 if (playWAV("/drip.wav")) {
    Serial.println("Playback done");
  } else {
    Serial.println("Playback error");
 }
if (counter%1000 == 0)
{
//speed1 = random(-1000,1000);
//speed2 = random(-1000,1000);
}
if (counter%2000 == 0)
{
stopRequested = !StopRequested;
}
  //stepper.spin(output);
  //stepper2.spin(-output);
  //stepper.loop();
  //stepper2.loop();
}
