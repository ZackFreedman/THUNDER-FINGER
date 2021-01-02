#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_DotStarMatrix.h>
#include <Adafruit_DotStar.h>
#include <Fonts/TomThumb.h>
#include <neosensory_bluefruit.h>

#define FLUXGATEPIN A4
#define FGREFERENCEPIN A5
#define DATAPIN    7
#define CLOCKPIN   16

#define BRIGHTNESS 75
//#define MAXREADING 3.0
#define MAXREADING 2000

Adafruit_DotStarMatrix matrix = Adafruit_DotStarMatrix(
                                  12, 6, DATAPIN, CLOCKPIN,
                                  DS_MATRIX_BOTTOM     + DS_MATRIX_LEFT +
                                  DS_MATRIX_ROWS + DS_MATRIX_PROGRESSIVE,
                                  DOTSTAR_BGR);

NeosensoryBluefruit NeoBluefruit;
float **rumble_frames;

#define timeDelta 70
#define window 2000
#define stability 150

#define historyLength (window / timeDelta)

int readingHistory[historyLength] = {};
byte historyPointer;

int zeroPoint = 0;

float displayHistory[12];

const uint16_t colorScheme[] = {
  matrix.Color(255, 0, 0),   //A red
  matrix.Color(255, 125, 0), //D orange
  matrix.Color(200, 255, 0), //A yellowish
  matrix.Color(0, 255, 0),   //F green
  matrix.Color(0, 255, 225), //R blue
  matrix.Color(150, 0, 255), //U purple
  matrix.Color(255, 0, 220), //I pink
  matrix.Color(255, 65, 0),  //T reddish
  matrix.Color(255, 220, 0)  //! orange/yellow
};

void setup() {
  Serial.begin(9600);

  analogReference(AR_INTERNAL_1_8); // 1.8V reference, up to 1.8V
  analogReadResolution(14);

  matrix.begin();
  matrix.setFont(&TomThumb);
  matrix.setTextWrap(false);
  matrix.setBrightness(BRIGHTNESS);

  NeoBluefruit.begin();
  NeoBluefruit.setConnectedCallback(onConnected);
  NeoBluefruit.setDisconnectedCallback(onDisconnected);
  NeoBluefruit.setReadNotifyCallback(onReadNotify);
  NeoBluefruit.startScan();
  set_rumble_frames();

  for (int i = 0; i < historyLength; i++) {
    readingHistory[i] = getReading();
  }

  zeroPoint = readingHistory[0];
}

void loop() {
  unsigned long benchmark = millis();
  
  for (int i = 0; i < 11; i++) {
    displayHistory[i] = displayHistory[i + 1];
  }

  int sensorValue = getReading();
  readingHistory[historyPointer] = sensorValue;
  historyPointer = (historyPointer + 1) % historyLength;

  int total = 0;
  for (int i = 0; i < historyLength; i++) {
    total += readingHistory[i];
  }

  int average = total / historyLength;

  bool resetZero = true;
  int minimum = readingHistory[0];
  int maximum = readingHistory[0];

  if (abs(average - zeroPoint) >= stability) {
    for (int i = 0; i < historyLength; i++) {
      if (abs(readingHistory[i] - zeroPoint) <= stability) {
        resetZero = false;
        break;
      }

      if (readingHistory[i] < minimum)
        minimum = readingHistory[i];
      else if (readingHistory[i] > maximum)
        maximum = readingHistory[i];
    }
  }
  else
    resetZero = false;

  if (resetZero && maximum - minimum <= stability) {
    zeroPoint = average;
    Serial.print("New zero: ");
    Serial.println(zeroPoint);
  }

  int compensatedValue = abs(sensorValue - zeroPoint);

  compensatedValue += sin(double(millis()) / 300.0) * 1000.0 + 800.0;

  Serial.print("Reading: ");
  Serial.print(sensorValue);
  Serial.print("\tCompensated: ");
  Serial.println(compensatedValue);

  displayHistory[11] = min(float(compensatedValue) / float(MAXREADING) * 7.0, 6);

  for (int i = 0; i < 12; i++) {
    int ledsToLight = displayHistory[i] - 1;

    for (int j = 0; j < 6; j++) {
      if (ledsToLight >= j)
        matrix.drawPixel(i, j, colorScheme[ledsToLight]);
      else
        matrix.drawPixel(i, j, 0);
    }
  }

  matrix.show();

  if (NeoBluefruit.isConnected() && NeoBluefruit.isAuthorized()) {
    float intensity = float(compensatedValue) / float(MAXREADING);
    
    float intensities[] = {
      intensity,
      intensity,
      intensity,
      intensity
    };

    NeoBluefruit.vibrateMotors(intensities);
  }
  else delay(timeDelta);  // Allow the BT stuff to delay

  Serial.print("That took ");
  Serial.println(millis() - benchmark);
}

void set_rumble_frames() {
  rumble_frames = new float*[NeoBluefruit.max_frames_per_bt_package()];
  for (int i = 0; i < NeoBluefruit.max_frames_per_bt_package(); i++) {
    rumble_frames[i] = new float[NeoBluefruit.num_motors()];
    for (int j = 0; j < NeoBluefruit.num_motors(); j++) {
      rumble_frames[i][j] = (i % 2) == (j % 2);
    }
  }
}

void rumble() {
  NeoBluefruit.vibrateMotors(rumble_frames, 1);
}

void onConnected(bool success) {
  if (!success) {
    Serial.println("Attempted connection but failed.");
    return;
  }
  Serial.println("Connected!");

  // Once we are successfully connected to the wristband,
  // send developer autherization command and commands
  // to stop sound-to-touch algorithm.
  NeoBluefruit.authorizeDeveloper();
  NeoBluefruit.acceptTermsAndConditions();
  NeoBluefruit.stopAlgorithm();
}

void onDisconnected(uint16_t conn_handle, uint8_t reason) {
  Serial.println("\nDisconnected");
}

void onReadNotify(BLEClientCharacteristic * chr, uint8_t* data, uint16_t len) {
  for (int i = 0; i < len; i++) {
    Serial.write(data[i]);
  }
}

int getReading() {
  return constrain(abs(int(analogRead(FLUXGATEPIN)) - int(analogRead(FGREFERENCEPIN))), 0, 8000);
}
