
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// TCA9548A I2C çoklayıcı adresi
#define TCAADDR 0x70

MPU6050 mpu[3] = { MPU6050(0x68), MPU6050(0x68), MPU6050(0x68) };

#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

// MPU kontrol/durum değişkenleri
bool dmpReady[3] = { false, false, false };  // DMP başlatma başarılıysa true
uint8_t devStatus[3];                        // Her cihaz için geri dönüş durumu (0 = başarı)
uint16_t packetSize[3];                      // Beklenen DMP paket boyutu
uint8_t fifoBuffer[64];                      // FIFO depolama tamponu

// Dönüşüm hareket değişkenleri
Quaternion q[3];

// TCA9548A kanal seçimi
void tcaSelect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // MPU6050 cihazlarını başlatma
  for (int i = 0; i < 3; i++) {
    tcaSelect(i);
    Serial.print("MPU6050 (Kanal ");
    Serial.print(i);
    Serial.println(") başlatılıyor...");
    mpu[i].initialize();
    Serial.println(mpu[i].testConnection() ? "Bağlantı başarılı" : "Bağlantı başarısız");

    // DMP başlatma
    Serial.println("DMP başlatılıyor...");
    devStatus[i] = mpu[i].dmpInitialize();

    if (devStatus[i] == 0) {
      mpu[i].setDMPEnabled(true);
      Serial.println("DMP etkinleştirildi.");
      dmpReady[i] = true;
      packetSize[i] = mpu[i].dmpGetFIFOPacketSize();
    } else {
      Serial.print("DMP Başlatma başarısız (Kod ");
      Serial.print(devStatus[i]);
      Serial.println(")");
    }
  }
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  for (int i = 0; i < 3; i++) {
    if (!dmpReady[i]) continue;

    tcaSelect(i);

    if (mpu[i].dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu[i].dmpGetQuaternion(&q[i], fifoBuffer);

      // Veriyi Unity tarafından kolayca işlenebilir bir formatta yazdır
      Serial.print("Channel ");
      Serial.print(i);
      Serial.print(":");
      Serial.print(q[i].w);
      Serial.print(",");
      Serial.print(q[i].x);
      Serial.print(",");
      Serial.print(q[i].y);
      Serial.print(",");
      Serial.println(q[i].z);
    }
  }
}
