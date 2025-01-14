#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

#define PGA 0.602
#define acsOffset 1.67
#define acsSens 0.0134
#define lsb 0.0000763

// Inisialisasi objek LCD (alamat I2C 0x27 untuk LCD 16x2)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Inisialisasi objek ADS1115 (alamat I2C default adalah 0x48)
Adafruit_ADS1115 ads;

float getChannel0(){
  float rawA0;
  return rawA0 = ads.readADC_SingleEnded(0); 
}

float getChannel2(){
  float rawA2;
  return  rawA2 = ads.readADC_SingleEnded(2);
}

float getVoltFromRaw(){
  float rawAds0 = getChannel0();
  // Serial.println("ADS0: " + String(rawAds0));
  float realVolt1 = (rawAds0 * lsb) / PGA;
  // Serial.println("VOLT1: " + String(realVolt1));
  return realVolt1 * 10;
}

float getCurrentFromRaw(){
  float rawAds2 = getChannel2();
  float realVolt2 = (rawAds2 * lsb) / PGA;
  return (realVolt2 - acsOffset) / acsSens;
}

float getPower(){
  return getVoltFromRaw() * getCurrentFromRaw();
}

void setup() {
  // Mulai komunikasi I2C
  Wire.begin();
  
  // Inisialisasi LCD
  lcd.begin(16, 2);
  lcd.backlight();
  
  // Inisialisasi ADS1115
  if (!ads.begin()) {
    lcd.print("Tidak ada ADS");
    while (1);
  }

  // Set gain menjadi 1 (±4.096V)
  ads.setGain(GAIN_ONE);

  // Tampilkan pesan awal di LCD
  lcd.setCursor(0, 0);
  lcd.print("Mengambil data...");
}

void loop() {
  lcd.clear();  // Bersihkan layar LCD
  
  // Menampilkan nilai channel A0
  lcd.setCursor(0, 0);  // Baris pertama, kolom pertama
  lcd.print("V:");
  lcd.print(getVoltFromRaw());
  lcd.print("V");

  // Menampilkan nilai channel A2
  lcd.setCursor(9, 0);
  lcd.print("I:");
  lcd.print(getCurrentFromRaw());
  lcd.print("A");

  lcd.setCursor(0, 1);
  lcd.print("Power:");
  lcd.print(getPower());
  lcd.print("W");



  // Delay sebelum pembacaan berikutnya
  delay(1000);  // 1 detik
}
