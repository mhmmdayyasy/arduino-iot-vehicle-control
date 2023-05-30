//Inisialisasi Library yang digunakan
#include <TinyGPS++.h>
#include <lorawan.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Inisialisasi pin
#define LED 26
#define INPUTLV 34
#define ENGPS 32
#define RELAY 25
#define MPU_ADDRESS 0x68

//Inisialisasi variabel tambahan
#define VOLTAGE 3.3 //MCU Voltage
#define ABIT  0.00024414 // 1/12 bit 
#define DEVIDER_RATIO 11 // 1.1M/100k Resistor
#define MULTIPLIER 1.2222 //Calibrator

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

//Declare variabel untuk parameter yang digunakan
String latitude, longitude, velo, m, d, y, hr, mn, sc;
String dataSend;
String GPSData, MPUData;
float inputlv, velocity;
float rotX, rotY, rotZ;
bool gpsvalid, datevalid, timevalid, locating = true;
bool confirminterval = false, motor, parking = true;
unsigned long T, t, Interval;
bool newmessage = false;
int vibration = 0, sample = 0, i = 0;
char sbuffer[50];

size_t EPROM_MEMORY_SIZE = 4096;

//Declare device address LoRa
const char *devAddr = "ff8e1836";
const char *nwkSKey = "f177a85edd0693130000000000000000";
const char *appSKey = "0000000000000000889218359ff062c9";

//Declare variabel untuk menerima pesan downlink
char myStr[50];
char outStr[255];
byte recvStatus = 0;

//PINOUT ESP32
const sRFM_pins RFM_pins = {
  .CS = 15,
  .RST = 0,
  .DIO0 = 27,
  .DIO1 = 2,
};

//Start Program
void setup() {
  Serial.begin(115200); //Start Serial
  delay(5000);
  
  Serial1.begin(9600, SERIAL_8N1, 16, 17);//Serial gps baud
  EEPROM.begin(EPROM_MEMORY_SIZE);
  EEPROM.get(0, sbuffer);
  Interval = String(sbuffer).toInt();
  if (Interval == NULL || Interval < 0)
  {
    Serial.println("[PGM] GET EEPROM Failed");
    Interval = 10;
    String sInterval = String(Interval);
    sInterval.toCharArray(sbuffer, sInterval.length() + 1);
    Serial.print("[INTVAL] "); Serial.print(Interval); 
    Serial.println(" s");
    EEPROM.put(0, sbuffer);
    EEPROM.commit();
    Serial.println("[PGM] Set EEPROM Success");
    Serial.println();
  }
  else
  {
    Serial.println("[PGM] GET EEPROM Success..");
    Serial.print("[INTVAL] "); 
    Serial.print(Interval); 
    Serial.println(" s");
    Serial.println();
  }

  //set PIN LED, ENGPS, RELAY sebagai output
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(ENGPS, OUTPUT);
  digitalWrite(ENGPS, HIGH); //turn voltage gps on
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  delay(500);

  //Start Modul MPU6050
  if (!mpu.begin(MPU_ADDRESS)) {
    Serial.println("[PGM] MPU 6050 Not Found..");
    Serial.println();
    delay(5000);
    return;
  }

  //Set sensitivitas Accelerometer ke level 16g
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.print("[PGM] Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  
  //Set sensitivitas Gyrometer ke level 250 degree
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("   [PGM] Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  //Set filter low pass digital ke pita 184 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  Serial.print("[PGM] Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");

  //Start LoRa RFM95
  if (!lora.init()) {
    Serial.println("[PGM] RFM95 Not Found..");
    Serial.println();
    delay(5000);
    return;
  }

  // Set LoRaWAN Class ke CLASS_C
  lora.setDeviceClass(CLASS_C);

  // Set Data Rate dengan Spreading Factor 12
  lora.setDataRate(SF12BW125);

  // set channel to random
  lora.setChannel(MULTI);

  // Set ABP Key and DevAddress
  lora.setNwkSKey(nwkSKey);
  lora.setAppSKey(appSKey);
  lora.setDevAddr(devAddr);
}

void loop() {
  while (Serial1.available() > 0)
  {
  //Start GPS
    if (gps.encode(Serial1.read()))
      GetGPSData();//Memanggil fungsi untuk menjalankan GPS
  }

  //Merubah Interval Uplink
  if (confirminterval == true)
  {
    Serial.print("[SEND");
    Serial.println(dataSend);
    dataSend.toCharArray(myStr, dataSend.length() + 1);
    lora.sendUplink(myStr, strlen(myStr), 0);
    Serial.println("[PGM] Send NEW Interval Confirmation");
    Serial.print("[INTVAL]"); 
    Serial.print(Interval); 
    Serial.println(" s");
    Serial.println();

    confirminterval = false;
  }

  //Melakukan pengukuran parameter getaran setiap 5 detik
  while(i < 50){
    GetMPUData();    
    i++;
    delay(100);
  }
  if (gpsvalid && datevalid && timevalid)
  {
    if(vibration > 28 || velocity > 5.5){
      //Uplink pada interval 6 detik
      if ((millis() - T) > (Interval * 1000))
      {
        motor = true;
        Serial.println();
        Serial.println("[PGM] Looping..");
        Serial.println();
        Serial.print("[LAT]"); Serial.println(latitude);
        Serial.print("[LONG]"); Serial.println(longitude);
        Serial.print("[VELO]"); Serial.println(velo);
        Serial.print("[VIBR]"); Serial.println(vibration);
        Serial.print("[SEND]");
        
        lora.setFramePortTx(1);//Set port lora ke port 1
    
        dataSend = "$," + GPSData + "," + String(motor) 
        + "," + String(parking) + ",#";
        Serial.println(dataSend);
        dataSend.toCharArray(myStr, dataSend.length() + 1);
        lora.sendUplink(myStr, strlen(myStr), 0); //Uplink
    
        gpsvalid = false;
        datevalid = false;
        timevalid = false;
        T = millis();
      }
    }
    else{
      if((millis()- t) > 3600000){//Uplink interval 3600 detik
        motor = false;
        Serial.println();
        Serial.println("[PGM] Looping..");
        Serial.println();
        Serial.print("[LAT]"); Serial.println(latitude);
        Serial.print("[LONG]"); Serial.println(longitude);
        Serial.print("[VELO]"); Serial.println(velo);
        Serial.print("[VIBR]"); Serial.println(vibration);
        Serial.print("[SEND]");
        
        lora.setFramePortTx(1);
    
        dataSend = "$," + GPSData + "," + String(motor) 
        + "," + String(parking) + ",#";
        Serial.println(dataSend);
        dataSend.toCharArray(myStr, dataSend.length() + 1);
        lora.sendUplink(myStr, strlen(myStr), 0);
    
        gpsvalid = false;
        datevalid = false;
        timevalid = false;
        t = millis();
      }
    }
  }
  else if (!gpsvalid || !datevalid || !timevalid)
  {
    if((millis()- t) > 20000){
      Serial.println();
    Serial.println("[GPS] CANNOT GET LOCATION...");
    Serial.print("[BATT]"); 
    Serial.print(inputlv, 2); 
    Serial.println(" V");
    Serial.print("[SEND]");
    
    lora.setFramePortTx(1);
    
    dataSend = "$,0,0,0," + String(digitalRead(RELAY)) + "," + String(motor) + "," + String(parking) + ",#";
    Serial.println(dataSend);
    dataSend.toCharArray(myStr, dataSend.length() + 1);
    lora.sendUplink(myStr, strlen(myStr), 0);
    t = millis();
    }
  }

  if(motor == true && parking == true)
  {
    lora.setFramePortTx(2); //Set port LoRa ke port 2
    
    delay(500);
    String dataSend = "tes"; //Sinyal trigger Notifikasi
    Serial.println(dataSend);
    dataSend.toCharArray(myStr, dataSend.length() + 1);
    lora.sendUplink(myStr, strlen(myStr), 0);
    delay(500);
    
    motor = false;
  }

  //Pembacaan Sinyal Downlink
  recvStatus = lora.readData(outStr);
  if (recvStatus) {
    Serial.println("[PGM] Get Downlink Message..");
    Serial.print("[MSG] "); Serial.println(outStr);
    Serial.println();
    String downlink;
    downlink = String(outStr);
    if (downlink.toInt() == 0) //Sinyal Relay OFF
    {
      digitalWrite(RELAY, LOW);
      Serial.println("[PGM] Receive Relay Command, Relay OFF");
    }
    else if (downlink.toInt() == 1) //Sinyal Relay ON
    {
      digitalWrite(RELAY, HIGH);
      Serial.println("[PGM] Receive Relay Command, Relay ON");
    }
    else if (downlink.toInt() == 2) //Sinyal Status Parkir OFF
    {
      parking = false;
    }
    else if (downlink.toInt() == 3) //Sinyal Status Parkir ON
    {
      parking = true;
    }
    else
    {
      Interval = 10;
      Serial.println("[PGM] Receive Set Interval not a number,"
      "Interval set to Default");
      Serial.print("[INTVAL] "); 
      Serial.print(Interval); 
      Serial.println(" s");
      String sInterval = String(Interval);
      sInterval.toCharArray(sbuffer, sInterval.length() + 1);
      EEPROM.put(0, sbuffer);
      EEPROM.commit();
      Serial.println("   [PGM] Set EEPROM Success");
      Serial.println();
      confirminterval = true;
    }

  }
  lora.update();
  i = 0;
}

//Fungsi untuk Menjalankan GPS
void GetGPSData()
{
  float vsampling = 0;
  for (int i = 0; i < 10; i++)
  {
    vsampling += analogRead(INPUTLV);
    delayMicroseconds(10);
  }
  vsampling = vsampling / 10;
  inputlv = vsampling * ABIT * VOLTAGE *DEVIDER_RATIO*MULTIPLIER;


  if (gps.location.isValid())
  {
    gpsvalid = true;
    latitude = String(gps.location.lat(), 6); //Membaca latitude
    longitude = String(gps.location.lng(), 6);//Membaca longitude
    velocity = (gps.speed.kmph());            //Membaca kecepatan
    velo = String(gps.speed.kmph());
  }
  else
  {
    gpsvalid = false;
    if (gpsvalid == false && locating == true)
    {
      Serial.println(F("[GPS] Locating..."));
      locating = false;
    }

  }
  if (gps.date.isValid())
  {
    datevalid = true;
    if (gps.date.month() < 10)
      m = "0" + String(gps.date.month());
    else
      m = String(gps.date.month());

    if (gps.date.day() < 10)
      d = "0" + String(gps.date.day());
    else
      d = String(gps.date.day());

    y = String(gps.date.year());
  }
  else
  {
    datevalid = false;
    Serial.println(F("   [GPS] Initializing Date..."));
  }
  if (gps.time.isValid())
  {
    timevalid = true;
    if ((gps.time.hour() + 7) < 10)
      hr = "0" + String(gps.time.hour() + 7);
    else
      hr = String(gps.time.hour() + 7);

    if (gps.time.minute() < 10)
      mn = "0" + String(gps.time.minute());
    else
      mn = String(gps.time.minute());

    if (gps.time.second() < 10)
      sc = "0" + String(gps.time.second());
    else
      sc = String(gps.time.second());
  }
  else
  {
    timevalid = false;
    Serial.println(F("   [GPS] Initializing Time..."));
  }

  if (gpsvalid && datevalid && timevalid)
  {
    //    GPSData =  latitude + "," + longitude + "," + velo + "," + String(inputlv, 2);
    GPSData =  latitude + "," + longitude + "," + velo + "," + String(digitalRead(RELAY));
  }
  else
    GPSData = GPSData;
  
}

//Fungsi menjalankan modul MPU6050
void GetMPUData()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  rotX = abs(float(g.gyro.x)); //Membaca rotasi sumbu x
  rotY = abs(float(g.gyro.y)); //Membaca rotasi sumbu y
  rotZ = abs(float(g.gyro.z)); //Membaca rotasi sumbu z

  if(rotX >= 0.15 || rotY >= 0.25){
     vibration++;
     Serial.print("getaran : ");
     Serial.println(vibration);
  }
  if(sample==50){
      sample = 0;
      vibration = 0;
  }
  sample++;
}
