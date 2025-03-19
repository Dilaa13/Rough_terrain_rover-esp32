// Rover_sensors.ino

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <MPU6050.h>

Adafruit_BMP280 bmp;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    
    // Initialize BMP280
    if (!bmp.begin(0x76)) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }

    // Initialize SHT31
    if (!sht31.begin(0x44)) {
        Serial.println("Could not find a valid SHT31 sensor, check wiring!");
        while (1);
    }

    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    Serial.println("All sensors initialized successfully.");
}

void loop() {
    // Read BMP280 data
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F;  // Convert to hPa
    Serial.print("BMP280 Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    // Read SHT31 data
    float shtTemp = sht31.readTemperature();
    float humidity = sht31.readHumidity();
    Serial.print("SHT31 Temperature: ");
    Serial.print(shtTemp);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    // Read MPU6050 data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("MPU6050 Accel: ");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.print(az);
    Serial.print(" | Gyro: ");
    Serial.print(gx);
    Serial.print(", ");
    Serial.print(gy);
    Serial.print(", ");
    Serial.println(gz);

    delay(2000); // Wait 2 seconds before next reading
}
