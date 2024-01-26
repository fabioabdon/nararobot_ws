#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>

#define REPORTING_PERIOD_MS 1000

#define TEMPERATURE_ADDR 0x5A

uint32_t tsLastReport = 0;

PulseOximeter pox;
Adafruit_MLX90614 mlx;

void onBeatDetected() {
  Serial.println("Batimento detectado");
}

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando funcionamento dos sensores");

  Wire.begin();

  if (!pox.begin()) {
    Serial.println("Falha ao iniciar o oxímetro");
    while (1);
  }

  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  mlx.begin(TEMPERATURE_ADDR);
}


void loop() {


  pox.update();


  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    Serial.print("Taxa de Batimento: ");
    //double bat = pox.getHeartRate();
    Serial.print(pox.getHeartRate());
    Serial.print(" bpm / Saturacao de Oxigenio (SpO2): ");
    Serial.print(pox.getSpO2());
    Serial.println("%");

    double temp_amb = mlx.readAmbientTempC();
    double temp_obj = mlx.readObjectTempC();

    Serial.print("Temperatura Ambiente: ");
    Serial.print(temp_amb);
    Serial.print(" °C / Temperatura do Corpo: ");
    Serial.print(temp_obj);
    Serial.println(" °C");

    tsLastReport = millis();
  }

  
}
