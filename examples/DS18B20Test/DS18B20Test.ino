/*
 * Sample of using unidimensional Kalman filter
 *
 * Copyright (c) 2018 Damian Wrobel <dwrobel@ertelnet.rybnik.pl>
 *
 * THIS MATERIAL IS PROVIDED AS IS, WITH ABSOLUTELY NO WARRANTY EXPRESSED
 * OR IMPLIED. ANY USE IS AT YOUR OWN RISK.
 *
 * Permission is hereby granted to use or copy this program
 * for any purpose, provided the above notices are retained on all copies.
 * Permission to modify the code and to distribute modified code is granted,
 * provided the above notices are retained, and a notice that the code was
 * modified is included with the above copyright notice.
 *
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <TrivialKalmanFilter.h>

#ifdef ARDUINO_ARCH_STM32
#define ONE_WIRE_BUS PB8        // The pin DS18B20 is connected to.
#else
#define ONE_WIRE_BUS A1         // The pin DS18B20 is connected to.
#endif

#define DT_COVARIANCE_RK 4.7e-3 // Estimation of the noise covariances (process)
#define DT_COVARIANCE_QK 1e-5   // Estimation of the noise covariances (observation)

OneWire onewire(ONE_WIRE_BUS);
DallasTemperature sensor(&onewire);
TrivialKalmanFilter<float> filter(DT_COVARIANCE_RK, DT_COVARIANCE_QK);

void setup(void) {
  Serial.begin(115200);

  sensor.begin();

  if (sensor.getDS18Count() == 0) {
    Serial.println("Could not find any DS18xx device.");
    while (true)
      ;
  }

  if (sensor.isParasitePowerMode()) {
    Serial.println("Parasite mode is not supported.");
    while (true)
      ;
  }

  sensor.setWaitForConversion(false);
}

void loop(void) {
  sensor.requestTemperatures();
  while (!sensor.isConversionComplete())
    ;

  for (auto i = 0; i < sensor.getDeviceCount(); i++) {
    DeviceAddress deviceAddress;

    if (sensor.getAddress(deviceAddress, i) && sensor.validFamily(deviceAddress)) {
      const auto temperature = sensor.getTempC(deviceAddress);
      Serial.print(temperature);
      Serial.print(" ");
      Serial.println(filter.update(temperature));
      return;
    }
  }

  Serial.println("Connection problem.");
  while (true)
    ;
}
