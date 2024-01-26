  #include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <MLX90614.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

#include <std_msgs/Float32.h>

#define REPORTING_PERIOD_MS 1000

#define TEMPERATURE_ADDR 0x5A

double bat;

uint32_t tsLastReport = 0;

PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//ros
ros::NodeHandle  nh;

std_msgs::UInt8 bat_msg;
ros::Publisher bat_card("bat_card", &bat_msg);


std_msgs::UInt8 spo2_msg;
ros::Publisher spo2("spo2", &spo2_msg);

void onBeatDetected() {
}

void setup() {

    nh.getHardware()->setBaud(57600);


  Wire.begin();

  if (!pox.begin()) {
    while (1);
  }

  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  nh.initNode();
  nh.advertise(bat_card);
  nh.advertise(spo2);
  
}

  //ros


void loop() {


  pox.update();  
  bat_msg.data = pox.getHeartRate();
  spo2_msg.data = pox.getSpO2();


  bat_card.publish( &bat_msg );
  spo2.publish( &spo2_msg );
  

  nh.spinOnce(); 
}
