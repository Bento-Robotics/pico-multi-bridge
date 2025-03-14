#include <Arduino.h>
#include <Servo.h>

#include "Adafruit_USBD_CDC.h"
#include "micro_rosso.h"

#include "ticker.h"
Ticker ticker;

#include "sync_time.h"
SyncTime sync_time;

#include "ros_status.h"
RosStatus ros_status;

#include "micro_rosso_2dof_arm.h"
Two_DOF_Arm two_DOF_arm;

Servo servo_top;
Servo servo_bottom;

void led_callback(int64_t last_call_time) {
  static bool status;
  digitalWrite(LED_BUILTIN, status);
  status = !status;
}

void setup1() {
  D_println("Booting...");

  delay(3000);

  // DS3240 limits:  min: 400 , max: 2600
  // Values constrained by mounting position
  servo_top.attach(14, 850, 2530);//, PRECISE_SERVO_270deg);
  servo_bottom.attach(15, 900, 2520);//, PRECISE_SERVO_270deg);

  D_print("Setting up transport... ");
  set_microros_serial_transports(SerialTinyUSB);
  //
  // while (true) {
  //   for (float i = 0; i <= 180; i += 0.1) {
  //     servo_bottom.write(i);
  //     delay(5);
  //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   }
  //   for (int i = 180; i >= 0; i--) {
  //     servo_bottom.write(i);
  //     delay(50);
  //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   }
  // }
  //
  if (!micro_rosso::setup("my_node_name"))
    D_println("FAIL micro_rosso.setup()");

  if (!ticker.setup())
    D_println("FAIL ticker.setup()");
  // ticker.timer_tick.callbacks.push_back(&led_callback);

  if (!sync_time.setup())
    D_println("FAIL sync_time.setup()");

  if (!ros_status.setup())
    D_println("FAIL ros_status.setup()");

  const uint32_t linkage_bottom_length_mm = 140;
  const uint32_t linkage_top_length_mm = 250;
  if (!two_DOF_arm.setup(&servo_top, &servo_bottom, linkage_bottom_length_mm, linkage_top_length_mm))
    D_println("FAIL two_DOF_arm.setup()");

  D_println("Boot completed.");
}

void loop1() { micro_rosso::loop(); }
