#include <Arduino.h>

#include <Adafruit_ADS1X15.h>
#include <ODriveArduino.h>

// Printing with stream operator
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}

// Serial to the ODrive
HardwareSerial odrive_serial(2);

// ODrive object
ODriveArduino odrive(odrive_serial);

// Velocity
float velocity = 0.0;

// Position sensor
Adafruit_ADS1115 ads;

float positionVoltage1 = 0;
float positionVoltage2 = 0;
float positionVoltage3 = 0;

float mappedVelocity = 0;

void setup()
{
  // ADS
  if (!ads.begin())
  {
    Serial.println("Error initializing ADS");
    return;
  }

  // ODrive uses 115200 baud
  odrive_serial.begin(115200, SERIAL_8N1, 16, 17);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial)
    ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis)
  {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop()
{

  if (Serial.available())
  {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0' || c == '1')
    {
      int motornum = c - '0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, false); // don't wait
    }

    if (c == '+')
    {
      velocity += 10;
      Serial.print("Setting velocity ");
      Serial.println(velocity);
      odrive.SetVelocity(0, velocity);
    }

    if (c == '-')
    {
      velocity -= 10;
      Serial.print("Setting velocity ");
      Serial.println(velocity);
      odrive.SetVelocity(0, velocity);
    }

    // Read bus voltage
    if (c == 'b')
    {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p')
    {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while (millis() - start < duration)
      {
        for (int motor = 0; motor < 2; ++motor)
        {
          odrive_serial << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }
  }
  positionVoltage1 = ads.readADC_Differential_0_1();
  positionVoltage2 = ads.readADC_Differential_0_3();

  Serial.print("S0: ");
  Serial.println(positionVoltage1 * 0.1875F);
  Serial.print("S1: ");
  Serial.println(positionVoltage2 * 0.1875F);

  mappedVelocity = map(positionVoltage1 * 0.1875F, 0, 5000, 0, 50);

  Serial.print("Mapped: ");
  Serial.println(mappedVelocity);

  odrive.SetVelocity(0, mappedVelocity);
}