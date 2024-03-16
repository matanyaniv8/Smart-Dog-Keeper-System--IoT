# Smart Dog Keeper
An ESP32 smart system aimed to help user control their pet's environment from a smartphone/ computer.

## Video
https://github.com/matanyaniv8/Smart-Dog-Keeper-System--IoT/assets/95882684/a825136d-6aa0-44b9-a64e-1c5d80164c80

## Features
This Arduino code is designed to create an interactive and dynamic smart dog keeper using an ESP32 controller and Blynk, make.com, and adafruit.io services.

### System Components:
The system uses a servo motor to turn the AC ON/OFF, temperature, and humidity sensor for measuring the room's temperature and humidity, and a water sensor for measuring the water level within a bowl. 

#### Our two scenarios controlled by the ESP32 are:
  1. The ESP32 monitors the room's temperature. We created an automation with the ESP32 such that when it senses that the gap between the room's temperature and outside temperature is more than 2 degrees, it activates the servo motor and turns on the AC. 
  Once the gap has minimized to less than 2 degrees, it activates the servo again and turns the AC OFF.
  2. The ESP32 checks and update the water level in the dog's water bowl and notifies the dog's owner whnever the water level becomes low.

#### Our two scenarios coming to the ESP32 from outside services are:
  1. The user city's outside temperature value coming from make.com weather services is being updated every 15 minutes within the make.com, which sends the value to the virtual pin of the outside temperature,
  that triggers an update off the outside temperature variable that ESP32 saves.
  This update can trigger the automation mentioned above in section 1. 
  2. The user pressed the ON/OFF button in the Blynk interface – the ESP32 turns ON/OFF accordingly using the servo motor. 

Also, we create a dashboard with Adafruit using MQTT requests to notify the user by email once the water level is low and the room's temperature is too high.
Adafruit also creates graphs and statistics on the above information so the user can track its dog water consumption and room temperature.

## Getting Started
We'ved created a simple and accessible guide for creating this system using Instructable.com, please [press here](https://www.instructables.com/Building-Your-Own-Smart-Dog-Keeper-System-With-ESP/) if you would like to try it yourself.

## License

MIT License.
