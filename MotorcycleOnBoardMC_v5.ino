// Credits
// ra6070    for https://github.com/ra6070/BLE-TPMS/
// redyellow for https://forum.arduino.cc/t/simultaneous-central-peripheral-roles-on-one-device/651356/8 
// polldo    for https://github.com/arduino-libraries/ArduinoBLE/pull/183

// Include libraries
#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>
#include <MaximWire.h>
#include <OneButton.h>

// Constants declaration
const String        deviceNameFinal         = "McGyver's NC750XAJ"; // Set device name
const unsigned long temperatureInterval     = 5000;                 // How often (ms) to update the temperature reading
const int           intTemperatureOffset    = 4;                    // Onboard temperature sensor offset
const int           extTemperatureSensorPin = 2;                    // Pin where DS18B20 temperature sensor is connected
const unsigned long voltageInterval         = 1000;                 // How often (ms) to update the voltage reading
const int           voltageSensorPin        = A0;                   // Pin to read external voltage from
const float         R1                      = 100000.0;             // Resistance of R1 (100kΩ)
const float         R2                      = 10000.0;              // Resistance of R2 (10kΩ)
const int           button1Pin              = 4;                    // Pin where button 1 is connected
const int           button2Pin              = 5;                    // Pin where button 2 is connected
const unsigned long tireInterval            = 5000;                 // How often (ms) to switch scanning on/off

// Variables initialization
bool  internalTempSensor = true;
bool  tpmsScanning       = false;
int   temperature        = 0;
float voltage            = 0.0;

unsigned long previousTemperatureMillis = 0;
unsigned long previousVoltageMillis     = 0;
unsigned long previousTireMillis        = 0;
unsigned long currentMillis             = 0;

byte manufacturerData[2] = { highByte(temperature), lowByte(temperature) };

// DS18B20 temperature sensor setup
MaximWire::Bus oneWireBus(extTemperatureSensorPin);
MaximWire::DS18B20 oneWireDevice;

// OneButton buttons setup
OneButton button1(button1Pin, false, false);
OneButton button2(button2Pin, false, false);

// BLE services declaration
BLEService motorcycleService("8f735304-e5f1-465f-8f33-071d1b493050");

// BLE characteristics declaration
BLEShortCharacteristic  temperatureCharacteristic  ("8f735304-e5f1-465f-8f33-071d1b493051", BLERead | BLENotify);
BLEFloatCharacteristic  voltageCharacteristic      ("8f735304-e5f1-465f-8f33-071d1b493052", BLERead | BLENotify);
BLECharacteristic       buttonCharacteristic       ("8f735304-e5f1-465f-8f33-071d1b493053", BLERead | BLENotify, "XX");

BLEShortCharacteristic  frontTirePresCharacteristic("8f735304-e5f1-465f-8f33-071d1b493061", BLERead | BLENotify);
BLEShortCharacteristic  frontTireTempCharacteristic("8f735304-e5f1-465f-8f33-071d1b493062", BLERead | BLENotify);
BLEShortCharacteristic  frontTireBattCharacteristic("8f735304-e5f1-465f-8f33-071d1b493063", BLERead | BLENotify);

BLEShortCharacteristic  rearTirePresCharacteristic ("8f735304-e5f1-465f-8f33-071d1b493071", BLERead | BLENotify);
BLEShortCharacteristic  rearTireTempCharacteristic ("8f735304-e5f1-465f-8f33-071d1b493072", BLERead | BLENotify);
BLEShortCharacteristic  rearTireBattCharacteristic ("8f735304-e5f1-465f-8f33-071d1b493073", BLERead | BLENotify);

void setup() {
  // Start serial communication
  Serial.begin(9600);
  // Uncomment next line to wait for serial connection before continuing (debugging)
  // while (!Serial);

  // Button 1 functions
  button1.attachClick(button1Click);
  button1.attachDoubleClick(button1DoubleClick);
  button1.attachLongPressStop(button1LongClick);

  // Button 2 functions
  button2.attachClick(button2Click);
  button2.attachDoubleClick(button2DoubleClick);
  button2.attachLongPressStop(button2LongClick);

  // Initialize bluetooth
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE module!");
    while (1);
  }

  // BLE event functions
  BLE.setEventHandler(BLEConnected,    blePeripheralConnect);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnect);
  BLE.setEventHandler(BLEDiscovered,   bleCentralDiscover);

  // Set manufacturer data
  BLE.setManufacturerData(manufacturerData, 2);
  
  // Set device name
  BLE.setLocalName (deviceNameFinal.c_str());
  BLE.setDeviceName(deviceNameFinal.c_str());

  // Add characteristics to services
  motorcycleService.addCharacteristic(temperatureCharacteristic);
  motorcycleService.addCharacteristic(voltageCharacteristic);
  motorcycleService.addCharacteristic(buttonCharacteristic);

  motorcycleService.addCharacteristic(frontTirePresCharacteristic);
  motorcycleService.addCharacteristic(frontTireTempCharacteristic);
  motorcycleService.addCharacteristic(frontTireBattCharacteristic);

  motorcycleService.addCharacteristic(rearTirePresCharacteristic);
  motorcycleService.addCharacteristic(rearTireTempCharacteristic);
  motorcycleService.addCharacteristic(rearTireBattCharacteristic);
  
  // Add the BLE services
  BLE.addService(motorcycleService);

  // Set the advertising service
  BLE.setAdvertisedService(motorcycleService);

  // Set the connection interval
  BLE.setConnectionInterval(6, 10);
  
  // Start advertising
  BLE.advertise();

  // Initialize internal temperature sensor
  if (!HTS.begin()) {
    Serial.println("Failed to initialize internal temperature sensor!");
    internalTempSensor = false;
  }

  // Initialize external temperature sensor
  if (oneWireBus.Discover().FindNextDevice(oneWireDevice) && oneWireDevice.GetModelCode() == MaximWire::DS18B20::MODEL_CODE) {
    oneWireDevice.Update(oneWireBus);
  } else {
    Serial.println("Failed to initialize external temperature sensor!");
    oneWireDevice.Reset();
  }

  // Initialize pins as input
  pinMode(voltageSensorPin, INPUT);

  Serial.println("Waiting for connections...");
}

void loop() {
  // Used for different time intervals for probing sensors
  // Overflows every 49 days
  currentMillis = millis();

  // Poll for BLE events
  BLE.poll();

  // Buttons monitoring
  button1.tick();
  button2.tick();

  // Read temperature sensors
  if (currentMillis - previousTemperatureMillis > temperatureInterval) {
    previousTemperatureMillis = currentMillis;
    
    if (oneWireDevice.IsValid() && !isnan(oneWireDevice.GetTemperature<float>(oneWireBus))) {
      // Read temperature from external sensor, round it, offset it and constrain it
      temperature = round(oneWireDevice.GetTemperature<float>(oneWireBus));
      temperature = constrain(temperature, -99, 99);
  
      Serial.print("External temperature: ");
      oneWireDevice.Update(oneWireBus);
    } else {
      // Try to initialize again external temperature sensor (case of temporary disconnection)
      if (oneWireBus.Discover().FindNextDevice(oneWireDevice) && oneWireDevice.GetModelCode() == MaximWire::DS18B20::MODEL_CODE) {
        oneWireDevice.Update(oneWireBus);
      } else if (internalTempSensor == true) {
        // Read temperature from internal sensor, round it, offset it and constrain it
        temperature = round(HTS.readTemperature()) - intTemperatureOffset;
        temperature = constrain(temperature, -99, 99);
    
        Serial.print("Internal temperature: ");
      }
    }
    Serial.println(temperature);
    temperatureCharacteristic.writeValue(temperature);
    
    // Put temperature to manufacturer data
    manufacturerData[0] = highByte(temperature);
    manufacturerData[1] = lowByte(temperature);
    // Stop advertising, update manufacturer data and advertise again
    BLE.stopAdvertise();
    BLE.setManufacturerData(manufacturerData, 2);
    BLE.advertise();
  }

  // Read voltage
  if (currentMillis - previousVoltageMillis > voltageInterval) {
    previousVoltageMillis = currentMillis;

    voltage = round(
      (
        (
          (analogRead(voltageSensorPin) * 3.3) / 1023.0
        ) / ( R2 / ( R1 + R2 ) )
      ) * 10
    ) / 10;

    Serial.print("Voltage: ");
    Serial.println(voltage);
    voltageCharacteristic.writeValue(voltage);
  }

  // Read TPMS sensors
  if (currentMillis - previousTireMillis > tireInterval && BLE.connected()) {
    previousTireMillis = currentMillis;

    // Switch BLE scanning on/off
    if (tpmsScanning == false) {
      tpmsScanning = true;
      Serial.println("Start BLE scan");
      BLE.scanForUuid("fbb0", true);
    } else {
      tpmsScanning = false;
      Serial.println("Stop BLE scan");
      BLE.stopScan();
    }
  }
}

// This function will be called when button 1 is pressed once (single click)
void button1Click() {
  Serial.println("Button 1 click.");
  buttonCharacteristic.writeValue("1C");
}

// This function will be called when button 1 is pressed twice (double click)
void button1DoubleClick() {
  Serial.println("Button 1 double click.");
  buttonCharacteristic.writeValue("1D");
}

// This function will be called when button 1 is released after a long press (long click)
void button1LongClick() {
  Serial.println("Button 1 long click.");
  buttonCharacteristic.writeValue("1L");
}

// This function will be called when  button 2 is pressed once (single click)
void button2Click() {
  Serial.println("Button 2 click.");
  buttonCharacteristic.writeValue("2C");
}

// This function will be called when button 2 is pressed twice (double click)
void button2DoubleClick() {
  Serial.println("Button 2 double click.");
  buttonCharacteristic.writeValue("2D");
}

// This function will be called when button 2 is released after a long press (long click)
void button2LongClick() {
  Serial.println("Button 2 long click.");
  buttonCharacteristic.writeValue("2L");
}

// This function will be called when a connection with a central is established
void blePeripheralConnect(BLEDevice central) {
  Serial.print("Connected to ");
  Serial.println(central.address());
  BLE.setConnectable(false);
  previousTireMillis = currentMillis;
}

// This function will be called when the connection with the central is terminated
void blePeripheralDisconnect(BLEDevice central) {
  Serial.print("Disconnected from ");
  Serial.println(central.address());
  BLE.setConnectable(true);
  BLE.stopScan();
}

// This function will be called whenever an advertisement is captured
void bleCentralDiscover(BLEDevice peripheral) {
  if (peripheral.hasManufacturerData()) {
    // Initialize local buffer to hold manufacturer data
    byte buffer[peripheral.manufacturerDataLength()];
    
    // Read manufacturer data in buffer
    peripheral.manufacturerData(buffer, peripheral.manufacturerDataLength());        
    
    // Get pressure
    int tirePressure = buffer[8]|buffer[9]<<8|buffer[10]<<16|buffer[11]<<24;
    // Convert to PSI and constraint
    tirePressure = constrain((tirePressure / 1000.0) * 0.145, 0, 99);
    Serial.print("Pressure: ");
    Serial.println(tirePressure);
    
    // Get temperature
    int tireTemperature = buffer[12]|buffer[13]<<8|buffer[14]<<16|buffer[15]<<24;
    // Convert and constraint
    tireTemperature = constrain(tireTemperature / 100.0, -99, 99);
    Serial.print("Temperature: ");
    Serial.println(tireTemperature);      
    
    // Get battery percentage
    int tireBattery = buffer[16];
    Serial.print("Battery: ");
    Serial.println(tireBattery);

    // Write characteristics based on MAC address
    // Front tire
    if (peripheral.address() == "81:ea:ca:10:34:ae" || peripheral.address() == "81:ea:ca:30:06:d5") {
      frontTirePresCharacteristic.writeValue(tirePressure);
      frontTireTempCharacteristic.writeValue(tireTemperature);
      frontTireBattCharacteristic.writeValue(tireBattery);
    }
    // Rear tire
    if (peripheral.address() == "81:ea:ca:20:3e:fc" || peripheral.address() == "83:ea:ca:40:09:29") {
      rearTirePresCharacteristic.writeValue(tirePressure);
      rearTireTempCharacteristic.writeValue(tireTemperature);
      rearTireBattCharacteristic.writeValue(tireBattery);
    }
  }  
}
