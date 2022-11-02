/*

  GenesGreenMachine.com

  This is the code I cobbled together to add Zwift/FMTS BLE to a pedal generator using an Arduino Nano 33 IoT (among others) 
  along with a DPSxxxx charge controller to provide the actual resistance while charging batteries with a pedal generator

  ArduinoBLE and ModbusMaster are used together to make it all happen.

  Training mode works amazingly well, SIM mode has some room for improvement.  
  
  By default, SIM mode is disabled by setting OVERRIDE = true which lets you manually control the resistance by changing 
  AMPS on the DPS

*/

#include <ArduinoBLE.h>
#include <ModbusMaster.h>

boolean debugging = false; 

// The Fitness Machine Control Point data type structure 

#define FMCP_DATA_SIZE 19 // Control point consists of 1 opcode (byte) and maximum 18 bytes as parameters

// This fmcp_data_t structure represents the control point data. The first octet represents the opcode of the request
// followed by a parameter array of maximum 18 octects
typedef struct __attribute__( ( packed ) )
{
  uint8_t OPCODE;
  uint8_t OCTETS[ FMCP_DATA_SIZE-1 ];
} fmcp_data_t;

typedef union // The union type automatically maps the bytes member array to the fmcp_data_t structure member values
{
  fmcp_data_t values;
  uint8_t bytes[ FMCP_DATA_SIZE ];
} fmcp_data_ut;

fmcp_data_ut fmcpData;
short fmcpValueLength;


BLEService fitnessMachineService("1826"); 
BLECharacteristic fitnessMachineFeatureCharacteristic("2ACC", BLERead, 8);                                  // Fitness Machine Feature, mandatory, read
BLECharacteristic indoorBikeDataCharacteristic("2AD2", BLENotify, 8);                                       // Indoor Bike Data, optional, notify
BLECharacteristic trainingStatusCharacteristic("2AD3", BLENotify | BLERead, 20);                            // Training Status, optional, read & notify
BLECharacteristic supportedResistanceLevelRangeCharacteristic("2AD6", BLERead, 4);                          // Supported Resistance Level, read, optional
BLECharacteristic fitnessMachineControlPointCharacteristic("2AD9", BLEWrite | BLEIndicate, FMCP_DATA_SIZE); // Fitness Machine Control Point, optional, write & indicate
BLECharacteristic fitnessMachineStatusCharacteristic("2ADA", BLENotify, 2);                                 // Fitness Machine Status, mandatory, notify

// Buffers used to write to the characteristics and initial values
unsigned char ftmsFeatureBuf[4] = { 0b00000011, 0b01000000, 0, 0 }; //, 0, 0, 0, 0};                        // Features: 0 (Avg speed), 1 (Cadence), 2 (Total distance), 7 (Resistance level), 10 (Heart rate measurement), 14 (Power measurement)
unsigned char indoorBikeDataBuf[8]  = { 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char ftmsControlPointBuf[20];

ModbusMaster node;    // instantiate ModbusMaster object

// Indoor Bike Data characteristic variables

const uint16_t flagMoreData = 1;
const uint16_t flagAverageSpeed = 2;
const uint16_t flagInstantaneousCadence = 4;
const uint16_t flagAverageCadence = 8;
const uint16_t flagTotalDistance = 16;
const uint16_t flagResistanceLevel = 32;
const uint16_t flagIntantaneousPower = 64;
const uint16_t flagAveragePower = 128;
const uint16_t flagExpendedEnergy = 256;
const uint16_t flagHeartRate = 512;
const uint16_t flagMetabolicEquivalent = 1024;
const uint16_t flagElapsedTime = 2048;
const uint16_t flagRemainingTime = 4096;

// Fitness Machine Control Point opcodes 

const uint8_t fmcpRequestControl = 0x00;
const uint8_t fmcpReset = 0x01;
const uint8_t fmcpSetTargetSpeed = 0x02;
const uint8_t fmcpSetTargetInclination = 0x03;
const uint8_t fmcpSetTargetResistanceLevel = 0x04;
const uint8_t fmcpSetTargetPower = 0x05;
const uint8_t fmcpSetTargetHeartRate = 0x06;
const uint8_t fmcpStartOrResume = 0x07;
const uint8_t fmcpStopOrPause = 0x08;
const uint8_t fmcpSetTargetedExpendedEngery = 0x09;
const uint8_t fmcpSetTargetedNumberOfSteps = 0x0A;
const uint8_t fmcpSetTargetedNumberOfStrided = 0x0B;
const uint8_t fmcpSetTargetedDistance = 0x0C;
const uint8_t fmcpSetTargetedTrainingTime = 0x0D;
const uint8_t fmcpSetTargetedTimeInTwoHeartRateZones = 0x0E;
const uint8_t fmcpSetTargetedTimeInThreeHeartRateZones = 0x0F;
const uint8_t fmcpSetTargetedTimeInFiveHeartRateZones = 0x10;
const uint8_t fmcpSetIndoorBikeSimulationParameters = 0x11;
const uint8_t fmcpSetWheelCircumference = 0x12;
const uint8_t fmcpSetSpinDownControl = 0x13;
const uint8_t fmcpSetTargetedCadence = 0x14;
const uint8_t fmcpResponseCode = 0x80;

BLEDevice central;

// *** change these variables to match your fitness/environment ***

float ftpWatts = 130;       // base FTP to use for sim calculations  
float losses = 1.10;         // losses multiplier, account for losses in drive train, rectifier, charge controller, etc

// Data for the resistance calculation in sim mode

float wind_speed = 0;       // meters per second, resolution 0.001
float grade = 0;            // percentage, resolution 0.01
float crr = 0;              // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;               // Wind resistance Kg/m, resolution 0.01;

// power variables

unsigned int instantaneous_cadence = 0;     // Global variable to hold the calculated cadence in RPM
unsigned int instantaneous_power = 0;       // Global variable to hold the calculated power in watts
unsigned int instantaneous_speed = 0;       // Global variable to hold the calculated speed in m/s

float Watts = 0.0;
float Amps = 0.0;
float Volts = 14.6;  
uint8_t dpsResult;  // for MODBUS node read
unsigned int setWatts = 50;
boolean OVERRIDE = true; // change to false to enable SIM mode
unsigned long WRITEDELAY = 750;  // 750 seems to work with Nano IoT 33 and DPS5020, YMMV
unsigned long lastWrite = 0;
unsigned long startTime;
unsigned long currentTime;

// Set the correct resistance level on the physical trainer
int setTrainerResistance(float wind_speed, float grade, float crr, float cw) {
  
  //    This is a really dumb way to get basic resistance changes to work in FTMS sim mode, good enough until a better calculation can be determined
  return  (int)(ftpWatts * (grade/10)) + ftpWatts;
  
}

void setup() {
  if (debugging) {
    Serial.begin(9600);
    Serial.println("Setting up serial comm with DPS, bluetooth device and BLE Fitness Machine Service");
  }
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  // start serial connection to DPSXXXX, wired to TX to RX, RX to TX, GND to GND and 3.3v to 3.3v of Arduino NANO 33 IoT
  Serial1.begin(9600);  
  // communicate with Modbus slave ID 1 over Serial
  node.begin(1, Serial1);

  if (!BLE.begin()) { // Error starting the bluetooth module
    if (debugging) {
      Serial.println("Error initiliazing bluetooth module, please restart");
    }
    while (1);
  }

  BLE.setDeviceName("GGM Pedal Generator");
  BLE.setLocalName("GGMPedGen");
  BLE.setAdvertisedService(fitnessMachineService);
  fitnessMachineService.addCharacteristic(fitnessMachineFeatureCharacteristic);
  fitnessMachineService.addCharacteristic(indoorBikeDataCharacteristic);
  fitnessMachineService.addCharacteristic(fitnessMachineControlPointCharacteristic);
  BLE.addService(fitnessMachineService);

  fitnessMachineFeatureCharacteristic.writeValue(ftmsFeatureBuf, 4);
  indoorBikeDataCharacteristic.writeValue(indoorBikeDataBuf, 8);

  // start advertising
  BLE.advertise();
  if (debugging) {
    Serial.println("BLE advertisement started");
  }

  if (debugging) {
    Serial.println("Arduino Smart Pedal Generator started!");
  }

}

void loop() {
  BLE.poll();

  central = BLE.central();
  if (central) {
    
     digitalWrite(LED_BUILTIN, HIGH);
     
     if (debugging) {
            Serial.println("Connected to central: ");
            Serial.print("Address: ");
            Serial.println(central.address());
     }
     while (central.connected()) {
      if (fitnessMachineControlPointCharacteristic.written()) {
          fmcpValueLength = fitnessMachineControlPointCharacteristic.valueLength();
          memset(fmcpData.bytes, 0, sizeof(fmcpData.bytes));
          fitnessMachineControlPointCharacteristic.readValue(fmcpData.bytes, fmcpValueLength);
          if (debugging) {
            Serial.println("Control point received");
            Serial.print("OpCode: ");
            Serial.println(fmcpData.values.OPCODE, HEX);
            Serial.println("Values: ");
            for (int i=0; i<fmcpValueLength-1; i++) Serial.println(fmcpData.values.OCTETS[i], HEX);
            Serial.println();
          }
          ftmsControlPointBuf[0] = fmcpResponseCode;
          ftmsControlPointBuf[1] = fmcpData.values.OPCODE;
          ftmsControlPointBuf[2] =  0x02; // OpCode not supported for now
          switch(fmcpData.values.OPCODE) {
            case fmcpRequestControl: {
              // Always allow control
              if (debugging) {
                Serial.println("Request Control");
              }
              ftmsControlPointBuf[2] =  0x01; // OpCode supported
              break;
            }
            case fmcpStartOrResume: {
              if (debugging) {
                Serial.println("Start the training");
              }
              startTime = millis();
              node.writeSingleRegister(9, 1); //set DPS power on !
              ftmsControlPointBuf[2] =  0x01; // OpCode supported 
              break;
            }
            case fmcpStopOrPause: {
              if (debugging) {
                Serial.println("Stop or pause training");
              }
              break;
            }
            case fmcpSetIndoorBikeSimulationParameters: {
              if (debugging) {
                Serial.println("Indoor bike simulation parameters request");
              }
              int16_t ws = (int16_t)((fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0]); // Short is 16 bit signed, so the windspeed is converted from two bytes to signed value. Highest bit is sign bit
              wind_speed = ws / 1000.0;
              int16_t gr = (int16_t)((fmcpData.values.OCTETS[3] << 8) + fmcpData.values.OCTETS[2]); // Short is 16 bit signed, so a negative grade is correctly converted from two bytes to signed value. Highest bit is sign bit
              grade = gr / 100.0;
              crr = fmcpData.values.OCTETS[4] / 10000.0;
              cw = fmcpData.values.OCTETS[5] / 100.0;
              if (debugging) { // Remember, if debuggingging with Zwift, that these values are divided by 2 if in normal settings!
                Serial.print("Wind speed (1000): "); Serial.println(wind_speed);  // seems to always be zero in Zwift
                Serial.print("Grade (100): "); Serial.println(grade);
                Serial.print("Crr (10000): "); Serial.println(crr);   // seems to always be 0.00 in Zwift
                Serial.print("Cw (100): "); Serial.println(cw);       // seems to always be 0.51 in Zwift
              }

              // check override flag and skip SIM calculation and wattage setting if over ride is on
              if (OVERRIDE) {
                if (debugging) {
                  Serial.println("OVERRIDE flag is on, not using SIM settings");
                }                          
              }
              else
              {
                setWatts = setTrainerResistance(wind_speed, grade, crr, cw);
                Amps = (float)((setWatts/losses)/Volts);
                 if ((millis() - lastWrite) <= WRITEDELAY) {
                  delay(millis() - lastWrite); 
                }                
                node.writeSingleRegister(1, round(Amps*100)); // 500 = 5 amps  
                lastWrite = millis();
                if (debugging) {
                  Serial.print("Set sim target power request: ");
                  Serial.println(setWatts);
                  Serial.print("Set DPSxxxx Amps to: ");
                  Serial.println(Amps);
                }                          
              }
              
              ftmsControlPointBuf[2] =  0x01;
              break;
              }
            case fmcpReset: {
              if (debugging) {
                Serial.println("Reset request");
              }
              ftmsControlPointBuf[2] =  0x01;
              break;
            }
            case fmcpSetTargetResistanceLevel: {
              if (debugging) {
                Serial.println("Set target resistance level");
              }
              short resistance = (fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0];
              if ((millis() - lastWrite) <= WRITEDELAY) {
                delay(millis() - lastWrite); 
              }              
              node.writeSingleRegister(1, round(resistance)); // 500 = 5 amps  
              lastWrite = millis();    
              ftmsControlPointBuf[2] =  0x01;
              break;
            }
            case fmcpSetTargetPower: {
              if (debugging) {
                Serial.println("Set target power");
              }
              setWatts = (fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0];
              Amps = (float)((setWatts/losses)/Volts);

              // writing too frequently fails, adding a delay
              if ((millis() - lastWrite) <= WRITEDELAY) {
                delay(millis() - lastWrite); 
              }  
              node.writeSingleRegister(1, round(Amps*100)); // 500 = 5 amps  
              lastWrite = millis();
              if (debugging) {
                Serial.print("Set target power request: ");
                Serial.println(setWatts);
                Serial.print("Set DPSxxxx Amps to: ");
                Serial.println(Amps);
              }    
              ftmsControlPointBuf[2] =  0x01;
              break;
            }
          }
          fitnessMachineControlPointCharacteristic.writeValue(ftmsControlPointBuf, 3);
        }
        indoorBikeDataBuf[0] = 0x00 | flagInstantaneousCadence | flagIntantaneousPower; // More Data = 0 (instantaneous speed present), bit 2: instantaneous cadence present
        indoorBikeDataBuf[1] = 0;
      
        int s = 7;  // TODO: actually calculate this -  instantaneous speed is m/s. IndoorBikeData needs km/h in resolution of 0.01
        indoorBikeDataBuf[2] = s & 0xFF; // Instantaneous Speed, uint16
        indoorBikeDataBuf[3] = (s >> 8) & 0xFF;
        
        int instantaneous_cadence = 85;  // TODO: actually determine the cadence
        indoorBikeDataBuf[4] = (int)round(instantaneous_cadence) & 0xFF; // Instantaneous Cadence, uint16
        indoorBikeDataBuf[5] = ((int)round(instantaneous_cadence) >> 8) & 0xFF;
      
        instantaneous_power = 0;
        dpsResult = node.readHoldingRegisters(2, 2);  // slave: read a range of 16-bit registers starting at register 2 to 3 (measured voltage and current)
      
      
        if (dpsResult == node.ku8MBSuccess)   // only do something with data if read is successful
        {        
          Volts =  ((float)node.getResponseBuffer(0) / 100 ); // get voltage from response buffer and convert to float
          Amps =  ((float)node.getResponseBuffer(1) / 100 ); // get current from response buffer and convert to float
        
          instantaneous_power = Volts*Amps;    /* random(90,180); analogRead(A0); */
          if (instantaneous_power < 0)
          {
            instantaneous_power = 0.0;
          }
          instantaneous_power = round(((float)instantaneous_power)*losses); // account for losses in bridge rectifier and charge controller
       
        
          indoorBikeDataBuf[6] = (int)round(instantaneous_power) & 0xFF; // Instantaneous Power, uint16
          indoorBikeDataBuf[7] = ((int)round(instantaneous_power) >> 8) & 0xFF;
          indoorBikeDataCharacteristic.writeValue(indoorBikeDataBuf, 8);
      
          if (debugging) {
            Serial.print("Power: ");
            Serial.println(instantaneous_power);
            Serial.println("Indoor Bike Data written");
          }

          // if the watts are off from what was set previously, adjust to match
          currentTime = millis();
          if (setWatts != instantaneous_power) {
              // wait 60 seconds after starting, then if user manually changed DPSxxxx to 20 watts or more higher (ie: cranked up the amps by hand), turn on override so they can hammer
              if (((instantaneous_power - 20) > setWatts) and ((currentTime - startTime) >= 60000)) {
                OVERRIDE = true;
              }
              if (!OVERRIDE) {
                Amps = (float)((setWatts/losses)/Volts);

                if ((millis() - lastWrite) <= WRITEDELAY) {
                  delay(millis() - lastWrite); 
                }
                node.writeSingleRegister(1, round(Amps*100)); // 500 = 5 amps  
                lastWrite = millis();
                if (debugging) {
                  Serial.print("Watts out of sync, adjusting amps to: ");
                  Serial.println(Amps);
                }   
              }
          }
        }
        // delay(500);
      }
      
      digitalWrite(LED_BUILTIN, LOW);
      
      if (debugging) {
          Serial.println("Disconnected from central... ");
          Serial.print("Address: ");
          Serial.println(central.address());
      }
  }
}
