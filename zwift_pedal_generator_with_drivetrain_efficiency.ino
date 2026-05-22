/**
 * Zwift BLE Pedal Generator — DPS5020 TTL UART Edition
 * ======================================================
 * Turns a bicycle + DC generator + DPS5020 programmable power supply into a
 * Zwift-compatible smart trainer.  Resistance is controlled by commanding the
 * DPS5020's current limit over Modbus RTU.  The harder Zwift asks you to work,
 * the more current the DPS draws from the generator, increasing pedal load.
 *
 * The DPS5020 exposes a 3.3 V TTL UART header (TX, RX, GND, 3.3V) — there
 * is NO RS-485 bus and NO direction-control (DE/RE) pin needed.  Connect
 * directly to Arduino Serial1.
 *
 * BLE services exposed to Zwift
 *   • Cycling Power Service  (CPS)  0x1818 — reports watts & cadence
 *   • Fitness Machine Service(FTMS) 0x1826 — receives ERG / resistance targets
 *
 * Wiring
 *   DPS5020 TX  → Arduino Serial1 RX (pin 0 on Nano 33 BLE)
 *   DPS5020 RX  → Arduino Serial1 TX (pin 1 on Nano 33 BLE)
 *   DPS5020 GND → Arduino GND
 *   DPS5020 3.3V → Arduino 3.3V  (optional; powers the UART logic)
 *   Hall sensor OUT → Arduino pin 2  (with 10 kΩ pull-up to 3.3 V)
 *   Magnet on crank arm
 *
 * Libraries required (install via Library Manager)
 *   • ArduinoBLE      (Arduino)
 *   • ModbusMaster    (Doc Walker / 4-20mA.com)
 *
 * ============================================================
 *  USER CONFIGURATION — only edit this block
 * ============================================================
 */

// Functional Threshold Power in watts — scales ERG and free-ride current
const int   USER_FTP_WATTS        = 160;

// Pedal power floor — reported to Zwift even at very low cadence (watts)
const float FREE_RIDE_MIN_WATTS   = 50.0f;

// Cadence at which the rider should feel their full FTP wattage
const float FTP_CADENCE_RPM       = 85.0f;

// Cadence at which DPS_MAX_CURRENT_AMPS is commanded (full sprint load)
const float MAX_CADENCE_RPM       = 105.0f;

// Maximum current the DPS will ever be commanded to (amps × 100 for Modbus)
// Keep below your generator's stall current and DPS rated output.
const float DPS_MAX_CURRENT_AMPS  = 20.0f;

// Nominal generator / battery voltage.  Used to convert watts → amps when
// a real voltage reading from the DPS is unavailable.
const float DPS_NOMINAL_VOLTAGE   = 14.6f;

// Minimum load current held even at zero cadence (keeps DPS output live)
const float DPS_MIN_CURRENT_AMPS  = 2.0f;

// Combined drivetrain + rectifier + controller efficiency (0.0–1.0).
// The DPS measures electrical power AFTER losses; dividing by this factor
// back-calculates the mechanical power the rider actually produced at the
// pedals, which is what gets reported to Zwift.
// Example: 0.68 means 32% losses → pedal power = DPS_watts / 0.68
const float DRIVETRAIN_EFFICIENCY = 0.68f;

// Modbus slave address set on the DPS unit (default from factory = 1)
const uint8_t DPS_MODBUS_ADDRESS  = 1;

// Hall effect sensor pin (crank cadence)
const int HALL_SENSOR_PIN         = 2;

// Number of magnets glued to the crank arm
const int MAGNETS_PER_REVOLUTION  = 1;

// ============================================================
//  DPS Modbus register map (RD = read, WR = write)
//  All values are unsigned 16-bit.  Scaling noted per register.
// ============================================================
//  0x0000  U-SET    Voltage setpoint  (× 100, e.g. 2400 = 24.00 V)   WR
//  0x0001  I-SET    Current setpoint  (× 100, e.g.  500 =  5.00 A)   WR
//  0x0002  U-OUT    Output voltage    (× 100)                         RD
//  0x0003  I-OUT    Output current    (× 100)                         RD
//  0x0006  P-OUT    Output power      (× 100, watts)                  RD
//  0x0005  V-IN     Input voltage     (× 100)                         RD
//  0x0008  LOCK     Key lock          0=unlocked 1=locked             WR
//  0x0009  ON-OFF   Output enable     0=off 1=on                      WR
// ============================================================

#define DPS_REG_V_SET   0x0000
#define DPS_REG_I_SET   0x0001
#define DPS_REG_V_OUT   0x0002
#define DPS_REG_I_OUT   0x0003
#define DPS_REG_P_OUT   0x0006
#define DPS_REG_V_IN    0x0005
#define DPS_REG_LOCK    0x0008
#define DPS_REG_ON_OFF  0x0009

// ============================================================

#include <ArduinoBLE.h>
#include <ModbusMaster.h>

// ---------------------------------------------------------------------------
// ModbusMaster instance — talks directly over Serial1 (TTL UART, no RS-485)
// ---------------------------------------------------------------------------
ModbusMaster dps;

// ---------------------------------------------------------------------------
// BLE services & characteristics
// ---------------------------------------------------------------------------

// Cycling Power Service
BLEService        cpsService("1818");
BLECharacteristic cpMeasurement("2A63", BLENotify,  8);
BLECharacteristic cpFeature    ("2A65", BLERead,    4);
BLECharacteristic sensorLocation("2A5D", BLERead,   1);

// Fitness Machine Service
BLEService        ftmsService("1826");
BLECharacteristic ftmFeature      ("2ACC", BLERead,             8);
BLECharacteristic bikeData        ("2AD2", BLENotify,           8);
BLECharacteristic ftmControlPoint ("2AD9", BLEWrite|BLEIndicate, 20);
BLECharacteristic ftmStatus       ("2ADA", BLENotify,           2);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

volatile unsigned long lastHallTime = 0;
volatile unsigned long hallInterval = 0;
volatile bool          newPulse     = false;

float   currentCadenceRPM = 0.0f;
float   currentPowerWatts = 0.0f;   // read back from DPS (real watts)
float   currentVoltage    = 0.0f;   // read back from DPS
float   currentAmps       = 0.0f;   // read back from DPS

int16_t targetPowerWatts  = -1;     // ERG target from Zwift (-1 = free ride)
uint8_t targetResistPct   = 0;      // resistance % target from Zwift

uint16_t crankRevolutions = 0;

unsigned long lastNotifyMs  = 0;
unsigned long lastModbusMs  = 0;
const unsigned long NOTIFY_INTERVAL_MS = 500;   // BLE 2.0 Hz - GEN changed to 500 from 1.0/1000
const unsigned long MODBUS_INTERVAL_MS = 750;    // DPS poll @ 4 Hz  - GEN changed from 250 to 750


bool dpsOnline = false;

// ---------------------------------------------------------------------------
// ISR — hall effect sensor
// ---------------------------------------------------------------------------
void hallISR() {
    unsigned long now = micros();
    if (lastHallTime > 0) {
        hallInterval = now - lastHallTime;
        newPulse = true;
    }
    lastHallTime = now;
    crankRevolutions++;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000);

    Serial.println("=== Zwift Pedal Generator — DPS Modbus Edition ===");
    Serial.print("FTP: "); Serial.print(USER_FTP_WATTS); Serial.println(" W");
    Serial.print("Max DPS current: ");
    Serial.print(DPS_MAX_CURRENT_AMPS); Serial.println(" A");

    // Modbus over Serial1 — direct TTL UART to DPS5020 (no direction pin needed)
    // DPS5020 default baud rate is 9600
    Serial1.begin(9600);
    dps.begin(DPS_MODBUS_ADDRESS, Serial1);

    // Probe DPS and configure initial state
    initDPS();

    // Use built in LED to show connected to Zwift
    pinMode(LED_BUILTIN, OUTPUT);

    // Hall effect sensor
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallISR, FALLING);

    // BLE init
    if (!BLE.begin()) {
        Serial.println("FATAL: BLE init failed");
        while (true);
    }

    BLE.setDeviceName("Zwift Pedal Generator");
    BLE.setLocalName("PedalGenerator");
    BLE.setAdvertisedService(cpsService);

    // Cycling Power Service
    static uint8_t cpFeatureVal[4]  = {0x00, 0x00, 0x00, 0x00};
    static uint8_t sensorLocVal[1]  = {0x06};  // left crank
    cpFeature.writeValue(cpFeatureVal, 4);
    sensorLocation.writeValue(sensorLocVal, 1);
    cpsService.addCharacteristic(cpMeasurement);
    cpsService.addCharacteristic(cpFeature);
    cpsService.addCharacteristic(sensorLocation);
    BLE.addService(cpsService);

    // FTMS
    // Feature flags: cadence (bit3) + power (bit7) supported
    // Target setting: power target (bit3) + resistance target (bit7)
    static uint8_t ftmFeatureVal[8] = {0x88, 0x00, 0x00, 0x00,
                                        0x88, 0x00, 0x00, 0x00};  
    ftmFeature.writeValue(ftmFeatureVal, 8);
    ftmsService.addCharacteristic(ftmFeature);
    ftmsService.addCharacteristic(bikeData);
    ftmsService.addCharacteristic(ftmControlPoint);
    ftmsService.addCharacteristic(ftmStatus);
    BLE.addService(ftmsService);

    ftmControlPoint.setEventHandler(BLEWritten, onFTMControlPoint);

    BLE.advertise();
    Serial.println("BLE advertising — waiting for Zwift");
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Zwift connected: ");
        Serial.println(central.address());

        digitalWrite(LED_BUILTIN, HIGH);  // turn on the onboard LED when connected

        // Enable DPS output when Zwift connects
        setDPSOutput(true);

        while (central.connected()) {
            unsigned long now = millis();

            updateCadence();

            // Poll DPS and update current setpoint at Modbus rate
            if (now - lastModbusMs >= MODBUS_INTERVAL_MS) {
                lastModbusMs = now;
                readDPSMeasurements();
                applyLoadToDPS();
            }

            // BLE notifications 
            if (now - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
                lastNotifyMs = now;
                sendCPMeasurement();
                sendBikeData();
            }

            BLE.poll();
        }

        digitalWrite(LED_BUILTIN, LOW);

        Serial.println("Zwift disconnected — coasting (min load)");
        setDPSCurrentAmps(DPS_MIN_CURRENT_AMPS);
        setDPSOutput(false);
        targetPowerWatts = -1;
        targetResistPct  = 0;
    }
}

// ---------------------------------------------------------------------------
// Cadence — ISR-driven with stale detection
// ---------------------------------------------------------------------------
void updateCadence() {
    if (lastHallTime > 0 && (micros() - lastHallTime) > 3000000UL) {
        currentCadenceRPM = 0.0f;
        hallInterval      = 0;
        newPulse          = false;
        return;
    }
    if (newPulse && hallInterval > 0) {
        newPulse = false;
        currentCadenceRPM = (60.0f * 1000000.0f) /
                            ((float)hallInterval * MAGNETS_PER_REVOLUTION);
    }
}

// ---------------------------------------------------------------------------
// Read voltage, current, power back from DPS registers
// ---------------------------------------------------------------------------
void readDPSMeasurements() {
    // Read 3 consecutive registers starting at V-OUT (0x0002)
    uint8_t result = dps.readHoldingRegisters(DPS_REG_V_OUT, 3);
    if (result == dps.ku8MBSuccess) {
        currentVoltage    = dps.getResponseBuffer(0) / 100.0f;  // reg 0x0002
        currentAmps       = dps.getResponseBuffer(1) / 100.0f;  // reg 0x0003
        // DPS reports electrical power after drivetrain/rectifier/controller
        // losses.  Divide by efficiency to recover pedal power at the crank.
        float electricalWatts = dps.getResponseBuffer(2) / 100.0f;  // reg 0x0004
        currentPowerWatts = electricalWatts / DRIVETRAIN_EFFICIENCY;
        dpsOnline = true;
    } else {
        // Modbus error — estimate from setpoint if DPS unreachable
        dpsOnline = false;
        Serial.print("DPS read error: 0x");
        Serial.println(result, HEX);
    }
}

// ---------------------------------------------------------------------------
// Translate Zwift target (or free-ride cadence) into a DPS current setpoint
//
// ERG mode:   target_amps = targetPowerWatts / measured_voltage
// Free-ride:  target_amps = (cadence / cadence_at_ftp) * MAX_CURRENT
// Resistance: target_amps = (resistPct / 100) * MAX_CURRENT
// ---------------------------------------------------------------------------
void applyLoadToDPS() {
    float targetAmps = DPS_MIN_CURRENT_AMPS;

    if (targetPowerWatts >= 0) {
        // --- ERG mode ---
        // Zwift sends a pedal-power target.  The DPS must absorb the electrical
        // equivalent after drivetrain/rectifier/controller losses, so multiply
        // by efficiency before dividing by voltage to get the current setpoint.
        float vRef          = (currentVoltage > 1.0f) ? currentVoltage
                                                       : DPS_NOMINAL_VOLTAGE;
        float electricalTarget = (float)targetPowerWatts * DRIVETRAIN_EFFICIENCY;
        targetAmps          = electricalTarget / vRef;

        Serial.print("[ERG] pedal_target=");
        Serial.print(targetPowerWatts);
        Serial.print("W  elec_target=");
        Serial.print(electricalTarget, 1);
        Serial.print("W  V=");
        Serial.print(vRef, 1);
        Serial.print("V  I_cmd=");
        Serial.print(targetAmps, 2);
        Serial.println("A");

    } else if (targetResistPct > 0) {
        // --- Resistance % mode ---
        targetAmps = (targetResistPct / 100.0f) * ((USER_FTP_WATTS/currentVoltage)*DRIVETRAIN_EFFICIENCY);

        Serial.print("[Resintance % Mode] percent=");
        Serial.print(targetResistPct);
        Serial.print(" FTP=");
        Serial.print(USER_FTP_WATTS);
        Serial.print("W  V=");
        Serial.print(currentVoltage, 1);
        Serial.print("V  I_cmd=");
        Serial.print(targetAmps, 2);
        Serial.println("A");

    } else {
        // --- Free-ride: two-segment piecewise linear power curve ---
        //
        // Segment 1 (0 → FTP_CADENCE_RPM):
        //   pedal watts interpolate from FREE_RIDE_MIN_WATTS → USER_FTP_WATTS
        //
        // Segment 2 (FTP_CADENCE_RPM → MAX_CADENCE_RPM):
        //   pedal watts interpolate from USER_FTP_WATTS → max electrical load
        //   (DPS_MAX_CURRENT_AMPS × voltage / efficiency)
        //
        // Both segments are converted to a DPS current command via:
        //   electrical_watts = pedal_watts × DRIVETRAIN_EFFICIENCY
        //   amps = electrical_watts / voltage

        float vRef = (currentVoltage > 1.0f) ? currentVoltage
                                              : DPS_NOMINAL_VOLTAGE;

        // Maximum pedal watts the DPS can absorb at full current
        float maxPedalWatts = (DPS_MAX_CURRENT_AMPS * vRef) / DRIVETRAIN_EFFICIENCY;

        float pedalWatts;

        if (currentCadenceRPM <= 0.0f) {
            pedalWatts = FREE_RIDE_MIN_WATTS;

        } else if (currentCadenceRPM <= FTP_CADENCE_RPM) {
            // Segment 1 — linear from min watts up to FTP watts
            float t    = currentCadenceRPM / FTP_CADENCE_RPM;  // 0.0 → 1.0
            pedalWatts = FREE_RIDE_MIN_WATTS +
                         t * ((float)USER_FTP_WATTS - FREE_RIDE_MIN_WATTS);

        } else if (currentCadenceRPM < MAX_CADENCE_RPM) {
            // Segment 2 — linear from FTP watts up to max watts
            float t    = (currentCadenceRPM - FTP_CADENCE_RPM) /
                         (MAX_CADENCE_RPM   - FTP_CADENCE_RPM); // 0.0 → 1.0
            pedalWatts = (float)USER_FTP_WATTS +
                         t * (maxPedalWatts - (float)USER_FTP_WATTS);

        } else {
            // At or above max cadence — full DPS load
            pedalWatts = maxPedalWatts;
        }

        float electricalWatts = pedalWatts * DRIVETRAIN_EFFICIENCY;
        targetAmps = electricalWatts / vRef;

        Serial.print("[Free Ride Mode] pedalWatts = ");
        Serial.print(pedalWatts);
        Serial.print("W FTP=");
        Serial.print(USER_FTP_WATTS, 1);
        Serial.print("W  cadence=");
        Serial.print(currentCadenceRPM, 1);
        Serial.print(" RPM  I_cmd=");
        Serial.print(targetAmps, 2);
        Serial.println("A");    
    }

    // Clamp to hardware limits
    if (targetAmps < DPS_MIN_CURRENT_AMPS) targetAmps = DPS_MIN_CURRENT_AMPS;
    if (targetAmps > DPS_MAX_CURRENT_AMPS) targetAmps = DPS_MAX_CURRENT_AMPS;

    setDPSCurrentAmps(targetAmps);
}

// ---------------------------------------------------------------------------
// Write a current setpoint to DPS register I-SET (0x0001)
// DPS expects value × 100 (e.g. 3.50 A → 350)
// ---------------------------------------------------------------------------
void setDPSCurrentAmps(float amps) {

    uint16_t regVal = (uint16_t)(amps * 100.0f + 0.5f);
    Serial.print("Setting Current Amps to: ");
    Serial.println(amps);

    uint8_t result  = dps.writeSingleRegister(DPS_REG_I_SET, regVal);

    if (result != dps.ku8MBSuccess) {
        Serial.print("DPS I-SET write error: 0x");
        Serial.println(result, HEX);
    }

}

// ---------------------------------------------------------------------------
// Enable or disable DPS output (register 0x0010)
// ---------------------------------------------------------------------------
void setDPSOutput(bool enable) {
    uint8_t result = dps.writeSingleRegister(DPS_REG_ON_OFF,
                                              enable ? 0x0001 : 0x0000);
    if (result == dps.ku8MBSuccess) {
        Serial.print("DPS output: ");
        Serial.println(enable ? "ON" : "OFF");
    } else {
        Serial.print("DPS ON-OFF write error: 0x");
        Serial.println(result, HEX);
    }
}

// ---------------------------------------------------------------------------
// Initialise DPS on startup:
//   • unlock keypad
//   • set voltage to nominal (generator charges battery at this voltage)
//   • set minimum current
//   • leave output OFF until Zwift connects
// ---------------------------------------------------------------------------
void initDPS() {
    Serial.println("Probing DPS via Modbus...");
    delay(200);

    // Unlock keypad
    dps.writeSingleRegister(DPS_REG_LOCK, 0x0000);

    // Set voltage setpoint (× 100)
    uint16_t vSetReg = (uint16_t)(DPS_NOMINAL_VOLTAGE * 100.0f);
    uint8_t result   = dps.writeSingleRegister(DPS_REG_V_SET, vSetReg);

    if (result == dps.ku8MBSuccess) {
        dpsOnline = true;
        Serial.print("DPS online. V-SET = ");
        Serial.print(DPS_NOMINAL_VOLTAGE, 1);
        Serial.println(" V");
    } else {
        Serial.println("WARNING: DPS not responding. Check RS-485 wiring.");
        Serial.println("  Continuing — will retry each Modbus cycle.");
    }

    setDPSCurrentAmps(DPS_MIN_CURRENT_AMPS);
    setDPSOutput(false);
}

// ---------------------------------------------------------------------------
// BLE — Cycling Power Measurement notification (CPS 0x2A63)
// Byte layout: Flags(2) | Power sint16(2) | CrankRevs uint16(2) | EventTime uint16(2)
// ---------------------------------------------------------------------------
void sendCPMeasurement() {
    uint16_t flags     = 0x0010;  // bit4 = crank revolution data present
    uint16_t eventTime = (uint16_t)((millis() * 1024UL) / 1000UL);
    int16_t  powerInt  = (int16_t)currentPowerWatts;

    uint8_t buf[8];
    buf[0] = (uint8_t)(flags    & 0xFF);
    buf[1] = (uint8_t)(flags    >> 8);
    buf[2] = (uint8_t)(powerInt & 0xFF);
    buf[3] = (uint8_t)(powerInt >> 8);
    buf[4] = (uint8_t)(crankRevolutions & 0xFF);
    buf[5] = (uint8_t)(crankRevolutions >> 8);
    buf[6] = (uint8_t)(eventTime & 0xFF);
    buf[7] = (uint8_t)(eventTime >> 8);

    cpMeasurement.writeValue(buf, 8);

    Serial.print("BLE TX — Power: ");
    Serial.print((int)currentPowerWatts);
    Serial.print(" W  Cadence: ");
    Serial.print((int)currentCadenceRPM);
    Serial.print(" RPM  DPS: ");
    Serial.print(currentVoltage, 1);
    Serial.print("V / ");
    Serial.print(currentAmps, 2);
    Serial.println("A");
}

// ---------------------------------------------------------------------------
// BLE — FTMS Indoor Bike Data notification (0x2AD2)
// Flags(2) | Cadence uint16 in 0.5RPM(2) | Power sint16(2)
// ---------------------------------------------------------------------------
void sendBikeData() {
    uint16_t flags       = 0x0044;  // bit2=cadence, bit6=power
    uint16_t cadenceHalf = (uint16_t)(currentCadenceRPM * 2.0f);
    int16_t  powerInt    = (int16_t)currentPowerWatts;
    int s = round((80.0*0.14587)*3.6); // speed in km/h - filler added by GEN

    uint8_t buf[8];
    buf[0] = (uint8_t)(flags       & 0xFF);
    buf[1] = (uint8_t)(flags       >> 8);
    buf[2] = (uint8_t)(s & 0xFF);
    buf[3] = (uint8_t)(s >> 8) & 0xFF;    
    buf[4] = (uint8_t)(cadenceHalf & 0xFF);
    buf[5] = (uint8_t)(cadenceHalf >> 8);
    buf[6] = (uint8_t)(powerInt    & 0xFF);
    buf[7] = (uint8_t)(powerInt    >> 8);

    bikeData.writeValue(buf, 8);
}

// ---------------------------------------------------------------------------
// FTMS Control Point handler — called when Zwift writes 0x2AD9
//   0x00  Request Control
//   0x01  Reset
//   0x05  Set Target Resistance Level  (uint8, 0–100 %)
//   0x11  Set Target Power             (sint16, watts)
// ---------------------------------------------------------------------------
void onFTMControlPoint(BLEDevice device, BLECharacteristic characteristic) {
    int len = characteristic.valueLength();
    if (len < 1) return;

    const uint8_t* data = characteristic.value();
    uint8_t opCode = data[0];

    uint8_t response[3];
    response[0] = 0x80;    // Response Code
    response[1] = opCode;
    response[2] = 0x01;    // Success (default)

    switch (opCode) {
        case 0x00:  // Request Control
            Serial.println("FTMS: Control granted to Zwift");
            setDPSOutput(true);
            break;

        case 0x01:  // Reset — return to free-ride, minimum load
            targetPowerWatts = -1;
            targetResistPct  = 0;
            setDPSCurrentAmps(DPS_MIN_CURRENT_AMPS);
            Serial.println("FTMS: Reset — free-ride mode");
            break;

        case 0x04:  // Set Target Resistance (0–100 %)
            if (len >= 2) {
                targetResistPct  = data[1];
                targetPowerWatts = -1;
                Serial.print("FTMS: Resistance = ");
                Serial.print(targetResistPct);
                Serial.println(" %");
            } else {
                response[2] = 0x02;
            }
            break;        
            
        case 0x05:  // Set Target Power
            if (len >= 3) {
                int16_t watts    = (int16_t)(data[1] | (data[2] << 8));
                targetPowerWatts = (watts < 0) ? 0 : watts;
                targetResistPct  = 0;
                Serial.print("FTMS: Power target = ");
                Serial.print(targetPowerWatts);
                Serial.println(" W");
            } else {
                response[2] = 0x02;
            }
            break;

        case 0x07:  // Resume or Start the training
            Serial.println("FTMS: Resume or Start the training");
            break;

        case 0x08:  // Stop or pause training — return to free-ride, minimum load
            targetPowerWatts = -1;
            targetResistPct  = 0;
            setDPSCurrentAmps(DPS_MIN_CURRENT_AMPS);
            Serial.println("FTMS: Reset — free-ride mode");
            break;

        case 0x11:  // Set Target Power (ERG) Sim mode
            if (len >= 3) {
                int16_t watts    = (int16_t)(data[1] | (data[2] << 8));
                targetPowerWatts = (watts < 0) ? 0 : watts;
                targetResistPct  = 0;
                Serial.print("FTMS: Bike Sim ERG target = ");
                Serial.print(targetPowerWatts);
                Serial.println(" W");
            } else {
                response[2] = 0x02;
            }
            break;

        default:
            response[2] = 0x02;  // Not supported
            Serial.print("FTMS: Unhandled op 0x");
            Serial.println(opCode, HEX);
            break;
    }

    ftmControlPoint.writeValue(response, 3);

    uint8_t status[2] = {0x02, 0x00};
    ftmStatus.writeValue(status, 2);
}
