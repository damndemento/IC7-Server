
/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    edited for D1 mini ESP32 with Schwinn IC7 bikes by damndemento
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define FTMS_UUID "00001826-0000-1000-8000-00805f9b34fb"
#define FITNESS_MACHINE_FEATURES_UUID "00002acc-0000-1000-8000-00805f9b34fb"
#define INDOOR_BIKE_DATA_CHARACTERISTIC_UUID "00002ad2-0000-1000-8000-00805f9b34fb"
#define LED_BUILTIN LED_BUILTIN

bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

struct EnergyData {
    uint16_t totalEnergy;     // cumulative kJ
    uint16_t energyPerHour;   // kJ/h
    uint8_t  energyPerMinute; // kJ/min
};

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
    setupBluetoothServer();
    setupHalSensor();
}

BLECharacteristic *fitnessMachineFeaturesCharacteristic = NULL;
BLECharacteristic *indoorBikeDataCharacteristic = NULL;
BLEServer *pServer = NULL;

void setupBluetoothServer()
{
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
    BLEDevice::init("IC7 Bike");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(FTMS_UUID);
    fitnessMachineFeaturesCharacteristic = pService->createCharacteristic(
        FITNESS_MACHINE_FEATURES_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);
    indoorBikeDataCharacteristic = pService->createCharacteristic(
        INDOOR_BIKE_DATA_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);
    //  BLE2803 and BLE2902 are the UUIDs for Characteristic Declaration (0x2803) and Descriptor Declaration (0x2902).
    fitnessMachineFeaturesCharacteristic->addDescriptor(new BLE2902());
    indoorBikeDataCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // add this for backwards compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(FTMS_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Waiting for a client connection to notify...");
}

int digitalPin = 27; // GPIO Pin number for ESP32 (or gpio32)

bool magStateOld;
unsigned long distanceTime = 0;
double tireValues[] = {0.005, 0.004, 0.012};                      //Clincher, Tubelar, MTB
double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
unsigned long caloriesTime = 0;

void setupHalSensor()
{
    pinMode(digitalPin, INPUT);
    Serial.begin(9600);
    magStateOld = digitalRead(digitalPin);
}

inline bool positiveEdge(bool state, bool &oldState)
{
    bool result = (state && !oldState);//latch logic
    oldState = state;
    return result;
}

EnergyData calculateEnergy(double runningCalories, unsigned long intervalMs) {
    EnergyData e;
    e.totalEnergy = (uint16_t)(runningCalories * 4.184); // kcal → kJ

    double intervalHours = intervalMs / 3600000.0;
    double intervalMinutes = intervalMs / 60000.0;

    e.energyPerHour = (uint16_t)((runningCalories * 4.184) / intervalHours);
    e.energyPerMinute = (uint8_t)((runningCalories * 4.184) / intervalMinutes);

    return e;
}

double calculateRpmFromRevolutions(int revolutions, unsigned long revolutionsTime)
{
    double instantaneousRpm = revolutions * 60 * 1000 / revolutionsTime;
    return instantaneousRpm;
}

double calculateKphFromRpm(double rpm)
{
    double WHEEL_RADIUS = 0.00034; // in km 28"Wheel
    //double WHEEL_RADIUS = 0.0003682; // in km 29"Wheel
    //double KM_TO_MI = 0.621371;

    double circumfrence = 2 * PI * WHEEL_RADIUS;
    double metricDistance = rpm * circumfrence;
    double kph = metricDistance * 60;
    return kph;
    //double mph = kph * KM_TO_MI;
    //return mph;
}

double calculateCadenceFromRpm(double rpm)
{
    double cadence = rpm / 4; // 4 revolutions per pedal turn

    return cadence;
}

double calculateDistanceFromKph(unsigned long distanceTimeSpan, double kph)
{
    double incrementalDistance = distanceTimeSpan * kph / 60 / 60 / 1000;
    return incrementalDistance;
}

double calculatePowerFromKph(double kph)
{
    //double velocity = mph * 0.44704; // translates to meters/second
    double velocity = kph * 0.277778; // translates to meters/second
    double riderWeight = 72.6;       //165 lbs
    double bikeWeight = 11.1;        //Cannondale road bike
    int theTire = 0;                 //Clinchers
    double rollingRes = tireValues[theTire];
    int theAero = 1; //Bartops
    double frontalArea = aeroValues[theAero];
    double grade = 0;
    double headwind = 0;        // converted to m/s
    double temperaturev = 15.6; // 60 degrees farenheit
    double elevation = 100;     // Meters
    double transv = 0.95;       // no one knows what this is, so why bother presenting a choice?

    /* Common calculations */
    double density = (1.293 - 0.00426 * temperaturev) * exp(-elevation / 7000.0);
    double twt = 9.8 * (riderWeight + bikeWeight); // total weight in newtons
    double A2 = 0.5 * frontalArea * density;       // full air resistance parameter
    double tres = twt * (grade + rollingRes);      // gravity and rolling resistance

    // we calculate power from velocity
    double tv = velocity + headwind;      //terminal velocity
    double A2Eff = (tv > 0.0) ? A2 : -A2; // wind in face so you must reverse effect
    return (velocity * tres + velocity * tv * tv * A2Eff) / transv;    
}

double calculateCaloriesFromPower(unsigned long caloriesTimeSpan, double powerv)
{
    double JOULE_TO_KCAL = 0.238902957619;
    // From the formula: Energy (Joules) = Power (Watts) * Time (Seconds)
    double incrementalCalories =  powerv * caloriesTimeSpan / 60 / 1000 * JOULE_TO_KCAL; 
    double wl = incrementalCalories / 32318.0;    // comes from 1 lb = 3500 Calories
    return incrementalCalories;
}

void printArray(byte input[], int sizeOfArray)
{
    for (size_t i = 0; i < sizeOfArray; i++) 
    {
        Serial.print(input[i]);
        Serial.print(' ');
    }
}

byte features[] = { 0x4F, 0x48, 0x00, 0x00 }; // little endian

void transmitFTMS(double kph, double avgKph, double cadence, double avgCadence,
                  double runningDistance, double power, double runningCalories,
                  double avgPower, unsigned long elapsedTime)
{
    EnergyData energy = calculateEnergy(runningCalories, elapsedTime);
    // ---------- Packet 1: Speed, Cadence, Distance, Power, Time, Energy ----------
    uint16_t flags1 = 0;
    flags1 |= (1 << 1);  // instantaneous speed (mandatory)
    flags1 |= (1 << 2);  // instantaneous cadence
    flags1 |= (1 << 4);  // total distance
    flags1 |= (1 << 6);  // instantaneous power
    flags1 |= (1 << 8);  // expended energy
    flags1 |= (1 << 11); // elapsed time

    uint16_t transmittedKph        = (uint16_t)(kph * 100);       // 0.01 km/h
    uint16_t transmittedCadence    = (uint16_t)(cadence * 2);     // 0.5 rpm resolution
    uint32_t transmittedDistance   = (uint32_t)(runningDistance * 1000); // meters
    uint16_t transmittedPower      = (uint16_t)power;             // watts
    uint16_t totalEnergy           = energy.totalEnergy;  // optional, if you want consistent naming
    uint16_t transmittedTime       = (uint16_t)(elapsedTime / 1000); // seconds

    uint8_t packet1[20];
    int i = 0;
    packet1[i++] = flags1 & 0xFF;
    packet1[i++] = (flags1 >> 8) & 0xFF;

    packet1[i++] = transmittedKph & 0xFF;
    packet1[i++] = transmittedKph >> 8;

    packet1[i++] = transmittedCadence & 0xFF;
    packet1[i++] = transmittedCadence >> 8;

    packet1[i++] = transmittedDistance & 0xFF;
    packet1[i++] = (transmittedDistance >> 8) & 0xFF;
    packet1[i++] = (transmittedDistance >> 16) & 0xFF;

    packet1[i++] = transmittedPower & 0xFF;
    packet1[i++] = transmittedPower >> 8;

    packet1[i++] = transmittedTime & 0xFF;
    packet1[i++] = transmittedTime >> 8;

    packet1[i++] = totalEnergy & 0xFF;
    packet1[i++] = totalEnergy >> 8;
    packet1[i++] = 0;                            // Energy per Hour (LSB)
    packet1[i++] = 0;                            // Energy per Hour (MSB)
    packet1[i++] = 0;                            // Energy per Minute (1B)


    // ---------- Packet 2: Averages only (Continuation) ----------
    uint16_t flags2 = 0;
    flags2 |= (1 << 0);  // More Data = 1 (continuation)
    flags2 |= (1 << 1);  // average speed
    flags2 |= (1 << 3);  // average cadence
    flags2 |= (1 << 7);  // average power

    uint16_t transmittedAvgKph     = (uint16_t)(avgKph * 100);
    uint16_t transmittedAvgCadence = (uint16_t)(avgCadence * 2);
    uint16_t transmittedAvgPower   = (uint16_t)avgPower;
    uint16_t energyPerHour = energy.energyPerHour;
    uint8_t  energyPerMinute = energy.energyPerMinute;

    uint8_t packet2[20];
    int j = 0;
    packet2[j++] = flags2 & 0xFF;
    packet2[j++] = (flags2 >> 8) & 0xFF;

    packet2[j++] = transmittedAvgKph & 0xFF;
    packet2[j++] = transmittedAvgKph >> 8;

    packet2[j++] = transmittedAvgCadence & 0xFF;
    packet2[j++] = transmittedAvgCadence >> 8;

    packet2[j++] = transmittedAvgPower & 0xFF;
    packet2[j++] = transmittedAvgPower >> 8;

    packet2[j++] = energyPerHour & 0xFF;
    packet2[j++] = energyPerHour >> 8;
    
    packet2[j++] = energyPerMinute & 0xFF;


    // ---------- Connection Handling ----------
    bool disconnecting = !deviceConnected && oldDeviceConnected;
    bool connecting = deviceConnected && !oldDeviceConnected;

    if (deviceConnected) {
        indoorBikeDataCharacteristic->setValue(packet1, i);
        indoorBikeDataCharacteristic->notify();
        printArray(packet1, i); // Debug
        delay(5); // small delay to avoid BLE congestion
        indoorBikeDataCharacteristic->setValue(packet2, j);
        indoorBikeDataCharacteristic->notify();
        printArray(packet2, j); // Debug
    }
    
    if (disconnecting) // give the bluetooth stack the chance to get things ready & restart advertising
    {
        delay(500);                  
        pServer->startAdvertising(); 
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (connecting) // execute one time notification of supported features
    { 
        oldDeviceConnected = deviceConnected;
        fitnessMachineFeaturesCharacteristic->setValue((byte*)&features, 4);
        fitnessMachineFeaturesCharacteristic->notify();
    }
}

unsigned long elapsedTime = 0;
unsigned long elapsedSampleTime = 0;
int rev = 0;
double intervalEntries = 0;
double totalRpm = 0;
double totalCadence = 0;
double totalKph = 0;
double totalPower = 0;
double runningCalories = 0.0;
double runningDistance = 0.0;

void loop()
{
    unsigned long intervalTime = millis() - elapsedTime;
    unsigned long sampleTime = millis() - elapsedSampleTime;
    bool state = digitalRead(digitalPin);
    
    // Count revolutions on positive edge
    if (sampleTime > 5 && state != magStateOld)
    {
        rev += (int)positiveEdge(state, magStateOld);
        elapsedSampleTime = millis();
    }

    if (rev >= 2 || intervalTime > 1000) 
    {
        double rpm = calculateRpmFromRevolutions(rev, intervalTime);
        double cadence = calculateCadenceFromRpm(rpm);
        double kph = calculateKphFromRpm(rpm);
        double power = calculatePowerFromKph(kph);

        // Only update totals and averages if there was a revolution
        if (rev > 0)
        {
            intervalEntries++;
            totalRpm += rpm;
            totalCadence += cadence;
            totalKph += kph;
            totalPower += power;
            runningDistance += calculateDistanceFromKph(intervalTime, kph);
            runningCalories += calculateCaloriesFromPower(intervalTime, power);
        }

        double avgRpm     = (intervalEntries > 0) ? totalRpm / intervalEntries : 0;
        double avgCadence = (intervalEntries > 0) ? totalCadence / intervalEntries : 0;
        double avgKph     = (intervalEntries > 0) ? totalKph / intervalEntries : 0;
        double avgPower   = (intervalEntries > 0) ? totalPower / intervalEntries : 0;

        // Serial Debug Printout
        /*
        Serial.println("\n----------------------------------------------------");
        Serial.printf("elapsedTime: %lu, rev: %d \n", elapsedTime, rev);
        Serial.printf("rpm: %2.2f, avgRpm: %2.2f \n", rpm, avgRpm);
        Serial.printf("cadence: %2.2f, avgCadence: %2.2f \n", cadence, avgCadence);
        Serial.printf("kph: %2.2f, avgKph: %2.2f \n", kph, avgKph);
        Serial.printf("power: %2.2f watts, avgPower: %2.2f watts \n", power, avgPower);
        Serial.printf("distance: %2.2f, calories: %2.5f \n", runningDistance, runningCalories);
        */
        
        // Send FTMS packets
        transmitFTMS(kph, avgKph, cadence, avgCadence, runningDistance, power, runningCalories, avgPower, elapsedTime);

        // Reset revolutions for next interval
        rev = 0;
        elapsedTime = millis();
    }
}
