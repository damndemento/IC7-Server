#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>

#define FTMS_UUID "00001826-0000-1000-8000-00805f9b34fb"
#define FITNESS_MACHINE_FEATURES_UUID "00002acc-0000-1000-8000-00805f9b34fb"
#define INDOOR_BIKE_DATA_CHARACTERISTIC_UUID "00002ad2-0000-1000-8000-00805f9b34fb"

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

void setup()
{
    setupBluetoothServer();
    setupHalSensor();
}

BLECharacteristic *fitnessMachineFeaturesCharacteristic = NULL;
BLECharacteristic *indoorBikeDataCharacteristic = NULL;
BLEServer *pServer = NULL;
void setupBluetoothServer()
{
    Serial.begin(115200);
    Serial.println("Starting BLE Server");
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

int digitalPin = 27;

bool magStateOld;
void setupHalSensor()
{
    pinMode(digitalPin, INPUT);
    Serial.begin(9600);
    magStateOld = digitalRead(digitalPin);
}

inline bool positiveEdge(bool state, bool &oldState)
{
    bool result = (state && !oldState);
    oldState = state;
    return result;
}

double calculateCadenceFromRevolutions(int revolutions, unsigned long revolutionsTime)
{
    double instantaneousCadence = revolutions * 60 * 1000 / revolutionsTime / 4;
    return instantaneousCadence;
}

double calculateKphFromRevolutions(int revolutions)
{
    double WHEEL_RADIUS = 0.00034; // in km
    double KM_TO_MI = 0.621371;

    double circumfrence = 2 * PI * WHEEL_RADIUS;
    double metricDistance = revolutions * circumfrence;
    double kph = metricDistance * 60;
    //double mph = kph * KM_TO_MI; 
    return kph;
}

unsigned long distanceTime = 0;
double calculateDistanceFromKph(unsigned long distanceTimeSpan, double kph)
{
    double incrementalDistance = distanceTimeSpan * kph / 60 / 60 / 1000;
    return incrementalDistance;
}

double tireValues[] = {0.005, 0.004, 0.012};                      //Clincher, Tubelar, MTB
double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
unsigned long caloriesTime = 0;

double calculatePowerFromKph(double kph)
{
    //double velocity = mph * 0.44704; // translates to meters/second
    double velocity = kph * 0.277778; // translates to meters/second
    double riderWeight = 67.6;       //165 lbs
    double bikeWeight = 10.1;        //Cannondale road bike
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

// FTMS supported features bitmask (example: cadence, power, distance, speed, etc.)
byte features[] = {0xBE, 0x42, 0x00, 0x00};  // Adjust bits depending on what you actually support

void transmitFTMS(double kph, double cadence, double power, double avgKph, double avgCadence, 
                  double runningDistance, double runningCalories, unsigned long elapsedTime)
{
    // Convert to FTMS resolution
    uint16_t transmittedKph        = (uint16_t)(kph * 100);           // 0.01 km/h
    uint16_t transmittedCadence    = (uint16_t)(cadence * 2);         // 0.5 rpm
    uint16_t transmittedPower      = (uint16_t)(power * 2);           // 0.5 W
    uint16_t transmittedAvgKph     = (uint16_t)(avgKph * 100);        // 0.01 km/h
    uint16_t transmittedAvgCadence = (uint16_t)(avgCadence * 2);      // 0.5 rpm
    uint32_t transmittedDistance   = (uint32_t)(runningDistance * 1000); // meters
    uint16_t transmittedTotalCal   = (uint16_t)(runningCalories * 10); // 0.1 kcal
    uint16_t transmittedTime       = (uint16_t)(elapsedTime / 1000);  // seconds

    bool disconnecting = !deviceConnected && oldDeviceConnected;
    bool connecting = deviceConnected && !oldDeviceConnected;

    // Correct FTMS Indoor Bike Data order:
    // Flags (2 bytes) | Instantaneous Speed | Instantaneous Cadence | Instantaneous Power |
    // Average Speed | Average Cadence | Average Power | Total Distance | Total Calories | Elapsed Time
    byte bikeData[20] = {
        0x56, 0x09, // Flags

        // Instantaneous Speed
        (uint8_t)transmittedKph,
        (uint8_t)(transmittedKph >> 8),

        // Instantaneous Cadence
        (uint8_t)transmittedCadence,
        (uint8_t)(transmittedCadence >> 8),

        // Instantaneous Power
        (uint8_t)transmittedPower,
        (uint8_t)(transmittedPower >> 8),

        // Average Speed
        (uint8_t)transmittedAvgKph,
        (uint8_t)(transmittedAvgKph >> 8),

        // Average Cadence
        (uint8_t)transmittedAvgCadence,
        (uint8_t)(transmittedAvgCadence >> 8),

        // Total Distance (24-bit little-endian)
        (uint8_t)transmittedDistance,
        (uint8_t)(transmittedDistance >> 8),
        (uint8_t)(transmittedDistance >> 16),

        // Total Calories
        (uint8_t)transmittedTotalCal,
        (uint8_t)(transmittedTotalCal >> 8),

        // Elapsed Time
        (uint8_t)transmittedTime,
        (uint8_t)(transmittedTime >> 8)
    };

    // Send data over BLE if connected
    if (deviceConnected)
    {
        indoorBikeDataCharacteristic->setValue((uint8_t*)&bikeData, 20);
        indoorBikeDataCharacteristic->notify();
    }

    // Handle disconnect
    if (disconnecting)
    {
        delay(500);                  
        pServer->startAdvertising(); 
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // Handle first-time connect: send features
    if (connecting)
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
double totalCadence = 0;
double totaKph = 0;
double totalPower = 0;
double runningCalories = 0.0;
double runningDistance = 0.0;

void loop()
{
    unsigned long intervalTime = millis() - elapsedTime;
    unsigned long sampleTime = millis() - elapsedSampleTime;
    bool state = digitalRead(digitalPin);

    if (sampleTime > 5 && state != magStateOld)
    {
        rev += (int)positiveEdge(state, magStateOld);
        elapsedSampleTime = millis();
    }
    if (rev >= 2 || intervalTime > 1000)
    {
        double kph = calculateKphFromRevolutions(rev);
        double cadence = calculateCadenceFromRevolutions(rev,intervalTime);
        double power = calculatePowerFromKph(kph);
        
        intervalEntries++;
        totalCadence += cadence;
        totaKph += kph;
        totalPower += power;
        
        double avgCadence = totalCadence / intervalEntries;
        double avgKph   = totaKph    / intervalEntries;
        runningDistance += calculateDistanceFromKph(intervalTime, kph);
        runningCalories += calculateCaloriesFromPower(intervalTime, power);

        Serial.println("\n----------------------------------------------------");
        Serial.printf("elapsedTime: %d, rev: %d \n", elapsedTime, rev);
        Serial.printf("cadence: %2.2f, avgCadence: %2.2f \n", cadence, avgCadence);
        Serial.printf("kph: %2.2f, avgKph: %2.2f \n", kph, avgKph);
        Serial.printf("power: %2.2f watts, distance: %2.2f", power, runningDistance);
        Serial.printf("calories:  %2.5f \n", runningCalories);

        transmitFTMS(kph,cadence,power,avgKph,avgCadence,runningDistance,runningCalories,elapsedTime);

        // reset rev and elapsedTime
        rev = 0;
        elapsedTime = millis();
    }
}
