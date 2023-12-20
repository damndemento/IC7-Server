
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

int digitalPin = 32;
bool magStateOld;
void setupHalSensor()
{
    pinMode(digitalPin, INPUT);
    Serial.begin(9600);
    //debugging
    //magStateOld = digitalRead(simulateSensor());
    magStateOld = digitalRead(digitalPin);
}

/*
// Globale Variable zur Simulation der Zeit
unsigned long simulatedTime = 0;

// Funktion zur Simulation von 20 Umdrehungen pro Minute
bool simulateSensor() {
    // Eine Umdrehung alle 250 Millisekunden entspricht 240 Umdrehungen pro Minute
    unsigned long revolutionInterval = 250; // in Millisekunden

    // Berechnen Sie die Zeit seit der letzten "Umdrehung"
    unsigned long timeSinceLastRevolution = millis() - simulatedTime;
    
    // debugging
    Serial.print("millis: ");
    Serial.println(millis()); // Gibt die aktuelle Zeit aus
    Serial.print("simtime:");
    Serial.println(simulatedTime); // Gibt die simulierte Zeit aus
    
    // Wenn genug Zeit vergangen ist, simulieren Sie eine Umdrehung
    if (timeSinceLastRevolution >= revolutionInterval) {
        simulatedTime = millis();
        return true;
    } else {
        return false;
    }
} */

//incrementRevolutions() used to synchronously update rev rather than using an ISR.
inline bool positiveEdge(bool state, bool &oldState)
{
    bool result = (state && !oldState);//latch logic
    oldState = state;
    return result;
}
/*unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 5;    // the debounce time; increase if the output flickers

inline bool positiveEdge(bool state, bool &oldState)
{
    bool result = false;
    unsigned long currentTime = millis();

    // If the state has changed from LOW to HIGH, reset the debouncing timer
    if (state && !oldState) {
        lastDebounceTime = currentTime;
    }

    // Whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    if ((currentTime - lastDebounceTime) > debounceDelay) {
        // If the state has changed from LOW to HIGH
        if (state && !oldState) {
            oldState = state;
            result = true;
        }
        // If the state has changed from HIGH to LOW
        else if (!state && oldState) {
            oldState = state;
        }
    }

    return result;
}*/

double calculateRpmFromRevolutions(int revolutions, unsigned long revolutionsTime)
{
    double instantaneousRpm = revolutions * 60 * 1000 / revolutionsTime;
    //    Serial.printf("revolutionsTime: %d, rev: %d , instantaneousRpm: %2.9f \n",
    //                    revolutionsTime, revolutions, instantaneousRpm);
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
    //double mph = kph * KM_TO_MI; 
                                        //    Serial.printf("rpm: %2.2f, circumfrence: %2.2f, metricDistance %2.5f , imperialDistance: %2.5f, mph: %2.2f \n",
                                        //                   rpm, circumfrence, metricDistance, imperialDistance, mph);
    return kph;
}

double calculateCadenceFromRpm(double rpm)
{
    double cadence = rpm / 4; // 4 revolutions per pedal turn

    return cadence;
}

unsigned long distanceTime = 0;
double calculateDistanceFromKph(unsigned long distanceTimeSpan, double kph)
{
    double incrementalDistance = distanceTimeSpan * kph / 60 / 60 / 1000;
    //    Serial.printf("mph: %2.2f, distanceTimeSpan %d , incrementalDistance: %2.9f \n",
    //                   mph, distanceTimeSpan, incrementalDistance);
    return incrementalDistance;
}

double tireValues[] = {0.005, 0.004, 0.012};                      //Clincher, Tubelar, MTB
double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
unsigned long caloriesTime = 0;
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



//Used for debugging, i.e. `printArray(bikeData, sizeof(bikeData));` 
//NOTE: sizeOfArray parameter is necessary 
//This is because `sizeOf()` will return the size of the pointer to the array, not the array
void printArray(byte input[], int sizeOfArray)
{
    for (size_t i = 0; i < sizeOfArray; i++) 
    {
        Serial.print(input[i]);
        Serial.print(' ');
    }
}

byte features[] = {0x07,0x52,0x00,0x00}; 
// 0x07,0x52 (0x48,0x4a in big-endian) are flags for
// avgSpeed (0), cadence (1), total distance (2), expended energy (9), elapsed time (12), power measurement (14)
void transmitFTMS(double kph, double avgKph, double cadence, double avgCadence, double runningDistance,
                  double power, double runningCalories, double avgPower, unsigned long elapsedTime)
{
    uint16_t transmittedKph        = (uint16_t) (kph * 100);         //(0.01 resolution)
    uint16_t transmittedAvgKph     = (uint16_t) (avgKph * 100);      //(0.01 resolution)
    uint16_t transmittedCadence    = (uint16_t) (cadence * 2);
    uint16_t transmittedAvgCadence = (uint16_t) (avgCadence * 2);
    uint32_t transmittedDistance   = (uint32_t) (runningDistance * 1000);// runningDistance in km, need m 
    uint16_t transmittedPower      = (uint16_t) (power * 2);      //(1.0 resolution)
    uint16_t transmittedTotalCal   = (uint16_t) (runningCalories * 10);                   //(1.0 resolution)
    uint16_t transmittedCalHr      = (uint16_t) (runningCalories * 60 * 60 / elapsedTime);//(1.0 resolution) 
    uint8_t transmittedCalMin      = (uint8_t)  (runningCalories * 60 / elapsedTime);     //(1.0 resolution)
    uint16_t transmittedAvgPower   = (uint16_t) (avgPower * 2);        //(1.0 resolution)
    uint16_t transmittedTime       = (uint16_t) (elapsedTime / 1000);//(1.0 resolution)
    
    bool disconnecting = !deviceConnected && oldDeviceConnected;
    bool connecting = deviceConnected && !oldDeviceConnected;
    
    byte bikeData1[20]={0x0E,0x15,
                (uint8_t)transmittedKph,        (uint8_t)(transmittedKph >> 8),
                (uint8_t)transmittedAvgKph,     (uint8_t)(transmittedAvgKph >> 8),
                (uint8_t)transmittedCadence,    (uint8_t)(transmittedCadence >> 8),
                (uint8_t)transmittedAvgCadence, (uint8_t)(transmittedAvgCadence >> 8),
                (uint8_t)transmittedDistance,   (uint8_t)(transmittedDistance >> 8), (uint8_t)(transmittedDistance >> 16),        
                (uint8_t)transmittedPower,      (uint8_t)(transmittedPower >> 8),                   
                (uint8_t)transmittedTotalCal,   (uint8_t)(transmittedTotalCal >> 8),                    
                (uint8_t)transmittedCalHr,      (uint8_t)(transmittedCalHr >> 8),                    
                transmittedCalMin
    };
    byte bikeData2[6]={0x01, 0x40,
                (uint8_t)transmittedAvgPower,   (uint8_t)(transmittedAvgPower >> 8),
                (uint8_t)transmittedTime,      (uint8_t)(transmittedTime >> 8)
    };

    if (deviceConnected)
    {
        //NOTE: Even though the ATT_MTU for BLE is 23 bytes, android by default only captures the first 20 bytes.
        indoorBikeDataCharacteristic->setValue((uint8_t *)&bikeData1, 20);
        indoorBikeDataCharacteristic->notify();
        delay(250);
        indoorBikeDataCharacteristic->setValue((uint8_t *)&bikeData2, 6);
        indoorBikeDataCharacteristic->notify();
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
double totaKph = 0;
double totalPower = 0;
double runningCalories = 0.0;
double runningDistance = 0.0;

void loop()
{
    /*
    // debugging
    bool state = simulateSensor();
    if (simulateSensor()) {
        rev++;
    }
    Serial.println(state); // Gibt den Zustand des simulierten Sensors aus
    */
    unsigned long intervalTime = millis() - elapsedTime;
    unsigned long sampleTime = millis() - elapsedSampleTime;
    bool state = digitalRead(digitalPin); //off for debugging purposes
    if (sampleTime > 5 && state == HIGH)
    //if (sampleTime > 5 && state != magStateOld)
    {
        rev += (int)positiveEdge(state, magStateOld);
        elapsedSampleTime = millis();
    }
    if (intervalTime > 500)
    {
        Serial.print("rev:    ");
        Serial.println(rev);
        Serial.print("intTime:");
        Serial.println(intervalTime);

        double rpm = calculateRpmFromRevolutions(rev, intervalTime);
        double cadence = calculateCadenceFromRpm(rpm);
        double kph = calculateKphFromRpm(rpm);
        double power = calculatePowerFromKph(kph);
        
        // debugging
        Serial.print("rpm:    ");
        Serial.println(rpm); // Gibt die berechneten RPM aus
        Serial.print("cadence:");
        Serial.println(cadence); // Gibt die berechnete Trittfrequenz aus
        Serial.print("kph:    ");
        Serial.println(kph); // Gibt die berechnete Geschwindigkeit aus

        intervalEntries++;
        totalRpm += rpm;
        totalCadence += cadence;
        totaKph += kph;
        totalPower += power;
        
        double avgRpm     = totalRpm     / intervalEntries;
        double avgCadence = totalCadence / intervalEntries;
        double avgKph     = totaKph      / intervalEntries;
        double avgPower   = totalPower   / intervalEntries;
        runningDistance += calculateDistanceFromKph(intervalTime, kph);
        
        runningCalories += calculateCaloriesFromPower(intervalTime, power);

        // bluetooth becomes congested if too many packets are sent. In a 6 hour test I was able to go as frequent as 3ms.
        transmitFTMS(kph,avgKph,cadence,avgCadence,runningDistance,power,runningCalories,avgPower,elapsedTime);

        rev = 0;
        elapsedTime = millis();
    }
}