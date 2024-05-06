#include <Wire.h>
#include "vl53l5cx_arduino.h"
#include "debugger.hpp"

#define TRIGGER_DISTANCE 2000 // milimeters

static const uint8_t INT_PIN = 4; // Set to 0 for polling
static const uint8_t LPN_PIN =  23;

// Set to 0 for continuous mode
static const uint8_t INTEGRAL_TIME_MS = 0; // The integration time + 1 ms overhead must be lower than the measurement period

static VL53L5CX_Arduino _sensor(LPN_PIN, INTEGRAL_TIME_MS, VL53L5CX::RES_4X4_HZ_5);

static volatile bool _gotInterrupt;

int distance_buffer[16];
int zone_th[16];
int zone_trig[4];

int people_nr = 0;
int entry_set, exit_set;
// Initialize entry_zone and exit_zone with worst-case values
int entry_zone = 1;
int exit_zone = 2;

static void interruptHandler() 
{
    _gotInterrupt = true;
}

void setActiveZones(){ // Zones are lines consisting of 4 pixels each
    for(int i = 0; i<4; i++){ // Cycle through zones
        for(int j = 0; j<4; j++){ // Cycle through pixels in each zone
            if(zone_trig[i] == -1) // If zone is unavailable skip all pixels in that zone
            break;
            else if(distance_buffer[j+(i*4)]<zone_th[j+i*4]){ // If the distance in a given pixel is smaller than threshold set zone as triggered
                zone_trig[i] = 1;
                break; // Break if we find at least one pixel in the zone that is bellow threshold
            } 
            else zone_trig[i] = 0;
        }
    }
}

void readPixels(){
    for(int i=0; i<_sensor.getPixelCount(); i++){ // Cycle through pixels
        if (_sensor.getTargetDetectedCount(i) > 0) { // if there is a measurement available 
            distance_buffer[i] = _sensor.getDistanceMm(i); // read the distance into a buffer
        }
        else {
            distance_buffer[i] = 9999; // 9999 if the distance is too big or the measurement unsuccessful
        }
    }
}
void setThresholds(){
    for(int i=0;i<16;i++){
        if(distance_buffer[i] < (TRIGGER_DISTANCE+100) && distance_buffer[i] > 200) // If pixel distance is less than 60 cm 
                                                                    //but more than 20cm set the threshold to (distance - 10cm)
            zone_th[i] = distance_buffer[i]-200; // Set thresholds 
        else if(distance_buffer[i] <= 200) 
            zone_th[i] = 1; // Set threshold to 1cm, which means disable that pixel
        else 
            zone_th[i] = TRIGGER_DISTANCE; // For all unrestrained pixels set threshold to be 50cm
    }
    Debugger::printf("Zone thresholds: ");
    Debugger::printf("%4d  %4d  %4d  %4d\n\r",zone_th[3],zone_th[2],zone_th[1],zone_th[0]);
    Debugger::printf("%4d  %4d  %4d  %4d\n\r",zone_th[7],zone_th[6],zone_th[5],zone_th[4]);
    Debugger::printf("%4d  %4d  %4d  %4d\n\r",zone_th[11],zone_th[10],zone_th[9],zone_th[8]);
    Debugger::printf("%4d  %4d  %4d  %4d\n\r",zone_th[15],zone_th[14],zone_th[13],zone_th[12]);
}
void configureZones(){
    int unavailableZones = 0;
    // Check to see if an entire zone isn't blocked, if it is don't use it
    for(int i=0; i<4; i++){
        for (int k=0;k<4;k++){
            if(zone_th[k+i*4] != 1) break;
            else {
                zone_trig[i] = -1; // Set zone to -1 to indicate that zone is unavailable
                unavailableZones++;
                if(unavailableZones > 2){
                    Debugger::printf("Not enough available zones, sensor won't work");
                    while(1); 
                }
            }
        }
    }
    // Find the furthest apart available zones for entry and exit trigger
    for (int i = 0; i < 4; ++i) {
        // Check if the current zone is available
        if (zone_trig[i] != -1) {
            // Update entry_zone and exit_zone if available zone found
            if (i < entry_zone) entry_zone = i; // Set smaller index to entry_zone
            if (i > exit_zone) exit_zone = i;   // Set higher index to exit_zone
        }
    }
    Debugger::printf("Entry zone: %d, Exit zone: %d\n\n\n\r",entry_zone,exit_zone);
}
void printData(){
    // Debugger::printf("%4d  %4d  %4d  %4d\n\r",distance_buffer[3],distance_buffer[2],distance_buffer[1],distance_buffer[0]);
    // Debugger::printf("%4d  %4d  %4d  %4d\n\r",distance_buffer[7],distance_buffer[6],distance_buffer[5],distance_buffer[4]);
    // Debugger::printf("%4d  %4d  %4d  %4d\n\r",distance_buffer[11],distance_buffer[10],distance_buffer[9],distance_buffer[8]);
    // Debugger::printf("%4d  %4d  %4d  %4d\n\r",distance_buffer[15],distance_buffer[14],distance_buffer[13],distance_buffer[12]);
    Debugger::printf("Active zones: ");
        for(int i=0; i<4; i++){
            printf("%d ",zone_trig[i]);
        }
        printf("\n");
}
void printDataAlt(){
    for (auto i=0; i<_sensor.getPixelCount(); i++) {

            // Print per zone results 
            Debugger::printf("Zone : %2d, Nb targets : %2u, Ambient : %4lu Kcps/spads, ",
                    i, _sensor.getTargetDetectedCount(i), _sensor.getAmbientPerSpad(i));

            // Print per target results 
            if (_sensor.getTargetDetectedCount(i) > 0) {
                Debugger::printf("Target status : %3u, Distance : %4d mm\n",
                        _sensor.getTargetStatus(i), _sensor.getDistanceMm(i));
                distance_buffer[i] = _sensor.getDistanceMm(i);
            }
            else {
                Debugger::printf("Target status : 255, Distance : No target\n");
                distance_buffer[i] = 9999;
            }
        }
        Debugger::printf("\n");
}

void setup(void)
{
    Serial.begin(115200);
    Debugger::printf("Serial begun!\n");

    pinMode(INT_PIN, INPUT);     
    pinMode(15,OUTPUT);
    pinMode(2,OUTPUT);
    Wire.begin();   //Start I2C, set I2C frequency to 400kHz 
    Wire.setClock(400000);      
    delay(1000);

    if (INT_PIN > 0) {
        attachInterrupt(INT_PIN, interruptHandler, FALLING); // interrupts whenever new data is available
    }

    _sensor.begin();

    int calibrated = 0;

    digitalWrite(15,HIGH);

    delay(5000); // wait for people to move away

    digitalWrite(15,LOW);
    Debugger::printf("Starting calibration\n\r");
    while(!calibrated){
        // Calibration, if there is a door near it should be closed
        if(INT_PIN == 0 || _gotInterrupt){
            _gotInterrupt = false;
            while (!_sensor.dataIsReady()) { // Wait for data
                delay(10);
            }
            _sensor.readData();
            readPixels();
            setThresholds();
            calibrated = 1;
        }
    }
    Debugger::printf("Calibration finished\n\r");
    configureZones();

    digitalWrite(2,HIGH); // Turn on builtin blue led when the sensor starts working
}

void loop(void)
{
    if (INT_PIN == 0 || _gotInterrupt) {

        _gotInterrupt = false;
        while (!_sensor.dataIsReady()) { // Wait for data
            delay(10);
        }
        _sensor.readData();
        readPixels();        
        setActiveZones();
        printData();

        // PEOPLE DETECION
        // Entry/Exit detection
        if(zone_trig[entry_zone] == 0 && zone_trig[exit_zone] == 1 && entry_set) { // Entry complete
            people_nr++;
            entry_set = 0;
            exit_set = 1;
        }
        else if(zone_trig[entry_zone] == 0 && zone_trig[exit_zone] == 1) exit_set = 1; // Exit set

        if(zone_trig[entry_zone] == 1 && zone_trig[exit_zone] == 0 && exit_set && people_nr>0){ // Exit complete
            people_nr--;
            entry_set = 1;
            exit_set = 0;
        }
        else if(zone_trig[entry_zone] == 1 && zone_trig[exit_zone] == 0) entry_set = 1; // Entry set

        if(zone_trig[0] != 1 && zone_trig[1] != 1 && zone_trig[2] != 1 && zone_trig[3] != 1){ // If all zones empty reset flags
            entry_set = 0;
            exit_set = 0;
        }
        printf("Number of people: %d \n",people_nr);
        if(people_nr > 0){
            digitalWrite(15,HIGH);
        }
        else digitalWrite(15,LOW);
    } 
}
