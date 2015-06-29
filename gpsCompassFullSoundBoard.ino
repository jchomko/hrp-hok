#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <avr/pgmspace.h>
//#include <math.h> 

//Magnetometer
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//GPS
SoftwareSerial mySerial(3, 4);



Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Waypoints
int wayPointIndex = 0;
const int nrWayPoints = 24;
const static float PROGMEM wayPoints[nrWayPoints] = {
 -0.08663704669734762,51.54877195418958,1000,0.02,
-0.08746145813916084,51.54878862889121,600,0.01,
-0.08720047218794047,51.54913161855076,1000,0.01,
-0.08445619532468696,51.54901217659432,600,0.01,
-0.08433878673073991,51.54859635355947,550,0.01,
-0.08604099348546002,51.54874467479521,550,0.01,
};


long checkSensorTimer = millis();

//const char NUM_SAMPLES = 5;
double originLat;
double originLong;


double targetLat;
double targetLong;
float targetMinDist;

float declinationDegree = -0.80; //0.80° W  ± 0.37°  changing by  0.15° E per year

//Heading calculations
float distanceToWaypoint, bearing, deltaLatitudeRadians, deltaLongitudeRadians;
float latitudeRadians, wayPointLatitudeRadians, longitudeRadians, wayPointLongitudeRadians;
const float pi = 3.14159265;
const int radiusOfEarth = 6371; // in km
float heading;


//Heartbeat samples
SoftwareSerial soundSerial(12,13);
#define SFX_RST 15


long heartbeatTimer;
float heartbeatFreq;

int heartbeatLength;

int targetFreq;
int currentFreq;
float freqInc = 0.01;


//Volume
#define LINE_BUFFER_SIZE  30
char line_buffer[LINE_BUFFER_SIZE];

int maxVol = 204;
int minVol = 0;
int volume = 0;
int targetVol = 204;

//end sequence
bool endSeq;
long endSeqTimer;
int endSeqDuration;

//debug new devices - don't wait for gps
bool hasFix = true;


bool debug = true;
bool ended = false;
bool runHeartbeat = false;

void setup() {

  //if(debug){
    Serial.begin(115200);
  //}
  
  delay(1000);
  //Setup GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  //Not sure if these are needed
  delay(1000);
  // Ask for firmware version
  //mySerial.println(PMTK_Q_RELEASE);
 if(debug);
  Serial.print("startup ");
   
  //Compass init
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    //digitalWrite(13, HIGH);
    /* There was a problem detecting the BNO055 ... check your connections */
    if(debug);
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  bno.setExtCrystalUse(true);
  delay(500);
  
  //Sound control 
  soundSerial.begin(9600);
  
  heartbeatTimer = 0;
  heartbeatFreq = 800;
  
  //End Sequence
  endSeq = false;
  endSeqDuration = 10*1000; // one minute
  ended = false;
  endSeqTimer = millis();
  
  //Debug
  wayPointIndex = 0;
  targetMinDist = 0.01;
  getNexWayPoint();
  
}


void getNexWayPoint() {
   
   if(wayPointIndex >= nrWayPoints){
      
      ended = true;
      endSeqTimer = millis();
      wayPointIndex = 0;
      Serial.println("ending");
      
   } 
   
    
   if(wayPointIndex < nrWayPoints) {
     //Lat and long are switched on KML google maps output
     //Long is first value, lat is second
     //Reading from SRAM 
     targetLong = pgm_read_float_near(wayPoints + wayPointIndex);
     targetLat = pgm_read_float_near(wayPoints + wayPointIndex + 1);
     targetFreq = pgm_read_float_near(wayPoints + wayPointIndex + 2);
     targetMinDist = pgm_read_float_near(wayPoints + wayPointIndex + 3);
     //targetLong = wayPoints[wayPointIndex];
     //targetLat = wayPoints[wayPointIndex + 1];
     //targetFreq = wayPoints[wayPointIndex + 2];
     
     wayPointIndex += 4;
    
    //we've reached the first point and on to the second
    //if we're on to the second waypoint
    if(wayPointIndex >= 8 && runHeartbeat == false){
      runHeartbeat = true;
      ended = false;
    }
    
    
    
    // if(debug){
      
      Serial.print("N,");
      Serial.println(wayPointIndex);
      Serial.print("T,");
      Serial.print(targetLat, 6);
      Serial.print(",");
      Serial.println(targetLong, 6);
      
      Serial.print("target freq : ");
      Serial.println(targetFreq);
      Serial.println(heartbeatFreq);
    //}
    
  }
}

void loop() {

  //if (checkSensorTimer > millis())  checkSensorTimer = millis();
 
 //if(debug){
    if(Serial.available() > 0){
        //heartbeatFreq = Serial.parseInt();
        //heartbeatSpace = Serial.parseInt();
        Serial.read();
        getNexWayPoint();
    }
// }
    

  //might make it easier to find the right direction
  //float diff = map(targetVol, 170,204, 400,0);
  //float diff = 0;
  
  if(heartbeatFreq > targetFreq){
    heartbeatFreq -= freqInc;
    //Serial.println(heartbeatFreq);
  }
  
  if(heartbeatFreq < targetFreq){
    heartbeatFreq += freqInc;
    //tar.println(heartbeatFreq);
  }
  
  heartbeatFreq = constrain(heartbeatFreq, 500,1400);
  
  //check gps and compass
  if (millis() - checkSensorTimer > 200) {
    
    mySerial.listen();
  
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    
    if (GPS.fix) {
      //hasFix = true;
      
      //if( GPS.latitudeDegrees > 0.0){
        originLat = GPS.latitudeDegrees;
      //}
      
      //if( GPS.longitudeDegrees > 0.0){
        originLong = GPS.longitudeDegrees;
      //}
      
     if(debug){
        Serial.print("C,");
        Serial.print(originLat, 6);
        Serial.print(",");
        Serial.println(originLong, 6);
        Serial.print("T,");
        Serial.print(targetLat, 6);
        Serial.print(",");
        Serial.println(targetLong, 6);
      }
      
    } 

    //Compass
    sensors_event_t event;
    bno.getEvent(&event);
    heading = event.orientation.x;
    
    bearing = calculateBearing();
    // bearing += declinationDegree;
   
    int diff = abs(bearing - heading);
    diff = abs((diff + 180) % 360 - 180);
    
    float dist = calculateDistance();
    
    if (dist <= targetMinDist) { // && wayPointIndex < nrWayPoints
        getNexWayPoint();
    }
    
    if(debug){
      Serial.print("K,"); Serial.println(heading);
      Serial.print("Y,"); Serial.println(bearing);
      Serial.print("D,"); Serial.println(diff);
      Serial.print("V,"); Serial.println(volume);
    }
    
    //This is the accuracy value
    diff = diff * 7 ;
    float amt = map(diff, 0, 180, 204, 150);
    
    if(!ended){
      targetVol = amt;
      targetVol = constrain(targetVol, 150, 204);
     }else{
      targetVol = 204;
    }
    
    checkSensorTimer = millis(); // reset the timer

  }


  soundSerial.listen();
   
  if(targetVol < volume ){
       
      soundSerial.write(45); //-  
      soundSerial.write(10); //NL
      delay(5);
     
  }
  
  if(targetVol > volume ){
       
       soundSerial.write(43); //++
       soundSerial.write(10); //NL
       delay(5);
  }
  
  if(soundSerial.available() ){
  
   int x  = soundSerial.readBytesUntil('\n', line_buffer, LINE_BUFFER_SIZE);
   line_buffer[x] = 0;
   //Serial.println(line_buffer);
   
   //if is volume 
   if( x == 6){
       //probably getting bad volume value here every x seconds or so
       int v = atoi(line_buffer);
        if( v > 0 && v <= 204){
         volume = v;
         if(debug){
         Serial.println(volume);
         }
       }
     }
  }
  
  
  if( ended && millis() - endSeqTimer < endSeqDuration){
    //could do something here like increase heartrate
  }
  
  
  if( ended && millis() - endSeqTimer > endSeqDuration && runHeartbeat){
    //Kill king after 30 secs
    runHeartbeat = false;
    Serial.println("ending heartbeat" );
    //ended = false;
  }
  
  //after 4 minutes - allow for startup
  if(ended && millis() - endSeqTimer > 20*1000){
    ended = false;
    Serial.println("resetting" );
    getNexWayPoint();
  }
  
  //Play heartbeat
  if( millis() > heartbeatTimer && runHeartbeat == true){ // && hasFix && !ended
      
      soundSerial.print('#');
      soundSerial.println(1);
      heartbeatTimer = millis() + heartbeatFreq;
      
  }

}

double angleFromCoordinate(double lat1, double long1, double lat2, double long2) {

  double dLon = (long2 - long1);

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1)
             * cos(lat2) * cos(dLon);

  double brng = atan2(y, x);
  brng =  brng * 180 / M_PI;
  brng = brng + 360;
  brng = (int)brng % 360;
  brng = 360 - brng;
  return brng;
}


void radianConversion() {

  deltaLatitudeRadians = (targetLat - originLat) * pi / 180;
  deltaLongitudeRadians = (targetLong - originLong) * pi / 180;
  latitudeRadians = originLat * pi / 180;
  wayPointLatitudeRadians = targetLat * pi / 180;
  longitudeRadians = originLong * pi / 180;
  wayPointLongitudeRadians = targetLong * pi / 180;

}


// calculate bearing from present location to next way point
float calculateBearing() {

  radianConversion();

  float y = sin(deltaLongitudeRadians) * cos(wayPointLatitudeRadians);
  float x = cos(latitudeRadians) * sin(wayPointLatitudeRadians) -
            sin(latitudeRadians) * cos(wayPointLatitudeRadians) * cos(deltaLongitudeRadians);

  bearing = atan2(y, x) / pi * 180;
  
  // bearing += declinationDegree;

  if (bearing < 0) {
    bearing = 360 + bearing;
  }

  return bearing;
}


float calculateDistance() {

  radianConversion();

  float a = sin(deltaLatitudeRadians / 2) * sin(deltaLatitudeRadians / 2) +
            sin(deltaLongitudeRadians / 2) * sin(deltaLongitudeRadians / 2) *
            cos(latitudeRadians) * cos(wayPointLatitudeRadians);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = radiusOfEarth * c; // distance in kilometers
  return d;
  //return d * 0.621371192; // distance in miles

}



SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }

}

