#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <avr/pgmspace.h>
//#include <math.h> 

bool debug = false;

//Magnetometer
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//GPS
SoftwareSerial mySerial(3, 4);


//float startLat = 51.50377023880998;
//float startLong = -0.1369838400094925;

Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Waypoints
int wayPointIndex = 0;
const int nrWayPoints = 75;
const static float PROGMEM wayPoints[nrWayPoints] = {
-0.1369838400094925,51.50377023880998,1000,
-0.1368811176538509,51.50366174958437,500,
-0.1363985210878116,51.50384910355171,700,
-0.1363376213810341,51.50372420981144,700,
-0.1361385795431025,51.50352879390383,700,
-0.1357797265929273,51.50349478788194,650,
-0.1352453948709686,51.5037516245444,750,
-0.1341916689411571,51.50418701651314,850,
-0.132295188544943,51.5049237415812,900,
-0.1310848318345359,51.50415317469113,700,
-0.1289332780924568,51.50438397334561,900,
-0.1289498203464035,51.50361453996044,950,
-0.129068292374851,51.50334649946367,1000,
-0.1294389249014005,51.50291282735877,1000,
-0.1296158413571358,51.50257116882091,1000,
-0.1296273215960753,51.50231348766234,600,
-0.1265036760831473,51.50219079580769,800,
-0.1259168870410809,51.50219329761568,900,
-0.1259491283343261,51.50318147959608,950,
-0.125819776400552,51.503184896044,800,
-0.1258843403960619,51.50363446579164,750,
-0.1259214654869534,51.50401918117986,700,
-0.1260542809803467,51.50401469189003,600,
-0.1260499425416484,51.5040116478938,500,
-0.1262576451380604,51.50479780571238,0,


};


long checkSensorTimer = millis();

//const char NUM_SAMPLES = 5;
double originLat;
double originLong;


double targetLat;
double targetLong;

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


bool heart1;
bool heart2; 
long heart1timer;
long heart2timer;

//long heartbeat;

long heartbeatTimer;
float heartbeatFreq;

//int heartbeatSpace;
int heartbeatLength;

int targetFreq;
int currentFreq;
float freqInc = 0.05;


//Volume
#define LINE_BUFFER_SIZE  30
char line_buffer[LINE_BUFFER_SIZE];

int maxVol = 204;
int minVol = 0;
int volume = 0;
int targetVol = 204;
//
//int volUpPin = 5;
//int volDownPin = 6;
//int trig0Pin = 9;
//int trig1Pin = 8;
//
//bool volDown = false;
//long volDownTimer;
//bool volUp = false;
//long volUpTimer;
//
//int volDelay = 10;

//end sequence
bool endSeq;
long endSeqTimer;
int endSeqDuration;
bool ended;
bool hasFix = false;


void setup() {

  Serial.begin(115200);

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
 Serial.print("startup ");
  //Compass init
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    //digitalWrite(13, HIGH);
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  bno.setExtCrystalUse(true);
  delay(500);
  
//  //Volume Control
//  pinMode(volUpPin, INPUT);
//  pinMode(volDownPin, INPUT);
//  digitalWrite(volUpPin, HIGH);
//  digitalWrite(volDownPin, HIGH);
//  
//  pinMode(trig0Pin, INPUT);
//  pinMode(trig1Pin, INPUT);
//  volUp = false;
//  volDown = false;
  
  //Heartbeat 
  soundSerial.begin(9600);
  //heart1 = false;
  //heart2 = false;
  
  heartbeatTimer = 0;
  heartbeatFreq = 800;
  heartbeatLength = 170;
  
  //Debug
  //pinMode(13, OUTPUT);
  
  //End Sequence
  endSeq = false;
  endSeqDuration = 60*1000; // one minute
  ended = false;
  
  //Debug
  //originLat = 51;
  //originLong = -0.085971;

  //Get first waypoint
  wayPointIndex = 0;
  getNexWayPoint();

}


void getNexWayPoint() {

  
  if(wayPointIndex < nrWayPoints) {
     //Lat and long are switched on KML google maps output
     //Long is first value, lat is second
     //Reading from SRAM 
     targetLong = pgm_read_float_near(wayPoints + wayPointIndex);
     targetLat = pgm_read_float_near(wayPoints + wayPointIndex + 1);
     targetFreq = pgm_read_float_near(wayPoints + wayPointIndex + 2);
     
     //targetLong = wayPoints[wayPointIndex];
     //targetLat = wayPoints[wayPointIndex + 1];
     //targetFreq = wayPoints[wayPointIndex + 2];
 
    wayPointIndex += 3;
    
    Serial.print("N,");
    Serial.println(wayPointIndex);
    Serial.print("T,");
    Serial.print(targetLat, 6);
    Serial.print(",");
    Serial.println(targetLong, 6);
    
    Serial.print("target freq : ");
    Serial.println(targetFreq);
    Serial.println(heartbeatFreq);
    
    if(targetFreq == 0){
      ended = true;
    }
    //nextWaypointDist = calculateDistance();
    
  } else {
     
     //ended = true;
     wayPointIndex = 0;
     //Serial.println("journey done, start killing king");

  }
    
  

}



void loop() {
  
  
  
  if (checkSensorTimer > millis())  checkSensorTimer = millis();
 
  if(Serial.available() > 0){
      
      //heartbeatFreq = Serial.parseInt();
      //heartbeatSpace = Serial.parseInt();
      
      Serial.read();
      getNexWayPoint();
      
  }
    

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
      hasFix = true;
      
      //if( GPS.latitudeDegrees > 0.0){
        originLat = GPS.latitudeDegrees;
      //}
      
      //if( GPS.longitudeDegrees > 0.0){
        originLong = GPS.longitudeDegrees;
      //}
      
      
      
      if(!debug){
      
      Serial.print("C,");
      Serial.print(originLat, 6);
      Serial.print(",");
      Serial.println(originLong, 6);

      Serial.print("T,");
      Serial.print(targetLat, 6);
      Serial.print(",");
      Serial.println(targetLong, 6);
      }

    } else {
      
    }

    //Compass
    sensors_event_t event;
    bno.getEvent(&event);
    heading = event.orientation.x;

    //Adjust declination
    //float declinationDegree = 14.59;
    //heading += declinationDegree;
    
    bearing = calculateBearing();
    // bearing += declinationDegree;
   
    int diff = abs(bearing - heading);
    diff = abs((diff + 180) % 360 - 180);
    
    float dist = calculateDistance();
    
    //heartbeatFreq = map(dist, nextWaypointDist, 0.01, lastWaypointFreq, nextWaypointFreq);
    //heartbeatFreq = constrain(heartbeatFreq, 500,1300);
    
    if (dist <= 0.02 && wayPointIndex < nrWayPoints) {
        getNexWayPoint();
    }
    
    if(!debug){
      Serial.print("K,"); Serial.println(heading);
      Serial.print("Y,"); Serial.println(bearing);
      Serial.print("D,"); Serial.println(diff);
      Serial.print("V,"); Serial.println(volume);
    }
    
    //This is the accuracy value
    diff = diff * 7 ;
    float amt = map(diff, 0, 180, 204, 150);
    
    
    
    targetVol = amt;
    targetVol = constrain(targetVol, 150, 204);
   // targetVol = 204;
    
    checkSensorTimer = millis(); // reset the timer

  }
  
 
//   while(soundSerial.available()) {
//      soundSerial.read();
//   }
   
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
         Serial.println(volume);
       }
      
   }
   
  }
  
  
  //Play heartbeat
  if( millis() > heartbeatTimer && hasFix && !ended ){
      
      soundSerial.print('#');
      soundSerial.println(1);
      heartbeatTimer = millis() + heartbeatFreq;
      
      if(endSeq){
      
      }
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

