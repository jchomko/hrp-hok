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
Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Waypoints
int wayPointIndex = 0;
const int nrWayPoints = 165;
const static float PROGMEM wayPoints[nrWayPoints] = {
-0.1368907070722702,51.50374231398688,1000,
-0.1367733353700928,51.50367268553593,1000,
-0.1367425024225077,51.5036574418566,1000,
-0.1367003347890328,51.5036728657903,1000,
-0.1364694652490672,51.50381624422109,400,
-0.1364359452045472,51.50383462708184,850,
-0.1363882397439953,51.50381226001926,700,
-0.1363524646521663,51.50377790967539,750,
-0.1361119171621994,51.5035727909398,800,
-0.1360862380983341,51.50355489334942,850,
-0.1360588579583388,51.50354705825301,900,
-0.1360208963924181,51.50355873084585,900,
-0.1359826066403191,51.50358781567201,900,
-0.1359267105278905,51.50361866224996,950,
-0.1358868707091665,51.50361679349424,1000,
-0.1358559615956079,51.50358472257599,1000,
-0.1358030822540124,51.50354731642867,1000,
-0.1357594961885256,51.50353456997557,950,
-0.1357170021015863,51.503538323179,800,
-0.135675205626905,51.50356402938309,750,
-0.1352453948709686,51.5037516245444,600,
-0.1341916689411571,51.50418701651314,550,
-0.1323501354193735,51.50493145975183,500,
-0.1310414509494418,51.50426770678978,600,
-0.1303933881231789,51.50421911932911,900,
-0.129813774387002,51.50426897575026,1000,
-0.1291784612517122,51.50439304386552,900,
-0.1290078105104542,51.50390374968751,900,
-0.1289612997011602,51.50360945261307,900,
-0.129070044928693,51.50334283713119,900,
-0.1292266390515118,51.50310195341267,900,
-0.1294451022298193,51.50285377759403,900,
-0.1295667163240244,51.50263808926961,900,
-0.1296747989329772,51.50243774321953,900,
-0.129644994954532,51.50236011136204,900,
-0.1295487832499087,51.50231911298264,900,
-0.1293460039676431,51.50231885972333,900,
-0.1286061434920183,51.50228489071829,900,
-0.1279392948089653,51.50223803680529,900,
-0.1272816983945735,51.50220307345162,900,
-0.126735034385923,51.50217771651002,900,
-0.1264403913947676,51.50216398125168,900,
-0.1262421901156885,51.50219749134006,900,
-0.1261984331228294,51.50219900398593,900,
-0.1261207369583872,51.50219750477234,900,
-0.126047529621035,51.50219361405294,900,
-0.1259749330153581,51.50219317178686,900,
-0.1259168870410809,51.50219329761568,900,
-0.1259349871937576,51.50312115441,900,
-0.1259673390933869,51.50332418951741,900,
-0.1260014907537166,51.50365414233244,900,
-0.126019595090846,51.50392856932151,900,
-0.1260495320705213,51.50423119462774,900,
-0.126104680394199,51.50438329518128,900,
-0.1261526192758933,51.504551885458,500,
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
int maxVol = 100;
int minVol = 0;
int volume = 0;
int targetVol = 100;

int volUpPin = 5;
int volDownPin = 6;
int trig0Pin = 9;
int trig1Pin = 8;

bool volDown = false;
long volDownTimer;
bool volUp = false;
long volUpTimer;

int volDelay = 10;

//end sequence
bool endSeq;
long endSeqTimer;
int endSeqDuration;
bool ended;


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
  
  //Volume Control
  pinMode(volUpPin, INPUT);
  pinMode(volDownPin, INPUT);
  digitalWrite(volUpPin, HIGH);
  digitalWrite(volDownPin, HIGH);
  
  pinMode(trig0Pin, INPUT);
  pinMode(trig1Pin, INPUT);
  volUp = false;
  volDown = false;
  
  //Heartbeat 
  heart1 = false;
  heart2 = false;
  
  heartbeatTimer = 0;
  heartbeatFreq = 800;
  heartbeatLength = 170;
  
  //Debug
  pinMode(13, OUTPUT);
  
  //End Sequence
  endSeq = false;
  endSeqDuration = 60*1000; // one minute
  ended = false;
  
  //Debug
  originLat = 51;
  originLong = -0.085971;

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
    //nextWaypointDist = calculateDistance();
    
  } else {
     
     //ended = true;
     wayPointIndex = 0;
     //Serial.println("journey done, start killing king");

  }
    
  

}



void loop() {

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  //if millis() or timer wraps around, we'll just reset it
 // if (checkSensorTimer > millis())  checkSensorTimer = millis();
 
  if(Serial.available() > 0){
      
      //heartbeatFreq = Serial.parseInt();
      //heartbeatSpace = Serial.parseInt();
      //targetVol =
//      int r = Serial.parseInt();
//      if(r != 0){
//        targetVol = r;
//      }
//      
//      r = Serial.parseInt();
//      if(r != 0){
//        volDelay = r;
//      }
//      
      // int heartFreq = Serial.parseInt();
      //Serial.println(heartbeatFreq);
      // Serial.println(heartbeatSpace);
//      Serial.println(targetVol);
//      Serial.println(volDelay);
      Serial.read();
      getNexWayPoint();
      
  }
  
//  
  //might make it easier to find the right direction
  float diff = map(targetVol, 50,100, 200,0);
  
  if(heartbeatFreq > targetFreq - diff){
    heartbeatFreq -= freqInc;
    //Serial.println(heartbeatFreq);
  }
  
  if(heartbeatFreq < targetFreq -diff){
    heartbeatFreq += freqInc;
    //Serial.println(heartbeatFreq);
  }
  
  heartbeatFreq = constrain(heartbeatFreq, 200,1400);
  
  //check gps and compass
  if (millis() - checkSensorTimer > 200) {
    
    if (GPS.fix) {
      
      originLat = GPS.latitudeDegrees;
      originLong = GPS.longitudeDegrees;
      
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
    
    if (dist <= 0.01 && wayPointIndex < nrWayPoints) {
        getNexWayPoint();
    }
    
    if(!debug){
      Serial.print("K,"); Serial.println(heading);
      Serial.print("Y,"); Serial.println(bearing);
      Serial.print("D,"); Serial.println(diff);
      Serial.print("V,"); Serial.println(volume);
    }
    
    //This is the accuracy value
    diff = diff * 6 ;
    float amt = map(diff, 0, 180, 100, 50);
    
    
    
    targetVol = amt;
    targetVol = constrain(targetVol, 50, 100);
    
    checkSensorTimer = millis(); // reset the timer

}

  
  //This only needs to be very short
  int sig = volDelay;
  
  if(targetVol < volume){
    pinMode(volDownPin, OUTPUT);
    delay(sig);
    pinMode(volDownPin, INPUT);
    delay(sig);
    volume --;
  }
  
  if(targetVol > volume ){
    pinMode(volUpPin, OUTPUT);
    delay(sig);
    pinMode(volUpPin, INPUT);
    delay(sig);   
    volume ++;
  }
  
  if(targetVol > 95){
    pinMode(volUpPin, OUTPUT);
    delay(sig);
    pinMode(volUpPin, INPUT);
    delay(sig);   
    volume ++;
  }
  
//  if( targetVol < volume && !volDown && !volUp && millis() - volDownTimer >  sig*2 ){ //&& millis()- volDownTimer > 50 
//      volume --;
//      volDown = true; 
//      volDownTimer = millis();
//      //digitalWrite(volDownPin, LOW);
//      pinMode(volDownPin, OUTPUT);
//      Serial.println(volume);
//  }
//  
//  if(volDown && millis() - volDownTimer > sig){
//      pinMode(volDownPin, INPUT);
//      //digitalWrite(volDownPin, HIGH);
//
//      volDown = false;
//      //Serial.println(volume);
//  }
//  
//  
//  if( targetVol > volume && !volUp && !volDown & millis()- volUpTimer > sig*2 ) { //
//      volume ++;
//      volUp = true;
//      volUpTimer = millis();
//      //digitalWrite(volUpPin, LOW);
//      pinMode(volUpPin, OUTPUT);
//      Serial.println(volume);
//  }
//  
//  if( volUp && millis() - volUpTimer > sig ){
//       
//      pinMode(volUpPin, INPUT);
//       //pinMode(volUpPin, OUTPUT);
//       
//       //digitalWrite(volUpPin, HIGH);
//       
//       volUp = false;
//       //Serial.println(volume);
//  }
//  
  if( millis() > heartbeatTimer){
       
       int triggerTimer = heartbeatLength;
       int interval = map(heartbeatFreq,230,1000,50,110);
 
       if(heart1 == false &&  millis() > heartbeatTimer  ){
          pinMode(trig0Pin, OUTPUT);
          heart1 = true;
          //Serial.println("heart 1 start ");
       }
       
       if(heart1 == true &&  millis() - heartbeatTimer >  triggerTimer){
          pinMode(trig0Pin, INPUT);
          //heart1 = false;
          //Serial.println("heart 1 stop ");
       }
       
       if( millis() - heartbeatTimer > triggerTimer + interval && heart2 == false){
          heart2 = true;
          pinMode(trig1Pin, OUTPUT);
          //Serial.println("heart 2 start ");
       }
       
       if( millis() - heartbeatTimer >  (triggerTimer * 2 ) + interval && heart2 == true){
          pinMode(trig1Pin, INPUT);
          //Serial.println("heart 2 stop");
          heartbeatTimer = millis() + heartbeatFreq;
          heart1 = false;
          heart2 = false;
       }
       
    }
    
    if(endSeq){
      
      
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

