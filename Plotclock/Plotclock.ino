// Plotclock
// cc - by Johannes Heberlein 2014
// v 1.02
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// time library see http://playground.arduino.cc/Code/time 
// RTC  library see http://playground.arduino.cc/Code/time 
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html  
// Change log:
// 1.01  Release by joo at https://github.com/9a/plotclock
// 1.02  Additional features implemented by Dave:
//       - added ability to calibrate servofaktor seperately for left and right servos
//       - added code to support DS1307, DS1337 and DS3231 real time clock chips
//       - see http://www.pjrc.com/teensy/td_libs_DS1307RTC.html for how to hook up the real time clock
//
// 1.03  Remix/mod by Kjetil Egeland
// 1.04 Modified by Alexander Gruhl
//       - added invDrawTo and invSetXY functions to draw in inputted directions on the whiteboard
//       - added cases 14, 15, and 16 in the number function switch statement to draw on whiteboard
//          in different directions

// delete or mark the next line as comment if you don't need these
//#define CALIBRATION      // enable calibration mode
//#define REALTIMECLOCK    // enable real time clock

// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
#define SERVOFAKTORLEFT 820
#define SERVOFAKTORRIGHT 612

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
//#define SERVOLEFTNULL 2250
//#define SERVORIGHTNULL 920
#define SERVOLEFTNULL 2680
#define SERVORIGHTNULL 495
//#define SERVOLEFTNULL 2145
//#define SERVORIGHTNULL 650

#define SERVOPINLIFT  2
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

// lift positions of lifting servo
//#define LIFT0 1080 // on drawing surface
//#define LIFT1 925  // between numbers
//#define LIFT2 725  // going towards sweeper
#define LIFT0 1130  // on drawing surface
#define LIFT1 825  // between numbers
#define LIFT2 605  // going towards sweeper

// speed of liftimg arm, higher is slower
//#define LIFTSPEED 1500
#define LIFTSPEED 3000

// length of arms
#define L1 35
#define L2 55.1 //57.1
#define L3 13.2
#define L4 45

// origin points of left and right servo 
//#define O1X 22
//#define O1Y -25
//#define O2X 47
//#define O2Y -25
#define O1X 21
#define O1Y -25
#define O2X 48
#define O2Y -25

#define PARKX 77
#define PARKY 47
#define ERASEMAXX 60

#include <TimeLib.h> // see http://playground.arduino.cc/Code/time 
#include <Servo.h>

#ifdef REALTIMECLOCK
// for instructions on how to hook up a real time clock,
// see here -> http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// DS1307RTC works with the DS1307, DS1337 and DS3231 real time clock chips.
// Please run the SetTime example to initialize the time on new RTC chips and begin running.

  #include <Wire.h>
  #include <DS1307RTC.h> // see http://playground.arduino.cc/Code/time    
#endif

int servoLift = 1500;

Servo servo1;  // 
Servo servo2;  // 
Servo servo3;  // 

volatile double lastX = 75;
volatile double lastY = PARKY;

int last_min = 0;

int leftUp = 33, leftSide = 110;
int rightUp = 90, rightSide = 1;

void setup() 
{ 
  Serial.begin(9600);
#ifdef REALTIMECLOCK
  //Serial.begin(9600);
  //while (!Serial) { ; } // wait for serial port to connect. Needed for Leonardo only

  // Set current time only the first to values, hh,mm are needed  
  tmElements_t tm;
  if (RTC.read(tm)) 
  {
    setTime(tm.Hour,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year);
    Serial.println("DS1307 time is set OK.");
  } 
  else 
  {
    if (RTC.chipPresent())
    {
      Serial.println("DS1307 is stopped.  Please run the SetTime example to initialize the time and begin running.");
    } 
    else 
    {
      Serial.println("DS1307 read error!  Please check the circuitry.");
    } 
    // Set current time only the first to values, hh,mm are needed
    setTime(15,05,0,0,0,0);
  }
#else  
  // Set current time only the first to values, hh,mm are needed
  setTime(15,35,0,0,0,0);
#endif

  drawTo(PARKX, PARKY);
  lift(2);
  servo1.attach(SERVOPINLIFT);  //  lifting servo
  servo2.attach(SERVOPINLEFT);  //  left servo
  servo3.attach(SERVOPINRIGHT);  //  right servo
  delay(1000);

} 

void loop() 
{ 

#ifdef CALIBRATION

  // Servohorns will have 90° between movements, parallel to x and y axis
  lift(0);
  drawTo(-3, 29.2);
  delay(500);
  drawTo(74.1, 28);
  delay(500);
  /*delay(500);
  lift(2);
  delay(500);
  lift(1);
  delay(500);
  lift(0);
  delay(500);

  //make recalibration for new left 90 and 180, and right 0 and 90
  servo2.write(82); //upright
  servo3.write(0);
  delay(1000);
  servo2.write(180); //pointing left
  servo3.write(90);
  //servo2.write(0); //farthest right
  delay(1000);*/
  

  //drawTo(0, 0);
  lift(1);
  
#else 
  

  int i = 0;
  if (last_min != minute()) {

    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
    
    number(30, 40, 14, 1);
    number(30, 30, 15, 1);
    lift(1);

    i=0;
    while ((i+1)*10 <= minute())
    {
      i++;
    }

    lift(1);
    last_min = minute();

    servo1.detach();
    servo2.detach();
    servo3.detach();
  }

#endif

} 

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void number(float bx, float by, int num, float scale) {

  switch (num) {

  case 0:
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(0);
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    lift(1);
    break;
  case 1:

    drawTo(bx + 3 * scale, by + 15 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(1);
    break;
  case 2:
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    drawTo(bx + 1 * scale, by + 0 * scale);
    drawTo(bx + 12 * scale, by + 0 * scale);
    lift(1);
    break;
  case 3:
    drawTo(bx + 2 * scale, by + 17 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    lift(1);
    break;
  case 4:
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 6 * scale);
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(1);
    break;
  case 5:
    drawTo(bx + 2 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    drawTo(bx + 5 * scale, by + 20 * scale);
    drawTo(bx + 12 * scale, by + 20 * scale);
    lift(1);
    break;
  case 6:
    drawTo(bx + 2 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    drawTo(bx + 11 * scale, by + 20 * scale);
    lift(1);
    break;
  case 7:
    drawTo(bx + 2 * scale, by + 20 * scale);
    lift(0);
    drawTo(bx + 12 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 0);
    lift(1);
    break;
  case 8:
    drawTo(bx + 5 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    lift(1);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawTo(bx + 5 * scale, by + 0);
    lift(1);
    break;

  case 10:
    lift(1);
    drawTo(bx + 3 * scale, by + scale - 15);
    lift(0);
    drawTo(bx + 3 * scale, by + scale);
    lift(1);
    drawTo(bx + 3 * scale, by + scale - 15);
    lift(0);
    drawTo(bx + 30 * scale, by + scale - 13);
    lift(1);
    break;

  case 11:
    drawTo(bx + 5 * scale, by + 15 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    drawTo(bx + 5 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    break;

  case 12:
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    lift(1);
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    lift(1);
    //drawTo(bx + 1 * scale, by + 0 * scale);
    //drawTo(bx + 12 * scale, by + 0 * scale);
    lift(1);
    break;

  case 13:
    drawTo(bx + 0 * scale, by + 0 * scale);
    bogenUZS(bx + 0 * scale, by + 0 * scale, 12 * scale, 2, -5, 1);
    lift(1);
    drawTo(bx + 3 * scale, by + 3 * scale);
    lift(0);
    lift(1);
    drawTo(bx - 6 * scale, by + 3 * scale);
    lift(0);
    lift(1);
    bogenUZS(bx + 0 * scale, by + 0 * scale, 5 * scale, 0, -3, 1);
    lift(1);
    break;

  case 14:
    //lift(1);
    Serial.println("invert");
    invSetXY(bx, by);
    invDrawTo(bx, by, 15, 0, 0);

    lift(1);
    break;

  case 15:
    invSetXY(bx, by);
    invDrawTo(bx, by, 0, 10, 1);
    
    break;
  
  case 16:
    invSetXY(bx, by);
    invDrawTo(bx, by, 15, 6, 2);

    break;
    
  case 111:

    lift(0);
    drawTo(70, 46);
    drawTo(ERASEMAXX, 43);

    drawTo(ERASEMAXX, 49);
    drawTo(5, 49);
    drawTo(5, 45);
    drawTo(ERASEMAXX, 45);
    drawTo(ERASEMAXX, 40);

    drawTo(5, 40);
    drawTo(5, 35);
    drawTo(ERASEMAXX, 35);
    drawTo(ERASEMAXX, 30);

    drawTo(5, 30);
    drawTo(5, 25);
    drawTo(ERASEMAXX, 25);
    drawTo(ERASEMAXX, 20);

    drawTo(5, 20);
    drawTo(60, 44);

    drawTo(PARKX, PARKY);
    lift(2);

    break;

  }
}


void lift(char lift) {
  switch (lift) {
    // room to optimize  !

  case 0: //850

      if (servoLift >= LIFT0) {
      while (servoLift >= LIFT0) 
      {
        servoLift--;
        servo1.writeMicroseconds(servoLift);        
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT0) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }

    }

    break;

  case 1: //150

    if (servoLift >= LIFT1) {
      while (servoLift >= LIFT1) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }
    } 
    else {
      while (servoLift <= LIFT1) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }

    }

    break;

  case 2:

    if (servoLift >= LIFT2) {
      while (servoLift >= LIFT2) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT2) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);        
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;
  }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
    lift(0);
  } 
  while ((start + count) > ende);

}

//in radians
void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;
  
  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) <= ende);
}

//draws a line on the white board from pX and pY to in the direction on incX and incY
//incDirection provides the direction to move
void invDrawTo(double pX, double pY, int incX, int incY, int incDirection){
  double dx, dy;
  Serial.print("pX ");
  Serial.println(pX);
  Serial.print("pY ");
  Serial.println(pY);
  double inc = sqrt(incX * incX + incY * incY); 
  //Serial.println(inc);
  
  switch (incDirection){
    case 0:
      for(double i = 0; i <= incX; i += 1){
        dx = pX + i;
        dy = pY;
        invSetXY(dx, dy);
        lift(0);
      }
      break;
      
    case 1:
      for(double i = 0, j = 0; i <= incY; i += 1, j -= 0.2){
        dx = pX;
        dy = pY + i;
        invSetXY(dx, dy);
        lift(0);
      }
      break;
    case 2:
      for(double i = 0; i < inc; i += 0.1){
        invSetXY(pX + i, pY + i);
      }

      dx = pX;
      dy = pY;

      for(double j = 0; j < inc; j += 0.1){
        invSetXY(dx + j, dx - j);
      }
      break;
    case 3:
      for(double i = 0; i < inc; i += 0.5){
        invSetXY(pX + i, pY - i);
      }
      break;
    case 4:
      for(double i = 0; i < inc; i += 0.5){
        invSetXY(pX - i, pY - i);
      }
      break;
    case 5:
      for(double i = 0; i < inc; i += 0.5){
        invSetXY(pX - i, pY + i);
      }
      break;
  }
  
}

//movement from curent position to new position (change from current pX/pY 
//to pX/pY after adding the double in the for loop parameters)
//calculations in Inverse Kinematics Calculations Photo
void invSetXY(double Tx, double Ty){
  delay(1);
  double B1x, B1y, c, a1, a2, Hx, Hy, L2x, L2y, H2x, H2y;
  double e1, e2, e3;
  double L1Ang1, L1Ang2, M1Ang1, M1Ang2, L1Ang, M1Ang, L1RevAng, L2Ang;
  double dx, dy;

  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;
  
  e1 = -2 * dy * L1;
  e2 = -2 * dx * L1;
  e3 = dx * dx + dy * dy + L1 * L1 - L2 * L2;

  // calculate triangle between pen, servoLeft and arm joint
  // calculate x and y values of each arm
  /*L1x = cos(L1);
  L1y = sin(L1);
  L2x = cos(L2);
  L2y = sin(L2);

  //calculate x and y values of pen holder
  Lex = ra1 + L1x + L2x;
  Ley = L1y + L2y;*/

  //calc L1 angle
  L1Ang1 = 2 * atan ((-e1 + sqrt(e1 * e1 + e2 * e2 - e3 * e3)) / (e3 - e2));
  L1Ang2 = 2 * atan ((-e1 - sqrt(e1 * e1 + e2 * e2 - e3 * e3)) / (e3 - e2));

  //Serial.print("L1ang: " );
  //Serial.println(L1Ang1);
  
  L1RevAng = M_PI - L1Ang1;
  
  //positioning issue for lengths L1 = 35, y increasing for horizontal line
  B1x = O1X - L1 * cos(L1RevAng); //O1x first
  B1y = L1 * sin(L1RevAng) - O1Y;

  if(B1y > 0){
    B1y = O1Y * -1 - L1 * sin(L1RevAng);//positive
  }

  Serial.print("b1x: " );
  Serial.println(B1x);
  Serial.print("b1y: " );
  Serial.println(B1y);

  L2x = Tx - B1x;
  L2y = Ty - B1y;
  L2Ang = atan2(L2y, L2x);

  double L2length = sqrt(L2x * L2x + L2y * L2y);

  H2x = Tx - L3 * cos(L2Ang);
  H2y = Ty - L3 * sin(L2Ang);

  L1Ang1 = floor(L1Ang1 * (180 / M_PI));
  L1Ang2 = L1Ang2 * (180 / M_PI);

  servo2.write(L1Ang1);

  //calc M1 angle  
  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx); 
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = floor(H2x - O2X);
  dy = floor(H2y - O2Y);

  e1 = -2 * dy * L1;
  e2 = -2 * dx * L1;
  e3 = dx * dx + dy * dy + L1 * L1 - L4 * L4;
  
  M1Ang1 = 2 * atan ((-e1 + sqrt(e1 * e1 + e2 * e2 - e3 * e3)) / (e3 - e2));
  M1Ang2 = 2 * atan ((-e1 - sqrt(e1 * e1 + e2 * e2 - e3 * e3)) / (e3 - e2));
  
  M1Ang1 = floor(M1Ang1 * (180 / M_PI));
  M1Ang2 =floor(M1Ang2 * (180 / M_PI));

  servo3.write(M1Ang2);
}

void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;

  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty) 
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

  double dx2 = Tx - O2X;
  double dy2 = Ty - O2Y;

  // calculate joint arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  double a6 = return_angle(L1, L2, c) * (180 / M_PI);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);
  
  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));

}
