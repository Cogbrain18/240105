// Uncomment this line to activate an advanced mouse mode
// Need to install AdvMouse library (copy /library/AdvMouse to the Arduino libraries)
// AdvMouse module: https://github.com/SunjunKim/PMW3360_Arduino/tree/master/library/AdvMouse
#define ADVANCE_MODE

#include <SPI.h>
#include <avr/pgmspace.h>
#include "PMW3389.h"
#include "GainFunction.h" 
#ifdef ADVANCE_MODE
#include "AdvMouse.h"
#define MOUSE_BEGIN       AdvMouse.begin()
#define MOUSE_PRESS(x)    AdvMouse.press_(x)
#define MOUSE_RELEASE(x)  AdvMouse.release_(x)
#else
#include <Mouse.h>
#define MOUSE_BEGIN       Mouse.begin()
#define MOUSE_PRESS(x)    Mouse.press(x)
#define MOUSE_RELEASE(x)  Mouse.release(x)
#endif

// User define values
#define DEFAULT_CPI  800
#define SENSOR_DISTANCE 72  // in mm
//#define DEFAULT_GF

#define MAX_CPI  16000

#define SS1  10          // Slave Select pin. Connect this to SS on the module. (Front sensor)
#define SS2  9         // Slave Select pin. Connect this to SS on the module. (Rear sensor)
#define NUMBTN 2        // number of buttons attached
#define BTN1 4          // left button pin
#define BTN2 8          // right button pin
#define DEBOUNCE  10    // debounce itme in ms. Minimun time required for a button to be stabilized.

int btn_pins[NUMBTN] = { BTN1, BTN2 };
char btn_keys[NUMBTN] = { MOUSE_LEFT, MOUSE_RIGHT };

// Don't need touch below.
PMW3389 sensor1, sensor2;
GainFunction GainFn;
int posRatio = 50;      // located at 55% (centerd but slightly at the front side)

// button pins & debounce buffers
bool btn_state[NUMBTN] = { false, false };
uint8_t btn_buffers[NUMBTN] = {0xFF, 0xFF};

// internal variables
unsigned long lastTS;
unsigned long lastButtonCheck = 0;

float remain_dx, remain_dy;

bool reportSQ = false;  // report surface quality
bool lastNA = false;

float sensor_dist_inch = (float)SENSOR_DISTANCE / 25.4;
int current_cpi = DEFAULT_CPI;
// float current_gf = DEFAULT_GF;
float cpi_divider = (float)MAX_CPI / DEFAULT_CPI;

void setup() {
  Serial.begin(9600);
  //while(!Serial);
  //sensor.begin(10, 1600); // to set CPI (Count per Inch), pass it as the second parameter

  sensor2.begin(SS2);
  sensor1.begin(SS1);
  delay(250);

  if (sensor1.begin(SS1)) // 10 is the pin connected to SS of the module.
    Serial.println("Sensor1 initialization successed");
  else
    Serial.println("Sensor1 initialization failed");

  if (sensor2.begin(SS2)) // 10 is the pin connected to SS of the module.
    Serial.println("Sensor2 initialization successed");
  else
    Serial.println("Sensor2 initialization failed");

  sensor1.setCPI(MAX_CPI);    // or, you can set CPI later by calling setCPI();
  sensor2.setCPI(MAX_CPI);    // or, you can set CPI later by calling setCPI();

  // sensor1.writeReg(REG_Control, 0b11000000); // turn 90 deg configuration
  // sensor2.writeReg(REG_Control, 0b11000000);

  // sensor1.writeReg(REG_Lift_Config, 0b11);  // set lift detection height = norminal height + 3 mm
  // sensor2.writeReg(REG_Lift_Config, 0b11);

  int cpi1 = sensor1.getCPI();
  int cpi2 = sensor2.getCPI();

  if(cpi1 != cpi2 || cpi1 != MAX_CPI)
    Serial.println("WARNING: CPI initialization failed");

  remain_dx = remain_dy = 0.0;

  setSoftCPI(DEFAULT_CPI);

  MOUSE_BEGIN;
  buttons_init();

  // clear the previous buffers
  lastTS = micros();
  sensor1.readBurst();
  sensor2.readBurst();
}


bool sent = false;
void loop() {
  unsigned long currentTS = micros();
  unsigned long elapsed = currentTS - lastTS;

  check_buttons_state();

  if (elapsed > 1993)
  {
    lastTS = currentTS;

    PMW3389_DATA data1 = sensor1.readBurst();
    PMW3389_DATA data2 = sensor2.readBurst();

    // rotate data1's dx, dy by -90 deg
    int tmp = data1.dx;
    data1.dx = data1.dy;
    data1.dy = -tmp;
    // rotate data1's dx, dy by 90 deg
    tmp = data2.dx;
    data2.dx = -data2.dy;
    data2.dy = tmp;


    PMW3389_DATA data;

    data.isOnSurface = data1.isOnSurface && data2.isOnSurface;
    data.isMotion = data1.isMotion || data2.isMotion;

    float rat = posRatio / 100.0;

    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    float misalignment_angle = -0.231;
    float realignment_angle = misalignment_angle * PI / 180;

    float aligned_dx = data2.dx * cos(realignment_angle) - data2.dy * sin(realignment_angle);
    float aligned_dy = data2.dx * sin(realignment_angle) + data2.dy * cos(realignment_angle);

    data2.dx = aligned_dx;
    data2.dy = aligned_dy;
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////

    // dx and dy are calculated in MAX_CPI
    float dx = data1.dx * (1.0-rat) + data2.dx * (rat) + remain_dx;
    float dy = (data1.dy + data2.dy) / 2.0 + remain_dy;

    data.isMotion = (dx != 0) || (dy != 0);


    // divide down to the soft CPI
    int mdx = int(dx / cpi_divider);
    int mdy = int(dy / cpi_divider);

    remain_dx = dx - mdx * cpi_divider;
    remain_dy = dy - mdy * cpi_divider;


    ///////////////////////////////////
    mdx = GainFn.apply_gain_function(mdx)*mdx;
    mdy = GainFn.apply_gain_function(mdy)*mdy;
    ///////////////////////////////////

    data.dx = mdx;
    data.dy = mdy;

    data.SQUAL = (data1.SQUAL + data2.SQUAL) / 2;

    //data.dx = data1.dx + data2.dx;
    //data.dy = data1.dy + data2.dy;

    bool moved = data1.dx != 0 || data1.dy != 0 || data2.dx != 0 || data2.dy != 0;

    #ifdef ADVANCE_MODE
    moved = moved || AdvMouse.needSendReport();
    #endif

    byte btn_report = btn_state[0] + (btn_state[1] << 1);


    if(moved)
    {
      Serial.print(micros());
      Serial.print('\t');

      if(data1.isOnSurface)
      {
        Serial.print(data1.dx);
        Serial.print('\t');
        Serial.print(data1.dy);
      }
      else
      {
        Serial.print("NA\tNA");
      }

      Serial.print('\t');

      if(data2.isOnSurface)
      {
        Serial.print(data2.dx);
        Serial.print('\t');
        Serial.print(data2.dy);
      }
      else
      {
        Serial.print("NA\tNA");
      }
      Serial.print('\t');

      Serial.print(data.dx);
      Serial.print('\t');
      Serial.print(data.dy);

      Serial.print('\t');
      Serial.println(btn_report);
      lastNA = false;
    }
    else if(!data.isOnSurface && !lastNA)
    {
      Serial.print(micros());
      Serial.println("\tNA\tNA\tNA\tNA\tNA\tNA\tNA");
      lastNA = true;
    }
  ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    else
    {
      Serial.print(micros());
      Serial.print('\t');
      Serial.print(data1.dx);
      Serial.print('\t');
      Serial.print(data1.dy);
      Serial.print('\t');
      Serial.print(data2.dx);
      Serial.print('\t');
      Serial.print(data2.dy);
      Serial.print('\t');
      Serial.print(data.dx);
      Serial.print('\t');
      Serial.print(data.dy);
      Serial.print('\t');
      Serial.println(-1);
    }
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////

#ifdef ADVANCE_MODE
    if (AdvMouse.needSendReport() || (data.isOnSurface && moved))
    {
      //if (AdvMouse.needSendReport() && !data.isMotion)
      //  Serial.println("Btn report");
      AdvMouse.move(data.dx, data.dy, 0);
    }
#else
    if (data.isOnSurface && moved)
    {
      signed char mdx = constrain(data.dx, -127, 127);
      signed char mdy = constrain(data.dy, -127, 127);

      Mouse.move(mdx, mdy, 0);
    }
#endif

    if (reportSQ && data.isOnSurface) // print surface quality
    {
      Serial.println(data.SQUAL);
    }
  }


  // command process routine
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == 'Q') // Toggle reporting surface quality
    {
      reportSQ = !reportSQ;
    }
    else if (c == 'c') // report current CPI
    {
      Serial.print("Current CPI: ");
      Serial.println(current_cpi);
      Serial.println(sensor1.getCPI());
      Serial.println(sensor2.getCPI());
      delay(5000);
    }
    else if (c == 'C')   // set CPI
    {
      int newCPI = readNumber();

      setSoftCPI(newCPI);
      Serial.print(current_cpi);
      Serial.print('\t');
      Serial.println(cpi_divider);
    }
    
    /////////////////////////////////////////////
    /////////////////////////////////////////////  
    else if (c == 'g') // report current gain function
    {
      Serial.print("Current Gain Function: ");
      Serial.println(GainFn.getGainFn());
      delay(5000);
    }

    
    else if (c == 'G')   // update new gain function
    {
      Serial.println("Insert the order of gain function");
      int orderGainFn = readNumber() + 1;
      float* coefficients = new float[orderGainFn];
      
      Serial.println("Insert the Coefficient of gain function by descending order");
      Serial.println("ex. If gain function is 3.1x^3 + 7.4x + 1.2, insert 3.1, 0, 7.4, 1.2 and press enter");
      readCoefficients(coefficients, orderGainFn);
      GainFn.setGainFn(coefficients, orderGainFn);

      delete[] coefficients;
    }
    /////////////////////////////////////////////
    /////////////////////////////////////////////
    
    else if (c == 'P')  // set sensor position
    {
      int newPos = readNumber();
      posRatio = constrain(newPos, 0, 100);
      Serial.println(posRatio);
    }
    else if(c == 'R') // report current values
    {
      Serial.print("Pos:\t");
      Serial.println(posRatio);
      Serial.print("CPI:\t");
      Serial.println(current_cpi);
    }
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    else if(c == 'I')
    {
      Serial.println("yes");
    }
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
  }
}

// s1_dx, s1_dy => sensor 1 (=front) (dx, dy)
// s2_dx, s2_dy => sensor 2 (=rear) (dx, dy)
// current_cpi  =>
void translate_virtual_sensor(int s1_dx, int s1_dy, int s2_dx, int s2_dy, float weight_x, float weight_y, float &vs_pos_x, float &vs_pos_y)
{
  int vx = s2_dx - s1_dx;
  int vy = s2_dy - s1_dy;

  float d = sensor_dist_inch * (float)current_cpi; // inter-sensor distance in counts

  float theta = (float)vx / d; // value is in radian.

  float cos_t = cos(theta);
  float sin_t = sin(theta);

  float sa_x = d * weight_x;
  float sa_y = d * weight_y;

  vs_pos_x = (float)s2_dx + cos_t*sa_x - sin_t*sa_y - sa_x;
  vs_pos_y = (float)s2_dy + sin_t*sa_x + cos_t*sa_y - sa_y;
}

void buttons_init()
{
  for (int i = 0; i < NUMBTN; i++)
  {
    pinMode(btn_pins[i], INPUT_PULLUP);
  }
}

void setSoftCPI(int CPI)
{
  current_cpi = constrain(CPI, 100, 12000);
  cpi_divider = (float)MAX_CPI / current_cpi;
}

/*
void set


*/
// Button state checkup routine, fast debounce is implemented.
void check_buttons_state()
{
  unsigned long elapsed = micros() - lastButtonCheck;

  // Update at a period of 1/8 of the DEBOUNCE time
  if (elapsed < (DEBOUNCE * 1000UL / 8))
    return;

  lastButtonCheck = micros();

  // Fast Debounce (works with mimimal latency most of the time)
  for (int i = 0; i < NUMBTN ; i++)
  {
    int state = digitalRead(btn_pins[i]);
    btn_buffers[i] = btn_buffers[i] << 1 | state;

    // btn_buffer detects 0 when the switch shorts, 1 when opens.
    if (btn_state[i] == false &&
        (btn_buffers[i] == 0xFE || btn_buffers[i] == 0x00) )
        // 0xFE = 0b1111:1110 button pressed for the first time (for fast press detection w. minimum debouncing time)
        // 0x00 = 0b0000:0000 force press when consequent on state (for the DEBOUNCE time) is detected
    {
      MOUSE_PRESS(btn_keys[i]);
      btn_state[i] = true;
    }
    else if ( btn_state[i] == true &&
              (btn_buffers[i] == 0x07 || btn_buffers[i] == 0xFF) )
              // 0x07 = 0b0000:0111 button released consequently 3 times after stabilized press (not as fast as press to prevent accidental releasing during drag)
              // 0xFF = 0b1111:1111 force release when consequent off state (for the DEBOUNCE time) is detected
    {
      MOUSE_RELEASE(btn_keys[i]);
      btn_state[i] = false;
    }
  }
}


unsigned long readNumber()
{
  String inString = "";
  for (int i = 0; i < 10; i++)
  {
    while (Serial.available() == 0);
    int inChar = Serial.read();
    if (isDigit(inChar))
    {
      inString += (char)inChar;
    }

    if (inChar == '\n')
    {
      int val = inString.toInt();
      return (unsigned long)val;
    }
  }

  // flush remain strings in serial buffer
  while (Serial.available() > 0)
  {
    Serial.read();
  }
  return 0UL;
}


void readCoefficients(float* coefficients, int order) 
{
  float inNumber;
  for (int i = 0; i < order; i++)
  {
    while (Serial.available() == 0);
    inNumber = Serial.parseFloat();
    coefficients[i] = inNumber;
  }

  // flush remaining strings in serial buffer
  while (Serial.available() > 0)
  {
    Serial.read();
  }
}

//String readCoefficient() {
//    String inString = "";
//    for (int i = 0; i < 10; i++) 
//    {
//        while (Serial.available() == 0); // Wait for input
//        int inChar = Serial.read();
//        if (isDigit(inChar)) 
//        {
//            inString += (char)inChar; // Append digit to string
//        }
//        if (inChar == '\n') 
//        {
//            return inString; 

//        }
//    }
//
//    // Flush remaining strings in serial buffer
//    while (Serial.available() > 0) {
//        Serial.read();
//    }
//
//    return "1.0f";
//}

