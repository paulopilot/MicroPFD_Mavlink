#include "images.h"
#include "icons.h"

/***************************************************************************************
** Function name:           setHomeVars
** Description:             Home Distance and Direction Calculation
*************************************************************************************x*/
void setHomeVars(void)
{
  float dstlon, dstlat;
  long bearing;
  
  if(home.got_home == 0 && apm.fix_type > GPS_NO_FIX){
    home.position.lat = apm.position.lat;
    home.position.lon = apm.position.lon;
    home.position.alt = apm.alt;
    home.got_home = 1;
  } else if(home.got_home == 1){
    // JRChange: home.position.alt: check for stable apm.alt (must be stable for 25*120ms = 3s)
    if(apm.alt < 25){
      if(fabs(alt_prev - apm.alt) > 0.5){
        alt_cnt = 0;
        alt_prev = apm.alt;
      } else {
        if(++alt_cnt >= 25){
          home.position.alt = apm.alt;  // take this stable apm.alt as home.position.alt
        }
      }
    }
    // shrinking factor for longitude going to poles direction
    float rads = fabs(home.position.lat) * 0.0174532925;
    double scaleLongDown = cos(rads);
    double scaleLongUp   = 1.0f/cos(rads);

    //DST to Home
    dstlat = fabs(home.position.lat - apm.position.lat) * 111319.5;
    dstlon = fabs(home.position.lon - apm.position.lon) * 111319.5 * scaleLongDown;
    home.distance = (float)(sqrt(sq(dstlat) + sq(dstlon)));

    //DIR to Home
    dstlon = (home.position.lon - apm.position.lon); //OffSet_X
    dstlat = (home.position.lat - apm.position.lat) * scaleLongUp; //OffSet Y
    bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775); //absolut home direction
    if(bearing < 0) bearing += 360;//normalization
    bearing = bearing - 180;//absolut return direction
    if(bearing < 0) bearing += 360;//normalization
    bearing = bearing - apm.heading;//relative home direction
    if(bearing < 0) bearing += 360; //normalization
    home.direction = round((float)(bearing/360.0f) * 16.0f) + 1;//array of arrows =)
    if(home.direction > 16) home.direction = 0;
    // Calcule ETE in seconds
    home.ete = ((home.distance / 1000) / (apm.groundspeed * converts)) * 3600 ;

    Serial.printf("HOME Dist: %03.0f CRS: %03d DIR: %03d\n", home.distance, (int)bearing, (int)home.direction);
  }
}

void flightMode(uint16_t x, uint16_t y, uint16_t color){
    char* mode_str="";
    if (apm_mav_type == 2){ //ArduCopter MultiRotor or ArduCopter Heli
        if (flight_mode == 0)       mode_str = "STABILIZED"; //Stabilize: hold level position
        else if (flight_mode == 1)  mode_str = "ACRO"; //Acrobatic: rate control
        else if (flight_mode == 2)  mode_str = "ALT HOLD"; //Altitude Hold: auto control
        else if (flight_mode == 3)  mode_str = "AUTO"; //Auto: auto control
        else if (flight_mode == 4)  mode_str = "GUIDED"; //Guided: auto control
        else if (flight_mode == 5)  mode_str = "LOITER"; //Loiter: hold a single location
        else if (flight_mode == 6)  mode_str = "RTL"; //Return to Launch: auto control
        else if (flight_mode == 7)  mode_str = "CIRCLE"; //Circle: auto control
        else if (flight_mode == 8)  mode_str = "POSITION"; //Position: auto control
        else if (flight_mode == 9)  mode_str = "LANDING"; //Land:: auto control
        else if (flight_mode == 10) mode_str = "OF LOITER"; //OF_Loiter: hold a single location using optical flow sensor
        else if (flight_mode == 11) mode_str = "DRIFT"; //Drift mode: 
        else if (flight_mode == 13) mode_str = "SPORT"; //Sport: earth frame rate control
        else if (flight_mode == 14) mode_str = "FLIP"; //Flip: flip the vehicle on the roll axis
        else if (flight_mode == 15) mode_str = "AUTOTUNE"; //Auto Tune: autotune the vehicle's roll and pitch gains
        else if (flight_mode == 16) mode_str = "HYBRID"; //Hybrid: position hold with manual override
    } else if(apm_mav_type == 1){ //ArduPlane
        if (flight_mode == 0)       mode_str = "MANUAL"; //Manual
        else if (flight_mode == 1)  mode_str = "CIRCLE"; //Circle
        else if (flight_mode == 2)  mode_str = "STABILIZED"; //Stabilize
        else if (flight_mode == 3)  mode_str = "TRAINING"; //Training
        else if (flight_mode == 4)  mode_str = "ACRO"; //Acro
        else if (flight_mode == 5)  mode_str = "FBWA"; //Fly_By_Wire_A
        else if (flight_mode == 6)  mode_str = "FBWB"; //Fly_By_Wire_B
        else if (flight_mode == 7)  mode_str = "CRUISE"; //Cruise
        else if (flight_mode == 8)  mode_str = "AUTOTUNE"; //Auto Tune
        else if (flight_mode == 10) mode_str = "AUTO"; //Auto
        else if (flight_mode == 11) mode_str = "RTL"; //Return to Launch
        else if (flight_mode == 12) mode_str = "LOITER"; //Loiter
        else if (flight_mode == 15) mode_str = "GUIDED"; //Guided
        else if (flight_mode == 16) mode_str = "INITIALIZING"; //Initializing
    }
    tft.setCursor(x,y);
    tft.setTextColor(color);
    tft.printf("%s", mode_str);
}

/***************************************************************************************
** Function name:           pushRotated
** Description:             Push a rotated copy of the Sprite to TFT screen
*************************************************************************************x*/
void rotateImage(const unsigned short* image, int16_t x, int16_t y, uint16_t width, uint16_t height, float angle, uint16_t fColor = TFT_TRANSPARENT, uint16_t tColor = TFT_TRANSPARENT) {
  float midX, midY;
  float deltaX, deltaY;
  int rotX, rotY;
  int i, j;

  //uint16_t * buff = (uint16_t *)malloc(width * height * sizeof(uint16_t));

  midX = width / 2.0f;
  midY = height / 2.0f;
  float sa = sin(-angle * DEG_TO_RAD);
  float ca = cos(-angle * DEG_TO_RAD);

  //tft->setAddrWindow(x, y, x + width - 1, y + height - 1);
  for (i = 0; i < width; i++){
    for (j = 0; j < height; j++) {      
     
      deltaX = i - midX;
      deltaY = j - midY;
      
      rotX = (int)(midX + deltaX * sa + deltaY * ca);
      rotY = (int)(midY + deltaX * ca - deltaY * sa);
      if (rotX >= 0 && rotX < width && rotY >= 0 && rotY < height) {
        //buff[(j * width + i)] = pgm_read_word(&image[(rotX * width + rotY)]);
        uint32_t poss = (j * 320 + (y * 320) + (i + x));
        uint16_t pixel = pgm_read_word(&image[(rotX * width + rotY)]);
        if (pixel != tColor) {
          if (fColor == TFT_TRANSPARENT)
            scr[poss] = pixel;
          else
            scr[poss] = fColor;
        }
        //scr[(j * width + i)] = pgm_read_word(&image[(rotX * width + rotY)]);
        //Serial.printf("%3d: 0x%04x\n", (j * width + i), pgm_read_word(&image[(rotX * width + rotY)]));
      }
    }
  }

  //uint32_t x0 = (x % width) + x;
  //uint32_t y0 = (x / width) + y;
  //uint32_t poss = (y0 * H_COLS) + x0;
  //scr[poss] = buff[posf]
  //tft->pushColors(buff, 100*100);
  //free(buff);
}

/*
 * DirIconIndex
 */
uint16_t dirIconIndex(float direction) {
  uint16_t idx = round((direction / 45.0f));
  if (idx > 7) idx = 0;
  return idx;
}

/*
 * drawIcon - Put image from icon
 */
void drawIcon(const unsigned short* image, uint16_t x, uint16_t y, uint16_t index, uint16_t color = 0){
  uint16_t idx = index * icon_S;
  uint16_t ix = 0;
  for (uint32_t i = idx; i < (idx + icon_S); i++) {
    uint32_t x0 = (ix % icon_W) + x;
    uint32_t y0 = (ix / icon_W) + y;
    uint32_t poss = (y0 * H_COLS) + x0;
    uint16_t pix_color = pgm_read_word(&image[i]);
    if (pix_color == TRANSPARENT) 
      pix_color = scr[poss];
    else
      if (color != 0) pix_color = color;
    scr[poss] = pix_color;
    ix++;
  }
} //drawIcon

/*
 * drawArc
 * Draw arc center x and y ray r start angle to end angle and color
 */
void drawArc(int x, int y, int r, int startAngle, int endAngle, uint16_t color) {
  /* original code from Henning Karlsen (http://www.rinkydinkelectronics.com)
    This library is free software; you can redistribute it and/or
    modify it under the terms of the CC BY-NC-SA 3.0 license.
    Please see the included documents for further information.
  */
  uint16_t cx, cy;
  uint32_t poss;
  startAngle -= 90;
  endAngle   -= 90;

  if (startAngle != endAngle) {
    for (int d = startAngle + 1; d < endAngle + 1; d++) {
      cx = x + cos((d * 3.14) / 180) * r;
      cy = y + sin((d * 3.14) / 180) * r;
      poss = cy * H_COLS + cx;
      scr[poss] = color;
      //tft.drawPixel(cx, cy, color);
    }
  } else {
    cx = x + cos((startAngle * 3.14) / 180) * r;
    cy = y + sin((startAngle * 3.14) / 180) * r;
    poss = cy * H_COLS + cx;
    scr[poss] = color;
  }
}

/*
 * Draw image scale
 */
void drawPitchScale(const unsigned short* image, float value, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float factor) {

  value = value + 80;

  uint32_t ix = (uint32_t)(value * factor)*w;

  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y;
    uint32_t posj = ix + i;
    uint32_t poss = y0 * H_COLS + x0;
    uint16_t color = pgm_read_word(&image[posj]);
    if (color != TRANSPARENT) {
      //color = scr[poss];
      scr[poss] = color;
    }
  }
} //drawPitchScale

/*
 * drawHorizon
 * Draw artifitial horizon corn x and y 
 */
void drawHorizon(int x, int y, float pitch, float roll){ 
  
  float lubber, lubberH;
  int i, j;
  int posn;
  float pitchScale = pitch;

  Serial.print("  Pitch=");
  Serial.print(pitch, 1);

  pitch = pitch * 4.4;
  if (pitch > 180) pitch = 180;

  Serial.print("  Pitch2=");
  Serial.print(pitch, 1);

  if (pitch > 180) {
    lubber = 68 + (360 - pitch);
    lubberH = lubber;
  } else {
    lubber = 68 - pitch;
    if (lubber < 8) {
      lubber = 8;
    }
    lubberH = lubber;
    if (lubber > 210) lubber = 210;
    if (lubberH > 190) lubberH = 190;
  }

  Serial.print("  Lubber=");
  Serial.println(lubber, 1);

  // Draw Sky and Ground
  for (i = 0; i < lubberH; i++) {
    for (j = 0; j < H_COLS; j++) {
      posn = (i + x) * H_COLS + (j + y * H_COLS);
      scr[posn] = SKY_COLOR; //0x4CDF;
    }
  }
  
  // Ground
  for (i = lubberH; i < H_ROWS - 10; i++) {
    for (j = 0; j < H_COLS; j++) {
      posn = (i + x) * H_COLS + (j + y * H_COLS);
      scr[posn] = GND_COLOR;
    }
  }

  if (pitchScale < -80) pitchScale = -80;
  if (pitchScale > 80) pitchScale = 80;
  drawPitchScale(pitch_scale, pitchScale, 136, 42, pitch_scale_W, 80, 4.4);
  
  /*
  // Pitch sacle
  //int16_t max_scale = lubber;
  //if (max_scale > 104) max_scale = 104;
  uint16_t k;
  bool inv = true;
  for (i = lubber; i>14; i -= 11) {
    if (inv) k = 16; else k = 8;
    inv = !inv;
    for (j = (H_COLS / 2) - k; j < (H_COLS / 2) + k; j++) {
      posn = (i + x) * H_COLS + (j + y * H_COLS);
      scr[posn] = ILI9341_LIGHTGREY;
      scr[posn + H_COLS] = ILI9341_LIGHTGREY;
    }
  }
  inv = true;
  for (i = lubber; i<(H_ROWS - 104); i += 11) {
    if (inv) k = 16; else k = 8;
    inv = !inv;
    for (j = (H_COLS / 2) - k; j < (H_COLS / 2) + k; j++) {
      posn = (i + x) * H_COLS + (j + y * H_COLS);
      scr[posn] = ILI9341_LIGHTGREY;
      scr[posn + H_COLS] = ILI9341_LIGHTGREY;
    }
  }
  */
  
  // draw horizon line
  posn = 0;
  
  if (lubber > 0 && lubber < 200) {
    for (i = lubber; i < lubber + 1; i++) {
      for (j = 0; j < H_COLS; j++) {
        //posn = i * H_COLS + j;
        posn = (i + x) * H_COLS + (j + y * H_COLS);
        scr[posn] = ILI9341_LIGHTGREY;
        scr[posn + H_COLS] = ILI9341_LIGHTGREY;
      }
    }
  }
} // drawHorizon

/*
 * Draw image scale
 */
void drawScale(const unsigned short* image, float value, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float factor) {

  uint32_t ix = ((614 - h) - (uint32_t)(value * factor))*w;

  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y;
    uint32_t posj = ix + i;
    uint32_t poss = y0 * H_COLS + x0;
    uint16_t color = pgm_read_word(&image[posj]);
    if (color == TFT_TRANSPARENT) {
      uint16_t color2 = scr[poss];
      switch (color2) {
        case SKY_COLOR:
          color = SKY_COLOR_DARK;
          break;
        case GND_COLOR:
          color = GND_COLOR_DARK;
          break;
        case ILI9341_LIGHTGREY:
          color = ILI9341_DARKGREY;
          break;
      }
    }
    scr[poss] = color;
  }
} //drawScale

void drawimage(const unsigned short* image, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fColor = TFT_TRANSPARENT, uint16_t tColor = TFT_TRANSPARENT) {

  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y;
    uint32_t poss = (y0 * H_COLS) + x0;

    uint16_t pixel;
    pixel = pgm_read_word(&image[i]);

    if (pixel != tColor) {
      if (fColor == TFT_TRANSPARENT)
        scr[poss] = pixel;
      else
        scr[poss] = fColor;
    }
  }
} //drawimage


void drawScreen(){
  // draw screen
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.drawFastHLine(0, 18, 320, ILI9341_DARKGREY);
  tft.drawFastVLine(66, 0, 17, ILI9341_DARKGREY);
  tft.drawFastVLine(252, 0, 17, ILI9341_DARKGREY);
  tft.drawFastHLine(66, 8, 186, ILI9341_DARKGREY);
  //tft.drawFastHLine(0, 232, 320, ILI9341_WHITE);

  tft.setCursor(0,0);
  tft.println("COM1 129.00");
  tft.setCursor(254,0);
  tft.println("NAV1 114.10");
  tft.setCursor(0,10);
  tft.println("COM2 133.00");
  tft.setCursor(254,10);
  tft.println("NAV2 109.30");

  drawIcon(icons, 68, 0, ICO_HOME);
  drawIcon(icons, 207, 0, ICO_TIME);
  drawIcon(icons, 68, 10, ICO_WAYPOINT);
  drawIcon(icons, 109, 10, ICO_EAST);
  drawIcon(icons, 207, 10, ICO_TIME);
} // drawScreen


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
