#include "images.h"
#include "icons.h"

/***************************************************************************************
** Function name:           setHomeVars
** Description:             Home Distance and Direction Calculation
*************************************************************************************x*/
void setHomeVars(void)
{
  float dstlon, dstlat;
  float bearing;
  
  if(home.got_home == 0 && apm.fix_type > GPS_FIX_TYPE_NO_FIX){
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
    //float rads = fabs(home.position.lat) * 0.0174532925; 
    float rads = fabs(home.position.lat) * DEG_TO_RAD;
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
    if(bearing < 0) bearing += 360.0f;//normalization
    bearing = bearing - 180.0f;//absolut return direction
    if(bearing < 0) bearing += 360.0f;//normalization
    //bearing = bearing - apm.heading;//relative home direction
    //if(bearing < 0) bearing += 360.0f; //normalization
    //home.direction = round((float)(bearing/360.0f) * 16.0f) + 1;//array of arrows =)
    home.direction = bearing;//array of arrows =)
    //if(home.direction > 16) home.direction = 0;
    // Calcule ETE in seconds
    home.ete = ((home.distance / 1000) / (apm.groundspeed * converts)) * 3600 ;

    //Serial.printf("Distance:%03.0f Crs:%03d Dir:%03d GS:%d ETE:%d\n", home.distance, (int)bearing, (int)home.direction, (int)(apm.groundspeed * converts), (int)home.ete);
  }
}

void flightMode(uint16_t x, uint16_t y, uint16_t color){
    char* mode_str="";
    if (apm_mav_type == 2){ //ArduCopter MultiRotor or ArduCopter Heli
        if (flight_mode < 0)        mode_str = "";
        else if (flight_mode == 0)  mode_str = " STABILIZED "; //Stabilize: hold level position
        else if (flight_mode == 1)  mode_str = "    ACRO    "; //Acrobatic: rate control
        else if (flight_mode == 2)  mode_str = "  ALT HOLD  "; //Altitude Hold: auto control
        else if (flight_mode == 3)  mode_str = "    AUTO    "; //Auto: auto control
        else if (flight_mode == 4)  mode_str = "   GUIDED   "; //Guided: auto control
        else if (flight_mode == 5)  mode_str = "   LOITER   "; //Loiter: hold a single location
        else if (flight_mode == 6)  mode_str = "     RTL    "; //Return to Launch: auto control
        else if (flight_mode == 7)  mode_str = "   CIRCLE   "; //Circle: auto control
        else if (flight_mode == 8)  mode_str = "  POSITION  "; //Position: auto control
        else if (flight_mode == 9)  mode_str = "  LANDING   "; //Land:: auto control
        else if (flight_mode == 10) mode_str = "  OF LOITER "; //OF_Loiter: hold a single location using optical flow sensor
        else if (flight_mode == 11) mode_str = "    DRIFT   "; //Drift mode: 
        else if (flight_mode == 13) mode_str = "    SPORT   "; //Sport: earth frame rate control
        else if (flight_mode == 14) mode_str = "    FLIP    "; //Flip: flip the vehicle on the roll axis
        else if (flight_mode == 15) mode_str = "  AUTOTUNE  "; //Auto Tune: autotune the vehicle's roll and pitch gains
        else if (flight_mode == 16) mode_str = "   HYBRID   "; //Hybrid: position hold with manual override
    } else if(apm_mav_type == 1){ //ArduPlane
        if (flight_mode < 0)        mode_str = "";
        else if (flight_mode == 0)  mode_str = "   MANUAL   "; //Manual
        else if (flight_mode == 1)  mode_str = "   CIRCLE   "; //Circle
        else if (flight_mode == 2)  mode_str = " STABILIZED "; //Stabilize
        else if (flight_mode == 3)  mode_str = "  TRAINING  "; //Training
        else if (flight_mode == 4)  mode_str = "    ACRO    "; //Acro
        else if (flight_mode == 5)  mode_str = "    FBWA    "; //Fly_By_Wire_A
        else if (flight_mode == 6)  mode_str = "    FBWB    "; //Fly_By_Wire_B
        else if (flight_mode == 7)  mode_str = "   CRUISE   "; //Cruise
        else if (flight_mode == 8)  mode_str = "  AUTOTUNE  "; //Auto Tune
        else if (flight_mode == 10) mode_str = "    AUTO    "; //Auto
        else if (flight_mode == 11) mode_str = "    RTL     "; //Return to Launch
        else if (flight_mode == 12) mode_str = "   LOITER   "; //Loiter
        else if (flight_mode == 15) mode_str = "   GUIDED   "; //Guided
        else if (flight_mode == 16) mode_str = "INITIALIZING"; //Initializing
    }
    pfd.setCursor(x,y);
    pfd.setTextColor(color, ILI9341_BLACK);
    pfd.printf("%s", mode_str);
}

/***************************************************************************************
** Function name:           pushRotated
** Description:             Push a rotated copy of the Sprite to TFT screen
*************************************************************************************x*/
void rotateImage(const unsigned short* image, int16_t x, int16_t y, uint16_t width, uint16_t height, float angle, uint16_t fColor = TFT_OPAQUE, uint16_t tColor = TFT_OPAQUE) {
  float midX, midY;
  float deltaX, deltaY;
  int rotX, rotY;
  int i, j;

  midX = width / 2.0f;
  midY = height / 2.0f;
  
  float sa = sin(-angle * DEG_TO_RAD);
  float ca = cos(-angle * DEG_TO_RAD);

  //pfd->setAddrWindow(x, y, x + width - 1, y + height - 1);
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
          if (fColor == TFT_OPAQUE)
            scr[poss] = pixel;
          else
            scr[poss] = fColor;
        }
      }
    }
  }
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
void drawIcon(const unsigned short* image, uint16_t x, uint16_t y, uint16_t index, uint16_t color = 0, uint16_t bgcolor = TFT_TRANSPARENT){
  uint16_t idx = index * icon_S;
  uint16_t ix = 0;
  for (uint32_t i = idx; i < (idx + icon_S); i++) {
    uint32_t x0 = (ix % icon_W) + x;
    uint32_t y0 = (ix / icon_W) + y;
    uint32_t poss = (y0 * H_COLS) + x0;
    uint16_t pix_color = pgm_read_word(&image[i]);
    if (pix_color == TFT_TRANSPARENT){ 
      if (bgcolor == TFT_TRANSPARENT){
        pix_color = scr[poss];
      }else{
        pix_color = bgcolor;
      }
    }else{
      if (color != 0) pix_color = color;
    }
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
      //pfd.drawPixel(cx, cy, color);
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
void drawPitchScale(const unsigned short* image, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float factor, AHRS *ahrs) {  

  float pitch = ahrs->pitch;
  float roll = ahrs->roll;
  
  if (pitch < -80) pitch = -80;
  if (pitch > 80) pitch = 80;
  pitch = pitch + 80;
  int32_t ix = (int32_t)(pitch * factor)*w;

  int32_t px = x + w/2;
  int32_t py = y + h/2;

  for (uint32_t i = 0; i < (w * h); i++) {
    int32_t x0 = (i % w) + x;
    int32_t y0 = (i / w) + y;

    int32_t x1 = ahrs->cosRoll * (x0 - px) - ahrs->sinRoll * (y0 - py) + px;
    int32_t y1 = ahrs->sinRoll * (x0 - px) + ahrs->cosRoll * (y0 - py) + py;

    if (x1 < 0) x1=0;
    if (x1 > 320) x1=320;
    if (y1 < 0) y1=0;
    if (y1 > 240) y1=240;
    
    int32_t posj = ix + i;
    int32_t poss = y1 * H_COLS + x1;
    uint16_t color = pgm_read_word(&image[posj]);
    if (color != TFT_TRANSPARENT) {
      scr[poss] = color;
    }
  }
} //drawPitchScale

/*
 * drawBackGround
 * Draw artifitial horizon corn x and y 
 */
//void drawBackGround(float roll, float pitch) {
void drawBackGround(AHRS *ahrs) {
  int32_t oldyf1 = -1;
  int32_t oldyf2 = -1;
  float roll = ahrs->roll;
  float pitch = -ahrs->pitch * 4.4;
  if (pitch > 180) pitch = 180;
  if (pitch < -180) pitch = -180;
  
  float ta = tan(roll * DEG_TO_RAD);
  
  int32_t xc = X_PIVOT;
  if (pitch < -60) pitch = -60;
  if (pitch > 120) pitch = 120;
  int32_t yc = pitch + Y_PIVOT;
  if (yc < YMIN) yc = YMIN;
  if (yc > YMAX) yc = YMAX;
  int32_t x1 = XMIN;
  int32_t y1 = yc;
  int32_t x2 = XMAX;
  int32_t y2 = yc;
  int32_t cadj = xc - x1;
  int32_t co = ta * cadj;
  int32_t yf1 = y1 - co;
  int32_t yf2 = y2 + co;

  if ((yf1 >= YMIN) && (yf1 <= YMAX)){
    oldyf1 = yf1; 
  } else {
    if (yf1 < YMIN) oldyf1 = YMIN;
    if (yf1 > YMAX) oldyf1 = YMAX;
  }
  if ((yf2 >= YMIN) && (yf2 <= YMAX)){
    oldyf2 = yf2; 
  } else {
    if (yf2 < YMIN) oldyf2 = YMIN;
    if (yf2 > YMAX) oldyf2 = YMAX;
  }
  if ((yf1 > YMIN) && (yf2 > YMIN) && (yf1 < YMAX) && (yf2 < YMAX)) {
    if (yf1 == yf2) {
      pfd.fillRect(x1, YMIN, x2, oldyf2 - YMIN, SKY_COLOR);
      pfd.fillRect(x1, y2, x2, YMAX - oldyf1, GND_COLOR);
    }else{
      if (yf1 > yf2) {
        pfd.fillRect(x1, YMIN, x2, oldyf2 - YMIN, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x1,oldyf2, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2+1, x2,oldyf1, GND_COLOR);
        pfd.fillRect(x1, oldyf1, x2, YMAX - oldyf1, GND_COLOR);
      }else{
        pfd.fillRect(x1, YMIN, x2, oldyf1 - YMIN, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x2,oldyf1, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x1,oldyf2, GND_COLOR);
        pfd.fillRect(x1, oldyf2, x2, YMAX - oldyf2, GND_COLOR);
      }
    }
    pfd.drawLine(x1,oldyf1, x2,oldyf2,ILI9341_LIGHTGREY);
  } else {
    if (yf1 <= YMIN) {
      if (yf1 > 0){
        co = YMIN - yf1;
      }else{
        co = YMIN + abs(yf1);
      }
      x1 = co / ta;
    }
     if (yf1 > YMAX) {
      co = yf1 - YMAX;
      x1 = co / abs(ta);
    }
 
    if (yf2 <= YMIN) {
      if (yf2 > 0){
        co = YMIN - yf2;
      }else{
        co = YMIN + abs(yf2);
      }
      x2 = XMAX - (co / abs(ta));
    }
     if (yf2 > YMAX) {
      co = yf2 - YMAX;
      x2 = XMAX - (co / abs(ta));
    }
      if (ta > 0 ) {
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x2,oldyf1, SKY_COLOR);
        pfd.fillRect(XMIN, YMIN, x2, oldyf1 - YMIN, SKY_COLOR);
        pfd.fillRect(x2, YMIN, XMAX - x2, YMAX - YMIN, SKY_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x1,oldyf2, GND_COLOR);
        pfd.fillRect(XMIN, YMIN, x1, YMAX - YMIN, GND_COLOR);
        pfd.fillRect(x1, oldyf2, XMAX - x1, YMAX - oldyf2, GND_COLOR);
      }else{
        pfd.fillTriangle(x1,oldyf1, x2,oldyf2, x2,oldyf1, GND_COLOR);
        pfd.fillRect(x2, oldyf2, XMAX - x2, YMAX - YMIN, GND_COLOR);
        pfd.fillRect(x1, oldyf1, x2, YMAX - oldyf1 , GND_COLOR);
        pfd.fillTriangle(x1,oldyf1, x2, oldyf2, x1,oldyf2, SKY_COLOR);
        pfd.fillRect(XMIN, YMIN, x1, YMAX - YMIN, SKY_COLOR);
        pfd.fillRect(XMIN, YMIN, x2, oldyf2 - YMIN, SKY_COLOR);
      }
      pfd.drawLine(x1, oldyf1, x2, oldyf2,ILI9341_LIGHTGREY);
  }
}

void drawRollScale(float roll) {

  if (roll < -60) roll = -60;
  if (roll > 60) roll = 60;
  drawArc(X_PIVOT, Y_PIVOT, 59, -60 + roll, 60 + roll, ILI9341_WHITE);
  //drawArc(TFT_WIDTH/2, 88, 60, -60 + apm.ahrs.roll, 60 + apm.ahrs.roll, ILI9341_WHITE);

  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 10, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 20, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 30, 59, 7, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 45, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll - 60, 59, 7, ILI9341_WHITE);
    
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 10, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 20, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 30, 59, 7, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 45, 59, 4, ILI9341_WHITE);
  pfd.drawLineByAngle(X_PIVOT, Y_PIVOT-1, roll + 60, 59, 7, ILI9341_WHITE);

  float sa = sin(roll * DEG_TO_RAD);
  float ca = cos(roll * DEG_TO_RAD);

  POINT p[3];
  int x, y;
  
  for (int i=0; i<3;i++){
    switch (i) {
      case 0:
        x = 158;
        y = 24;
        break;
      case 1:
        x = 160;
        y = 29;
        break;
      case 2:
        x = 162;
        y = 24;
        break;
    }
    p[i].x = ca * (x - X_PIVOT) - sa * (y - Y_PIVOT-1) + X_PIVOT;
    p[i].y = sa * (x - X_PIVOT) + ca * (y - Y_PIVOT-1) + Y_PIVOT-1;


    if (p[i].x < 0) p[i].x=0;
    if (p[i].x > 320) p[i].x=320;
    if (p[i].y < 0) p[i].y=0;
    if (p[i].y > 240) p[i].y=240;
  }
  
  pfd.fillTriangle(p[0].x, p[0].y, p[1].x, p[1].y, p[2].x, p[2].y, ILI9341_WHITE);
}

/*
 * Set Dark Color
 */
uint16_t setDarkColor(uint16_t color, int percent){
  uint8_t r, g ,b;
  float perc = (float)percent / 100.0f;
  pfd.color565toRGB(color, r, g, b);
  //Serial.printf("ANTES R:%d G:%d B:%d\n", (int)r, (int)g, (int)b);
  r = (uint8_t)((float)r * perc);
  g = (uint8_t)((float)g * perc);
  b = (uint8_t)((float)b * perc);
  //Serial.printf("DEPOS R:%d G:%d B:%d\n", (int)r, (int)g, (int)b);
  return pfd.color565(r, g, b);
}

/*
 * Draw image scale
 */
void drawScale(const unsigned short* image, float value, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float factor, uint16_t imgH) {

  uint32_t ix = ((imgH - h) - (uint32_t)(value * factor))*w;

  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y;
    uint32_t posj = ix + i;
    uint32_t poss = y0 * H_COLS + x0;
    uint16_t color = pgm_read_word(&image[posj]);
    if (color == TFT_OPAQUE) {
      //uint16_t color2 = scr[poss];
      color = setDarkColor(scr[poss], 60);
      //switch (color2) {
      //  case SKY_COLOR:
      //    color = SKY_COLOR_DARK;
      //    break;
      //  case GND_COLOR:
      //    color = GND_COLOR_DARK;
      //    break;
      //  case ILI9341_LIGHTGREY:
      //    color = ILI9341_DARKGREY;
      //    break;
      //}
    }
    scr[poss] = color;
  }
} //drawScale

void drawScaleH(const unsigned short* image, float value, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float factor) {

  uint32_t ix = (value / factor);
  int32_t r = -1;
  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y -20;
    if (i % w == 0) {
      r++;
    }
    uint32_t posj = ix + i + (r * (compass_W - w));
    uint16_t color = pgm_read_word(&image[posj]);
    uint32_t poss = (y0 * H_COLS) + x0;
    if (color == TFT_OPAQUE) {
      //uint16_t color2 = scr[poss];
      color = setDarkColor(scr[poss], 60);
      //switch (color2) {
      //  case SKY_COLOR:
      //    color = SKY_COLOR_DARK;
      //    break;
      //  case GND_COLOR:
      //    color = GND_COLOR_DARK;
      //    break;
      //  case ILI9341_LIGHTGREY:
      //    color = ILI9341_DARKGREY;
      //    break;
      //}
    }
    scr[poss] = color;
  }
}

void drawimage(const unsigned short* image, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fColor = TFT_OPAQUE, uint16_t tColor = TFT_TRANSPARENT) {

  for (uint32_t i = 0; i < (w * h); i++) {
    uint32_t x0 = (i % w) + x;
    uint32_t y0 = (i / w) + y;
    uint32_t poss = (y0 * H_COLS) + x0;

    uint16_t pixel;
    pixel = pgm_read_word(&image[i]);

    if (pixel != tColor) {
      if (pixel == TFT_OPAQUE) {
        pixel = setDarkColor(scr[poss], 60);
      }
      if (fColor == TFT_OPAQUE)
        scr[poss] = pixel;
      else
        scr[poss] = fColor;
    }
  }
} //drawimage


void drawScreen(ILI9341_t3n *tft){
  // draw screen
  tft->fillScreen(ILI9341_BLACK);
  tft->setTextColor(ILI9341_WHITE);
  tft->drawFastHLine(0, 18, 320, ILI9341_DARKGREY);
  tft->drawFastVLine(66, 0, 17, ILI9341_DARKGREY);
  tft->drawFastVLine(252, 0, 17, ILI9341_DARKGREY);
  tft->drawFastHLine(66, 8, 186, ILI9341_DARKGREY);
  tft->drawFastHLine(0, 230, 320, ILI9341_DARKGREY);
  tft->drawFastVLine(100, 230, 10, ILI9341_DARKGREY);
  tft->drawFastVLine(270, 230, 10, ILI9341_DARKGREY);

  tft->setCursor(0,0);
  tft->print(F("COM1 129.00"));
  tft->setCursor(254,0);
  tft->print(F("NAV1 114.10"));
  tft->setCursor(0,10);
  tft->print(F("COM2 133.00"));
  tft->setCursor(254,10);
  tft->print(F("NAV2 109.30"));

  tft->setCursor(0, TFT_HEIGHT -8);
  tft->print(F("MODE"));

  tft->setCursor(214, TFT_HEIGHT -8);
  tft->print(F("GPS"));

  tft->setCursor(160,0);
  tft->setTextColor(ILI9341_CYAN);
  tft->print(F("BRG"));

  tft->setCursor(160,10);
  tft->setTextColor(ILI9341_ORANGE);
  tft->print(F("BRG"));

  drawIcon(icons, 68, 0, ICO_HOME);
  drawIcon(icons, 207, 0, ICO_TIME);
  drawIcon(icons, 68, 10, ICO_WAYPOINT);
  drawIcon(icons, 109, 10, ICO_EAST);
  drawIcon(icons, 207, 10, ICO_TIME);
  //drawIcon(icons, 228, 240-8, ICO_GPS);
} // drawScreen

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drawEFIS() {
  //pfd.refreshOnce();
  drawBackGround(&apm.ahrs);
  drawPitchScale(pitch_scale, 136, 42, pitch_scale_W, 80, 4.4, &apm.ahrs);
  drawScale(speed_scale_km, apm.airspeed, 59, 41, speed_scale_km_W, 104, 4, 614);  // 0 - 125
  if (apm.alt > 250.0){
    debug.printf(F("Error Alt limit: %fm\n"), apm.alt);
    apm.alt = 250.0;
  }
  drawScale(altitude_scale, apm.alt, 236, 41, altitude_scale_W, 104, 2, 614);
  drawimage(alt_bug, 236, 32, alt_bug_W, alt_bug_H);
  drawimage(vbar, (H_COLS /2) - (vbar_W / 2), 80, vbar_W, vbar_H);
  drawimage(vbar1, (H_COLS /2) - (vbar1_W / 2), 83 + vbar1_H, vbar1_W, vbar1_H);
  drawimage(leftbox, 59, 78, bugbox_W, bugbox_H);
  drawimage(rightbox, 238, 78, bugbox_W, bugbox_H);
  drawimage(vs_scale, 236 + altitude_scale_W + 1, 45, vs_scale_W, vs_scale_H);
   
  //pfd.drawRect(58,40,27,106, ILI9341_WHITE);
  //pfd.drawRect(235,40,28,106, ILI9341_WHITE);

  int16_t climb = (int16_t)((apm.climb * 100.0f) / 5); // m/s -> cm/s -> pixel/s
  drawimage(vs_bug, 236 + altitude_scale_W + 1, -climb + 85, vs_bug_W, vs_bug_H);     //  45 < y < 125
  pfd.setCursor(236 + altitude_scale_W + 2, -climb + 86);
  pfd.setTextColor(ILI9341_WHITE);
  pfd.printf("%3.0f", apm.climb * 100.0f);
  drawIcon(icons, 236 + altitude_scale_W + 1, 73, ICO_BUG, ILI9341_MAGENTA); //  46 < y < 126

  //rotateImage(arrow, (TFT_WIDTH / 2) - (arrow_W / 2) -1, 140, arrow_W, arrow_H, home.direction, ILI9341_GREEN, TFT_TRANSPARENT);
  //rotateImage(arrow, (TFT_WIDTH / 2) - (arrow_W / 2) -1, 140, arrow_W, arrow_H, apm.ahrs.yaw, ILI9341_MAGENTA, TFT_TRANSPARENT);
  //pfd.drawRect(X_PIVOT-32, 160, 63, 20, ILI9341_BLACK);

  // Draw compass
  pfd.drawRect(X_PIVOT-34, 138, 65, 19, ILI9341_WHITE);
  drawScaleH(compass, apm.heading, X_PIVOT-32, 160, 61, 16, 1.5);
  
  // Put arrow to Home
  //float bearingToHome = (home.direction - apm.heading);//relative home direction
  //if(bearingToHome < 0) bearingToHome += 360.0f; //normalization
  //Serial.printf("Bearing To Home: %3.0f\n", bearingToHome);
  
  drawIcon(icons, X_PIVOT-33, 158, ICO_NORTH, ILI9341_CYAN);
  drawIcon(icons, X_PIVOT   , 158, ICO_NORTH, ILI9341_ORANGE);
    
  drawRollScale(apm.ahrs.roll);
  pfd.fillTriangle(158, 37, 160, 31, 162, 36, ILI9341_WHITE);
  //pfd.fillTriangle(157, 37, 160, 31, 163, 37, ILI9341_WHITE);
  // Put Aispeed and Altitude
  pfd.setTextColor(ILI9341_WHITE);
  pfd.setCursor(59,85);
  pfd.printf("%3.0f", apm.airspeed);
  //sprintf (buf, "%3d", (int)old_apm.airspeed);
  //pfd.drawString(59, 85, buf, 1, ILI9341_BLACK, ILI9341_BLACK);
  //sprintf (buf, "%3d", (int)apm.airspeed);
  //pfd.drawString(59, 85, buf, 1, ILI9341_WHITE, ILI9341_BLACK);
  //old_apm.airspeed = apm.airspeed;
   
  float speedUnid = abs(apm.airspeed);
  if (speedUnid >= 100){
    speedUnid = fmod(speedUnid,100.0);
  }
  if (speedUnid >= 10){
    speedUnid = fmod(speedUnid,10.0);
  }
  drawScale(num_scale_V, speedUnid, 71, 80, num_scale_V_W, 17, 10, num_scale_V_H);
    
  pfd.setCursor(238,240-8);
  pfd.printf("%3.0f", apm.alt);
  float altUnid = abs(apm.alt);
  if (altUnid >= 100){
    altUnid = fmod(altUnid,100.0);
  }
  if (altUnid >= 10){
    altUnid = fmod(altUnid,10.0);
  }
  drawScale(num_scale_V, altUnid, 256, 80, num_scale_V_W, 17, 10, num_scale_V_H);

  pfd.setCursor(233,240-8);
  pfd.setTextColor(ILI9341_WHITE, ILI9341_RED);
  switch(apm.fix_type){
    case GPS_FIX_TYPE_NO_FIX:
      pfd.printf(F("NO FIX"));
      break;
    case GPS_FIX_TYPE_2D_FIX:
    pfd.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
      pfd.printf(F("  2D  "));
      break;
    case GPS_FIX_TYPE_3D_FIX:
    case GPS_FIX_TYPE_DGPS:
    case GPS_FIX_TYPE_RTK_FLOAT:
    case GPS_FIX_TYPE_RTK_FIXED:
    case GPS_FIX_TYPE_STATIC:
    case GPS_FIX_TYPE_PPP:
      pfd.setTextColor(ILI9341_WHITE, ILI9341_NAVY);
      pfd.printf(F("  3D  "));
      break;
    default:
      pfd.printf(F("NO GPS"));
      break;
  }

  //  PITCH > 30º OR PITCH < -20º  Hide fields
  //  -15º < PITCH < 25º Show fields
  if ((apm.ahrs.pitch > -15.0) && (apm.ahrs.pitch < 25.0)) {
    
    // Put Heading
    //pfd.drawRect(TFT_WIDTH/2 - 12, 125, 23, 11, ILI9341_WHITE);
    pfd.fillRect(TFT_WIDTH/2 - 11, 126, 21, 9, ILI9341_BLACK);
    pfd.setTextColor(ILI9341_WHITE);
    pfd.setCursor(TFT_WIDTH/2 - 9 ,127);
    pfd.printf("%03d", (int)apm.heading);

    // Field Ground Speed
    pfd.fillRect(59,145,25,9, ILI9341_BLACK);
    // Put GroundSpeed
    pfd.setTextColor(ILI9341_WHITE);
    pfd.setCursor(64,146);
    //pfd.printf("%03.0f", apm.groundspeed);
    pfd.printf("%3d", (int)apm.groundspeed);
    //drawIcon(icons, 78, 148, ICO_GS);

    // Field QNH
    pfd.fillRect(236,145,27,9, ILI9341_BLACK);
     
    // Home information
    pfd.setCursor(77,0);
    pfd.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    if (home.distance > 1000)
      pfd.printf("%3.1fKm", home.distance / 1000.0);
    else
      pfd.printf("%6.0fm", home.distance);
    // Direction
    pfd.setCursor(187,0);
    pfd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    pfd.printf("%03.0f", home.direction);
    // Course to Home
    drawIcon(icons, 178, 0, dirIconIndex(home.direction), ILI9341_CYAN, ILI9341_BLACK);
    // Time to Home
    if (apm.groundspeed < 2) {
      home.ete = 0;
      nav.ete = 0;
    }
    
    uint32_t ete = home.ete;
    uint16_t hh = ete / 3600;
    ete %= 3600;
    uint16_t mm = ete / 60;
    uint16_t ss = ete % 60;
    //pfd.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
    pfd.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    pfd.setCursor(216,0);
    if (hh > 0) {
      if (hh > 23) {
        uint16_t dd = hh / 24;
        hh -= 24;
        if (hh > 99) hh = 99;
        pfd.printf("%02dd%02dh", (int)dd,(int)hh);
      } else {
        pfd.printf("%02d:%02dh", (int)hh,(int)mm);
      }
    } else {
      pfd.printf("%02d:%02dm", mm,ss);
    }
    // Time to WP
    ete = nav.ete;
    hh = ete / 3600;
    ete %= 3600;
    mm = ete / 60;
    ss = ete % 60;
    pfd.setTextColor(ILI9341_DARKGREEN, ILI9341_BLACK);
    pfd.setCursor(216,10);
    if (hh > 0) {
      if (hh > 23) {
        uint16_t dd = hh / 24;
        hh -= 24;
        if (hh > 99) hh = 99;
        pfd.printf("%02dd%02dh", (int)dd,(int)hh);
      } else {
        pfd.printf("%02d:%02dh", (int)hh,(int)mm);
      }
    } else {
      pfd.printf("%02d:%02dm", mm,ss);
    }
    // Next Waypoint Information
    pfd.setCursor(77,10);
    pfd.setTextColor(ILI9341_DARKGREEN, ILI9341_BLACK);
    pfd.printf("%3dWP", (int)nav.wp_number);
    pfd.setCursor(118,10);
    pfd.printf("%3.0fm", (float)nav.wp_dist);
    // Course to WP
    drawIcon(icons, 178, 10, dirIconIndex(nav.ahrs.yaw));
    pfd.setCursor(187,10);
    pfd.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
    pfd.printf("%3.0f", nav.ahrs.yaw);
    // Altitute to WP
    pfd.setCursor(244, 33);
    pfd.setTextColor(ILI9341_BLACK, 0x279E);
    pfd.printf("%3.0f", 150.0);
  }

  flightMode(27, 240-8, ILI9341_YELLOW);
  pfd.updateScreen();
}

void updateTime(){
  // Time to WP
  uint32_t ss = time_boot_ms / 1000;
  uint32_t hh = ss / 3600;
  ss -= hh * 3600;
  uint32_t mm = ss / 60;
  ss -= mm * 60;
  if (hh > 59) hh = 0;
  if (mm > 59) mm = 0;
  if (ss > 59) ss = 0;

  pfd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  pfd.setCursor(272,240-8);
  pfd.printf("%02d:%02d:%02d", hh, mm, ss);
}

int drawImageSD(char *filename, ILI9341_t3n *tft = NULL, int16_t x = 0, int16_t y = 0, int16_t rawWidth = 320, int16_t rawHeight = 240){
  if (!file.open(filename, O_READ)) {
    SD.errorHalt("open failed");
    return 0;
  }
  
  //uint16_t flashBuffer[(rawWidth * ROWS_TO_WRITE)];   // Flash read buffer
  //uint16_t *flashBuffer = (uint16_t*)malloc(rawWidth * ROWS_TO_WRITE * sizeof(uint16_t));

  int rd = 0;
  int bytesread;
  const uint32_t framesize = 153600;
  uint16_t * p = scr;
  rd = framesize;

  do {
    bytesread = file.read(p, rd);
    if (bytesread <= 0) {
      break;
    }
    p += bytesread / sizeof(uint16_t);
    rd -= bytesread;
    if (tft != NULL) {
      tft->writeRect(x, y, rawWidth, rawHeight, scr);
    }
  }while (bytesread > 0 && rd > 0);
  file.close();
}
