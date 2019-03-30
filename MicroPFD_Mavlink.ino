#include "SPI.h"
#include <ILI9341_t3DMA.h>
#include <GCS_MAVLink.h>

#include "defines.h"
#include "function.h"
#include "mavlink_functions.h"



void setup() {
  // put your setup code here, to run once:
  debug.begin(DEBUG_SPEED);
  telem.begin(TELEMETRY_SPEED);

  tft.begin();

  //START DMA MODE
  tft.refresh();
    
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.println("INITIALIZING...");
  delay(2000);
  drawScreen();
}

/* 
 *  PITCH > 30º OR PITCH < -20º  Hide fields
 *  -15º < PITCH < 25º Show fields
 */

uint64_t amountRead = 0;
uint32_t samples = 0;
uint32_t anaVal;


void loop() {
  // put your main code here, to run repeatedly:
  handle_Messages();

  if (samples++ > 100) {
    anaVal = amountRead / samples;
    amountRead = 0;
    samples = 0;
    //apm.airspeed = mapfloat(float(anaVal), 0.0, 1023.0, 0.0, 125.0);
    //apm.alt = mapfloat(float(anaVal), 0.0, 1023.0, -20.0, 250.0);
    //apm.ahrs.roll = mapfloat(float(anaVal), 0.0, 1023.0, -180.0, 180.0);
    apm.ahrs.pitch = mapfloat(float(anaVal), 0.0, 1023.0, 90.0, -90.0);
    //apm.ahrs.yaw = mapfloat(float(anaVal), 0.0, 1023.0, -180.0, 180.0);
    //apm.heading = mapfloat(float(anaVal), 0.0, 1023.0, 0.0, 360.0);
    
  } else {
    amountRead += analogRead(A9);
  }
  
  
  
  //Pack and send heartbeat at specific interval to the GCS
  /*
  if((millis() - heartbeatTimer_TX) > heartbeatInterval_TX)
  {
    drawIcon(icons, 300, 220, ICO_HEART_ON);
    heartbeatTimer_TX = millis();
    send_heartbeat();
    //send_status();  //fake data sent for now.  Seen this at .5sec
    //refreshInterval += 100;
    //Serial.printf("tempo : %d\n", (int)refreshInterval);
    //Serial.printf("Tempo %dus\n", micros() - t);
    drawIcon(icons, 300, 220, ICO_HEART_OFF);
  }
  */
  if((millis() - refreshTimer) > refreshInterval)
  {
    setHomeVars();
    drawHorizon(0, 19, -apm.ahrs.pitch, 0);
    drawScale(speed_scale_km, apm.airspeed, 59, 41, speed_scale_km_W, 104, 4);  // 0 - 125
    drawScale(altitude_scale, apm.alt, 236, 41, altitude_scale_W, 104, 2);
    drawimage(vbar, (H_COLS /2) - (vbar_W / 2), 76, vbar_W, vbar_H, TFT_TRANSPARENT, ILI9341_MAGENTA);
    drawimage(leftbox, 59, 80, bugbox_W, bugbox_H, ILI9341_BLACK, ILI9341_MAGENTA);
    drawimage(rightbox, 238, 80, bugbox_W, bugbox_H, ILI9341_BLACK, ILI9341_MAGENTA);
    tft.drawRect(58,40,27,106, ILI9341_WHITE);
    tft.drawRect(235,40,28,106, ILI9341_WHITE);
    

    //for (float a = 0; a < 360.0; a=a+0.1)
    rotateImage(arrow, (TFT_WIDTH / 2) - (arrow_W / 2) -1, 140, arrow_W, arrow_H, (float)home.direction, ILI9341_GREEN, ILI9341_MAGENTA);
    rotateImage(arrow, (TFT_WIDTH / 2) - (arrow_W / 2) -1, 140, arrow_W, arrow_H, apm.ahrs.yaw, ILI9341_MAGENTA, ILI9341_MAGENTA);
    
    //tft.drawCircle(TFT_WIDTH/2, 88, 40, ILI9341_WHITE);
    //tft.drawCircleHelper(TFT_WIDTH/2, 88, 60,3 , ILI9341_WHITE);
    //tft.drawCircleHelper(TFT_WIDTH/2, 88, 59,3 , ILI9341_WHITE);
    drawArc(TFT_WIDTH/2, 88, 59, -60 + apm.ahrs.roll, 60 + apm.ahrs.roll, ILI9341_WHITE);
    drawArc(TFT_WIDTH/2, 88, 60, -60 + apm.ahrs.roll, 60 + apm.ahrs.roll, ILI9341_WHITE);
    tft.fillTriangle(157, 21, 160, 27, 163, 21, ILI9341_WHITE);
    tft.fillTriangle(157, 37, 160, 31, 163, 37, ILI9341_WHITE);

    // Put Aispeed and Altitude
    //tft.setFont(Arial_8);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(59,85);
    tft.printf("%3.0f", apm.airspeed);
    tft.setCursor(244,85);
    tft.printf("%3.0f", apm.alt);

    //  PITCH > 30º OR PITCH < -20º  Hide fields
    //  -15º < PITCH < 25º Show fields
    if ((apm.ahrs.pitch > -15.0) && (apm.ahrs.pitch < 25.0)) {
      // Field Ground Speed
      tft.fillRect(59,147,25,9, ILI9341_BLACK);
      tft.drawRect(58,146,27,11, ILI9341_WHITE);
      
      // Put Heading
      tft.drawRect(TFT_WIDTH/2 - 12, 125, 23, 11, ILI9341_WHITE);
      tft.fillRect(TFT_WIDTH/2 - 11, 126, 21, 9, ILI9341_BLACK);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(TFT_WIDTH/2 - 9 ,127);
      tft.printf("%03d", (int)apm.heading);
      
      // Put GroundSpeed
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(60,148);
      tft.printf("%03.0fG", apm.groundspeed);
      
      // Home information
      tft.setCursor(77,0);
      tft.setTextColor(ILI9341_MAGENTA);
      //tft.printf("%3.1fKm", 123.5);
      tft.printf("%3.0fm", home.distance);
      tft.setCursor(187,0);
      tft.setTextColor(ILI9341_WHITE);
      tft.printf("%03.0f", (float)home.direction);
      // Course to Home
      drawIcon(icons, 178, 0, dirIconIndex((float)home.direction));
      // Time to Home
      uint32_t ete = home.ete;
      uint16_t hh = ete / 3600;
      ete %= 3600;
      uint16_t mm = ete / 60;
      uint16_t ss = ete % 60;
      tft.setTextColor(ILI9341_CYAN);
      tft.setCursor(216,0);
      if (hh != 0) 
        tft.printf("%02d:%02dm", (int)hh,(int)mm);
      else
        tft.printf("%02d:%02ds", mm,ss);
  
      // Next Waypoint Information
      tft.setCursor(77,10);
      tft.setTextColor(ILI9341_DARKGREEN);
      tft.printf("%3dWP", (int)nav.wp_number);
      tft.setCursor(118,10);
      tft.printf("%3.0fm", (float)nav.wp_dist);
      // Course to WP
      drawIcon(icons, 178, 10, dirIconIndex(nav.ahrs.yaw));
      tft.setCursor(187,10);
      tft.setTextColor(ILI9341_ORANGE);
      tft.printf("%3.0f", nav.ahrs.yaw);
      // Time to WP
      ete = nav.ete;
      hh = ete / 3600;
      ete %= 3600;
      mm = ete / 60;
      ss = ete % 60;
      tft.setTextColor(ILI9341_GREEN);
      tft.setCursor(216,10);
      if (hh != 0) 
        tft.printf("%02d:%02dm", (int)hh,(int)mm);
      else
        tft.printf("%02d:%02ds", mm,ss);
    }

    flightMode(0, 240-8, ILI9341_WHITE);
    refreshTimer = millis();
  }
}
