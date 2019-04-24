#include <Thread.h>
#include <SPIN.h>
#include "SPI.h"
#include <ILI9341_t3n.h>
#include <SdFat.h>
//#include <GCS_MAVLink.h>

static float last_roll = 0;
static float last_pitch = 0;

#include "defines.h"
#include "function.h"
#include "mavlink_functions.h"

Thread::Task tasks[] = {
  THREADTASK(drawEFIS, 44.787 ,true),
  THREADTASK(setHomeVars,500,true),
  THREADTASK(updateTime,1000,true),
  //THREADTASK(debug,100,true),
  THREADEND
};

void setup() {
  // put your setup code here, to run once:
  debug.begin(DEBUG_SPEED);
  telem.begin(TELEMETRY_SPEED);

  // create frame buffer
  //scr = (uint16_t *)malloc(FRAME_BUFFER_SIZE);
  //if (scr == NULL){
  //  debug.println(F("Error to create buffer.\nSystem halted!"));
  //  while(1);
  //}
  //memset(scr, 0, FRAME_BUFFER_SIZE);

  SD.begin();
  SPI.begin();
 
  pfd.begin();
  pfd.setRotation(1);
  pfd.fillScreen(ILI9341_BLACK);
  pfd.setTextColor(ILI9341_WHITE);
  pfd.setTextSize(1);
  pfd.println("INITIALIZING SYSTEM");
  
  
#ifdef USE_MFD  
  mfd.begin();
  mfd.setRotation(1);
  mfd.fillScreen(ILI9341_BLACK);
  mfd.setTextColor(ILI9341_WHITE);
  mfd.setTextSize(1);
  mfd.println("INITIALIZING SYSTEM");
#endif
  setDarkColor(pfd.color565(255, 200, 100), 80);
  delay(2000);
  mfd.fillScreenVGradient(ILI9341_NAVY, ILI9341_BLACK);
  pfd.setFrameBuffer(scr);
  pfd.useFrameBuffer(true);
  drawImageSD("I0001.RAW" , &mfd); // We need run once time because the frame buffer retain data to both displays
  pfd.updateScreen();
  delay(3000);
  drawImageSD("I0022.RAW" , &mfd);
  drawScreen(&pfd);   // Change frame buffer before show into display
  pfd.updateScreen();
  Thread::load_tasks(tasks);
}

/* 
 *  PITCH > 30º OR PITCH < -20º  Hide fields
 *  -15º < PITCH < 25º Show fields
 */

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t now = micros();
  handle_Messages();
  Thread::run(now);
  
#ifdef TEST_MODE
  apm.fix_type = GPS_3D;
  // SBFZ
  home.position.lat = -3.77583;
  home.position.lon = -38.5322;
  // SBRF
  apm.position.lat =  -8.12638;
  apm.position.lon =  -34.9227;

  if (samples++ > 100) {
    anaVal = amountRead / samples;
    amountRead = 0;
    samples = 0;
    apm.airspeed = mapfloat(float(anaVal), anaValMin, anaValMax, 0.0, 125.0);
    apm.alt = mapfloat(float(anaVal), anaValMin, anaValMax, -20.0, 250.0);
    apm.ahrs.roll = mapfloat(float(anaVal), anaValMin, anaValMax, -180.0, 180.0);
    apm.ahrs.pitch = mapfloat(float(anaVal), anaValMin, anaValMax, 90.0, -90.0);
    apm.ahrs.yaw = mapfloat(float(anaVal), anaValMin, anaValMax, -180.0, 180.0);
    apm.heading = mapfloat(float(anaVal), anaValMin, anaValMax, 0.0, 360.0);
    refreshInterval = map(anaVal, anaValMin, anaValMax, 60000, 1000000); //171 37ms
    apm.ahrs.sinRoll = sin(apm.ahrs.roll * DEG_TO_RAD);
    apm.ahrs.cosRoll = cos(apm.ahrs.roll * DEG_TO_RAD);
    apm.groundspeed = apm.airspeed;
  } else {
    uint32_t ana = analogRead(A9);
    if (ana < anaValMin) anaValMin = ana;
    if (ana > anaValMax) anaValMax = ana;
    amountRead += ana;
  }
#endif
  
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
}
