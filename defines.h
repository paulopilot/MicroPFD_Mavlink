#define DEBUG_SPEED     115200
#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port

#define debug Serial
#define telem Serial1

//#define TFT_DC        15
//#define TFT_CS        10
//#define TFT_RST       4     // 255 = unused, connect to 3.3V
#define TFT_MOSI        MOSI  //11
#define TFT_SCLK        SCK   //13
#define TFT_MISO        MISO  //12
#define TFT_DC          20
#define TFT_CS          21
#define TFT_RST         255  // = unused, connected to 3.3V




uint32_t heartbeatTimer_TX        = millis();
uint32_t heartbeatTimer_RX        = millis();
uint32_t heartbeatInterval_TX     = 1.0L * 1000L;
uint32_t heartbeatInterval_RX     = 3.0L * 1000L;
bool     mavLink_Connected        = false;

uint32_t refreshTimer = millis();
//uint32_t refreshInterval = 0.5L * 1000L;
uint32_t refreshInterval = 0.1L * 1000L;

float     converts = 3.6;   // convert m/s to km/h
float     converth = 1.0;   // disatance to m

ILI9341_t3DMA tft = ILI9341_t3DMA(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

//uint32_t * scr = screen32;
uint16_t * scr = screen16;
#define TFT_WIDTH           ILI9341_TFTWIDTH
#define TFT_HEIGHT          ILI9341_TFTHEIGHT 
#define H_ROWS              210
#define H_COLS              320
#define X_PIVOT             H_COLS / 2
#define Y_PIVOT             H_ROWS / 2

#define SKY_COLOR           0x027F
#define SKY_COLOR_DARK      0x1A35 // 0x010E
#define GND_COLOR           0x79C2 //0x4B02 // 
#define GND_COLOR_DARK      0x38E1 //0x3A04 // 
#define TRANSPARENT         ILI9341_MAGENTA
#define TFT_TRANSPARENT     tft.color565(0, 36, 0)

static float                alt_prev = 0;             // previous altitude
static uint8_t              alt_cnt = 0;              // counter for stable osd_alt
//MAVLink session control
static boolean              mavbeat = 0;
static float                lastMAVBeat = 0;
static boolean              waitingMAVBeats = 1;
static uint8_t              apm_mav_system; 
static uint8_t              apm_mav_component;
static boolean              enable_mav_request = 0;
static uint8_t              flight_mode = 0;                   // Navigation mode from RC AC2 = CH5, APM = CH8
static uint8_t              osd_nav_mode = 0;               // Navigation mode from RC AC2 = CH5, APM = CH8
static uint8_t              base_mode=0;
static uint8_t              apm_mav_type;

typedef struct {
  float                   roll = 0;                   // [rad] Roll angle (-pi..+pi)
  float                   pitch = 0;                  // [rad] Pitch angle (-pi..+pi)
  float                   yaw = 0;                    // [rad] Yaw angle (-pi..+pi)
} __attribute__((__packed__))AHRS;

typedef struct {
  float                   lat ;
  float                   lon;
  float                   alt;
} __attribute__((__packed__))WAYPOINT;

typedef struct {
  uint8_t                 got_home = 0;               // tels if got home position or not
  WAYPOINT                position; 
  float                   distance = 0;               // distance from home
  uint8_t                 direction;                  // Arrow direction pointing to home (1-16 to CW loop)
  float                   glide = 0;
  uint32_t                ete = 0;                    // etimated time enroute in seconds
} __attribute__((__packed__))HOME;

typedef struct {
  AHRS                    ahrs;                       // [deg] Current desired roll/pitch/yaw
  WAYPOINT                wp;
  int16_t                 wp_target_bearing = 0;      // [deg] Current desired heading
  int16_t                 target_bearing;             // [deg] Bearing to current waypoint/target
  uint16_t                wp_dist = 0;                // [m] Distance to active waypoint
  float                   alt_error = 0;              // Current altitude error in meters
  float                   aspd_error = 0;             // Current airspeed error in meters/second
  float                   xtrack_error = 0;           // Current crosstrack error on x-y plane in meters
  uint8_t                 wp_number = 0;              // Current waypoint number
  uint32_t                ete = 0;                    // etimated time enroute in seconds
} __attribute__((__packed__))NAV;

#define                   GPS_NO      0
#define                   GPS_NO_FIX  1
#define                   GPS_2D      2
#define                   GPS_3D      3

typedef struct {
  bool                    connected = false;
  uint16_t                hb_count = 0;               // heart beat count
  float                   vbatA = 0;                  // Battery A voltage in milivolt
  float                   ibatA = 0;                  // Battery A current
  int8_t                  remainingA = 0;             // 0 to 100 <=> 0 to 1000
  bool                    motor_armed = 0;
  /* MAVLINK_MSG_ID_GPS_RAW_INT */
  WAYPOINT                position;
  uint8_t                 fix_type = GPS_NO;          // GPS lock 0-1=no fix, 2=2D, 3=3D
  uint8_t                 satellites_visible = 0;     // number of satelites
  /* MAVLINK_MSG_ID_VFR_HUD 74 */
  float                   airspeed = 0;               // [m/s] Current indicated airspeed (IAS).
  float                   groundspeed = 0;            // [m/s] Current ground speed.
  float                   alt = 0;                    // [m] Current altitude (MSL).
  float                   climb = 0;                  // [m/s] Current climb rate.
  int16_t                 heading = 0;                // [deg] Current heading in compass units (0-360, 0=north).
  uint16_t                throttle = 0;               // [%] Current throttle setting (0 to 100).
  
  AHRS                    ahrs;
} __attribute__((__packed__))APM;

APM       apm;
NAV       nav;
HOME      home;

#define MAX_PITCH   60.0f
#define MIN_PITCH   -60.0f
