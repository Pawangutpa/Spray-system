#include <Arduino.h>
#include <time.h>
#include "button.h"

#include "MQTT.h"

// =================================================================================
//                                 CONFIGURATION
// =================================================================================

// ---- Modem UART Pins & Settings ---
#define MODEM_RX            15
#define MODEM_TX            14
#define MODEM_RESET         18
static const uint32_t MODEM_BAUD = 115200;

// --- Network Settings (APN) ---
// CHANGE THIS to match your SIM provider (e.g., "airteliot.com", "jionet", etc.)
static const char SIM_APN[] = "airteliot.com";

// --- MQTT Broker Settings ---
static const char CLIENT_ID[]   = "ESP32_EC200U_Client";
static const char DEVICE_ID[]   = "10:20:BA:47:BD:3C";  // MAC-like ID used for Topic path
static const char MQTT_BROKER[] = "3.110.107.194";
static const uint16_t MQTT_PORT = 1883;

// --- System Timing ---
static const unsigned long READ_POLL_MS      = 5;     // UART polling interval
static const unsigned long RSSI_PERIOD       = 5000;  // Signal strength check interval
static const unsigned long GPS_POLL_MS       = 2000;  // GPS location check interval
static const unsigned long PUBLISH_PERIOD_MS = 1000;  // Heartbeat interval

// --- State Timers ---
static unsigned long lastPublishMs = 0;
static unsigned long lastPubAckMs  = 0;
static unsigned long lastRssiMs    = 0;
static unsigned long lastGpsPollTs = 0;



// =================================================================================
//                                 KALMAN FILTER
// =================================================================================
/**
 * Simple 1D Kalman Filter to smooth GPS coordinates.
 * Helps reduce "jitter" when the vehicle is stationary.
 */
class KalmanFilter {
public:
    float Q = 0.0009; // Process noise: Lower = more stable, slower to react
    float R = 0.0005;   // Measurement noise: Higher = trust measurement less
    float P = 1;        // Estimation error covariance
    float X = 0;        // Value state

    float update(float m) {
        P += Q;
        float K = P / (P + R);
        X += K * (m - X);
        P *= (1 - K);
        return X;
    }
};

static KalmanFilter kLat, kLon;

// =================================================================================
//                                GLOBAL VARIABLES
// =================================================================================

// --- GNSS & Telemetry State ---
static uint32_t gps_unix_timestamp = 0;
static int      cached_rssi        = 0;
static int      gps_fix_quality    = 0;

// --- Output Variables (Filtered for JSON) ---
static double disp_lat = 0.0;
static double disp_lon = 0.0;

// --- Internal GNSS Logic ---
static double rawLat = 0, rawLon = 0;
static double filtLat = 0, filtLon = 0;
static double lastLat = 0, lastLon = 0;
static double lastLat_wheel = 0, lastLon_wheel = 0;
static double gps_speed_kmph = 0;
static bool   kalmanInit     = false;

// --- Modem Buffer ---
static String rxAcc = "";

// --- MQTT Publishing State Machine ---
enum PubState { 
    P_IDLE, 
    P_WAIT_URC 
};

// --- MQTT / MODEM STATUS FLAGS (FOR LED) ---
static bool mqtt_sim_registered = false;
static bool mqtt_internet_up    = false;
static bool mqtt_connected      = false;


static PubState      state          = P_IDLE;
static String        pendingPayload = "";
static String        pendingTopic   = "";
static unsigned long stateTs        = 0;

// =================================================================================
//                                   UTILITIES
// =================================================================================

/**
 * Calculates CRC32 checksum for data integrity.
 * Used to verify the JSON payload on the server side.
 */
uint32_t crc32(const char *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    while (length--) {
        uint8_t byte = *data++;
        crc ^= byte;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

/**
 * Removes trailing Carriage Returns (\r) and Newlines (\n).
 */
static void trimStr(String &s) {
    while (s.length() && (s.endsWith("\r") || s.endsWith("\n"))) {
        s.remove(s.length() - 1);
    }
}

/**
 * Converts NMEA DDMM.MMMM format to Decimal Degrees.
 * e.g., 2628.1234 -> 26.468723
 */
double nmeaToDecimal(double raw, char dir) {
    int deg = int(raw / 100);
    double min = raw - deg * 100;
    double dec = deg + min / 60.0;
    if (dir == 'S' || dir == 'W') dec *= -1;
    return dec;
}

/**
 * Sends a raw AT command to the Modem via Serial2.
 */
static void atSend(const char *cmd) {
    Serial2.print(cmd);
    Serial2.print("\r\n");
    Serial.print("TX: ");
    Serial.println(cmd);
}

/**
 * Reads available bytes from Modem Serial without blocking.
 */
static String readAvailableNonBlock() {
    String s = "";
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        s += c;
    }
    return s;
}

/**
 * Blocking wait for a specific response (Used mainly during initialization).
 */
static bool waitForResponse(const char *expected, unsigned long timeoutMs) {
    unsigned long start = millis();
    String acc = "";
    while (millis() - start < timeoutMs) {
        if (Serial2.available()) {
            char c = (char)Serial2.read();
            acc += c;
            if (acc.indexOf(expected) >= 0) {
                Serial.println("RX: " + acc);
                return true;
            }
        }
        delay(1);
    }
    Serial.println("TIMEOUT waiting for: " + String(expected));
    Serial.println("RX Buffer: " + acc);
    return false;
}

// =================================================================================
//                           NETWORK & CONNECTION LOGIC
// =================================================================================

static bool waitForNetwork(unsigned long timeoutMs) {
    unsigned long start = millis();
    String acc = "";

    while (millis() - start < timeoutMs) {
        atSend("AT+CREG?");
        unsigned long t = millis();
        acc = "";

        while (millis() - t < 1000) {
            while (Serial2.available()) {
                acc += (char)Serial2.read();
            }
        }

        // Check for registered home network (1) or roaming (5)
        if (acc.indexOf("+CREG: 0,1") >= 0 || acc.indexOf("+CREG: 0,5") >= 0) {
            Serial.println("Network registered");
            return true;
        }
        delay(1000);
    }

    Serial.println("Network registration timeout");
    return false;
}

static bool waitForDataAttach() {
    for (int i = 0; i < 10; i++) {
        atSend("AT+CGATT?");
        if (waitForResponse("+CGATT: 1", 2000)) {
            Serial.println("Data attached");
            return true;
        }
        delay(2000);
    }
    return false;
}

// =================================================================================
//                               SPEED CALCULATION
// =================================================================================

static float computeInstantSpeed(unsigned long cumulative_count) {
    const float WHEEL_DIAMETER      = 0.69f;
    const float WHEEL_CIRCUMFERENCE = 3.1415926f * WHEEL_DIAMETER;
    const float PULSES_PER_REV      = 5.0f;

    static unsigned long count_hist[3] = {0, 0, 0};
    static unsigned long time_hist[3]  = {0, 0, 0};
    static bool init = false;

    unsigned long now = millis();

    // Shift history buffer
    count_hist[2] = count_hist[1];
    count_hist[1] = count_hist[0];
    count_hist[0] = cumulative_count;
    time_hist[2]  = time_hist[1];
    time_hist[1]  = time_hist[0];
    time_hist[0]  = now;

    if (!init) {
        if (time_hist[2] != 0) init = true;
        return 0.0f;
    }

    long delta_counts     = (long)count_hist[0] - (long)count_hist[1];
    unsigned long dt_ms   = time_hist[0] - time_hist[1];

    if (delta_counts <= 0 || dt_ms == 0) return 0.0f;

    float dt_sec         = dt_ms / 1000.0f;
    float pulses_per_sec = delta_counts / dt_sec;
    float speed_mps      = (pulses_per_sec / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE;

    return speed_mps * 3.6f; // Convert m/s to km/h
}

// =================================================================================
//                                PARSING LOGIC
// =================================================================================

/**
 * Parses +QGPSLOC response, validates data, and applies Kalman Filtering.
 */
static void parseQGPSLOC_Kalman(String resp) {
    // Format: +QGPSLOC: <time>,<lat>,<lon>,<hdop>,<alt>,<fix>,<cog>,<spkm>,<spkn>,<date>,<nsat>
    int idx = resp.indexOf("+QGPSLOC:");
    if (idx < 0) return;
    
    resp = resp.substring(idx + 10);
    resp.trim();

    // Helper Lambda to extract CSV fields by index
    auto getField = [&](int n) -> String {
        int count = 0;
        int lastP = 0;
        for (int i = 0; i < resp.length(); i++) {
            if (resp.charAt(i) == ',') {
                if (count == n) return resp.substring(lastP, i);
                lastP = i + 1;
                count++;
            }
        }
        if (count == n) return resp.substring(lastP);
        return "";
    };

    String timeStr  = getField(0);
    String latStr   = getField(1);
    String lonStr   = getField(2);
    String speedStr = getField(7); // Index 7 is Speed in km/h

    // Update Timestamp
    if (timeStr.length() >= 6) gps_unix_timestamp = millis() / 1000;

    // Validate Lat/Lon
    if (latStr.length() < 2 || lonStr.length() < 2) return;

    // Handle N/S E/W direction characters
    char latDir = latStr.charAt(latStr.length() - 1);
    if (!isdigit(latDir)) latStr = latStr.substring(0, latStr.length() - 1);
    else latDir = 'N';

    char lonDir = lonStr.charAt(lonStr.length() - 1);
    if (!isdigit(lonDir)) lonStr = lonStr.substring(0, lonStr.length() - 1);
    else lonDir = 'E';

    // Convert to Decimal
    rawLat = nmeaToDecimal(latStr.toDouble(), latDir);
    rawLon = nmeaToDecimal(lonStr.toDouble(), lonDir);
    gps_speed_kmph = speedStr.toFloat();

    // Initialize Kalman Filter if first run
    if (!kalmanInit) {
        kLat.X = rawLat;
        kLon.X = rawLon;
        filtLat = rawLat;
        filtLon = rawLon;
        lastLat = rawLat;
        lastLon = rawLon;
        lastLat_wheel = rawLat;
        lastLon_wheel = rawLon;
        kalmanInit = true;
    }

    // Dead Reckoning: If speed is very low (< 0.8 km/h), assume stationary
    // This prevents GPS drift while parked.
    // if (gps_speed_kmph < 0.8) {
    //     filtLat = lastLat;
    //     filtLon = lastLon;
    // } else {
    //     filtLat = kLat.update(rawLat);
    //     filtLon = kLon.update(rawLon);
    //     lastLat = filtLat;
    //     lastLon = filtLon;
    // }


    filtLat = kLat.update(rawLat);
    filtLon = kLon.update(rawLon);

    // lastLat = filtLat;
    // lastLon = filtLon;

    // disp_lat = filtLat;
    // disp_lon = filtLon;

    String fixStr = getField(5);
    gps_fix_quality = fixStr.toInt();
}

/**
 * Polls RSSI (Signal Strength) periodically.
 * Does not run if MQTT is currently busy transmitting.
 */
static void pollAndCacheRssi() {
    if (millis() - lastRssiMs < RSSI_PERIOD) return;
    if (state != P_IDLE) return; 
    
    atSend("AT+CSQ");
    lastRssiMs = millis();
}

// =================================================================================
//                                JSON BUILDER
// =================================================================================

static String buildHeartbeatJSON(const LidarData &l, const ProxiData &p, const ButtonData &b) {
    int left_sol       = digitalRead(4);
    int right_sol      = digitalRead(5);
    int spray_mode     = b.autonomous;
    float water_liter      = p.count1;
    float cpu_temp     = temperatureRead();
    uint32_t uptime    = millis() / 1000;
    uint32_t free_heap = ESP.getFreeHeap();
    float wheel_speed  = computeInstantSpeed((unsigned long)p.count_avg);
     if (wheel_speed < 0.3) {
        disp_lat =lastLat_wheel ;
        disp_lon =lastLon_wheel ;
    } else {
        lastLat_wheel = filtLat;
        lastLon_wheel = filtLon;
        disp_lat = lastLat_wheel;
        disp_lon = lastLon_wheel;
    }
   
    // Constructing JSON Array
    String j = "[";
    j += gps_unix_timestamp;        j += ",";       // heartbeat (unix time)
    j += gps_fix_quality;           j += ",";       // gps_signal_quality
    j += cached_rssi;               j += ",";       // sim_signal_quality (dBm)
    j += free_heap;                 j += ",";       // free_heap_bytes
    j += uptime;                    j += ",";       // uptime_sec
    j += String(disp_lat, 6);       j += ",";
    j += String(disp_lon, 6);       j += ",";
    j += String((float)l.rawDistA, 2); j += ",";    // left_distance
    j += String((float)l.rawDistB, 2); j += ",";    // right_distance
    j += String((float)l.rawStrA, 2);  j += ",";    // left_density
    j += String((float)l.rawStrB, 2);  j += ",";    // right_density
    j += String(wheel_speed);       j += ",";
    j += String((float)l.rawStrA, 2);  j += ","; // Duplicate flow placeholder?
    j += left_sol;                  j += ",";
    j += right_sol;                 j += ",";
    j += spray_mode;                j += ",";
    j += String((float)l.rawStrB, 2);j += ",";    //tank_level; j += ",";
    j += String(water_liter,3) ;    j += ",";
    j += String(cpu_temp, 2);       j += ",";

    // Append CRC for validation
    uint32_t crc = crc32(j.c_str(), j.length());
    j += crc;
    j += "]";
    return j;
}

// =================================================================================
//                                PUBLIC API
// =================================================================================

/**
 * Initializes Modem, Network, and MQTT Connection.
 * This function contains blocking delays during startup.
 */
void MQTT_begin(uint32_t baud) {
    // --- Hardware Reset (Optional/Uncomment if needed) ---
    // pinMode(MODEM_RESET, OUTPUT);
    // digitalWrite(MODEM_RESET, LOW);
    // delay(300);
    // digitalWrite(MODEM_RESET, HIGH);
    // delay(5000);

    Serial2.begin(baud ? baud : MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    Serial.println("Initializing Modem...");
    delay(2000); // Allow modem power-up time

    // 1. Basic Setup (Echo off, verbose errors)
    atSend("ATE0");
    waitForResponse("OK", 1000);
    atSend("AT+CMEE=2");
    waitForResponse("OK", 1000);

    // 2. Check SIM Status
    atSend("AT+CPIN?");
    if (!waitForResponse("READY", 3000)) {
        Serial.println("SIM ERROR: SIM card not detected or locked");
        return;
    }
     
    // 2.1 Wait for network registration
    if (!waitForNetwork(30000)) {
        Serial.println("No network registration");
        return;
    }
    // ✅ SIM successfully registered on network
mqtt_sim_registered = true;


    // 2.2 Wait for data attach
    if (!waitForDataAttach()) {
        Serial.println("Data attach failed");
        return;
    }

    // 3. Configure Network Context (APN)
    String cgdcont = String("AT+CGDCONT=1,\"IP\",\"") + SIM_APN + "\"";
    atSend(cgdcont.c_str());
    waitForResponse("OK", 2000);

    // Setup QICSGP (Context ID 1, IPV4, APN)
    String qicsgp = String("AT+QICSGP=1,1,\"") + SIM_APN + "\",\"\",\"\",1";
    atSend(qicsgp.c_str());
    waitForResponse("OK", 2000);

    // 4. Activate Context
    atSend("AT+QIACT=1");
    waitForResponse("OK", 10000); // Network activation can take time
    // ✅ PDP context active (Internet available)
mqtt_internet_up = true;

    // 5. GPS Setup
    atSend("AT+QGPS=1");
    waitForResponse("OK", 2000);
    atSend("AT+QGPSCFG=\"nmeasrc\",1"); // Enable NMEA source
    waitForResponse("OK", 1000);

    // 6. MQTT Protocol Configuration
    atSend("AT+QMTCFG=\"version\",0,4");    // Set MQTT Version to 3.1.1
    waitForResponse("OK", 1000);
    atSend("AT+QMTCFG=\"clean\",0,1");      // Enable Clean Session
    waitForResponse("OK", 1000);
    atSend("AT+QMTCFG=\"keepalive\",0,30"); // 30 seconds keepalive
    waitForResponse("OK", 1000);
    atSend("AT+QMTCFG=\"retrans\",0,1");    // Enable auto retransmission
    waitForResponse("OK", 1000);

    // 7. Open MQTT Socket
    atSend("AT+QMTCLOSE=0"); // Ensure socket is closed before opening
    waitForResponse("OK", 1000);
    delay(500);

    String open = String("AT+QMTOPEN=0,\"") + MQTT_BROKER + "\"," + String(MQTT_PORT);
    atSend(open.c_str());

    // Critical: Wait for +QMTOPEN: 0,0 (0 = success)
    static int mqttFailCount = 0;
    if (!waitForResponse("+QMTOPEN: 0,0", 15000)) {
        mqttFailCount++;
        Serial.println("MQTT OPEN FAILED, retrying...");
        if (mqttFailCount < 5) {
            delay(5000);
            MQTT_begin(baud);
            return;
        }
        Serial.println("MQTT OPEN FAILED permanently");
        return;
    }
    mqttFailCount = 0;

    // 8. Connect MQTT Client
    String conn = String("AT+QMTCONN=0,\"") + CLIENT_ID + "\"";
    atSend(conn.c_str());

    // Critical: Wait for +QMTCONN: 0,0,0 (Success)
    if (!waitForResponse("+QMTCONN: 0,0,0", 10000)) {
        Serial.println("MQTT CONNECT FAILED (Check Credentials/ClientID)");
        return;
    }
    // ✅ MQTT connected to broker
mqtt_connected = true;



    Serial.println("MQTT CONNECTED SUCCESSFULLY!");
    lastGpsPollTs = millis();
    state = P_IDLE;
    cached_rssi = 0;
}

/**
 * Publishes telemetry data to the broker.
 * returns: true if queued successfully, false if busy.
 */
bool MQTT_publishHeartbeat(const LidarData &lidar, const ProxiData &prox, const ButtonData &btn) {
    if (state != P_IDLE) return false; // Prevent overlapping publications

    pendingPayload = buildHeartbeatJSON(lidar, prox, btn);
    
    // Topic Format: /<DeviceID>/heartbeat
    pendingTopic = String("/") + DEVICE_ID + "/heartbeat";

    // Command: AT+QMTPUBEX (Publish Extended)
    // Send payload length immediately to avoid waiting for prompt
    String cmd = String("AT+QMTPUBEX=0,0,0,0,\"") + pendingTopic + "\"," + String(pendingPayload.length());
    atSend(cmd.c_str());

    delay(20);                       // Small delay to allow modem buffer prep
    Serial2.print(pendingPayload);   // Send raw JSON payload

    stateTs = millis();
    return true;
}

/**
 * Main Loop Ticker. 
 * Handles incoming URCs (Unsolicited Result Codes), GPS parsing, and RSSI checks.
 */
void MQTT_tick() {
    static bool modemStarted = false;
    static unsigned long bootTs = millis();

    // Allow EC200U to fully boot before initializing
    if (!modemStarted) {
        if (millis() - bootTs < 15000) return; 
        modemStarted = true;
        MQTT_begin(MODEM_BAUD);
        return;
    }

    // 1. Read Bytes from Modem
    String s = readAvailableNonBlock();
    if (s.length()) rxAcc += s;

    // 2. Process Complete Lines
    while (true) {
        int eol = -1;
        // Find newline
        for (int i = 0; i < (int)rxAcc.length(); ++i) {
            if (rxAcc.charAt(i) == '\n') {
                eol = i;
                break;
            }
        }

        if (eol < 0) break; // No full line yet

        String line = rxAcc.substring(0, eol + 1);
        rxAcc = rxAcc.substring(eol + 1);
        trimStr(line);
        if (line.length() == 0) continue;

        // --- RESPONSE HANDLERS ---

        // Handle GPS Location Response
        if (line.indexOf("+QGPSLOC:") >= 0) {
            parseQGPSLOC_Kalman(line);
        }
        // Handle Signal Strength Response
        else if (line.indexOf("+CSQ:") >= 0) {
            int p = line.indexOf("+CSQ:");
            if (p >= 0) {
                int comma = line.indexOf(',', p);
                if (comma > p) {
                    int n = line.substring(p + 5, comma).toInt();
                    if (n == 99) cached_rssi = 0;                      // Unknown
                    else if (n >= 0 && n <= 31) cached_rssi = -113 + 2 * n; // Convert to dBm
                }
            }
        }
        // Handle MQTT Publish Confirmation
        else if (line.indexOf("+QMTPUBEX:") >= 0) {
            // Check for +QMTPUBEX: 0,0,0 (Success)
            Serial.println("Pub Status: " + line);
            lastPubAckMs = millis(); // Track successful publish
            state = P_IDLE;          // Reset state to allow next publish
        }
    }

    // 3. Periodic RSSI Poll
    pollAndCacheRssi();

    // 4. Periodic GPS Poll
    if (millis() - lastGpsPollTs >= GPS_POLL_MS) {
        lastGpsPollTs = millis();
        if (state == P_IDLE) {
            atSend("AT+QGPSLOC?");
        }
    }

    // ================= MQTT WATCHDOG =================
    // If no publish acknowledgement for 2 minutes, restart MQTT
    if (millis() - lastPubAckMs > 120000) { 
        Serial.println("MQTT inactive, reconnecting...");
        //  reset states before reconnect
    mqtt_connected   = false;
    mqtt_internet_up = false;
    mqtt_sim_registered = false;
        MQTT_begin(MODEM_BAUD);
        lastPubAckMs = millis();
    }
}

// =================================================================================
//                                    GETTERS
// =================================================================================

double MQTT_getLatitude() {
    return disp_lat;
}

double MQTT_getLongitude() {
    return disp_lon;
}

int MQTT_getSatellites() {
    return 8; // Placeholder: Quectel text mode doesn't always provide sat count in LOC
}

int MQTT_getFixQuality() {
    return gps_fix_quality;
}


// =================================================================================
//                          MQTT STATUS GETTERS (FOR LED)
// =================================================================================

bool MQTT_isSimRegistered() {
    return mqtt_sim_registered;
}

bool MQTT_isInternetUp() {
    return mqtt_internet_up;
}

bool MQTT_isConnected() {
    return mqtt_connected;
}
