/*
 *  Tentative to roughly position a an object in a 3D field of WIFI
 *  based on the scan of the different RSSI
 *
 */

////////////////////////// External libraries //////////////////////
#include "ESP8266WiFi.h"

////////////////////////// Global constants ////////////////////////

//#define VERBOSE

const int N_DIMS = 3; // 3D or 2D space

const float GAMMA = 0.1; // learning rate
const int N_ITER = 100; // Number of iterations

const float NA = -999999.0;

const int NUMBER_BEACONS = 4;

const String BEACON_MAC_ADDRESSES[NUMBER_BEACONS] = { 
    "5E:CF:7F:0F:79:76",     // Fish 1
    "5E:CF:7F:0F:78:71",     // Fish 4
    "5E:CF:7F:0F:79:33",     // Fish 6   
    "5E:CF:7F:0E:9A:39"      // Fish10
};     

// 3D coordinates of beacons
const float scale = 0.305;
const float BEACONS_POSITION[NUMBER_BEACONS][3] = {
    {  scale * 12,  scale * 11,  1.0},
    {  scale * 0.0,  scale * 11.0,  1.0},
    {  scale * 12, scale * 0.0,  1.0},
    {  scale * 0.0,  scale * 0.0,  1.0}
};

// Calibration constants for each beacon. Signal strenght dBm at 1 meter
const float BEACON_CALIBRATION[NUMBER_BEACONS] = {
    -45.0,
    -45.0,
    -45.0,
    -45.0
};

// Minimal and maximal posible fish position
const float POSITION_MIN_MAX[3][2] = {
    {  0.0,  7.0}, // x min, x max
    {  0.0,  7.0}, // y min, y max
    {  0.0,  2.5}  // z min, z max
};

//////////////////////// Global functions ////////////////////////
/*
 * Compute RSSI of an emitter given a distance
 *   rssi = -10 * n * log10(d) + A
 *   or
 *   d = 10^ ( -(rssi - A) / (10 * n))
 *
 *   :param d: distance from emitter in meters
 *   :param n: propagation constant or path-loss exponent i.e. 2.7 to 4.3 (Free space has n =2 for reference)
 *   :param a: received signal strength in dBm at 1 meter
 *
 *   http://electronics.stackexchange.com/questions/83354/calculate-distance-from-rssi
 */
float Distance2RSSI(float d, float a, float n=2.7) {
    return -10.0 * n * log(d) / log(10.0) + a;
}

/*
 * Compute distance to an emitter given RSSI
 *
 *   rssi = -10 * n * log10(d) + A
 *
 *   d = 10^ ( -(rssi - A) / (10 * n))
 *
 *   :param s: RSSI in dBm
 *   :param n: propagation constant or path-loss exponent i.e. 2.7 to 4.3 (Free space has n =2 for reference)
 *   :param a: received signal strength in dBm at 1 meter
 *
 *   http://electronics.stackexchange.com/questions/83354/calculate-distance-from-rssi
 */
float RSSI2Distance(float s, float a, float n=2.7) {
    return pow(10.0, -0.1 * (s - a)/n);
}


/*
 * Print Wifi info
 */
void printWifiInfo(int i) {
    String ssid  =  WiFi.SSID(i);
    Serial.print(i + 1);
    Serial.print("\tSSID: ");
    Serial.print(ssid);
    Serial.print("\tBSSID:");
    Serial.print(WiFi.BSSIDstr(i));
    Serial.print("\tChannel:");
    Serial.print(WiFi.channel(i));
    Serial.print(" Encription:");
    Serial.print(WiFi.encryptionType(i));
    Serial.print("\tRSSI:");
    Serial.print(WiFi.RSSI(i));
}

/*
 * Update estimated positions `estimatedPositions` of the fish given by each beacon
 * knowing `distances` between beacons and fish and a testing position `p`
 */
void _updateEstimatedPositions(float estimatedPositions[NUMBER_BEACONS][N_DIMS], 
                               float p[N_DIMS],
                               float distances[NUMBER_BEACONS]) {
    for (int i=0; i<NUMBER_BEACONS; i++) {

        if (!(distances[i] > NA)) {
            continue;
        }

        float vec[N_DIMS];
        // compute unit vector between testing position `p` and beacon position
        float norm = 0.0;
        for (int j=0; j<N_DIMS; j++) {
            vec[j] = p[j] - BEACONS_POSITION[i][j];
            norm += vec[j]*vec[j];
        }
        norm = sqrt(norm);
        
        for (int j=0; j<N_DIMS; j++) {
            estimatedPositions[i][j] = distances[i] * vec[j]/norm + BEACONS_POSITION[i][j];
        }
    }
}

/*
 * Compute fish position from distances to beacons
 *
 * :param p: inital position, output computed position
 * :param distances: distances to beacons
 *
 */
void computePosition(float p[N_DIMS], float distances[NUMBER_BEACONS]) {

    float estimatedPositions[NUMBER_BEACONS][N_DIMS];
    // Initialize estimated position
    for (int i=0; i<NUMBER_BEACONS; i++) {
        for (int j=0; j<N_DIMS; j++) {
            estimatedPositions[i][j] = NA;
        }
    }

    for (int i=0; i<N_ITER; i++) {
#ifdef VERBOSE
        Serial.print(i);
        Serial.print(" | ");
#endif
        _updateEstimatedPositions(estimatedPositions, p, distances);
        for (int j=0; j<N_DIMS; j++) {
            // Compute sum of errors: errors = sum(p[k] - estimated_positions[:,k])
            float delta = 0.0;
            for (int k=0; k<NUMBER_BEACONS; k++) {
                if (estimatedPositions[k][j] > NA) {
                    delta += p[j] - estimatedPositions[k][j];
                }
            }
            p[j] -= GAMMA * delta;
#ifdef VERBOSE
            Serial.print(p[j]);
            Serial.print(" ");
#endif
        }
#ifdef VERBOSE
        Serial.println("");
#endif
    }
}


//////////////////////// Setup & Loop ////////////////////////

float POSITION[N_DIMS]; // Fish position

void setup() {
  
    Serial.begin(115200);
    Serial.println("----- Setup START -----");
    WiFi.mode(WIFI_STA);     // Set WiFi to station mode
// ....    
//    WiFi.disconnect();       // disconnect
    pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
    delay(100);
    Serial.println("----- Setup COMPLETE -----");

    Serial.println("Initialize fish position");
    for(int i=0; i<N_DIMS; i++) {
        POSITION[i] = random(POSITION_MIN_MAX[i][0], POSITION_MIN_MAX[i][1]);    // randomly define the initial position within the bounds
        POSITION[i] += 0.5;
        Serial.print(POSITION[i]);
        Serial.print(",\t");
    }
    Serial.println("");


}

void loop() {

    Serial.println("---- scan start -----");
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);         // Turn the LED on (Note that LOW is the voltage level
    float t0 = millis();
    int numberFoundNetworks = WiFi.scanNetworks();               // get the number of networks found
    float t1 = millis();
    digitalWrite(LED_BUILTIN, HIGH);        // Turn the LED off by making the voltage HIGH
    Serial.print("---- scan done  ----- ");
    Serial.print(t1-t0);
    Serial.println("ms");

    if (numberFoundNetworks == 0) {
        Serial.println("No networks found");
        return;
    }

    Serial.print(numberFoundNetworks);
    Serial.println(" networks found");

    float distances[NUMBER_BEACONS]; // Distances to beacons
    int numberFoundBeacons = 0;

    for(int j=0; j<NUMBER_BEACONS; ++j) {
        distances[j] = NA;

        for (int i=0; i<numberFoundNetworks; ++i) { // Print SSID and RSSI for each network found

            if( WiFi.BSSIDstr(i).equals(BEACON_MAC_ADDRESSES[j]) ) {
                numberFoundBeacons++;
                float rssi = WiFi.RSSI(i);
                distances[j] = RSSI2Distance(rssi, BEACON_CALIBRATION[j]);
                Serial.print("\t rid:");
                Serial.print(j);
                Serial.print("\t RSSI:");
                Serial.print(rssi);
                Serial.print("\t Dist:");
                Serial.print(distances[j]);
                Serial.println("");
            }
        }
        Serial.println("");
        delay(10);
    }

    if (numberFoundBeacons < 2) {
        Serial.println("Not enough beacons detected");

        for (int i=0; i<numberFoundNetworks; ++i) { // Print SSID and RSSI for each network found
            Serial.print("- ");
            Serial.print(WiFi.BSSIDstr(i));
            Serial.print(" | ");
            Serial.print(WiFi.SSID(i));
            Serial.println("");
        }
        Serial.println("");
        delay(10);       
        return;
    }

    Serial.println("----- Compute position -----");
    computePosition(POSITION, distances);
    for(int i=0; i<N_DIMS; i++) {
        Serial.print(POSITION[i]);
        Serial.print(",\t");
    }
    Serial.println("");

}

