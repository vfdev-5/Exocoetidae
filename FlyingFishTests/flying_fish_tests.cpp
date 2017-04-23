/*
 *  Tentative to roughly position a an object in a 3D field of WIFI
 *  based on the scan of the different RSSI
 *
 */

#include "moc_arduino.h"

////////////////////// Global constants ////////////////////////
const int N_DIMS = 3; // 3D or 2D space

const float GAMMA = 0.1; // learning rate
const int N_ITER = 100; // Number of iterations

const float NA = -999999.0;

const int NUMBER_BEACONS = 4;

const String BEACON_MAC_ADDRESSES[NUMBER_BEACONS] = { 
    "CC:61:E5:CC:14:D4",     //
    "BC:EE:B8:FA:38:46",     //
    "7B:8F:7E:1A:FE:34",     //
    "F4:CA:E5:BF:E4:68"      //
};     

// 3D coordinates of beacons
const float BEACONS_POSITION[NUMBER_BEACONS][3] = {
    {  -12.34,  0.0,  1.0},
    {  -2.345,  5.67,  1.0},
    {  3.456,  6.789,  1.0},
    {  13.456,  1.234,  1.0}
};

// Calibration constants for each beacon. Signal strenght dBm at 1 meter
const float BEACON_CALIBRATION[NUMBER_BEACONS] = {
    -120.0,
    -120.0,
    -120.0,
    -120.0
};

// Minimal and maximal posible fish position
const float POSITION_MIN_MAX[3][2] = {
    {  -15.0, 15.0}, // x min, x max
    {  -15.0, 15.0}, // y min, y max
    {  0.0,  2.5}  // z min, z max
};

//////////////////// Global functions ////////////////////////
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

        Serial.print("\n--- update est.pos : distances[i]=");
        Serial.println(distances[i]);
        if (!(distances[i] > NA)) {
            continue;
        }

        Serial.print("--- update est.pos : norm=");

        float vec[N_DIMS];
        // compute unit vector between testing position `p` and beacon position
        float norm = 0.0;
        for (int j=0; j<N_DIMS; j++) {
            vec[j] = p[j] - BEACONS_POSITION[i][j];
            norm += vec[j]*vec[j];
        }
        norm = sqrt(norm);
        Serial.print(norm);
        Serial.println("");

        Serial.print("---> v[j]=");
        for (int j=0; j<N_DIMS; j++) {
            Serial.print(vec[j] / norm);
            Serial.print(" ");
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
        Serial.print("- compPos: distances[i]=");
        Serial.print(distances[i]);
        Serial.println("");
        for (int j=0; j<N_DIMS; j++) {
            estimatedPositions[i][j] = NA;
        }
    }

    for (int i=0; i<N_ITER; i++) {
        Serial.print(i);
        Serial.print(" | ");
        _updateEstimatedPositions(estimatedPositions, p, distances);
        for (int ii=0; ii<NUMBER_BEACONS; ii++) {
            Serial.print("\n--- est.pos:");
            for (int jj=0; jj<N_DIMS; jj++) {
                Serial.print(estimatedPositions[ii][jj]);
                Serial.print(" ");
            }
        }
        Serial.println("");


        for (int j=0; j<N_DIMS; j++) {
            // Compute sum of errors: errors = sum(p[k] - estimated_positions[:,k])
            float delta = 0.0;
            for (int k=0; k<NUMBER_BEACONS; k++) {
                if (estimatedPositions[k][j] > NA) {
                    delta += p[j] - estimatedPositions[k][j];
                }
            }
            p[j] -= GAMMA * delta;
            Serial.print(p[j]);
            Serial.print(" ");
        }
        Serial.println(" ");
    }
}

//////////////////// Setup & Loop ////////////////////////

void setup() {
    Serial.begin(115200);
    Serial.println("----- Setup START -----");
    WiFi.mode(WIFI_STA);     // Set WiFi to station mode
    WiFi.disconnect();       // disconnect
    pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
    delay(100);
    Serial.println("----- Setup COMPLETE -----");
}

void loop() {
    DEBUG("-");
    Serial.println("---- scan start -----");
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);         // Turn the LED on (Note that LOW is the voltage level
    float t0 = millis();
    int numberFoundNetworks = WiFi.scanNetworks();               // get the number of networks found
    float t1 = millis();
    digitalWrite(LED_BUILTIN, HIGH);        // Turn the LED off by making the voltage HIGH
    Serial.print("---- scan done  ----- ");
    Serial.print(t1-t0);
    Serial.println("ms");

    DEBUG("--");

    if (numberFoundNetworks == 0) {
        Serial.println("No networks found");
        return;
    }

    DEBUG("---");

    Serial.print(numberFoundNetworks);
    Serial.println(" networks found");

    float distances[NUMBER_BEACONS]; // Distances to beacons
    int numberFoundBeacons = 0;

    for(int j=0; j<NUMBER_BEACONS; ++j) {
        distances[j] = NA;

        for (int i=0; i<numberFoundNetworks; ++i) { // Print SSID and RSSI for each network found

            if( WiFi.BSSIDstr(i).equals(BEACON_MAC_ADDRESSES[j]) ) {
                numberFoundBeacons++;
                distances[j] = RSSI2Distance(WiFi.RSSI(i), BEACON_CALIBRATION[j]);
                Serial.print("\t rid:");
                Serial.print(j);
                Serial.print("\t Dist:");
                Serial.print(distances[j]);
            }
        }
        Serial.println("");
        delay(10);
    }

    if (numberFoundBeacons < 2) {
        Serial.println("Not enough beacons detected");
        return;
    }

    float p[N_DIMS]; // Fish position
    Serial.println("Initialize fish position");
    for(int i=0; i<N_DIMS; i++) {
        // p[i] = random(POSITION_MIN_MAX[i][0], POSITION_MIN_MAX[i][1]);    // randomly define the initial position within the bounds
        p[i] = i < 2 ? 0.0 : 1.0;
        Serial.print(p[i]);
        Serial.print(",\t");
    }
    Serial.println("");

    Serial.println("----- Compute position -----");
    computePosition(p, distances);
    for(int i=0; i<N_DIMS; i++) {
        Serial.print(p[i]);
        Serial.print(",\t");
    }
    Serial.println("");

}


int main() {

    //setup();
    for (int i=0; i<1; i++) {
        loop();
    }
    return 0;
}

