/*
 *  Tentative to roughly position a an object in a 3D field of WIFI
 *  based on the scan of the different RSSI
 *  
 */

////////////////////////// External libraries //////////////////////
#include "ESP8266WiFi.h"

////////////////////////// Global constants ////////////////////////
const int N_DIMS = 3; // 3D or 2D space

const float GAMMA = 0.01; // learning rate
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
  {  0.0,  7.0,  0.0},
  {  4.0,  0.3,  1.3},
  {  7.0,  7.0,  1.3},
  {  0.0,  0.5,  2.1}
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
  {  0.0,  7.0}, // x min, x max
  {  0.0,  7.0}, // y min, y max
  {  0.0,  2.5}  // z min, z max
};
                               

// Signal calibration  0m,    10m for each ground reciver  
//const float calibR[NposR][2]= {{     -17,    -0},
//                               {     -17,    -0},
//                               {     -10,    -0},
//                               {      -8,    -0}};


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

void foo(float a[N_DIMS]) {
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
      }
  } 
}

//float calcMaxDist() {
//  float d = 0;
//  for(int i=0; i<3; i++) d += pow(posMinMax[i][1]-posMinMax[i][0], 2);
//  return sqrt(d);
//}
//
//float maxDist = calcMaxDist();
//
//
//float calcNorm(float* vect) {
//    float norm = 0;
//    for(int i=0; i < sizeof(vect); i++) norm += pow(vect[i],2);
//    return sqrt(norm);
//}
//
//float calcDist(int reciverID, float rssi, int channel) {
//  float attenuation      = rssi - calibR[reciverID][0];
//  float wavelengthVac    = c / (channels[channel-1]*1000000.0);
//  float dist             = (pow(attenuation/20.0, 10.0)*wavelengthVac)/(4*pi);
//  if(dist > maxDist) { 
//    Serial.print("\n WARNING: dist:");
//    Serial.print(dist);
//    Serial.print("truncated to maxDist:"); 
//    Serial.println(maxDist);
//    dist = maxDist;    // limit max distance to possible max distance
//  }
//  Serial.print("\t Att:");
//  Serial.print(attenuation);
//  return(dist);                   
//}
//
//float* calcPosError(float* pos, float* dists) {
//  float distEr[NposR][3],dEr[NposR];
//  for(int i=0;i<NposR;i++){
//    Matrix.Subtract(pos, posR[i], 1, 3, distEr[i]);
//    dEr[i] = calcNorm(distEr[i]);
//    //dEr[i] = dists[i] - sqrt(pow(pos[0]-posR[i][0],2) + pow(pos[1]-posR[i][1],2) + pow(pos[2]-posR[i][2],2) );
//  }
//  //Serial.println("Dist error");
//  //for(int k =0;k<4;k++) Serial.println(dEr[k]);
//  //Matrix.Print(   (float*)pos,       4, 1, "Dist error");
//  return dEr;
//}

//////////////////////// Setup & Loop ////////////////////////

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
   
  if (numberFoundNetworks == 0) {
    Serial.println("No networks found");
    return;
  }
  
  Serial.print(numberFoundNetworks);
  Serial.println(" networks found");

  float distances[NUMBER_BEACONS]; // Distances to beacons
  int numberFoundBeacons = 0;
  for (int i=0; i<numberFoundNetworks; ++i) { // Print SSID and RSSI for each network found
      
      for(int j=0; j<NUMBER_BEACONS; ++j) {
        distances[j] = NA;
        
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
    Serial.println("Too less beacons detected");
    return;
  }

  float p[N_DIMS]; // Fish position
  Serial.println("Initialize fish position");
  for(int i=0; i<N_DIMS; i++) {
    p[i] = random(POSITION_MIN_MAX[i][0], POSITION_MIN_MAX[i][1]);    // randomly define the initial position within the bounds
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
  
//  err  = calcPosError(pos, dists);
//  Serial.println("Initial Error");
//  for( k =0;k<NposR;k++) Serial.println(err[k]);
//  //Matrix.Print(    (float*)err,     1, 4, "Initial Error");
//  do {
//    for(int i=0; i<3; i++) Dpos[i] = 0.0;                  // initialise the sum of delta arrays with zeros
//    for(int l=0; l<NposR ;l++) {
//      Matrix.Subtract(pos, posR[l], 1, 3, DposR[l]);       // Difference in pos between object & reviver
//      NDposR[l] = calcNorm(DposR[l]);                      // Norm of the difference
//      Matrix.Copy(   DposR[l], 1, 3,    NNDposR[l]);
//      Matrix.Scale(NNDposR[l], 1, 3, 1.0/NDposR[l]);       // divide difference by norm to get unit vector
//      Matrix.Scale(NNDposR[l], 1, 3,      dists[l]);       // multiply by measured distance
//      Matrix.Subtract(pos, NNDposR[l], 1, 3,PNNDposR[l]);  // pos - the above
//      Matrix.Add(Dpos, PNNDposR[l], 1, 3, Dpos);           // somme des ecarts
//    } 
//    //Matrix.Print(   (float*)pos,       1, 3, "position of  fish");
//    //Matrix.Print(   (float*)dists, NposR, 1, "Distance to recivers");
//    //Matrix.Print(   (float*)DposR, NposR, 3, "Delta position of reciver to fish");
//    //Matrix.Print(  (float*)NDposR, NposR, 1, "Norm of Delta position of reciver to fish");
//    //Matrix.Print( (float*)NNDposR, NposR, 3, "Normalised Delta position of reciver to fish multiplied by measured distance");
//    //Matrix.Print((float*)PNNDposR, NposR, 3, "Diff with pos");
//    //Matrix.Print(    (float*)Dpos,     1, 3, "Summ of Diff with pos");
//
//    Matrix.Scale(Dpos, 1, 3,      relax*1/NposR);    // divide by number of pos & apply relaxiation factor
//    Matrix.Subtract(pos, Dpos, 1, 3, pos);           // update position
//    err  = calcPosError(pos, dists);                 // update Error
//    serr = 0;
//    for( k =0;k<4;k++) serr += pow(err[k],2);
//    serr = sqrt(serr);
//    //Matrix.Print(    (float*)pos,     1, 3, "New Pos");
//    //Matrix.Print(    (float*)err,     4, 1, "New Error");
//    Serial.println("Delta Pos");
//    for( k =0;k<3;k++) {
//      Serial.print(Dpos[k]);
//      Serial.print(",\t");
//    }
//    Serial.println("");
//    Serial.println("New Pos");
//    for( k =0;k<3;k++) {
//      Serial.print(pos[k]);
//      Serial.print(",\t");
//    }
//    Serial.println("");
//    //Serial.println("New Error");
//    //for( k =0;k<NposR;k++) Serial.println(err[k]);
//    Serial.print("XXXXXX Sum of Error:");
//    Serial.println(serr);
//    z++;
//  } while(z<50);

}
