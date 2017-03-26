/*
 *  Tentative to roughly position a an object in a 3D field of WIFI
 *  based on the scan of the different RSSI
 *  
 */
#include "ESP8266WiFi.h"
#include "MatrixMath.h"

const float relax           = 0.5; // relaxation factor pos update

const int    NposR          = 4;


const String MAC[NposR]     = { "CC:61:E5:CC:14:D4",     // 
                                "BC:EE:B8:FA:38:46",     //                    
                                "7B:8F:7E:1A:FE:34",     // 
                                "F4:CA:E5:BF:E4:68"};    // 

      float posR[NposR][3]  = {{  0.0,  7.0,  0.0},
                               {  4.0,  0.3,  1.3},
                               {  7.0,  7.0,  1.3},
                               {  0.0,  0.5,  2.1}};

const float posMinMax[3][2] = {{  0.0,  7.0},
                               {  0.0,  7.0},
                               {  0.0,  2.5}};
                               
               // Signal calibration  0m,    10m for each ground reciver  
const float calibR[NposR][2]= {{     -17,    -0},
                               {     -17,    -0},
                               {     -10,    -0},
                               {      -8,    -0}};
                      
const float pi              = 3.14159265359;
const float c               = 299792458;                   // m/s Speed of light in vacum
const int   channels[14]    = {2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472, 2484 }; // WiFi channels frequency in MHz

float calcMaxDist() {
  float d = 0;
  for(int i=0; i<3; i++) d += pow(posMinMax[i][1]-posMinMax[i][0], 2);
  return sqrt(d);
}

float maxDist = calcMaxDist();


float calcNorm(float* vect) {
    float norm = 0;
    for(int i=0; i < sizeof(vect); i++) norm += pow(vect[i],2);
    return sqrt(norm);
}

float calcDist(int reciverID, float rssi, int channel) {
  float attenuation      = rssi - calibR[reciverID][0];
  float wavelengthVac    = c / (channels[channel-1]*1000000.0);
  float dist             = (pow(attenuation/20.0, 10.0)*wavelengthVac)/(4*pi);
  if(dist > maxDist) { 
    Serial.print("\n WARNING: dist:");
    Serial.print(dist);
    Serial.print("truncated to maxDist:"); 
    Serial.println(maxDist);
    dist = maxDist;    // limit max distance to possible max distance
  }
  Serial.print("\t Att:");
  Serial.print(attenuation);
  return(dist);                   
}

float* calcPosError(float* pos, float* dists) {
  float distEr[NposR][3],dEr[NposR];
  for(int i=0;i<NposR;i++){
    Matrix.Subtract(pos, posR[i], 1, 3, distEr[i]);
    dEr[i] = calcNorm(distEr[i]);
    //dEr[i] = dists[i] - sqrt(pow(pos[0]-posR[i][0],2) + pow(pos[1]-posR[i][1],2) + pow(pos[2]-posR[i][2],2) );
  }
  //Serial.println("Dist error");
  //for(int k =0;k<4;k++) Serial.println(dEr[k]);
  //Matrix.Print(   (float*)pos,       4, 1, "Dist error");
  return dEr;
}


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
  String        mySSID; 
  int           l, n, z, k;
  unsigned long t0,t1;
  float         pos[3], Dpos[3], serr;
  float         dists[NposR], NDposR[NposR], DposR[NposR][3], NNDposR[NposR][3], PNNDposR[NposR][3];
  float*        err, dist;
  
  Serial.println("---- scan start -----");
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);         // Turn the LED on (Note that LOW is the voltage level
  t0 = millis();
  n  = WiFi.scanNetworks();               // get the number of networks found
  t1 = millis();
  digitalWrite(LED_BUILTIN, HIGH);        // Turn the LED off by making the voltage HIGH
  Serial.print("---- scan done  ----- ");
  Serial.print(t1-t0);
  Serial.println("ms");
  int NGndR = 0;
  if (n == 0) Serial.println("no networks found");
  else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) { // Print SSID and RSSI for each network found
      mySSID  =  WiFi.SSID(i);
      l       = mySSID.length();
      while(l < 20) {
        mySSID += " ";
        l       = mySSID.length();
      }
      Serial.print(i + 1);
      Serial.print("\tSSID: ");
      Serial.print(mySSID);
      Serial.print("\tBSSID:");
      Serial.print(WiFi.BSSIDstr(i));
      Serial.print("\tChannel:");
      Serial.print(WiFi.channel(i));
      Serial.print(" Encription:");
      Serial.print(WiFi.encryptionType(i));
      Serial.print("\tRSSI:");
      Serial.print(WiFi.RSSI(i));
      for(int j=0; j<NposR; j++) {
        //Serial.println(WiFi.BSSIDstr(i));
        //Serial.println(MAC[j]);
        if(WiFi.BSSIDstr(i).equals(MAC[j])) {
          NGndR++;
          dist     = calcDist(j, WiFi.RSSI(i), WiFi.channel(i));
          dists[j] = dist;
          Serial.print("\t rid:");
          Serial.print(j);
          Serial.print("\t Dist:");
          Serial.print(dist);
        }
      }
      Serial.println("");
      delay(10);
    }
  }
  Serial.println("Initial Pos");
  if(NGndR > 3) {
    for(int n=0; n<3; n++) {
      pos[n] = random(posMinMax[n][0], posMinMax[n][1]);    // randomly define the initial position within the bounds
      Serial.print(pos[n]);
      Serial.print(",\t");
    }
    Serial.println("");
    
    z = 0;
    Serial.println("----- Positionning -----");
    err  = calcPosError(pos, dists);
    Serial.println("Initial Error");
    for( k =0;k<NposR;k++) Serial.println(err[k]);
    //Matrix.Print(    (float*)err,     1, 4, "Initial Error");
    do {
      for(int i=0; i<3; i++) Dpos[i] = 0.0;                  // initialise the sum of delta arrays with zeros
      for(int l=0; l<NposR ;l++) {
        Matrix.Subtract(pos, posR[l], 1, 3, DposR[l]);       // Difference in pos between object & reviver
        NDposR[l] = calcNorm(DposR[l]);                      // Norm of the difference
        Matrix.Copy(   DposR[l], 1, 3,    NNDposR[l]);
        Matrix.Scale(NNDposR[l], 1, 3, 1.0/NDposR[l]);       // divide difference by norm to get unit vector
        Matrix.Scale(NNDposR[l], 1, 3,      dists[l]);       // multiply by measured distance
        Matrix.Subtract(pos, NNDposR[l], 1, 3,PNNDposR[l]);  // pos - the above
        Matrix.Add(Dpos, PNNDposR[l], 1, 3, Dpos);           // somme des ecarts
      } 
      //Matrix.Print(   (float*)pos,       1, 3, "position of  fish");
      //Matrix.Print(   (float*)dists, NposR, 1, "Distance to recivers");
      //Matrix.Print(   (float*)DposR, NposR, 3, "Delta position of reciver to fish");
      //Matrix.Print(  (float*)NDposR, NposR, 1, "Norm of Delta position of reciver to fish");
      //Matrix.Print( (float*)NNDposR, NposR, 3, "Normalised Delta position of reciver to fish multiplied by measured distance");
      //Matrix.Print((float*)PNNDposR, NposR, 3, "Diff with pos");
      //Matrix.Print(    (float*)Dpos,     1, 3, "Summ of Diff with pos");

      Matrix.Scale(Dpos, 1, 3,      relax*1/NposR);    // divide by number of pos & apply relaxiation factor
      Matrix.Subtract(pos, Dpos, 1, 3, pos);           // update position
      err  = calcPosError(pos, dists);                 // update Error
      serr = 0;
      for( k =0;k<4;k++) serr += pow(err[k],2);
      serr = sqrt(serr);
      //Matrix.Print(    (float*)pos,     1, 3, "New Pos");
      //Matrix.Print(    (float*)err,     4, 1, "New Error");
      Serial.println("Delta Pos");
      for( k =0;k<3;k++) {
        Serial.print(Dpos[k]);
        Serial.print(",\t");
      }
      Serial.println("");
      Serial.println("New Pos");
      for( k =0;k<3;k++) {
        Serial.print(pos[k]);
        Serial.print(",\t");
      }
      Serial.println("");
      //Serial.println("New Error");
      //for( k =0;k<NposR;k++) Serial.println(err[k]);
      Serial.print("XXXXXX Sum of Error:");
      Serial.println(serr);
      z++;
    } while(z<50);
  }

}
