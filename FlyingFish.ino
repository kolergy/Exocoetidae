/*
 *  Tentative to roughly position a an object in a 3D field of WIFI
 *  based on the scan of the different RSSI
 *  
 */
#include "ESP8266WiFi.h"

float posR[4][3] = {{  0,  0,  0},
                    {100,  0,  0},
                    {100,100,  0},
                    {  0,  0,100}};



float* calcPosError(float* pos, float* dists) {
  float distEr[sizeof(dists)];
  for(int i=0;i<sizeof(dists);i++){
    distEr[i] = dists[i] - sqrt(pow(pos[0]-posR[i][0],2) + pow(pos[1]-posR[i][1],2) + pow(pos[2]-posR[i][2],2) );
    Serial.print("Sensor[");
    Serial.print(i);
    Serial.print("] Err:");
    Serial.print(distEr[i]);
    Serial.print("\t");
  }
  Serial.println("");
  return distEr;
}


void setup() {
  Serial.begin(115200);
  Serial.println("----- Setup START -----");
  WiFi.mode(WIFI_STA);     // Set WiFi to station mode
  WiFi.disconnect();       // disconnect 
  delay(100);
  Serial.println("----- Setup COMPLETE -----");
}

void loop() {
  String        mySSID; 
  int           l, n ;
  unsigned long t0,t1;
  Serial.println("---- scan start -----");
  delay(2000);
  t0 = millis();
  n  = WiFi.scanNetworks();  // get the number of networks found
  t1 = millis();
  Serial.print("---- scan done  ----- ");
  Serial.print(t1-t0);
  Serial.println("ms");
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
      Serial.println("");
      delay(10);
    }
  }
  Serial.println("");
  float  pos[3]   = {10, 10, 10};
  float  dists[4] = {10, 90, 100, 90};
  float* err;
  err == calcPosError(pos, dists);
}
