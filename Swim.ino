/*
Swim
*/

#define LEFT   16   // D0 
#define RIGHT  05   // D1 


void setup() {
  pinMode(LEFT, OUTPUT);    
  pinMode(RIGHT, OUTPUT);   
  
  }  


// the loop function runs over and over again forever
void loop() {
  digitalWrite(LEFT, LOW);   // Turn the Motor on 
  delay(200);                      // Wait 
  digitalWrite(LEFT, HIGH);  // Turn the Motor off 
  delay(1000);                      // Wait 
  digitalWrite(RIGHT, LOW);  // Turn the Motor on reverse
  delay(200);                      // Wait 
  digitalWrite(RIGHT, HIGH);  // Turn the Motor off by making the voltage HIGH
  delay(1000);                      // Wait 
}
