void setup() { 
 Serial.begin(115200); 
 Serial.println("setup"); 
} 
 
void loop() { 
 esp_sleep_enable_timer_wakeup(3000000); //3 seconds 
 int ret = esp_light_sleep_start(); 
 Serial.print("light_sleep:"); 
 Serial.println(ret); 
} 