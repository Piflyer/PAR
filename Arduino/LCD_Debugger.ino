#include <LiquidCrystal.h>
LiquidCrystal lcd(3, 2, 4, 5, 6, 7);
String command;
void setup() {
  Serial.begin(9600);
  Serial.println("Testing");
}

void loop() {
  if(Serial.available()){
        lcd.begin(16,2);
        command = Serial.readStringUntil('\n');
        lcd.print(command);
        delay(3000);
        lcd.setCursor(0, 1);
        lcd.clear();
        for(int i = 1; i <= command.toInt(); i++) {
          delay(3500);
          Serial.println("Photo");
        }
    }
}