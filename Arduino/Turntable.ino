#include <Stepper.h>
#include <LiquidCrystal.h>

const int steps = 2048;
Stepper myStepper(steps, 8, 10, 9, 11);
LiquidCrystal lcd(2,3,4,5,6,7);
int menumode = 1;
int scroll = 0;
int joyx = 0;
int submenu = 0;
int inputvalue = 0;
String command = "";
int inputval = 0;
int intervals = 0;
int joyy = 0;
const int fullrot = 14336; //Based on values from original project
int def_speed = 10; //Default speed for Revolve mode
int buttonstate = 0;
#define x_pin A0 //Corresponds to wiring
#define y_pin A1 //Corresponds to wiring
#define SW 12
char *menus[] = {"Serial", "Manual", "Revolve"};
void setup() {
  myStepper.setSpeed(10);
  lcd.begin(16,2);
  lcd.print("Starting up...");
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
  delay(2000);
  lcd.clear();
}

void loop() {
  buttonstate = digitalRead(SW);
  joyy = analogRead(x_pin); //Flipped due to oreintation
  joyx = analogRead(y_pin); // Flipped due to oreintation
  //Serial mode
  if (menumode == 2) {
    if (submenu == 1) {
      lcd.setCursor(0,0);
      lcd.print("Sync Mode"); // Syncs with camera rig using Python
      if (joyx > 900) {
         submenu = 0;
         lcd.clear();
         Serial.print("exit");
      }
       if(Serial.available()) {
          command = Serial.readStringUntil("\n");
          inputval = command.toInt();
          myStepper.step(inputval);
          delay(500);
          Serial.println("Photo"); //Photo output to trigger camera
          delay(3000);
          Serial.flush();
        } 
    }
    else if (submenu == 2) {
      lcd.setCursor(0,0);
      lcd.print("Serial input"); //Default Serial mode
      if (joyx > 900) {
         submenu = 0;
         lcd.clear();
       }
      if(Serial.available()) {\
        command = Serial.readStringUntil("\n");
        inputval = command.toInt();
        if (0 == inputval) {
          Serial.println("Try something else"); //Can't divide by zero
        }
        else if(inputval > 150) {
          Serial.println("Try something smaller");
        }
        else {
          intervals = 14336/abs(inputval);
          if(intervals < 1437 && intervals > 96) {
            for(int i = 1; i <= inputval; i++) {
              myStepper.setSpeed(10);
              myStepper.step(intervals);
              delay(500);
              Serial.println("Photo");
              delay(3000);
            }
          }
        }
      }
    }
    else { 
     lcd.setCursor(0,1);
     lcd.print("Down for Set");
     lcd.setCursor(0,0);
     lcd.print("Up for Sync");
     if (joyx > 900) {
      menumode = 1;
      lcd.clear();
     }
     if (joyy < 200) {
      submenu = 1;
      lcd.clear();
     } 
     if (joyy > 900) {
      submenu = 2;
      lcd.clear();      
     }
    }
  }
  // Manual Mode
  else if (menumode == 3) {
     lcd.setCursor(0,0);
     lcd.print("Use Up and");
     lcd.setCursor(0,1);
     lcd.print("Down to control");
     if (joyy > 900){
      myStepper.step(20);
     }
     if (joyy < 200) {
      myStepper.step(-20);
     }
     if (joyx > 900) {
      menumode = 1;
      lcd.clear();
     }
  }
  //Revolve
  else if (menumode == 4) {
     lcd.setCursor(0,0);
     lcd.print("Set Speed:");
     lcd.setCursor(0,1);
     char speedbuff[3];
     sprintf(speedbuff,"RPM: %-3d", def_speed);
     lcd.print(speedbuff);
     if (joyy < 100 && def_speed != 17) {
      def_speed += 1;
      delay(500);
     }
     if (joyy > 900 && def_speed != 1) {
      def_speed -= 1;
      delay(500);
     }
     if (buttonstate == LOW) {
      lcd.clear();
      lcd.print("Running...");
      myStepper.setSpeed(def_speed);
      myStepper.step(fullrot);
      lcd.clear();
     }
     if (joyx > 900) {
      menumode = 1;
     }
     
  }
  //Home Screen
  else {
  char bufferd[9];
    lcd.setCursor(0,0);
    lcd.print("Select Mode:");
    lcd.setCursor(0,1);
    sprintf(bufferd, "%-9s", menus[scroll]);
    lcd.print(bufferd);
    if (joyx < 100 or joyy < 100) {
      if (scroll == 2) {
        scroll = 0;
      }
      else {
        scroll += 1;
      }
      delay(500); 
    }
    else if (joyx > 900 or joyy > 900) {
      if (scroll == 0) {
        scroll = 2;
      }
      else {
        scroll -= 1;
      }
      delay(500);
    }
    else if (buttonstate == LOW) {
      menumode = scroll + 2;
      lcd.clear();
      delay(500);
    }    
  }
}
