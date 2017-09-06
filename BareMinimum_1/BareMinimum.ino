// Call libraries
#include <Servo.h>
#include <PID_v1.h>
#include <max6675.h>

//PID IO
double SP_PID, iPID, oPID;
//PID Parameters
double Kp = 2, Ki = 5, Kd = 0;
//Servo IO
int angle;
//SPI
const int SPI_DI = 11;
const int SPI_DO = 12;
const int SPI_CLK = 13;
// Thermocouple
const int convTransCS = 9;
const int finTransCS = 10;
// Joystick
int joystickVarX = A0;
int joystickVarY = A1;
const int joystickPB = 4;
int joystickPB_state = 0;
// Program Variables
int convTemp;
int valX;
int valY;
unsigned long storeMilli500 = 0;
unsigned long storeMilli1000 = 0;
unsigned long compareMilli = 0;
int setPoint;
int modeSelect = 0;
// Buzzer
const int buzzerPin = 5;
int buzzer = LOW;

//Define objects that use libraries
Servo myServo;
PID myPID(&iPID, &oPID, &SP_PID, Kp, Ki, Kd, DIRECT);
MAX6675 convTrans(SPI_CLK, convTransCS, SPI_DO);
MAX6675 finTrans(SPI_CLK, finTransCS, SPI_DO);

void setup()
{
	Serial.begin(9600);
	myServo.attach(3);
// Assign DI DO
        pinMode(joystickPB, INPUT);
        pinMode(buzzerPin, OUTPUT);

	Serial.println("Welcome to BBQ Master 2000!");
// Delay for SPI comms and devices to initialize
	delay(500);
// Turn PID on - to move down into loop eventually to stop PID spinning up whilest not needed
        myPID.SetMode(AUTOMATIC);
        setPoint = 50;
}

void loop()
{
// Joystick control of setpoint
	valX = analogRead(joystickVarX);
	valX = map(valX, 0, 1010, -2, 2);
	valY = analogRead(joystickVarY);
	valY = map(valY, 0, 1010, -2, 2);

// Refresh Compare Milli
	compareMilli = millis();

// Set task timers to 0 if milli lapses
        if (storeMilli500 > (compareMilli + 5000)){
        storeMilli500 = 0;
        storeMilli1000 = 0;
	Serial.print("Reset Counters!");
        }

//
// 250ms Tasks
	if (compareMilli >= storeMilli500 + 500) {
	storeMilli500 = millis();
	
// Add Value of the mapped joystick to the setpoint
	setPoint = setPoint + valX;
// Limit it to a max of 199
		if (setPoint > 200){
			setPoint = 199;
		}
// Limit to a min of 1
		if (setPoint < 0){
			setPoint = 1;
		}
      
// Map Temp to PID
        iPID = convTrans.readCelsius();
	}
// End of 250ms task

//
// 1000ms Tasks
        if (compareMilli >= storeMilli1000 + 1000) {
	storeMilli1000 = millis();
	Serial.println("*******************");
	Serial.print("Setpoint C = ");
	Serial.println(SP_PID);
	Serial.print("Ambient Temp = ");
	Serial.println(iPID);
	Serial.print("Control Output = ");
	Serial.println(oPID);
	Serial.print("Alarm Temp C = ");
	Serial.println(SP_PID*1.3);
	Serial.print("Alarm Transmitter Temp C = ");
	Serial.println(finTrans.readCelsius());
	Serial.print("Buzzer Status = ");
	Serial.println(buzzer);

// Buzzer Control
        if (((finTrans.readCelsius()) > (1.3 * SP_PID)) and (buzzer == LOW)){
              buzzer = HIGH;
        }
      else
              buzzer = LOW;
      
          digitalWrite(buzzerPin, buzzer);  
    }

// End of 1000ms task

// Activate PID on PB

// Map Setpoint to PID
	SP_PID = setPoint;
// Compute PID
        myPID.Compute();
// Write adjustment to Servo motor
        myServo.write(oPID);

}
