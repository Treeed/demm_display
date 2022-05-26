/* date: 25.05.2022
   I2C SCL is blue
   SDA is black
*/

#include <Servo.h>
#include "FastLED.h"
#include <AvrI2c.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <EEPROM.h>

#define I2C_ADDRESS 0x3C

#define NUM_LEDS 2
#define DATA_PIN 5
#define LIGHT_PIN A7
#define SERVO_PIN 3
#define BUTTON_PIN 2

#define MAX_STATE 6
#define CYCLE_TIME 30
#define NUMBER_BYTES 9
#define SLAVE_ADRESS 8
#define SHUFFLE_TIME 3500
#define MAX_KMH 45
#define MIN_VOLTAGE 4

SSD1306AsciiAvrI2c oled;
AvrI2c i2c;
CRGB leds[NUM_LEDS];
Servo Tacho;

int button_state = 0;
unsigned int DisplayMode = 0;
unsigned long previousMillis = 0;
unsigned long previousDisplayMillis = 0;
int DisplayIndex = 0;
bool modeChange = true;

struct vesc_data {
    int Current;
    float Km;
    float Ah;
    unsigned int  KmAbs;
    float Voltage;
    float kmh;
    int Temp;
    unsigned long timestamp;
};

vesc_data vals;
vesc_data previousVals;

void Backlighting();
void SetTacho();
void displayAh();
void displayVoltage();
void displayTemp();
void displayKm();
void displayKmAbs();
void displayCurrent();
void DisplayShuffle();
void getState();
void getData();

void setup() {
    Serial.begin(2000000);
    Serial.println("startup");

    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(CalLite24);

    pinMode(LIGHT_PIN, INPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(BUTTON_PIN , INPUT);
    digitalWrite(BUTTON_PIN, HIGH);

    FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);

    oled.set2X();
    oled.clear();
    oled.print("DEMM");
    for (auto & led : leds) {
        led.r = 0;
        led.b = 255;
        led.g = 255;
    }
    FastLED.show();
    Tacho.attach(SERVO_PIN);
    Tacho.write(180);
    delay(600);
    Tacho.write(0);
    delay(600);
    Tacho.write(180);
}

void loop() {
    if (millis() - previousMillis > CYCLE_TIME) {
        previousMillis = millis();
        getState();
        Backlighting();
        getData();
        SetTacho();

        switch (DisplayMode) {
            case 0:
                DisplayShuffle();
                break;
            case 1:
                displayVoltage();
                break;
            case 2:
                displayAh();
                break;
            case 3:
                displayCurrent();
                break;
            case 4:
                displayKm();
                break;
            case 5:
                displayKmAbs();
                break;
            case 6:
                displayTemp();
                break;
            default:
                DisplayMode = 0;
                break;
        }

    }
}


void Backlighting() {
    if (analogRead(LIGHT_PIN) > 150) {
        for (auto & led : leds) {
            led.r = 155;
            led.b = 0;
            led.g = 255;
        }
    }
    else {
        for (auto & led : leds) {
            led.r = 0;
            led.b = 0;
            led.g = 0;
        }
    }
    FastLED.show();
}

void SetTacho() {
    vals.kmh = constrain(vals.kmh, 0, MAX_KMH);
    Tacho.write(map(vals.kmh, 0, MAX_KMH, 180, 0));

}

void displayAh() {
    if ((vals.Ah != previousVals.Ah) or modeChange) {
        previousVals.Ah = vals.Ah;
        modeChange = false;
        oled.clear();
        oled.set2X();
        oled.print(vals.Ah);
        oled.set1X();
        oled.print("A");
        oled.setCursor(113, 3);
        oled.print("h");
    }
}
void displayVoltage() {
    if ((vals.Voltage != previousVals.Voltage)  or modeChange) {
        previousVals.Voltage = vals.Voltage;
        modeChange = false;
        oled.clear();
        oled.set2X();
        oled.print(vals.Voltage);
        oled.print(" V");
    }
}

void displayTemp() {
    if ((vals.Temp != previousVals.Temp)  or modeChange) {
        previousVals.Temp = vals.Temp;
        modeChange = false;
        oled.clear();
        oled.set2X();
        oled.print(vals.Temp);
        oled.print(" C");
    }
}

void displayKm() {
    if ((vals.Km != previousVals.Km)  or modeChange) {
        previousVals.Km = vals.Km;
        modeChange = false;
        oled.clear();
        oled.set2X();
        oled.print(vals.Km);
        oled.set1X();
        oled.print(" k");
        oled.setCursor(110, 3);
        oled.print("m");
    }
}

void displayKmAbs() {
    if ((vals.KmAbs != previousVals.KmAbs)  or modeChange) {
        previousVals.KmAbs = vals.KmAbs;
        modeChange = false;
        oled.clear();
        oled.set1X();
        oled.println(vals.KmAbs + (unsigned int)vals.Km);
        oled.print("km   total");
    }
}

void displayCurrent() {
    if ((vals.Current != previousVals.Current) or modeChange) {
        previousVals.Current = vals.Current;
        modeChange = false;
        oled.clear();
        oled.set2X();
        oled.print(vals.Current);
        oled.print(" A");
    }
}

void DisplayShuffle() {
    if (millis() - previousDisplayMillis > SHUFFLE_TIME) {
        previousDisplayMillis = millis();
        modeChange = true;
        if (DisplayIndex >= (MAX_STATE)) {
            DisplayIndex = 0;
        }
        else {
            DisplayIndex++;
        }
    }
    switch (DisplayIndex) {
        case 0:
            displayTemp();
            break;
        case 1:
            displayVoltage();
            break;
        case 2:
            displayAh();
            break;
        case 3:
            displayCurrent();
            break;
        case 4:
            displayKm();
            break;
        case 5:
            displayKmAbs();
            break;
        default:
            displayVoltage();
            break;
    }
}

void getState() {
    if (!digitalRead(BUTTON_PIN)) {
        if (button_state > 2) {
            button_state = 0;
            modeChange = true;
            if (DisplayMode == MAX_STATE) {
                DisplayMode = 0;
                oled.clear();
                oled.set2X();
                oled.print("Shuffle");
                delay(500);
            }
            else {
                DisplayMode++;
            }
        }
        else {
            button_state++;
        }
    }
    else {
        button_state = 0;

    }
}

void getData(){
    const int baseInputVoltage = 2;
    const int baseMotorTemp = -20;
    const int AhFactor = 10;
    const int voltFactor = 100;
    const int kmhFactor = 3;
    const int kmFactor = 100;



    //extra byte for checksum
    i2c.start((SLAVE_ADRESS << 1) | 1);

    uint8_t receivedBytes[NUMBER_BYTES];
    uint8_t sum = 0;
    for(auto & received : receivedBytes){
        if(!i2c.read(&received, false)){
            Serial.println("bad response");
            return;
        }
        sum += received;
    }

    uint8_t csum;
    if(!i2c.read(&csum, true)){
        Serial.println("bad response");
        return;
    }
    i2c.stop();

    if(csum != sum){
        Serial.println("bad checksum");
        return;
    }

    vals.Current = receivedBytes[0];
    vals.Km = (float) (receivedBytes[1] + ((uint16_t)receivedBytes[2] << 8))/kmFactor;
    vals.Ah = (float)receivedBytes[3]/AhFactor;
    vals.KmAbs = (receivedBytes[4] + ((uint16_t)receivedBytes[5] << 8));
    vals.Voltage = (float)receivedBytes[6]/voltFactor+baseInputVoltage;
    vals.kmh = (float)receivedBytes[7]/kmhFactor;
    vals.Temp = receivedBytes[8]+baseMotorTemp;
    vals.timestamp = millis();
}