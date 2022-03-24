// Simple str&& test for Adafruit Dot Star RGB LED strip.
// This is a basic diagnostic tool, NOT a graphics demo...helps confirm
// correct wiring && tests each pixel's ability to display red, green
// && blue && to forward data down the line.  By limiting the number
// && color of LEDs, it's reasonably safe to power a couple meters off
// the Arduino's 5V pin.  DON'T try that with other code!

#include <Adafruit_NeoPixel.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 50 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define LED_PIN 2
Adafruit_NeoPixel strip(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno = pin 11 for data, 13 for clock, other boards are different).
//Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BRG);
const int pin1 = 4;
const int pin2 = 5;
const int pin3 = 6;
const int pin4 = 7;

void setup() {
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  pinMode(pin3, INPUT_PULLUP);
  pinMode(pin4, INPUT_PULLUP);
}

// Runs 10 LEDs at a time along strip, cycling through red, green && blue.
// This requires about 200 mA for all the 'on' pixels + 1 mA per 'off' pixel.

int      dot = 8, pattern = 0; // Index of first 'on' && 'off' pixels
uint32_t color = 0xFF0000;      // 'On' color (starts red)

//  strip.setPixelColor(head, color); // 'On' pixel at head
//  strip.setPixelColor(tail, 0);     // 'Off' pixel at tail
//  strip.show();                     // Refresh strip
//  delay(20);                        // Pause 20 milliseconds (~50 FPS)
//
//  if(++head >= NUMPIXELS) {         // Increment head index.  Off end of strip?
//    //head = 0;                     //  Yes, reset head index to start
//
//    if((color >>= 1) == 0)          //  Next color (R->G->B) ... past blue now?
//      color = 0xFF0000;             //   Yes, reset to red
//  }
//  if(++tail >= NUMPIXELS) tail = 0; // Increment, reset tail index
void bouncingYellow() {
  strip.show();
  color = 0x808000;
  dot = 8;
  for (int i = NUMPIXELS / 2, n = NUMPIXELS / 2; i < NUMPIXELS + dot; i++, n--) {
    strip.setPixelColor(i, color);
    strip.setPixelColor(n, color);
    strip.setPixelColor(i - dot, 0);
    strip.setPixelColor(n + dot, 0);
    strip.show();
  }
  for (int i = NUMPIXELS, n = 0; i > NUMPIXELS / 2; i--, n++) {
    strip.setPixelColor(i, color);
    strip.setPixelColor(n, color);
    strip.setPixelColor(i + dot, 0);
    strip.setPixelColor(n - dot, 0);
    strip.show();
  }
}
void white() {
  for (int i = 0; i >= NUMPIXELS; i++) {
    strip.setPixelColor(i, 0xFFFFFF);
  }
  strip.show();
}
void autoBegin() {
  color = 0xFFFF00;
  static int n = 5;
  if (n >= NUMPIXELS / 2) {
    for (int i; i <= NUMPIXELS; i++) {
      strip.setPixelColor(i, 0x00FF00);
      strip.show();
    }
  }
  else {

    for (int i = 0; i <= NUMPIXELS; i++) {
      strip.setPixelColor(i, color);
      if (i < NUMPIXELS - n and i > 0 + n) {
        strip.setPixelColor(i - 5, 0);
        strip.show();
        delay(17);
      }
    }

    for (int i = NUMPIXELS; i >= 0; i--) {
      strip.setPixelColor(i, color);
      if (i < NUMPIXELS - n and i > 0 + n) {
        strip.setPixelColor(i + 5, 0);
        strip.show();
        delay(17);
      }
    }
    n++;
  }
}
void teleOpBegin() {
  for (int i = 0; i <= NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x00FF00);
  }
  strip.show();
  delay(100);
  for (int i = 0; i <= NUMPIXELS; i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
  delay(100);
}
void climbTime() {
  for (int i = 0; i <= NUMPIXELS; i++) {
    strip.setPixelColor(i, 0xFF0000);
  }
  strip.show();
  delay(100);
  for (int i = 0; i <= NUMPIXELS; i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
  delay(100);
}
void balls(int balls) {
  if (balls == 0) {
    color = 0xFF0000;
  }
  if (balls == 1) {
    color = 0xFF7F00;
  }
  if (balls == 2) {
    color = 0x00FF00;
  }
  for (int i = 0; i <= NUMPIXELS; i++) {
    strip.setPixelColor(i, color);
    strip.show();
  }
}
void acquisitionOut(int balls) {
  if (balls == 0) {
    color = 0xFF0000;
  }
  if (balls == 1) {
    color = 0xFF7F00;
  }
  if (balls == 2) {
    color = 0x00FF00;
  }

  for (int i = NUMPIXELS / 2, n = NUMPIXELS / 2; i <= NUMPIXELS; i++, n--) {
    strip.setPixelColor(i, color);
    strip.setPixelColor(i + 5, 0xFFFFFF);
    strip.setPixelColor(n, color);
    strip.setPixelColor(n - 5, 0xFFFFFF);
    strip.show();
    delay(20);
  }
}


void firstBar() {
  uint32_t roygbiv[] = {0xFF0000, 0xFF7F00, 0xFFFF00, 0x00FF00, 0x0000FF, 0x9400D3};
  for (int i = 0; i <= NUMPIXELS * 2 + 10; i++) {
    color = roygbiv[0];
    strip.setPixelColor(i + 10, 0);
    strip.setPixelColor(i, color);
    color = roygbiv[1];
    strip.setPixelColor(i - 10, color);
    color = roygbiv[2];
    strip.setPixelColor(i - 20, color);
    color = roygbiv[3];
    strip.setPixelColor(i - 30, color);
    color = roygbiv[4];
    strip.setPixelColor(i - 40, color);
    color = roygbiv[5];
    strip.setPixelColor(i - 50, color);
    strip.setPixelColor(i - 60, 0);
    delay(20);
    strip.show();
  }
}
void secondBar() {
  static int r = 255;
  static int g = 0;
  static int b = 0;
  static int i = 0;

  for (int i = 0; i <= NUMPIXELS; i++) {

    strip.setPixelColor(i, r, g, b);

    strip.show();
    delay(50);
    if (r == 255 && g < 255 && b == 0) {
      g += 5;
    }
    if (r > 0 and g == 255 and b == 0) {
      r -= 5;
    }
    if (b < 255 && r == 0 && g == 255) {
      b += 5;
    }
    if (r == 0 && g > 0 && b == 255) {
      g -= 5;
    }
    if (r < 255 && g == 0 && b == 255) {
      r += 5;
    }
    if (r == 255 and g == 0 and b > 0) {
      b -= 5;
    }
  }
}

void thirdBar() {
  static int r = 255;
  static int g = 0;
  static int b = 0;
  static int i = 0;

  for (int i = 0; i <= NUMPIXELS; i++) {
    strip.setPixelColor(i, r, g, b);
    strip.show();

    if (r == 255 && g < 255 && b == 0) {
      g++;
    }
    if (r > 0 and g == 255 and b == 0) {
      r--;
    }
    if (b < 255 && r == 0 && g == 255) {
      b++;
    }
    if (r == 0 && g > 0 && b == 255) {
      g--;
    }
    if (r < 255 && g == 0 && b == 255) {
      r++;
    }
    if (r == 255 and g == 0 and b > 0) {
      b--;
    }
  }
}
void loop() {
  pattern = digitalRead(pin1) | digitalRead(pin2) << 1 | digitalRead(pin3) << 2 | digitalRead(pin4) << 3;

  switch (pattern) {
    case 0:
      bouncingYellow();
      break;

    case 1:
      autoBegin();
      break;

    case 2:
      teleOpBegin();
      break;

    case 3:
      balls(0);
      break;

    case 4:
      balls(1);
      break;

    case 5:
      balls(2);
      break;

    case 6:
      acquisitionOut(0);
      break;

    case 7:
      acquisitionOut(1);
      break;

    case 8:
      acquisitionOut(2);
      break;

    case 9:
      climbTime();
      break;

    case 10:
      firstBar();
      break;

    case 11:
      secondBar();
      break;

    case 12:
      thirdBar();
      break;

    case 15:
      white();
      break;

    default:
      bouncingYellow();
      break;
  }
}
