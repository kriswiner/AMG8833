/* AMG88331IRArray_Ladybug
 *  
 *  Basic program to get 8 x 8 temperature array data from the AMG8833 I2C thermal sensor
 *  
 *  Intended to run on the Butterfly as host, but any host with an I2C bus should work well.  
 *  
 */
// This Ladybug native optimized version requires specific pins
//
#define sclk 13  // SCLK can also use pin 14
#define mosi 11  // MOSI can also use pin 7
#define cs   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define dc   5   //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define rst  4   // RST can use any pin
#define sdcs 3   // CS for SD card, can use any pin

#include <TFT_ST7735.h>
#include <SPI.h>
#include <Wire.h>

TFT_ST7735 tft = TFT_ST7735(cs, dc);
float p = 3.1415926f;

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

uint16_t setColor[8] = {BLACK, BLUE, RED, GREEN, CYAN, MAGENTA, YELLOW, WHITE};

// Status and configuration registers
#define AMG8833_PCTL     0x00
#define AMG8833_RST      0x01
#define AMG8833_FPSC     0x02
#define AMG8833_INTC     0x03
#define AMG8833_STAT     0x04
#define AMG8833_SCLR     0x05
#define AMG8833_AVE      0x07
#define AMG8833_INTHL    0x08
#define AMG8833_INTHH    0x09
#define AMG8833_INTLL    0x0A
#define AMG8833_INTLH    0x0B
#define AMG8833_IHYSL    0x0C
#define AMG8833_IHYSH    0x0D
#define AMG8833_TTHL     0x0E
#define AMG8833_TTHH     0x0F

//  Interrupt result registers
#define AMG8833_INT0     0x10  // threshold interrupt for pixels 0 - 7, etc
#define AMG8833_INT1     0x11
#define AMG8833_INT2     0x12
#define AMG8833_INT3     0x13
#define AMG8833_INT4     0x14
#define AMG8833_INT5     0x15
#define AMG8833_INT6     0x16
#define AMG8833_INT7     0x17

// 64, 16-bit words for the 8 x 8 IR array
#define AMG8833_DATA01L  0x80 // pixel 1 low byte
#define AMG8833_DATA01H  0x81 // pixel 1 high byte
//  ...
#define AMG8833_DATA64L  0xFE // pixel 64 low byte
#define AMG8833_DATA64H  0xFF // pixel 64 high byte

#define AMG8833_ADDRESS 0x68  // 0x68 when ADO = LOW, 0x69 when ADO = HIGH

float minTemp, maxTemp, Tambient, tmpTemp;
uint8_t rawData[128];   //raw data array
float temperatures[64]; //Contains the calculated temperatures of each pixel in the array
int16_t irData[64];     //Contains the raw IR data from the sensor
uint16_t color;
uint8_t rgb, red, green, blue;

// define pin connections
#define intPin  8
#define ledPin  13

uint8_t rgb_colors[600] =
{  0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   5,   0,   7,
   9,   0,  14,
  14,   0,  21,
  18,   0,  29,
  23,   0,  36,
  27,   0,  43,
  32,   0,  50,
  37,   0,  57,
  41,   0,  64,
  46,   0,  72,
  50,   0,  79,
  55,   0,  86,
  60,   0,  93,
  64,   0, 100,
  69,   0, 107,
  73,   0, 115,
  78,   0, 122,
  82,   0, 129,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  84,   6, 132,
  82,  12, 128,
  79,  17, 124,
  77,  23, 120,
  74,  29, 116,
  72,  35, 113,
  69,  40, 109,
  67,  46, 105,
  64,  52, 101,
  62,  58,  97,
  59,  64,  93,
  56,  69,  89,
  54,  75,  85,
  51,  81,  81,
  49,  87,  77,
  46,  92,  74,
  44,  98,  70,
  41, 104,  66,
  39, 110,  62,
  36, 115,  58,
  33, 121,  54,
  31, 127,  50,
  28, 133,  46,
  26, 139,  42,
  23, 144,  38,
  21, 150,  35,
  18, 156,  31,
  16, 162,  27,
  13, 167,  23,
  11, 173,  19,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
  20, 183,  14,
  32, 186,  14,
  43, 190,  13,
  55, 193,  12,
  67, 197,  11,
  79, 201,  11,
  90, 204,  10,
 102, 208,   9,
 114, 212,   9,
 126, 215,   8,
 137, 219,   7,
 149, 222,   6,
 161, 226,   6,
 173, 230,   5,
 184, 233,   4,
 196, 237,   4,
 208, 241,   3,
 220, 244,   2,
 231, 248,   1,
 243, 251,   1,
 255, 255,   0,
 255, 246,   0,
 255, 237,   0,
 255, 228,   0,
 255, 219,   0,
 255, 209,   0,
 255, 200,   0,
 255, 191,   0,
 255, 182,   0,
 255, 173,   0,
 255, 164,   0,
 255, 155,   0,
 255, 146,   0,
 255, 137,   0,
 255, 128,   0,
 255, 118,   0,
 255, 109,   0,
 255, 100,   0,
 255,  91,   0,
 255,  82,   0,
 255,  73,   0,
 255,  64,   0,
 255,  55,   0,
 255,  46,   0,
 255,  36,   0,
 255,  27,   0,
 255,  18,   0,
 255,   9,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 253,  13,  40,
 250,  27,  80,
 248,  40, 120,
 245,  53, 160,
 243,  67, 200,
 240,  80, 240,
 242, 102, 242,
 244, 124, 244,
 246, 146, 246,
 248, 168, 248,
 249, 189, 249,
 251, 211, 251,
 253, 233, 253,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255
};

enum mode {  // define IR Array operating mode
  normal_mode = 0, // default
  sleep_mode,
  standby_mode_60sec,
  standbymode_10sec
};

// Define configuration
  uint8_t mode = normal_mode; // set IR Array operation mode

  float thermistorTemp= 0.0f; // thermistor temperature
  
void setup() 
{
  pinMode(sdcs, INPUT_PULLUP);  // don't touch the SD card
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // start with led off (active HIGH)
  
  Serial.begin(38400);
  delay(2000);
  Serial.println("AMG8833 Grid-Eye 8 x 8 IR Array");
  
  tft.begin();   // initialize a ST7735S chip, black tab
  tft.setRotation(3); // 0, 2 are portrait mode, 1,3 are landscape mode
  Serial.println("initialize display");

  // Initialize I2C bus and check for devices on the bus
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  

  I2Cscan();

  writeByte(AMG8833_ADDRESS, AMG8833_RST, 0x3F);  // initialize with a reset
    
  writeByte(AMG8833_ADDRESS, AMG8833_PCTL, mode); // set operating mode

  writeByte(AMG8833_ADDRESS, AMG8833_FPSC, 0x00); // sample rate (0x00 = 10 fps or 0x01 = 1 fps)

}
/* End of setup*/

void loop() 
{
    thermistorTemp = 0.0625f * (float)readThermistor();
    Serial.print("Thermistor temperature = "); Serial.print(thermistorTemp, 2); Serial.println(" oC");

    readBytes(AMG8833_ADDRESS, AMG8833_DATA01L, 128, &rawData[0]);
    for(uint16_t ii = 0; ii < 64; ii++) {
      temperatures[ii] = (float) ((int16_t) ( (int16_t) rawData[2*ii + 1] << 8) | rawData[2*ii]);
      temperatures[ii] *=0.25f; // scale to get temperatures in degrees C
    }
    
    // Get min and max temperatures
    minTemp = 1000.0f;
    maxTemp =    0.0f;
    for(int y=0; y<8; y++){ //go through all the rows
    for(int x=0; x<8; x++){ //go through all the columns
      if(temperatures[y+x*8] > maxTemp) maxTemp = temperatures[y+x*8];
      if(temperatures[y+x*8] < minTemp) minTemp = temperatures[y+x*8];
    }
    }

    tft.fillScreen(BLACK); // clear the screen
    for(int y=0; y<8; y++){ //go through all the rows
    for(int x=0; x<8; x++){ //go through all the columns
 
      tmpTemp = temperatures[y+x*8];
      Serial.print(tmpTemp, 1); Serial.print(","); // use the serial monitor to plot the data, TFT diplay would be better
      if(x == 7) Serial.println(" "); 
      if(y+x*8 == 63) Serial.println(" "); 

      rgb =(uint8_t) (((tmpTemp - minTemp)/(maxTemp - minTemp)) * 199); // 0 - 199 = 200 possible rgb color values

      red   = rgb_colors[rgb*3] >> 3; // keep 5 MS bits
      green = rgb_colors[rgb*3 + 1] >> 2; // keep 6 MS bits
      blue  = rgb_colors[rgb*3 + 2] >> 3; // keep 5 MS bits
      color = red << 11 | green << 5 | blue; // construct rgb565 color for tft display

      tft.fillRect(x*10, y*10, 10, 10, color);
//      tft.drawRect(x*10, y*10, 10, 10, BLACK);  

    }
    }

      tft.setTextScale(0);
      tft.setTextColor(WHITE);
      tft.setCursor(10, 50 );
      tft.print("min T = "); tft.print((uint8_t) minTemp);
      tft.setCursor(10, 66 );
      tft.print("max T = "); tft.print((uint8_t) maxTemp);

      Serial.print("min T = "); Serial.println((uint8_t) minTemp);
      Serial.print("max T = "); Serial.println((uint8_t) maxTemp);
 
      tmpTemp = 0;
      
      digitalWrite(ledPin, HIGH); delay(500); digitalWrite(ledPin, LOW);
}


/* End of Loop*/

/*
************************************************************************************************************
* Useful Functions
************************************************************************************************************
*/
int16_t readThermistor()
{
 uint8_t rawData[2] = {0, 0};
 readBytes(AMG8833_ADDRESS, AMG8833_TTHL, 2, &rawData[0]);
 return (int16_t) (((int16_t) rawData[1] << 8) | rawData[0]);
}

// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    error = Wire.transfer(address, NULL, 0, NULL, 0);

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


       // I2C read/write functions for the AMG8833

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }
