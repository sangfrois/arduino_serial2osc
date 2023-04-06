/*  Pulse Sensor Amped 1.5    by Joel Murphy and Yury Gitman   http://www.pulsesensor.com

----------------------  Notes ----------------------  ----------------------
This code reads raw data from a Pulse Sensor connected to analog pin 0 and
sends it to a specified serial port.

Read Me:
https://github.com/WorldFamousElectronics/PulseSensor_Amped_Arduino/blob/master/README.md
 ----------------------       ----------------------  ----------------------
*/

//  Variables
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0

//  SLIP
const int END=192;
const int ESC=219; 
const int ESC_END=220;
const int ESC_ESC=221;

// SET THE SERIAL OUTPUT TYPE TO YOUR NEEDS
// PROCESSING_VISUALIZER works with Pulse Sensor Processing Visualizer
//      https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer
// SERIAL_PLOTTER outputs sensor data for viewing with the Arduino Serial Plotter
//      run the Serial Plotter at 115200 baud: Tools/Serial Plotter or Command+L
//static int outputType = SERIAL_PLOTTER;

void setup(){
  Serial.begin(115200);             // start serial communication
  // IF YOU ARE POWERING The Pulse Sensor AT VOLTAGE LESS THAN THE BOARD VOLTAGE,
  // UN-COMMENT THE NEXT LINE AND APPLY THAT VOLTAGE TO THE A-REF PIN
  // analogReference(EXTERNAL);
}

//  Where the Magic Happens
void loop(){
  int rawSignal = analogRead(pulsePin);
  Serial.write(END);   // start packet
  SLIPSerialWrite(rawSignal >> 8); // send MSB of the 2-byte integer
  SLIPSerialWrite(rawSignal & 255); // send LSB of the 2-byte integer
  Serial.write(END);   // end packet
  delay(3); // wait a little before reading analog input again
}

// SLIPSerialWrite function from https://github.com/bakercp/Arduino-Serial-SLIPLibrary
void SLIPSerialWrite(uint8_t b) {
  switch (b) {
    case END:
      Serial.write(ESC);
      Serial.write(ESC_END);
      break;
    case ESC:
      Serial.write(ESC);
      Serial.write(ESC_ESC);
      break;
    default:
      Serial.write(b);
      break;
  }
}
