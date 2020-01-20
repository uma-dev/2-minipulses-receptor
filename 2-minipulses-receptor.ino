/* ***************************************************************************************************** */

//   The next program computes the listed parameters of a OTDR system like and displays it on a LCD. 
//   All the parameters are adjusted to check with a STM32F446RE NUCLEO Board

//    x Frequency of refreshing      x Delay of arrive
//    x Pulse width                  x Amplitude/ attenuation 
//

//         °°°         °°°         °°°         °°°         °°°         °°°         °°°         °°°
//         ° °         ° °         ° °         ° °         ° °         ° °         ° °         ° °
//    °°°°°° °°°°°°°°°°° °°°°°°°°°°° °°°°°°°°°°° °°°°°°°°°°° °°°°°°°°°°° °°°°°°°°°°° °°°°°°°°°°° °°°°°°°
//                                                                                  Writed by Omar Roldan
/* *****************************************************************************************************/

#include <Adafruit_GFX.h>                                          // Core graphics library
#include <Adafruit_ST7735.h>                                       // Hardware-specific library for ST7735
#include <SPI.h>                                                   //      
#define TFT_CS        10                                           // Display definitions
#define TFT_RST        9                                           // 
#define TFT_DC         8                                           //
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);    //

#define pin   D0                                                   // Conections to the protection circuit 
#define pin2  D1                                                   //

uint32_t          channelRising, channelFalling, channel;
volatile uint32_t FrequencyMeasured, LastPeriodCapture = 0, CurrentCapture;
volatile float    PulseWidthMeasured, HighStateMeasured;
uint32_t          input_freq = 0;
HardwareTimer     *MyTim, *MyTim2;
volatile float    global=0;

void TIMINPUT_Capture_Rising_IT_callback(HardwareTimer*){
  // Frequency computation
  CurrentCapture = MyTim->getCaptureCompare(channelRising);
  
  if (CurrentCapture > LastPeriodCapture)  {
    FrequencyMeasured = input_freq / (CurrentCapture - LastPeriodCapture);
    PulseWidthMeasured = (HighStateMeasured * 100) / (CurrentCapture - LastPeriodCapture);
  }
  else if (CurrentCapture <= LastPeriodCapture)  {
    //0x1000 is max overflow value
    FrequencyMeasured = input_freq / (0x10000 + CurrentCapture - LastPeriodCapture);
    PulseWidthMeasured = (HighStateMeasured * 100) / (0x10000 + CurrentCapture - LastPeriodCapture);
  }
  LastPeriodCapture = CurrentCapture;
}


void TIMINPUT_Capture_Falling_IT_callback(HardwareTimer*){
  // PulseWidth computation 
  CurrentCapture = MyTim->getCaptureCompare(channelFalling);

  if (CurrentCapture > LastPeriodCapture)  {
    HighStateMeasured = CurrentCapture - LastPeriodCapture;
  }
  else if (CurrentCapture <= LastPeriodCapture)  {
    // 0x1000 is max overflow value 
    HighStateMeasured = 0x10000 + CurrentCapture - LastPeriodCapture;
  }
}

void printMenu(void){
    tft.initR(INITR_BLACKTAB);                                     // Init ST7735S chip, black tab
  tft.setRotation(1);                                              // Horizontal mode 
  tft.fillScreen(ST77XX_BLACK);                                    // Background color (dark)
  
  tft.setCursor(5, 20);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(true);
  tft.print("Freq:               [Hz]");

  tft.setCursor(5, 50);
  tft.print("Width:              [us]");

  tft.setCursor(5, 80);
  tft.print("Delay:              [us]");  
}


void setup(){
  printMenu();
  Serial.begin(115200);
  
  // Automatically retrieve TIM instance and channelRising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  channelRising         = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

  // channelRisings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channelRising1 is associated to channelFalling and channelRising3 is associated with channelRising4
  switch (channelRising) {
    case 1:
      channelFalling = 2; break;
    case 2:
      channelFalling = 1; break;
    case 3:
      channelFalling = 4; break;
    case 4:
      channelFalling = 3; break;
  }

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim = new HardwareTimer(TIM1);

  // Configure rising edge detection to measure frequency
  MyTim->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, pin);
  uint32_t PrescalerFactor = 3;
  MyTim->setPrescaleFactor(PrescalerFactor);
  MyTim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim->attachInterrupt(channelRising, TIMINPUT_Capture_Rising_IT_callback);
  MyTim->attachInterrupt(channelFalling, TIMINPUT_Capture_Falling_IT_callback);
  MyTim->resume();

  // Compute this scale factor only once
  input_freq = MyTim->getTimerClkFreq() / MyTim->getPrescaleFactor();

  channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin2), PinMap_PWM));
  MyTim2 = new HardwareTimer(TIM2);
  //MyTim2->setMode(channel, TIMER_INPUT_CAPTURE_RISING, pin2);
  uint32_t Prescaler2 = 1;
  MyTim2->setPrescaleFactor(Prescaler2);
  MyTim2->setOverflow(0x10000); 
  
  pinMode(pin, INPUT_PULLDOWN);
  pinMode(pin2, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(pin), firstPulseArrival, RISING );  
  attachInterrupt(digitalPinToInterrupt(pin2), secondPulseArrival, RISING );  
  MyTim2->resume();
}

void firstPulseArrival(void){
  MyTim2->resume();
}

void secondPulseArrival(void){
  MyTim2->pause();
  global = MyTim2->getCount();
  MyTim2->setCount(0);
}


void loop(){  
  Serial.print((String)"Frecuencia = " + int(FrequencyMeasured)+ " [Hz]");
  Serial.print((String)"\t\t Ancho del pulso = " + PulseWidthMeasured*10 + " [us]");
  Serial.println((String)" Desfase = " + global/90 + " [us]");
  delay(300);
}
