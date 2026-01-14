// SPDX-FileCopyrightText: 2024 john park for Adafruit Industries
//
// SPDX-License-Identifier: MIT
/**
 * For USB MIDI Host Feather RP2040 with mini OLED FeatherWing and MIDI FeatherWing
 * Modified 12 Jun 2024 - @todbot -- added USB MIDI forwarding
 * Modified by @johnedgarpark -- added UART MIDI forwarding and display/message filtering
 * originally from: https://github.com/rppicomidi/EZ_USB_MIDI_HOST/blob/main/examples/arduino/EZ_USB_MIDI_HOST_PIO_example/EZ_USB_MIDI_HOST_PIO_example.ino
 */
 
 /* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 rppicomidi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/**
 * This demo program is designed to test the USB MIDI Host driver for a single USB
 * MIDI device connected to the USB Host port. It also
 * forwards MIDI received from the USB MIDI device to USB and UART MIDI devices.
 *
 * This program works with a single USB MIDI device connected via a USB hub, but it
 * does not handle multiple USB MIDI devices connected at the same time.
 * 
 *  Libraries (all available via library manager): 
 *  - MIDI -- https://github.com/FortySevenEffects/arduino_midi_library

 */
// Be sure to set the CPU clock to 120MHz or 240MHz before uploading to board
// USB Stack is TinyUSB
// Press A to change output MIDI channel
// Press B to change Program Change banks in groups of 8
// Press C for MIDI panic



//#define WIRE Wire1  //only if display needs it.
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for OLED FeatherWing 128x32

class Button {
  public:
    Button( int apin, int astate = LOW ): pin(apin), state(astate), lastState(LOW), debounce(0)  {}

    const int pin;
    int state;
    int lastState;
    unsigned long debounce;

    void init() {
        pinMode(pin, INPUT_PULLUP);
    }
    bool pressed() {
        const int debounceDelay(80);
        bool result = false;
        int  reading  = digitalRead( pin );

        if( reading != lastState ){
            debounce = millis();
        }
        if ( ( millis() - debounce) > debounceDelay ) {//Stays long in one state
            if( reading != state ) {
                state = reading;                        
                if ( state == LOW  ) {
                    result = true;
                }
            }
        }
        lastState = reading;
        return result;
    }
};


constexpr int potScale( 1024 );
constexpr int potFilter( 128 );
constexpr unsigned long debouncePeriod( 256 );
constexpr int tolerance( 32 );
constexpr int smallTolerance( 12 );

class Pot {
  public:
    Pot( int apin, int divisor=8, int astate=0 ): pin( apin ), state( astate ), lastState(0), debounce(0), div(divisor) {  }
    void init() {state = analogRead( pin )/8; lastState = state; debounce = millis(); }
    const int pin;
    int state;
    int lastState;
    int max;
    unsigned long maxTS;
    int min;
    unsigned long minTS;
    unsigned long debounce;
    int div; 
    //int read() { lastState = (potScale - potFilter*lastState + potFilter*analogRead(pin) ); state = lastState/potScale; return state; }
    //bool changed() { return read() == lastState; }
    int value() { return lastState/div; }
    bool moved() { 
        bool result( false );
        state = analogRead( pin );
        unsigned long currentTS( millis() );
        
        int maxDelta( state - max );
        int minDelta( min - state );

        if( maxDelta > tolerance ) { max = state+tolerance; min = state - tolerance; maxTS = currentTS; }
        else if( maxDelta > smallTolerance ) { max = state + smallTolerance; maxTS = currentTS; }
        else if( currentTS-maxTS > debouncePeriod ) { maxTS == currentTS; max -= 2; }

        if( minDelta > tolerance ) { min = state-tolerance; max = state+tolerance; minTS = currentTS; }
        else if( minDelta > smallTolerance ) { min = state - smallTolerance; minTS = currentTS; }
        else if( currentTS-minTS > debouncePeriod ) { minTS == currentTS; min += 2; }

        int average = (max+min)/2;
        if( lastState != average ) { lastState = average; result = true; }
        return result;
    }
};

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Display {
  public: 
    Display():disp( SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET ) {} ;

    void printProg( int userProgOffset, int program ) {
        disp.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        disp.setCursor(0,24);
        disp.printf("Progs %u-%u   [%u]   \r\n", userProgOffset, (userProgOffset + 7), (program + userProgOffset));
        disp.display();
    }
    void printCh( int chSrc, int chDest, int note, bool on ){
        disp.setCursor(0,12);
        disp.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        disp.setTextWrap(false);
        disp.printf("Ch %u > %u  Note %u %c  \r\n", chSrc, chDest, note, on?'o':'/');
        disp.display();
    }
    void printUserCh( int channel ) {
        mUserCh = channel;
        dirtyUserCh = true;
    }
    void printNote( int ch, int note, int vel ){
        mNCh = ch;
        mNNote = note;
        mVel = vel;
        dirtyNote = true;
    }
    void printPot( int potNo, int potVal){
        mPot = potNo,
        mPotValue = potVal;
        dirtyPot = true;
    }
    void printBtn( int button ){
        mBtn = button;
        dirtyBtn = true;
    }
    void printPanic(){
        dirtyPanic = true;
    }
    void printProgramShift( int offset ) {
        disp.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        disp.setCursor(0,24);
        disp.printf("Progs %u-%u \r\n", offset, (offset + 7));
        disp.display();
    }
    void printCC( int controller ){
        mCC = controller;
        dirtyCC = true;
    }
    void init(){
        if(!disp.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
           //Serial.println(F("SSD1306 allocation failed"));
           for(;;); // Don't proceed, loop forever
        }
        // Show initial display buffer contents on the screen --
        // the library initializes this with an Adafruit splash screen.
        disp.display();
        delay(2000); // Pause for 2 seconds
        // Clear the buffer
        disp.clearDisplay();
        disp.display();
    }
    void startupScreen(){
        disp.setTextSize(1);
        disp.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        disp.setCursor(0,0);
        disp.print("USB Host Router");
        // display.setCursor(0,12);
        // display.print("Ch x > 1  Note_ CC#_\r\n");
        // display.setCursor(0,24);
        // display.printf("Progs %u-%u     \r\n", userProgOffset, (userProgOffset + 7));
        disp.display();
    }

    void print(){
        disp.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        disp.setTextWrap(false);
        if( dirtyPanic ) {
            disp.setCursor(0,12);
            disp.printf("MIDI Panic   \r\n" );
            dirtyPanic = false;
        }
        if( dirtyCC ){
            disp.setCursor(0,12);
            disp.printf("CC# %u                \r\n", mCC );
            dirtyCC = false;
        }
        if( dirtyPot ){
            disp.setCursor( 72, 24 );
            disp.printf("P%i=%03u \r\n", mPot, mPotValue ); 
            dirtyPot = false;
        }
        if( dirtyBtn ){
            disp.setCursor(0,12);
            disp.printf("Button# %u                \r\n", mBtn );
            dirtyBtn = false; 
        }
        if( dirtyNote ){
            disp.setCursor(0,12);
            disp.printf("Ch %u Note %u vel %u\r\n", mNCh, mNNote, mVel );
            dirtyNote = false;
        } 
        if( dirtyUserCh ){
            disp.setCursor(0,12);
            disp.printf("Ch out %u          \r\n", mUserCh );
            dirtyUserCh = false; 
        }
        disp.display();
    }

  private:
    Adafruit_SSD1306 disp;
    bool   dirtyChannel;
    int mSrc;
    int mDest;
    int mChNote;
    bool mNoteOn;
    bool   dirtyUserCh;
    int mUserCh;
    bool   dirtyPot;
    int mPot;
    int mPotValue;
    bool   dirtyBtn;
    int mBtn;
    bool   dirtyCC;
    int mCC;
    bool   dirtyNote;
    int mNCh;
    int mNNote;
    int mVel;
    bool   dirtyPanic;
};



Button buttonA( 5);
Button buttonB( 6);
Button buttonC( 7);
//8 TXD1
//9 RXD1
Button buttonD(10);

Pot pot1( 26);
Pot pot2( 27);
Pot pot3( 28);
Pot pot4( 29);

Display display;

int userChannel = 1;  //1-16
int userProgOffset = 0;
   

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>  

#if defined(USE_TINYUSB_HOST) || !defined(USE_TINYUSB)
  #error "Please use the Menu to select Tools->USB Stack: Adafruit TinyUSB"
#endif


#define HOST_PIN_DP   16   // Pin used as D+ for host, D- = D+ + 1
#include "EZ_USB_MIDI_HOST.h"

#include "pio_usb.h"
#include "pio_usb_configuration.h"


// USB Host object
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_USBH_Host USBHost;

USING_NAMESPACE_MIDI
USING_NAMESPACE_EZ_USB_MIDI_HOST

RPPICOMIDI_EZ_USB_MIDI_HOST_INSTANCE(usbhMIDI, MidiHostSettingsDefault)

//MIDI_CREATE_INSTANCE(Adafruit_USBH_Host, usbMIDI, MIDIusb);  // USB MIDI
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDIuart);      // Serial MIDI over MIDI FeatherWing
//MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDIuart);

// static void sendAllUSBNoteOnff(){
//   for( uint8_t midiDevAddr(0); midiDevAddr < RPPICOMIDI_TUH_MIDI_MAX_DEV; ++midiDevAddr ){
//     auto intf( usbhMIDI.getInterfaceFromDeviceAndCable(midiDevAddr, usbhMIDI.getNumOutCables(midiDevAddr)));
//     if( intf ) {
//         intf->sendNoteOn(offNote, 0,1);
//     }
//   }
// }

static uint8_t midiDevAddr = 0;

static bool core0_booting = true;
static bool core1_booting = true;

/* MIDI IN MESSAGE REPORTING */
static void onMidiError(int8_t errCode)
{
    Serial.printf("MIDI Errors: %s %s %s\r\n", (errCode & (1UL << ErrorParse)) ? "Parse":"",
        (errCode & (1UL << ErrorActiveSensingTimeout)) ? "Active Sensing Timeout" : "",
        (errCode & (1UL << WarningSplitSysEx)) ? "Split SysEx":"");
}
int last_cc_cntrl = 1;

static void midiPanic()
{
    for (int i=0; i<128; i++)
    {
      //MIDIusb.sendNoteOff(i, 0, userChannel);
      MIDIuart.sendNoteOff(i, 0, userChannel);
      Serial.printf("note %u off\r\n", i);
      last_cc_cntrl = 0;  // dirty this
    }
}

static void onNoteOff(Channel channel, byte note, byte velocity)
{
    //MIDIusb.sendNoteOff(note, velocity, userChannel);
    MIDIuart.sendNoteOff(note, velocity, userChannel);
    Serial.printf("ch%u: Note off#%u v=%u\r\n", userChannel, note, velocity);

    display.printCh( channel, userChannel, note, false );

    last_cc_cntrl = 0; 
}

static void onNoteOn(Channel channel, byte note, byte velocity)
{
    //MIDIusb.sendNoteOn(note, velocity, userChannel);
    MIDIuart.sendNoteOn(note, velocity, userChannel);
    Serial.printf("ch%u: Note on#%u v=%u\r\n", userChannel, note, velocity);

    display.printCh( channel, userChannel, note, true);

    last_cc_cntrl = 0; 
    
}

static void onPolyphonicAftertouch(Channel channel, byte note, byte amount)
{
    Serial.printf("ch%u: PAT#%u=%u\r\n", userChannel, note, amount);
    //MIDIusb.sendAfterTouch(note, amount, userChannel);
    MIDIuart.sendAfterTouch(note, amount, userChannel);
}


static void onControlChange(Channel channel, byte controller, byte value)
{
    //MIDIusb.sendControlChange(controller, value, userChannel);
    MIDIuart.sendControlChange(controller, value, userChannel);
    Serial.printf("Ch %u CC#%u=%u\r\n", userChannel, controller, value);
    if (last_cc_cntrl != controller){
      display.printCC( controller );
      last_cc_cntrl = controller;
    }
    
}

static void onProgramChange(Channel channel, byte program)
{
    Serial.printf("ch%u: Prog=%u\r\n", userChannel, program);
    //MIDIusb.sendProgramChange(program + userProgOffset, userChannel);
    display.printProg( userProgOffset, program );
    MIDIuart.sendProgramChange(program + userProgOffset, userChannel);
    last_cc_cntrl = 0;  // dirty this
    
}

static void onAftertouch(Channel channel, byte value)
{
    Serial.printf("ch%u: AT=%u\r\n", userChannel, value);
    //MIDIusb.sendAfterTouch(value, userChannel);
    MIDIuart.sendAfterTouch(value, userChannel);
}

static void onPitchBend(Channel channel, int value)
{
    Serial.printf("ch%u: PB=%d\r\n", userChannel, value);
    //MIDIusb.sendPitchBend(value, userChannel);
    MIDIuart.sendPitchBend(value, userChannel);
}

static void onSysEx(byte * array, unsigned size)
{
    Serial.printf("SysEx:\r\n");
    unsigned multipleOf8 = size/8;
    unsigned remOf8 = size % 8;
    for (unsigned idx=0; idx < multipleOf8; idx++) {
        for (unsigned jdx = 0; jdx < 8; jdx++) {
            Serial.printf("%02x ", *array++);
        }
        Serial.printf("\r\n");
    }
    for (unsigned idx = 0; idx < remOf8; idx++) {
        Serial.printf("%02x ", *array++);
    }
    Serial.printf("\r\n");
}

static void onSMPTEqf(byte data)
{
    uint8_t type = (data >> 4) & 0xF;
    data &= 0xF;    
    static const char* fps[4] = {"24", "25", "30DF", "30ND"};
    switch (type) {
        case 0: Serial.printf("SMPTE FRM LS %u \r\n", data); break;
        case 1: Serial.printf("SMPTE FRM MS %u \r\n", data); break;
        case 2: Serial.printf("SMPTE SEC LS %u \r\n", data); break;
        case 3: Serial.printf("SMPTE SEC MS %u \r\n", data); break;
        case 4: Serial.printf("SMPTE MIN LS %u \r\n", data); break;
        case 5: Serial.printf("SMPTE MIN MS %u \r\n", data); break;
        case 6: Serial.printf("SMPTE HR LS %u \r\n", data); break;
        case 7:
            Serial.printf("SMPTE HR MS %u FPS:%s\r\n", data & 0x1, fps[(data >> 1) & 3]);
            break;
        default:
          Serial.printf("invalid SMPTE data byte %u\r\n", data);
          break;
    }
}

static void onSongPosition(unsigned beats)
{
    Serial.printf("SongP=%u\r\n", beats);
    //MIDIusb.sendSongPosition(beats);
    MIDIuart.sendSongPosition(beats);
}

static void onSongSelect(byte songnumber)
{
    Serial.printf("SongS#%u\r\n", songnumber);
    //MIDIusb.sendSongSelect(songnumber);
    MIDIuart.sendSongSelect(songnumber);
}

static void onTuneRequest()
{
    Serial.printf("Tune\r\n");
    //MIDIusb.sendTuneRequest();
    MIDIuart.sendTuneRequest();
}

static void onMidiClock()
{
    Serial.printf("Clock\r\n");
    //MIDIusb.sendClock();
    MIDIuart.sendClock();
}

static void onMidiStart()
{
    Serial.printf("Start\r\n");
    //MIDIusb.sendStart();
    MIDIuart.sendStart();
}

static void onMidiContinue()
{
    Serial.printf("Cont\r\n");
    //MIDIusb.sendContinue();
    MIDIuart.sendContinue();
}

static void onMidiStop()
{
    Serial.printf("Stop\r\n");
    //MIDIusb.sendStop();
    MIDIuart.sendStop();
}

static void onActiveSense()
{
    Serial.printf("ASen\r\n");
}

static void onSystemReset()
{
    Serial.printf("SysRst\r\n");
}

static void onMidiTick()
{
    Serial.printf("Tick\r\n");
}

static void onMidiInWriteFail(uint8_t devAddr, uint8_t cable, bool fifoOverflow)
{
    if (fifoOverflow)
        Serial.printf("Dev %u cable %u: MIDI IN FIFO overflow\r\n", devAddr, cable);
    else
        Serial.printf("Dev %u cable %u: MIDI IN FIFO error\r\n", devAddr, cable);
}

static void registerMidiInCallbacks()
{
    auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midiDevAddr, 0);
    if (intf == nullptr)
        return;
    intf->setHandleNoteOff(onNoteOff);                      // 0x80
    intf->setHandleNoteOn(onNoteOn);                        // 0x90
    intf->setHandleAfterTouchPoly(onPolyphonicAftertouch);  // 0xA0
    intf->setHandleControlChange(onControlChange);          // 0xB0
    intf->setHandleProgramChange(onProgramChange);          // 0xC0
    intf->setHandleAfterTouchChannel(onAftertouch);         // 0xD0
    intf->setHandlePitchBend(onPitchBend);                  // 0xE0
    intf->setHandleSystemExclusive(onSysEx);                // 0xF0, 0xF7
    intf->setHandleTimeCodeQuarterFrame(onSMPTEqf);         // 0xF1
    intf->setHandleSongPosition(onSongPosition);            // 0xF2
    intf->setHandleSongSelect(onSongSelect);                // 0xF3
    intf->setHandleTuneRequest(onTuneRequest);              // 0xF6
    intf->setHandleClock(onMidiClock);                      // 0xF8
    // 0xF9 as 10ms Tick is not MIDI 1.0 standard but implemented in the Arduino MIDI Library
    intf->setHandleTick(onMidiTick);                        // 0xF9
    intf->setHandleStart(onMidiStart);                      // 0xFA
    intf->setHandleContinue(onMidiContinue);                // 0xFB
    intf->setHandleStop(onMidiStop);                        // 0xFC
    intf->setHandleActiveSensing(onActiveSense);            // 0xFE
    intf->setHandleSystemReset(onSystemReset);              // 0xFF
    intf->setHandleError(onMidiError);

    auto dev = usbhMIDI.getDevFromDevAddr(midiDevAddr);
    if (dev == nullptr)
        return;
    dev->setOnMidiInWriteFail(onMidiInWriteFail);
}

/* CONNECTION MANAGEMENT */
static void onMIDIconnect(uint8_t devAddr, uint8_t nInCables, uint8_t nOutCables)
{
    Serial.printf("MIDI device at address %u has %u IN cables and %u OUT cables\r\n", devAddr, nInCables, nOutCables);
    midiDevAddr = devAddr;
    registerMidiInCallbacks();
}

static void onMIDIdisconnect(uint8_t devAddr)
{
    Serial.printf("MIDI device at address %u unplugged\r\n", devAddr);
    midiDevAddr = 0;
}


/* MAIN LOOP FUNCTIONS */

static void blinkLED(void)  
{
    const uint32_t intervalMs = 250;
    static uint32_t startMs = 0;

    static bool ledState = false;
    if ( millis() - startMs < intervalMs )
        return;
    startMs += intervalMs;

    ledState = !ledState;
    digitalWrite(25, ledState ? HIGH:LOW); 
}


static void powerUSBHost()
{
    #if ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST 
        pinMode(18, OUTPUT);  // Sets pin USB_HOST_5V_POWER to HIGH to enable USB power
        digitalWrite(18, HIGH);
    #endif
}

//######################################################## Core 1 ########################################################

void setup1() {
    powerUSBHost();
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = HOST_PIN_DP;
    USBHost.configure_pio_usb(1, &pio_cfg);
    usbhMIDI.begin(&USBHost, 0, onMIDIconnect, onMIDIdisconnect);
    core1_booting = false;
    while(core0_booting) ;
}

// core1's loop
void loop1()
{
    blinkLED();
    USBHost.task();
    usbhMIDI.readAll();
    usbhMIDI.writeFlushAll();
}

//######################################################## core 1 ########################################################


//######################################################## core 0 ########################################################1y

void setup()
{
    TinyUSBDevice.setManufacturerDescriptor("Cuiet");
    TinyUSBDevice.setProductDescriptor("MIDI Router");
    Serial.begin(115200);

    display.init();
    display.startupScreen();

    buttonA.init();
    buttonB.init();
    buttonC.init();
    buttonD.init();
    pinMode( 25, OUTPUT ); //LED
    pinMode(LED_BUILTIN, OUTPUT);
    pot1.init();
    pot2.init();
    pot3.init();
    pot4.init();
  
    MIDIuart.begin(MIDI_CHANNEL_OMNI); // don't forget OMNI

    Serial.println("USB Host to MIDI Messenger\r\n");
    core0_booting = false;
    while(core1_booting) ;
}

void loop() {    
    if( MIDIuart.read() ) {
         auto type = MIDIuart.getType();
         int data1 = MIDIuart.getData1();
         int data2 = MIDIuart.getData2();
         int channel = MIDIuart.getChannel();
         MIDIuart.send( type, data1, data2, channel );
         display.printNote( channel, data1, data2);
    }

    // Do other non-USB host processing

    if( buttonA.pressed() ){
          userChannel = (userChannel % 16) + 1 ; // increment from 1-16
          Serial.printf("Ch%u\r\n", userChannel);
          display.printUserCh( userChannel );
    }

    //if( buttonPressed( lastDebounceBTime, lastButtonBState, buttonBState, buttonBPin )){
    if( buttonB.pressed() )    {
          userProgOffset = (userProgOffset + 8) % 128 ; 
          Serial.printf("Prog Progs %u   through %u\r\n", userProgOffset, (userProgOffset + 7));
          display.printProgramShift( userProgOffset );
    }

    if( buttonC.pressed() ){
          midiPanic();
          display.printPanic();
    }

    if( buttonD.pressed() ){
          midiPanic();
          display.printBtn(4);
    }

    if( pot1.moved() ) {  display.printPot( 1, pot1.value() ); }
    
    if( pot2.moved() ) {  display.printPot( 2, pot2.value() ); }

    if( pot3.moved() ) {  display.printPot( 3, pot3.value() ); }

    if( pot4.moved() ) {  display.printPot( 4, pot4.value() ); }

    display.print();

}
