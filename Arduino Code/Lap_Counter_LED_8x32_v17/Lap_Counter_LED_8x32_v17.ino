/********************************************************
  /****** Scalextric Lap Counter Unit ********************
  /****** by Matt Little **********************************
  /****** Date: 18/7/18 **********************************
  /****** info@re-innovation.co.uk ************************
  /****** www.re-innovation.co.uk *************************
  /********************************************************

  /*
  Code for Sure 8 x 32 LED matrix HT1632 controller board

  This reads data from two lap counter switches. This is for player 1 and 2 timers.
  These work with interrupts 1 and 2 which control the timers.

  Two additional buttons are added:
  Green is Start
  Red is edit: change the number of laps (puts into edit mode)

  Usual operation:
  When powered on it will show two numbers - this is how many laps to count down from.
  Start button will reset these.
  When Start button is pressed:
  Show "Ready   Set   GO!"
  Then display number of laps and max lap count (adjustable - stored in EEPROM) for each track
  Wach time a track interrupt happens then the display will show the lap time for 1 second (adjustable).

  At end (laps = 0) then the display shows the winner and their fastest time on the display.

  Press edit button:
  Then set number of laps (0-50 available) Use green button to scroll through.
  This needs to be stored in EEPROM for next time.

  The display is updated often, but only out of the interrupt loop.

  Interrupts:
  They will go low when triggered.
  The cause the timer to stop and reset. It will also decrement the lap counter
  The lap is then reduced and the previous time displayed

  Pin allocations:
  D2: Track 1 interrupt
  D3: Track 2 interrupt

  LED matrix connections:
  D4: CS1
  D5: CS2
  D6: CS3
  D7: CS4
  B8: WR
  B9: DATA
  D10: RD

  D11:
  D12: Button 1: Start
  D13: Button 2: Edit / Status LED

  This is code for a plug-in board for these Sure Electronics LED displays.
  http://www.sureelectronics.net/goods.php?id=1119

  The PCB has been developed by Renewable Energy Innovation.
  http://www.re-innovation.co.uk/

  The HT1632 code was obtained from here:
  https://github.com/gauravmm/HT1632-for-Arduino
  The HT1632.h file must ensure that the USE_NMOS was set to 1.
  This was for my Sure displays.
  // Note: I've had to updated HT1632.cpp to change the line:
  #if defined TYPE_3208_MONO
  //writeCommand(HT1632_CMD_COMS00);  // For HT1632c
  writeCommand(HT1632_CMD_COMS10);     // Changed for HT1632

  Code started:  18/7/18
  By:  Matt Little (matt@re-innovation.co.uk)

  TO DO:
*/
#include <HT1632.h>
#include <font_5x4.h>
#include <images.h>
#include <stdlib.h>
#include <EEPROM.h>        // For writing values to the EEPROM
#include <avr/eeprom.h>    // For writing values to EEPROM

#include <avr/interrupt.h>
#include <avr/io.h>

// constants won't change. They're used here to set pin numbers:
const int buttonStart = 12;     // Start button (will be D11 - D3 for testing)
const int buttonEdit = 13;     // Edit Button
const int debounceDelay = 5; //

// This is for the HT1632 LED display
//Initialise with
//HT1632.begin(pinCS1 [, pinCS2 [, pinCS3 [, pinCS4]]], pinWR, pinDATA);
//  HT1632.begin(4, 8, 9);


// ******* Variables *************

int lapMax = 5;     // Holds the number of laps to count down (stored in EEPROM when adjusted)
int lapCount1;   // Holds the actual number of laps so far
int lapCount2;   // Holds the actual number of laps so far

volatile float lap1time;
float lap1timeAct;  // This is the actual lap time in mS
float lap1timeMin = 999999;  // Stores minimum time for laps
float lap1timeOld; // for comparison
volatile bool track1flag = LOW; // Set on track 1 interrupt

volatile float lap2time;
float lap2timeAct;  // This is the actual lap time in mS
float lap2timeMin = 999999;  // stores minimum time for laps
float lap2timeOld; // for comparison
volatile bool track2flag = LOW;  // Set on track 2 interrupt

bool endRaceFlag = LOW; // Set to show the winner

bool raceRunningFlag = LOW;

bool buttonValue;
bool lastButtonAState = LOW;
int debounceACounter = 0;

bool buttonValueB;
bool lastButtonBState = LOW;
int debounceBCounter = 0;

//int wd;  // for display?

int displayCounterMax = 50;  // This is how often (in multiples of 5mS) the display will update
int displayCounter = 0;

int displayMode = 0;  // Initial display mode

bool editFlag = LOW;  // For the edit button
bool startFlag = LOW; // For the start flag

unsigned long enterMillis = 0;

int winner = 0; // Stores the winner number
char result[8] = ""; // Buffer big enough for 7-character float

bool setInt1flag = LOW;

//static unsigned long last_interrupt_time1 = 0;
//unsigned long interrupt_time1;
//static unsigned long last_interrupt_time2 = 0;
//unsigned long interrupt_time2;

// ******* END Variables *************

// Need to put ISR here x 2
void track1Interupt(void)
{
  static unsigned long last_interrupt_time1 = 0;
  unsigned long interrupt_time1 = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time1 - last_interrupt_time1 > 2)
  {
    //detachInterrupt(0);
    track1flag = HIGH;
    lap1time = millis();
    Serial.println("int1!");
  }
  last_interrupt_time1 = interrupt_time1;
  // Does this on interrupt so code can continue
}

void track2Interupt(void)
{
  static unsigned long last_interrupt_time2 = 0;
  unsigned long interrupt_time2 = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time2 - last_interrupt_time2 > 50)
  {
    //detachInterrupt(1);
    track2flag = HIGH;
    lap2time = millis();
    Serial.println("int2!");
  }
  last_interrupt_time2 = interrupt_time2;
  // Does this on interrupt so code can continue
}

void setup() {

  // initialize the pushbutton pins as an input:
  // DEBUG!! pinMode(buttonStart, INPUT_PULLUP);
  pinMode(buttonStart, INPUT_PULLUP);
  pinMode(buttonEdit, INPUT_PULLUP);

  pinMode(2, INPUT);
  pinMode(3, INPUT);

  // // For battery voltage measurement
  // analogReference(INTERNAL);  // Sets the voltage ref to the internal 1.1V

  Serial.begin(115200);    // Set up a serial output for data display and changing parameters
  Serial.flush();

  // Want to read in any saved value of Max Laps from EEPROM
  // This is saved into space 0
  // EEPROM.write(0, lapMax); // Example of writing to EEPROM
  lapMax = EEPROM.read(0);

  Serial.print("Max Laps is: ");
  Serial.println(lapMax);

  //Initialise with
  //HT1632.begin(pinCS1 [, pinCS2 [, pinCS3 [, pinCS4]]], pinWR, pinDATA);
  // HT1632 pin connections:
  // CS1: D4
  // CS2: D5
  // CS3: D6
  // CS4: D7
  // WR:  D8
  // RD:  D10
  // DATA:D9
  // OSC: NC
  // SYNC:NC

  HT1632.begin(4, 5, 8, 9);
  // Set pin 2 & 3 as interrupt and attach handler:
  //attachInterrupt(0, track1Interupt, FALLING);
  //attachInterrupt(1, track2Interupt, FALLING);
}

void loop ()
{
  // Start of main loop
  // Check track flags and deal with them
  if (raceRunningFlag == HIGH)
  {
    checkFlags();
  }

  // Check switches
  checkButton();

  // Update display, if required (every 0.25 seconds in this example)
  if (displayCounter >= displayCounterMax)
  {
    updateDisplay();
    //Reset the display counter
    displayCounter = 0;
  }
  displayCounter++; // increment the counter

  // Delay and slow things down
  delay(5);  // Slow things down
}

void checkButton()
{
  //******** SWITCH CHECK *******************************************
  // Check the button A and debounce it and latch it
  buttonValue = digitalRead(buttonStart);
  if (buttonValue != lastButtonAState && debounceACounter >= debounceDelay)
  {
    //buttonPressed=HIGH;  // set the button pressed flag.

    // Button Start has been pressed
    startFlag = HIGH;
    raceRunningFlag = LOW;  // Just in case we wer halfway through before reset
    Serial.print("START:");
    Serial.println(startFlag);

  }
  else if (buttonValue == LOW)
  {
    debounceACounter = 0;
  }
  lastButtonAState = buttonValue;
  debounceACounter++;

  // Check the button B and debounce it and latch it
  buttonValueB = digitalRead(buttonEdit);
  if (buttonValueB != lastButtonBState && debounceBCounter >= debounceDelay)
  {
    //buttonPressed=HIGH;  // set the button pressed flag.
    // Button Start has been pressed
    Serial.print("EDIT:");
    if (editFlag == LOW)
    {
      editFlag = HIGH;
      displayMode = 6;  // Go into Edit Mode
    }
    else
    {
      editFlag = LOW;
      enterMillis = millis(); // Display for one second that it has saved
      displayMode = 7;  // write data to EEPROM
    }
    Serial.println(editFlag);
  }
  else if (buttonValueB == LOW)
  {
    debounceBCounter = 0;
  }
  lastButtonBState = buttonValueB;
  debounceBCounter++;

  // Sort out the start of the race:
  if (startFlag == HIGH && editFlag == LOW)
  {
    displayMode = 1;
    enterMillis = millis();
    startFlag = LOW;
  }

}

void checkFlags()
{
  if (endRaceFlag == HIGH)
  {
    track1flag = LOW;
    track2flag = LOW;
  }
  // Check track flag 1
  if (track1flag == HIGH  && endRaceFlag == LOW)
  {
    // Lap one has been completed
    lap1timeAct = lap1time - lap1timeOld;
    lapCount1++;  // Increment the lap counter

    Serial.print("Time1:");
    Serial.println(lap1timeAct);
    Serial.print("Lap: ");
    Serial.print(lapCount1);
    Serial.print(" of ");
    Serial.println(lapMax);

    if ( lap1timeAct < lap1timeMin)
    {
      // This is a new miniumum time
      lap1timeMin = lap1timeAct;
      Serial.print("Min:");
      Serial.println(lap1timeMin);
    }

    if (lapCount1 >= lapMax)
    {
      // Here track 1 is the winner
      Serial.println("Track 1 wins!");
      Serial.print("Min:");
      Serial.println(lap1timeMin);
      endRaceFlag = HIGH;
      winner = 1;
      startFlag = LOW;   // Reset the start flag
      Serial.print("START:");
      Serial.println(startFlag);
    }
    lap1timeOld = lap1time;
    track1flag = LOW;
  }
  
  // Check track flag 2
  if (track2flag == HIGH  && endRaceFlag == LOW)
  {
    // Lap one has been completed
    lap2timeAct = lap2time - lap2timeOld;
    lapCount2++;  // Increment the lap counter

    Serial.print("Time2:");
    Serial.println(lap2timeAct);
    Serial.print("Lap: ");
    Serial.print(lapCount2);
    Serial.print(" of ");
    Serial.println(lapMax);

    if (lap2timeAct < lap2timeMin)
    {
      // This is a new miniumum time
      lap2timeMin = lap2timeAct;
      Serial.print("Min:");
      Serial.println(lap2timeMin);
    }
    if (lapCount2 >= lapMax)
    {
      // Here track 2 is the winner
      Serial.println("Track 2 wins!");
      Serial.print("Min:");
      Serial.println(lap2timeMin);
      endRaceFlag = HIGH;
      winner = 2;
      startFlag = LOW;   // Reset the start flag
      Serial.print("START:");
      Serial.println(startFlag);
    }
    lap2timeOld = lap2time;
    track2flag = LOW;
  }
}


void updateDisplay()
{
  switch (displayMode)
  {
    case 0:
      // Initial display mode
      // Display "ready" and blank rest of screen
      HT1632.renderTarget(0);
      HT1632.clear();
      HT1632.drawText("READY" , 6, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();
      HT1632.renderTarget(1);
      HT1632.clear();
      HT1632.render();
      break;

    case 1:
      if (millis() >= (enterMillis + 1000))
      {
        displayMode = 2;
        enterMillis = millis();
      }
      HT1632.renderTarget(0);
      HT1632.clear();
      HT1632.drawText("READY" , 6, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();
      HT1632.renderTarget(1);
      HT1632.clear();
      HT1632.render();

      break;

    case 2:
      if (millis() >= (enterMillis + 1000))
      {
        displayMode = 3;
        enterMillis = millis();
      }
      HT1632.renderTarget(0);
      HT1632.clear();
      HT1632.drawText("SET" , 9, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();
      HT1632.renderTarget(1);
      HT1632.clear();
      HT1632.render();

      break;

    case 3:
      if (millis() >= (enterMillis + 1000))
      {
        displayMode = 4;
      }
      HT1632.renderTarget(0);
      HT1632.clear();
      HT1632.drawText("GO!!!" , 9, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();
      HT1632.renderTarget(1);
      HT1632.clear();
      HT1632.render();

      // Here we start the race and set all the starting variables
      winner = 0;
      lap1timeAct = 0;
      lap2timeAct = 0;
      // Reset all the values
      lapCount1 = 0;
      lapCount2 = 0;
      endRaceFlag = LOW;
      lap1timeMin = 999999;
      lap1timeOld = millis();  // Set the start time
      lap2timeMin = 999999;
      lap2timeOld = millis();  // Set the start time
      raceRunningFlag = HIGH; // Start the actual timers
      
      attachInterrupt(0, track1Interupt, FALLING);
      attachInterrupt(1, track2Interupt, FALLING);
      break;

    case 4:
      // In this section the displayMode number is used to show what is to be displayed
      // The race is running here
      // Display number of laps for each track

      if (lap1timeAct > 0)
      {

        HT1632.renderTarget(0);
        HT1632.clear();
        dtostrf(lapCount1, 2, 0, result);
        HT1632.drawText(result , 1, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
        HT1632.drawText("of" , 11, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
        dtostrf(lapMax, 2, 0, result);
        HT1632.drawText(result , 21, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
        HT1632.render();
      }
      if (lap2timeAct > 0)
      {
        HT1632.renderTarget(1);
        HT1632.clear();
        dtostrf(lapCount2, 2, 0, result);
        HT1632.drawText(result , 1, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
        HT1632.drawText("of" , 11, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
        dtostrf(lapMax, 2, 0, result);
        HT1632.drawText(result , 21, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
        HT1632.render();
      }

      if (endRaceFlag == HIGH)
      {
        displayMode = 5;
        raceRunningFlag = LOW;  // Stop the timers
      }
      break;

    case 5:
      // We have a winner!
      detachInterrupt(0);
      detachInterrupt(1);
      // Display "WINNER:?"
      // Then min lap time for winner
      dtostrf(winner, 1, 0, result);
      HT1632.renderTarget(0);
      HT1632.clear();
      HT1632.drawText(result, 1, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.drawText("WINS!" , 7, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();

      if (winner == 1)
      {
        dtostrf(lap1timeMin / 1000.0, 6, 2, result); // Leave room for too large numbers!
      }
      else if (winner == 2)
      {
        dtostrf(lap2timeMin / 1000.0, 6, 2, result); // Leave room for too large numbers!
      }
      HT1632.renderTarget(1);
      HT1632.clear();
      HT1632.drawText(result , 1, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();
      break;

    case 6:
      // Edit button has been pressed
      // Check Start button to increment max laps
      HT1632.renderTarget(0);
      HT1632.clear();
      HT1632.drawText("MAX LAPS" , 1, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();
      HT1632.renderTarget(1);
      HT1632.clear();
      dtostrf(lapMax, 2, 0, result);
      HT1632.drawText(result , 1, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();

      if (startFlag == HIGH)
      {
        lapMax++;
        if (lapMax > 50)
        {
          lapMax = 1;  // Roll over
        }
        startFlag = LOW;
      }
      break;

    case 7:
      if (millis() >= (enterMillis + 1000))
      {
        // Here we write to EEPROM on leaving, so just done once
        EEPROM.write(0, lapMax); // Example of writing to EEPROM
        displayMode = 0;
      }
      HT1632.renderTarget(0);
      HT1632.clear();
      HT1632.drawText("SAVED" , 1, 1, FONT_5X4, FONT_5X4_END, FONT_5X4_HEIGHT);
      HT1632.render();

      break;

    default:
      // In case things have gone wrong
      break;
  }

}

