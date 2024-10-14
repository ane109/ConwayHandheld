// Headers for Adafruit Dotstar
#include <Adafruit_DotStar.h>
//#include <SPI.h>

// Dotstar Macros
#define DATAPIN    8
#define CLOCKPIN   9

// Conway Macros
#define NUM_ROWS 8
#define NUM_COLS 8
#define ALIVE_CELL 1
#define DEAD_CELL 0

// Pin definitions
const uint8_t leftButtonPin = 3;
const uint8_t rightButtonPin = 6;
const uint8_t ledData = 8;
const uint8_t ledClock = 9;
const int potPin = A1;

// Other constants
// Pot input ranges from 0-1024. Dividing by 128 to get a 0-8 brightness scale
const uint32_t potDivider = 128;
long standardInterval = 2000;

// Button variables
// 0 means pressed, except for the pot, obviously
volatile int leftButtonState = 1;
volatile int rightButtonState = 1;
volatile int potValue = 1;
volatile int lastLeftButtonState = 1;
volatile int lastRightButtonState = 1;
volatile int lastPotValue = 1;

// Global arrays
uint32_t conwayMatrix [NUM_ROWS][NUM_COLS];
uint32_t futureConwayMatrix [NUM_ROWS][NUM_COLS];

// Global Variables
uint8_t loopCount;  // loop count tracker

// Led strip
Adafruit_DotStar strip = Adafruit_DotStar(
  (NUM_ROWS*NUM_COLS), DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// Led variables
uint32_t baseLedColor = 0x110000;
uint32_t previousLedColorAndBrightness = 0x1;
uint32_t currentLedColorAndBrightness = 0x1;
unsigned long previousMillis = 0;
unsigned long lastColorChange = 0;
unsigned long lastIntervalChange = 0;

// Delay variable to prevent too many button presses being registered at once
unsigned long updateDelay = 250;

// Debounce variables
unsigned long previousDebounceCountLeft = 0;
unsigned long previousDebounceCountRight = 0;
unsigned long previousDebounceCountPot = 0;
const unsigned long debounceDelay = 50;

// Generates random values for a matrix
void randomizeMatrix(uint32_t matrix[NUM_ROWS][NUM_COLS], uint32_t upperBound) {
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      // random() returns a value between 0 and upperBound - 1
      matrix[i][j] = random(0,upperBound);
    }
  }
}

// Checks if a matrix is empty. Returns true if empty
bool checkEmptyMatrix(const uint32_t matrix[NUM_ROWS][NUM_COLS]) {
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      if (matrix[i][j]) {
        return false;
      }
    }
  }
  return true;
}

// Compares to matrices. Returns true of they are the same
bool compareMatrix(const uint32_t matrixA[NUM_ROWS][NUM_COLS], const uint32_t matrixB[NUM_ROWS][NUM_COLS]) {
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      if (matrixA[i][j] != matrixB[i][j]) {
        return false;
      }
    }
  }
  return true;
}

// Function that searches adjacent cells to count 'alive' neighbors and return that value
uint32_t neighborCount(int cellRow, int cellCol, const uint32_t matrix[NUM_ROWS][NUM_COLS]) {
  //Initialize counter to zero
  uint32_t count = 0;
  
  for (int i = cellRow - 1; i <= cellRow + 1; i++) {
    for(int j = cellCol - 1; j <= cellCol + 1; j++) {
      // Check to see if the cell is a vaid portion of the array first
      if ( (i == cellRow && j == cellCol) || (i < 0) || (j < 0) || (i > NUM_ROWS) || (j > NUM_COLS)) {
        continue;
      }
      if (matrix[i][j] == 1) {
        count++;
      }
    }
  }
  
  return count; 
}

// Returns the value of the next cell, based on the rules from Conway's Game of Life
uint32_t applyConwayRules(uint32_t aliveNeighbors, uint32_t currentCell) {
  if ((aliveNeighbors == 2 || aliveNeighbors == 3) && currentCell == 1) {
    return ALIVE_CELL;
  }
  else if (aliveNeighbors == 3 && currentCell == 1){
    return ALIVE_CELL;
  }
  else {
    return DEAD_CELL;
  }
}

// Function to populate a matrix for the next round
void scanAndGenerate(const uint32_t currentMatrix[NUM_ROWS][NUM_COLS], uint32_t nextMatrix[NUM_ROWS][NUM_COLS]) {
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      nextMatrix[i][j] = applyConwayRules(neighborCount(i, j, currentMatrix), currentMatrix[i][j]);
    }
  }
}

void printMatrixSerial(const uint32_t matrix[NUM_ROWS][NUM_COLS]) {
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      SerialUSB.print(matrix[i][j]);
      SerialUSB.print(" ");
    }
    SerialUSB.println();
  }
  SerialUSB.println();
}

// Sends a 2d matrix to the strip struct, so it can be displayed
void CopyMatrixToLEDs(const uint32_t matrix[NUM_ROWS][NUM_COLS]) {
  for (int i = 0; i < NUM_ROWS; i++) {
    for (int j = 0; j < NUM_COLS; j++) {
      if (matrix[i][j])
      {
        strip.setPixelColor(((i*NUM_ROWS)+j), currentLedColorAndBrightness);
      } else {
        strip.setPixelColor(((i*NUM_ROWS)+j), 0);
      }


    }
  }
}


// the setup routine
void setup() {

  // Dotstar Code:
  delay(10);
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  // initialize serial communication at 9600 bits per second:
  SerialUSB.begin(9600);
  while (!Serial)
  {
    delay(10);
  }

  // Set "random" seed for troubleshooting
  //randomSeed(1);

  // Give user time to check serial port
  //delay(1000);
  
  // Generate first instance of a matrix randomly
  //randomizeMatrix(conwayMatrix, 2);
  //printMatrixSerial(conwayMatrix);
  //delay(STD_DELAY);

  // Initialize button inputs
  pinMode(leftButtonPin, INPUT);
  pinMode(rightButtonPin, INPUT);
  pinMode(potPin, INPUT);

  // Initialize LED data/clock outputs
  pinMode(ledData, OUTPUT);
  pinMode(ledClock, OUTPUT);

  // Initalize loop counter
  loopCount = 0;
  
}


// the loop routine
void loop() {

  unsigned long currentMillis = millis();

  // Read current button positions
  leftButtonState = digitalRead(leftButtonPin);
  rightButtonState = digitalRead(rightButtonPin);
  potValue = analogRead(potPin);
  
  //Pot brightness adjust
  currentLedColorAndBrightness = baseLedColor * (((uint32_t)potValue / potDivider)+1);

  // Print input state
  //SerialUSB.println(leftButtonState);
  //SerialUSB.println(rightButtonState);
  //SerialUSB.println(potValue);

  //Check to see if the set interval (2 sec) has passed
  if (currentMillis - previousMillis >= standardInterval) {
    previousMillis = currentMillis;

    // Get next game state
    scanAndGenerate(conwayMatrix, futureConwayMatrix);

    //Check to see if the new future matrix is empty, a repeat, or if the loop counter hits 10.
    // if any are true call randomizeMatrix(conwayMatrix, 2) to make a new matrix, otherwise copy the future matrix and increment the counter
    if (checkEmptyMatrix(futureConwayMatrix) || compareMatrix(conwayMatrix, futureConwayMatrix) || loopCount >= 10)
    {
      randomizeMatrix(conwayMatrix, 2);
      loopCount = 0;
    } else {
      memcpy(conwayMatrix, futureConwayMatrix, sizeof(conwayMatrix));
    }
    
    CopyMatrixToLEDs(conwayMatrix);
    strip.show();

    // Misc serial debugging
    // printMatrixSerial(conwayMatrix);
    //SerialUSB.println(potValue);
    //SerialUSB.println((potValue/potDivider));
    //SerialUSB.println(standardInterval);
  }

  // Display adjustments
  potDebounceAndAdjust();
  leftButtonCheck();
  rightButtonCheck();

}

// Real time brightness adjust and "pot" debounce
void potDebounceAndAdjust() {
  // Update debounce counter if a change is registered
  if (previousLedColorAndBrightness != currentLedColorAndBrightness)
    {
      previousDebounceCountPot = millis();
    }
    // Check to see the value is held for at least the delay value, then update brightness
    if ((millis() - previousDebounceCountPot) > debounceDelay)
    {
      currentLedColorAndBrightness = baseLedColor * (((uint32_t)potValue / potDivider)+1);
      CopyMatrixToLEDs(conwayMatrix);
      strip.show();
    }
    // Save the reading for the next loop
    previousLedColorAndBrightness = currentLedColorAndBrightness;
}

// Real time color change and left button debounce
void  leftButtonCheck() {
  // Update debounce counter if a change is registered
  if (lastLeftButtonState != leftButtonState)
  {
    previousDebounceCountLeft = millis();
  }
  // Check that the button is held for at least the debounce delay value 
  if ((millis() - previousDebounceCountLeft) > debounceDelay)
  {
    // Only change if button is pressed (low)
    if (leftButtonState == 0) {
      // Check for appropriate time lapse between changing colors, otherwise colors will cycle too quickly
      if ((millis() - lastColorChange) > updateDelay) {
        // Update last color change time
        lastColorChange = millis();
        // Cycle through RGB colors
        switch (baseLedColor)
        {
        case 0x110000:
          baseLedColor = 0x001100;
          break;
        case 0x001100:
          baseLedColor = 0x000011;
          break;
        case 0x000011:
          baseLedColor = 0x110000;
          break;

        default:
          baseLedColor = 0x110000;
          break;
        }
        CopyMatrixToLEDs(conwayMatrix);
        strip.show();
      }
    }
  }
  // Save the reading for the next loop
  lastLeftButtonState = leftButtonState;
}

// Real time delay change and left button debounce
void  rightButtonCheck() {
  // Update debounce counter if a change is registered
  if (lastRightButtonState != rightButtonState)
  {
    previousDebounceCountRight = millis();
  }
  // Check that the button is held for at least the debounce delay value
  if (millis() - previousDebounceCountRight > debounceDelay)
  {
    // Only change if button is pressed (low)
    if (rightButtonState == 0) {
      // Check for appropriate time lapse between changing colors, otherwise colors will cycle too quickly
      if  ((millis() - lastIntervalChange) > updateDelay) {
        // Update last interval change time
        lastIntervalChange = millis();
        // Cycle through update speed
        switch (standardInterval)
        {
        case 2000:
          standardInterval = 4000;
          break;
        case 4000:
          standardInterval = 500;
          break;
        case 500:
          standardInterval = 2000;
          break;

        default:
          standardInterval = 2000;
          break;
        }
        CopyMatrixToLEDs(conwayMatrix);
        strip.show();
      }
    }
  }
  // Save the reading for the next loop
  lastRightButtonState = rightButtonState;
}