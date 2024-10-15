# ConwayHandheld
Arduino code for a conway game displayed on a DotStar LED matrix

## Description
This is code for a zero-player handheld that runs Conway's Game of Life. The software provisions three pins for user input, two digital and one analog.
The inputs control the following:
- "Left" button digital input: Cycles the DotStar display between green, red, and blue pixels
- "Right" button digital input: Changes the time each generation is displayed, between 2, 4, and 0.5 seconds
- Analog input: Sets the brightness of the DotStar display

## Hardware Used
- SparkFun SAMD21 Dev Breakout
- Adafruit DotStar High Density 8x8 Grid
- Level shifter and voltage regulator to interface between the DotStar matrix and SAMD21 board

![Gif of a handheld device displaying Conway's Game of Life on an 8x8 LED matrix](https://github.com/ane109/ConwayHandheld/blob/main/Media/ConwayDemo.gif)
