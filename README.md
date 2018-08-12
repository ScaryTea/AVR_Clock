# AVR Clock
### Simple 4-mode clock on 7-segment indicator.

Components used:
* Atmega328p on Arduino Uno
* 7-segment 4-digit indicator
* 3 buttons
* 1 active buzzer

The clock has 4 modes:
* Time
* Clock
* Timer
* Stopwatch

It has 4 states to function as a FSM:
* running
* changing 1st number
* changing 2nd number
* alarm

The current state determines how the input will be processed.

[Video demo](https://youtu.be/RJHlGrIwRSM) demonstrates typical usage.

7-segment dynamic indicating library by [thodnev](https://github.com/thodnev) is used.

