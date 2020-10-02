# Cryptkeeper

### Introduction ###
This program `Cryptkeeper` is the core of a suite of tools used for SD/MMC serial communication between  
SD/MMC memory and AVR Chips via the `SPI` protocol. The main focus of these tools is performing CMD  
transactions with an SD card, primarily `CMD42` interaction. This CMD42 is for acessing/reading/writing  
a hardware level passcode lock on an SD or MMC memory. This tool is used to perform a read of SD card  
registers, determine lock status, Write a CMD42 passcode and lock a card, unlock a card for the current  
power session, or clear a passcode entirely, as well as some other features. 

`Cryptkeeper` was designed specifically for the Atmel `ATMega328p` a very popular and heavily supported  
but outdated chip. However, the core CMD interaction of the program is mostly portable and can be used  
for anything that supports hardware level SPI interaction. The core of the program has already been used  
and compiled for an ATTiny804 for a related tool `SD-Stoplight` with only certain registers, pins, ports,  
etc requiring changes. The core CMD interaction remains the same. 

#### Setup ####
This program was writting in `C` and compiled via `AVR-GCC`. I have successfully built the code on a Windows 10  
Machine via `WinAVR` as well as a Ubuntu VM via native `AVR-GCC`. Once the code is compiled to a `.hex` file it  
is flashed to the chip mounted in an Arduino UNO via `avrdude`. Once the code is flashed and the chip is ready  
I have been using `PuTTY` to connect to the arduino over a serial connection. This will start up the USART terminal  
and the program will begin. There are many other ways to set this up but for now this is how i've been running it. 
The goal in the future is custom designed hardware to support this code. 

##### Credit #####
UART source code is from Mika Tuupola here:  
https://www.appelsiini.net/2011/simple-usart-with-avr-libc  
All credit for the UART code goes to Mika Tuupola  
  
The idea for this project came from the Original SDLocker 2 project here:  
http://www.seanet.com/~karllunt/sdlocker2.html  
  
Created by Reilley Ford  
  
`Lice in a fur collar`  
`Hide deep out of sight,`  
`But the devil hides deeper`  
`By far, in man's heart.`  
- Johann Wolfgang Von Goethe
