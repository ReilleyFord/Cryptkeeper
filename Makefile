Polyphemus: main.c src/uart.c
	avr-gcc -std=c99 -Wall -Os -DF_CPU=8000000 -mmcu=atmega328p -c main.c -o Cryptkeeper.o
	avr-gcc -std=c99 -Wall -Os -DF_CPU=8000000 -mmcu=atmega328p -o Cryptkeeper.elf Cryptkeeper.o uart.o
	avr-objcopy -j .text -j .data -O ihex Cryptkeeper.elf Cryptkeeper.hex
