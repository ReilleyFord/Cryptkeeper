Polyphemus: main.c src/uart.c
	avr-gcc -Wall -Os -DF_CPU=8000000 -mmcu=atmega328p -c main.c -o main.o
	avr-gcc -Wall -Os -DF_CPU=8000000 -mmcu=atmega328p -o main.elf main.o uart.o
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
