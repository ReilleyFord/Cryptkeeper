Cryptkeeper: main.c src/uart.c
	avr-gcc -std=c99 -Wall -Os -DF_CPU=8000000 -mmcu=atmega328p -c main.c -o out/Cryptkeeper.o
	avr-gcc -std=c99 -Wall -Os -DF_CPU=8000000 -mmcu=atmega328p -o out/Cryptkeeper.elf out/Cryptkeeper.o out/uart.o
	avr-objcopy -j .text -j .data -O ihex out/Cryptkeeper.elf out/Cryptkeeper.hex
