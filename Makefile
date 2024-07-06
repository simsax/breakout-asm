pong: pong.o
	ld -o pong pong.o

pong.o: pong.asm
	nasm -felf64 -g -F dwarf pong.asm
