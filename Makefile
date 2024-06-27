pong: pong.o
	ld -o pong pong.o

pong.o: pong.asm common.asm x11.asm
	nasm -felf64 -g pong.asm
