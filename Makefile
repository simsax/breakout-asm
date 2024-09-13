breakout: breakout.o
	ld -s -o breakout breakout.o

breakout.o: breakout.asm
	nasm -felf64 breakout.asm
