# breakout-asm

A Breakout game implemented in x64 assembly using the nasm assembler.
Doesn't rely on libc, Xlib, or any other libraries. The game communicates directly with the X server via sockets.
It *should* work on any Linux system running X11, but I have tested it only on my machine.

![](https://imgur.com/5kL4vDO.gif)

## References

* https://gaultier.github.io/blog/x11_x64.html for setting up the communication with X11
