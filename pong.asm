%include "common.asm"
%include "x11.asm"

global _start
section .text

strlen:
    ; size_t strlen(const char* s);
    xor rax, rax
.loop:
    mov dl, [rdi]
    test dl, dl
    jz .done
    inc rax
    inc rdi
    jmp .loop
.done:
    ret

print:
    ; void print(char* msg);
    push rdi ; message
    call strlen
    mov rdx, rax ; message_len
    mov rax, SYS_WRITE
    mov rdi, STDOUT
    pop rsi ; message
    syscall
    ret

println:
    ; void println(char* msg);
    call print
    mov rdi, newline
    call print
    ret

_start:
    call x11_connect_to_server
    mov r15, rax ; Store the socket file descriptor in r15.

    mov rdi, rax
    call x11_send_handshake

    mov r12d, eax ; Store the window root id in r12.

    call x11_next_id
    mov r13d, eax ; Store the gc_id in r13.

    call x11_next_id
    mov r14d, eax ; Store the font_id in r14.

    mov rdi, r15
    mov esi, r14d
    call x11_open_font


    mov rdi, r15
    mov esi, r13d
    mov edx, r12d
    mov ecx, r14d
    call x11_create_gc

    call x11_next_id
    
    mov ebx, eax ; Store the window id in ebx.

    mov rdi, r15 ; socket fd
    mov esi, eax
    mov edx, r12d
    mov ecx, [root_visual_id]
    mov r8d, 200 | (200 << 16) ; x and y are 200
    %define WINDOW_W 800
    %define WINDOW_H 600
    mov r9d, WINDOW_W | (WINDOW_H << 16)
    call x11_create_window

    mov rdi, r15 ; socket fd
    mov esi, ebx
    call x11_map_window

    mov rdi, r15 ; socket fd
    call set_fd_non_blocking

    mov rdi, r15 ; socket fd
    mov esi, ebx ; window id
    mov edx, r13d ; gc id
    call poll_messages

    mov rax, SYS_EXIT
    xor rdi, rdi ; exit 0
    syscall

section .rodata

newline: db 10
static newline:data

sun_path: db "/tmp/.X11-unix/X0", 0
static sun_path:data

hello_world: db "Hello, world!"
static hello_world:data

section .data

id: dd 0
static id:data

id_base: dd 0
static id_base:data

id_mask: dd 0
static id_mask:data

root_visual_id: dd 0
static root_visual_id:data

section .bss

; general purpose buffer
buffer:
    resb 256
buffer_end:


