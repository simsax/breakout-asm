%define WINDOW_W 800
%define WINDOW_H 600
%define IMAGE_SIZE (WINDOW_W * WINDOW_H * 3) ; 24-bit pixels

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

; @param rdi: pointer to data
; @param si: filler byte
; @param rdx: length of data
memset:
    mov al, sil
    mov rcx, rdx
    rep stosb
    ret

; @param rdi: pointer to image data
; @param esi: rgb color, encoded as 0x00RRGGBB
; @param rdx: length of data
color_image:
    ; blue
    mov r8b, sil
    ; green
    shr rsi, 8
    mov r9b, sil
    ; red
    shr rsi, 8
    mov r10b, sil

    xor rcx, rcx ; counter
.loop:
    mov byte [rdi + 0 + rcx], r8b
    mov byte [rdi + 1 + rcx], r9b
    mov byte [rdi + 2 + rcx], r10b

    cmp rcx, rdx
    jz .done
    add rcx, 3
    jmp .loop

.done:
    ret

_start:
    ; initialize image data
    lea rdi, [image]
    mov esi, 0x0000FF00 ; green
    mov rdx, IMAGE_SIZE
    call color_image

    call x11_connect_to_server
    mov r15, rax ; Store the socket file descriptor in r15.

    mov rdi, rax
    call x11_send_handshake

    mov r12d, eax ; Store the window root id in r12.

    call x11_next_id
    mov r14d, eax ; Store the font_id in r14.

    mov rdi, r15
    mov esi, r14d
    call x11_open_font

    call x11_next_id
    mov r13d, eax ; Store the gc_id in r13.

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

image:
    resb IMAGE_SIZE
image_end:


