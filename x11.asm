; code adapted from: https://gaultier.github.io/blog/x11_x64.html
%ifndef X11_ASM
%define X11_ASM

%include "common.asm"

; Create a UNIX domain socket and connect to the X11 server.
; @returns The socket file descriptor.
x11_connect_to_server:
static x11_connect_to_server:function
    push rbp
    mov rbp, rsp

    ; open a UNIX socket: socket(2)
    mov rdi, AF_UNIX
    mov rsi, SOCK_STREAM
    mov rdx, 0 ; automatic protocol
    mov rax, SYS_SOCKET
    syscall

    cmp rax, 0
    jle die

    mov rdi, rax ; store socket fd in rdi

    sub rsp, 112 ; reserve stack space for struct sockaddr_un

    ; Set sockaddr_un.sun_family to AF_UNIX
    ; Fill sockaddr_un.sun_path with: "/tmp/.X11-unix/X0"
    mov word [rsp], AF_UNIX
    lea rsi, sun_path
    mov r12, rdi
    lea rdi, [rsp + 2]
    cld ; ensure the copy is done forwards
    mov ecx, 19 ; length is 19 with the null terminator
    rep movsb ; move RCX bytes from [RSI] to [RDI]

    ; Connect to the server: connect(2)
    mov rax, SYS_CONNECT
    mov rdi, r12
    lea rsi, [rsp]
    %define SIZEOF_SOCKADDR_UN 2+108
    mov rdx, SIZEOF_SOCKADDR_UN
    syscall

    cmp rax, 0
    jne die

    mov rax, rdi ; returns socket fd

    add rsp, 112 ; restore stack space
    pop rbp
    ret

die:
die_poll:
die_pollerr:
die_pollhup:
die_reply:
    mov rax, SYS_EXIT
    mov rdi, 1
    syscall

; Send the handshake to the X11 server and read the returned system information.
; @param rdi The socket file descriptor
; @returns The window root id (uint32_t) in rax.
x11_send_handshake:
static x11_send_handshake:function
    push rbp
    mov rbp, rsp

    ; prepare handshake
    sub rsp, 1 << 15 ; reserve 2^(15) bytes on the stack
    mov byte [rsp + 0], 'l' ; set order to little endian
    ; [rsp + 1] is pad, which is ignored
    mov word [rsp + 2], 11 ; set major version to 11

    ; send handshake to the server
    mov rax, SYS_WRITE
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, 12 ; struct is 12 bytes long
    syscall

    cmp rax, 12 ; check that all bytes were written
    jnz die

    ; read server response: read(2)
    ; Use the stack for the read buffer.
    ; The X11 server first replies with 8 bytes. Once these are read, it replies with a much bigger message.
    mov rax, SYS_READ
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, 8
    syscall

    cmp rax, 8 ; check that server replied with 8 bytes
    jnz die

    cmp byte [rsp], 1 ; check that server sent 'success' (first byte is 1)
    jnz die

    ; read rest of server response
    ; like before, use stack as read buffer
    mov rax, SYS_READ
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, 1 << 15
    syscall

    cmp rax, 0 ; check that server replied with something
    jle die

    ; set id_base globally.
    mov edx, dword [rsp + 4]
    mov dword [id_base], edx

    ; set id_mask globally.
    mov edx, dword [rsp + 8]
    mov dword [id_mask], edx

    ; read the information we need, skip over the rest.
    lea rdi, [rsp] ; pointer that will skip over some data.

    mov cx, word [rsp + 16] ; vendor length (v).
    movzx rcx, cx

    mov al, byte [rsp + 21]; Number of formats (n).
    movzx rax, al ; Fill the rest of the register with zeroes to avoid garbage values.
    imul rax, 8 ; sizeof(format) == 8

    add rdi, 32 ; Skip the connection setup
    add rdi, rcx ; Skip over the vendor information (v).

    ; Skip over padding.
    add rdi, 3
    and rdi, -4

    add rdi, rax ; Skip over the format information (n*8).

    mov eax, dword [rdi] ; Store (and return) the window root id.

    ; Set the root_visual_id globally.
    mov edx, dword [rdi + 32]
    mov dword [root_visual_id], edx

    add rsp, 1 << 15
    pop rbp
    ret

; Increment the global id.
; @return The new id.
x11_next_id:
static x11_next_id:function
    push rbp
    mov rbp, rsp

    mov eax, dword [id] ; Load global id.

    mov edi, dword [id_base] ; Load global id_base.
    mov edx, dword [id_mask] ; Load global id_mask.

    ; Return: id_mask & (id) | id_base
    and eax, edx
    or eax, edi

    add dword [id], 1 ; Increment id.

    pop rbp
    ret

; Open the font on the server side.
; @param rdi The socket file descriptor.
; @param esi The font id.
x11_open_font:
static x11_open_font:function
    push rbp
    mov rbp, rsp

    ; lenght of the font name
    %define OPEN_FONT_NAME_BYTE_COUNT 5
    ; number of bytes to align the fontname to 4 byte boundary
    %define OPEN_FONT_PADDING ((4 - (OPEN_FONT_NAME_BYTE_COUNT % 4)) % 4)
    ; total number of 32-bit words (3 words are the header, the rest of the words are the font name)
    %define OPEN_FONT_PACKET_U32_COUNT (3 + (OPEN_FONT_NAME_BYTE_COUNT + OPEN_FONT_PADDING) / 4)
    ; opcode for OpenFont function
    %define X11_OP_REQ_OPEN_FONT 0x2d

    sub rsp, 6*8

    ; Request format
    ; every request contains an 8-bit major opcode and a 16-bit length field expressed in units of four bytes
    ; here using OPEN_FONT_NAME_BYTE_COUNT or OPEN_FONT_PACKET_U32_COUNT is the same since they are both 5
    mov dword [rsp + 0*4], X11_OP_REQ_OPEN_FONT | (OPEN_FONT_PACKET_U32_COUNT << 16)
    mov dword [rsp + 1*4], esi
    mov dword [rsp + 2*4], OPEN_FONT_NAME_BYTE_COUNT
    mov byte [rsp + 3*4 + 0], 'f'
    mov byte [rsp + 3*4 + 1], 'i'
    mov byte [rsp + 3*4 + 2], 'x'
    mov byte [rsp + 3*4 + 3], 'e'
    mov byte [rsp + 3*4 + 4], 'd'

    mov rax, SYS_WRITE
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, OPEN_FONT_PACKET_U32_COUNT*4
    syscall

    cmp rax, OPEN_FONT_PACKET_U32_COUNT*4
    jnz die

    add rsp, 6*8

    pop rbp
    ret

; Create a X11 graphical context.
; @param rdi The socket file descriptor.
; @param esi The graphical context id.
; @param edx The window root id.
; @param ecx The font id.
x11_create_gc:
static x11_create_gc:function
    push rbp
    mov rbp, rsp

    sub rsp, 8*8

%define X11_OP_REQ_CREATE_GC 0x37
%define X11_FLAG_GC_BG 0x00000004
%define X11_FLAG_GC_FG 0x00000008
%define X11_FLAG_GC_FONT 0x00004000
%define X11_FLAG_GC_EXPOSE 0x00010000

%define CREATE_GC_FLAGS X11_FLAG_GC_BG | X11_FLAG_GC_FG | X11_FLAG_GC_FONT
%define CREATE_GC_PACKET_FLAG_COUNT 3
%define CREATE_GC_PACKET_U32_COUNT (4 + CREATE_GC_PACKET_FLAG_COUNT)
%define MY_COLOR_RGB 0x0000ffff

    mov dword [rsp + 0*4], X11_OP_REQ_CREATE_GC | (CREATE_GC_PACKET_U32_COUNT << 16)
    mov dword [rsp + 1*4], esi
    mov dword [rsp + 2*4], edx
    mov dword [rsp + 3*4], CREATE_GC_FLAGS
    mov dword [rsp + 4*4], MY_COLOR_RGB
    mov dword [rsp + 5*4], 0
    mov dword [rsp + 6*4], ecx

    mov rax, SYS_WRITE
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, CREATE_GC_PACKET_U32_COUNT*4
    syscall

    cmp rax, CREATE_GC_PACKET_U32_COUNT*4
    jnz die
    
    add rsp, 8*8

    pop rbp
    ret

; Create the X11 window.
; @param rdi The socket file descriptor.
; @param esi The new window id.
; @param edx The window root id.
; @param ecx The root visual id.
; @param r8d Packed x and y.
; @param r9d Packed w and h.
x11_create_window:
static x11_create_window:function
    push rbp
    mov rbp, rsp

    %define X11_OP_REQ_CREATE_WINDOW 0x01
    %define X11_FLAG_WIN_BG_COLOR 0x00000002
    %define X11_EVENT_FLAG_KEY_RELEASE 0x0002
    %define X11_EVENT_FLAG_EXPOSURE 0x8000
    %define X11_FLAG_WIN_EVENT 0x00000800
    
    %define CREATE_WINDOW_FLAG_COUNT 2
    %define CREATE_WINDOW_PACKET_U32_COUNT (8 + CREATE_WINDOW_FLAG_COUNT)
    %define CREATE_WINDOW_BORDER 1
    %define CREATE_WINDOW_GROUP 1

    sub rsp, 12*8

    mov dword [rsp + 0*4], X11_OP_REQ_CREATE_WINDOW | (CREATE_WINDOW_PACKET_U32_COUNT << 16)
    mov dword [rsp + 1*4], esi
    mov dword [rsp + 2*4], edx
    mov dword [rsp + 3*4], r8d
    mov dword [rsp + 4*4], r9d
    mov dword [rsp + 5*4], CREATE_WINDOW_GROUP | (CREATE_WINDOW_BORDER << 16)
    mov dword [rsp + 6*4], ecx
    mov dword [rsp + 7*4], X11_FLAG_WIN_BG_COLOR | X11_FLAG_WIN_EVENT
    mov dword [rsp + 8*4], 0
    mov dword [rsp + 9*4], X11_EVENT_FLAG_KEY_RELEASE | X11_EVENT_FLAG_EXPOSURE


    mov rax, SYS_WRITE
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, CREATE_WINDOW_PACKET_U32_COUNT*4
    syscall

    cmp rax, CREATE_WINDOW_PACKET_U32_COUNT*4
    jnz die

    add rsp, 12*8

    pop rbp
    ret

; Map a X11 window.
; @param rdi The socket file descriptor.
; @param esi The window id.
x11_map_window:
static x11_map_window:function
    push rbp
    mov rbp, rsp

    sub rsp, 16

    %define X11_OP_REQ_MAP_WINDOW 0x08
    mov dword [rsp + 0*4], X11_OP_REQ_MAP_WINDOW | (2<<16)
    mov dword [rsp + 1*4], esi

    mov rax, SYS_WRITE
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, 2*4
    syscall

    cmp rax, 2*4
    jnz die

    add rsp, 16

    pop rbp
    ret

; Set a file descriptor in non-blocking mode.
; @param rdi The file descriptor.
set_fd_non_blocking:
static set_fd_non_blocking:function
    push rbp
    mov rbp, rsp

    %define F_GETFL 3
    %define F_SETFL 4

    %define O_NONBLOCK 2048

    mov rax, SYS_FCNTL
    mov rdi, rdi 
    mov rsi, F_GETFL
    mov rdx, 0
    syscall

    cmp rax, 0
    jl die

    ; `or` the current file status flag with O_NONBLOCK.
    mov rdx, rax
    or rdx, O_NONBLOCK

    mov rax, SYS_FCNTL
    mov rdi, rdi 
    mov rsi, F_SETFL
    mov rdx, rdx
    syscall

    cmp rax, 0
    jl die

    pop rbp
    ret

; Read the X11 server reply.
; @return The message code in al.
x11_read_reply:
static x11_read_reply:function
    push rbp
    mov rbp, rsp

    sub rsp, 32
    
    mov rax, SYS_READ
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, 32
    syscall

    cmp rax, 1
    jle die_reply

    mov al, byte [rsp]

    add rsp, 32

    pop rbp
    ret

; Poll indefinitely messages from the X11 server with poll(2).
; @param rdi The socket file descriptor.
; @param esi The window id.
; @param edx The gc id.
poll_messages:
static poll_messages:function
    push rbp
    mov rbp, rsp

    sub rsp, 32

    %define POLLIN 0x001
    %define POLLPRI 0x002
    %define POLLOUT 0x004
    %define POLLERR  0x008
    %define POLLHUP  0x010
    %define POLLNVAL 0x020

    mov dword [rsp + 0*4], edi
    mov dword [rsp + 1*4], POLLIN
    mov dword [rsp + 16], esi ; window id
    mov dword [rsp + 20], edx ; gc id

    .loop:
        mov rax, SYS_POLL
        lea rdi, [rsp]
        mov rsi, 1
        mov rdx, -1
        syscall

        cmp rax, 0
        jle die_poll

        cmp dword [rsp + 2*4], POLLERR  
        je die_pollerr

        cmp dword [rsp + 2*4], POLLHUP  
        je die_pollhup

        mov rdi, [rsp + 0*4]
        call x11_read_reply

        %define X11_EVENT_EXPOSURE 0xc
        cmp eax, X11_EVENT_EXPOSURE
        jnz .received_other_event

        .received_exposed_event:
        mov byte [rsp + 24], 1 ; Mark as exposed.

        .received_other_event:

        cmp byte [rsp + 24], 1 ; exposed?
        jnz .loop

        ;.draw_text:
        ;    mov rdi, [rsp + 0*4] ; socket fd
        ;    lea rsi, [hello_world] ; string
        ;    mov edx, 13 ; length
        ;    mov ecx, [rsp + 16] ; window id
        ;    mov r8d, [rsp + 20] ; gc id
        ;    mov r9d, 300 ; x
        ;    shl r9d, 16
        ;    or r9d, 400 ; y
        ;    call x11_draw_text

        ;.draw_image:
        ;    mov rdi, [rsp] ; socket fd
        ;    mov rsi, [rsp + 16] ; window id
        ;    mov edx, [rsp + 20] ; gc id
        ;    lea rcx, [image]
        ;    mov r8d, WINDOW_W
        ;    shl r8d, 16
        ;    or r8d, WINDOW_H
        ;    mov r9d, 0 ; x
        ;    shl r9d, 16
        ;    or r9d, 0 ; y
        ;    call x11_put_image

        .draw_image_stack:
            mov rdi, [rsp + 0*4] ; socket fd
            mov rsi, [rsp + 16] ; window id
            mov edx, [rsp + 20] ; gc id
            lea rcx, [tiny_image]
            mov r8d, 10 ; width
            shl r8d, 16
            or r8d, 10 ; height
            mov r9d, 0 ; x
            shl r9d, 16
            or r9d, 0 ; y
            call x11_put_image_stack

        jmp .loop

    add rsp, 32
    pop rbp
    ret

; calculates padding required to 4-byte align the data
; @param rdi: the data length
; @returns eax: padding
calc_padding:
static calc_padding:function
    ; Compute padding with division and modulo 4.
    mov eax, edi ; Put dividend in eax.
    mov ecx, 4 ; Put divisor in ecx.
    cdq ; Sign extend eax (produce a quadword dividend from a doubleword before doubleword division)
    idiv ecx ; Compute eax / ecx, and put the remainder (i.e. modulo) in edx.
    ; LLVM optimizer magic: `(4-x)%4 == -x & 3`, for some reason.
    neg edx
    and edx, 3
    mov eax, edx
    ret

; Draw text in a X11 window with server-side text rendering.
; @param rdi The socket file descriptor.
; @param rsi The text string.
; @param edx The text string length in bytes.
; @param ecx The window id.
; @param r8d The gc id.
; @param r9d Packed x and y.
x11_draw_text:
static x11_draw_text:function
    push rbp
    mov rbp, rsp

    sub rsp, 1024

    mov dword [rsp + 1*4], ecx ; Store the window id directly in the packet data on the stack.
    mov dword [rsp + 2*4], r8d ; Store the gc id directly in the packet data on the stack.
    mov dword [rsp + 3*4], r9d ; Store x, y directly in the packet data on the stack.

    mov qword [rsp + 1024 - 8], rdi ; Store the socket file descriptor on the stack to free the register.

    ; length is in edx
    mov r8d, edx ; Store the string length in r8 since edx will be overwritten next.
    mov edi, edx
    call calc_padding
    mov r9d, eax ; p
    mov eax, r8d ; n
    add eax, r9d ; n + p
    shr eax, 2 ; (n + p) / 4
    add eax, 4 ; 4 + (n + p) / 4

    %define X11_OP_REQ_IMAGE_TEXT8 0x4c
    mov dword [rsp + 0*4], r8d
    shl dword [rsp + 0*4], 8
    or dword [rsp + 0*4], X11_OP_REQ_IMAGE_TEXT8
    mov ecx, eax
    shl ecx, 16
    or [rsp + 0*4], ecx

    ; Copy the text string into the packet data on the stack.
    mov rsi, rsi ; Source string in rsi.
    lea rdi, [rsp + 4*4] ; Destination
    cld ; Move forward
    mov ecx, r8d ; String length.
    rep movsb ; Copy.

    mov rdx, rax ; packet u32 count
    imul rdx, 4
    mov rax, SYS_WRITE
    mov rdi, qword [rsp + 1024 - 8] ; fd
    lea rsi, [rsp]
    syscall

    cmp rax, rdx
    jnz die

    add rsp, 1024

    pop rbp
    ret

; my code below

; Put image in a X11 window
; @param rdi The socket file descriptor
; @param esi The window id
; @param edx The gc id
; @param rcx The image data
; @param r8d packed width, height
; @param r9d Packed x and y
x11_put_image:
static x11_put_image:function
    push rbp
    mov rbp, rsp

    sub rsp, 1024

    mov dword [rsp + 1*4], esi ; drawable
    mov dword [rsp + 2*4], edx ; gcontext
    mov dword [rsp + 3*4], r8d ; width, height
    mov dword [rsp + 4*4], r9d ; x, y
    mov dword [rsp + 4*5], 24 ; depth
    shl dword [rsp + 4*5], 8 ; left-pad is zero


    %define X11_OP_PUT_IMAGE 0x48
    %define FORMAT 0x2 ; ZPixmap
    mov dword [rsp + 0*4], X11_OP_PUT_IMAGE | (FORMAT << 8)

    mov qword [rsp + 1024 - 8], rdi ; Store the socket file descriptor on the stack to free the register.
    mov qword [rsp + 1024 - 16], rcx ; Store the image data on the stack

    mov eax, r8d
    and eax, 0xFFFF
    shr r8d, 16
    mul r8d
    imul rax, 3

    ; eax should have the length now
    mov edi, eax
    mov r8d, eax
    call calc_padding
    mov r9d, eax ; p
    mov eax, r8d ; n
    add eax, r9d ; n + p
    shr eax, 2 ; (n + p) / 4
    add eax, 6 ; 6 + (n + p) / 4

    mov r8d, eax ; packet count
    shl eax, 16
    or [rsp + 0*4], eax

    ; write header first
    mov rdx, 24
    mov rdi, qword [rsp + 1024 - 8] ; fd
    mov rax, SYS_WRITE
    lea rsi, [rsp]
    syscall

    cmp rax, rdx
    jnz die

    ; now write image data
    ; note: write() may transfer fewer than count bytes, so I call it in a loop
    sub r8, 6 ; subtract header since it was sent before

    mov rdx, r8
    imul rdx, 4
    ; now rdx has total number of bytes to write

    ;; TODO: double check code below
    mov rdi, qword [rsp + 1024 - 8] ; fd
    mov r10, qword [rsp + 1024 - 16] ; pointer to data
.write:
    mov rax, SYS_WRITE
    mov rsi, r10
    syscall

    cmp rax, rdx
    jz .done
    add r10, rax ; increment pointer by number of bytes written
    sub rdx, rax
    jmp .write

.done:
    add rsp, 1024
    pop rbp
    ret

; Put image in a X11 window
; @param rdi The socket file descriptor
; @param esi The window id
; @param edx The gc id
; @param rcx The image data
; @param r8d packed width, height
; @param r9d Packed x and y
x11_put_image_stack:
static x11_put_image_stack:function
    push rbp
    mov rbp, rsp

    sub rsp, 1024

    mov dword [rsp + 1*4], esi ; drawable
    mov dword [rsp + 2*4], edx ; gcontext
    mov dword [rsp + 3*4], r8d ; width, height
    mov dword [rsp + 4*4], r9d ; x, y
    mov dword [rsp + 4*5], 24 ; depth
    shl dword [rsp + 4*5], 8 ; left-pad is zero


    %define X11_OP_PUT_IMAGE 0x48
    %define FORMAT 0x2 ; ZPixmap
    mov dword [rsp + 0*4], X11_OP_PUT_IMAGE | (FORMAT << 8)

    mov qword [rsp + 1024 - 8], rdi ; store the socket file descriptor on the stack to free the register.
    mov qword [rsp + 1024 - 16], rcx ; store the image data on the stack

    mov eax, r8d
    and eax, 0xFFFF
    shr r8d, 16
    mul r8d
    imul rax, 3

    ; eax should have the length now
    mov edi, eax
    mov r8d, eax
    call calc_padding
    mov r9d, eax ; p
    mov eax, r8d ; n
    add eax, r9d ; n + p
    shr eax, 2 ; (n + p) / 4
    add eax, 6 ; 6 + (n + p) / 4

    mov r8d, eax ; packet count
    shl eax, 16
    or [rsp + 0*4], eax

    mov r10d, r8d
    sub r10d, 6 ; remove header

    ; copy the image data into the packet data on the stack
    mov rsi, qword [rsp + 1024 - 16]
    lea rdi, [rsp + 4*6] ; destination
    cld ; move forward
    mov ecx, r10d ; image length
    rep movsb ; copy

    ; write header first
    mov rdx, r8
    imul rdx, 4
    ; rdx has total number of bytes to write
    mov rdi, qword [rsp + 1024 - 8] ; fd
    mov rax, SYS_WRITE
    lea rsi, [rsp]
    syscall

    cmp rax, rdx
    jnz die

    add rsp, 1024
    pop rbp
    ret

; try to change pixmap, use memset, check that stack actually contains correct image data etc

%endif
