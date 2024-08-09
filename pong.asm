%define STDIN 0
%define STDOUT 1
%define STDERR 2
%define AF_UNIX 1
%define SOCK_STREAM 1
%define KEYCODE_LEFT 113
%define KEYCODE_RIGHT 114
%define SYS_READ 0
%define SYS_WRITE 1
%define SYS_POLL 7
%define SYS_SOCKET 41
%define SYS_CONNECT 42
%define SYS_EXIT 60
%define SYS_FCNTL 72
%define SYS_GETTIME 228
%define EAGAIN -11
%define POLL_TIMEOUT 0

%define WINDOW_W 800
%define WINDOW_H 800
%define WINDOW_SIZE (WINDOW_W * WINDOW_H * 4) ; BGRX format
%define BALL_COLOR 0x00666666 
%define PAD_COLOR 0x000000FF ; blue
%define BLUE_BRICKS_COLOR   0x000000FF
%define GREEN_BRICKS_COLOR  0x0000FF00
%define YELLOW_BRICKS_COLOR 0x00FFFF00
%define ORANGE_BRICKS_COLOR 0x00FF8800
%define RED_BRICKS_COLOR    0x00FF0000
%define BORDER_BRICKS_COLOR 0
%define NUM_BRICKS 16

global _start
section .text

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
    lea rsi, [sun_path]
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

    ;mov dl, byte [rsp + 26]
    ;mov byte [min_keycode], dl
    ;mov dl, byte [rsp + 27]
    ;mov byte [max_keycode], dl

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

    %define X11_OP_REQ_CREATE_WINDOW   0x01
    %define X11_FLAG_WIN_BG_COLOR      0x00000002
    %define X11_EVENT_FLAG_KEY_PRESS   0x00000001
    %define X11_EVENT_FLAG_KEY_RELEASE 0x00000002
    %define X11_EVENT_FLAG_EXPOSURE    0x00008000
    %define X11_FLAG_WIN_EVENT         0x00000800
    
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
    mov dword [rsp + 9*4], X11_EVENT_FLAG_EXPOSURE | X11_EVENT_FLAG_KEY_RELEASE | X11_EVENT_FLAG_KEY_PRESS

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
; @param rdi The socket file descriptor.
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
    jle die

    mov al, byte [rsp]

    add rsp, 32
    pop rbp
    ret

; Read the X11 server reply for query extension
; @param rdi The socket file descriptor.
; @return The extension opcode in eax
x11_read_extension_reply:
static x11_read_extension_reply:function
    push rbp
    mov rbp, rsp

    sub rsp, 32
    
    mov rax, SYS_READ
    mov rdi, rdi
    lea rsi, [rsp]
    mov rdx, 32
    syscall

    cmp rax, 1
    jle die

    ; extract opcode
    mov eax, [rsp + 2*4]
    shr eax, 8
    and eax, 0xFF

    add rsp, 32

    pop rbp
    ret


; Poll indefinitely messages from the X11 server with poll(2)
; @param rdi The socket file descriptor.
; @param esi The window id.
; @param edx The gc id.
poll_messages:
static poll_messages:function
    push rbp
    mov rbp, rsp

    sub rsp, 64

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

    mov byte [rsp + 24], 0 ; moving left
    mov byte [rsp + 25], 0 ; moving right

    .loop:
        mov rax, SYS_POLL
        lea rdi, [rsp]
        mov rsi, 1
        mov rdx, POLL_TIMEOUT
        syscall

        cmp rax, 0
        je .update ; timeout
        jl die

        cmp dword [rsp + 2*4], POLLERR  
        je die

        cmp dword [rsp + 2*4], POLLHUP  
        je die

        mov rdi, [rsp + 0*4]
        call x11_read_reply

        %define X11_EVENT_KEY_PRESS 0x2
        %define X11_EVENT_KEY_RELEASE 0x3
        cmp eax, X11_EVENT_KEY_PRESS
        jnz .key_release

        ; KEYPRESS EVENT
        mov sil, byte [rsp - 16 - 31] ; directly use the keycode instead of keysym for simplicity
        cmp sil, KEYCODE_LEFT
        jnz .keypress_right
        ; left press
        mov byte [rsp + 24], 1
        ;mov byte [rsp + 12], KEYCODE_LEFT
        jmp .update

        .keypress_right:
        cmp sil, KEYCODE_RIGHT
        jnz .update
        ; right press
        mov byte [rsp + 25], 1
        ;mov byte [rsp + 12], KEYCODE_RIGHT
        jmp .update

        .key_release:
        cmp eax, X11_EVENT_KEY_RELEASE
        jnz .update

        ; KEYRELEASE EVENT
        mov r15b, byte [rsp - 16 - 31] ; keycode
        mov r10w, word [rsp - 16 - 30] ; sequence number

        ; handle x11 auto-repeat issue (https://wiki.tcl-lang.org/page/Disable+autorepeat+under+X11)
        ; check if there is a keypress event right after release event that has the same sequence id
        mov rax, SYS_POLL
        lea rdi, [rsp]
        mov rsi, 1
        mov rdx, POLL_TIMEOUT
        syscall

        cmp rax, 0
        je .keyrelease_left
        jl die

        mov rdi, [rsp]
        call x11_read_reply
        cmp eax, X11_EVENT_KEY_PRESS
        jnz .keyrelease_left
        mov r11w, word [rsp - 16 - 30]
        cmp r10w, r11w ; check if this key-press has same sequence number as previous release
        jz .update ; fake release, ignore

        .keyrelease_left:
        cmp r15b, KEYCODE_LEFT
        jnz .keyrelease_right
        ; left release
        mov byte [rsp + 24], 0
        jmp .update

        .keyrelease_right:
        cmp r15b, KEYCODE_RIGHT
        jnz .update
        ; right release
        mov byte [rsp + 25], 0
        jmp .update

        .update:
        ; calculate delta time for current frame
        call update_current_time
        movsd xmm14, [cur_time]
        movsd xmm15, [prev_time]
        subsd xmm14, xmm15 ; xmm14 has now the delta time
        cvtsd2ss xmm14, xmm14
        ; prev_time = cur_time
        movsd xmm15, [cur_time]
        movsd [prev_time], xmm15

        movd [rsp + 60], xmm14

        ; update pad
        movss xmm0, [pad_x]
        movss xmm1, [pad_dx]
        movss xmm2, [pad_min]
        movss xmm3, [pad_max]

        mov al, byte [rsp + 24]
        mov cl, byte [rsp + 25]
        cmp al, cl
        ; when both are pressed you can stay still
        jz .update_ball 

        test al, al
        jz .pad_move_right
        ; move left
        ucomiss xmm0, xmm2
        jb .update_ball
        mulss xmm1, xmm14
        subss xmm0, xmm1
        movss [pad_x], xmm0
        jmp .update_ball

        .pad_move_right:
        test cl, cl
        jz .update_ball
        ; move right
        ucomiss xmm0, xmm3
        jae .update_ball
        mulss xmm1, xmm14
        addss xmm0, xmm1
        movss [pad_x], xmm0

        .update_ball:
        movss xmm0, [ball_x]
        movss xmm10, [ball_y]
        movss xmm1, [ball_max]
        movss xmm4, [ball_min] ; == ball ray
        movss xmm12, [ball_dy]
        movss xmm5, [pad_x]
        movss xmm6, [pad_y]
        movss xmm8, [pad_height]
        movss xmm9, [pad_width]
        addss xmm6, xmm8 ; top of pad
        movss xmm13, [pad_y]
        movss xmm7, xmm10
        subss xmm7, xmm4 ; bottom of ball
        addss xmm9, xmm5 ; pad right

        ; pad (TODO: fix and treat it like a brick)
        ucomiss xmm12, [zero]
        ja .bricks_collision ; skip pad collision if going up
        ucomiss xmm7, xmm6 ; ball_y_bottom <= pad_y_top
        ja .bricks_collision
        ucomiss xmm7, xmm13 ; ball_y_bottom <= pad_y_center (to fix)
        jb .bricks_collision
        ucomiss xmm0, xmm5 ; 
        jb .bricks_collision
        ucomiss xmm0, xmm9
        jb .inverty

        .bricks_collision:
        ; find out if colliding with any row
        movss xmm1, [ball_y]
        movss xmm3, [ball_ray]
        addss xmm1, xmm3 ; xmm1 = ball_top
        movss xmm2, [ball_y]
        subss xmm2, xmm3 ; xmm2 = ball_bottom
        
        movss xmm11, [bricks_height]
        .blue:
        movss xmm15, [blue_bricks_y]
        ucomiss xmm1, xmm15
        jb .walls_collision
        addss xmm15, xmm11
        ucomiss xmm2, xmm15
        jae .green
        ; collision on blue row
        lea rdi, [blue_bricks]
        call collide_row
        
        .green:
        movss xmm15, [green_bricks_y]
        ucomiss xmm1, xmm15
        jb .walls_collision
        addss xmm15, xmm11
        ucomiss xmm2, xmm15
        jae .yellow
        ; collision on green row
        lea rdi, [green_bricks]
        call collide_row

        .yellow:
        movss xmm15, [yellow_bricks_y]
        ucomiss xmm1, xmm15
        jb .walls_collision
        addss xmm15, xmm11
        ucomiss xmm2, xmm15
        jae .orange
        ; collision on yellow row
        lea rdi, [yellow_bricks]
        call collide_row
    
        .orange:
        movss xmm15, [orange_bricks_y]
        ucomiss xmm1, xmm15
        jb .walls_collision
        addss xmm15, xmm11
        ucomiss xmm2, xmm15
        jae .red
        ; collision on orange row
        lea rdi, [orange_bricks]
        call collide_row

        .red:
        movss xmm15, [red_bricks_y]
        ucomiss xmm1, xmm15
        jb .walls_collision
        addss xmm15, xmm11
        ucomiss xmm2, xmm15
        jae .walls_collision
        ; collision on red row
        lea rdi, [red_bricks]
        call collide_row

        movd xmm14, [rsp + 60]

        .walls_collision:
        movss xmm2, [ball_dx]
        movss xmm12, [ball_dy]
        movss xmm1, [ball_max]
        movss xmm4, [ball_min] ; == ball ray
        ; x axis
        ucomiss xmm0, xmm1
        ja .invertx
        ucomiss xmm0, xmm4
        jae .movex
        .invertx:
        mulss xmm2, [minus_one]
        movss [ball_dx], xmm2
        .movex:
        mulss xmm2, xmm14 ; ds = dv * dt
        addss xmm0, xmm2
        movss [ball_x], xmm0

        ; y axis
        movss xmm10, [ball_y]
        ucomiss xmm10, xmm1
        ja .inverty
        ucomiss xmm10, xmm4
        jae .movey
        .inverty:
        mulss xmm12, [minus_one]
        movss [ball_dy], xmm12
        .movey:
        mulss xmm12, xmm14 ; ds = dv * dt
        addss xmm10, xmm12
        movss [ball_y], xmm10

        ; render
        .render_image:
        lea rdi, [image]
        mov esi, WINDOW_W
        mov edx, WINDOW_H
        call render_game

        .draw_image:
        mov rdi, [rsp] ; socket fd
        mov rsi, [rsp + 16] ; window id
        mov edx, [rsp + 20] ; gc id
        lea rcx, [image]
        mov r8d, WINDOW_H
        shl r8d, 16
        or r8d, WINDOW_W
        xor r9d, r9d
        ; pass image size on the stack
        sub rsp, 16 ; maintain 16-byte alignment
        mov dword [rsp], WINDOW_SIZE
        call x11_put_image_ext
        add rsp, 16

        jmp .loop

    add rsp, 64
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
    .write:
    mov rax, SYS_WRITE
    mov rdi, qword [rsp + 1024 - 8] ; fd
    lea rsi, [rsp]
    syscall

    cmp rax, EAGAIN
    je .write
    cmp rax, rdx
    jnz die

    add rsp, 1024

    pop rbp
    ret

; Clears the window 
; @param rdi The socket file descriptor
; @param esi The window id
; @param edx packed width, height
x11_clear_window:
static x11_clear_window:function
    push rbp
    mov rbp, rsp

    sub rsp, 16

    %define X11_OP_CLEAR_AREA 0x3D
    %define REQUEST_LENGTH 0x4

    mov dword [rsp + 0*4], X11_OP_CLEAR_AREA | (REQUEST_LENGTH << 16)
    mov dword [rsp + 1*4], esi
    mov dword [rsp + 2*4], 0
    mov dword [rsp + 3*4], edx

    .write:
    mov rax, SYS_WRITE
    mov rdx, (REQUEST_LENGTH * 4)
    lea rsi, [rsp]
    syscall

    cmp rax, EAGAIN
    je .write

    cmp rax, rdx
    jnz die

    add rsp, 16

    pop rbp
    ret

int_to_ascii:
    ; functions that converts unsigned integer to ascii
    ; returns pointer to beginning of the string
    mov r8, buffer_end - 1 ; last byte should be 0 for a null terminated string
    xor rax, rax
    mov eax, edi
    mov r9, 10
.loop:
    xor edx, edx
    dec r8
    div r9 ; eax quotient, edx reminder
    add edx, '0' ; convert reminder to ASCII
    mov [r8], dl
    test eax, eax
    jz .done
    jmp .loop
.done:
    mov rax, r8
    ret


; Put image in a X11 window, using extended length protocol (this one is by me)
; @param rdi The socket file descriptor
; @param esi The window id
; @param edx The gc id
; @param rcx The image data
; @param r8d packed width, height
; @param r9d Packed x and y
; @param [rsp] Image size
x11_put_image_ext:
static x11_put_image:function
    push rbp
    mov rbp, rsp

    sub rsp, 1024

    mov dword [rsp + 2*4], esi ; drawable
    mov dword [rsp + 3*4], edx ; gcontext
    mov dword [rsp + 4*4], r8d ; width, height
    mov dword [rsp + 5*4], r9d ; x, y
    mov dword [rsp + 6*4], 24 ; depth
    shl dword [rsp + 6*4], 8 ; left-pad is zero

    mov qword [rsp + 1024 - 8], rdi ; store the socket file descriptor on the stack to free the register
    mov qword [rsp + 1024 - 16], rcx ; store the image data on the stack

    %define X11_OP_PUT_IMAGE 0x48
    %define FORMAT 0x2 ; ZPixmap
    mov dword [rsp + 0*4], X11_OP_PUT_IMAGE | (FORMAT << 8)

    ; image size (+16 because [rbp] contains previous rbp and [rbp + 8] contains the return address pushed by call
    mov dword edi, [rbp + 16]
    call calc_padding
    mov r9d, eax ; p
    mov eax, edi ; n
    add eax, r9d ; n + p
    shr eax, 2 ; (n + p) / 4
    add eax, 7 ; 7 + (n + p) / 4

    mov dword [rsp + 1*4], eax ; extended length field

    ; write header first
    .write_header:
    mov rdx, 28
    mov rdi, qword [rsp + 1024 - 8] ; fd
    mov rax, SYS_WRITE
    lea rsi, [rsp]
    syscall

    cmp rax, rdx
    jnz die

    ; now write data
    .write_data:
    mov dword edx, [rbp + 16] ; total number of bytes to write
    mov rdi, qword [rsp + 1024 - 8] ; fd
    mov r10, qword [rsp + 1024 - 16] ; pointer to data

    .write:
    mov rax, SYS_WRITE
    mov rsi, r10
    syscall

    cmp rax, rdx
    jz .done
    cmp rax, EAGAIN
    je .write
    add r10, rax ; increment image data pointer by number of bytes written
    sub rdx, rax
    jmp .write

    .done:
    add rsp, 1024
    pop rbp
    ret

; Determines if the named extension is present
; @param rdi The socket file descriptor
; @param rsi The extension name
; @param edx Length of extension name
x11_query_extension:
static x11_query_extension:function
    push rbp
    mov rbp, rsp

    sub rsp, 1024

    %define X11_OP_QUERY_EXTENSION 0x62
    mov qword [rsp + 1024 - 8], rdi 

    mov r8d, edx
    mov edi, edx
    call calc_padding
    mov r9d, eax ; p
    mov eax, r8d ; n
    add eax, r9d ; n + p
    shr eax, 2 ; (n + p) / 4
    add eax, 2 ; 2 + (n + p) / 4
    mov r10d, eax
    shl eax, 16
    or eax, X11_OP_QUERY_EXTENSION
    mov dword [rsp + 0*4], eax
    mov dword [rsp + 1*4], r8d

    lea rdi, [rsp + 2*4]
    cld
    mov ecx, r8d
    rep movsb

    .write:
    mov rdx, r10
    imul rdx, 4
    mov rax, SYS_WRITE
    lea rsi, [rsp]
    mov rdi, qword [rsp + 1024 - 8] ; fd
    syscall

    cmp rax, EAGAIN
    je .write

    cmp rax, rdx
    jnz die

    add rsp, 1024

    pop rbp
    ret

; Enable big request extension
; @param rdi The socket file descriptor
; @param rsi The extension opcode
x11_big_req_enable:
static x11_big_req_enable:function
    push rbp
    mov rbp, rsp
    sub rsp, 16

    mov dword [rsp], 1
    shl dword [rsp], 16
    or [rsp], rsi

    mov rdx, 4
    .write:
    mov rax, SYS_WRITE
    lea rsi, [rsp]
    syscall

    cmp rax, EAGAIN
    je .write

    cmp rax, rdx
    jnz die

    add rsp, 16
    pop rbp
    ret

; @param rdi: pointer to string
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

; @param rdi: pointer to message
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

; @param rdi: pointer to message
println:
    ; void println(char* msg);
    call print
    lea rdi, [newline]
    call print
    ret

; @param rdi: pointer to data
; @param sil: filler byte
; @param rdx: length of data
memset_byte:
    mov al, sil
    mov rcx, rdx
    rep stosb
    ret

; @param rdi: pointer to image data
; @param esi: rgb color, encoded as 0xXXRRGGBB
; @param rdx: length of data
color_image:
    mov eax, esi
    mov rcx, rdx
    rep stosd
    ret

; @param xmm0: x coord (normalized)
; @param xmm1: y coord (normalized)
; @return eax: rgb color
ball_fragment:
    movss xmm2, [ball_x]
    movss xmm3, [ball_y]
    movss xmm4, [ball_ray]

    ; x_coord >= ball_x - ball_ray and x_coord < ball_x + ball_ray
    subss xmm2, xmm4
    ucomiss xmm0, xmm2
    jb .black
    movss xmm2, [ball_x]
    addss xmm2, xmm4
    ucomiss xmm0, xmm2
    jae .black

    ; y_coord >= ball_y - ball_ray and y_coord < ball_y + ball_ray
    subss xmm3, xmm4
    ucomiss xmm1, xmm3
    jb .black
    movss xmm3, [ball_y]
    addss xmm3, xmm4
    ucomiss xmm1, xmm3
    jae .black

    mov eax, BALL_COLOR
    jmp .done
    .black:
    mov eax, 0
    .done:
    ret


; @param xmm0: x coord (normalized)
; @param xmm1: y coord (normalized)
; @return eax: rgb color
ball_fragment_old:
    movss xmm2, [ball_x]
    movss xmm3, [ball_y]
    movss xmm4, [ball_ray]

    ; (x_coord - ball_x)^2 + (y_coord - ball_y)^2 <= ball_ray^2
    mulss xmm4, xmm4 ; ray^2
    subss xmm0, xmm2
    mulss xmm0, xmm0 ; x^2
    subss xmm1, xmm3
    mulss xmm1, xmm1 ; y^2
    addss xmm0, xmm1
    ucomiss xmm0, xmm4
    ja .black
    mov eax, BALL_COLOR
    jmp .done
    .black:
    mov eax, 0
    .done:
    ret

; @param xmm0: x coord (normalized)
; @param xmm1: y coord (normalized)
; @return eax: rgb color
pad_fragment:
    movss xmm2, [pad_x]
    movss xmm3, [pad_y]
    movss xmm4, [pad_width]
    movss xmm5, [pad_height]

    ; if x >= pad_x and x < (pad_x + width) and y >= pad_y and y < (pad_y + height) 
    ucomiss xmm0, xmm2
    jb .black
    addss xmm2, xmm4
    ucomiss xmm0, xmm2
    jae .black

    ucomiss xmm1, xmm3
    jb .black
    addss xmm3, xmm5
    ucomiss xmm1, xmm3
    jae .black

    mov eax, PAD_COLOR
    jmp .done
    .black:
    mov eax, 0
    .done:
    ret

; @param xmm0: x coord (normalized)
; @param xmm1: y coord (normalized)
; @return eax: rgb color
bricks_fragment:
    mov rsi, NUM_BRICKS
    movss xmm2, [blue_bricks_y]
    movss xmm3, [green_bricks_y]
    movss xmm4, [yellow_bricks_y]
    movss xmm5, [orange_bricks_y]
    movss xmm6, [red_bricks_y]
    movss xmm7, [bricks_height]

    ; find which row of bricks this pixel belongs to

    ; if y >= y_brick and y < y_brick + bricks height
    .blue:
    ucomiss xmm1, xmm2
    jb .black
    addss xmm2, xmm7
    ucomiss xmm1, xmm2
    jae .green
    lea r10, [blue_bricks]
    movss xmm3, [blue_bricks_y]
    mov eax, BLUE_BRICKS_COLOR
    jmp .draw_brick

    .green:
    addss xmm3, xmm7
    ucomiss xmm1, xmm3
    jae .yellow
    lea r10, [green_bricks]
    movss xmm3, [green_bricks_y]
    mov eax, GREEN_BRICKS_COLOR
    jmp .draw_brick

    .yellow:
    addss xmm4, xmm7
    ucomiss xmm1, xmm4
    jae .orange
    lea r10, [yellow_bricks]
    movss xmm3, [yellow_bricks_y]
    mov eax, YELLOW_BRICKS_COLOR
    jmp .draw_brick

    .orange:
    addss xmm5, xmm7
    ucomiss xmm1, xmm5
    jae .red
    lea r10, [orange_bricks]
    movss xmm3, [orange_bricks_y]
    mov eax, ORANGE_BRICKS_COLOR
    jmp .draw_brick

    .red:
    addss xmm6, xmm7
    ucomiss xmm1, xmm6
    jae .black
    lea r10, [red_bricks]
    movss xmm3, [red_bricks_y]
    mov eax, RED_BRICKS_COLOR
    jmp .draw_brick

    .draw_brick:
    ; find index of brick
    cvtsi2ss xmm4, rsi ; num bricks
    mulss xmm4, xmm0
    cvtss2si rcx, xmm4 ; round index to int
    cvtsi2ss xmm5, rcx
    ucomiss xmm5, xmm4
    jbe .next
    subss xmm5, [one] ; floor
    .next:
    ; xmm5 has the index
    cvtss2si rcx, xmm5
    ; check if brick exists
    mov dil, byte [r10 + rcx]
    test dil, dil
    jz .black
    ; check if point is in bricks border
    movss xmm10, [bricks_border]
    cvtsi2ss xmm4, rsi
    movss xmm6, [one]
    divss xmm6, xmm4 ; width of brick
    mulss xmm5, xmm6 ; brick_x
    ; x < (brick_x + border) or x >= (brick_x + width - border)
    movss xmm11, xmm5
    addss xmm11, xmm10
    ucomiss xmm0, xmm11
    jb .border
    movss xmm11, xmm5
    addss xmm11, xmm6
    subss xmm11, xmm10
    ucomiss xmm0, xmm11
    jae .border
    movss xmm2, [bricks_height]
    ; y < (brick_y + border) or y >= (brick_y + height - border)
    movss xmm11, xmm3 ; brick_y
    addss xmm11, xmm10
    ucomiss xmm1, xmm11
    jb .border
    movss xmm11, xmm3
    addss xmm11, xmm2
    subss xmm11, xmm10
    ucomiss xmm1, xmm11
    jae .border
    jmp .done
    .border:
    mov eax, BORDER_BRICKS_COLOR
    jmp .done
    .black:
    mov eax, 0
    .done:
    ret

; @param rdi: pointer to bricks row
; @param xmm15: top_bricks_y
; @param xmm11: bricks_height
collide_row:
    push rbp
    mov rbp, rsp

    sub rsp, 64

    movss xmm4, [ball_x]
    addss xmm4, xmm3 ; xmm4 = ball_right
    movss xmm5, [ball_x]
    subss xmm5, xmm3 ; xmm5 = ball_left

    ; clamp to [0,1]
    movss xmm7, [zero]
    ucomiss xmm5, xmm7
    jae .one
    movss xmm5, xmm7 ; 0
    .one:
    movss xmm7, [one]
    ucomiss xmm4, xmm7
    jb .calc_collision
    .one_clamp:
    movss xmm4, xmm7 ; 1
    subss xmm4, [point_one]

    .calc_collision:
    mov rsi, NUM_BRICKS
    cvtsi2ss xmm6, rsi ; num bricks
    mulss xmm6, xmm5
    cvtss2si rcx, xmm6 ; round index to int
    cvtsi2ss xmm5, rcx
    ucomiss xmm5, xmm6
    jbe .rightmost
    subss xmm5, [one] ; floor
    .rightmost:
    ; xmm5 has index of leftmost
    cvtsi2ss xmm6, rsi ; num bricks
    mulss xmm6, xmm4
    cvtss2si rcx, xmm6 ; round index to int
    cvtsi2ss xmm4, rcx
    ucomiss xmm4, xmm6
    jbe .check_coll
    subss xmm6, [one] ; floor
    .check_coll:
    ; xmm6 has index of rightmost

    ; now iterate from leftmost brick to rightmost brick and check for ball to rectangle collision
    ; for now I'm going to do aabb because it's easier

    ; just erase all the touched bricks
    cvtss2si rcx, xmm5
    cvtss2si rdx, xmm6

    ; debug
    mov [rsp], rdi
    mov [rsp + 8], rcx
    mov [rsp + 16], rdx

    mov rdi, rcx
    call int_to_ascii
    mov rdi, rax
    call println

    mov rdi, [rsp + 16]
    call int_to_ascii
    mov rdi, rax
    call println
    
    mov rdi, [rsp]
    mov rcx, [rsp + 8]
    mov rdx, [rsp + 16]

    movss xmm13, [one]
    mov r8d, NUM_BRICKS
    cvtsi2ss xmm12, r8d
    divss xmm13, xmm12 ; xmm13 = bricks width
    
    .brick_loop: 
    mov al, byte [rdi + rcx]
    test al, al
    jz .continue
    ; collision
    mov byte [rdi + rcx], 0 ; destroy brick

    ; check which side is colliding, reverse velocity depending on the side
    ; min(
    ;    dist(ball_y, brick_top),
    ;    dist(ball_y, brick_bottom),
    ;    dist(ball_x, brick_left),
    ;    dist(ball_x, brick_right),
    ;)

    movd xmm10, [sign_bit_mask]
    movss xmm7, [ball_y]
    subss xmm7, xmm15
    ; abs (set sign bit to 0)
    andps xmm7, xmm10 ; xmm7 = dist(ball_y, brick_top)
    movss xmm8, [ball_y]
    subss xmm15, xmm11
    subss xmm8, xmm15
    andps xmm8, xmm10 ; xmm8 = dist(ball_y, brick_bottom)

    movss xmm9, [ball_x]
    ; get left and right border of this brick, using index and width
    cvtsi2ss xmm5, rcx ; index of brick
    mulss xmm5, xmm13 ; brick_left
    movss xmm14, xmm5
    addss xmm14, xmm13 ; brick_right
    subss xmm5, xmm9
    andps xmm5, xmm10 ; xmm5 = dist(ball_x, brick_left)
    subss xmm14, xmm9
    andps xmm14, xmm10 ; xmm14 = dist(ball_x, brick_right)

    movss xmm13, [zero]

    ; find closest side
    ucomiss xmm7, xmm8
    jae .next1
    ucomiss xmm7, xmm5
    jae .next2
    ucomiss xmm7, xmm14
    jae .next3
    ; min = xmm7
    jmp .goup
    .next3:
    ; min = xmm14
    jmp .goright
    .next2:
    ucomiss xmm5, xmm14
    jae .next3
    ; min = xmm5
    jmp .goleft
    .next1:
    ucomiss xmm8, xmm5
    jae .next2
    ucomiss xmm8, xmm14
    jae .next3
    ; min = xmm8
    jmp .godown

    .goup:
    movss xmm15, [ball_dy]
    ucomiss xmm15, xmm13
    jae .continue
    jmp .inverty

    .godown:
    movss xmm15, [ball_dy]
    ucomiss xmm15, xmm13
    jbe .continue
    jmp .inverty

    .goleft:
    movss xmm15, [ball_dx]
    ucomiss xmm15, xmm13
    jbe .continue
    jmp .invertx

    .goright:
    movss xmm15, [ball_dx]
    ucomiss xmm15, xmm13
    jae .continue
    jmp .invertx

    .inverty:
    mulss xmm15, [minus_one]
    movss [ball_dy], xmm15
    jmp .continue

    .invertx:
    mulss xmm15, [minus_one]
    movss [ball_dx], xmm15
    jmp .continue

    .continue:
    cmp rcx, rdx
    jz .done
    inc rcx
    jmp .brick_loop
    .done:
    add rsp, 64
    pop rbp
    ret

; @param rdi: pointer to image data
; @param esi: image width
; @param edx: image height
render_game:
    push rbp
    mov rbp, rsp

    sub rsp, 1024

    mov qword [rsp], rdi

    xor r8, r8 ; row index
    xor r9, r9 ; col index

    mov r12d, esi 
    mov r13d, edx
        
    cvtsi2ss xmm14, esi ; width
    cvtsi2ss xmm15, edx ; height
    movss xmm5, [max_rgb]

    .row_loop:
    cmp r8d, r13d
    jge .done
    .col_loop:
    cmp r9d, r12d
    jge .next_row

    movss xmm9, [one]
    ; u coord
    cvtsi2ss xmm2, r9
    divss xmm2, xmm14
    movss [rsp + 8], xmm2

    ; v coord
    cvtsi2ss xmm3, r8
    divss xmm3, xmm15
    movss xmm4, xmm3
    movss xmm3, xmm9
    subss xmm3, xmm4
    movss [rsp + 12], xmm3

    ; render ball
    movss xmm0, xmm2 ; u
    movss xmm1, xmm3 ; v
    call ball_fragment
    cmp eax, 0
    jne .color

    ; render pad
    movss xmm0, [rsp + 8] ; u
    movss xmm1, [rsp + 12] ; v
    call pad_fragment
    cmp eax, 0
    jne .color

    ; render bricks
    movss xmm0, [rsp + 8]
    movss xmm1, [rsp + 12]
    call bricks_fragment
    cmp eax, 0
    jne .color

    .color:
    mov r10d, eax ; rgb color
    
    ; calculate pointer offset
    mov eax, r8d
    mul r12d
    add eax, r9d
    imul eax, 4
    
    mov rdi, [rsp]
    mov dword [rdi + rax], r10d

    inc r9
    jmp .col_loop
    .next_row:
    xor r9, r9
    inc r8
    jmp .row_loop

    .done:
    add rsp, 1024
    pop rbp
    ret


; updates [cur_time] with the current time
update_current_time:
    mov rax, SYS_GETTIME
    xor rdi, rdi
    lea rsi, [timespec]
    syscall
    cmp rax, 0
    jl die
    mov edi, [timespec] ; seconds
    mov rsi, [timespec + 8] ; nanoseconds
    cvtsi2sd xmm0, edi
    cvtsi2sd xmm1, rsi
    divsd xmm1, [one_billion]
    addsd xmm0, xmm1
    movsd [cur_time], xmm0
    ret

_start:
    call x11_connect_to_server
    mov r15, rax ; Store the socket file descriptor in r15.

    mov rdi, rax
    call x11_send_handshake

    mov r12d, eax ; Store the window root id in r12.

    call x11_next_id
    mov r14d, eax ; Store the font_id in r14.

    ; check if big request extension is present
    mov rdi, r15
    lea rsi, [big_req_ext_name]
    mov edx, [big_req_ext_len]
    call x11_query_extension
    call x11_read_extension_reply
    test rax, rax
    jz die

    ; enable big request extension
    mov rsi, rax
    call x11_big_req_enable
    call x11_read_reply
    test rax, rax
    jz die

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

    ; init cur_time
    call update_current_time
    movsd xmm0, [cur_time]
    movsd [prev_time], xmm0

    ; init bricks
    lea rdi, [blue_bricks]
    mov rsi, 1
    mov rdx, NUM_BRICKS
    call memset_byte

    lea rdi, [green_bricks]
    mov rsi, 1
    mov rdx, NUM_BRICKS
    call memset_byte

    lea rdi, [yellow_bricks]
    mov rsi, 1
    mov rdx, NUM_BRICKS
    call memset_byte

    lea rdi, [orange_bricks]
    mov rsi, 1
    mov rdx, NUM_BRICKS
    call memset_byte

    lea rdi, [red_bricks]
    mov rsi, 1
    mov rdx, NUM_BRICKS
    call memset_byte

    mov rdi, r15 ; socket fd
    mov esi, ebx ; window id
    mov edx, r13d ; gc id
    call poll_messages

    mov rax, SYS_EXIT
    xor rdi, rdi ; exit 0
    syscall

section .data

newline: dd 10

sun_path: db "/tmp/.X11-unix/X0", 0

big_req_ext_name: db "BIG-REQUESTS"
big_req_ext_len: dd 12
id: dd 0
id_base: dd 0
id_mask: dd 0
root_visual_id: dd 0

; ball
ball_ray: dd 0.012
ball_min: equ ball_ray
ball_max: dd 0.988 ; 1 - ball_ray
ball_x: dd 0.05
ball_y: dd 0.8
ball_dx: dd 0.5
ball_dy: dd 0.5

; pad
pad_x: dd 0.5
pad_dx: dd 1.0
pad_y: dd 0.1
pad_width: dd 0.15
pad_height: dd 0.025
pad_min: dd 0.0
pad_max: dd 0.85 ; 1 - pad_width

; bricks
bricks_height: dd 0.025
blue_bricks_y: dd 0.5
green_bricks_y: dd 0.525
yellow_bricks_y: dd 0.550
orange_bricks_y: dd 0.575
red_bricks_y: dd 0.6
bricks_border: dd 0.0024
;bricks_border: dd 0.0045

max_rgb: dd 255.0 
one: dd 1.0
zero: dd 0.0
point_one: dd 0.000001
minus_one: dd -1.0
sign_bit_mask: dd 0x7FFFFFFF

cur_time: dq 0.0
prev_time: dq 0.0
one_billion: dq 1000000000.0 

; debug utils
blue: db "BLUE", 0
green: db "GREEN", 0
yellow: db "YELLOW", 0
orange: db "ORANGE", 0
red: db "RED", 0

section .bss

timespec: resb 16

image: resb WINDOW_SIZE

blue_bricks: resb NUM_BRICKS
green_bricks: resb NUM_BRICKS
yellow_bricks: resb NUM_BRICKS
orange_bricks: resb NUM_BRICKS
red_bricks: resb NUM_BRICKS

; general purpose buffer
buffer:
    resb 256
buffer_end:

; TODO: cap framerate to 60 fps (nanosleep)
