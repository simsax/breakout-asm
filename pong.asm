%define STDIN 0
%define STDOUT 1
%define STDERR 2
%define AF_UNIX 1
%define SOCK_STREAM 1

; syscalls
%define SYS_READ 0
%define SYS_WRITE 1
%define SYS_POLL 7
%define SYS_SOCKET 41
%define SYS_CONNECT 42
%define SYS_EXIT 60
%define SYS_FCNTL 72

; graphics constants
%define WINDOW_W 800
%define WINDOW_H 600
%define BALL_W 400
%define BALL_H 400
%define BALL_SIZE (BALL_W * BALL_H * 4) ; BGRX format

global _start
section .text

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; x11 code is adapted from https://gaultier.github.io/blog/x11_x64.html ;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
        jle die

        cmp dword [rsp + 2*4], POLLERR  
        je die

        cmp dword [rsp + 2*4], POLLHUP  
        je die

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

        .clear_window:
            mov rdi, [rsp] ; socket fd
            mov rsi, [rsp + 16] ; window id
            mov edx, WINDOW_H
            shl edx, 16
            or edx, WINDOW_W
            call x11_clear_window

        .draw_ball:
            mov rdi, [rsp] ; socket fd
            mov rsi, [rsp + 16] ; window id
            mov edx, [rsp + 20] ; gc id
            lea rcx, [ball]
            mov r8d, BALL_H
            shl r8d, 16
            or r8d, BALL_W
            ;cvtss2si r9d, [ball_y]
            ;shl r9d, 16
            ;cvtss2si r10d, [ball_x]
            ;or r9d, r10d
            mov r9d, [ball_y_int]
            shl r9d, 16
            mov r10d, [ball_x_int]
            or r9d, r10d
            ; pass image size on the stack
            sub rsp, 16 ; maintain 16-byte alignment
            mov dword [rsp], BALL_SIZE
            call x11_put_image_ext
            add rsp, 16

        ;.update_ball_position:
        ;    mov r10d, [ball_x_int]
        ;    add r10d, 10
        ;    mov [ball_x_int], r10d

            ;movss xmm0, [ball_x]
            ;cvtss2si r10d, xmm0
            ;cmp r10d, (WINDOW_W - BALL_W)
            ;jge .loop
            ;addss xmm0, [ball_offset]
            ;movss [ball_x], xmm0
            
        ;jmp .loop
    .loop2:
        jmp .loop2

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

    mov rax, SYS_WRITE
    mov rdx, (REQUEST_LENGTH * 4)
    lea rsi, [rsp]
    syscall

    add rsp, 16

    pop rbp
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

    mov rdx, r10
    imul rdx, 4
    mov rax, SYS_WRITE
    lea rsi, [rsp]
    mov rdi, qword [rsp + 1024 - 8] ; fd
    syscall

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
    sub rsp, 8

    mov dword [rsp], 1
    shl dword [rsp], 16
    or [rsp], rsi

    mov rdx, 4
    mov rax, SYS_WRITE
    lea rsi, [rsp]
    syscall

    cmp rax, rdx
    jnz die

    add rsp, 8
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
    mov rdi, newline
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

_start:
    ; initialize image data
    lea rdi, [ball]
    mov rdx, (BALL_SIZE / 4) ; rdx contains number of dwords
    mov esi, 0x0000FF00 ; green
    call color_image

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

big_req_ext_name: db "BIG-REQUESTS"
static big_req_ext_name:data

big_req_ext_len: db 12
static big_req_ext_len:data

section .data

id: dd 0
static id:data

id_base: dd 0
static id_base:data

id_mask: dd 0
static id_mask:data

root_visual_id: dd 0
static root_visual_id:data

ball_x: dd 0.0
ball_y: dd 0.0
ball_offset: dd 0.1

ball_x_int: dd 0
ball_y_int: dd 0

section .bss

ball:
    resb BALL_SIZE

