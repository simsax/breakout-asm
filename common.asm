%ifndef COMMON_ASM
%define COMMON_ASM

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

%endif
