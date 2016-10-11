;==============================================================================
; Oscillogram for SoundBlaster v1.10
;   Craig Jackson & Andr‚ Baresel
;------------------------------------------------------------------------------
; Requirements: 8086, DOS 1.0, VGA/MCGA, SoundBlaster (see BASEADDR)
; Supports    : Microphone, Line, CD
; Resolutions : 8-bit
;------------------------------------------------------------------------------

    .MODEL tiny
    .8086

    BASEADDR    EQU 0220h               ;SoundBlaster base address
    VIDEOCOLS   EQU 320                 ;Video columns
    VIDEOROWS   EQU 200                 ;Video rows
    SAMPLERANGE EQU 128                 ;Range for scaled samplevalues

    .CODE
    .STARTUP

    ; Initialize graphics
    mov     ax,00013h                   ;Set video mode (320x200x256)
    int     010h                        ;   Interrupt: Video
    mov     ax,0A000h                   ;AX = Video segment
    mov     es,ax                       ;ES = Video segment

loopMain:
    ; Clear screen
    mov     di,(VIDEOROWS-SAMPLERANGE)/2*VIDEOCOLS
                                           ;DI = Offset for first column top
    mov     cx,(VIDEOCOLS * SAMPLERANGE)/2 ;CX = Count of words in active area
    mov     ax,00101h                      ;AX = C:C
    rep     stosw                          ;Clear screen

    ; Draw oscillogram
    mov     cx,VIDEOCOLS                ;CX = Video columns
    mov     di,(VIDEOROWS-SAMPLERANGE)/2*VIDEOCOLS
                                        ;DI = Offset for first column top
loopRefresh:
    mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
    mov     al,020h                     ;AL = Direct ADC
    out     dx,al                       ;   Output: DSP Write Data or Command

    add     dx,002h                     ;DX = DSP Data Available Status
L0: in      al,dx                       ;
    or      al,al                       ;Check for available sample
    jns     L0                          ;   Jump: Continue available sample check

    sub     dx,004h                     ;DX = DSP Read Data
    in      al,dx                       ;AL = ADC Data

    push    di                          ;Preserve DI
    mov     dl,VIDEOCOLS/2              ;DL = Video columns / scalar
    and     al,0FEh                     ;AL = ADC Data forced scalar aligned
    mul     dl                          ;AX = Video row offset for sample
    add     di,ax                       ;DI = Video offset to pixel
    mov     ax,00107h                   ;AH = Check for character function
                                        ;AL = Color for oscillogram
    stosb                               ;Store sample
    pop     di                          ;Restore DI

    inc     di                          ;DI = Offset for next video column
    int     016h                        ;   Interrupt: Keyboard
    loopz   loopRefresh                 ;   Loop: Continue sampling loop, until keypress
    jz      loopMain                    ;   Jump: Refresh screen, until keypress

    ; Terminate program
    xor     ah,ah                       ;Read character, flush keypress
    int     016h                        ;   Interrupt: Keyboard

    mov     ax,00003h                   ;Set video mode (80x25x16)
    int     010h                        ;   Interrupt: Video

    ret                                 ;Terminate program (old style)

    END
