;==============================================================================
; Record and play in direct mode for SoundBlaster v1.00
;   Andr‚ Baresel & Craig Jackson
;------------------------------------------------------------------------------
; Requirements: 8086, DOS 1.0, SoundBlaster (see BASEADDR)
; Supports    : Microphone, Line, CD
; Resolutions : 8-bit
;
    .MODEL tiny
    .8086

    BASEADDR           EQU 0220h       ;SoundBlaster base address
    SAMPLEBUFFERLENGTH EQU 30000       ;count of samples in buffer

;...
;   Arguments: DX = Status port
;   Returns  : n/a
;   Destroys : AL
WAITWRITE   MACRO
    LOCAL   loopWait

loopWait:
    in      al,dx                       ;AL = ...
    or      al,al                       ;...
    js      loopWait                    ;   Jump: ...
ENDM
                                        ;samplerate (samples per second) = 1000000/pause
;...
;   Arguments: DX = Status port
;   Returns  : n/a
;   Destroys : AL
WAITREAD    MACRO
    LOCAL   loopWait

loopWait:
    in      al,dx                       ;AL = ...
    or      al,al                       ;...
    jns     loopWait                    ;   Jump: ...
ENDM

    .DATA
    rectxt DB 13,10,'recording now ...','$'
    plytxt DB 13,10,'playback now ...','$'

    .DATA?
    SAMPLEBUFFER DB SAMPLEBUFFERLENGTH DUP (?)  ;Samplebuffer

    .CODE
    .STARTUP


loopMain:

    ; write an information to screen
    mov     dx,offset rectxt
    mov     ah,9
    int     21h             ; text output
    ; first record a bit
    mov     cx,SAMPLEBUFFERLENGTH               ;CX = Video columns
    mov     di,offset samplebuffer
recordloop:
    mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
    WAITWRITE                           ;
    mov     al,020h                     ;AL = Direct ADC
    out     dx,al                       ;   Output: DSP Write Data or Command

    add     dx,002h                     ;DX = DSP Data Available Status
;; -- I'm starting to use ADDs/SUBs so this stuff will work better if we
;;      decided to autodetect the BASEADDR -- also it generates shorter
;;      code (one byte of sign-extended immediate versus two bytes for MOV)
    WAITREAD                            ;
    sub     dx,004h                     ;DX = DSP Read Data
    in      al,dx                       ;AL = ADC Data (high byte on SB16)
    stosb                               ;Store sample in samplebuffer

    mov     ah,01                       ;AH = Check for character function
    int     016h                        ;   Interrupt: Keyboard
    loopz   recordloop                  ;   Loop: Continue sampling loop, until keypress
    jnz     exit                        ;   Jump: if keypressed terminate program

    add     dx,002h                     ;DX = DSP Write Data or Command
    mov     al,0D1h                     ;AL = Enable speaker
    out     dx,al                       ;   Output: DSP Write Data or Command

    ; write an information to screen
    mov     dx,offset plytxt
    mov     ah,9
    int     21h             ; text output
    ; now playback samplebuffer
    mov     si,offset samplebuffer
    mov     cx,SAMPLEBUFFERLENGTH
playbackloop:
    mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or >Command<
    WAITWRITE                           ;
    mov     al,010h                     ;AL = Direct DAC
    out     dx,al                       ;   Output: DSP Write Data or Command

    WAITWRITE                           ;
    lodsb                               ;load next byte from buffer
    out     dx,al                       ;AL = ADC Data (high byte on SB16)

    add     dx,002h                     ;DX = DSP Data Available Status
    WAITWRITE                           ;

    mov     ah,01                       ;AH = Check for character function
    int     016h                        ;   Interrupt: Keyboard
    loopz   playbackloop                ;   Loop: ...

    mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
    mov     al,0D3h                     ;AL = Disable speaker
    out     dx,al                       ;   Output: DSP Write Data or Command
    jz      loopmain                    ;   Jump: ...

exit:
    ; Terminate program
    xor     ah,ah                       ;Read character, flush keypress
    int     016h                        ;   Interrupt: Keyboard

    ret                                 ;Terminate program (old style - only with COM !!!)

;;waitdspready:
;;        push      cx
;;        mov       dx,BASEADDR+00Ch
;;        mov       cx,100
;;@@litl: in        al,dx
;;        dec       cx
;;        jz        @@ende
;;        or        al,al
;;        js        @@litl
;;@@ende: pop       cx
;;        ret
;;  -- I nuked this in favor of in-line stuff because you don't actually do
;;          anything when CX=0 on return -- macros only 5 bytes each versus
;;          3 bytes for the 'CALL xxx' (not counting code inside of function)

;;waitabit:
;;        push    cx
;;        mov     cx,pause
;;@o:     mov     ax,cx
;;        loop    @o
;;        pop     cx
;;        ret
;;  -- I think this is unnecessary if you query read/write status before
;;          doing anything

    END
