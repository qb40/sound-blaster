;мммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммммм
; Play 8bit DMA mode for SoundBlaster v1.00
;   Andr┌ Baresel (with some help from Craig Jackson)
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; Requirements: 80286, DOS 1.0, SoundBlaster (see BASEADDR,DMA channel,IRQ number)
; Resolutions : 8-bit / 4..23khz (no highspeed - but on SB16 4..44kHz)
; Parameters  : none
; Notes:
;  Ч To creat a 8 bit mono unsigned file do :   "VOC2RAW TEST1.VOC /I"
;
; Ч DSP command 48h  ... set DMA block size
; Ч DSP command 1Ch  ... play 8bit mono autoinit
; Ч DSP command 40h  ... set sample rate
; Ч DSP command D1h  ... Enable Speaker
; Ч DSP command D3h  ... Disable Speaker
;

.MODEL small
.286

; CONSTANTS ддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд

; SoundBlaster SETUP
BASEADDR           EQU 0220h       ;SoundBlaster base address
IRQ7               EQU 15          ;SoundBlaster IRQ
DMAchannel         EQU 1           ;SoundBlaster DMA channel

; PIC MASKS FOR MASK/DEMASK IRQ
PICANDMASK         EQU 01111111b   ;'AND' PIC mask for clear IRQ7
PICORMASK          EQU 10000000b   ;'OR' PIC mask for set IRQ7

; DMA CONTROLLER REGISTERS :
WRITEMASK          EQU 00ah         ;WRITE MASK REGISTER
WRITEMODE          EQU 00bh         ;WRITE MODE REGISTER
CLEARFLIPFLOP      EQU 00ch
PAGE_CHN           EQU 083h         ;PAGE REGISTER FOR DMAchannel 1
BASE_CHN           EQU 002h         ;BASEADDRESS REGISTER DMA 1
COUNT_CHN          EQU 003h         ;COUNT REGISTER DMAchannel 1

; SAMPLERATE : (if you change it pay attention to maximum samplerate)
TIMECONST          EQU 165          ; = 10989 Hz (256-1000000/11000)

; DMA WRITE MODE
WANTEDMODE         EQU 01011000b    ; singlemode, autoinit, readmode


;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; MACRO DEFINITIONs
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
STARTUP                 MACRO
; MASM 5.x COMPATIBILITY
__start:                mov     ax,DGROUP
                        mov     ds,ax
                        mov     bx,ss
                        sub     bx,ax
                        shl     bx,004h
                        mov     ss,ax
                        add     sp,bx
ENDM

WAITWRITE               MACRO
LOCAL                   loopWait,endloop
;          Arguments : DX = Status port (BASEADDR+0Ch)
;          Returns   : n/a
;          Destroys  : AL

                        push    cx
                        xor     cx,cx           ; need that for slow SBs !
loopWait:               dec     cx
                        jz      endloop
                        in      al,dx           ; AL = WRITE COMMAND STATUS
                        or      al,al
                        js      loopWait        ; Jump if bit7=1 - writing not allowed
endloop:                pop     cx
ENDM

WAITREAD                MACRO
LOCAL                   loopWait,endloop
;          Arguments : DX = Status port   (normaly BASEADDR+0Eh)
;          Returns   : n/a
;          Destroys  : AL

                        push    cx
                        xor     cx,cx           ; need that for slow SBs !
loopWait:               dec     cx
                        jz      endloop
                        in      al,dx           ; AL = DATA AVAILABLE STATUS
                        or      al,al
                        jns     loopWait        ; Jump if bit7=0 - no data available
endloop:                pop     cx
ENDM

RESET_DSP               MACRO
local                   SBthere
;          Arguments : n/a
;          Returns   : n/a
;          Destroys  : DX,AL

                        mov      dx,BASEADDR+06h
                        mov      al,1
                        out      dx,al          ; start DSP reset

                        in       al,dx
                        in       al,dx
                        in       al,dx
                        in       al,dx          ; wait 3 Фsec

                        xor      al,al
                        out      dx,al          ; end DSP Reset

                        add      dx,08h         ; dx = DSP DATA AVAILABLE
                        WAITREAD
                        sub      dx,4           ; dx = DSP Read Data
                        in       al,dx
                        cmp      al,0aah        ; if there is a SB then it returns 0AAh
                        je       SBthere
                        jmp      RESET_ERROR    ; No SB - exit program
SBthere:
ENDM
;ддд End of Macrodefinitions дддддддддддддддддддддддддддддддддддддддддддддддддд

.STACK 100h

.DATA
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; TWO COPIES FOR PAGE OVERRIDE REASONS :

SAMPLEBUFFER LABEL BYTE
    INCLUDE TEST1.INC        ; FIRST COPY OF SAMPLE SOUND
SAMPLEBUFFEREND LABEL BYTE
    INCLUDE TEST1.INC        ; SECOND COPY OF SAMPLE SOUND

    part                db 1

    information         db 13,10,'DMASTEP3.EXE - repeat a 8bit mono sound again and again'
                        db 13,10,'using autoinit mode (not highspeed) - hope you',39,'re not to bored ;)'
                        db 13,10,'stop playing with <ESC> ...','$'
    txtpart0            db 13,10,'playing part 0','$'
    txtpart1            db 13,10,'playing part 1','$'
    sberror             db 13,10,'No SoundBlaster at this BASEADDR ! PROGRAM HALTED.','$'

    OLDInterruptSEG     dw ?
    OLDInterruptOFS     dw ?

    SAMPLEBUFFERLENGTH = offset SAMPLEBUFFEREND - offset SAMPLEBUFFER
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
.CODE
 STARTUP

           RESET_DSP

           ; WRITE INFORMATION TO SCREEN :
           mov     dx,offset information
           mov     ah,9
           int     21h                         ; write program information to screen

           ; ENABLE SB SPEAKERS (for all SBs <SB16)
           mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
           WAITWRITE
           mov     al,0D1h                     ;AL = Enable speaker
           out     dx,al                       ;   Output: DSP Write Data or Command

           ; SETUP IRQ :
           xor     ax,ax
           mov     es,ax                       ; es to page 0 (Interrupt table)
           mov     si,IRQ7*4                   ; si = position in interrupt table

           ; DISABLE IRQ
           in      al,021h
           and     al,PICANDMASK               ; SET MASK REGISTER BIT TO DISABLE INTERRUPT
           out     021h,al

           ; CHANGE POINTER IN INTERRUPT TABLE
           mov     ax,es:[si]
           mov     [OLDInterruptOFS],ax        ; save offset of old interupt vector for restoring
           mov     ax,OFFSET OWN_IRQ
           mov     es:[si],ax                  ; set offset of new interrupt routine
           mov     ax,es:[si+2]
           mov     [OLDInterruptSEG],ax        ; save segment of old interupt vector for restoring
           mov     ax,cs
           mov     es:[si+2],ax                ; set segment of new interrupt routine

           ; CHANGE PIC MASK :
           in      al,021h
           and     al,PICANDMASK   ; CLEAR MASK REGISTER BIT TO ENABLE INTERRUPT
           out     021h,al

;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; calculate page and offset for DMAcontroller :
;
; segment*16+offset - 20bit memory location -> upper 4 bits  = page
;                                              lower 16 bits = offset
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
           mov     si,offset samplebuffer
           mov     cx,SAMPLEBUFFERLENGTH-1

           mov     ax,ds
           rol     ax,4                ; * 16 - higher 4 bits in al
           mov     bl,al
           and     bl,00fh             ; BL - higher 4 bits
           and     al,0f0h             ; clear higher 4bits in AL
           add     si,ax               ; SI = offset
           adc     bl,0                ; BL = page
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; check for DMApage override :
; ... problem: DMA controller separates memory into 64KB pages, you can only
; transfer data is placed in one page - no page overrides are allowed
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; To solve that :
; creat a DMA buffer with double size you want - if the first part is placed
; on a page border the second part is for sure not
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
           neg     si          ; si = 65536 - si   (bytes left to DMA page border)
           cmp     si,cx       ; if si (bytes left to border) > cx (bytes to play)
           ja      nooverride  ; then there's no page override

           ; WE HAVE TO USE SECOND PART
           neg     si          ; si = offset of first part
           add     si,cx       ; si = si + length of one part
           inc     si          ; si=si+1 - start of second part
           inc     bl          ; second part is then on the next page
           neg     si          ; look at the next command ;)
                               ; (that is better than a jump ?)
nooverride:
           neg     si

;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; Setup DMA-controller :
;
; 1st  MASK DMA CHANNEL
;
           mov     al,DMAchannel
           add     al,4
           out     WRITEMASK,al
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; 2nd  CLEAR FLIPFLOP
;
           out     CLEARFLIPFLOP,al
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; 3rd  WRITE TRANSFER MODE
;
           mov     al,WANTEDMODE
           add     al,DMAchannel
           out     WRITEMODE,al
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; 4th  WRITE PAGE NUMBER
;
           mov     al,bl
           out     PAGE_CHN,al
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; 5th  WRITE BASEADDRESS
;
           mov     ax,si
           out     BASE_CHN,al
           mov     al,ah
           out     BASE_CHN,al
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; 6th  WRITE SAMPLELENGTH-1
;
           mov     al,cl
           out     COUNT_CHN,al
           mov     al,ch
           out     COUNT_CHN,al
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; 7th  DEMASK CHANNEL
;
           mov     al,DMAchannel
           out     WRITEMASK,al

;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; Setup SoundBlaster :
;
; 1st  SET TIMECONSTANTE
;
           mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
           WAITWRITE
           mov     al,040h                     ;AL = Set timeconstant
           out     dx,al
           WAITWRITE
           mov     al,TIMECONST
           out     dx,al

;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; 2nd  use DMAplay 8bit mono autoinit (DSPcommand-01ch)
;
           ; SETUP SIZE
           WAITWRITE
           mov     al,048h                     ;AL = DMA DAC 8bit
           out     dx,al
           mov     cx,SAMPLEBUFFERLENGTH
           shr     cx,1                        ; generate IRQ every half buffer
           dec     cx
           WAITWRITE
           mov     al,cl                       ;AL = LOWER PART SAMPLELENGTH
           out     dx,al
           WAITWRITE
           mov     al,ch                       ;AL = HIGHER PART SAMPLELENGTH
           out     dx,al

           ; SETUP PLAYMODE
           WAITWRITE
           mov     al,1ch                      ;AL = Auto-Initialize DMA DAC, 8-bit
           out     dx,al

; TRANSFER STARTS ....... NOW ..... :)

waitloop:
           mov     ah,01                       ;AH = Check for character function
           int     016h                        ;   Interrupt: Keyboard
           jz      waitloop
           xor     ah,ah
           int     016h
           cmp     al,27                       ; wait for <ESC>
           jne     waitloop

           ; RESET SOUNDBLASTER
           RESET_DSP

exit:      ; RESTORE PIC MASK
           in      al,021h
           or      al,PICORMASK                ;<-- SET REGISTER MASK BITS TO DISABLE
           out     021h,al

           ; RESTORE IRQ :
           xor     ax,ax
           mov     es,ax                       ; es to page 0 (Interrupt table)
           mov     si,IRQ7*4
           mov     ax,[OLDInterruptOFS]
           mov     es:[si],ax                  ; set old interrupt routine
           mov     ax,[OLDInterruptSEG]
           mov     es:[si+2],ax

           ; CLEAR KEYBUFFER
           mov     ah,01                       ;AH = Check for character function
           int     16h                         ;   Interrupt: Keyboard
           jz      return2dos
           xor     ah,ah                       ;Read character, flush keypress
           int     016h                        ;   Interrupt: Keyboard

           ; TERMINATE EXE:
return2dos:
           mov     ax,04c00h
           int     21h

; display information if Soundblaster is not on this baseaddress
RESET_ERROR:
           mov     dx,offset sberror
           mov     ah,9
           int     21h                         ; text output
           jmp     return2dos

;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
; Our own IRQ for detecting buffer half SB currently plays
; It's generated by the SoundBlaster hardware
;дддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддддд
OWN_IRQ:
           pusha
           mov     dx,BASEADDR+00Eh            ;DX = DSP DATA AVAILABLE (IRQ ACKNOWLEDGE)
           in      al,dx
           mov     ax,@data
           mov     ds,ax
           mov     dx,offset txtpart0
           cmp     [part],0
           je      notpart1
           mov     dx,offset txtpart1
notpart1:
           mov     ah,9
           int     21h             ; text output
           neg     [part]
           inc     [part]          ; part = 1-part  result : 0,1,0,1,0,....
           mov     al,020h
           out     020h,al                     ;ACKNOWLEDGE HARDWARE INTERRUPT
           popa
           IRET

END     __start
