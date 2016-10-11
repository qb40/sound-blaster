;ÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ
; Play with Cxh/Bxh commands on SoundBlaster 16/16ASP (here 16/stereo/unsigned)
;   Andr‚ Baresel (with some help from Craig Jackson)
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; STATUS: DOES WORK ON SB16
; ş sound crackles after a while - dounno yet what it is ...
; (somethings going wrong in CONVERT_HALF...)
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; Requirements: 80286, SoundBlaster 16/16ASP (see BASEADDR,DMA channel,IRQ number)
; Resolutions : 16-bit / 4..44khz / stereo
; Note        : ş We use only 8 bit stereo data and convert it while playing
;                 into stereo 16bit, look at CONVERT_HALF (at the end of this file)
;               ş To creat a 8 bit stereo unsigned file do :
;                 "VOC2RAW TEST1.VOC /I /R"
;
; ş DSP command 41h  ... set sample rate
; ş DSP command D5h  ... Halt Autoinit 16 bit DMA operation
; ş DSP command D6h  ... Continue Autoinit 16 bit DMA operation
; ş DSP command B6h 20h ... autoinit 16 bit stereo data with no sign
;

.MODEL small
.286
; CONSTANTS ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ

; SoundBlaster SETUP
BASEADDR           EQU 0220h       ;SoundBlaster base address
DMAchannel         EQU 5           ;SoundBlaster DMA channel
IRQ7               EQU 15          ;SoundBlaster IRQ

; PIC MASKS FOR MASK/DEMASK IRQ
PICANDMASK         EQU 01111111b   ;AND PIC mask for clear IRQ
PICORMASK          EQU 10000000b   ;OR PIC mask for set IRQ

; DMA CONTROLLER REGISTERS (16bit)
WRITEMASK          EQU 0D4h         ;WRITE MASK REGISTER
WRITEMODE          EQU 0D6h         ;WRITE MODE REGISTER
CLEARFLIPFLOP      EQU 0D8h
PAGE16_CHN         EQU 08Bh         ;PAGE REGISTER FOR DMAchannel 5
BASE16_CHN         EQU 0C4h         ;BASEADDRESS REGISTER DMA 5
COUNT16_CHN        EQU 0C6h         ;COUNT REGISTER DMAchannel 5

; SAMPLERATE :
RATE               EQU 02AEDh       ; = 10989 Hz

; DMA MODE
WANTEDMODE         EQU 01011000b    ; singlemode, autoinit, readmode

; DMABuffer size :
DMABUFFERSIZE      EQU 8*1024

;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; MACRO DEFINITIONs
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
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
                        in       al,dx          ; wait 3 æsec

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
;ÄÄÄ End of Macrodefinitions ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ

.STACK 100h

.DATA
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; Creat TEST1.INC with calling "VOC2RAW TEST1.VOC /I /R" or creat your own
; textfile with sampledata
;
SAMPLEBUFFER LABEL BYTE
    INCLUDE TEST1.INC
SAMPLEBUFFEREND LABEL BYTE

    PART                db 0

    information         db 13,10,'DMASTP11.EXE - play 16bit stereo data unsigned (that does only work on a'
                        db 13,10,'SB16/SB16ASP)'
                        db 13,10,'Pause playing with key "p" and continue it then with any key.'
                        db 13,10,'Stop playing with <ESC>.',10,'$'
    memerror            db 13,10,'Not enough memory to creat the DMA buffer','$'
    txtpart0            db 13,'playing part 0','$'
    txtpart1            db 13,'playing part 1','$'
    sberror             db 13,10,'No SoundBlaster at this BASEADDR ! PROGRAM HALTED.','$'

    OLDInterruptSEG     dw ?
    OLDInterruptOFS     dw ?

    ; OFFSET AND PAGE FOR DMAC
    DMAbufferOFS        dw ?
    DMAbufferPage       db ?

    ; OFFSET AND SEGMENT FOR CPU ACCESS :)
    DMABufferDOSOFS     dw ?
    DMABufferDOSSEG     dw ?

    ; POSITION IN SAMPLEBUFFER WHILE CONVERTING
    position            dw 0

    SAMPLEBUFFERLENGTH = offset SAMPLEBUFFEREND - offset SAMPLEBUFFER
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
.CODE
 STARTUP

           ; FIRST FREE NOT USED MEMORY :
           mov     bx,ss
           mov     ax,es
           sub     bx,ax

           mov     ax,sp
           add     ax,15

           shr     ax,4

           add     bx,ax
           mov     ah,04ah
           int     21h

           ; NOW ALLOCATE DMABUFFER
           mov     bx,DMABUFFERSIZE*2/16       ; count of 16byte blocks for two buffers
           mov     ah,48h
           int     21h

           jnc     enoughmem
           mov     dx,offset memerror
           mov     ah,9
           int     21h       ; WRITE MSG 2 SCRN THAT THERE'S NOT ENOUGH MEM
           jmp     return2dos
enoughmem: ; AX = segment of DMA buffer / offset = 0

;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; calculate page and offset for 16bit DMAcontroller :
;
; segment*16+offset=20bit memory location-> upper 3 bits *2 = page
;                                           next 16 bits = offset
;                                           last 1 bit - lost in space :)
;                                           (because of word access)
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
           rol     ax,4
           mov     bl,al
           and     bl,00eh
           mov     [DMAbufferPage],bl
           and     al,0f1h
           ror     ax,1
           mov     [DMABufferOFS],ax
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; check for DMApage override :
; ... problem: DMA controller separates memory into 64KB pages, you can only
; transfer data is placed in one page - no page overrides are allowed
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; To solve that :
; creat a DMA buffer with double size you want - if the first part is placed
; on a page border the second part is for sure not
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
           mov     cx,DMABUFFERSIZE/2 ; we check for 128KB pages and DMABUFFERSIZE
                                      ; in WORDs
           neg     ax          ; ax = 65536 - ax   (bytes left to DMA page border)
           cmp     ax,cx
           ja      nooverride

           ; USE SECOND PART :
           neg     ax               ; ax = offset first data
           add     ax,cx            ; use second part
           mov     [DMABufferOFS],ax
           add     [DMABufferPage],2 ; 2nd part is on next page !
nooverride:

;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; now fill the  whole buffer with first words of data
; (2 times CALL CONVERT_HALF)
;
; but first - calculate the DOS SEG/OFS from the DMAPage/OFS (you know
; maybe we have to use second buffer half we don't know about ofs/seg yet)
           mov     al,byte ptr [DMABufferOFS]
           and     al,07h
           xor     ah,ah
           shl     ax,1
           mov     di,ax       ; di = offset of DMAbuffer
           mov     ax,[DMABufferOFS]
           and     al,0f8h
           mov     bl,[DMABufferPage]
           shr     bl,1
           or      al,bl
           ror     ax,3
           mov     es,ax       ; es = segment of DMABuffer
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; save these values for later CONVERT_HALF calls
;
           mov     [DMABufferDOSOFS],di
           mov     [DMABufferDOSSEG],ax
           xor     ax,ax

           ; DS:SI - samples in dataseg
           ; ES:DI - DMABuffer

           ; fill the whole buffer with sample data
           CALL    CONVERT_HALF
           CALL    CONVERT_HALF

           ; NOW WE'RE READY FOR SB STUFF:

           RESET_DSP

           ; WRITE INFOMRATION TO SCREEN :
           mov     dx,offset information
           mov     ah,9
           int     21h                  ; write program information to screen

           ; ENABLE SB SPEAKERS (for all SBs <SB16)
           mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
           WAITWRITE
           mov     al,0D1h                     ; AL = Enable speaker
           out     dx,al                       ; Output: DSP Write Data or Command

           ; SETUP IRQ :
           xor     ax,ax
           mov     es,ax                       ; es to page 0 (Interrupt table)
           mov     si,IRQ7*4                   ; si = position in interrupt table

           ; DISABLE IRQ (if it was enabled somehow)
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

           mov     cx,DMABUFFERsize/2-1       ; count of words we need :)
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; Setup DMA-controller :
;
; 1st  MASK DMA CHANNEL
;
           mov     al,DMAchannel-4             ; channels 0..3 for the 2nd DMAC
           add     al,4
           out     WRITEMASK,al
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; 2nd  CLEAR FLIPFLOP
;
           out     CLEARFLIPFLOP,al
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; 3rd  WRITE TRANSFER MODE
;
           mov     al,WANTEDMODE
           add     al,DMAchannel-4
           out     WRITEMODE,al
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; 4th  WRITE PAGE NUMBER
;
           mov     al,[DMAbufferPage]
           out     PAGE16_CHN,al
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; 5th  WRITE BASEADDRESS
;
           mov     ax,[DMABufferOFS]
           out     BASE16_CHN,al
           mov     al,ah
           out     BASE16_CHN,al
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; 6th  WRITE BUFFERLENGTH (in words)-1
;
           mov     al,cl
           out     COUNT16_CHN,al
           mov     al,ch
           out     COUNT16_CHN,al
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; 7th  DEMASK CHANNEL
;
           mov     al,DMAchannel-4
           out     WRITEMASK,al

;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; Setup SoundBlaster :
;
; 1st  SET SAMPLERATE
;
           mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
           WAITWRITE
           mov     al,041h                     ;AL = Set DAC Samplerate
           out     dx,al
           WAITWRITE
           mov     cx,RATE
           mov     al,ch
           out     dx,al
           WAITWRITE
           mov     al,cl
           out     dx,al
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; 2nd  USE 16bit STEREO UNSIGNED MODE (DSPC B6h 20h)
;
           WAITWRITE
           mov     al,0B6h                     ;AL = DMA DAC 16bit autoinit
           out     dx,al
           WAITWRITE
           mov     al,020h                     ;AL = stereo unsigned data
           out     dx,al
           mov     cx,DMABUFFERSIZE/4-1        ;half 16bit buffer
           WAITWRITE
           mov     al,cl                       ;AL = LOWER PART SAMPLELENGTH
           out     dx,al
           WAITWRITE
           mov     al,ch                       ;AL = HIGHER PART SAMPLELENGTH
           out     dx,al

; TRANSFER STARTs.....NOW.... :)
waitloop:  mov     ah,01                       ;AH = Check for character function
           int     016h                        ;   Interrupt: Keyboard
           jz      waitloop                    ; wait for a key (sound in background)

           xor     ah,ah                       ;Read character, flush keypress
           int     016h                        ;   Interrupt: Keyboard
           cmp     al,'p'                      ; check for pause key
           je      pause                       ; ok
           cmp     al,27
           jne     waitloop
           jmp     exit
pause:     ; NOW PAUSE PLAYING (on DSPv4.04 you can also use d0h,d4h)
           mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
           WAITWRITE
           mov     al,0D5h
           out     dx,al

           ; WAIT FOR ANY KEY
           xor     ah,ah                       ;Read character, flush keypress
           int     016h                        ;   Interrupt: Keyboard

           ; CONTINUE PLAYING
           mov     dx,BASEADDR+00Ch            ;DX = DSP Write Data or Command
           WAITWRITE
           mov     al,0D6h
           out     dx,al

           jmp     waitloop

exit:      RESET_DSP

           ; RESTORE PIC MASK
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
           mov     ah,01
           int     16h
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

;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; Our own IRQ for detecting buffer half SB currently plays
; It's generated by the SoundBlaster hardware
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ

OWN_IRQ:   pusha
           mov     dx,BASEADDR+00Fh            ;DX = IRQ ACKNOWLEDGE 16Bit
           in      al,dx
           mov     ax,@data
           mov     ds,ax
           mov     dx,offset txtpart0
           cmp     [part],0
           je      notpart1
           mov     dx,offset txtpart1
notpart1:  mov     ah,9
           int     21h             ; text output
           call    CONVERT_HALF    ; fill next half...
           mov     al,020h
           out     020h,al                     ;ACKNOWLEDGE HARDWARE INTERRUPT
           popa
           IRET

;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
; Convert_half is for copying 8bit data from dataseg to dmabuffer with 16bit
; values (16bitvalue= 8bit value*256)
; one call - convert one buffer half
; next call - convert the other buffer half
; ... 2B Continued ...
; Note: no problem with stereo - it's stored like this :
; 1st byte for left,2nd for right,left,right .... etc.
; That's why we can forget about this while converting
;ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ
CONVERT_HALF:
           mov     cx,DMABUFFERSIZE/2            ; half buffer size in bytes
           mov     di,[DMABufferDOSOFS]
           cmp     [part],0
           je      not2nd
           add     di,cx
not2nd:    shr     cx,1                        ; count of words in half buffer
           mov     ax,[DMABufferDOSSEG]
           mov     es,ax
           mov     si,offset samplebuffer
           add     si,[position]
           xor     al,al
cloop:     mov     ah,ds:[si]
           stosw
           inc     si
           cmp     si,offset samplebuffer + samplebufferlength-2
           ja      samplerestart
           loop    cloop
           jmp     afterloop
samplerestart:
           ; restart sample
           mov     si,offset samplebuffer
           loop    cloop
afterloop:
           sub     si,offset samplebuffer
           mov     [position],si
           neg     [part]
           inc     [part]      ; part = 1-part  result: 0,1,0,1,0,1,...
           RET

END     __start
