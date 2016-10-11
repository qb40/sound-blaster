;==============================================================================
; memory to memory transfer
;   Andr‚ Baresel
;------------------------------------------------------------------------------
; Requirements: 80286, DOS 1.0,
;

    .MODEL small
    .286

    ; DMA controller registers :
    WRITEMASK          EQU 00ah         ;WRITE MASK REGISTER
    WRITEMODE          EQU 00bh         ;WRITE MODE REGISTER
    COMMAND            EQU 008h         ;WRITE COMMAND REGISTER
    STATUS             EQU 008h         ;READ STATUS REGISTER
    START              EQU 009h         ;WRITE START REGISTER
    CLEARFLIPFLOP      EQU 00ch

    PAGE_CHN0          EQU 087h         ;PAGE REGISTER FOR DMAchannel 0
    BASE_CHN0          EQU 000h         ;BASEADDRESS REGISTER DMA 0
    COUNT_CHN0         EQU 001h         ;COUNT REGISTER DMAchannel 0

    PAGE_CHN1          EQU 083h         ;PAGE REGISTER FOR DMAchannel 1
    BASE_CHN1          EQU 002h         ;BASEADDRESS REGISTER DMA 1
    COUNT_CHN1         EQU 003h         ;COUNT REGISTER DMAchannel 1

    ; for DMA COMMAND REGISTER
    COMMANDVALUE       EQU 00000001b
    ; for DMA WRITE MODE REGISTER
    MODE_CH0           EQU 00000100b  ; singlemode, nonautoinit, writemode ch0
    MODE_CH1           EQU 00001001b  ; singlemode, nonautoinit, readmode ch1

    .STACK 100h

    .DATA

    origintxt   db 13,10,'Data to copy :',13,10
    origin      db 80 DUP ('#')
                db '$'
    destitxt    db 13,10,'Destination (F stand for not copied right):',13,10
    destination db 80 DUP (01h)
                db '$'

    override    db  'I',39,'m sorry a DMA page override ...','$'
    errorcopy   db  'Hmm, somethings going wrong while copying with DMA ...','$'
    ok          db  1

    .CODE
    .STARTUP

    ;
    ; calculate page and offset for DMAcontroller :
    ;
    ; segment*16+offset - 20bit memory location -> upper 4 bits  = page
    ;                                              lower 16 bits = offset
    ;
    mov     si,offset origin
    mov     di,offset destination
    mov     ax,ds
    rol     ax,4
    mov     bl,al
    and     bl,00fh
    mov     bh,bl
    and     al,0f0h
    add     si,ax        ; offset for origin (channel 0)
    adc     bh,0         ; page for origin (channel 0)
    add     di,ax        ; offset for destination (channel 1)
    adc     bl,0         ; page for destination (channel 1)

    cmp     si,-80
    jna     ok1         ; shit a page override
    jmp     exit
ok1:
    cmp     di,-80
    jna     ok2         ; shit a page override
    jmp     exit
ok2:
    ;
    ; Setup channel 0:
    ;
    ; 1st  mask DMA channel 0
    ;
    mov     al,4
    out     WRITEMASK,al
    ;
    ; 2nd  clear flipflop
    ;
    out     CLEARFLIPFLOP,al
    ;
    ; 3rd  write transfer mode
    ;
    mov     al,MODE_CH0
    out     WRITEMODE,al
    ;
    ; 4th  write PAGE number
    ;
    mov     al,bh
    out     PAGE_CHN0,al
    ;
    ; 5th  write baseaddress
    ;
    mov     ax,si
    out     BASE_CHN0,al
    mov     al,ah
    out     BASE_CHN0,al
    ;
    ; 6th  write samplelength-1
    ;
    mov     cx,80-1
    mov     al,cl
    out     COUNT_CHN0,al
    mov     al,ch
    out     COUNT_CHN0,al
    ;
    ; 7th  demask channel 0
    ;
    mov     al,0
    out     WRITEMASK,al

    ; ------------- channel 0 setup ok -------------------

    ;
    ; Setup channel 1:
    ;
    ; 1st  mask DMA channel 1
    ;
    mov     al,5
    out     WRITEMASK,al
    ;
    ; 2nd  clear flipflop
    ;
    out     CLEARFLIPFLOP,al
    ;
    ; 3rd  write transfer mode
    ;
    mov     al,MODE_CH1
    out     WRITEMODE,al
    ;
    ; 4th  write PAGE number
    ;
    mov     al,bl
    out     PAGE_CHN1,al
    ;
    ; 5th  write baseaddress
    ;
    mov     ax,di
    out     BASE_CHN1,al
    mov     al,ah
    out     BASE_CHN1,al
    ;
    ; 6th  write samplelength-1
    ;
    mov     cx,80-1
    mov     al,cl
    out     COUNT_CHN1,al
    mov     al,ch
    out     COUNT_CHN1,al
    ;
    ; 7th  demask channel 1
    ;
    mov     al,1
    out     WRITEMASK,al

    ; ------------- channel 1 setup ok -------------------

    ; set command :
    mov     al,commandvalue
    out     command,al

    ; start transfer :
    mov     al,00000100b
    out     START,al

waitforterminal:
    in      al,STATUS
    and     al,1
    jz      waitforterminal

    ; compare it :
    mov     cx,80
    mov     si,offset origin
    mov     di,offset destination
cloop:
    lodsb
    cmp     al,ds:[di]
    je      next
    mov     ds:[di],byte ptr 'F'
    mov	    [ok],0
next:
    inc     di
    loop    cloop

    ; let's show the result
    mov     dx,offset origintxt
    mov     ah,9
    int     21h

    mov     dx,offset destitxt
    mov     ah,9
    int     21h

    cmp     [ok],1
    jne     notcopied

exit:
    ; Terminate EXE:
return2dos:
    mov     ax,04c00h
    int     21h

over:  mov     dx,offset override
       mov     ah,9
       int     21h
       jmp     exit

notcopied: mov     dx,offset errorcopy
           mov     ah,9
           int     21h
           jmp     exit

    END
