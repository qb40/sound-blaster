model large,pascal

.DATA
noeffect  EQU    dw offset checkonlyPara  ; no effect
noeffect2 EQU    dw offset handlenothing  ; nothing to do for this effect here ...

INCLUDE GENERAL.DEF
EXTRN ORDER      : BYTE
EXTRN CHANNEL    : TCHANNEL
EXTRN PATTERN    : WORD
EXTRN INSTRUMENTs : WORD
EXTRN usedchannels : BYTE
EXTRN curline      : BYTE
EXTRN curOrder     : BYTE
EXTRN curPattern   : BYTE
EXTRN curtick      : BYTE
EXTRN curspeed     : BYTE
EXTRN Userate   : WORD
EXTRN loopS3M      : BYTE
EXTRN lastorder    : BYTE
EXTRN EndOfSong    : BYTE
EXTRN gvolume      : BYTE
EXTRN amigalimits  : BYTE
EXTRN sinuswave    : BYTE
EXTRN rampwave     : BYTE
EXTRN Squarewave   : BYTE
EXTRN Insnum       : WORD
EXTRN patterndelay : BYTE
EXTRN Ploop_on     : BYTE
EXTRN Ploop_no     : BYTE
EXTRN Ploop_to     : BYTE
EXTRN patEMShandle : WORD
EXTRN FrameSeg     : WORD
EXTRN patlength    : WORD

      wavetab      DW offset sinuswave
                   DW offset rampwave
                   DW offset squarewave   ; looks not like a square but anyway
                   ; 'random wave' is not a table, but a call for a random number !

      st3periods   dw 1712,1616,1524,1440,1356,1280,1208,1140,1076,1016, 960, 907

      ; we have to init all the effects :
      initeffects  noeffect
                   dw offset setspeed       ; effect 'A'  ok !
                   dw offset jump2order     ; effect 'B'  ok !
                   dw offset patternbreak2  ; effect 'C'  ok !
                   dw offset VolumeEfcts    ; effect 'D'  ok !
                   dw offset Pitchdowns     ; effect 'E'  ok !
                   dw offset Pitchups       ; effect 'F'  ok !
                   dw offset Portamento     ; effect 'G'  ok !
                   dw offset vibrato        ; effect 'H'  ok !
                   dw offset Tremor         ; effect 'I'   <- shity can't get it :( ... ugly implementation
                   dw offset Arpeggio       ; effect 'J'  ok !
                   dw offset Vib_Vol        ; effect 'K'  ok !
                   dw offset Port_Vol       ; effect 'L'  ok !
                   noeffect
                   noeffect
                   noeffect
                   noeffect
                   dw offset Retrigg        ; effect 'Q'  ok !
                   dw offset Tremolo        ; effect 'R'  ok !
                   dw offset Specialsets    ; effect 'S'  look at special 1
                   dw offset setTempo       ; effect 'T'  ok !
                   dw offset vibrato        ; effect 'U'  ok ! (here equal to vibrato)
                   dw offset globalvolume   ; effect 'V'  ok !

      ; Ok now some tables for multichoise effects (e.g. 'Syx' but also 'Dxy',E,F)
      ; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      special1     noeffect                   ; S0? - nothin
                   noeffect                   ; set filter -> not implemented (by ST3)
                   noeffect                   ; set glissando -> not implemented (by ST3)
                   noeffect                   ; set finetune -> not here ! look special2
                   dw offset setvibwav     ;OK !
                   dw offset settremwav    ;OK !
                   noeffect                   ; does not exist
                   noeffect                   ; does not exist
                   noeffect                   ; maybe later (it's E8x - panning )
                   noeffect                   ; does not exist
                   noeffect                   ; stereo control not implemented
                   dw offset cmdPatloop    ;
                   dw offset InitNotecut   ;
                   dw offset InitNotedelay ;
                   dw offset InitPatdelay  ;
                   noeffect                   ; funkrepeat -> not implemented

      retrig_dif   dw offset nosld            ; all done ...
                   dw offset slddown
                   dw offset slddown
                   dw offset slddown
                   dw offset slddown
                   dw offset slddown
                   dw offset use2div3
                   dw offset use1div2
                   dw offset nosld
                   dw offset sldup
                   dw offset sldup
                   dw offset sldup
                   dw offset sldup
                   dw offset sldup
                   dw offset use3div2
                   dw offset use2times

      ; and some effects we have to handle after reading vol/inst/note
      ; maybe refresh etc. - remember MODplayer ...
      ; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      handleeffects  noeffect2              ; really no effect ;)
                     noeffect2
                     noeffect2
                     noeffect2
                     dw offset Hdl_Volfx    ; effect 'D'  ok!
                     dw offset Hdl_pitchdwn ; effect 'E'  ok!
                     dw offset Hdl_pitchup  ; effect 'F'  ok!
                     dw offset Hdl_porta    ; effect 'G'  ok!
                     dw offset Hdl_Vibrato  ; effect 'H'  ok!
                     noeffect2
                     dw offset Hdl_arpeggio ; effect 'J'  ok!
                     dw offset hdl_Vib_Vol  ; effect 'K'  ok!
                     dw offset Hdl_Port_Vol ; effect 'L'  ok!
                     noeffect2
                     noeffect2
                     dw offset Hdl_setsmpofs ; effect 'O'  ok!
                     noeffect2
                     noeffect2
                     dw offset Hdl_tremolo  ; effect 'R'  ok!
                     dw offset Hdl_Special  ; effect 'S'  look at special 2
                     noeffect2
                     dw offset Hdl_Vibrato  ; effect 'U'  ok! (here equal to vibrato
                     noeffect2

      ; Ok now some tables for multichoise effects (e.g. 'Syx' but also 'Dxy',E,F)
      ; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      Vol_cmd2nd     noeffect2
                     noeffect2
                     dw offset FineVSlideDwn
                     dw offset FineVSlideUp
      Pitdwn_cmd2nd  noeffect2
                     dw offset Finepitch_down
                     dw offset XFinepitch_down
      Pitup_cmd2nd   noeffect2
                     dw offset Finepitch_up
                     dw offset XFinepitch_up

      special2     noeffect2                  ; S0? - nothin
                   noeffect2                  ; set filter -> not implemented (by ST3)
                   noeffect2                  ; set glissando -> not implemented (by ST3)
                   dw offset Hdl_finetune
                   noeffect2                  ; not here -> special 1
                   noeffect2                  ; not here -> special 1
                   noeffect2                  ; does not exist
                   noeffect2                  ; does not exist
                   noeffect2                  ; not here -> special 1
                   noeffect2                  ; does not exist
                   noeffect2                  ; stereo control -> not implemented (by ST3)
                   noeffect2                  ; not here -> special 1
                   noeffect2                  ; not here -> special 1
                   noeffect2                  ; not here -> special 1
                   dw offset Hdl_patterndly
                   noeffect2                  ; funkrepeat -> not implemented

      chnCounter   db ?
      jump2flag    db ?
      jump2where   db ?
      breakflag    db ?
      break2where  db ?
      pLoop_now    db ?
      gvolFlag     db ?
      newgvol      db ?
      ; to save portamento values :
      portaFlag    db ?
      Period_old   dw ?
      sStep_old    dd ?
      ; a little one for arpeggio :
      arp_chg      db ?
      ; save effect,parameter for patterndelay
      sav_cmd      dw ?
      sav_para     db ?
      ; now some variables I added after I found out those amazing things
      ; about patterndelay
      inpatterndly db ?   ;<- call 'readnotes' inside a patterndelay, if then ignore all
                          ;notes/inst/vol !
      curnote      db ?   ;<- normaly it will be a copie of es:[di], but in
                          ; patterndelay =0 <- ignore note =)
      curinst      db ?   ; the same thing for instrument
      curVol       db ?   ; and for volume

ENDS

.CODE
.386

PUBLIC READNEWNOTES
PUBLIC SetupNewInst
PUBLIC SetNewNote
EXTRN  SET_TEMPO
EXTRN  EMSMAP : FAR       ; <- somehow that does not work (assembler creats:
                          ;    "push cs" + "near call", but that's wrong !!!

CalcFrequStep  MACRO
; IN: ax = period
; OUT: destroys EDX,EBX

             and     eax,0ffffh      ; clear upper 16bit
             xor     edx,edx
             mov     dx,[Userate]
             mul     edx             ; EAX = Userate*Period
             mov     ebx,eax

             xor     edx,edx
             mov     dx,0dah
             mov     eax,77900000h   ; EDX:EAX = 1712*8363*10000h

             div     ebx

             ;        1712 * 8363 * 10000h
             ; EAX = ----------------------
             ;        Userate * Period
ENDM

CalcPeriod   macro
local notbelow,notabove
; IN:  AL - Note (higher 4=name,lower 4=octave)
;      FS - Segment of instrument
; OUT: AX = Period
; DESTROYs: BX,CL
             ; calc speriod (with st3 finetune) :
             mov     bl,al
             mov     cl,al
             shr     cl,4
             xor     bh,bh
             and     bl,0fh
             shl     bl,1
             ; ok calculate the period
             mov     ax,[st3periods+bx]
             shl     ax,4                 ; period(note)*16
             shr     ax,cl                ; period(note)*16 >> octave(note)
             mov     dx,8363
             mul     dx                   ; dx:ax = 8363*period(note)*16 >> octave(note)
             mov     bx,fs:[TInstrument.c2speed]
             div     bx

             ;       8363*(period(note)*16 shr octave(note))
             ; ax = --------------------------------------
             ;            C2Speed of current Sample

             ; Now check borders :
             cmp     ax,[channel.lower_border+si]
             jnb     notbelow
             mov     ax,[channel.lower_border+si]
notbelow:    cmp     ax,[channel.upper_border+si]
             jna     notabove
             mov     ax,[channel.upper_border+si]
notabove:
endM

CheckPara0 MACRO
local cp
           cmp      al,0
           jnz      cp
           mov      al,[channel.parameter+si]
cp:        mov      [channel.parameter+si],al
ENDM

CheckPara0not MACRO
local cp
              cmp      al,0
              je       cp
              mov      [channel.parameter+si],al
cp:
ENDM

SetupNewInst PROC NEAR
; IN : AL = number of new instrument
;      SI = offset to channel
;      DS = Dataseg
;
; DESTROYs FS,BX,AX,DX
;
             dec     al
             mov     bl,al
             xor     bh,bh
             shl     bx,2
             xor     ah,ah
             add     bx,ax
             mov     ax,ds:[Instruments+2]
             add     bx,ax
             mov     fs,bx
             mov     [channel.instrSEG+si],bx
             mov     al,fs:[TInstrument.vol]
             cmp     al,64
             jb      volwell
             mov     al,63
volwell:     mul     [gvolume]
             shr     ax,6
             mov     [channel.SampleVol+si],al
             mov     ax,fs:[TInstrument.memseg]
             mov     [channel.sampleSEG+si],ax
             mov     al,fs:[TInstrument.flags]
             and     al,00000001b       ; bit 0 - loop sample ?
             mov     [channel.sLoopflag+si],al
             mov     ax,fs:[TInstrument.loopbeg]
             mov     [channel.sLoopstart+si],ax
             mov     ax,fs:[TInstrument.loopend]
             cmp     [channel.sLoopflag+si],1
             je      weloop                            ; =:)
             mov     ax,fs:[TInstrument.slength]     ; <- we don't loop :( anyway ...
weLoop:      mov     [channel.sLoopend+si],ax
             ; calc period borders
             mov     bx,fs:[TInstrument.c2speed]
             cmp     bx,0
             jne     c2speedok
             ; c2speed = 0 -> don't play it !! it's wrong !
             mov     [channel.InstrNo+si],0
c2speedok:   mov     [channel.ssmpstart+si],0  ; reset start value
             cmp     [amigalimits],1
             je      takeamigalimits
             ; B-7 :
             mov     ax,907 * 16 / 128   ; period(note)*16 >> 7
             mov     dx,8363
             mul     dx                   ; dx:ax = 8363*period(note)*16 >> octave(note)
             div     bx
             mov     [channel.lower_border+si],ax
             ; C-0 :
             mov     ax,1712 * 16         ; period(note)*16 >> 0
             mov     dx,8363
             mul     dx                   ; dx:ax = 8363*period(note)*16 >> octave(note)
             cmp     bx,3500
             jb      c2below
             div     bx
             mov     [channel.upper_border+si],ax
             jmp     after1
c2below:     mov     bx,3500
             div     bx
             mov     [channel.upper_border+si],ax
             jmp     after1
takeamigalimits:
             ; first C-3 :
             mov     ax,1712 * 16 / 8     ; period(note)*16 >> 3
             mov     dx,8363
             mul     dx                   ; dx:ax = 8363*period(note)*16 >> octave(note)
             div     bx
             mov     [channel.upper_border+si],ax
             ; then B-5 :
             mov     ax,907 * 16 / 32     ; period(note)*16 >> 5
             mov     dx,8363
             mul     dx                   ; dx:ax = 8363*period(note)*16 >> octave(note)
             div     bx
             mov     [channel.lower_border+si],ax
after1:      mov     al,[curInst]

             ret
ENDP

SetNewNote   PROC NEAR
; IN: AL = note & octave
;     SI = offset to channel
;
; DESTROYs: EAX,EBX,EDX,FS,CX
;
             ; clear it first - just to make sure we really set it
             mov     [channel.speriod+si],0
             cmp     [channel.instrNo+si],0
             je      after2                  ; if there's no instrumnet
             ; set pointer to instrument
             push    ax
             mov     ax,[channel.instrSEG+si]
             mov     fs,ax
             pop     ax
             cmp     fs:[TInstrument.Typ],1  ; only calc if instrument does exist
             jne     after2

             calcperiod

             mov     [channel.speriod+si],ax

             ; now step calculations :
             CalcFrequStep

             mov     [channel.sStep+si],EAX

             cmp     [portaFlag],1
             je      after2             ; it's porta, do not restart !
             ; restart instrument
             xor     ebx,ebx
             mov     bx,[channel.sSmpstart+si]
             shl     ebx,16
             mov     [channel.sCurpos+si],ebx
             mov     [channel.enabled+si],1
after2:      ret
ENDP

; put next notes into channels
READNEWNOTES PROC NEAR
             mov     [jump2flag],0
             mov     [breakflag],0
             mov     [gvolFlag],0
             mov     [Ploop_now],0
             mov     [inpatterndly],0
             cmp     [patterndelay],0
             je      nopatdly
             mov     [inpatterndly],1
nopatdly:    mov     al,[usedchannels]
             mov     [chnCounter],al
             ; get next postion in current pattern
             mov     al,[usedchannels]
             mov     ah,al
             shl     al,2
             add     al,ah        ; al = 5*[usedchannels]
             mov     ah,[curline]
             mul     ah
             ; ok that was the offset :
             mov     di,ax
             ; now the segment of current pattern :
             xor     bh,bh
             mov     bl,[curorder]
             mov     bl,[order+bx]
             cmp     bl,254
             jae     nextorder
             shl     bx,1
             mov     ax,[pattern+bx]
             or      ax,ax
             jz      nopatloop
             cmp     ax,0C000h
             jb      noEMSpattern
             push    ax
             push    di
             ; Set page number:
             mov     ah,044h
             xor     bx,bx
             mov     bl,al              ; bx = logical page
             xor     al,al              ; al = physical page
             mov     dx,[patEMShandle]  ; dx = handle
             int     67h
             cmp     ah,0
             je      noemsprob
             mov     dl,0
             div     dl         ; <- cause a "div by 0" because EMSdriver does not work correct
noemsprob:   pop     di
             pop     ax
             ; change offset to part #(ah and 0fh)
             shr     ax,8
             and     al,3fh             ; bit 13-8 = partno.
             xor     ah,ah
             mov     dx,[patlength]
             mul     dx                 ; ax = partno * pattern length
             add     di,ax
             mov     ax,[frameSEG]
noemspattern:mov     es,ax
             ; ES:DI - pointer to current position in current pattern
             xor     si,si               ; extra channel offset (running through the channels)
chnLoop:     cmp     [channel.channeltyp+si],2
             ja      donothing           ; <- for adlib channels
             mov     [portaFlag],0      ; <- set Flag back
             ; ok first do read current note,inst,vol -> if in patterndelay then ignore them !
             mov     [curNote],0ffh
             mov     [curInst],00h
             mov     [curVol] ,0ffh
             cmp     [inpatterndly],1
             je      ignorethem
             mov     al,es:[di]
             mov     [curNote],al
             mov     al,es:[di+1]
             mov     [curInst],al
             mov     al,es:[di+2]
             mov     [curVol],al
ignorethem:  ; read effects - it may change the read instr/note !
             ; ~~~~~~~~~~~~
             mov     al,es:[di+3]        ; read effect number
             xor     ah,ah
             shl     ax,1
             mov     [channel.continueEf+si],0
             cmp     ax,2*8              ; Vibrato ...
             je      checkifcontV
             cmp     ax,2*11             ; Vibrato Volefcs
             je      checkifcontV
             cmp     ax,2*21             ; fine Vibrato
             je      checkifcontV
             ; -> no vibrato effect, so check if last row there was one ...
             ; if there was - do refresh sStep
             cmp     [channel.command+si],2*8      ; was vibrato
             je      Vibend
             cmp     [channel.command+si],2*21     ; was fine vibrato
             je      Vibend
             cmp     [channel.command+si],2*11     ; was vibrato+volef
             jne     noVibend
             ; refresh frequency
vibend:      push    ax
             mov     ax,[channel.OldPeriod+si]
             mov     [channel.sPeriod+si],ax
             cmp     ax,0
             je      novibcalc
             CalcFrequStep
             mov     [channel.sStep+si],EAX
novibcalc:   pop     ax
novibend:    cmp     ax,2*18             ; Tremolo ...
             je      checkifcontTrm
             cmp     ax,2*10             ; Arpeggio ...
             je      checkifcontA
             ; - currently no arpeggio, but arpeggio was last row ?
             ; if there was - set sStep = Step0 (refresh frequ)
             cmp     [channel.command+si],2*10
             jne     noarpegend
             ; arpeggio ends :
             push    ax
             mov     eax,[channel.step0+si]
             mov     [channel.sStep+si],eax
             pop     ax
noarpegend:  cmp     ax,2*17               ; Note retrigg
             je      checkifcontRetr
aftercontcheck:
             mov     bx,[channel.command+si]
             mov     [sav_cmd],bx               ; to save it for pattern delay ...
             mov     [channel.command+si],ax
             mov     [channel.cmd2nd+si],0
             cmp     al,44
             jbe     noproblem
             mov     [channel.command+si],0
             xor     ax,ax
noproblem:   mov     bx,ax
             mov     al,es:[di+4]        ; read effect parameter
             jmp     [initeffects+bx]
back2reality:

effectdone:  ; read instrument
             ; ~~~~~~~~~~~~~~~
             ; reinit instrument data but don't restart !
             mov     al,[curInst]
             cmp     al,00
             je      no_newinstr

             ; ------------------ check if instrument does exist
             push    ax
             dec     al
             mov     bl,al
             xor     bh,bh
             shl     bx,2
             xor     ah,ah
             add     bx,ax
             mov     ax,ds:[Instruments+2]
             add     bx,ax
             mov     fs,bx
             cmp     fs:[TInstrument.typ],1
             je      instok
             pop     ax
             xor     al,al
             mov     [curInst],al
             jmp     no_newinstr
instok:      pop     ax

             comment #
             cmp     [channel.enabled+si],0       ; if channel is disabled then restart definitly
             je      restart
             cmp     [channel.InstrNo+si],al      ; but if it's enabled & same instrno then don't restart
             je      dontrestart
restart:     cmp     [portaFlag],1                ; and if portamento then don't restart ;)
             je      dontrestart
             mov     bx,[channel.sSmpstart+si]
             mov     [channel.sCurpos+si],bx
dontrestart:
             #

             mov     [channel.InstrNo+si],al
             call near ptr SetupNewInst
no_newinstr: ; read note ...
             ; ~~~~~~~~~~~~~
             mov     al,[curNote]
             cmp     al,0ffh
             je      no_newnote
             cmp     al,0feh
             jne     normal_note
             mov     [channel.enabled+si],0     ; stop mixing
             jmp     no_newnote
normal_note: mov     [channel.Note+si],al
             call near ptr SetNewNote
no_newnote:  ; read volume - last but not least ;)
             ; ~~~~~~~~~~~
             mov     al,[curVol]
             cmp     al,0ffh
             je      no_vol
             cmp     al,64
             jb      volok
             mov     al,63
volok:       mul     [gvolume]
             shr     ax,6
             mov     [channel.SampleVol+si],al

no_vol:      ; ok now the effect handling after reading vol/instr/note
             mov     bx,[channel.command+si]
             jmp     [handleeffects+bx]
handlenothing:

donothing:   add     di,5                ; to next channel in pattern
             add     si,size Channel     ; to next channel in channel mix info
             dec     [chnCounter]        ; one channel done
             jnz     chnLoop
             cmp     [gvolFlag],0
             je      nonewglobal
             mov     al,[newgvol]
             mov     [gvolume],al
nonewglobal: mov     al,[curspeed]
             mov     [curtick],al
             cmp     [breakflag],1
             je      patternbreak
             cmp     [jump2flag],1
             je      jumporder
             cmp     [Ploop_now],1
             je      dopatloop
nopatloop:   inc     [curline]
             cmp     [curline],64
             jne     only_next
             ; ok - next order ...
             mov     [curline],0
nextorder:   inc     [curorder]
otherorder:  mov     [Ploop_to],0
             mov     al,[lastorder]
             cmp     [curorder],al
             ja      end_reached
doloop:      xor     bh,bh
             mov     bl,[curorder]
             mov     al,[Order+bx]
             cmp     al,254
             jae     nextorder
             mov     [curpattern],al
only_next:   ret

patternbreak:mov     al,[break2where]
             mov     [curline],al
             jmp     nextorder

jumporder:   mov     [curline],0
             mov     al,[jump2where]
             mov     [curorder],al
             jmp     otherorder

dopatloop:   dec     [Ploop_no]
             jz      patloopends
             mov     al,[Ploop_to]
             mov     [curline],al
             jmp     only_next
patloopends: mov     al,[curline]
             inc     al
             mov     [PLoop_to],al
             mov     [Ploop_on],0
             jmp     nopatloop

end_reached: cmp     [loopS3M],1
             jne     donotloop
             mov     [curorder],0
             jmp     doloop
donotloop:   mov     [EndOfSong],1
             jmp     only_next

checkifcontV:  ; check if continue Vib/Vib_vol
             cmp     [curNote],0feh
             jb      aftercontcheck
             cmp     [channel.command+si],11*2  ; command before was vibrato
             je      checkok1
             cmp     [channel.command+si],21*2  ; command before was fine vibrato
             je      checkok1
             cmp     [channel.command+si],8*2   ; command before was vibrato+volef
             jne     aftercontcheck
checkok1:    mov     [channel.continueEf+si],1
             jmp     aftercontcheck

checkifcontA:     ; check if continue Arpeg
             cmp     [curNote],0feh
             jb      aftercontcheck
checkifcontTrm:   ; check if continue a tremolo
checkifcontRetr:  ; check if continue a note retrig
             cmp     [channel.command+si],ax
             jne     aftercontcheck
             mov     [channel.continueEf+si],1
             jmp     aftercontcheck

; Effects :
; ~~~~~~~~~

checkonlyPara: checkPara0
               jmp      back2reality

setspeed:      ; effect 'A'
               checkPara0not
               cmp      al,0                ; speed = 0 does not exist
               je       back2reality
               mov      [curspeed],al
               jmp      back2reality
jump2order:    ; effect 'B'
               checkPara0not
               mov      [jump2flag],1
               mov      [jump2where],al
               jmp      back2reality
patternbreak2: ; effect 'C'
               checkPara0not
               mov      [breakflag],1
               mov      ah,al
               and      al,0fh
               shr      ah,4
               AAD
               mov      [break2where],al
               jmp      back2reality
VolumeEfcts:   ; effect 'D' or jump from dual commands ...
               checkPara0
               mov      ah,al
               shr      ah,4
               cmp      ah,0fh
               jne      nofinevoldown
               cmp      al,0f0h
               je       volup
               ; normal fine volume down   DFx
               mov      [channel.cmd2nd + si],04
               jmp      back2reality
nofinevoldown: mov      ah,al
               and      ah,0fh
               cmp      ah,0fh
               jne      nofinevolup
               cmp      al,0fh
               je       voldown
               ; normal fine volume up     DxF
               mov      [channel.cmd2nd + si],06
               jmp      back2reality
nofinevolup:   mov      ah,al
               and      ah,00fh
               jz       notVslidedown
               ; normale slide down effect D0x
voldown:       mov      [channel.cmd2nd + si],00
               jmp      back2reality
notVslidedown: ; normal slide up effect    Dx0
volup:         mov      [channel.cmd2nd + si],02
               jmp      back2reality
Pitchdowns:    ; effect 'E'
               checkPara0
               cmp      al,0DFh      ; al is parameter
               ja       extraptdowns
               mov      [channel.cmd2nd + si],00
               jmp      back2reality
extraptdowns:  cmp      al,0EFh
               ja       Finepitchdown
               ; extra fine slides
               mov      [channel.cmd2nd + si],04
               jmp      back2reality
Finepitchdown: mov      [channel.cmd2nd + si],02
               jmp      back2reality
Pitchups:      ; effect 'F'
               checkPara0
               cmp      al,0DFh      ; al is parameter
               ja       extraptups
               mov      [channel.cmd2nd + si],00
               jmp      back2reality
extraptups:    cmp      al,0EFh
               ja       Finepitchup
               ; extra fine slides
               mov      [channel.cmd2nd + si],04
               jmp      back2reality
Finepitchup:   mov      [channel.cmd2nd + si],02
               jmp      back2reality
Portamento:    ; effect 'G'
               checkPara0not
               cmp      al,0
               je       nonewPortpara
               mov      [channel.PortPara+si],al
nonewPortpara: mov      [portaFlag],1
               ; check first if portamento really possible:
               cmp      [channel.enabled+si],0
               je       stopporta
               cmp      [curNote],0feh
               jae      back2reality    ; <- continue portamento
               ; now save some values (we wanna slide from ...)
               mov      eax,[channel.sStep+si]
               mov      [sStep_old],eax
               mov      ax,[channel.sPeriod+si]
               mov      [Period_old],ax
               jmp      back2reality
stopporta:     mov      [channel.command+si],0   ;<-noeffect
               mov      [portaFlag],0
               jmp      back2reality
Vibrato:       ; effect 'H'
               checkpara0not
               cmp      [channel.continueEf+si],1
               je       norestart
               mov      [channel.Tablepos+si],0
norestart:     cmp      al,0
               jne      newVibvalue
               mov      al,[channel.VibPara+si]
newVibValue:   mov      ah,al
               shr      ah,4
               cmp      ah,0
               jne      Vibspeedok
               mov      ah,[channel.VibPara+si]
               and      ah,0f0h
               or       al,ah
VibspeedOk:    mov      [channel.VibPara+si],al
               jmp      back2reality
Tremor:        ; effect 'I'
               checkPara0
               jmp      back2reality
Arpeggio:      ; effect 'J'
               cmp      al,0
               je       uselastPara
               mov      [arp_chg],1
               mov      [channel.parameter+si],al
               jmp      back2reality
uselastPara:   mov      [arp_chg],0
               jmp      back2reality
Vib_Vol:       ; effect 'K'
               cmp      [channel.continueEf+si],1
               je       volumeefcts
               mov      [channel.Tablepos+si],0
               jmp      volumeefcts
Port_Vol:      ; effect 'L'
               mov      [portaFlag],1
               ; check first if portamento really possible:
               cmp      [channel.enabled+si],0
               je       stopporta             ; <- channel plays nothing -> no porta and volfx usefull ;)
               cmp      [curNote],0feh
               jae      volumeefcts           ; <- continue portamento
               ; now save some values (we wanna slide from ...)
               push     ax
               mov      eax,[channel.sStep+si]
               mov      [sStep_old],eax
               mov      ax,[channel.sPeriod+si]
               mov      [Period_old],ax
               pop      ax
               jmp      volumeefcts    ; <- and now volume effects

Retrigg:       ; effect 'Q'
               cmp      al,0
               jne      newretrPara
               mov      al,[channel.parameter+si]
               jmp      aftersetretr
newretrPara:   mov      [channel.parameter+si],al
               mov      ah,al
               and      ah,0fh
               cmp      ah,0
               je       noretrigg
               dec      ah
               mov      [channel.ctick+si],ah
aftersetretr:  mov      bl,al
               shr      bl,4
               shl      bl,1
               xor      bh,bh
               jmp      [retrig_dif+bx]
nosld:         mov      [channel.cmd2nd+si],0
               jmp      back2reality
slddown:       mov      [channel.cmd2nd+si],2
               jmp      back2reality
use2div3:      mov      [channel.cmd2nd+si],4
               jmp      back2reality
use1div2:      mov      [channel.cmd2nd+si],6
               jmp      back2reality
sldup:         mov      [channel.cmd2nd+si],8
               jmp      back2reality
use3div2:      mov      [channel.cmd2nd+si],10
               jmp      back2reality
use2times:     mov      [channel.cmd2nd+si],12
               jmp      back2reality
noretrigg:     mov      [channel.command+si],0
               jmp      back2reality
Tremolo:       ; effect 'R'
               cmp      [channel.continueEf+si],1
               je       noTrmrestart
               mov      [channel.Tablepos+si],0
noTrmrestart:  cmp      al,0
               jne      newTrmvalue
               mov      al,[channel.Parameter+si]
newTrmValue:   mov      ah,al
               shr      ah,4
               cmp      ah,0
               jne      Trmspeedok
               mov      ah,[channel.Parameter+si]
               and      ah,0f0h
               or       al,ah
TrmspeedOk:    mov      [channel.Parameter+si],al
               jmp      back2reality               
Specialsets:   ; effect 'S'
               checkPara0
               xor      bh,bh
               mov      bl,al
               shr      bl,4
               shl      bx,1
               mov      [channel.cmd2nd+si],bx
               ; do effects we have to check before reading note/vol/instr
               jmp      [special1+bx]
setvibwav:     ; for every channel a seperate choise !
               and      al,3
               add      al,al
               xor      ah,ah
               mov      bx,ax
               shl      bx,1
               mov      ax,[wavetab+bx]
               mov      [channel.VibTabOfs+si],ax
               jmp      back2reality
settremwav:    ; for every channel a seperate choise !
               and      al,3
               add      al,al
               xor      ah,ah
               mov      bx,ax
               shl      bx,1
               mov      ax,[wavetab+bx]
               mov      [channel.TrmTabOfs+si],ax
               jmp      back2reality
cmdPatloop:    and      al,0fh
               cmp      al,0
               je       set2where
               cmp      [Ploop_on],1
               je       alreadyinside
               mov      [Ploop_on],1
               inc      al
               mov      [Ploop_no],al
alreadyinside: mov      [Ploop_now],1
               jmp      back2reality
set2where:     mov      al,[curline]
               mov      [Ploop_to],al
               jmp      back2reality
InitNotecut:   and      al,0fh
               cmp      al,0
               je       noCut
               ;inc      al
               mov      [channel.ndtick+si],al
               jmp      back2reality
noCut:         mov      [channel.command+si],9
               jmp      back2reality
InitNotedelay: and      al,0fh
               mov      [channel.ndTick+si],al
               cmp      [inpatterndly],1
               je       donothing
               mov      al,[curNote]      ; new note for later
               mov      [channel.savnote+si],al
               mov      al,[curInst]    ; new instrument for later
               mov      [channel.savInst+si],al
               mov      al,[curVol]    ; new volume for later
               mov      [channel.savVol+si],al
               jmp      donothing       ; setup note/instr/vol later while mixing !
InitPatdelay:  cmp      [inpatterndly],1
               je       back2reality            ; <- hehe we are allread in this patterndelay ...
               and      al,0fh
               inc      al
               mov      [patterndelay],al
               mov      al,[channel.parameter+si]
               mov      [sav_para],al
               jmp      back2reality
setTempo:      ; effect 'T'
               checkPara0not
               xor      ah,ah
               push     si di fs es
               push     ax
               call far ptr SET_TEMPO
               pop      es fs di si
               jmp      back2reality

globalvolume:  ; effect 'V'
               checkPara0not
               mov      [gvolFlag],1
               cmp      al,64
               jbe      gvolok
               mov      al,64
gvolok:        mov      [newgvol],al
               jmp      back2reality

;  _____________________________________
;  Effect after reading vol/instr/note :
;  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Hdl_Volfx:    ; effect 'D'
              mov       bx,[channel.cmd2nd + si]
              jmp       [vol_cmd2nd+bx]
FineVSlideDwn:mov       al,[channel.Parameter+si]
              and       al,0fh
              sub       [channel.SampleVol+si],al
              jnc       handlenothing
              mov       [channel.SampleVol+si],0
              jmp       handlenothing
FineVSlideUp: mov       al,[channel.Parameter+si]
              shr       al,4
              add       [channel.SampleVol+si],al
              cmp       [channel.SampleVol+si],64
              jb        handlenothing
              mov       [channel.SampleVol+si],63
              jmp       handlenothing
Hdl_pitchdwn: ; effect 'E'
              mov       bx,[channel.cmd2nd + si]
              jmp       [pitdwn_cmd2nd+bx]
Finepitch_down:
              ; we pitch down, but increase period ! (so check upper_border)
              mov       ax,[channel.sPeriod+si]
              mov       bl,[channel.Parameter+si]
              and       bl,0fh
              xor       bh,bh
              shl       bx,2
              add       ax,bx
              cmp       ax,[channel.upper_border+si]
              jb        ptok
              mov       ax,[channel.upper_border+si]
ptok:         ; now calc new frequency step for this period
              mov       [channel.sPeriod+si],ax
              CalcFrequStep
              mov       ds:[channel.sStep+si],EAX
              jmp       handlenothing
XFinepitch_down:
              ; we pitch down, but increase period ! (so check upper_border)
              mov       ax,[channel.sPeriod+si]
              mov       bl,[channel.Parameter+si]
              and       bl,0fh
              xor       bh,bh
              add       ax,bx
              cmp       ax,[channel.upper_border+si]
              jb        ptok
              mov       ax,[channel.upper_border+si]
              jmp       ptok
Hdl_pitchup:  ; effect 'F'
              mov       bx,[channel.cmd2nd + si]
              jmp       [pitup_cmd2nd+bx]
Finepitch_up: ; we pitch up, but decrease period ! (so check lower_border)
              mov       ax,[channel.sPeriod+si]
              mov       bl,[channel.Parameter+si]
              and       bl,0fh
              xor       bh,bh
              shl       bx,2
              sub       ax,bx
              cmp       ax,[channel.lower_border+si]
              ja        ptok
              mov       ax,[channel.lower_border+si]
              jmp       ptok
XFinepitch_up:; we pitch up, but decrease period ! (so check lower_border)
              mov       ax,[channel.sPeriod+si]
              mov       bl,[channel.Parameter+si]
              and       bl,0fh
              xor       bh,bh
              sub       ax,bx
              cmp       ax,[channel.lower_border+si]
              ja        ptok
              mov       ax,[channel.lower_border+si]
              jmp       ptok
Hdl_porta:     ; effect 'G'
               cmp      [curNote],0feh
               jae      handlenothing         ; <- is a portamento continue
               mov      ax,[Period_Old]
               xchg     ax,[channel.sPeriod+si]
               mov      [channel.wantedPeri+si],ax
               mov      eax,[sStep_old]
               mov      [channel.sStep+si],eax
               jmp      handlenothing
Hdl_Vibrato:   ; effect 'H'
               cmp      [channel.continueEf+si],1
               je       handlenothing                ; continue this effect !
               mov      ax,[channel.sPeriod+si]
               mov      [channel.OldPeriod+si],ax
               jmp      handlenothing
Hdl_arpeggio:  ; effect 'J'
               cmp      [arp_chg],1
               je       newPara
               cmp      [channel.continueEf+si],1
               je       handlenothing                ; no new note !
               ; start arpeggio:
               mov      [channel.ArpegPos+si],0
newPara:       mov      al,[channel.parameter+si]
               mov      ah,[channel.note+si]
               ; calc note 1:
               mov      bl,ah
               mov      bh,al
               shr      bh,4            ; upper 4 bits for note 1
               add      bl,bh
               mov      bh,bl
               and      bh,0f0h
               and      bl,00fh
arpt1:         cmp      bl,12
               jb       arpok1
               sub      bl,12
               add      bh,10h
               jmp      arpt1
arpok1:        or       bh,bl
               mov      [channel.note1+si],bh
               ; now note 2:
               mov      bl,ah
               mov      bh,al
               and      bh,0fh          ; lower 4 bits for note 2
               add      bl,bh
               mov      bh,bl
               and      bh,0f0h
               and      bl,00fh
arpt2:         cmp      bl,12
               jb       arpok2
               sub      bl,12
               add      bh,10h
               jmp      arpt2
arpok2:        or       bh,bl
               mov      [channel.note2+si],bh
               ; now calc the Steps:
               mov     ax,[channel.instrSEG+si]
               mov     fs,ax
               mov      al,[channel.Note+si]
               call near ptr calcpart
               mov      [channel.Step0+si],eax
               mov      al,[channel.Note1+si]
               call near ptr calcpart
               mov      [channel.Step1+si],eax
               mov      al,[channel.Note2+si]
               call near ptr calcpart
               mov      [channel.Step2+si],eax
               jmp      handlenothing

calcpart:      calcperiod
               cmp       ax,0
               je        nostep
               calcfrequStep
               retn
nostep:        xor       eax,eax
               retn

Hdl_Vib_Vol:   cmp      [channel.continueEf+si],1
               je       Hdl_Volfx
               mov      ax,[channel.sPeriod+si]
               mov      [channel.OldPeriod+si],ax
               jmp      Hdl_Volfx

Hdl_Port_Vol:  ; effect 'L'
               ; first handle Portamento (and then jump to volume effect handling)
               cmp      [curNote],0feh
               jae      Hdl_Volfx
               mov      ax,[Period_Old]
               xchg     ax,[channel.sPeriod+si]
               mov      [channel.wantedPeri+si],ax
               mov      eax,[sStep_old]
               mov      [channel.sStep+si],eax
               jmp       Hdl_Volfx
Hdl_setsmpofs:; effect 'O'
               xor       eax,eax
               mov       ah,[channel.parameter+si]
               mov       [channel.ssmpstart+si],ax
               shl       eax,16
               cmp       [curnote],0ffh
               je        handlenothing
               mov       [channel.sCurPos+si],eax
               jmp       handlenothing
Hdl_tremolo:  ; effect 'R' (Tremolo)
               cmp      [curInst],0
               jne      savevol                      ; new instrument
               cmp      [curVol],0ffh
               jne      savevol                      ; new volume
               cmp      [channel.continueEf+si],1
               je       handlenothing                ; continue this effect !
              ; save volume
savevol:      mov       al,[channel.SampleVol+si]
              mov       [channel.OldVolume+si],al
              jmp       handlenothing
Hdl_Special:  ; effect 'S'
              mov       bx,[channel.cmd2nd+si]
              jmp       [special2+bx]
Hdl_finetune: jmp       handlenothing
Hdl_patterndly: mov       al,[sav_para]
                mov       [channel.parameter+si],al
                mov       ax,[sav_cmd]
                mov       [channel.command+si],ax
                mov       [channel.cmd2nd+si],0
                jmp       handlenothing

READNEWNOTES ENDP

ENDS

END