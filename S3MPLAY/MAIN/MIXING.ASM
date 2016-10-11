model large,pascal

noeffect EQU    dw offset no_effect

.DATA
INCLUDE GENERAL.DEF
EXTRN tickbuffer  :DWORD
EXTRN post8bit    :WORD
EXTRN curtick     :BYTE
EXTRN curspeed    :BYTE
EXTRN curline     :BYTE
EXTRN BPT         :WORD
EXTRN EndOfSong   :BYTE
EXTRN usedchannels:BYTE
EXTRN volumetableptr :DWORD
EXTRN CHANNEL        :TChannel
EXTRN Userate     :WORD
EXTRN gvolume        :BYTE
EXTRN patterndelay   :BYTE
EXTRN smpEMShandle   :WORD
EXTRN frameseg       :WORD
EXTRN DMArealBufsize :WORD
EXTRN TickBytesLeft  :WORD
nextPosition      DW ?
sample2calc       DW ?     ; in mono number of bytes/ in stereo number of words
curchannel        DB ?
calleffects       DB ?

effects            noeffect
                   noeffect
                   noeffect
                   noeffect
                   dw offset VolumeEfcts    ; effect 'D'
                   dw offset Pitchdowns     ; effect 'E'
                   dw offset Pitchups       ; effect 'F'
                   dw offset Portamento     ; effect 'G'
                   dw offset Vibrato        ; effect 'H'
                   dw offset Tremor         ; effect 'I'
                   dw offset Arpeggio       ; effect 'J'
                   dw offset Vib_Vol        ; effect 'K'
                   dw offset Port_Vol       ; effect 'L'
                   noeffect
                   noeffect
                   noeffect
                   noeffect
                   dw offset Retrigg        ; effect 'Q'
                   dw offset Tremolo        ; effect 'R'
                   dw offset Specialsets    ; effect 'S'   ; <- for notecut/delay reasons
                   noeffect
                   dw offset fineVibrato    ; effect 'U'
                   noeffect

vol_cmd2nd         dw offset volslidedown
                   dw offset volslideup
                   noeffect
                   noeffect
pitdwn_cmd2nd      dw offset pitchdown
                   noeffect
                   noeffect
pitup_cmd2nd       dw offset pitchup
                   noeffect
                   noeffect
retrig_cmd2nd      noeffect
                   dw offset slddown
                   dw offset use2div3
                   dw offset use1div2
                   dw offset sldup
                   dw offset use3div2
                   dw offset use2times

      ; and for all special commands :
special_cmd2nd     noeffect                   ; S0? - nothin
                   noeffect                   ; set filter
                   noeffect                   ; set glissando
                   noeffect                   ; set finetune
                   noeffect                   ; set vibrato waveform
                   noeffect                   ; set tremolo waveform
                   noeffect                   ; does not exist
                   noeffect                   ; does not exist
                   noeffect                   ; maybe later (it's E8x - panning )
                   noeffect                   ; does not exist
                   noeffect                   ; stereo control
                   noeffect                   ; Pattern loop things
                   dw offset Notecut
                   dw offset Notedelay
                   noeffect                   ; Pattern delay
                   noeffect                   ; funkrepeat

dwofs macro name, no
      dw offset &name&no
endm

; stereo innerloop table:
zaehler = 0
st_innerloop_tbl LABEL WORD
rept 32
   dwofs st_inner, %zaehler
   zaehler = zaehler + 1
endm

; mono innerloop table:
zaehler = 0
mn_innerloop_tbl LABEL WORD
rept 32
   dwofs mn_inner, %zaehler
   zaehler = zaehler + 1
endm

ENDS

.CODE
.386

PUBLIC calc_stereo_tick
PUBLIC calc_mono_tick
EXTRN  readnewnotes
EXTRN  SetupNewInst
EXTRN  SetNewNote

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

INCLUDE BORDER.INC

INCLUDE STEREO.INC

INCLUDE MONO.INC

; for stereo and mono the same :

no_effect:     retn     ; <- sorry for this :(
VolumeEfcts:   ; effect 'D'
               mov       bx,ds:[channel.cmd2nd+bp]
               jmp       [vol_cmd2nd+bx]
volslidedown:  mov       al,ds:[channel.Parameter+bp]
               and       al,0fh
               sub       ds:[channel.SampleVol+bp],al
               jnc       vlsdwn
               mov       ds:[channel.SampleVol+bp],0
vlsdwn:        retn
volslideup:    mov       al,ds:[channel.Parameter+bp]
               shr       al,4
               add       ds:[channel.SampleVol+bp],al
               cmp       ds:[channel.SampleVol+bp],64
               jb        vlsup
               mov       ds:[channel.SampleVol+bp],63
vlsup:         mov       al,ds:[channel.SampleVol+bp]
               mul       [gvolume]
               shr       ax,6
               mov       ds:[channel.SampleVol+bp],al
               retn
Pitchdowns:    ; effect 'E'
               mov       bx,ds:[channel.cmd2nd+bp]
               jmp       [pitdwn_cmd2nd+bx]
Pitchdown:     ; we pitch down, but increase period ! (so check upper_border)
               mov       ax,ds:[channel.sPeriod+bp]
               mov       bl,ds:[channel.Parameter+bp]
               xor       bh,bh
               shl       bx,2
               add       ax,bx
               cmp       ax,ds:[channel.upper_border+bp]
               jb        calcnewSF
               mov       ax,ds:[channel.upper_border+bp]
calcnewSF:     ; now calc new frequency step for this period
               mov       ds:[channel.sPeriod+bp],ax
               cmp       ax,0
               je        donotcalc
               CalcFrequStep
               mov     ds:[channel.sStep+bp],EAX
donotcalc:     retn
Pitchups:      ; effect 'F'
               mov       bx,ds:[channel.cmd2nd+bp]
               jmp       [pitup_cmd2nd+bx]
Pitchup:       ; we pitch up, but decrease period ! (so check lower_border)
               mov       ax,ds:[channel.sPeriod+bp]
               mov       bl,ds:[channel.Parameter+bp]
               xor       bh,bh
               shl       bx,2
               sub       ax,bx
               cmp       ax,ds:[channel.lower_border+bp]
               ja        calcnewSF
               mov       ax,ds:[channel.lower_border+bp]
               jmp       calcnewSF
Portamento:    ; effect 'G'
               mov       bl,ds:[channel.PortPara+bp]
               xor       bh,bh
               shl       bx,2    ; <- use amiga slide = para*4
               mov       ax,ds:[channel.sPeriod+bp]
               cmp       ax,ds:[channel.wantedPeri+bp]
               jg        porta_down
               add       ax,bx
               cmp       ax,ds:[channel.wantedPeri+bp]
               jle       calcnewSF
               mov       ax,ds:[channel.wantedPeri+bp]
               jmp       calcnewSF
porta_down:    sub       ax,bx
               cmp       ax,ds:[channel.wantedPeri+bp]
               jge       calcnewSF
               mov       ax,ds:[channel.wantedPeri+bp]
               jmp       calcnewSF
Vibrato:       ; effect 'H'
               cmp       ds:[channel.enabled+bp],0
               je        novib
               ; next position in table:
               mov       al,ds:[channel.VibPara+bp]
               mov       dl,al
               and       dl,0fh
               shr       al,4
               mov       bl,ds:[channel.Tablepos+bp]
               add       bl,al

               cmp       bl,64
               jb        endoftest
               sub       bl,64
endoftest:
               mov       ds:[channel.Tablepos+bp],bl
               xor       bh,bh
               add       bx,ds:[channel.VibTabOfs+bp]
               mov       al,ds:[bx]
               imul      dl
               sar       ax,4
               mov       bx,ds:[channel.Oldperiod+bp]
               add       ax,bx
               jmp       calcnewSF
novib:         retn
Tremor:        ; effect 'I'
               retn
Arpeggio:      ; effect 'J'
               mov       bl,ds:[ArpegPos+bp]
               xor       bh,bh
               inc       bx
               cmp       bx,3
               jb        inside         ; Oh I love this song - INSIDE.S3M ;)
               xor       bx,bx
inside:        mov       ds:[ArpegPos+bp],bl
               shl       bx,2
               ; cool way to address Step 0,1,2 :
               add       bp,bx
               mov       eax,ds:[channel.step0+bp]  ; <- don't think we load here _everytime_ step0 !
               sub       bp,bx
               ; now use it :
               mov       ds:[channel.sStep+bp],eax
               retn
Vib_Vol:       ; effect 'K'
               ; first do volslides :
               call near ptr VolumeEfcts  ; oh well I love ASM
                                          ; -> JUMP ARROUND, JUMP JUMP JUMP
               ; and now vibrato:
               jmp       vibrato          ; that's nice ;) no need for more :)
Port_Vol:      ; effect 'L'
               ; first do volslides :
               call near ptr VolumeEfcts
               ; and portamento
               jmp       portamento
               retn

retrigg:       ; effect 'Q'
               ; do retrigg counter ...
               cmp      ds:[channel.ctick+bp],0
               jz       doretrigg
               dec      ds:[channel.ctick+bp]
               jz       doretrigg
               retn
doretrigg:     mov      ds:[channel.sCurPos+bp],0
               mov      al,ds:[channel.Parameter+bp]
               and      al,0fh
               jnz      dovolchanges
               retn
dovolchanges:  ; do volume change :
               mov      ds:[channel.ctick+bp],al
               mov      bx,ds:[channel.cmd2nd+bp]
               jmp      [retrig_cmd2nd+bx]
slddown:       mov      cl,ds:[channel.parameter+bp]
               shr      cl,4
               mov      al,1
               shl      al,cl
               sub      ds:[channel.SampleVol+bp],al
               jnc      slddwnok
               mov      ds:[channel.SampleVol+bp],0
slddwnok:      retn
use2div3:      ; (it's 5/8 in real life ;)
               mov      al,ds:[channel.SampleVol+bp]
               mov      ah,al
               shl      al,2             ; al = 4*volume , ah = volume
               add      al,ah            ; al = 5*volume
               shr      al,3             ; al = 5*volume/8
               mov      ds:[channel.SampleVol+bp],al
               retn
use1div2:      shr      ds:[channel.SampleVol+bp],1
               retn
sldup:         mov      cl,ds:[channel.parameter+bp]
               shr      cl,4
               mov      al,1
               shl      al,cl
               add      al,ds:[channel.SampleVol+bp]
voltest:       cmp      al,64
               jb       sldupok
               mov      al,63
sldupok:       mov      ds:[channel.SampleVol+bp],al
               retn
use3div2:      mov      al,ds:[channel.SampleVol+bp]
               mov      ah,al
               add      al,al            ; al = 2*volume , ah = volume
               add      al,ah            ; al = 3*volume
               shr      al,1             ; al = 3*volume/2
               jmp      voltest
use2times:     mov      al,ds:[channel.SampleVol+bp]
               shl      al,1
               jmp      voltest
Tremolo:       ; effect 'R'
               ; next position in table:
               mov       al,ds:[channel.Parameter+bp]
               mov       dl,al
               and       dl,0fh
               shr       al,4
               mov       bl,ds:[channel.Tablepos+bp]
               add       bl,al

               cmp       bl,64
               jb        endoftest2
               sub       bl,64
endoftest2:
               mov       ds:[channel.Tablepos+bp],bl
               xor       bh,bh
               add       bx,ds:[channel.TrmTabOfs+bp]
               mov       al,ds:[bx]
               imul      dl
               sar       ax,6
               mov       bl,ds:[channel.oldvolume+bp]
               xor       bh,bh
               add       bx,ax
               cmp       bx,63
               jng       ok1
               mov       bl,63
ok1:           cmp       bx,0
               jnl       ok2
               mov       bl,0
ok2:           mov       ds:[channel.SampleVol+bp],bl
               retn

Specialsets:   ; effect 'S'
               mov      bx,ds:[channel.cmd2nd+bp]
               jmp      [special_cmd2nd+bx]
               retn
Notecut:       dec      ds:[channel.ndTick+bp]
               jz       docut
               retn
docut:         mov      ds:[channel.enabled+bp],0       ;disable it ...
               retn
Notedelay:     dec      ds:[channel.ndTick+bp]
               jz       StartNewNote
               retn
StartNewNote:  ; Ok now we have to calc things for the new note/instr ...
               ; 1. Setup Instrument
               push     fs              ; segment to volumetable, but we destroy it here ...
               mov      si,bp
               mov      al,[channel.savInst+si]
               cmp      al,00
               je       nonewinst
               mov      [channel.InstrNo+si],al
               call near ptr SetupNewInst
nonewinst:     mov      al,[channel.savNote+si]
               cmp      al,0ffh
               je       no_newnote
               cmp      al,0feh
               jne      normal_note
               mov      [channel.enabled+si],0     ; stop mixing
               jmp      no_newnote
normal_note:   mov      [channel.enabled+si],1     ; yo do mixing
               mov      [channel.Note+si],al
               call near ptr SetNewNote
no_newnote:    mov      al,[channel.savVol+si]
               cmp      al,0ffh
               je       no_vol
               mul      [gvolume]
               shr      ax,6
               mov      [channel.SampleVol+si],al
no_vol:        pop      fs
               mov      ds:[channel.command+si],0       ; <- no more Notedelay
               retn

fineVibrato:   ; effect 'U'
               cmp       ds:[channel.enabled+bp],0
               je        novib
               ; next position in table:
               mov       al,ds:[channel.VibPara+bp]
               mov       dl,al
               and       dl,0fh
               shr       al,4
               mov       bl,ds:[channel.Tablepos+bp]
               add       bl,al

               cmp       bl,64
               jb        f_endoftest
               sub       bl,64
f_endoftest:
               mov       ds:[channel.Tablepos+bp],bl
               xor       bh,bh
               add       bx,ds:[channel.VibTabOfs+bp]
               mov       al,ds:[bx]
               imul      dl
               sar       ax,8
               mov       bx,ds:[channel.Oldperiod+bp]
               add       ax,bx
               jmp       calcnewSF

ENDS
END