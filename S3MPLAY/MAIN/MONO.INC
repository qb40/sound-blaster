mono_tick0:    cmp   [patterndelay],0
               je    nodelay     ; or pattern delay done ...
               dec   [patterndelay]
               jz    nodelay
               dec   [curline]
nodelay:       setborder 4
               call near ptr [READNEWNOTES]
               setborder 1
               jmp  continuecalc

aNewtick:      mov           ax,[BPT]
               mov           [TickBytesLeft],ax
               mov           [calleffects],1
               cmp           [curtick],1
               jbe           mono_tick0
               dec           [curtick]
               jmp           continuecalc

calc_mono_tick PROC NEAR
               push          bp
               ; first fill tickbuffer with ZERO = 2048+offset post8bit
               ; for 16bit play then ofcourse a bit different value ...
               ; just only for 8bit play mode
               mov           ax,word ptr [offset tickbuffer +2]
               mov           es,ax
               mov           ax,2048+offset post8bit
               xor           di,di
               mov           cx,[DMArealBufsize+2]
               setborder     3
               rep stosw
               setborder     1
               mov           [nextPosition],0
               mov           [calleffects],0
               cmp           [TickBytesLeft],0
               jz            aNewTick
continuecalc:
               mov           ax,word ptr [offset tickbuffer +2]
               mov           es,ax 
               cmp           [EndOfSong],1
               je            afterall
               mov           al,[usedchannels]
               mov           [curchannel],al
               
               ; number of ticks we calc for every tick :
               mov           ax,[TickBytesLeft]
               mov           cx,[DMArealBufsize+2]
               sub           cx,[nextPosition]
               cmp           cx,ax
               jbe           cantfinishTick
               mov           cx,ax     ; finish that Tick and loop to fill the whole tickbuffer
cantfinishTick:
               mov           [sample2calc],cx
               cmp           cx,0
               je            afterall

               xor           bp,bp
chnLoop:       cmp           ds:[channel.channeltyp+bp],0
               je            nextchannel
               cmp           ds:[channel.channeltyp+bp],2
               ja            nextchannel
               cmp           [calleffects],0
               je            noeff
               ; do effects for this channel :
               mov           al,[curtick]
               cmp           al,[curspeed]
               je            noeff_forfirst
doeff:         mov           bx,ds:[channel.command+bp]
               cmp           bx,255*2
               je            noeff
               call          [effects + bx]
noeff:
noeff_forfirst:
               ; check if mixing :
               cmp           ds:[channel.enabled+bp],0
               je            nextchannel

               mov           ax,ds:[channel.SampleSEG+bp]
               ; well now check if in EMS :
               cmp           ax,0f000h
               jb            noEMSsample
               and           ax,0fffh
               mov           bx,ax
               mov           di,ds:[channel.sLoopend+bp]
               mov           ax,04400h
               mov           dx,[smpEMShandle]  ; dx = handle
onemorepage:   push          bp bx ax dx di
               ; Set page number:
               int     67h
               cmp     ah,0
               je      noemsprob
               mov     dl,0
               div     dl         ; <- cause a "div by 0" because EMSdriver does not work correct
noemsprob:     pop           di dx ax bx bp
               inc           al
               inc           bx
               sub           di,16*1024
               jnc           onemorepage
               mov           ax,[frameseg]
noEMSsample:   mov           gs,ax

               xor           ebx,ebx
               mov           bh,ds:[channel.SampleVol+bp]

               lfs           si,[volumetableptr]

               mov           si,[nextPosition]
               shl           si,1
               mov           cx,[sample2calc]

               mov           edi,ds:[channel.sCurpos+bp]
               rol           edi,16

               mov           edx,ds:[channel.sStep+bp]
               rol           edx,16

               cmp           di,ds:[channel.sLoopend+bp]
               jae           sampleends1
back2calc1:
               ; ES:SI - pointer to tickbuffer
               ; GS:DI - pointer to sampledata
               ; FS:BX - pointer to volumetable
               ; DX    - decision part of current position in sample
               ; DI    - integer part of current position in sample
               ; BH    - volume of instrument
               ; CX    - number of values to calc
               ; DS,BP - under use, but not in inner loop <- not optimized (hey come on, I just started to code this)

               ; jump into innerloop :
               push          bp
               mov           bp,cx
               and           bp,31
               shr           cx,5
               inc           cx

               neg           bp
               add           bp,32

               cmp           bp,32  ; <- that cost me some minutes to think about :(
               jne           no0

               dec           cx
               xor           bp,bp

no0:           shl           bp,1
               sub           si,bp
               ;sub           si,bp

               add           bp,offset mn_innerloop_tbl
               ; before jump arround =) swap fs,ds
               push         ds fs
               mov          ax,ds
               mov          fs,ax
               pop          ds
               push         ds
               jmp          word ptr fs:[bp]

macro_mninner  MACRO no
align 2
mn_inner&no:   mov           bl,gs:[di]

               add           edi,edx
               adc           di,0

               mov           ax,ds:[ebx+ebx]  ; convert samplevalue with volumetable

               add           es:[si+pos],ax     ; mix value to other channels
ENDM

tickloop:
z = 0
pos = 0
rept 32        ; I know I'm crazy :) but what we do for fast inner loops ...
macro_mninner %z
z = z + 1
pos = pos + 2
endm
               add           si,32*2
               dec           cx
               jnz           tickloop

               pop           fs
               pop           ds
               pop           bp

aftercalc:     rol           edi,16
               mov           ds:[channel.sCurpos+bp],edi

nextchannel:   add           bp,size channel
               dec           [curchannel]
               jnz           chnLoop

               mov           ax,[sample2calc]
               sub           [TickBytesLeft],ax
               add           [nextPosition],ax
               mov           ax,[DMArealBufsize+2]
               cmp           [nextPosition],ax
               jb            aNewTick

afterall:      pop           bp
               ret

sampleends1:   cmp           ds:[channel.sloopflag+bp],0
               je            no_loopflag1
tryagain1:     sub           di,ds:[channel.sloopEnd+bp]
               add           di,ds:[channel.sloopstart+bp]
               cmp           di,ds:[channel.sloopEnd+bp]
               jae           tryagain1
               jmp           back2calc1
no_loopflag1:  mov           ds:[channel.enabled+bp],0
               jmp           aftercalc

calc_mono_tick ENDP