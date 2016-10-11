{$A+,B-,D+,E-,F-,G+,I-,L+,N-,O-,P-,Q-,R-,S-,T-,V-,X+,Y+}
{$M 16384,0,655360}
Unit Blaster;

{ -------------------------------------------------------------------- }
{ This is a specially for detecting and using the soundblaster card.   }
{ autodetect routines work for dsp adresses 210 to 280,                }
{ 8bit dmachannels 0,1,3/ 16bit dmachannel 5 and for interrupts 2,5,7  }
{ (known problems with IRQ2)                                           }
{ -------------------------------------------------------------------- }
{ no more work on autodetection - wait for craigs source, takes better }
{ advantage to our knowledge about different SB models, we got in last }
{ time. i.e. you can read IRQ#/DMA8/DMA16 from mixer port on SB16      }
{ --------------------------------------------------------------------- }
{ I read about 16Bit improvment for SB PRO3/4.0 but don't know about -  }
{ anybody can help me with that ?                                       }
{ how to use it ... any specifications for it ?                         }
{ --------------------------------------------------------------------- }
{ STATUS: TESTED ON SB16/ASP,SB PRO2,SB2.0                              }
{         COMMENTs READY                                                }
{   KNOWN PROBLEMS : - IRQ2 detection is wrong on some maschines        }
{                    - no 16bit IRQ autodetection :(                    }
{                    - no check if values are right in                  }
{                               * UseBlasterEnv                         }
{                               * InputBlasterValues                    }
{                    - no mixedup detection (i.e. some values as        }
{                      parameters and detect the rest)                  }
{                      I though about this while I had a SB PRO and a   }
{                      SB16 at the same time 'inside' on base 220h,240h }
{ --------------------------------------------------------------------- }

Interface

{ the different SoundBlaster versions and its hardware : }
{  0. - no soundblaster present
   1. Soundblaster 1.0/1.5
        23kHz(mono)                   8bit        no mixer

   2. Soundblaster Pro
        45kHz(mono)    23kHz(stereo)  8bit        mixer

   3. Soundblaster 2.0/ Audioblaster 2.5
        45kHz(mono)                   8bit        audiob. with mixer

   4. Soundblaster Pro3/Audioblaster Pro 4.0
        45kHz(mono)    23kHz(stereo)  8bit        mixer

   5. Soundblaster Pro (Mircochannel)
        Special version for PS/2 - technical data = 4

   6. Soundblaster 16/16ASP
        45kHz(mono)    45kHz(stereo)  8/16bit        mixer

   DSP versions :

   SoundBlaster 1.0/1.5        1.xx
   SoundBlaster 2.0/2.5        2.xx
   SoundBlaster Pro/PRO3/PRO4  3.xx
   SoundBlaster 16/ASP         4.xx
}

CONST defaultvolume=31;    { full power =;) }
      IRQ_TABLE:array[0..15] of byte = ($08,$09,$0a,$0B,$0C,$0D,$0E,$0F,
                                        $70,$71,$72,$73,$74,$75,$76,$77);

VAR
    stereo_possible:Boolean;    (* flag if stereo is possible on detected SB *)
    _16Bit_possible:Boolean;    (* flag if 16bit play is possible on detected SB *)
    maxstereorate:word;         (* max stereo samplerate on detected SB *)
    maxmonorate:word;           (* max mono samplerate on detected SB *)

    Stereo:Boolean;             (* flag if stereo-play on/off *)
    _16Bit:Boolean;             (* flag if 16bit-play on/off *)
    SBNo:byte;                  (* SoundBlaster typ (look some rows above) *)
    signeddata:boolean;         (* play signed data ? (only on SB16 possible) *)

    IRQ_No:Byte;                (* IRQ detected SB uses *)
    DSP_Addr:Word;              (* Baseaddress detected SB uses *)
    DMA_Channel:Byte;           (* DMA channel for 8 Bit play *)
    DMA_16BitChannel:byte;      (* DMA channel for 16 Bit play *)

PROCEDURE Forceto(typ,dma,dma16,irq:byte;dsp:word);     (* force to use these values for playing *)
FUNCTION UseBlasterEnv:boolean;                         (* use values set in enviroment BLASTER *)
FUNCTION InputSoundblasterValues:Boolean;               (* Input Soundbaster values by hand (in textmode) *)
FUNCTION DetectSoundblaster(prot:boolean):Boolean;      (* detects Soundblastercard *)

FUNCTION Detect_DSP_Addr(prot:boolean):Boolean;         (* detects DSP ADDR *)
FUNCTION Detect_DMA_Channel_IRQ(prot:boolean):Boolean;  (* detects DMA Channel,DSPIRQ *)
FUNCTION Get_BlasterVersion:Word;                       (* reads the detected soundblaster version *)

procedure check_samplerate(var rate:word;var stereo:boolean); (* check min/max samplerate *)

procedure set_DMAvalues(p:pointer;Length:word;autoinit:boolean);
  (* config DMAcontroller for different transfer modes *)
procedure play_firstBlock(length:word);
  (* set the SBinterrupt to "interrupt" every "length" bytes
     the best way to use it, is to play the half buffersize and then
     write the new data in the allready played part
     - you have to setup DMA controller by calling set_DMAvalues *)
PROCEDURE play_oneBlock(p:pointer;length:word);
  (* it's a play routine for playing only one buffer (NOT CONTINOUSLY
     PLAYING ! - it does not work in that way on a SB16 !)
     we use non autoinit mode here for all SBs...
     this proc. setup DMA controller *)
PROCEDURE Initblaster(var frequ:Word;stereoon,_16Biton:Boolean);
  (* set frequency and stereo/mono and 8/16 Bit mode *)

PROCEDURE wr_mixerreg(reg,wert:byte);
  (* writes something to the mixing chip *)
FUNCTION  rd_mixerreg(reg:byte):byte;
  (* reads something from the mixing chip *)

PROCEDURE set_ready_irq(p:pointer);
  (* set user irq - (irq if buffer ends !)
     attention look at my default irq !
     You need those two port commands you'll find there !
     And pay attention to 16bit/8bit mode on SB16 (different
     ackknowledgement ports) *)
PROCEDURE stop_play;         (* stops playing ! *)
PROCEDURE pause_play;        (* stops playing, but you can continue
                                with "continue_play" *)
PROCEDURE continue_play;     (* continues playing after pause play *)
PROCEDURE restore_irq;       (* restore old interrupt vector *)
PROCEDURE set_sign(signed:boolean); (* sets flag to play signed data
               - does only work on SB16 (set it before start playing) *)
PROCEDURE setvolume(vol:Byte);   (* what do you think ? *)
PROCEDURE speaker_on;            (* Does not work on SB16 *)
PROCEDURE speaker_off;
procedure write_zaehler;         (* It's for 8 & 16 Bit mode to get the DMA counter *)
function  get_zaehler:word;      (* It's for 8 & 16 Bit mode to get the DMA counter *)
procedure writelnSBConfig;       (* what do you expect ? - write current setup to screen,
                                    but detect SB before calling that proc. *)

Implementation

uses dos,crt;

{ Flags and variables for detect part : }
VAR SB_Detect:Boolean;              { Flag if SB is detected }
    DSPIRQ_Detect:Boolean;          { Flag if IRQ number is detected }
    DSPADR_Detect:Boolean;          { Flag if Baseaddress is detected }
    DMACHN_Detect:Boolean;          { Flag if DMAchannel is detected }
    MIXER_Detect:Boolean;           { Flag if Mixerchip is detected }
    SBVersHi:Byte;                  { Soundblaster version major }
    SBVersLo:Byte;                  { Soundblaster version minor }

    check:byte;                     { for detecting }
    savvect:pointer;                {  "       "    }

{ Soundblaster handling : }

function try_reset(p:word):Boolean; assembler;
  asm
        mov        bl,1
        mov        dx,p
        add        dx,6
        mov        al,1         { write 1 to port 2x6 }
        out        dx,al
        in         al,dx
        in         al,dx
        in         al,dx
        in         al,dx
        xor        al,al
        out        dx,al        { after 3,3 æs write 0 to port 2x6 }

{ And now check the answer }
        add        dx,8
        mov        si,200
@@readloop:
        mov        cx,0ffffh      { SB2.0/1.0 are that slow :( }
@@testl:                          { check for data available }
        in         al,dx
        dec        cx
        jz         @@not
        or         al,al
        jns        @@testl

        sub        dx,4
        in         al,dx        { read data comming through }
        cmp        al,0aah
        je         @@aSB
        add        dx,4
        dec        si
        jnz        @@readloop
@@not:  mov        bl,0         { it's not a SB :( }
@@aSB:  xor        ah,ah
        mov        al,bl
  end;

procedure wr_dsp; assembler;
{ it's WAITWRITE and then WRITE IT }
  asm
        push      bx
        push      cx
        mov       bh,al
        mov       dx,dsp_addr
        add       dx,0ch
        mov       cx,0ffffh        { ya know, slow SBs }
        { Wait for writing : }
@@litl: in        al,dx
        dec       cx
        jz        @@ende
        or        al,al
        js        @@litl  { check bit 7 if we can write to port 2xC }
        mov       al,bh
        out       dx,al   { write it }
@@ende: pop       cx
        pop       bx
  end;

procedure speaker_off;
  begin
    asm
      mov       al,0d3h
      call      wr_dsp
      push      220             { needs a bit time to switch it off }
      call far ptr delay
    end;
  end;

procedure speaker_on;
  begin
    asm
      mov       al,0d1h
      call      wr_dsp
      push      110             { needs a bit time to switch it on }
      call far ptr delay
      end;
  end;

function rd_dsp:byte; assembler;
{ It's WAITREAD and then READ byte }
  asm
        mov       dx,dsp_addr
        add       dx,0eh
        mov       cx,0ffffh    { ya know - slow SBs. You can believe me ! }
        { check for data available : }
@@litl: in        al,dx
        dec       cx
        jz        @@ende
        or        al,al
        jns       @@litl   { bit 7 set ? if not then wait }
        sub       dx,0eh-0ah
        in        al,dx    { write data }
        xor       ah,ah
@@ende:
  end;

procedure wr_mixerreg(reg,wert:byte); assembler;
{ this routine may not work for all registers because of different timings.}
  asm
    cmp       [SBNo],1
    je        @@nomixer                   { SB 1.0/1.5 has no mixer ! }
    cmp       SBNo,3
    je        @@nomixer                   { SB 2.0/2.5 has no mixer ! }
    mov       al,reg
    mov       dx,dsp_addr
    add       dx,4
    out       dx,al
    inc       dx
    in        al,dx
    mov       al,wert
    out       dx,al
@@nomixer:
  end;

function rd_mixerreg(reg:byte):byte; assembler;
  asm
    cmp       [SBNo],1
    je        @@nomixer                   { SB 1.0/1.5 has no mixer ! }
    cmp       SBNo,3
    je        @@nomixer                   { SB 2.0/2.5 has no mixer ! }
    mov     dx,dsp_addr
    add     dx,4
    mov     al,reg
    out     dx,al
    inc     dx
    in      al,dx
    xor     ah,ah
@@nomixer:
  end;

{ Sorry no cool macro like in C is possible }
function loword(l:longint):word; assembler;
  asm
    mov         ax,word ptr(l)
  end;

function hiword(l:longint):word; assembler;
  asm
    mov         ax,word ptr(l+2)
  end;

procedure set_DMAvalues(p:pointer;Length:word;autoinit:boolean);
{ If you want to know more about how to setup DMA controller, please
  refer to our documentation SBLASTER.ZIP. }
const pagetable:array[0..7] of word = ($0087  { channel 0 },
                                       $0083  { channel 1 },
                                       $0081  { channel 2 <- not used by SB },
                                       $0082  { channel 3 },
                                       $008F  { channel 4 <- not used by SB },
                                       $008B  { channel 5 },
                                       $0089  { channel 6 },
                                       $008A  { channel 7 });
  begin
    asm
                { start : }
                cmp       [_16Bit],1      { setup 16bit DMA channels 4..7 is
                                            ab bit different }
                je        @@higherDMA
                { first the SBPRO stereo bugfix : }
                cmp       [stereo],0
                je        @@nostereo
                cmp       [sbNo],6
                jae       @@sbhigher
                { well ... should be a SB PRO in stereo mode ... }
                { let's send one byte ! }
                mov       al,10h
                call      wr_dsp
                mov       al,128   { nothin but silence ! }
                call      wr_dsp
@@sbhigher:
@@nostereo:
                { convert pointer in realadress :
                  dmapage*65536+dmaoffset = memsegment*16 + memoffset }
                mov       ax,word ptr(p+2)
                rol       ax,4
                mov       cl,al
                and       al,0f0h
                and       cl,0fh
                mov       di,ax         { cl:di - realadress ! }

                mov       bh,dma_channel { bh with dma_channel }
                mov       bl,bh
                shl       bl,1          { bl with dma_channel*2 }
                mov       ch,048h       { ch = 010010xx }
                                         {     \| |+- read
                                                | +- autoinit flag
                                                +- singlemode }
                mov     al,[autoinit]
                shl     al,4
                or      ch,al              { set the autoinit flag }
                add     ch,dma_channel     { prepare for dma_channel }

                mov     al,4
                add     al,bh           { bh = dma_channel }
                out     0ah,al          { mask the channel }

                xor     al,al
                out     0ch,al          { clear flipflop }

                mov     al,ch           { ch = DMAmode }
                out     0bh,al          { set dmatransfer mode }
                mov     ax,di           { di = DMAbuffer offset }
                push    bx
                xor     bh,bh
                mov     dx,bx           { bx = 2*dma_channel }
                out     dx,al           { lower adress }
                mov     al,ah
                out     dx,al           { higer adress }
                mov     dx,word ptr (pagetable+bx)
                mov     al,cl           { cl = DMAbuffer page }
                out     dx,al           { data page }
                mov     dx,bx           { bx = 2*dma_channel }
                mov     ax,Length
                dec     ax
                inc     dx              { dx = write base count }
                out     dx,al           { write lower length }
                mov     al,ah
                out     dx,al           { write higer length }
                pop     bx
                mov     al,dma_channel
                out     0ah,al          { demask channel }
                jmp     @@endofsetauto
@@higherDMA:    { jump to here if 16bit DMA setup }
                { convert pointer in realadress : }
                mov       ax,word ptr(p+2)      { ax = segment of buffer }
                rol       ax,4                  { attention no offset ! }
                mov       cl,al
                and       al,0f1h
                and       cl,00eh
                ror       ax,1
                mov       di,ax                 { cl:di - realadress ! }

                mov     bh,dma_16Bitchannel     { bh with dma_channel }
                sub     bh,4                    { channel 4-7 to number 0-3 }
                mov     bl,bh
                shl     bl,2
                add     bl,0c0h       { bl = DMA adressport for current DMAchannel }
                                      { bh = DMA16Bitchannel - 4  (0..3) }
                mov     ch,048h       { ch = 010010xx }
                                       {     \| |+-read
                                              | +-autoinitflag
                                              +-singlemode }
                mov     al,[autoinit]
                shl     al,4
                or      ch,al          { set the autoinitflag }
                add     ch,bh          { ch = command for DMAchannel # }

                mov     al,4
                add     al,bh           { bh = 16bitDMA_channel - 4 }
                out     0d4h,al         { mask the channel }
                xor     al,al
                out     0d8h,al         { clear flipflop }
                mov     al,ch           { ch = dmamode }
                out     0d6h,al         { set dmatransfer mode }
                mov     ax,di           { lower part of adress }
                push    bx
                xor     bh,bh           { bx now = c0h/c4h/c8h/cch (0..3) }
                mov     dx,bx           { bx = addressport for current channel }
                out     dx,al           { write lower adress }
                mov     al,ah
                out     dx,al           { write higer adress }
                mov     dl,dma_16Bitchannel
                xor     dh,dh
                mov     si,dx           { si = dma16Bitchannel }
                shl     si,1            { si - position in pagetable }
                mov     dx,word ptr (pagetable+si)
                mov     al,cl           { dmabuffer page }
                out     dx,al           { data page }
                mov     dx,bx           { old dx value ;) c0h/c4h/c8h/cch }
                mov     ax,Length
                dec     ax
                add     dx,2            { seperated by 2 -> dx now DMA base count }
                out     dx,al           { write lower length }
                mov     al,ah
                out     dx,al           { write higer length }
                pop     bx
                mov     al,bh
                out     0d4h,al         { demask channel }
@@endofsetauto:
  end;
end;

function get_zaehler:word; assembler;
{ get the dma base counter of dmachannel is used by SB
  you can check if sound transfer does work ;) }
    asm
       cmp      [_16Bit],1
       je       @@get16
       xor      al,al
       out      0ch,al           { clear flipflop }
       mov      dl,dma_channel
       xor      dh,dh
       shl      dx,1
       inc      dx               { dx = channel * 2 + 1 = base counter }
       in       al,dx            { al = lower byte }
       mov      bl,al
       in       al,dx            { al = higher byte }
       mov      bh,al
       mov      ax,bx            { AX = high and low part together ;) - return that }
                                 { bytes left to send = ax + 1 }
       jmp      @@endofget
@@get16:
       xor      al,al
       out      0d8h,al           { clear flipflop }
       mov      dl,dma_16Bitchannel
       xor      dh,dh
       sub      dl,4              { channel 4..7 to number 0..3 }
       shl      dx,2
       add      dx,0c2h           { dx = 0c2h + 4 * (channel-4) = 16bit base counter }
       in       al,dx             { AL = lower part }
       mov      bl,al
       in       al,dx             { AL = higher part }
       mov      bh,al
       mov      ax,bx             { AX = 16bit value ;) -
                                    number WORDS (!) left to send = ax + 1 }
@@endofget:
    end;

procedure write_zaehler;
{ A stupid function I know, but get_zaehler did not exist in testphase
  of my player, that was all in write_zaehler implemented, but later I
  thought it would be usefull to implement get_zaehler for debugging. }
  begin
    write(' ',get_zaehler,' ');
  end;

procedure play_firstBlock(length:word);
{ call this if you want to do continues play }
  begin
    asm
                cmp       [SBNo],6
                je        @@sb16init          { use special commands on SB16 }

                mov       bl,90h              { DSP 90h - autoinit highspeed DMA }
                cmp       [SBNo],1
                jne       @@highspeed         { >SB1.0 use highspeed modes }
                { for SB1.0 : }
                mov       bl,1ch              { DSP 1Ch - autoinit normal DMA }
@@highspeed:
                mov       cx,length
                dec       cx
                mov       al,048h             { DSP 48h - setup DMA buffer size }
                call      wr_dsp
                mov       al,cl               { lower part of size }
                call      wr_dsp
                mov       al,ch               { higher part of size }
                call      wr_dsp
                mov       al,bl               { DSP command depends on SB }
                call      wr_dsp
                jmp       @@ende

@@sb16init:     mov       cx,length
                dec       cx
                cmp       [_16Bit],1          { other command for 16bit play ... }
                je        @@play16Bit
                mov       al,0c6h             { DSP c6h - use 8bit autoinit }
                call      wr_dsp
                mov       al,signeddata
                shl       al,4                { 2nd command byte: bit 4 = 1 - signed data }
                cmp       [stereo],0
                je        @@nostereo
                or        al,020h             { 2nd command byte: bit 5 = 1 - stereo data }
@@nostereo:     call      wr_dsp              { write 2nd command byte }
                mov       al,cl               { lower part of size }
                call      wr_dsp
                mov       al,ch               { higher part of size }
                call      wr_dsp
                jmp       @@ende
@@play16Bit:    mov       al,0B6h             { DSP B6h - use 16bit autoinit }
                call      wr_dsp
                mov       al,signeddata
                shl       al,4                { 2nd command byte: bit 4 = 1 - signed data }
                cmp       [stereo],0
                je        @@nostereo2
                or        al,020h             { 2nd command byte: bit 5 = 1 - stereo data }
@@nostereo2:    call      wr_dsp              { write 2nd command byte }
                mov       al,cl               { lower part of size }
                call      wr_dsp
                mov       al,ch               { higher part of size }
                call      wr_dsp
@@ende:
    end;
  end;

PROCEDURE play_oneBlock(p:pointer;length:word);
{ call this if you want to play only ONE (!) block - I'm sure you can do
  continues play with this proc. on SBs <SB16 (I have seen that often in
  other sources, but it'll definitly not work on a SB16 ! It'll cause
  'ticks' }
  begin
    set_DMAvalues(p,length,false);
    asm
                
                cmp       [SBNo],6
                je        @@sb16init          { use special commands on SB16 }

                mov       bl,91h              { DSP 91h - nonautoinit highspeed DMA }
                cmp       [SBNo],1
                je        @@highspeed         { >SB1.0 use highspeed mode }
                { On SB1.0 : }
                mov       bl,14h              { DSP 14h - nonautoinit normal DMA }
@@highspeed:
                mov       cx,length
                dec       cx
                mov       al,048h             { DSP 48h - setup DMA buffer size }
                call      wr_dsp
                mov       al,cl               { lower part of size }
                call      wr_dsp
                mov       al,ch               { higher part of size }
                call      wr_dsp
                mov       al,bl               { DSP command depends on SB }
                call      wr_dsp
                jmp       @@ende

@@sb16init:     mov       cx,length
                dec       cx
                cmp       [_16Bit],1          { other command for 16bit play ... }
                je        @@play16Bit
                mov       al,0c2h             { DSP c2h - use 8bit nonautoinit }
                call      wr_dsp
                mov       al,signeddata
                shl       al,4                { 2nd command byte: bit 4 = 1 - signed data }
                cmp       [stereo],0
                je        @@nostereo
                or        al,020h             { 2nd command byte: bit 5 = 1 - stereo data }
@@nostereo:     call      wr_dsp              { write 2nd command byte }
                mov       al,cl               { lower part of size }
                call      wr_dsp
                mov       al,ch               { higher part of size }
                call      wr_dsp
                jmp       @@ende
@@play16Bit:    mov       al,0B2h             { DSP B2h - use 16bit nonautoinit }
                call      wr_dsp
                mov       al,signeddata
                shl       al,4                { 2nd command byte: bit 4 = 1 - signed data }
                cmp       [stereo],0
                je        @@nostereo2
                or        al,020h             { 2nd command byte: bit 5 = 1 - stereo data }
@@nostereo2:    call      wr_dsp              { write 2nd command byte }
                mov       al,cl               { lower part of size }
                call      wr_dsp
                mov       al,ch               { higher part of size }
                call      wr_dsp
@@ende:
    end;
  end;

{ -------------------- continue commenting here ---------------------- }

PROCEDURE SetTimeConst(tc:byte);
{ Setup samplerate with time constant, take this :
  TC = 256- TRUNC(1000000/SAMPLERATE) }
  begin
    asm
      mov       al,040h
      call      wr_dsp
      mov       al,tc
      call      wr_dsp
    end;
  end;

PROCEDURE Initblaster(var frequ:Word;stereoon,_16Biton:boolean);
{ Initblaster does this :   1. check samplerates for its borders
                            2. Reset DSP chip
                            3. setup samplerate
                            4. setup stereo/mono mode
 if you want to play signed data on SB16, call 'set_sign' after Initblaster }

var tc:byte;
    w:word;
  begin
    { first reset SB : }
    asm
      mov    dx,dsp_addr
      add    dx,0eh
      in     al,dx
      inc    dx
      in     al,dx
    end;
    stop_play;
    { Now init : }
    check_samplerate(frequ,stereoon);
    _16bit:=(SBNo=6) and _16Biton;
    stereo:=stereoon;
    { calculate timeconstant - pay attention on SB PRO you have to setup
      2*samplerate in stereo mode (so call it byterate) - on SB16 not ! }
    if (sbno=6) or not stereo then
      begin
        tc:=256-1000000 div frequ;
        frequ:=1000000 div (256-tc);
      end
    else
      begin
        tc:=256-1000000 div (2*frequ);
        frequ:=(1000000 div (256-tc)) div 2;
      end;
    w:=frequ;
    try_reset(dsp_addr);
    { set sampling rate }
    if (sbno<6) then
    asm
       { on all normal SB's :) }
       mov      al,040h
       call     wr_dsp
       mov      al,tc
       call     wr_dsp
    end
    else
    asm
       { on SB16 }
       mov      al,041h
       call     wr_dsp
       mov      ax,w
       xchg     al,ah
       call     wr_dsp
       mov      al,ah
       call     wr_dsp
    end;
    { setup stereo option on SB PRO - on SB16 it's set in DSP command }
    if stereo and (SBNo<>6) then
      wr_mixerreg($0e,rd_mixerreg($0e) or $02); { stereo option on (only SB PRO) }
    if SBNo in [2,4,5] then
      wr_mixerreg($0e,rd_mixerreg($0e) or $20); { filter option off (only SB PRO) }
    speaker_on;
  end;

{ -------------- now the procedures for my old autodetection ------------- }
{ No comments about it - it's old ;)                                       }
procedure irq2;interrupt;var a:byte; begin check:=2;port[$20]:=$20;a:=port[dsp_addr+$0e] end;
procedure irq5;interrupt;var a:byte; begin check:=5;port[$20]:=$20;a:=port[dsp_addr+$0e] end;
procedure irq7;interrupt;var a:byte; begin check:=7;port[$20]:=$20;a:=port[dsp_addr+$0e] end;
procedure ready_irq; interrupt;var a:byte; begin check:=1;port[$20]:=$20;a:=port[dsp_addr+$0e] end;

function hexword(w:word):string;
const hex:string= '0123456789ABCDEF';
  begin
   hexword:=hex[hi(w) div 16+1]+hex[hi(w) mod 16+1]+hex[lo(w) div 16+1]+hex[lo(w) mod 16+1];
  end;

FUNCTION Detect_DSP_Addr(prot:boolean):Boolean;
var p:word;
  begin
    if dspadr_detect then begin detect_dsp_addr:=true; exit end;
    if prot then writeln(' Now locating DSP-Adresse :'#13#10);
    detect_dsp_addr:=false;
    p:=$210;
    while not dspadr_detect and (p<$290) do
      begin
        if prot then write(' Trying ',hexword(p),' .... ');
        dspadr_detect:=try_reset(p);
        if not dspadr_detect then
          begin
            inc(p,$10);
            if prot then write('not ');
          end;
        if prot then writeln('succesfull ');
      end;
    if not dspadr_detect then exit;
    dsp_addr:=p;
    detect_dsp_addr:=true;
  end;

PROCEDURE reset_mixer;
  begin
    asm
      mov    dx,dsp_addr
      add    dx,4
      mov    al,0
      out    dx,al
      mov    cx,50
@@loop: loop @@loop
      inc    dx
      inc    al
      out    dx,al
    end;
  end;

FUNCTION Detect_DMA_Channel_IRQ(prot:boolean):Boolean;
const irqs:array[1..3] of byte = (10,13,15); (* IRQ 2,5,7 *)
var oldv:array[1..5] of pointer;
    i,nr:byte;
    fr:word;
    ov1,ov2:byte;
  begin
    asm
      mov al,0ffh
      out 0fh,al
      sti
    end;
    if dmachn_detect then begin detect_DMA_Channel_irq:=true;exit end;
    if prot then writeln(#13#10' Now locating DMA-channel and IRQ :'#13#10);
    detect_dma_channel_irq:=false;
    if not dspadr_detect then exit;
    for i:=1 to 3 do
      begin
        getintvec(irqs[i],oldv[i]);
      end;
    setintvec(10,addr(irq2));
    setintvec(13,addr(irq5));
    setintvec(15,addr(irq7));
    Detect_DMA_Channel_irq:=false;
    port[$21]:=port[$21] and $5F;  { 01011111b = 05Fh }
    nr:=0;
    while (nr<4) and not DMACHN_Detect do
      begin
        if prot then write(' Trying Channel ',nr,' .... ');
        Check:=0;
        DMA_Channel:=nr;
        fr:=10000;
        asm
          mov al,dma_channel        { mask channel - means stop transfer }
          out 0ah,al
        end;
        stop_play;speaker_off;
        Initblaster(fr,false,false);
        play_oneblock(ptr(0,0),1);
        delay(10);
        DMACHN_Detect:=check<>0;
        if not DMACHN_Detect then
          begin
            inc(nr);if nr=2 then nr:=3;
            if prot then write('not ');
          end;
        if prot then
          begin
            write('sucessful');
            if DMACHN_Detect then writeln(' with Interrupt ',IRQ_Table[check],' - IRQ ',check)
            else writeln;
          end;
      end;
    port[$21]:=port[$21] or $A0;  { 10100000b = 0A0h }
    for i:=1 to 3 do
      setintvec(irqs[i],oldv[i]);
    if not dmachn_detect then exit;
    Detect_DMA_Channel_irq:=true;
    DSPIRQ_detect:=true;
    IRQ_no:=Check;
    try_reset(dsp_addr);
  end;

procedure Fix_blastertype;
  var b1,b2:byte;
  begin
    asm
      mov       al,0E1h        { DSP E1h - get DSP version }
      call      wr_dsp
    end;
    sbversHi:=rd_dsp;
    sbversLo:=rd_dsp;
  end;

FUNCTION DetectSoundblaster(prot:boolean):Boolean;
  begin
    SBNo:=0;
    DetectSoundblaster:=false;
    SB_Detect:=False;
    DSPIRQ_Detect:=false;
    DSPADR_Detect:=false;
    DMACHN_Detect:=False;
    MIXER_Detect:=False;
    stereo_possible:=false;
    _16Bit_possible:=false;
    STEREO:=False;_16Bit:=False;
    if not Detect_DSP_Addr(prot) then
      begin
        if prot then writeln(' Can'#39't locate DSP-addresse ! ');
        exit;
      end;
    fix_blastertype;

    if (sbversHi<1) or (sbversHi>4) then
      begin
        if prot then writeln(' Sorry, unknown DSP chip version on this base address detected.');
        SBno:=0;
        exit;
      end;
    { for the first set SB1.0 - should work on all SBs }
    SBNo:=1;stereo_possible:=false;_16Bit_possible:=false;
    maxmonorate:=22050;maxstereorate:=0;
    stop_play;
    if not Detect_DMA_Channel_irq(prot) then
      begin
        if prot then writeln(' Can'#39't locate DMA-channel and IRQ ! ');
        sbNo:=0;
        exit;
      end;

    try_reset(dsp_addr);

{                              SBvers:
   SoundBlaster 1.0/1.5        1.xx
   SoundBlaster 2.0/2.5        2.xx
   SoundBlaster Pro/PRO3/PRO4  3.xx
   SoundBlaster 16/ASP         4.xx
}
    case sbversHi of
      1: begin
           SBNo:=1;stereo_possible:=false;_16Bit_possible:=false;
           maxmonorate:=22050;maxstereorate:=0
         end;
      2: begin
           SBNo:=3;stereo_possible:=false;_16Bit_possible:=false;
           maxmonorate:=44100;maxstereorate:=0
         end;
      3: begin
           SBNo:=2;stereo_possible:=true;_16Bit_possible:=false;
           maxmonorate:=44100;maxstereorate:=22700
         end;
      4: begin
           SBNo:=6;stereo_possible:=true;_16Bit_possible:=true;
           maxmonorate:=45454;maxstereorate:=45454
         end;
      else begin SBNo:=0;exit end;
    end;

    DetectSoundblaster:=true;
  end;

FUNCTION Get_BlasterVersion:Word;
  begin
    Get_BlasterVersion:=word(SBVersHi)*256+SBVersLo;
  end;

PROCEDURE set_ready_irq(p:pointer);
var b:byte;
  begin
    check:=0;
    getintvec(IRQ_Table[irq_no],savvect);
    if p=Nil then p:=addr(ready_irq);
    setintvec(IRQ_Table[irq_no],p);
    b:=1 shl irq_no;b:=b or 04; { no changes for IRQ2 }
    port[$21]:=port[$21] and not b; { masking ... }
  end;

PROCEDURE restore_irq;
var b:byte;
  begin
    b:=1 shl irq_no;b:=b and not 4; { no mask for IRQ2 }
    port[$21]:=port[$21] or b;
    setintvec(IRQ_Table[irq_no],savvect);
  end;

FUNCTION ready:boolean;
  begin
    ready:=check>0;
  end;

PROCEDURE stop_play;
  begin
    { for 16bit modes : }
    asm
      mov   al,0d0h
      call  wr_dsp
      mov   al,0d9h
      call  wr_dsp
      mov   al,0d0h
      call  wr_dsp
    end;
    { for 8bit modes : }
    asm
      mov   al,0d0h
      call  wr_dsp
      mov   al,0dah
      call  wr_dsp
      mov   al,0d0h
      call  wr_dsp
    end;
    try_reset(dsp_addr);   { reset is the best way to make sure SB stops playing ! }
    asm
      mov   al,dma_channel
      out   0ah,al
    end;
  end;

PROCEDURE pause_play;
  begin
    if not _16bit then
    asm
      mov        al,0D0h
      call       wr_dsp
    end
    else
    asm
      mov        al,0D5h
      call       wr_dsp
    end
  end;

PROCEDURE continue_play;
  begin
    if not _16bit then
    asm
      mov        al,0D4h
      call       wr_dsp
    end
    else
    asm
      mov        al,0D6h
      call       wr_dsp
    end
  end;

PROCEDURE set_sign(signed:boolean);
  begin
    signeddata:=signed;
  end;

procedure setfilter(how:boolean);
var b:byte;
  begin
    b:=rd_mixerreg($0e);
    if how then { on } b:=b or $20 else b:=b and not $20;
    wr_mixerreg($0e,b); { switch the filter option }
  end;

procedure setvolume(vol:byte);
var b:byte;
  begin
    if sbno<6 then
      begin
        if vol>=15 then vol:=15;
        b:=vol;
        b:=b shl 4;        { the other side }
        vol:=b+vol;
        wr_mixerreg($22,vol);
        wr_mixerreg($04,vol);
      end
    else
      begin
        { on SB16 the new mixer registers :) }
        wr_mixerreg($30,vol);  { master left }
        wr_mixerreg($31,vol);  { master right }
        wr_mixerreg($32,vol);  { Voice left }
        wr_mixerreg($33,vol);  { Voice right }
      end;
  end;

PROCEDURE Forceto(typ,dma,dma16,irq:byte;dsp:word);
  begin
    SB_Detect:=true;
    DSPIRQ_Detect:=true;
    DSPADR_Detect:=true;
    DMACHN_Detect:=true;
    stereo:=false;
    _16Bit:=false;

    MIXER_detect:=typ>1;
    stereo_possible:=typ in [2,4,5,6];
    _16Bit_possible:= typ=6;
    IRQ_No:=irq;
    DSP_Addr:=dsp;
    DMA_Channel:=DMA;
    DMA_16BitChannel:=dma16;
    SBNo:=typ;
    case typ of
      1: begin maxmonorate:=22050;maxstereorate:=0 end;
      2: begin maxmonorate:=44100;maxstereorate:=22050 end;
      3: begin maxmonorate:=44100;maxstereorate:=0 end;
      4: begin maxmonorate:=44100;maxstereorate:=22050 end;
      5: begin maxmonorate:=44100;maxstereorate:=22050 end;
      6: begin maxmonorate:=45454;maxstereorate:=45454 end;
    end;
  end;

function UseBlasterEnv:boolean;
var s:string;
    typ,dma,dma16,irq:byte;
    dsp:word;
  function upstr(s:string):string;
    var t:string;
        i:byte;
    begin
      t:='';
      for i:=1 to length(s) do
        t:=t+upcase(s[i]);
      upstr:=t;
    end;

var count,i:byte;
    u:string;
    er:integer;

  begin
    typ:=255;dma:=255;dma16:=255;irq:=255;dsp:=$ffff;
    { default values (totally crap), but if you get them after calling Use....}
    { you'll know that this value is not/wrong defined in the BLASTER env. }
    UseBlasterEnv:=false;
    s:=upstr(getenv('BLASTER'));
    if s='' then exit; { no chance :( }
    count:=0;
    { SET BLASTER=A220 I? D? H? P??? T? }
    i:=pos('T',s); { Soundblaster typ }
    if i>0 then
      begin
        u:=copy(s,i+1,1); { maybe for future blaster versions not right :( }
        val(u,typ,er);
        if er=0 then inc(count); { yeah we got this value ! }
      end;
    i:=pos('D',s); { DMAchannel }
    if i>0 then
      begin
        u:=copy(s,i+1,1);
        val(u,dma,er);
        if (er=0) and (dma<4) and (dma<>2) then inc(count) { yeah we got it ! }
          else dma:=255;
      end;
    i:=pos('I',s); { IRQ number }
    if i>0 then
      begin
        if s[i+2]<>' ' then u:=copy(s,i+1,2) else u:=copy(s,i+1,1);
        val(u,irq,er);
        if (er=0) and ((irq=2) or (irq=5) or (irq=7) or (irq=10)) then inc(count)
          else irq:=255;
      end;
    i:=pos('H',s); { 16Bit DMAchannel }
    if i>0 then
      begin
        u:=copy(s,i+1,1);
        val(u,dma16,er);
        if (er<>0) or (dma16<5) or (dma16>9) then dma16:=255;
        { it does not matter if there's no value }
      end;
    i:=pos('A',s); { DSPadress }
    if i>0 then
      begin
        u:=copy(s,i+1,3);
        val(u,dsp,er);
        dsp:=(dsp div 100)*256+ ((dsp div 10) mod 10)*16 + dsp mod 10;
        if (er=0) and (dsp div 256 = 2) and (((dsp mod 256) div 16) in [2,3,4,5,6,7,8]) then inc(count)
          else dsp:=$ffff;
      end;

    if count=4 then { we got all :) }
      forceto(typ,dma,dma16,irq,dsp)
    else exit; { was not enough detectable }
    UseBlasterEnv:=true;
  end;

procedure writelnSBConfig;
  begin
    writeln(#13#10' Soundblaster typ: ');
    case sbNo of
      0: writeln(' none ');
      1: writeln(' Soundblaster 1.0/1.5 (8 bit/mono-max 24kHz)');
      2: writeln(' Soundblaster Pro (8 bit/mono-max 44kHz/stereo-max 22kHz)');
      3: writeln(' Soundblaster 2.0/2.5/junior (8 bit/mono-max 44kHz)');
      4: writeln(' Soundblaster Pro3.0/PRO 4.0 (8 bit/mono-max 44kHz/stereo-max 22kHz)');
      5: writeln(' Soundblaster Pro<microchannel> (8 bit/mono-max 44kHz/stereo-max 22kHz)');
      6: writeln(' Soundblaster 16/16 ASP (8/16 bit/mono/stereo/max 45kHz)');
    end;
    write(#13#10' SB-Base : 2');write((dsp_addr div 16) mod 16);writeln('0h');
    writeln(' 8bit DMA : ',dma_channel);
    if SBNo=6 then writeln(' 16bit DMA : ',dma_16Bitchannel);
    writeln(' IRQ : ',IRQ_No,#13#10);
  end;

FUNCTION InputSoundblasterValues:Boolean;
var c:char;
  begin
    InputSoundblasterValues:=false;
    writeln(#13#10' Soundblaster typ ? ');
    writeln(' 0) none ');
    writeln(' 1) Soundblaster 1.0/1.5                   .....  8 bit mono   (max 24kHz)');
    writeln(' 2) Soundblaster 2.0/2.5/junior            .....  8 bit mono   (max 44kHz)');
    writeln(' 3) Soundblaster Pro/Pro3.0/PRO 4.0/micro  .....  8 bit stereo (max 22kHz)');
    writeln(' 4) Soundblaster 16/16 ASP                 ..... 16 bit stereo (max 45kHz)');
    repeat c:=readkey; until (c in ['0','1','2','3','4']);
    case c of
      '0': exit;
      '1': SBNo:=1;
      '2': SBNo:=3;
      '3': SBNo:=4; { also 2,5 }
      '4': SBNo:=6;
    end;
    writeln(#13#10' Soundblaster baseport 2X0h ?');
    write(' X = ');
    repeat c:=readkey; until (c in ['0'..'9']);
    writeln(c);
    dsp_addr:=$200+$10*(ord(c)-ord('0'));
    write(#13#10' 8 bit DMA channel (0,1,3) ? ');
    repeat c:=readkey; until (c in ['0','1','3']);
    writeln(c);
    dma_channel:=ord(c)-ord('0');
    if SBNo=6 then
      begin
        write(#13#10' 16 bit DMA (5,6,7) ? ');
        repeat c:=readkey; until (c in ['5','6','7']);
        writeln(c);
        DMA_16bitchannel:=ord(c)-ord('0');
      end;
    write(#13#10' IRQ (2,5,7) ? ');
    repeat c:=readkey; until (c in ['2','5','7']);
    writeln(c);
    IRQ_No:=ord(c)-ord('0');
    forceto(SBNo,dma_channel,dma_16bitchannel,IRQ_No,dsp_addr);
    InputSoundblasterValues:=true;
  end;

procedure check_samplerate(var rate:word;var stereo:boolean);
  begin
    stereo:=stereo and stereo_possible;
    if rate<4000 then rate:=4000;
    if stereo then
       begin
         if rate>maxstereorate then rate:=maxstereorate;
       end
    else
      if rate>maxmonorate then rate:=maxmonorate;
  end;

begin
  SB_Detect:=False;
  DSPIRQ_Detect:=false;
  DSPADR_Detect:=false;
  DMACHN_Detect:=False;
  MIXER_Detect:=False;
  stereo_possible:=false;
  _16Bit_possible:=false;
  STEREO:=False;_16Bit:=False;signeddata:=false;
  SBVersHi:=0;SBVersLo:=0;
  SBno:=0;
  IRQ_No:=7;
  DSP_Addr:=$220;
  DMA_Channel:=1;
  DMA_16Bitchannel:=5;
end.
