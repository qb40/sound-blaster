{$M 16000,0,0}
{$I-,X+,V-,G+,D+}
unit s3mplay;

INTERFACE

CONST version              = 1.70;
      { Variable ranges }
      MAX_samples          = 100; { 0..99 samples }
      MAX_patterns         = 100; { 1..100 patterns }
      MAX_orders           = 255; { 0..255 orders }
      MAX_channels         =  32; { 0..31 channels }
      { error constants }
      noerror              = 0;
      notenoughmem         = -1;
      wrongformat          = -2;
      filecorrupt          = -3;
      filenotexist         = -4;
      packedsamples        = -5; { sorry I don't know about DigiPlay 3.0 ADPCM packing
                                   was anyway not used yet in S3Ms ... }
      Allreadyallocbuffers = -6; { don't try to allocate memory for buffers twice }
      nota386orhigher      = -7; { for playing any sound we need a 386 or higher -
                                   sorry but I optimized it for a 486 (pipeline etc.) and it
                                   runs fine on a 386 ;)
                                   (Hey guys a 486DX is not that expensive - for
                                   the same price I got an slow 386SX in 1991) }
      nosounddevice        = -8; { before 'start playing' - set a sounddevice ! }
      noS3Minmemory        = -9; { before 'start playing' - load a S3M ! }
      ordercorrupt         = -10; { if there's no playable entry in order -> that would cause an endless
                                    loop in readnotes if you try to play it }
      internal_failure     = -11; { I'm sorry if this happend :( }
      sample2large         = -12; { I can't handle samples >64511 }

{$I TYPDEF.INC}

{ variables for public }
VAR load_Error:integer;
    player_Error:integer;
    { Tables : }
    Instruments:^TInstrArray;         { pointer to data for all instruments }
    PATTERN   :TPatternSarray;        { segment for every pattern }
                                      { $Fxyy -> at EMS page YY on Offset X*5120 }
    ORDER     :TOrderArray;           { song arrangement }
    Channel   :TchannelArray;         { all public/private data for every channel }
    songname:string[28];              { name given by the musician }
    { numbers of ? }
    ordnum:word;
    insnum:word;
    Patnum:word;
    usedchannels:byte;  { possible values : 1..32 (kill all Adlib) }
    patlength   :word;    { length of one pattern }
    savedunder:real;    { ST version file was created with }
    { songposition : (you can change them while playing to jump arround) }
    curorder   :word;   { position in song arrangement }
    curpattern :byte;   { current pattern - is specified also by [curorder] - so it's only for the user ... }
    curline    :byte;   { current line in pattern }
    curtick    :byte;   { current tick  - we only calc one tick per call (look at MIXING.ASM) }
    lastorder  :byte;   { -> last order to play ... }
    Ploop_on   :boolean;{ in a Pattern loop? }
    Ploop_no   :byte;   { number of loops left }
    Ploop_to    :byte;   { position to loop to }
    patterndelay:byte;

    gVolume    :byte;    { global volume -> usedvol = instrvol*gvolume/255 }
    loopS3M    :boolean; { flag if restart if we reach the end of the S3M module }
    EndOfSong  :boolean;
    toslow     :boolean;
    justinfill :boolean;
    rastertime :boolean;
    useEMS     :boolean;
    FPS        :byte;     { frames per second ... default is about 70Hz }
    LQmode     :boolean;  { flag if lowquality mode }

    DMArealbufsize:array[0..63] of word; { e.g. 0,128,256,384 <- positions of dmabuffer parts (changes with samplerate) }

    TickBytesLeft:word;    { Bytes left to next Tick }

    {$IFDEF BETATEST}
    startorder :word;
    {$ENDIF}

    playbuffer :pointer; { pointer to DMAbuffer - for public use, but don't write into it !!!
                           - it's never used for any action while mixing !
                           - while playing you can read the DMA base counter
                           and find out in that way what sample value the
                           SB currently plays ... refer to DMA Controller }
    DMAhalf     :byte;     { last DMAbuffer part to calculate }
    numBuffers  :byte;     { number of parts in DMAbuffer }
    { EMS things : }
    patEMShandle :WORD;    { handle to access EMS for patterns }
    smpEMShandle :WORD;    { hanlde to access EMS for samples <- I seperated them, but that does not matter, well ? }
    savHandle    :WORD;    { EMS handle for saving mapping while playing }
    EMSpat       :boolean; { patterns in EMS ? }
    EMSsmp       :boolean; { samples in EMS ? }
    PatperPage   :byte;    { count of patterns per page (<64!!!) }

FUNCTION  load_s3m(name:string):BOOLEAN;        { load S3M module into memory }
PROCEDURE done_module;                          { free memory used by S3M }
FUNCTION  Init_device(input:byte):boolean;       { = false if set device failed }
FUNCTION  Init_S3Mplayer:boolean;                { init DMAbuffer,tickbuffer,volumetable and some variables }
PROCEDURE Done_S3Mplayer;                       { free buffers used by player }
PROCEDURE setSampleRate(var SR:word;stereo:boolean); { set SampleRate for playing mono/stereo - higher frequency
                                                           means more processor time for calc sound
                                                           stereo question is because possible stereo/mono rates may differ }
FUNCTION  startplaying(var A_stereo,A_16Bit:boolean;LQ:Boolean):Boolean;
                                                              (* play totaly in background - you have nothin else to do
                                                                 for continue playing !
                                                                 It'll interrupt your program itself and calculate
                                                                 the next data is required *)
procedure set_mastervolume(vol:byte);
procedure set_ST3order(new:boolean);             (* look at ST3order *)
{ To get some infos : }
function getspeed:byte;
function gettempo:byte;
function get_mvolume:byte;
function get_delay:byte;
function getSamplerate:word;
function getusedEMSsmp:longint;    { get size of samples in EMS }
function getusedEMSpat:longint;    { get size of patterns in EMS }

{ not supported functions: }
FUNCTION getuseddevice(var typ:byte;var base:word;var dma8,dma16:byte;var irq:byte):byte;
FUNCTION load_specialdata(var p):boolean; { allocate memory and load special data from file }

IMPLEMENTATION

uses EMStool,blaster,crt,dos;

CONST DMAbuffersize=8*1024; { <- maximum size of DMAbuffer }

{ Internal variables : }
VAR S3M_inMemory:BOOLEAN;
    PROC386:boolean;      { A 386 processor ? }
    filename:string;      { name of file currently in memory }
    buffersreserved:boolean;
    sounddevice :boolean;
    Samplerate  :word;
    Userate     :word;
    { mixing variables : }
    tickbuffer  :pointer;  { the well known buffer for one tick - size depends on _currennt_tempo_ }
    DMAbuffer   :pointer;  { DMA and SB loop inside ... and we copy data into that buffer }
    AllocBuffer :pointer;  { position where we allocate DMA buffer - remember that we may use second half ... }
    lastready   :byte;     { last ready calculated DMAbuffer part }
    volumetablePTR : pointer; { pointer to volumetable (see CALCVolumetable) }
    { S3M flags : }
    st2vibrato  :boolean; { not supported }
    st2tempo    :boolean; { not supported }
    amigaslides :boolean; { not supported }
    SBfilter    :boolean; { not supported }
    costumeflag :boolean; { not supported - set if costumedata }
    vol0opti    :boolean; { PSIs volume 0 optimization }
    amigalimits :boolean; { check for amiga limits }
    stereoflag  :boolean; { not supported - we do what's possible on detected SB }
    signeddata  :boolean; { signed/unsigned data (only volumetable differs in those modes) }
    { options : }
    mvolume      :byte;   { master volume -> calc posttables }
    initspeed    :byte;   { initial speed }
    inittempo    :byte;   { initial tempo }
    curspeed     :byte;   { current speed - length of one tick }
    curtempo     :byte;   { current tempo - count of ticks per note }
    { own Flags : }
    ST3order   :boolean; { if true then handle order like ST3 - if a "--"=255 is found -
                           stop or loop to the song start (look loopS3M) }
                         { if false - play the whole order and simply skip the "--"
                           if curorder=ordnum then stop or loop to the beginning }
    
    BPT          :word;   { bytes per tick - depends on samplerate + tempo }

    { some saved values for correct restoring former status : }
    oldexitproc  :pointer;
    { tables for mixing : }

    post8bit     :array[0..4095] of byte;
    post16bit    :array[0..4095] of word;

    sinuswave,
    rampwave     :array[0..63] of shortint;
    squarewave   :array[0..63] of byte;

{$L DOSPROC.OBJ}
function getdosmem(var p;anz:longint):boolean; external;
procedure freedosmem(var p); external;
function getfreesize:word; external;
function setsize(var p;anz:longint):boolean; external;

{$L EMS4FCT.OBJ}
procedure setEMSnames; near; external;

{$L READNOTE.OBJ}
procedure readnewnotes; near; external;
procedure SetupNewInst; near; external; { don't call it from pascal - has its internal use }
procedure SetnewNote;   near; external; { don't call it from pascal - has its internal use }

{$L MIXING.OBJ}
procedure calc_mono_tick; near; external;
procedure calc_stereo_tick; near; external;

{$L VOLUME.OBJ}
procedure calcVolumeTable; near; external;

{$L PROCESSO.OBJ}
function check386:boolean; near; external;

{$L FILLDMA.OBJ}
procedure fill_DMAbuffer; near; external;
procedure mixroutines; near; external;

{ getuseddevice is not implemented yet }
FUNCTION getuseddevice(var typ:byte;var base:word;var dma8,dma16:byte; var irq:byte):byte;
{ = 0 ... no device set / = 1 ... use SB mixing / > 1 ... other devices not supported yet }
{ typ ... up2now only SB typ - look at BLASTER.PAS }
begin end;

PROCEDURE done_module;
var i:word;
    p:pointer;
    psmp:PsmpHeader;
  BEGIN
    if not S3M_inMemory then exit;
    { Free samples & instruments : }
    for i:=1 to MAX_Samples do
      begin
        psmp:=addr(Instruments^[i]);
        if (psmp^.typ=1) then
          begin
            if psmp^.mempos<$f000 then { no EMS instrument }
              begin
                p:=ptr(psmp^.mempos,0);
                psmp^.mempos:=0;
                if p<>Nil then freedosmem(p);
              end;
          end;
        Instruments^[i,0]:=0;
      end;
    { Free patterns : }
    for i:=0 to MAX_patterns do
      begin
        if pattern[i]<$C000 then
          begin
            { pattern in normal memory - it's a shame :) }
            p:=ptr(PATTERN[i],0);
            if p<>Nil then freedosmem(p);
            Pattern[i]:=0;
          end;
      end;
    if EMSpat then { patterns in EMS }
      begin
        EMSfree(savHandle);
        EMSfree(patEMShandle);EMSpat:=false;
      end;
    if EMSsmp then { samples in EMS }
      begin
        EMSfree(smpEMShandle);EMSsmp:=false;
      end;
    S3M_inMemory:=false;
  END;

PROCEDURE Done_S3Mplayer;
  begin
    restore_irq;
    if volumetablePtr<>Nil then freeDOSmem(volumetableptr);
    if AllocBuffer<>Nil then freeDOSmem(AllocBuffer);
    if Tickbuffer<>Nil then freeDOSmem(TickBuffer);
    buffersreserved:=false;
    playbuffer:=Nil;
    DMABuffer:=Nil;
  end;

PROCEDURE NewExitRoutine; Far;
  begin
    stop_play; { halt SB :) }
    speaker_off; { switch it off ... }
    if S3M_inMemory then done_module;
    if buffersreserved then done_S3Mplayer else restore_irq;
    exitproc:=oldexitproc;
  end;

{$I LOADPROC.INC}

FUNCTION Init_device(input:byte):boolean;
{  input= 0 ... use settings in BLASTER unit
        = 1 ... hardware autodetect SB
        = 2 ... read blaster enviroment
        = 3 ... input by hand }
  begin
    Init_device:=false;
    if Input = 0 then { 'checkthem' not yet implemented } sounddevice:=true
    else
    if Input = 1 then Sounddevice:=DetectSoundblaster(true)
    else
    if Input = 2 then Sounddevice:=UseBlasterEnv
    else
    if Input = 3 then Sounddevice:=InputSoundblasterValues;
    Init_device:=Sounddevice;
  end;

function checkoverride(var p;l:word):boolean; assembler;
  asm
    mov     bx,1
    mov     ax,word ptr [p+2]
    rol     ax,4
    and     al,00fh
    add     ax,l
    jc      @@anoverride
    xor     bx,bx
@@anoverride:
    mov     ax,bx
  end;

FUNCTION Init_S3Mplayer:boolean;
var p:pArray;
  begin
    Init_S3Mplayer:=false;
    if not proc386 then begin player_error:=nota386orhigher;exit end;
    if buffersreserved then begin player_error:=Allreadyallocbuffers;Init_S3Mplayer:=true;exit end;
    { buffersreserved = false ! }
    if not getdosmem(volumetablePTR,65*256*2) then begin player_error:=notenoughmem;exit end;
    if not getdosmem(Allocbuffer,(DMABuffersize+15)*2) then begin player_error:=notenoughmem;exit end;
    { ok and now check for DMA page overrides }
    if checkoverride(Allocbuffer^,DMAbuffersize) then
      { it's a page override in first DMAbuffer - use second }
      begin
        { Can't free the first part - sorry it's not possible with a DOS function }
        { I know I can creat my own PSP etc., maybe later, ok ? - it's a problem  }
        { for final activities. }
        p:=allocBuffer;
        DMAbuffer:=ptr(seg(p^)+Dmabuffersize div 16,0);
        {$IFDEF BETATEST}
        write(' Use second part of DMAbuffer ... at ',seg(Dmabuffer^));
        {$ENDIF}
      end
    else
      begin
        { use first buffer and free the rest }
        {setsize(Allocbuffer,DMABuffersize);}
        DMAbuffer:=AllocBuffer;
        {$IFDEF BETATEST}
        write(' Use first part of DMAbuffer ... at ',seg(DMAbuffer^));
        {$ENDIF}
      end;
    {
      in tick buffer we calc one DMA buffer half - that are dmabuffersize/2 words
    }
    if not getdosmem(Tickbuffer,DMAbuffersize) then
      begin
        freedosmem(Allocbuffer);
        freedosmem(VolumetablePTR);
        player_error:=notenoughmem;
        exit
      end;
    playBuffer:=DMABuffer;
    buffersreserved:=true;
    { clear those buffers : }
    fillchar(dmabuffer^,dmabuffersize,0);
    fillchar(tickbuffer^,dmabuffersize,0);
    fillchar(volumetablePtr^,65*256*2,0);
    Init_S3Mplayer:=true;
  end;

PROCEDURE setSampleRate(var SR:word;stereo:boolean);
var w:word;
    i,j:byte;
  begin
    check_Samplerate(SR,stereo);Samplerate :=SR;

    if LQmode then
      Userate:=SR div 2
    else
      Userate:=SR;

    w:=(1+ord(stereo))*(trunc(1000000/(trunc(1000000/Userate))/FPS)+1);
    i:=DMAbuffersize div w;
    j:=1;while j<i do j:=j shl 1;j:=j shr 1;
    if LQmode then j:=j shr 1;
    for i:=0 to j-1 do
      dmarealbufsize[i]:=i*w;
    NumBuffers:=j;
  end;

procedure set_tempo(tempo:byte); far;
  begin
    if (tempo>=32) then
      begin
        curtempo:=tempo;
      end
    else tempo:=curtempo;
    if curtempo<>0 then BPT:=trunc(Userate/50*125/curtempo);
  end;

function getspeed:byte;
  begin
    getspeed:=curspeed;
  end;

function gettempo:byte;
  begin
    gettempo:=curtempo
  end;

var inside:boolean;

PROCEDURE PLAY_IRQ; interrupt;
var x,y:integer;
  begin
    asm
      cli
    @wait:
      cmp       [inside],1
      je        @wait
      mov       [inside],1
      { change DMAhalf: }
      mov       ah,[numbuffers]
      dec       ah
      inc       [DMAhalf]
      and       [DMAhalf],ah
      mov       [inside],0
    end;
    if rastertime then
      asm
        { set screen border, if user wants to know testing ... }
        mov             dx,03dah
        in              al,dx
        mov             dx,03c0h
        mov             al,31h
        out             dx,al
        mov             al,1
        out             dx,al
      end;
    asm
      { ackknowledge the interrupt on SB : }
      mov       dx,dsp_addr
      add       dx,0eh
      add       dl,[_16Bit]         { in 16Bit mode we have to ackknowledge 22f ;) }
      in        al,dx
      { ackknowledge the hardwareinterrupt : }
      mov       al,20h
      out       0A0h,al
      out       020h,al
      { now new hardware interrupts are allowed ! }
    end;
    fill_dmabuffer;
    if rastertime then
      asm
        { screen border back to black ... }
        mov             dx,03dah
        in              al,dx
        mov             dx,03c0h
        mov             al,31h
        out             dx,al
        mov             al,0
        out             dx,al
        sti
      end;
  end;

procedure calcposttable(use16bit:boolean);
var z,i:integer;
    a,b,c:real;
    p:pointer;
  begin
    if use16bit then
      begin { not implemented yet }
      end
    else
      begin
        z:=mvolume and 127;
        c:=256*127/z;
        a:=2048-c/2;
        b:=2048+c/2;
        for i:=0 to 4095 do
          begin
            if (i<a) then post8bit[i]:=0 else
            if (i>b) then post8bit[i]:=255 else
            post8bit[i]:=trunc((i-a)*z/128);
          end;
      end;
  end;

procedure Initchannels;
var i:byte;
  begin
    for i:=0 to usedchannels-1 do
      begin
        channel[i].VibTabOfs:=ofs(sinuswave);
        channel[i].TrmTabOfs:=ofs(sinuswave);
      end;
  end;

procedure set_mastervolume(vol:byte);
  begin
    if vol>127 then vol:=127;
    mvolume:=vol;
    calcposttable(_16bit);
  end;

function get_mvolume:byte;
  begin
    get_mvolume:=mvolume;
  end;

function get_delay:byte;
  begin
    get_delay:=patterndelay;
  end;

function getSamplerate:word;
  begin
    getSamplerate:=Samplerate;
  end;

function handlesize(h:word):word; assembler;
    asm
      mov       ah,4ch
      mov       dx,h
      int       67h
      cmp       ah,0
      jz        @@ok
      xor       bx,bx
@@ok: mov       ax,bx
    end;

function getusedEMSsmp:longint;    { get size of samples in EMS }
  begin
    if EMSsmp then getusedEMSsmp:=16*handlesize(smpEMShandle) else getusedEMSsmp:=0;
  end;

function getusedEMSpat:longint;    { get size of patterns in EMS }
  begin
    if EMSpat then getusedEMSpat:=16*handlesize(patEMShandle) else getusedEMSpat:=0;
  end;

procedure set_ST3order(new:boolean);
var i:byte;
  begin
    ST3order:=new;
    if new then
      begin
        { search for first '--' }
        i:=0;
        while (i<ordnum-1) and (order[i]<255) do inc(i);
        dec(i);
        lastorder:=i
      end
    else
      begin
        { just for fun (is not important,
          you can also do simply lastorder=ordnum-1 }
        i:=ordnum-1;
        while (i>0) and (order[i]>=254) do dec(i);
        lastorder:=i;
      end;
  end;

FUNCTION startplaying(var A_stereo,A_16Bit:boolean;LQ:Boolean):boolean;
var key:boolean;
    p:parray;
  begin
    startplaying:=false;
    player_error:=0;
    lqmode:=LQ;
    A_stereo:=A_Stereo and Stereo_possible;
    A_16Bit:=A_16Bit and _16Bit_possible;
    if not sounddevice then begin player_error:=nosounddevice;exit; end; { sorry no device was set }
    if not S3M_inMemory then begin player_error:=noS3Minmemory;exit end; { hmm load it first ;) }
    set_ready_irq(@play_irq);
    Initblaster(Samplerate,a_stereo,a_16Bit);
    setSamplerate(Samplerate,a_stereo);
    calcVolumetable; { <- now after loading we know if signed data or not }
    calcposttable(A_16bit);
    curtick:=1; { last tick -> goto next note ! }
    curLine:=0; { <- next line to read from }
    {$IFDEF BETATEST}
    curorder:=startorder;
    {$ELSE}
    curOrder:=0; { <- next order to read from }
    {$ENDIF}
    curpattern:=order[0]; { next pattern to read from }
    patterndelay:=0;      { no patterndelay at start of course ! }
    Ploop_on:=false;
    Ploop_to:=0;
    curspeed:=initspeed;set_tempo(inittempo);
    set_ST3order(ST3order); { <- don't remove this ! it's important ! (setup lastorder) }
    EndOfSong:=false;toslow:=false;
    TickBytesLeft:=0;       { emmidiately next tick }
    Initchannels;
    if lqmode then
      begin
        set_DMAvalues(DMABuffer,2*(numBuffers*DMArealbufsize[1]),true); { loop through whole DMAbuffer }

        DMAhalf:=numbuffers-1;
        lastready:=numbuffers;
        fill_dmabuffer; { calc all buffer parts ... }

        play_firstblock(2*dmarealbufsize[1]); { double buffering  }
      end
    else
      begin
        set_DMAvalues(DMABuffer,NumBuffers*DMArealbufsize[1],true); { loop through whole DMAbuffer }

        DMAhalf:=numbuffers-1;
        lastready:=numbuffers;
        fill_dmabuffer; { calc all buffer parts ... }

        play_firstblock(dmarealbufsize[1]); { double buffering  }
      end;
    { ok, now everything works in background ... }
    startplaying:=true;
  end;

VAR i:byte;

procedure calcwaves;
  begin
    for i:=0 to 63 do
      begin
        squarewave[i]:=255*ord(i<64);
        sinuswave[i] :=round(sin(pi/32*i)*(127));
        rampwave[i]  :=i*2-127;
      end;
  end;

BEGIN
  inside:=false;


  PROC386:=check386;
  calcwaves;
  buffersreserved:=false;
  sounddevice:=false;
  oldexitproc:=exitproc;
  exitproc:=@newExitRoutine;
  volumetablePTR:=Nil;
  DMAbuffer:=Nil;
  AllocBuffer:=Nil;
  playBuffer:=Nil;
  Tickbuffer:=Nil;
  Samplerate:=22000; { not the highest but nice sounding samplerate :) }
  Userate:=22000;
  loopS3M:=false;
  ST3order:=false;   { Ok let's hear all patterns are saved ... }
  rastertime:=false;
  useEMS:=EMSinstalled;      { more space for Modules ! }
  if not getdosmem(instruments,5*16*max_samples) then
    begin
      asm
        mov     ax,3
        int     10h
      end;
      writeln(' Hey S3M-Player needs some DOSmem (programmers info: lower PAS-heap !) ');
      halt(1);
    end;

  FOR i:=1 TO MAX_Samples DO
    BEGIN
      Instruments^[i,0]:=0;
    END;
  FOR i:=0 TO MAX_patterns-1 DO
    BEGIN
      PATTERN[i]:=0;
    END;
  FPS:=70;
  LQmode:=false;
END.