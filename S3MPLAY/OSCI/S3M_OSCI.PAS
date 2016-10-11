{$M 16000,0,1000}
program example_for_s3mplay;

uses S3MPlay,crt,blaster,dos;

const stereo_calc=true ;
      _16bit_calc=false;        { 16bit play not yet possible }

type Parray = ^TArray;
     TArray = array[0..10000] of byte;

var samplerate:word;
    Stereo:Boolean;
    _16bit:Boolean;
    filename:string;

  {$L LINES.OBJ}
  procedure linie(x1,y1,x2,y2:word;f:byte); external;

  procedure init;
    begin
      if not load_S3M(filename) then halt;
      writeln(' ''',songname,''' loaded ... (was saved with ST',savedunder:4:2,')');
      if not Init_S3Mplayer then halt;
      if not init_device(2) then begin writeln(' Blaster enviroment not found sorry ... ');halt end;
      setsamplerate(samplerate,stereo);
      set_ST3order(true);
      loopS3M:=true;
    end;

  procedure bar(o,b,l:word); assembler;
    asm
      mov      ax,0a000h
      mov      es,ax
      mov      di,[o]
      mov      bx,320
      mov      dx,[b]
      add      bx,dx
      push     bp
      mov      bp,[l]
      cmp      bp,0
      je       @@n
      shl      bp,1
      mov      ax,7
    @@b:
      mov      cx,dx
      rep stosb
      sub      di,bx
      dec      bp
      jnz      @@b
    @@n:
      pop      bp
      push     bp
      mov      bp,[l]
      neg      bp
      add      bp,64
      shl      bp,1
      mov      ax,1
      cmp      bp,0
      jz       @@n2
    @@b2:
      mov      cx,dx
      rep stosb
      sub      di,bx
      dec      bp
      jnz      @@b2
    @@n2:
      pop      bp
    @@e:
    end;

  procedure waitretrace; assembler;
    asm
      { waitraster : }
      cli
      mov     dx,03dah
@@raster1:
      in      al,dx
      and     al,8
      jz      @@raster1
@@raster2:
      in      al,dx
      and     al,8
      jnz     @@raster2
      sti
    end;

var pos:word;
    i:word;
    h:Parray;
    scr:array[0..319] of byte;
    scr2:array[0..319] of byte;
    b:byte;
    yl,yr:integer;
    drawseg:word;

begin
  { setup defaults: }
  Samplerate:=45454;drawseg:=$a000;
  Stereo:=stereo_calc;
  _16bit:=_16bit_calc;
  filename:=paramstr(1);
  { end of default ... }
  textbackground(black);textcolor(lightgray);
  clrscr;
  writeln(' OSCI-S3M-PLAYER for SoundBlasters written by Cyder of Green Apple');
  writeln(' ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~');
  writeln(' Version : ',version:3:2);
  if (filename='') then halt;
  writeln;
  Init;
  if not startplaying(stereo,_16bit,false) then halt;
  writeln(#13#10' dmabuflength :',DMArealbufsize[1]);
  writeln(#13#10' Stop playing and exit with <ESC> ');
  writeln('press any key to switch to oscillator ... ');
  readkey;
  asm
    mov        ax,13h
    int        10h
    { Fill the screen with blue (in standard palette) : }
    mov        ax,0a000h
    mov        es,ax
    xor        di,di
    mov        cx,32000
    mov        ax,0101h
    rep stosw
  end;
  { DIsplay Oscilator : }
  if not stereo then
    begin
      h:=playbuffer;
      while not keypressed do
        begin
          waitretrace;
          for pos:=0 to usedchannels-1 do
            bar(320*170+pos*15+10,10,channel[pos].sampleVol*ord(channel[pos].enabled));
          yl:=h^[get_zaehler] shr 1;
          for pos:=1 to 319 do
            begin
              i:=get_zaehler; { current position in DMAbuffer }
              linie(pos-1,scr[pos-1],pos,scr[pos],1);
              scr[pos-1]:=yl;yl:=h^[i] shr 1;
              linie(pos-1,scr[pos-1],pos,yl,14);
            end;
          scr[319]:=yl;
        end;
    end
  else { in stereo mode : }
    begin
      h:=playbuffer;
      while not keypressed do
        begin
          for pos:=0 to usedchannels-1 do
            bar(320*170+pos*15+10,10,channel[pos].sampleVol*ord(channel[pos].enabled));
          i:=get_zaehler and $fffe;
          yl:=h^[i] shr 2;yr:=h^[i+1] shr 2;
          for pos:=1 to 319 do
            begin
              i:=get_zaehler and $fffe; { current position in DMAbuffer }
              if i>dmarealbufsize[1] then b:=7 else b:=4;
              { left channel : }
              linie(pos-1,36+scr[pos-1],pos,36+scr[pos],1);
              scr[pos-1]:=yl;yl:=h^[i] shr 2;
              linie(pos-1,36+scr[pos-1],pos,36+yl,14);
              { right channel : }
              linie(pos-1,110+scr2[pos-1],pos,110+scr2[pos],1);
              scr2[pos-1]:=yr;yr:=h^[i+1] shr 2;
              linie(pos-1,110+scr2[pos-1],pos,110+yr,14);
            end;
          scr[319]:=yl;
          scr2[319]:=yr;
        end;
    end;
  asm
    mov        ax,3
    int        10h
  end;
  stop_play;
  done_module;
  done_S3Mplayer;
end.