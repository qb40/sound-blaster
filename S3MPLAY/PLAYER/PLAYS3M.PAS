{$M 16000,0,2000}
program example_for_s3mplay;

uses emstool,S3MPlay,crt,blaster,dos;

const stereo_calc=true;
      _16bit_calc=false;
      switch:array[false..true] of string[3] = ('off','on ');

var samplerate:word;
    Stereo:Boolean;
    _16bit:Boolean;
    _LQ:boolean;
    ST3order:Boolean;
    help:boolean;
    volume:byte;
    how2input:byte; { 1-autodetect,2-read blaster enviroment,3-input by hand }
    disply_c:boolean;
    screen_no:byte;  { current info on screen }
    startchn:byte;

{$L DOSPROC.OBJ}
function getfreesize:word; external;

function tohexs(w:word):string;
const s:string='0123456789ABCDEF';
  begin
    tohexs:=s[(w shr 12)+1] + s[((w shr 8) and $0f)+1] + s[(w and $00ff) shr 4+1] + s[(w and $000f)+1];
  end;

procedure display_errormsg(err:integer);
  begin
    { I know case is stupid - like my code allways is :) }
    case err of
       0: write(' Hmm no error what''s wrong ? ');
      -1: begin
          if load_error=-1 then write(' Not enough memory for this module. ') else
          if player_error=-1 then write(' Not enough memory for internal buffers. ');
          write('PROGRAMMERS INFO: Try to lower PascalHeap or DMAbuffer. ');
          end;
      -2: write(' Wrong file format. Not a S3M ? ');
      -3: write(' File corrupt. ');
      -4: write(' File does not exist. ');
      -7: write(' Need a 386 or higher. ');
      -8: write(' No sounddevice set. (wrong code - shame on you programmer) ');
      -11: write(' Loading stoped by user <- only for betatest ! ');
    else write(' Somethings going wrong, but I dounno about that errorcode: ',err,'  ');
    end;
    writeln('PROGRAM HALTED.'#7);
    halt;
  end;

var filename:string;
    c:char;
    savchn:array[0..15] of byte;

procedure save_chntyps;
var i:byte;
  begin
    for i:=0 to 15 do savchn[i]:=channel[i].channeltyp;
  end;

procedure revers(n:byte);
  begin
    if channel[n].channeltyp=0 then channel[n].channeltyp:=savchn[n]
    else channel[n].channeltyp:=0
  end;

procedure hide_cursor; assembler;
 asm
   mov  ah,01
   mov  cx,32*256+32
   int  10h
 end;

procedure view_cursor; assembler;
 asm
   mov  ah,01
   mov  cx,15*256+16
   int  10h
 end;

var oldexit:pointer;

procedure local_exit; far;
  begin
    exitproc:=oldexit;
  end;

function nextord(nr:byte):byte;
  begin
    patterndelay:=0;Ploop_on:=false;Ploop_no:=0;Ploop_to:=0;
    inc(nr);
    while (nr<=lastorder) and (order[nr]>=254) do inc(nr);
    if nr>lastorder then
      if loopS3M then
        begin
          nr:=0;
          while (nr<=lastorder) and (order[nr]>=254) do inc(nr);
          if nr>lastorder then EndofSong:=true; { stupid order ! (no real entry) }
        end
      else begin nr:=0;EndofSong:=true end;
    nextord:=nr;
  end;

procedure disable_all;
var i:byte;
  begin
    for i:=0 to usedchannels-1 do
      channel[i].enabled:=false;    { <- use this if you jump to previous order ... }
  end;

function prevorder(nr:byte):byte;
  begin
    if nr=0 then begin prevorder:=nr;exit end;
    dec(nr);
    while (nr>0) and (order[nr]>=254) do dec(nr);
    if order[nr]>=254 then { to far - search next playable }
      begin
        while (nr<=lastorder) and (order[nr]>=254) do inc(nr);
        if nr>lastorder then EndofSong:=true; { stupid order ! (no real entry) }
      end;
    prevorder:=nr;
  end;

function upstr(s:string):string;
var i:byte;
  begin
    for i:=1 to length(s) do s[i]:=upcase(s[i]);
    upstr:=s;
  end;

procedure check_para(p:string);
var t:string;
    b:byte;
    w:word;
    i:integer;
  begin
    if (p[1]<>'-') and (p[1]<>'/') then
      begin
        filename:=p;
        exit;
      end;
    if upcase(p[2])='V' then { Volume }
      begin
        t:=copy(p,3,length(p)-2);
        val(t,b,i);
        if i=0 then volume:=b;
      end;
    if upcase(p[2])='S' then { Samplerate }
      begin
        t:=copy(p,3,length(p)-2);
        val(t,w,i);
        if i=0 then
          begin
            if w<100 then w:=w*1000;
            SampleRate:=w;
          end;
      end;
    if (upcase(p[2])='H') or (p[2]='?') then { help } help:=true;
    if upcase(p[2])='M' then { Mono - because default is stereo } stereo:=false;
    if p[2]='8' then { 8bit - default is 16bit } _16bit:=false;
    if upcase(p[2])='C' then { display SB config } disply_c:=true;
    if upcase(p[2])='R' then { show rastertime } rastertime:=true;
    if upcase(p[2])='O' then { use ST3 order } ST3order:=true;
    if upstr(copy(p,2,5))='NOEMS' then { don't use EMS } useEMS:=false;
    if upstr(copy(p,2,3))='ENV' then { read Blaster enviroment } how2input:=2;
    if upstr(copy(p,2,3))='CFG' then { input SB config by hand } how2input:=3;
    if upstr(copy(p,2,2))='LQ' then { mix in low quality mode } _LQ:=true;
    {$IFDEF BETATEST}
    if upcase(p[2])='B' then
      begin
        t:=copy(p,3,length(p));
        val(t,b,i);
        if i=0 then startorder:=b;
      end;
    if upcase(p[2])='F' then { set frame rate }
      begin
        t:=copy(p,3,length(p)-2);
        val(t,b,i);
        if i=0 then FPS:=b;
      end;
    {$ENDIF}
  end;

procedure display_keys;
  begin
    writeln(' Keys while playing : '#13#10);
    writeln(' <P> ... Pause (only on SB16)');
    writeln(' <L> ... enable/disable loopflag');
    writeln(' <D> ... doshelling :)');
    writeln(' <Alt> <1>..<''>,<Q>..<R> - Switch On/Off channel 1..16 ');
    writeln(' <+> ... Jump to next pattern');
    writeln(' <-> ... Jump to previous pattern');
    writeln(' <ESC> ... Stop playing');
    writeln(' <F1> ... help screen');
    writeln(' <F2> ... Display channel infos');
    writeln(' <F3> ... Display current pattern');
    writeln(' <F4> ... Display instrument infos');
    writeln(' <F5> ... Display sample memory positions');
  end;

procedure display_help;
  begin
    writeln(' Usage :');
    writeln('  PLAYS3M <options> <S3M Filename> '#13#10);
    writeln('    þ Order does not matter');
    writeln('    þ if no extension then ''.S3M'' is added');
    writeln('    þ Options:  (use prefixes ''/'' or ''-'' to mark it as option)');
    writeln('         /Vxxx    ... set master volume 0..255 ');
    writeln('                      (default=0 - use master volume is specified in S3M)');
    writeln('         /Sxxxxx  ... set samplerate ''4000...45454'' or ''4..46''(*1000)');
    writeln('                      (higher SampleRate -> better quality !)');
    writeln('         /H or /? ... Show this screen ');
    writeln('                      (funny eh - yo you get it easier with no parameter)');
    writeln('         /M       ... use mono mixing');
    writeln('                      (default is stereo if it''s possible on your SB)');
    writeln('         /8       ... use 8bit mixing');
    writeln('                      (default is 16bit if it''s possible on your SB)');
    writeln('         /C       ... display configuration after detecting');
    writeln('                      (default is display not)');
    writeln('         /ENV     ... use informations of blaster envirment');
    writeln('         /CFG     ... input SB config by hand');
    writeln('                      (default is SB hardware autodetect)');
    write(' a key for next page ...');
    readkey;
    write(#13);clreol;
    writeln('         /O       ... handle order like ST3 does');
    writeln('                      (default is my own way - play ALL patterns are defined');
    writeln('                      in Order)');
    writeln('         /R       ... display raster time');
    writeln('         /NOEMS   ... don''t use EMS for playing (player won''t use any EMS ');
    writeln('                      after this) - if there''s no free EMS, player''ll set');
    writeln('                      also <don''t use EMS>');

    writeln('         /LQ      ... use low quality mode');
    {$IFDEF BETATEST}
    writeln(' for debugging: ');
    writeln('         /Bxx     ... start at order xx (default is 0)');
    writeln('         /Fxx     ... set Frames Per Second (default is 70Hz)');
    {$ENDIF}
    if not help then writeln('Gimme a filename :)');
    halt(1);
  end;

procedure display_playercfg;
  begin
    writelnSBconfig;
  end;

procedure display_helpscreen;
  begin
    textcolor(white);textbackground(blue);
    window(1,8,80,25);clrscr;
    writeln;
    display_keys;
    window(1,1,80,25);
  end;

function getfreeEMS:longint;
var Regs : Registers;
begin
  getfreeEMS:=0;
  if not EMSinstalled then exit;
  Regs.ah := $42;                { Fkt.no.: get number of free pages }
  Intr($67, Regs);
  if (Regs.ah <>0 ) then exit    { something was not right ... :( }
  else getfreeEMS := Regs.bx;
end;

procedure mainscreen;
CONST SW_order:array[false..true] of string = ('Extended Order','Normal Order');
      SW_stereo:array[false..true] of string = ('Mono','Stereo');
      SW_qual:array[false..true] of string = ('Hiquality','Lowquality');
      sw_res:array[false..true] of string = ('8bit','16bit');
  begin
    textbackground(blue);window(1,1,80,25);clrscr;
    gotoxy(1,7);textbackground(yellow);clreol;writeln('Channel  Stereo ELC Inst Note  Period  Step  Vol Effect');
    textbackground(white);textcolor(black);
    gotoxy(1,1);clreol;write('Order:   (  ) Row:    Tick:                  that is Pattern:    ');
    textbackground(green);textcolor(black);gotoxy(1,6);clreol;write(' Title: ',songname);
    gotoxy(50,6);write('EMS usage: ',switch[useEMS],' Loop S3M : ');
    textbackground(blue);textcolor(lightgray);
    gotoxy(1,3);write(' Samplerate: ',getSamplerate:5,'  ',sw_stereo[stereo],', ',sw_res[_16bit],
    ', ',sw_order[ST3order],', ',sw_qual[LQmode]);
    gotoxy(1,4);write(' Free DOS memory : ',longint(16)*getfreesize:6,' bytes  Free EMS memory : ',getfreeEMS*16:5,' KB');
    gotoxy(1,5);write(' Used EMS Memory : ',(getusedEMSsmp+getusedEMSpat):5,' KB  <F1> - Help screen',
                      '':13,'Playerversion: ',version:3:2);
  end;

procedure refr_mainscr;
  begin
    textbackground(white);textcolor(black);
    gotoxy(8,1);write(curOrder:2);
    gotoxy(11,1);write(lastorder:2);
    gotoxy(20,1);write(curline:2);
    gotoxy(29,1);write(curtick:2);
    gotoxy(63,1);write(curpattern:2,' (',tohexs(pattern[curpattern]),')');
    textbackground(green);textcolor(black);
    gotoxy(76,6);write(switch[loopS3M]);
    gotoxy(1,2);
    textbackground(magenta);textcolor(yellow);
    write(' Speed: ',getspeed:3,' '#179' Tempo: ',gettempo:3,' '#179' GVol: ',
          gvolume:2,' '#179' MVol: ',get_mvolume:3,' '#179' Pdelay: ',get_delay:2,' '#179' Ploop: ');
    if Ploop_on then write(Ploop_to,'(',PLoop_no,')') else write(Ploop_to);
    clreol;
  end;

{$I REFRESH.INC}  { refresh the different screens }
{$I PREPARE.INC}  { prepare the different screens }

var i:byte;

begin
  { setup defaults: }
  Samplerate:=45454;
  Stereo:=stereo_calc;
  _16bit:=_16bit_calc;
  _LQ:=false;
  help:=false;
  volume:=0; { use volume given in S3M ... }
  how2input:=1; { autodetect SB }
  disply_c:=false;
  filename:='';
  ST3order:=false;
  {$IFDEF BETATEST}
  startorder:=0;
  {$ENDIF}
  { end of default ... }
  textbackground(black);textcolor(lightgray);
  oldexit:=exitproc;
  exitproc:=@local_exit;
  for i:=1 to paramcount do
    check_para(paramstr(i));
  clrscr;
  writeln(' S3M-PLAYER for SoundBlasters written by Cyder of Green Apple (Andre'' Baresel) ');
  writeln(' ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~');
  writeln(' Version : ',version:3:2);
  if (filename='') then display_help;
  writeln;
  {$IFDEF BETATEST}
  writeln(' Free memory before loading : ',longint(16)*getfreesize);
  writeln(' Free EMS memory before loading :',getfreeEMS*16,' KB');
  {$ENDIF}
  if not load_S3M(filename) then display_errormsg(load_error);
  {$IFDEF BETATEST}
  writeln(' Free memory after loading : ',longint(16)*getfreesize);
  writeln(' Free EMS after loading : ',getfreeEMS*16,' KB');
  {$ENDIF}
  writeln(' ''',songname,''' loaded ... (was saved with ST',savedunder:4:2,')');
  if not Init_S3Mplayer then display_errormsg(player_error);
  {$IFDEF BETATEST} writeln(' player init done ... ');
  display_keys;
  write(#13#10' press a key to continue...');readkey;clrscr;gotoxy(1,19);{$ENDIF}
  if not init_device(how2input) then begin writeln(' SoundBlaster not found sorry ... ');halt end;
  {$IFDEF BETATEST} writeln(' init device (SB) done ... '); {$ENDIF}
  if disply_c then
    begin
      display_playercfg;
      write(#13#10' press a key to continue...');readkey;clrscr;gotoxy(1,19);
    end;
  { And here we go :) }
  if volume>0 then set_mastervolume(volume);
  setsamplerate(samplerate,stereo);
  set_ST3order(ST3order);
  save_chntyps;
  loopS3M:=true;
  screen_no:=1;startchn:=1;
  if not startplaying(stereo,_16bit,_LQ) then display_errormsg(player_error);
  mainscreen;
  hide_cursor;
  repeat
    c:=#0;
    refr_mainscr;
    refresh_scr;
    if keypressed then c:=readkey;
    {if c<>#0 then write(ord(c));}
    if (c>='x') and (c<=chr(ord('x')+16)) then begin revers(ord(c)-ord('x'));c:=#0 end;
    if (ord(c)>=16) and (ord(c)<=19) then begin revers(ord(c)-4);c:=#0 end;
    if (c>=#59) { F1 } and (c<=#63) { F5 } then
      begin
        screen_no:=ord(c)-59;
        prepare_scr;c:=#0;
      end;
    if (upcase(c)='P') then
      begin
        pause_play;
        readkey;
        continue_play;
        c:=#0;
      end;
    if (c='+') then
        begin
          curorder:=nextord(curorder);
          lastrow:=0;curline:=0;curtick:=1;curpattern:=order[curorder];c:=#0
        end;
    if (c='-') then
        begin
          curorder:=prevorder(curorder);
          patterndelay:=0;Ploop_on:=false;Ploop_no:=0;Ploop_to:=0;
          disable_all;
          lastrow:=0;curline:=0;curtick:=1;curpattern:=order[curorder];c:=#0
        end;
    if upcase(c)='L' then loopS3M:=not loopS3M;
    if upcase(c)='D' then
      begin
        asm
          mov ax,3
          int 10h    { clear screen }
        end;
        writeln(' Return to player with ''EXIT'' ... ');
        swapvectors;
        exec(getenv('COMSPEC'),'');
        swapvectors;
        c:=#0;
        asm
          mov ax,3
          int 10h
        end;
        hide_cursor;
        if doserror<>0 then
          begin
            while keypressed do readkey;
            writeln(' Doserror ',doserror);
            writeln(' Hmm somethings going wrong with running a copy of COMMAND.COM ...');
            writeln(' press any key to continue ... ');
            readkey;
          end;
        mainscreen;
      end;
    if (c=#77) and (startchn<usedchannels) then begin inc(startchn);if screen_no=2 then prepare_scr; end;
    if (c=#75) and (startchn>1) then begin dec(startchn);if screen_no=2 then prepare_scr; end;
  until toslow or (c=#27) or (EndOfSong);
  if toslow then writeln(' Sorry your PC is to slow ... ');
  view_cursor;
  stop_play;
  done_module;
  done_S3Mplayer;
  gotoxy(1,8);
  textcolor(white);textbackground(blue);
  {$IFDEF BETATEST}
  writeln(' Memory after all : ',longint(16)*getfreesize);clreol;
  writeln(' EMS after all : ',getfreeEMS*16,' KB');clreol;
  {$ENDIF}
end.