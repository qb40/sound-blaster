procedure prep_channels;
  begin
    textcolor(white);textbackground(blue);
    window(1,8,80,25);clrscr;
    window(1,1,80,25);
  end;

procedure prep_patterns;
var w:word;
  begin
    textcolor(white);textbackground(black);
    window(1,8,80,25);clrscr;
    textbackground(white);textcolor(black);
    for w:=0 to 2 do begin gotoxy(w*28+6,1);write('    Chn ',2*w+startchn:2,'    '); end;
    textbackground(black);textcolor(white);
    for w:=0 to 1 do begin gotoxy(w*28+20,1);write('    Chn ',2*w+1+startchn:2,'    '); end;
    window(1,1,80,25);
    lastrow:=curline;
    w:=pattern[curpattern];
    if w>=$c000 then { switch EMSpage 0 to curpattern }
      asm
        mov         ax,w
        xor         bx,bx
        mov         ah,44h
        mov         bl,al
        xor         al,al
        mov         dx,[patEMShandle]
        int         67h
      end;
  end;

procedure prep_inst;
var i,l:word;
function posstring(b:byte):string;
var psmp:pSmpheader;
  begin
    psmp:=addr(Instruments^[b]);
    if psmp^.typ=1 then
      begin
        posstring:=w2hs(psmp^.mempos)+'h';
      end
    else
      begin
        posstring:='     ';
      end;
  end;
  begin
    textcolor(white);textbackground(blue);
    window(1,8,80,25);clrscr;
    writeln(' Sample positions in memory : ');
    l:=15;
    textcolor(yellow);
    if useEMS then begin write(#10' EMS used by samples :',getusedEMSsmp,' KB') end;
    textcolor(white);
    for i:=1 to l do
      begin
        write(#13#10,i:2,': ',posstring(i),'  ');
        write(i+l:2,': ',posstring(i+l),'  ');
        write(i+2*l,': ',posstring(i+2*l),'  ');
        write(i+3*l,': ',posstring(i+3*l),'  ');
        write(i+4*l,': ',posstring(i+4*l),'  ');
        write(i+5*l,': ',posstring(i+5*l),'  ');
        if i+6*l<100 then write(i+6*l,': ',posstring(i+6*l),'  ');
      end;
    textcolor(white);textbackground(blue);
    window(1,1,80,25);
  end;

function leftformat(s:string;b:byte):string;
var t:string;
    i:byte;
  begin
    t:=s;for i:=length(s) to b do t:=t+' ';
    t:=copy(t,1,b);
    leftformat:=t;
  end;

function hex2str(w:word):string;
const s:string[16] = '0123456789ABCDEF';
  begin
    hex2str:=s[w div 256 div 16+1]+s[w div 256 mod 16+1]+s[w mod 256 div 16+1]+s[w mod 16+1]+'h';
  end;

procedure prep_smp;
const what:array[false..true] of string[3] = ('OFF','ON ');
var i,j:byte;
    Psmp:PSmpHeader;
  begin
    textcolor(white);textbackground(blue);
    window(1,7,80,25);clrscr;
    textbackground(brown);textcolor(white);
    write('   #  Samplename                 Loop Beg   End   Len   C2Speed');clreol;
    gotoxy(1,2);
    textbackground(blue);
    if wassmp_scr then
      begin { only next page ... }
        inc(samplepage);
        i:=samplepage*18+1;
        if i>insnum then begin samplepage:=0;i:=1 end;
      end
    else
      begin
        i:=1; { start at instrument 1 }
        samplepage:=0;
      end;
    for j:=1 to 18 do
      if i<insnum+1 then
        begin
          psmp:=addr(Instruments^[i]);
          if j>1 then write(#10#13);
          if pSmp^.typ=1 then textcolor(white) else textcolor(lightgray);
          write(' ',i:2,'. ',leftformat(psmp^.name,28),' ',what[(psmp^.flags and 1) = 1],' ',
                hex2str(psmp^.loopbeg),' ',hex2str(psmp^.loopend),' ',hex2str(psmp^.length),' ',pSmp^.c2speed:5,'Hz');
          inc(i);
        end;
    window(1,1,80,25);
  end;

procedure prepare_scr;
  begin
    case screen_no of
      0: { help screen }      display_helpscreen;
      1: { channel view }     prep_channels;
      2: { pattern view }     prep_patterns;
      3: { Instrument infos } prep_inst;
      4: { sample infos }     prep_smp;
    end;
    wassmp_scr:=screen_no=4;
  end;
