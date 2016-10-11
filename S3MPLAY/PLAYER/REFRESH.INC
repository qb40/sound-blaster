procedure write_Note(b:byte);
const nname:array[0..11] of string[2] = ('C-','C#','D-','D#','E-','F-',
                                         'F#','G-','G#','A-','A#','B-');
  begin
    if b<254 then write(nname[b and $0f],b shr 4)
        else if b=254 then write('^^.') else write('...');
  end;

procedure display_row(ordr,row:byte);
const hex:string[16] = '0123456789ABCDEF';
var pA:Parray;
    i,b:byte;
    w:word;
  begin
    w:=PATTERN[ORDER[ordr]];
    if w>=$c000 then
      pA:=ptr(FrameSEG[0],((w shr 8) and $3F)*patlength)
    else
      pA:=ptr(w,0);
    write(row:2,'   ');
    for i:=startchn-1 to startchn+3 do
      begin
        if (i+1-startchn) mod 2 = 0 then begin textbackground(black);textcolor(lightgray) end
        else begin textbackground(white);textcolor(black) end;
        if i<=usedchannels-1 then
          begin
            { write Note : }
            b:=pA^[row*usedchannels*5+i*5];
            write_Note(b);
            { write Instrument : }
            b:=pA^[row*usedchannels*5+i*5+1];
            write(' ',b div 10,b mod 10);
            { write volume : }
            b:=pA^[row*usedchannels*5+i*5+2];
            if b<255 then write(' ',b div 10,b mod 10) else write(' ..');
            { write special command+info: }
            b:=pA^[row*usedchannels*5+i*5+3];
            if b<255 then write(' ',chr(ord('A')+b-1)) else write(' .');
            b:=pA^[row*usedchannels*5+i*5+4];
            write(hex[b div 16+1],hex[b mod 16+1]);
            write(' ');
          end
        else
          write('              ');
      end;
  end;

procedure display_currow;
  begin
    display_row(curorder,curline);writeln;
  end;

function b2hs(b:byte):string;
const hexs:string='0123456789ABCDEF';
  begin
    b2hs:=hexs[b div 16+1] + hexs[b mod 16+1];
  end;

function w2hs(w:word):string;
  begin
    w2hs:=b2hs(w div 256)+b2hs(w mod 256);
  end;

function b2ds(b:byte):string;
var s:string;
  begin
    str(b:3,s);
    b2ds:=s;
  end;

{$I EFX_NAME.INC}

procedure refr_channels;
const types:array[0..4] of string[6] = ('OFF','Left','Right','Adlib1','Adlib2');
var i:byte;
    j:byte;
  begin
    gotoxy(1,8);
    textbackground(blue);textcolor(white);
    i:=0;j:=0;
    while i<usedchannels do
      begin
        if (channel[i].channeltyp<3) and
           ((usedchannels<17) or (channel[i].channeltyp>0)) then
            { display only sample channels }
          begin
            inc(j);
            write(' Chn ',(i+1):2,' (',types[channel[i].channeltyp]:5,') ');
            if channel[i].enabled then write('*') else write(' ');
            if channel[i].sloopflag then write('!') else write(' ');
            if channel[i].continueEf then write('c') else write(' ');
            write('(',channel[i].InstrNo:2,') ');
            write('[');
            if channel[i].enabled then
              begin
                write_note(channel[i].Note);
                { Arpeggio test : }
                {write(',');write_note(channel[i].Note1);write(',');write_note(channel[i].Note2);}
              end
            else write('...');
            write('] ');
            write('<',channel[i].sPeriod:5,'>');
            write(' ',channel[i].sStep.DD/64/1024:7:4);
            { Arpeggio test : }
            {write(' ',channel[i].Step1.DD/64/1024:7:4);
            write(' ',channel[i].Step2.DD/64/1024:7:4);}
            write(' ',channel[i].SampleVol:2);
            if channel[i].command<=44 then
            write(' ',get_effectname(i,channel[i].command div 2,channel[i].cmd2nd div 2,channel[i].parameter));clreol;
            write(#13#10);
          end;
        inc(i);
      end;
    if j=0 then writeln(' No digital channels to mix ...     :( ');
  end;

var lastrow:byte;

procedure refr_patterns;
var i,j:byte;
    w:word;
  begin
    window(1,9,80,25);textbackground(white);textcolor(black);
    if curline+1<lastrow then
      begin
        j:=prevorder(curorder);
        for i:=lastrow to 63 do
          begin
            gotoxy(1,17);writeln;gotoxy(1,17);display_row(j,i);
          end;
        lastrow:=0;
        { new pattern }
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
    for i:=lastrow to curline do
      begin
        gotoxy(1,17);writeln;gotoxy(1,17);display_row(curorder,i);
      end;
    lastrow:=curline+1;
    window(1,1,80,25);
  end;

var chnlastinst:array[0..max_channels-1] of byte;

procedure refr_inst;
var i:byte;
    n:byte;
  procedure write_no(j:word);
    begin
      gotoxy((j div 15)*11+1,j mod 15+1);
      write(j+1:2);
    end;
  begin
    textbackground(blue);textcolor(white);
    if useEMS then window(1,11,80,25) else window(1,10,80,25);
    for i:=0 to usedchannels-1 do
      begin
        n:=ord(channel[i].enabled)* channel[i].InstrNo*ord(channel[i].channeltyp>0);
        if (chnlastinst[i]<>n) and (chnlastinst[i]>0) then
          begin textcolor(white);textbackground(blue);write_no(chnlastinst[i]-1) end;
        chnlastinst[i]:=n;if n<>0 then begin textcolor(blue);textbackground(white);write_no(n-1) end;
      end;
    window(1,1,80,25);
  end;

var samplepage:byte;
    wassmp_scr:boolean;

procedure refr_sample;
var i,n:byte;
  begin
    for i:=0 to usedchannels-1 do
      begin
        n:=ord(channel[i].enabled)* channel[i].InstrNo*ord(channel[i].channeltyp>0);
        if (chnlastinst[i]<>n) and (chnlastinst[i]>0) and
           (samplepage*18<chnlastinst[i]) and (samplepage*18+19>chnlastinst[i]) then
          begin
            textcolor(white);textbackground(blue);
            gotoxy(1,7+chnlastinst[i]-samplepage*18);
            write(chnlastinst[i]:3,'.');
          end;
        chnlastinst[i]:=n;
        if (n<>0) and (samplepage*18<n) and (samplepage*18+19>n) then
          begin
            textcolor(blue);textbackground(white);
            gotoxy(1,7+chnlastinst[i]-samplepage*18);
            write(n:3,'.');
          end;
      end;
  end;

procedure refresh_scr;
  begin
    case screen_no of
      0: { Helpscreen (do not refrsh) };
      1: { channel view }     refr_channels;
      2: { pattern view }     refr_patterns;
      3: { Instrument infos } refr_inst;
      4: { sample infos }     refr_sample;
    end;
  end;