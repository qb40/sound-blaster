{$I-,V-,I-}
{$M 8000,0,8000}
program voc2raw;

uses dos,crt;

type Tar = array[0..64*1024-2] of byte;
     Par = ^Tar;

var a,b:file;
    an,bn:string;
    l,p,c:longint;
    help:boolean;
    stereo:boolean;
    sign:boolean;
    _16bit:boolean;
    db:boolean;
    over:boolean;
    count:byte;
    i:byte;
    dat:Par;

  procedure getdosmem(var p;len:word); { max memory = 64*1024 in our case ... }
  var note:boolean;
    begin
      asm
         mov     [note],0
         les     di,p
         mov     bx,len
         shr     bx,4
         mov     ah,48h
         int     21h
         jc      @@ende
         mov     es:[di+2],ax
         xor     ax,ax
         mov     es:[di],ax
         mov     [note],1
@@ende:
      end;
      if not note then
        begin
          writeln(' Not enough free memory ! Program halted.');
          close(a);
          close(b);
          halt;
        end;
    end;

  procedure checkfilenames;
  var a,b,c:string;
    begin
      FSPLIT(an,a,b,c);  { a = path of VOCname
                           b = name of VOC (without extention)
                           c = extention }
      if c='' then an:=a+b+'.VOC';
      if db then c:='.INC' else c:='.RAW';
      if bn='' then bn:=a+b+c;
    end;

  procedure checkexist;
  var f:file;
      c:char;
      i:integer;
    begin
      assign(f,an);
      reset(f,1);
      i:=IOResult;
      if i=2 then
        begin
          writeln(' ',an,' does not exist ! Program halted.');
          halt;
        end;
      if i=3 then
        begin
          writeln(' path of ',an,' does not exist ! Program halted.');
          halt;
        end;
      close(f);
      assign(f,bn);
      reset(f,1);
      i:=IOResult;
      if i=3 then
        begin
          writeln(' path of ',bn,' does not exist ! Program halted.');
          halt;
        end;
      IF not over and (i=0) then
        begin
          writeln(' File ',bn,' does allready exist. Overwrite ? (Y/N) ');
          repeat c:=upcase(readkey) until c in ['Y','N'];
          if c='N' then
            begin
              writeln(' Don''t overwrite it ! Program halted.');
              halt;
            end
          else writeln(' Overwrite this file. ');
        end;
      close(f);
    end;

  function upstr(s:string):string;
  var i:byte;
      t:string;
    begin
      t:='';
      for i:=1 to length(s) do t:=t+upcase(s[i]);
      upstr:=t;
    end;

  procedure getparam(s:string);
    begin
      if (s='/H') or (s='/?') then help:=true;
      if (s='/16') then _16bit:=true;
      if (s='/S') then sign:=true;
      if (s='/R') then stereo:=true;
      if (s='/I') then db:=true;
      if (s='/O') then over:=true;
      if s[1]<>'/' then
        begin
          if count=0 then an:=s;
          if count=1 then bn:=s;
          inc(count);
        end;
    end;

  procedure writeRAW;
  label mono8,stereo8,mono16,stereo16,aftersave,ende;
  var by:byte;
      wo,wo1:word;
    begin
      assign(b,bn);
      rewrite(b,1);
      asm
        les     di,dat
        mov     cx,word ptr [l]
        mov     si,cx
        dec     si
@@loop: mov     al,es:[di]
        mov     ah,es:[si]
        { I know that code is ugly, but it works :) and I won't spend more
          time with it ... }

        cmp     [sign],0
        je      @@nosign
        sub     al,128
        sub     ah,128
@@nosign:
         cmp    [stereo],0
         je     @@mono
         mov    [wo],ax
         cmp    [_16Bit],0
         je     stereo8
         mov    dh,al
         xor    dl,dl
         xor    al,al
         mov    [wo],dx
         mov    [wo1],ax
         jmp    stereo16
@@mono:  cmp    [_16bit],0
         je     @@mono8
         mov    ah,al
         xor    al,al
         mov    [wo],ax
         jmp    mono16
@@mono8: mov    [by],al
         jmp    mono8
aftersave:
          inc   di
          dec   si
          loop  @@loop
          jmp   ende
      end;
mono8:
      asm
        push    es
        push    di
        push    cx
        push    si
      end;
      blockwrite(b,by,1);
      asm
        pop     si
        pop     cx
        pop     di
        pop     es
        jmp     aftersave
      end;
mono16:
      asm
        push    es
        push    di
        push    cx
        push    si
      end;
      blockwrite(b,wo,2);
      asm
        pop     si
        pop     cx
        pop     di
        pop     es
        jmp     aftersave
      end;
stereo8:
      asm
        push    es
        push    di
        push    cx
        push    si
      end;
      blockwrite(b,wo,2);
      asm
        pop     si
        pop     cx
        pop     di
        pop     es
        jmp     aftersave
      end;
stereo16:
      asm
        push    es
        push    di
        push    cx
        push    si
      end;
      blockwrite(b,wo,2);
      blockwrite(b,wo1,2);
      asm
        pop     si
        pop     cx
        pop     di
        pop     es
        jmp     aftersave
      end;
ende: close(b);
    end;

procedure writeINC;
var wo,w:word;
    by:byte;
    c:word;
    cas:byte;
    pos:byte;
    z:text;
procedure writeword(w:word);
  begin
    if pos=0 then
      begin
        write(z,#13#10'  dw ');pos:=10;
      end
    else write(z,',');
    write(z,w:5);dec(pos);
  end;

procedure writebyte(b:byte);
  begin
    if pos=0 then
      begin
        write(z,#13#10'  db ');pos:=16;
      end
    else write(z,',');
    write(z,b:3);dec(pos);
  end;

  begin
    pos:=0;
    assign(z,bn);
    rewrite(z);
    asm
      mov       al,1
      xor       ah,ah
      cmp       [sign],0
      je        @@nosign
      or        ah,al
@@nosign:
      shl       al,1
      cmp       [stereo],0
      je        @@nostereo
      or        ah,al
@@nostereo:
      shl       al,1
      cmp       [_16bit],0
      je        @@no16bit
      or        ah,al
@@no16bit:
       mov      [cas],ah
    end;
    case cas of
       4:{ 16bit yes ,stereo no ,sign no  }
         for w:=0 to l-1 do
           begin
             wo:=256*dat^[w];
             writeword(wo);
           end;
       6: { 16bit yes ,stereo yes ,sign no  }
          for w:=0 to l-1 do
           begin
             wo:=256*dat^[w];
             writeword(wo);
             wo:=256*dat^[l-w-1];
             writeword(wo);
           end;
       5: { 16bit yes ,stereo no ,sign yes  }
          for w:=0 to l-1 do
           begin
             wo:=256*(dat^[w]-128);
             writeword(wo);
           end;
       7: { 16bit yes ,stereo yes ,sign yes  }
          for w:=0 to l-1 do
           begin
             wo:=256*(dat^[w]-128);
             writeword(wo);
             wo:=256*(dat^[l-w-1]-128);
             writeword(wo);
           end;
       0: { 16bit no ,stereo no ,sign no  }
          for w:=0 to l-1 do
            writebyte(dat^[w]);
       2: { 16bit no ,stereo yes ,sign no  }
          for w:=0 to l-1 do
            begin
              by:=dat^[w];
              writebyte(by);
              by:=dat^[l-w-1];
              writebyte(by);
            end;
       1: { 16bit no ,stereo no ,sign yes  }
          for w:=0 to l-1 do
           begin
             by:=dat^[w]-128;
             writebyte(by);
           end;
       3: { 16bit no ,stereo yes ,sign yes  }
          for w:=0 to l-1 do
            begin
              by:=dat^[w]-128;
              writebyte(by);
              by:=dat^[l-w-1]-128;
              writebyte(by);
            end;
    end;
    close(z);
  end;

begin
  _16bit:=false;help:=false;stereo:=false;sign:=false;db:=false;count:=0;
  over:=false;an:='';bn:='';
  for i:=1 to paramcount do
    getparam(upstr(paramstr(i)));
  writeln(' ‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹ ');
  writeln(' € VOC2RAW.EXE   Copyright (c) 1994 by Andre'' Baresel € ');
  writeln(' ﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂ '#13#10);
  if help or (paramcount=0) then
    begin
      writeln(' Not enough parameters. I wanna help you :) ....'#13#10);
      writeln(' This is a li''l program to convert 8bit mono VOC files into RAW files.');
      writeln(' What are RAW files you''ll ask. It''s just the sampledata without any header.'#13#10);
      writeln(' What are the parameters ?');
      writeln('    /I convert to a INCfile');
      writeln('    /O no "Overwrite ? (Y/N)" question');
      writeln('    /16 convert the 8 bit data into 16 bit (samplevalues*256) ');
      writeln('    /S convert to signed data (values -128..127 or -32768..-32767 for 16bit)');
      writeln('    /R convert to stereo data (right channel reversed)'#13#10);
      writeln(' You can only convert good old 8bit mono VOCs with only one block, otherwise');
      writeln(' you''ll get some problems. Because VOC2RAW only cutoff the first 32 byte header');
      writeln(' and don''t check if there are other blocks - maximum size is 64 KB !');
      writeln(' Usage: ');
      writeln('     VOC2RAW <switch> VOCname<.VOC> <RAWname> ');
      writeln('        - if no RAWname is given program creats "VOCname.RAW" ');
      writeln('        - "*" is not allowed in VOCname (I was to lazy...)');
      writeln('        - Default is a 8 bit unsigned mono RAW as output file');
      write(' continue with any key ...');readkey;write(#13);clreol;
      writeln(' Why use this program ? ');
      writeln('    - as highlevel language (Pascal/C/Basic) programmer you can simply ');
      writeln('      load a RAWfile an play it, without the knowledge of the VOC format ');
      writeln('    - as ASM coder (what I prefer for speed reasons) you can use BINOBJ to');
      writeln('      convert the RAW file into an OBJ file and then you can link it to your');
      writeln('      code or you can include the INC-file (remember option /I) direct into');
      writeln('      your source code'#13#10);
      halt;
    end;
  checkfilenames;
  writeln(' VOC file read from : ',an);
  writeln(' RAW/INC file write to  : ',bn);
  if sign or stereo or _16bit or db then writeln('Options :');
  if sign then   writeln('  + signed');
  if stereo then writeln('  + stereo');
  if _16bit then writeln('  + 16 bit');
  if db then     writeln('  + creat a INC-file');
  if over then   writeln('  - no "overwrite" question');
  checkexist;
  if IORESULT<>0 then;
  assign(a,an);
  reset(a,1);
  if ioresult<>0 then exit;
  l:=filesize(a);p:=$19+7;
  if l<p then begin writeln(' File to short ! Program halted.');halt end;
  l:=l-p;if l>64*1024 then begin writeln(' File to large - convert only the first 64KB !');l:=64*1024 end;
  getdosmem(dat,64*1024-1);
  blockread(a,dat^,p);
  blockread(a,dat^,l);
  if db then writeINC else writeRAW;
  close(a);
end.
