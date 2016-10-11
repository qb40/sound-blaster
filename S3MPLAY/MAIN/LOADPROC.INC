FUNCTION SCRMtest(var p):boolean; ASSEMBLER; { yeah asm ! :) }
  asm
     xor      ax,ax
     les      di,p
     cmp      word ptr es:[di],'CS'
     jne      @@endoftest
     cmp      word ptr es:[di+2],'MR'
     jne      @@endoftest
     mov      ax,0101h
@@endoftest:
  end;

FUNCTION ST3test(var p):boolean; ASSEMBLER; { I love this :) }
  asm
    xor       ax,ax
    les       di,p
    mov       ax,es:[di]
    cmp       ax,01300h         { saved by ST3.00 }
    jb        @@endoftest
    cmp       ax,01303h         { saved by ST3.01 }
    ja        @@endoftest
    mov       ax,0101h
@@endoftest:
  end;

procedure convert2pas(var from,topas;maxchars:byte); ASSEMBLER; { yeah assembler strikes again }
  asm
    push     ds
    lds      si,from
    les      di,topas
    mov      bx,di
    xor      ch,ch
    mov      cl,[maxchars]
    xor      dl,dl              ;{ count of chars in string }
    inc      di                 ;{ first char ... }
@@loop:
    lodsb                       ;{ I know it's slow, but here we don't need speed here ;) }
    test     al,al
    jz       @@nomorechar
    inc      dl                 ;{ copy now one char }
    stosb                       ;{ put it into the destination string }
    loop     @@loop
@@nomorechar:
    mov      es:[bx],dl         ;{ save count of chars }
    pop      ds
  end;

function getchtyp(b:byte):byte; ASSEMBLER; { :) what else ... }
  asm
    xor     ah,ah
    mov     al,b
    cmp     al,7
    ja      @@notleft
    mov     al,1        ;{ left }
    jmp     @@endofget
@@notleft:
    cmp     al,15
    ja      @@notright
    mov     al,2        ;{ right }
    jmp     @@endofget
@@notright:
    cmp     al,23
    ja      @@notadlib1
    mov     al,3        ;{ adlib melody }
    jmp     @@endofget
@@notadlib1:
    cmp     al,31
    ja      @@notadlib2
    mov     al,4        ;{ adlib drums }
    jmp     @@endofget
@@notadlib2:
    xor     al,al       ;{ channel off :) }
@@endofget:
  end;

FUNCTION  LOAD_S3M(name:string):BOOLEAN;
var f:file;
    header:Theader;
    maxused:byte;
    inspara:array[1..Max_samples] of word;
    patpara:TPatternSarray;
    smppara:ARRAY[0..MAX_samples] OF LONGINT;
    i:byte;
    inspos,patpos,smppos,smpnum:byte;
    nextins,nextpat,nextsmp:longint;
    fileposit:longint;
    wdummy:word;
    p:pointer;
    pAr:PArray;
    buffer:PArray;
    { EMS things: }
    Ppagesleft:byte;  { number of pages left to use for patterns }
    curPpage:byte;    { current logical EMS page we fill with next pattern }
    curpart:byte;     { =0,1,2 -> every page is seperated in 3 parts (one part - one pattern) }
    curSpage:word;    { current logical EMS page we fill with next sample }
    Spagesleft:word;  { number of pages left to use for samples }
    fun:string;
    funptr:pointer;

  PROCEDURE allocEMSforSamples;
  var w,w0:word;
      i:integer;
      pSmp:PSMPheader;
    begin
      if EMSfreepages=0 then begin EMSsmp:=false;exit end;
      w:=0;
      for i:=1 to 99 do
        begin
          pSmp:=addr(Instruments^[i]);
          if pSmp^.typ=1 then { really a sample }
            begin
              if pSmp^.flags and 1 = 1 then w0:=pSmp^.loopend+1024 else w0:=pSmp^.length+1024;
              w:=w + w0 div (16*1024) + ord(w0 mod (16*1024)>0);
            end;
        end;
      {$IFDEF BETATEST }
      writeln(' Instruments to load : ',insnum);
      writeln(' EMS pages are needed for Samples : ',w);
      {$ENDIF}
      { w = number of 16Kb pages in EMS }
      if w>EMSfreepages then { not enough EMS for all samples }
        begin
          { use as many pages as possible :) }
          w:=EMSfreepages;
          smpEMShandle:=EMSalloc(w);
        end
      else { oh well enough, that's nice }
        begin
          { fine let's load everything into EMS }
          smpEMShandle:=EMSalloc(w);
        end;
      {$IFDEF BETATEST }
      writeln(' EMS pages allocated for Samples : ',w);
      {$ENDIF}
      Spagesleft:=w;
      EMSsmp:=true;
      curSpage:=0;
    end;

  PROCEDURE freeallmem;
    begin
      if buffer<>Nil then freedosmem(buffer);
      done_module;
    end;

  PROCEDURE forget(count:longint);
  var dummy:array[0..511] of byte;
      i:word;
    begin
      for i:=1 to count div 512 do blockread(f,dummy,512);
      if count mod 512 >0 then blockread(f,dummy,(count mod 512));
    end;

  FUNCTION load_instrument:boolean;
  var length:word;
      typ:byte;
      pAr:Parray;
      Psmp:PSmpHeader;
      PAdl:PAdlHeader;
    BEGIN
      load_instrument:=false;
      { first jump to position }
      if (fileposit>nextins*16) then
        { shit tables not sorted - more disk access }
        begin
          reset(f,1);
          seek(f,nextins*16); { <- we start reading from filestart again
                                and read till we are at start of this pattern ... }
          if IOresult<>0 then begin load_error:=filecorrupt;exit end;
          {$IFDEF BETATEST}
          writeln(#13#10'somethings going wrong with order. position was: ',fileposit,' but we need : ',nextins*16);
          {$ENDIF}
        end
      else
      if fileposit<nextins*16 then
        { that's better - we only have to forget some blocks }
        forget(nextins*16-fileposit);
      fileposit:=nextins*16;
      {$IFDEF LOADINFO}
      write('I',inspos-1);
      {$ENDIF}
      { now read instrument header : }
      blockread(f,Instruments^[inspos-1],5*16);
      inc(fileposit,5*16);
      pSmp:=addr(instruments^[inspos-1]);
      pAdl:=addr(instruments^[inspos-1]);
      if pSmp^.typ=1 then { that instrument is a sample }
        begin
          if pSmp^.packinfo <> 0 then begin load_error:=packedsamples;exit end;
          { calc position in file : }
          smppara[smpnum]:=(longint(256*256)*pSmp^.HI_mempos+pSmp^.mempos);
          pSmp^.mempos:=0;inc(smpnum);
          {$IFDEF LOADINFO}
          write('!');
          {$ENDIF}
        end
      else
        begin
          smppara[smpnum]:=0;inc(smpnum);
          {$IFDEF LOADINFO}
          write('$');
          {$ENDIF}
        end;
      {$IFDEF LOADINFO}
      write('*');
      {$ENDIF}
      load_instrument:=true;
    END;

  FUNCTION load_sample:boolean;
  var p:pointer;
      par:parray;
      pSmp:pSmpHeader;
      z,h:word;
      i:byte;
      smplen:word;
    begin
      load_sample:=false;
      if (fileposit>nextsmp*16) then
        { shit tables not sorted - more disk access }
        begin
          reset(f,1);
          seek(f,nextsmp*16); { <- we start reading from filestart again
                                and read till we are at start of this pattern ... }
          if IOresult<>0 then begin load_error:=filecorrupt;exit end;
          {$IFDEF BETATEST}
          writeln(#13#10'somethings going wrong with order. position was: ',fileposit,' but we need : ',nextsmp*16);
          {$ENDIF}
        end
      else
      if fileposit<nextsmp*16 then forget(nextsmp*16-fileposit);

      fileposit:=nextsmp*16;
      pSmp:=addr(Instruments^[smppos]);
      if (pSmp^.flags and 1)=1 then smplen:=pSmp^.loopend else smplen:=pSmp^.length;
      if smplen>64511 then begin load_error:=sample2large;exit end;
      {$IFDEF LOADINFO}
      write('S',smppos,'(',smplen,')');
      {$ENDIF}
      z:=((smplen+1024) div (16*1024))+ord((smplen+1024) mod (16*1024)>0);
      if useEMS and EMSsmp and (Spagesleft>=z) then
        begin
          {$IFDEF LOADINFO}
          write('E(',curSpage,'-',curSpage+z-1,')');
          {$ENDIF}
          pSmp^.mempos:=$f000+curSpage; { and z-1 pages after }
          for i:=0 to z-1 do
            if not EMSmap(smpEMShandle,curSpage+i,i) then write('<EMS-ERROR>');
          inc(curSpage,z);
          blockread(f,frameptr[0]^,smplen);par:=frameptr[0];
        end
      else { we have to use normal memory (geeee) for this sample }
        begin
          if not getdosmem(p,smplen+1024) then begin load_error:=notenoughmem;exit end;
          blockread(f,p^,smplen);
          pSmp^.mempos:=seg(p^);
          par:=p;
        end;
      if (Psmp^.flags and 1)=1 then
        { if loop then copy from loopstart : }
        begin
          h:=1024;
          while h>0 do
            begin
              if h>psmp^.loopend-psmp^.loopbeg+1 then
                begin
                  move(par^[psmp^.loopbeg],par^[smplen+1024-h],psmp^.loopend-psmp^.loopbeg);
                  dec(h,psmp^.loopend-psmp^.loopbeg);
                end
              else
                begin
                  move(par^[psmp^.loopbeg],par^[smplen+1024-h],h);h:=0;
                end;
            end;
        end
      else fillchar(par^[smplen],1024,128);
      if (pSmp^.flags and 1 = 1) and (pSmp^.loopend<pSmp^.length) then
        forget(pSmp^.length-pSmp^.loopend);
      inc(fileposit,pSmp^.length);
      if IORESULT<>0 then begin write(' Geeee ... (',fileposit,')');load_error:=filecorrupt;exit end;
      {$IFDEF LOADINFO}
      write('*');
      {$ENDIF}
      load_sample:=true;
    end;

  FUNCTION load_decrunc_pattern:boolean;
  var row:byte;
      crunch:byte;
      chn:byte;
      hp,hp2:pointer;
      length:word;
      linecount:byte;
    BEGIN
      load_decrunc_pattern:=false;
      if nextpat=0 then begin load_decrunc_pattern:=true;PATTERN[patpos-1]:=0;exit end;
      { first jump to position }
      if (fileposit>nextpat*16) then
        { shit tables not sorted - more dsik access :( }
        begin
          reset(f,1);
          seek(f,nextpat*16); { <- we start reading from filestart again
                                   and read till we are at start of this pattern ... }
          {$IFDEF BETATEST}
          writeln(#13#10'somethings going wrong with order. position was: ',fileposit,' but we need : ',nextpat*16);
          {$ENDIF}
          if IOresult<>0 then begin load_error:=filecorrupt;exit end;
        end
      else
      if fileposit<nextpat*16 then
        forget(nextpat*16-fileposit);
      fileposit:=nextpat*16;
      blockread(f,length,2); { <- length of packed pattern }
      {$IFDEF LOADINFO}
      write('P',patpos-1,'(',length,')');
      if length>10*1024 then
        begin
          writeln('Packed data longer then 10K - that''s not allowed ...'#7' PROGRAM HALTED.');
          halt;
        end;
      {$ENDIF}
      { read whole packed pattern }
      blockread(f,buffer^,length-2); { length=sizeof(packdata)+(sizeof(length)=2) }
      if IOresult<>0 then begin load_error:=filecorrupt;exit end;
      inc(fileposit,length);
      { first get memory : (if useEMS than try to put it into the EMS ... }
      if useEMS and EMSpat and (curpart<patperpage) then
        begin
          PATTERN[patpos-1]:=$C000+256*curpart+curPpage;
          if not EMSmap(patEMShandle,curPpage,0) then write('<EMS-ERROR>');
          p:=ptr(frameseg[0]+(patlength div 16)*word(curpart),0);
        end
      else
        begin
          if not getdosmem(p,longint(64*5)*usedchannels) then begin load_error:=notenoughmem;exit end;
          PATTERN[patpos-1]:=seg(p^);
        end;
      { we decrunc it now to full size - not all 32 channels,but all used channels }
      hp:=p;hp2:=buffer;
      asm
        { first setup default values. It looks difficult, but it isn't :
          set note FFh,instrument 00,command ffh, options ffh }
        les      di,hp
        xor      ch,ch
        mov      cl,[usedchannels]
        shl      cx,6             ;{ do it for every channel and every row :
                                  ;  usedchannels * 64 }
@@loop: mov      word ptr es:[di  ],00ffh
        mov      word ptr es:[di+2],0ffffh
        mov      byte ptr es:[di+4],0
        add      di,5
        loop     @@loop
        ; { yo and now decrunch it ... }
        push     ds
        push     bp
        mov      al,[usedchannels]
        mov      dh,al
        les      di,hp          ;{ es:[di] ... pointer to destination }
        lds      si,hp2         ;{ ds:[si] ... pointer to packed data }

        xor      ah,ah
        mov      bp,ax
        shl      bp,2
        add      bp,ax          ;{ bp = usedchannels*5 = size of one row }

        mov      dl,64 ;{ 64 rows to decrunch }

@@rowloop:
        { read first 'crunch' byte for this channel : }
        lodsb                   ;{ I know "mov,inc" would be faster but we }
                                ;{ don't need speed here }
        cmp      al,0
        jz       @@endofrow
@@dloop:
        mov      cl,al
        xor      bh,bh
        mov      bl,cl
        and      bl,31          ;{ bl = channel to write to }
        cmp      bl,dh          ;{ bl<usedchannels }
        jae      @@overread
@@ok:   mov      ax,bx
        shl      bx,2
        add      bx,ax          ;{ bx = offset from row start to channel to write to }
        test     cl,32
        je       @@nonew_note_instrument
        lodsw
        mov      es:[di+bx],ax
@@nonew_note_instrument:
        test     cl,64
        jz       @@nonew_volume
        lodsb
        mov      es:[di+bx+2],al
@@nonew_volume:
        test     cl,128
        jz       @@nonew_cmd_info
        lodsw
        mov      es:[di+bx+3],ax
@@nonew_cmd_info:
        ;{ read next 'crunch' byte : }
        lodsb
        cmp      al,0
        jnz      @@dloop        ; { if zero then EOR is reached ... }
@@endofrow:  ;{ =EOR :) }
        add      di,bp                  ;{ to next row ...}
        dec      dl
        jnz      @@rowloop
        pop      bp
        pop      ds
        jmp      @@done
@@overread:
        test     cl,32
        je       @@ov1
        lodsw
@@ov1:
        test     cl,64
        jz       @@ov2
        lodsb
@@ov2:
        test     cl,128
        jz       @@ov3
        lodsw
@@ov3:  jmp      @@nonew_cmd_info
@@done:
      end;
      if pattern[patpos-1]>=$C000 then
        begin
          {$IFDEF LOADINFO}
          write('E(',curPpage,',',curpart,')');
          {$ENDIF}
          { next position in EMS : }
          inc(curpart);
          if (curpart=patperpage) and (Ppagesleft>0) then
            begin
              dec(Ppagesleft);inc(curPpage);
              curpart:=0;
            end;
        end;
      {$IFDEF LOADINFO}
      write('*');
      {$ENDIF}
      load_decrunc_pattern:=true;
    END;

  function fileexist(s:string):boolean;
  var f:file;
    begin
      assign(f,s);reset(f,1);fileexist:=ioresult=0;close(f);if ioresult<>0 then;
    end;

var a,b,c:string;
    Inst_done:boolean;
    load_smp_later:boolean;
    firstSMP:boolean;

  BEGIN
    LOAD_S3M := FALSE;
    useEMS:=EMSinstalled and useEMS and (EMSfreepages>1); { we need one page for saving mapping while playing }
    load_error:=0;buffer:=Nil;
    fsplit(name,a,b,c);
    if not fileexist(a+b+c) then name:=a+b+'.S3M';
    assign(f,name);
    reset(f,1);               { open file - 16byte blocks :) }
    IF IORESULT<>0 THEN begin load_error:=filenotexist;exit end;
    { First read fileheader }
    blockread(f,header,sizeof(THeader));
    IF IORESULT<>0 THEN begin load_error:=wrongformat;exit end;
    { check if it's really a S3M ... }
    IF header.filetyp<>16 then begin load_error:=wrongformat;exit end;
    IF not SCRMtest(header.SCRM_ID) then begin load_error:=wrongformat;exit end;
    IF not ST3test(header.CWTV) then begin load_error:=wrongformat;exit end;
    { set some variables : }
    convert2pas(header.name,songname,28);
    ordnum:=header.ordnum;
    insnum:=header.insnum;
    patnum:=header.patnum;
    { setup flags }
    asm
      mov        bx,[header.flags]
      { flag bit 0 }
      xor        al,al
      shr        bx,1
      rcl        al,1
      mov        [st2vibrato],al
      { flag bit 1 }
      xor        al,al
      shr        bx,1
      rcl        al,1
      mov        [st2tempo],al
      { flag bit 2 }
      xor        al,al
      shr        bx,1
      rcl        al,1
      mov        [amigaslides],al
      { flag bit 3 }
      xor        al,al
      shr        bx,1
      rcl        al,1
      mov        [vol0opti],al
      { flag bit 4 }
      xor        al,al
      shr        bx,1
      rcl        al,1
      mov        [amigalimits],al
      { flag bit 5 }
      xor        al,al
      shr        bx,1
      rcl        al,1
      mov        [SBfilter],al
      { flag bit 7 }
      xor        al,al
      shr        bx,2
      rcl        al,1
      mov        [costumeflag],al
    end;
    savedunder:=(header.cwtv shr 8) and $0f+0.1*((header.cwtv shr 4) and $0f+0.01*(header.cwtv and $0f));
    signeddata:=(header.ffv=1);if not (header.ffv in [1,2]) then begin load_error:=wrongformat;exit end;
    gvolume:=header.gvolume;
    mvolume:=header.mvolume and $7f;
    stereo :=(header.mvolume shr 7)=1;  { bit 7 is stereo flag ... }
    initspeed:=header.initialspeed;
    inittempo:=header.initialtempo;
    { setup channels : }
    maxused:=0;
    for i:=0 to 31 do
      begin
        channel[i].enabled:=(header.channelset[i] and 128=0);
        channel[i].channeltyp:=getchtyp(header.channelset[i] and 31);
        if channel[i].enabled and (channel[i].channeltyp>0) and (channel[i].channeltyp<3) then maxused:=i+1;
      end;
    usedchannels:=maxused;
    {$IFDEF BETATEST}
    writeln(' Used channels :',usedchannels);
    {$ENDIF}
    { now load arrangment : }
    blockread(f,Order,ordnum);
    IF IORESULT<>0 THEN begin load_error:=filecorrupt;exit end;
    { check order if there's one 'real' (playable) entry ... }
    i:=0;while (i<ordnum) and (order[i]>=254) do inc(i);
    if i=ordnum then begin load_error:=ordercorrupt;exit end; { playable entry not found :( }
    blockread(f,inspara,insnum*2);
    IF IORESULT<>0 THEN begin load_error:=filecorrupt;exit end;
    blockread(f,patpara,patnum*2);
    IF IORESULT<>0 THEN begin load_error:=filecorrupt;exit end;
    close(f);
    { Ok now the difficult part ...
      (load patterns/samples/instrumentdata)
      - load them in a row (don't jump through the file, that costs time !
      - problem is that you don't know the order and possibly there's no !
    }
    patlength:=5*64*usedchannels;
    {$IFDEF BETATEST}
    writeln(' length of Patterns in memory: ',patlength);
    {$ENDIF}
    if useEMS then
      begin
        { we use EMS, then we need a page to save mapping in interrupt ! }
        savHandle:=EMSalloc(1); { 1 page is enough ? }
        { let's continue with loading: }
        PatPerPage:=(16*1024) div patlength;
        {$IFDEF BETATEST}
        writeln(' Patterns per Page: ',patperpage);
        {$ENDIF}
        { try to allocate EMS for all patterns : }
        if (EMSfreepages<(patnum+(patperpage-1)) div patperpage) then
          begin
            Ppagesleft:=EMSfreepages;patEMShandle:=EMSalloc(Ppagesleft);EMSpat:=true;
          end
        else
          begin
            patEMShandle:=EMSalloc((patnum+(patperpage-1)) div patperpage);
            Ppagesleft:=(patnum+(patperpage-1)) div patperpage;EMSpat:=true
          end;
      end;
    if useEMS and EMSpat then
      begin
        curpart:=0;curPpage:=0;
      end;
    { clear all samples }
    fillchar(instruments^,max_samples*5*16,0);
    { Now try to load everything in a row }
    {$IFDEF LOADINFO}
    writeln(#10#13'load report :');
    {$ENDIF}
    reset(f,1);
    fileposit:=0; { at start :) }
    Inst_done:=false;  { Instrument are not loaded yet :) }
    load_smp_later:=false; { load instruments not later (up to now we can say only this) }
    firstSMP:=true; { if we load now an instrument, then it's the first =) }
    { init buffer for fast loading : }
    if not getdosmem(buffer,10*1024) then begin load_error:=notenoughmem;exit end;
    { init some variables for loading : }
    inspos:=1;patpos:=0;smppos:=0;smpnum:=0;nextpat:=$7fffffff;nextins:=$7fffffff;nextsmp:=$7fffffff;
    while (inspos<insnum+1) or (patpos<patnum) or (smppos<smpnum)
          or (nextpat<$7fffffff) or (nextins<$7fffffff) or (nextsmp<$7fffffff) do
      begin
        {writeln('--->',inspos,',',patpos,',',smppos);readkey;}
        if (nextpat=$7fffffff) and (patpos<patnum) then
          begin
            nextpat:=patpara[patpos];inc(patpos)
          end;
        if (nextins=$7fffffff) and (inspos<insnum+1) then
          begin
            nextins:=inspara[inspos];inc(inspos)
          end;
        if (nextsmp=$7fffffff) and (smppos<smpnum) then
          begin
            nextsmp:=smppara[smppos];inc(smppos)
          end;
        if (nextpat<nextins) and (nextpat<nextsmp) then
          begin
            { pattern }
            if (nextpat<$7fffffff) then
              if not load_decrunc_pattern then begin freeallmem;exit end;
            nextpat:=$7fffffff;
          end
        else
        if (nextins<nextsmp) then
          begin
            { instrument }
            if (nextins<$7fffffff) then
              if not load_instrument then begin freeallmem;exit end;
            nextins:=$7fffffff;inst_done:=(inspos=insnum+1);
          end
        else { sampledata }
          begin
            if (nextsmp>0) and not load_smp_later then
              begin
                if not Inst_done and useEMS then load_smp_later:=true
                { if all instruments are not loaded yet and we want to load into the EMS then
                stop loading here - do it after all Instruments are done ... }
                  else
                    begin
                      if useEMS and firstSMP then begin allocEMSforSamples;firstSMP:=false end;
                      if (nextsmp<$7fffffff) then
                      if not load_sample then begin freeallmem;exit end;
                    end;
              end;
            nextsmp:=$7fffffff;
          end;
        if keypressed then
          if readkey=#27 then
            begin
              writeln(' Somethings going wrong with loading ? Or why do you pressed <ESC> ?');
              writeln(' If loading error - please report me.');
              load_error:=internal_failure;
              freeallmem;
              exit;
            end;
      end;
    { And now for ugly orders :
      if instrumentdata was not fully loaded as the first sampledata starts,
      then we have to wait, coze we don't know how many EMS we should acolate
      now we know it so let's start again at the beginning of the file and
      load the samples in a row ... }
    if UseEMS and load_smp_later then
      begin
        reset(f,1);
        fileposit:=0; { again to start }
        allocEMSforSamples;
        smppos:=0;smpnum:=0;nextpat:=$7fffffff;nextins:=$7fffffff;nextsmp:=$7fffffff;
        while (smppos<smpnum) or (nextsmp<$7fffffff) do
          begin
            if (nextsmp=$7fffffff) and (smppos<smpnum) then
              begin
                nextsmp:=smppara[smppos];inc(smppos)
              end;
            if (nextsmp<$7fffffff) then
              if not load_sample then begin freeallmem;exit end;
            nextsmp:=$7fffffff;
          end;
      end;
    {$IFDEF BETATEST}
    writeln(#10);
    {$ENDIF}
    { free buffer : }
    freedosmem(buffer);
    { Just for fun set names for EMS handles (does only work for EMS>= v4.0) }
    if EMSversion>=4.0 then setEMSnames;
    S3M_inMemory:=true;
    LOAD_S3M :=TRUE;
  END;

FUNCTION load_specialdata(var p):boolean; BEGIN { not implemented } END;