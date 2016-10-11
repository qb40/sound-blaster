{$G+}
unit EMStool;

interface

Type PHandle=^THandle;
     THandle=record handleNo:word;
                    next:PHandle;
                  end;

var FrameSEG:array[0..3] of Word; { real mem segment for every page }
    FramePTR:array[0..3] of pointer; { real mem pointer (ofs=0) for every page }
    EMSinstalled:boolean;
    EMSversion:real;
    EmsEC:Integer;
    HandleList:PHandle;

function EmsFreePages : integer;
function EmsAlloc( Pages : integer ) : integer;
function EmsFree(Handle : integer) : boolean;
function EmsMap(Handle,LogPage:integer; PhysPage:byte) : boolean;
function EmsSaveMap( Handle : integer ) : boolean;
function EmsRestoreMap( Handle : integer ) : boolean;
procedure PrintErr;

implementation

uses dos;

var oldexitproc:pointer;

function checkEMS:boolean;
type EmmName  = array [1..8] of char;
     EmmNaPtr = ^EmmName;
const Name : EmmName = 'EMMXXXX0';
var Regs  : Registers;

begin
  Regs.ax := $35 shl 8 + $67;
  msdos( Regs );
  checkEms := (EmmNaPtr(Ptr(Regs.ES,10))^ = Name);
end;

procedure getEmsVersion;
                                                                        
var Regs : Registers;      { Prozessorregister fÅr den Interruptaufruf }
                                                                        
begin
  Regs.ah := $46;
  Intr($67, Regs);
  if (Regs.ah <>0 ) then
    begin
      EmsEC := Regs.ah;
      EmsVersion := 0.0;
    end
  else
   EmsVersion := 0.1*(Regs.al and 15) + (Regs.al shr 4);
end;

procedure insert_handle(handle:word);
var n:PHandle;
  begin
    if Maxavail<sizeof(Thandle) then exit; { sorry - forget it }
    new(n);
    n^.next:=handlelist;
    handlelist:=n;
    n^.handleNo:=handle;
  end;

PROCEDURE remove_handle(handle:word);
var h,i:Phandle;
  begin
    h:=handlelist;i:=Nil;
    while (h<>Nil) and (h^.handleNo<>handle) do begin i:=h;h:=h^.next end;
    if h=Nil then exit;
    if i=Nil then
      begin
        handlelist:=h^.next;
        h^.next:=Nil;
        dispose(h);
      end
    else
      begin
        i^.next:=h^.next;
        h^.next:=Nil;
        dispose(h);
      end;
  end;

procedure freeallpages;
{ ha that's why I created this tool -
  if there's a division by zero or something
  then free the pages allocated by this program ! - no problem :) }
  begin
    while handlelist<>Nil do
      Emsfree(handlelist^.handleNo);
  end;

procedure EmsExitRoutine; far;
  begin
    if handlelist<>Nil then freeallpages;
    exitproc:=oldexitproc;
  end;

function EmsFrameSeg : word; assembler;
asm
  mov       ah,041h
  int       67h
  cmp       ah,0
  je        @@WasOk
  mov       bx,0ffffh
  shr       ax,8
  mov       [EmsEC],ax
@@WasOk:
  mov       ax,bx
end;

function EmsFreePages : integer; assembler;
asm
  mov         ah,42h
  xor         al,al
  int         67h
  cmp         ah,0
  je          @@wasOk
  shr         ax,8
  mov         [EmsEC],ax
  mov         bx,0
@@wasok:
  mov         ax,bx
end;

function EmsAlloc( Pages : integer ) : integer; assembler;
asm
   mov       ah,043h
   mov       bx,[Pages]
   int       67h
   cmp       ah,0
   je        @@wasOK
   mov       dx,0ffffh
   shr       ax,8
   mov       [EmsEC],ax
@@wasOk:
   mov       ax,dx
   cmp       ax,0ffffh
   je        @@donotinsert
   push      ax
   push      ax
   call      insert_handle
   pop       ax
@@donotinsert:
end;

function EmsFree(Handle : integer) : boolean; assembler;
asm
  mov       ah,045h
  mov       dx,[handle]
  int       67h
  shr       ax,8
  mov       [EmsEC],ax
  cmp       ax,0
  je        @@failed
  mov       dx,[handle]
  push      dx
  call      remove_handle
  mov      ax,-1
@@failed:
  inc      ax
end;

function EmsMap(Handle,LogPage:integer; PhysPage:byte) : boolean; assembler;
asm
  mov      ah,044h
  mov      al,[physPage]
  mov      bx,[LogPage]
  mov      dx,[handle]
  int      67h
  shr      ax,8
  mov      [EmsEC],ax
  cmp      ax,0
  je       @@failed
  mov      ax,-1
@@failed:
  inc      ax
end;

function EmsSaveMap( Handle : integer ) : boolean; assembler;
asm
  mov       ah,047h
  mov       dx,[handle]
  int       67h
  shr       ax,8
  mov       [EmsEC],ax
  cmp      ax,0
  je       @@failed
  mov      ax,-1
@@failed:
  inc      ax
end;

function EmsRestoreMap( Handle : integer ) : boolean; assembler;
asm
  mov       ah,048h
  mov       dx,[handle]
  int       67h
  shr       ax,8
  mov       [EmsEC],ax
  cmp      ax,0
  je       @@failed
  mov      ax,-1
@@failed:
  inc      ax
end;

procedure PrintErr;
begin
  writeln('ATTENTION! Error while accessing EMS memory !');
  write('     ... ');
  if ((EmsEC<$80) or (EmsEc>$8E) or (EmsEc=$82)) then
    writeln('unknown error code :',EmsEC)
  else
    case EmsEC of
      $80 : writeln('Internal EMS driver error');
      $81 : writeln('EMS hardware failure');
      $83 : writeln('Unknown EMS handle');
      $84 : writeln('This EMS-function does not exist');
      $85 : writeln('No more free EMS handles');
      $86 : writeln('Error with save/restore mapping');
      $87 : writeln('More pages requested than available');
      $88 : writeln('No enough free pages');
      $89 : writeln('0 pages requested ?');
      $8A : writeln('Problem with access - this logical page does not belong to this handle');
      $8B : writeln('Wrong page number');
      $8C : writeln('Not enough memory for save mapping');
      $8D : writeln('Mapping allready saved');
      $8E : writeln('Error restore mapping - was not saved before');
    end;
end;

begin
  EMSinstalled:=checkEMS;
  if EMSinstalled then
    begin
      getEMSversion;
      FrameSEG[0]:=EmsFrameSeg;
      FrameSEG[1]:=FrameSEG[0] + 1024;
      FrameSEG[2]:=FrameSEG[1] + 1024;
      FrameSEG[3]:=FrameSEG[2] + 1024;
      Frameptr[0]:=ptr(Frameseg[0],0);
      Frameptr[1]:=ptr(Frameseg[1],0);
      Frameptr[2]:=ptr(Frameseg[2],0);
      Frameptr[3]:=ptr(Frameseg[3],0);
    end;
  HandleList:=Nil;
  oldexitproc:=exitproc;
  exitproc:=@EmsExitRoutine;
end.