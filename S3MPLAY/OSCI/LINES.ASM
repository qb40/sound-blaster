
MODEL large,pascal

.DATA
col       db (?)
dabklein_ dw (?)
extrn     drawseg:word
ENDS

.CODE
.386

; think about to creat different parts for
;    1.  deltax>deltay
;    2.  deltax=deltay       <- that's important !
;    3.  deltax<deltay
; the result would be one memory acess less than before need in the drawloop
; and only two "j"umps and "cmp" at the beginning
; ist mir eben (29.4) noch eingefallen:
; weitere optimierung fuer seknrechte und waagerechte linien

PUBLIC linie

linie PROC NEAR x1:WORD,y1:WORD,x2:WORD,y2:WORD,farbe:BYTE
      push   bp
      mov    ax,drawseg
      mov    es,ax
      mov    al,farbe
      mov    col,al
      mov    di,x1
      mov    si,x2
      mov    cx,y1
      mov    dx,y2
      cmp    cx,dx
      jle    l1
      xchg   cx,dx
      xchg   si,di
l1:
      cmp    di,si
      jne    not_equal
      cmp    cx,dx
      jne    not_equal
      mov    ax,cx
      mov    dx,320
      mul    dx
      add    di,ax
      mov    al,farbe
      mov    es:[di],al
      jmp    ende
not_equal:
      mov    ax,1                ; addx = 1

      sub    si,di
      jg     dx_p
      neg    ax                  ; addx = -1
      neg    si
dx_p:
      sub    dx,cx               ; dx immer > cx
      mov    bp,ax
      push   dx
      mov    ax,cx               ; ax = y1
      mov    dx,320
      mul    dx
      add    di,ax               ; di beinhaltet Bildadresse vom 1.Punkt
      pop    dx
      cmp    si,dx
      jge    dx_gr               ; Jump if dx>dy damit ax=x und y = 0 (x ist Laufvariable)
      ;  dy>dx    ....... Y ist Laufvar
      xchg   si,dx
      mov    al,col
      mov    cx,si               ; cx wird counter
      shl    dx,1
      mov    si,dx
      sub    dx,cx
      mov    bx,dx
      sub    dx,cx
      inc    cx
      ; after this :
      ;               dx = dabgross_add
      ;               di = Pointposition
      ;               bp = dabgross
      ;               cx = Counter
      ;               bx = akt_wert
      ;               si = dabklein_add
      ;               al = Farbe
      ; look that is the lineroutine:      short is'nt it ? ;-)

l2:
      mov    es:[di],al
      add    di,320
      cmp    bx,0
      jge    l4
      add    bx,si
      loop   l2
      jmp    ende
l4:
      add    di,bp
      add    bx,dx
      loop   l2
      jmp    ende
dx_gr: ; X ist Laufvariable
      mov    al,col
      mov    cx,si               ; cx wird counter
      shl    dx,1
      mov    si,dx
      sub    dx,cx
      mov    bx,dx
      sub    dx,cx
      inc    cx
      ; after this :
      ;               dx = dabgross_add
      ;               di = Pointposition
      ;               bp = dabgross
      ;               dabklein - in Dataseg
      ;               cx = Counter
      ;               bx = akt_wert
      ;               si = dabklein_add
      ;               al = Farbe
      ; look that is the lineroutine:      short is'nt it ? ;-)

sl:
      mov    es:[di],al
      add    di,bp
      cmp    bx,0
      jge    dabgr
      add    bx,si
      loop   sl
      jmp    ende
dabgr:
      add    di,320
      add    bx,dx
      loop   sl
ende:
      pop    bp
      ret
linie ENDP

end