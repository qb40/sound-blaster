; DOSPROC.ASM - "DOS library"
; STATUS: TEST     :OK
;         COMMENTs :READY

; Not much to say about this file.
; It's a library for using DOS memory managment (640KB) in pascal

model large,pascal

.code
.386

public getdosmem          ; allocate memory
public freedosmem         ; free memory
public getfreesize        ; summary of all free memory blocks
public setsize            ; change size of an allready allocated memory block

getdosmem PROC NEAR segm:DWORD,len:DWORD
          les di,segm
          mov ebx,len
          add ebx,15
          shr ebx,4             ; div 16 ...
          mov ah,48h            ; function 48h - try to allocate dos memory
          int 21h               ; returns segment in AX
          jnc more
          xor ax,ax             ; return false
          ret
more:     mov es:[di+2],ax
          xor ax,ax
          mov es:[di],ax        ; offset = 0
          mov ax,0101h          ; return true
          ret
getdosmem ENDP

freedosmem  PROC NEAR segm:DWORD   ; free reserved memory
            les di,segm            ; segm - pointer to pointer variable
            les di,es:[di]         ; load ES with es:[di+2]
            mov ah,49h             ; function 49h - free memory block
            int 21h
            les di,segm
            xor ax,ax
            mov es:[di],ax
            mov es:[di+2],ax      ; set pointer to NIL !
            ret
freedosmem  ENDP

setsize  PROC NEAR segm:DWORD,len:DWORD   ; change size
         mov ebx,len
         add ebx,15
         shr ebx,4
         les di,segm
         les di,es:[di]           ; load ES with es:[di+2]
         mov ah,4Ah               ; function 4Ah - change size of a memory block
         int 21h
         mov ax,0001h
         jc  ok
         xor ax,ax
ok:      ret
setsize  ENDP

getfreesize PROC NEAR       ; get length of free memory
            mov bx,0ffffh
            mov ah,48h      ; function 48h - get 1MB dos memory
                            ; that's not possible, but you'll get free memory
                            ; in that way ...
            int 21h
            mov ax,bx
            ret
getfreesize ENDP

ends
end