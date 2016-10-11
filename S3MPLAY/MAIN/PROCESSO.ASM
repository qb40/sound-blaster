model large,pascal

.CODE
.386

PUBLIC Check386

Check386 PROC near
;Now we check if a 386 or higher is present ...
         mov             ax,7000h
         push            ax
         popf
         pushf
         pop             ax
         and             ax,07000h
         jz              endofcheck      ; ax = 0 ... check failed
         xor             ax,ax
         inc             ax              ; ax <> 0 ... check ok !
endofcheck:
         ret
Check386 ENDP

ENDS
END