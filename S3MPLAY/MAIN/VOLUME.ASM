model large,pascal

.DATA
EXTRN Signeddata : BYTE
EXTRN Volumetableptr : DWORD
ENDS

.CODE
.386
PUBLIC calcVolumeTable

calcVolumetable PROC NEAR
                cmp      [signeddata],0
                je       calcunsign
calcsign:       push    bp
                push    ds
                lds     si,volumetableptr     ; ds:si pointer to volumetable
                xor     di,di                 ; volume number 0..64
mainloop1:      mov     cx,0                  ; sample -128..+127
shortloop1:     mov     ax,di                 ; ax = current volume number
                movsx   dx,cl                 ; dx = current sample -128..127
                imul    dx
                sar     ax,6                  ; ax = sample * volume /64
ok1:            mov     ds:[si],ax            ; write into table
                add     si,2                  ; next position
                inc     cx                    ; next sample
                cmp     cx,256                ; check if all done with this volume
                jne     shortloop1
                inc     di                    ; next volume
                cmp     di,65                 ; check if all volumes done
                jne     mainloop1     ; <- 65 mal wiederholen
                pop     ds
                pop     bp
                RET

; I dounno if the following thing does work - could't test it yet :(
; ----------
calcunsign:
; ----------
                push    bp
                push    ds
                lds     si,volumetableptr     ; ds:si pointer to volumetable
                xor     di,di                 ; volume number 0..64
mainloop2:      mov     cx,0                  ; sample 0..255
shortloop2:     mov     ax,di                 ; ax = current volume number
                mov     dx,cx
                sub     dx,128
                imul    dx
                sar     ax,6                  ; ax = sample * volume /64
ok2:            mov     ds:[si],ax            ; write into table
                add     si,2                  ; next position
                inc     cx                    ; next sample
                cmp     cx,256                ; check if all done with this volume
                jne     shortloop2
                inc     di                    ; next volume
                cmp     di,65                 ; check if all volumes done
                jne     mainloop2     ; <- 65 mal wiederholen
                pop     ds
                pop     bp
                ret
calcVolumetable ENDP

ENDS
END