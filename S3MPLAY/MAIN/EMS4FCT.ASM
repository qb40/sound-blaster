model large,pascal

.data
EXTRN patEMShandle:WORD
EXTRN EMSpat:BYTE
EXTRN EMSsmp:BYTE
EXTRN smpEMShandle:WORD
EXTRN savhandle:WORD
      patname DB 'Pattern',0
      smpname DB 'Samples',0
      savname DB 'saveMAP',0
ends

.code

public setEMSnames

setEMSnames PROC NEAR
            cmp      [EMSpat],0
            je       endofset     ; No EMS reserved !
            ; first EMS pattern
            mov      si,offset patname
            mov      ah,53h
            mov      al,1
            mov      dx,[patEMShandle]
            int      67h
            ; then Save Handle:
            mov      si,offset savname
            mov      ah,53h
            mov      al,1
            mov      dx,[savhandle]
            int      67h
            ; now SMP handle if there's for samples:
            cmp      [EMSsmp],0
            je       endofset
            mov      si,offset smpname
            mov      ah,53h
            mov      al,1
            mov      dx,[smpEMShandle]
            int      67h
endofset:   ret
setEMSnames ENDP

ends
end