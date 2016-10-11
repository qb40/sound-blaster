showborder = 0
setborder MACRO no
          if showborder
          push          dx ax
          mov           dx,03dah
          in            al,dx
          mov           dx,03c0h
          mov           al,31h
          out           dx,al
          mov           al,no
          out           dx,al
          pop           ax dx
          endif
ENDM