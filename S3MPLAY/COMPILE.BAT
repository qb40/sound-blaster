cd blaster
bpc blaster.pas
cd ..\main
tasm *
bpc emstool.pas
bpc -U..\blaster s3mplay.pas
cd ..
bpc -Ublaster;main smalls3m
cd osci
tasm *
bpc -U..\blaster;..\main s3m_osci
cd ..\player
tasm *
bpc -U..\blaster;..\main plays3m
cd ..
