TYPE  { look also for assembler structurs in 'GENERAL.DEF' - must be equal !! }

      DWord= record      { you can access dword values now in different ways :
                          however it's never used :) for some action }
              case integer of
                0: (DD:Longint);
                1: (LX,HX:word);
                2: (Rea,Int:Word);
            end;
      THeader = record name:array[0..27] of char;
                      charEOF:char;  { should be 1Ah }
                      filetyp:byte;
                      dummy1 :word;
                      ordnum :word;
                      Insnum :word;
                      Patnum :word;
                      Flags  :word;
                      CWTV   :word; { bit 12 = 1 -> created with screamtracker }
                                    { bit 11..0  -> tracker version:
                                      11..8 = major version,7..0 = minor version }
                      FFv    :word; { fileformatversion }
                      SCRM_ID:array[0..3] of char; { should be 'SCRM' }
                      gvolume:byte;       { global volume }
                      initialspeed:byte;
                      initialtempo:byte;
                      mvolume:byte;       { mastervolume }
                      dummy2 :array[0..9] of byte;
                      special:word;       { not used up2now }
                      channelset:array[0..31] of byte;
                    end;

     PSmpHeader = ^TSmpHeader;
     TSmpHeader = record typ:byte;
                         dosname:array[0..11] of char;
                         hi_mempos:byte;
                         mempos:word;
                         length:longint;
                         loopbeg:longint;
                         loopend:longint;
                         vol:byte;
                         dummy1:byte;
                         packinfo:byte;
                         Flags:byte;
                         c2speed:longint;
                         dummy2:longint;
                         GUS_addr:word;
                         SB_Flags:word;
                         SB_last:longint;
                         name:array[0..27] of char;
                         SCRS_ID:array[0..3] of char;
                       end;

     PAdlHeader = ^TAdlHeader;
     TAdlHeader = record typ:byte;
                         dosname:array[0..11] of char;
                         dummy1:array[0..2] of byte;
                         Data:array[0..11] of byte;
                         Vol:byte;
                         Dsk:byte;
                         dummy2:word;
                         C2spd:longint;
                         dummy3:array[0..11] of byte;
                         name:array[0..27] of char;
                         SCRI_ID:array[0..3] of char;
                       end;

      TCHANNEL = record
                   { general switches : }
                   enabled     :boolean;  { byte: flag if =0 then nothing to mix at the moment }
                   channeltyp  :byte;     { 0=off,1=left,2=right,3,4=adlib ... if 0,3,4 -> everything ignored ! }
                   { current sampledata : }
                   InstrSEG    :word;     { pointer to current instrument data }
                   SampleSEG   :word;     { DOS segment of current sampledata }
                   InstrNo    :byte;      { number of instrument is currently playing }
                   Note        :byte;     { Notenumber is currently playing (except if effects change it...) }
                   { copy of sampledata (maybe it differs a bit): }
                   SampleVol   :byte;     { current sample volume }
                   sLoopflag   :boolean;  { flag if we have to loop sample }
                   sSmpstart   :word;     { default is 0, but if there's a set sample offset, it changes }
                   sLoopstart  :word;     { loop start of current sample =0ffffh if no loop }
                   sLoopend    :word;     { loop end of current sample }
                   sCurPos     :dword;    { fixed point value for current position in sample }
                   sStep       :dword;    { fixed point value of frequency step (distance of one step
                                            depends on period we play currently) }
                   sPeriod     :word;     { st3 period ... you know these amiga values (look at tech.doc of ST3) }
                                          { period does no influence playing a sample direct, but it's for sliding etc. }
                   lower_border:word;     { B-7 or B-5 period for current instrument to check limits }
                   upper_border:word;     { C-0 or C-3 period for current instrument to check limits }
                   { effect info : }
                   command     :word;     { 2 times effectnumber (for using a jmptable) }
                   cmd2nd      :word;     { 2 times additional command for multiple effects }
                   parameter   :byte;     { just the parameters }
                   { data for handling effects : }
                   continueEf  :boolean;  { Flag if we should continue effect - vibrato,tremolo }
                   VibtabOfs   :word;     { yo for each channel its own choise (default = OFS sinuswave) }
                   TrmtabOfs   :word;     { = Offset of wavetable for tremolo }
                   tablepos    :byte;     { <- we reset this if a effect starts uses such a table }
                   VibPara     :byte;     { <- for dual command Vib + Vol }
                   PortPara    :byte;     { <- for dual command Port + Vol }
                   OldPeriod   :word;     { save that value for some effects }
                   Oldvolume   :byte;     { save that value for tremolo }
                   WantedPeri  :word;     { <- period to slide to with Portamento }
                   ArpegPos    :byte;     { which of those 3 notes we currently play ... }
                   note1       :byte;     { \ }
                   note2       :byte;     { -+ note : 3 notes we do arpeggio between }
                   Step0       :dword;    { \ }
                   Step1       :dword;    {  |- the 3 step values we switch between in arpeggio effect (0 is start value
                                             <- we have to refesh after arpeggio) }
                   Step2       :dword;    { / }
                   ctick       :byte;     { ticks left to retrigg not }
                   savNote     :byte;     { \  }
                   savInst     :byte;     {  | - new values for notedelay ... }
                   SavVol      :byte;     {  | }
                   ndTick      :byte;     { /  }
                 end;

     TInstr         = array[0..16*5-1] of byte;
     TInstrArray    = array[1..MAX_Samples]   of TInstr;
     TPatternSarray = array[0..MAX_patterns]  of word;         { segment for every pattern }
     TOrderArray    = array[0..MAX_orders]    of byte;         { song arrangement }
     TchannelArray  = array[0..MAX_channels-1] of Tchannel;    { all public/private data for every channel }
     PArray         = ^TArray;
     TArray         = array[0..65532] of byte;