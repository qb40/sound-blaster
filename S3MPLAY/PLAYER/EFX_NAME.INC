const effect_name:array[0..53] of string[26] =
('','A: set speed',
    'B: jump to order',
    'C: pattern break to',

    'D: Volume slide down',
    'D: Volume slide up',
    'D: Fine volume down',
    'D: Fine volume up',

    'E: Slide pitch down',
    'E: Fine slide pitch down',
    'E: Xtrafine slidedown',

    'F: Slide pitch up',
    'F: Fine slide pitch up',
    'F: Xtrafine slide up',

    'G: Portamento to note',
    'H: Vibrato',
    '->I<-: Tremor',
    'J: Arpeggio',

    'K: Vibrato+Vol.slddown',
    'K: Vibrato+Vol.sldup',
    'K: Vibrato+FineVoldown',
    'K: Vibrato+FineVolup',

    'L: Portameto+Vol.slddown',
    'L: Portameto+Vol.sldup',
    'L: Portameto+FineVoldown',
    'L: Portameto+FineVolup',

    'O: Set sample offset',

    'Q: Retrigg note (no sld)',
    'Q: Retrigg note (vol.down)',
    'Q: Retrigg note (vol*2/3)',
    'Q: Retrigg note (vol*1/2)',
    'Q: Retrigg note (vol.up)',
    'Q: Retrigg note (vol*3/2)',
    'Q: Retrigg note (vol*2)',

    'R: Tremolo',

    '>S0: Set filter <not impl>',
    '>S1: Set glissando <not impl>',
    '>S2: Set finetune',
    '>S3: Set vibrato waveform',
    '>S4: Set tremolo waveform',
    '',
    '',
    '',
    '>S8: Amiga command E8xh',
    '',
    '>SA: Stereo control <not implemented>',
    '>SB: Pattern loop',
    '>SC: Note cut',
    '>SD: Note delay',
    '>SE: Pattern delay',
    '>SF: Funk repeat <not implemented>',

    'T: set tempo',
    'U: fine vibrato',
    'V: set global volume');
function get_effectname(chn_no,cinfo,c2nd:byte;para:byte):string;
const conv:array[0..22] of byte = (0,1,2,3,4,8,11,14,15,16,17,18,22,0,0,26,0,27,34,35,51,52,53);
var s,t:string[40];
  begin
    s:=effect_name[conv[cinfo]+c2nd];
    if cinfo=1 then { Set speed }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=2 then { jump to order }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=3 then { pattern break to }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=4 then { volume effects }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=5 then { slide down effects }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=6 then { slide down effects }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=7 then { portamento }
        s:=s+' :'+b2hs(channel[chn_no].PortPara);
    if cinfo=8 then { Vibrato }
        s:=s+' :'+b2hs(channel[chn_no].VibPara);
    if cinfo=9 then { Tremor                           ... not done yet };
    if cinfo=10 then { Arpeggio }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=11 then { Vibrato+Vol }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=12 then { portamento+Vol }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=15 then { Sample offset }
        s:=s+': '+b2hs(para)+'00h';
    if cinfo=17 then { Retrigger }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=18 then { Tremolo }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=19 then { Special commands            ... not fully done yet }
        s:=effect_name[conv[cinfo]+channel[chn_no].Parameter shr 4]+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=20 then { Set tempo }
        s:=s+' :'+b2hs(channel[chn_no].Parameter);
    if cinfo=21 then { Fine Vibrato }
        s:=s+' :'+b2hs(channel[chn_no].VibPara);
    if cinfo=22 then { Set global volume }
        s:=s+' :'+b2ds(channel[chn_no].Parameter);
    str(cinfo,t);
    get_effectname:=s;
  end;