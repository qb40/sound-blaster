/*=============================================================================*
 * DSPSCAN v3.10 - SoundBlaster Scanning Program
 *	Craig Jackson
 *-----------------------------------------------------------------------------
 * Requires: 8086, DOS 3.0, SoundBlaster
 *-----------------------------------------------------------------------------
 * Revision 1.00	Initial DSPC evaluation (RWR, DMA/IRQ)
 *			2.00	Added DSPC status (0FBh, 0FCh)
 *          3.00    Merged DSPC associativity and read tests, mixer, 0E1h check
 *          3.10    Added BLASTER display, mixer bitscan, wierd TestDSPx DREQs
 *-----------------------------------------------------------------------------
 * þ Virtualized DMA yields unreliable DMA polling results
 * þ DSP scanning method 0xE1 returns some invalid information (R:0 W:0 R:0)
 * þ DSP scanning method 0xFD may return some invalid flags for old cards
 * þ DSP highspeed commands may not responed to scanning properly
 * þ DSP commands which lower DREQ are not properly polled (next version)
 * þ DSP scan mixer register is combination of two samplings.
 * þ Command line arguments may overflow if larger than 0xFFFF, some unused
 * þ Unreliable results on IBM PS/2s and most SoundBlaster compatibles
 * þ Available disk space for log file is not checked
 *-----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned char  BYTE;
typedef unsigned short WORD;

typedef int 		   BOOL;
#define FALSE	0
#define TRUE   (!FALSE)

#define isEnabled(x)	((x) ? txtEnabled : txtDisabled)

// Define LOG7BIT to use only ASCII characters in the log file
#ifdef LOG7BIT
#define CHRDEF_DSPSCANE1    '+'
#define CHRDEF_DSPWIERD     'O'
#define CHRDEF_DSPNOTWIERD  '-'
#define CHRDEF_LNA          '-'     // '---'
#define CHRDEF_LNB          '-'     // '---'
#define CHRDEF_LNC          '-'
#else
#define CHRDEF_DSPSCANE1    '+'
#define CHRDEF_DSPWIERD     'þ'
#define CHRDEF_DSPNOTWIERD  '-'
#define CHRDEF_LNA          'Ä'     // 'ÄÄÄ'
#define CHRDEF_LNB          '-'     // '---'
#define CHRDEF_LNC          ''
#endif  // LOG7BIT

#define F_DSPS		0x0001
#define F_DSPA      0x0002
#define F_DSPR      0x0004
#define F_MIXS      0x0008
#define F_MIXB      0x0010
#define F_MIXD      0x0020

typedef struct tagDSPC
{
    WORD wFlags;
	BYTE cFB, cFC;
	BYTE cX82;
	int  R0, W0, R1;
}DSPC;

#define D_UNKNOWN	0x0000
#define D_KNOWN 	0x0001
#define D_IRQ8		0x0002
#define D_IRQ16 	0x0004
#define D_IRQMPU	0x0008
#define D_PDMA8HI	0x0010
#define D_PDMA8LO	0x0020
#define D_PDMA16HI	0x0040
#define D_PDMA16LO	0x0080
#define D_WIERD     0x0100

typedef struct tagMIXER
{
    BYTE    cFlags;
	BYTE	cResetData;
	BYTE	cOldData;
}MIXER;

#define M_STATUS	0x01
#define M_CHANGED	0x02

BOOL ResetDSP(void);
BYTE ReadDSP(void);
void WriteDSP(BYTE cData);
void ResetMIX(void);
BYTE ReadMIX(BYTE cIndex);
void WriteMIX(BYTE cIndex, BYTE cData);
BOOL isMixer(void);
BOOL isDSPxFD(void);
BOOL TestDSPxE1(DSPC *pDSPC, int i);    // Method: DSP Version
BOOL TestDSPxFD(DSPC *pDSPC, int i);    // Method: DSP Command Status
BYTE *PtrDSPC(int i, BYTE *pBuf);

void Delay(int nTicks);

void putslna(void);
void putslnb(void);
void putslnc(void);

const char *txtHeader  = "DSP Command Scanner v3.10\n"        \
						 "  Copyright (C) 1994 - Craig Jackson\n\n";
const char *txtOptions = " DSPSCAN <file> [A:<n>] [I:<n>] [D:<n>] [DSP:<min>-<max>] [MIX:<min>-<max>]\n"  \
                         "                [S:<n>] [X:<n>] [MIXER] [NODSPS] [NODSPA] [NODSPR] [NOMIXS]\n"  \
                         "                [NOMIXB] [NOMIXD]\n"                                            \
                         "\n"                                                                             \
                         "  <file>              Output file (required)\n"                                 \
                         "  A:<n>               SoundBlaster base address (default 220)\n"                \
                         "  I:<n>               SoundBlaster IRQ (default 7, ignored on SB16)\n"          \
                         "  D:<n>               SoundBlaster DMA (default 1, ignored on SB16)\n"          \
                         "  DSP:<min>-<max>     Range for DSP command scans (default 00-FF)\n"            \
                         "  MIX:<min>-<max>     Range for mixer index scans (default 00-FF)\n"            \
                         "  S:<n>               Seed value for DSP scan writes (default 00)\n"            \
                         "  X:<n>               Index for DSP command mixer status (default 82)\n"        \
                         "  MIXER               Forces mixer detection\n"                                 \
                         "  NODSPS              Skips DSP command scan\n"                                 \
                         "  NODSPA              Skips DSP command associativity scan\n"                   \
                         "  NODSPR              Skips DSP command read scan\n"                            \
                         "  NOMIXS              Skips mixer index scan\n"                                 \
                         "  NOMIXB              Skips mixer bit scan\n"                                   \
						 "  NOMIXD              Skips mixer defaults scan\n";

const char *txtEnabled   = "Enabled";
const char *txtDisabled  = "Disabled";

FILE *fp;

WORD wBaseAddr = 0x0220;
BYTE cIRQ      = 7,
     cDMA8     = 1,
     cDMA16;
BOOL blMixer   = FALSE;
BYTE cMajorVersion, cMinorVersion;
WORD wFlags    = F_DSPS | F_DSPA | F_DSPR | F_MIXS | F_MIXB | F_MIXD;
BOOL blIOError = FALSE;
WORD nTimeout  = 0xFFFF;
BYTE cSeed	   = 0x00;
BYTE cMixerX   = 0x82;

BOOL (*fpTestDSP)(DSPC *pDSPC, int i);

DSPC    tblDSPC[256];
MIXER   tblMIX[256];

BYTE    pBufA[256*2];
BYTE    pBufB[256*2];

void main(int argc, char *argv[])
{
	int 	minDSPC = 0x00;
	int 	maxDSPC = 0xFF;

	int 	minMIX	= 0x00;
	int 	maxMIX	= 0xFF;

    int     i, j, k, l, m, n;
    char   *p;

	fputs(txtHeader, stderr);

	if (argc >= 2)
    {
        fp = fopen(argv[1], "w");
    }
    if (!fp)
	{
		fputs(txtOptions, stderr);
		exit(0);
	}

	for (i = 2; i < argc; i++)
	{
        if (!strnicmp(argv[i], "A:", 2))
        {
            wBaseAddr = (BYTE)strtoul(&argv[i][2], NULL, 16);
        }
        else if (!strnicmp(argv[i], "I:", 2))
        {
            if ((cIRQ = (BYTE)atoi(&argv[i][2])) > 15)
            {
                fputs(" ERROR: SoundBlaster IRQ invalid", stderr);
                exit(1);
            }
        }
        else if (!strnicmp(argv[i], "D:", 2))
        {
            if (((cDMA8 = (BYTE)atoi(&argv[i][2])) > 3) || (cDMA8 == 2))
            {
                fputs(" ERROR: SoundBlaster DMA invalid", stderr);
                exit(1);
            }
        }
        else if (!strnicmp(argv[i], "DSP:", 4))
		{
            minDSPC = strtoul(&argv[i][4], &p, 16);
            maxDSPC = strtoul(p+1, NULL, 16);
            if ((*p != '-') || (minDSPC > maxDSPC))
            {
                fputs(" ERROR: DSP range invalid", stderr);
                exit(1);
            }
		}
		else if (!strnicmp(argv[i], "MIX:", 4))
		{
            minMIX = strtoul(&argv[i][4], &p, 16);
            maxMIX = strtoul(p+1, NULL, 16);
            if ((*p != '-') || (minMIX > maxMIX))
            {
                fputs(" ERROR: Mixer range invalid", stderr);
                exit(1);
            }
		}
		else if (!strnicmp(argv[i], "S:", 2))
		{
			cSeed = (BYTE)strtoul(&argv[i][2], NULL, 16);
		}
		else if (!strnicmp(argv[i], "X:", 2))
		{
			cMixerX = (BYTE)strtoul(&argv[i][2], NULL, 16);
		}
        else if (!stricmp(argv[i], "MIXER"))
        {
            blMixer = TRUE;
        }
		else if (!stricmp(argv[i], "NODSPS"))
		{
			wFlags &= ~(F_DSPS);
		}
		else if (!stricmp(argv[i], "NODSPA"))
		{
			wFlags &= ~(F_DSPA);
		}
		else if (!stricmp(argv[i], "NODSPR"))
		{
			wFlags &= ~(F_DSPR);
		}
		else if (!stricmp(argv[i], "NOMIXS"))
		{
			wFlags &= ~(F_MIXS);
		}
        else if (!stricmp(argv[i], "NOMIXB"))
        {
            wFlags &= ~(F_MIXB);
        }
		else if (!stricmp(argv[i], "NOMIXD"))
		{
			wFlags &= ~(F_MIXD);
		}
		else
			fprintf(stderr, " WARNING: Unknown option \"%s\"; ignored\n\n", argv[i]);
	}

	if (!ResetDSP())
	{
		fputs(" ERROR: Invalid port address", stderr);
		exit(1);
	}

    if (!blMixer)
        blMixer = isMixer();

	WriteDSP(0xE1);
    cMajorVersion = ReadDSP();
    cMinorVersion = ReadDSP();

    WriteDSP(0xE1);
    if ((cMajorVersion != ReadDSP()) || (cMinorVersion != ReadDSP()) || (blIOError))
    {
        fputs(" ERROR: SoundBlaster port address conflicting with another card", stderr);
        exit(1);
    }

    putslna();

#ifdef LOG7BIT
    fputs("DSPSCAN V3.10 LOGFILE (ASCII)\n", fp);
#else
    fputs("DSPSCAN V3.10 LOGFILE\n", fp);
#endif // LOG7BIT
    putslna();
    fputc('\n', fp);

    if (cMajorVersion < 0x04)
    {
        nTimeout  = 0x04FF;     // unsure of exact value

        fprintf(fp, "SoundBlaster Port      : %04Xh\n"
                    "            *IRQ       : %2d\n"
                    "            *DMA8      :  %d\n", wBaseAddr, cIRQ, cDMA8);
        if (p = getenv("BLASTER"))
        {
            fprintf(fp, "             BLASTER   : %s\n", p);
        }
        fprintf(fp, "             Version   : %2d.%02d\n\n", cMajorVersion, cMinorVersion);
    }
    else
	{
        nTimeout  = 0x00FF;

		if ((k = (int)ReadMIX(0x80)) & 0x08)
			cIRQ = 10;
		else if (k & 0x04)
			cIRQ =	7;
		else if (k & 0x02)
			cIRQ =	5;
		else if (k & 0x01)
			cIRQ =	2;
		else
		{
			fputs(" ERROR: SoundBlaster IRQ indeterminate", stderr);
			exit(3);
		}

		if ((k = (int)ReadMIX(0x81)) & 0x08)
			cDMA8 = 3;
		else if (k & 0x02)
			cDMA8 = 1;
		else if (k & 0x01)
			cDMA8 = 0;
		else
		{
			fputs(" ERROR: SoundBlaster DMA8 indeterminate", stderr);
			exit(3);
		}

		if (k & 0x80)
			cDMA16 = 7;
		else if (k & 0x40)
			cDMA16 = 6;
		else if (k & 0x20)
			cDMA16 = 5;
		else
		{
			if (wFlags & F_DSPS)
				fputs(" WARNING: Aliased DMA16, unreliable DSPC scan DMA poll\n", stderr);
			cDMA16 = cDMA8;
		}

        fprintf(fp, "SoundBlaster Port      : %04Xh\n"
                    "             IRQ       : %2d\n"
                    "             DMA8      :  %d\n"
                    "             DMA16     :  %d\n", wBaseAddr, cIRQ, cDMA8, cDMA16);
        if (p = getenv("BLASTER"))
        {
            fprintf(fp, "             BLASTER   : %s\n", p);
        }
        fprintf(fp, "             Version   : %2d.%02d\n"
                    "             Copyright :", cMajorVersion, cMinorVersion);
		WriteDSP(0xE3);
		i = (int)' ';
		do
		{
#ifdef LOG7BIT
            fputc((i > 0x7F) ? '?' : i, fp);
#else
            fputc(i, fp);
#endif // LOG7BIT
			i = (int)ReadDSP();
		} while ((blIOError != TRUE) && (i != '\0'));
        fputs("\n\n", fp);
	}

    fpTestDSP = (isDSPxFD()) ? TestDSPxFD : TestDSPxE1;

    fprintf(fp, "Parameters  %cDSPC Scan             : %-8s (%02X-%02Xh : %02Xh)\n"
                "             DSPC Associative Scan : %-8s\n"
                "             DSPC Read Scan        : %-8s\n"
                "             MIXR Scan             : %-8s (%02X-%02Xh : %02Xh)\n"
                "             MIXR Bit Scan         : %-8s\n"
                "             MIXR Defaults Scan    : %-8s\n\n",
                (fpTestDSP == TestDSPxE1) ? CHRDEF_DSPSCANE1 : ' ',
                isEnabled(wFlags & F_DSPS), minDSPC, maxDSPC, cSeed,
                isEnabled(wFlags & F_DSPA),
                isEnabled(wFlags & F_DSPR),
                isEnabled((wFlags & F_MIXS) && (blMixer)), minMIX,  maxMIX, cMixerX,
                isEnabled((wFlags & F_MIXB) && (blMixer)),
                isEnabled((wFlags & F_MIXD) && (blMixer)));

	if (wFlags & (F_DSPS|F_DSPA|F_DSPR))
	{
        fprintf(stderr, "Evaluating DSP   ...      ");

		for (i = minDSPC; i <= maxDSPC; i++)
		{
            fprintf(stderr, "\b\b\b\b\b%c 0%02X", (i & 0x0001) ? 'þ' : 'ù', i);
            (*fpTestDSP)(&tblDSPC[i], i);
		}

        fputs("\b\b\b\b\b Done\n", stderr);
	}

	if (wFlags & F_DSPS)
	{
		putslnb();
        fputs("DSP COMMAND SCAN\n\n", fp);

		putslnc();
		for (i = minDSPC; i <= maxDSPC; i++)
		{
            if (tblDSPC[i].wFlags & D_KNOWN)
			{
			   /*----------------------------------------------------------------------------*
				 00 - R:00 W:00 R:00 - DACDMA8 ADCDMA8 DACDMA16 ADCDMA16 IRQ8	 IRQ16	 IRQMPU
					   (FB:00 FC:00)   SPEAKER TIMECNT AUTOINT2 AUTOINT4 DMA8xx  DMA16xx X82:00
				*----------------------------------------------------------------------------*/
                fprintf(fp, " %02X %c R%c%02X W:%02X R%c%02X - %7s %7s %8s %8s %7s %7s %6s\n"
                            "        (FB:%02X FC:%02X)  %7s %7s %8s %8s %5s%2s %5s%2s X%02X:%02X\n",
                            i,
                            (tblDSPC[i].wFlags & D_WIERD) ? CHRDEF_DSPWIERD : CHRDEF_DSPNOTWIERD,
                            (tblDSPC[i].R0 == 0xFF) ? '>' : ':', tblDSPC[i].R0,
                            tblDSPC[i].W0,
                            (tblDSPC[i].R1 == 0xFF) ? '>' : ':', tblDSPC[i].R1,
                            (tblDSPC[i].cFB    & 0x01)      ? "DACDMA8"   : "-------",
                            (tblDSPC[i].cFB    & 0x02)      ? "ADCDMA8"   : "-------",
                            (tblDSPC[i].cFB    & 0x04)      ? "DACDMA16"  : "--------",
                            (tblDSPC[i].cFB    & 0x08)      ? "ADCDMA16"  : "--------",
                            (tblDSPC[i].wFlags & D_IRQ8)    ? "IRQ8"      : "-------",
                            (tblDSPC[i].wFlags & D_IRQ16)   ? "IRQ16"     : "-------",
                            (tblDSPC[i].wFlags & D_IRQMPU)  ? "IRQMPU"    : "------",
                            tblDSPC[i].cFB, tblDSPC[i].cFC,
                            (tblDSPC[i].cFB    & 0x10)      ? "SPEAKER"   : "-------",
                            (tblDSPC[i].cFB    & 0x80)      ? "TIMECNT"   : "-------",
                            (tblDSPC[i].cFC    & 0x04)      ? "AUTOINT2"  : "--------",
                            (tblDSPC[i].cFC    & 0x10)      ? "AUTOINT4"  : "--------",
                            (tblDSPC[i].wFlags & (D_PDMA8HI|D_PDMA8LO)) ? "DMA8" : "-----",
                            (tblDSPC[i].wFlags & D_PDMA8HI) ? "HI" : (tblDSPC[i].wFlags & D_PDMA8LO) ? "LO" : "--",
                            (tblDSPC[i].wFlags & (D_PDMA16HI|D_PDMA16LO)) ? "DMA16" : "-----",
                            (tblDSPC[i].wFlags & D_PDMA16HI) ? "HI" : (tblDSPC[i].wFlags & D_PDMA16LO) ? "LO" : "--",
                            cMixerX, tblDSPC[i].cX82);

				putslnc();
			}
		}
	}

	if (wFlags & F_DSPA)
	{
		putslnb();
        fputs("DSP ASSOCIATIVE SCAN\n\n", fp);

        fputs("Crosslinking DSP ...          ", stderr);
        for (i = minDSPC; i <= maxDSPC; i++)
        {
            fprintf(stderr, "\b\b\b\b\b\b\b\b\b%c 0%02X 000", (i & 0x0001) ? 'þ' : 'ú', i);

            if ((tblDSPC[i].wFlags & D_KNOWN) && (tblDSPC[i].R0 != 0xFF))
            {
                for (k = minDSPC; k <= maxDSPC; k++)
                {
                    fprintf(stderr, "\b\b%02X", k);

                    if ((k != i) && (tblDSPC[k].wFlags & D_KNOWN) && (tblDSPC[k].R0 != 0xFF))
                    {
                        ResetDSP();
                        PtrDSPC(i, pBufA);

                        WriteDSP((BYTE)k);
                        for (l = 0; l < tblDSPC[k].R0; ReadDSP(), l++)
                            ;
                        for (l = 0; l < tblDSPC[k].W0; WriteDSP(cSeed), l++)
                            ;
                        for (l = 0; l < tblDSPC[k].R1; ReadDSP(), l++)
                            ;

                        if (memcmp(pBufA, PtrDSPC(i, pBufB), tblDSPC[i].R0 + tblDSPC[i].R1))
                        {
                            fprintf(fp, " %02X<>%02X - ", i, k);

                            for (l = 0, j = 0; l < tblDSPC[i].R0 + tblDSPC[i].R1; l++)
                            {
                                fprintf(fp, "%02X ", pBufA[l]);

                                if (++j > 21)
                                {
                                    fprintf(fp, "\n          ");
                                    j = 0;
                                }
                            }

                            fprintf(fp, "\n          ");

                            for (l = 0, j = 0; l < tblDSPC[i].R0 + tblDSPC[i].R1; l++)
                            {
                                fprintf(fp, "%02X ", pBufB[l]);

                                if (++j > 21)
                                {
                                    fprintf(fp, "\n          ");
                                    j = 0;
                                }
                            }

                            fputc('\n', fp);
                            putslnc();
                        }
                    }
                }
            }
        }
        fputs("\b\b\b\b\b\b\b\b\b Done    \n", stderr);
	}

	if (wFlags & F_DSPR)
	{
		putslnb();
        fputs("DSP READ SCAN\n\n", fp);

        fprintf(stderr, "Reading DSP      ...      ");

        for (i = minDSPC; i <= maxDSPC; i++)
        {
            fprintf(stderr, "\b\b\b\b\b%c 0%02X", (i & 0x0001) ? 'þ' : 'ù', i);

            if (tblDSPC[i].R0 + tblDSPC[i].R1)
            {
                ResetDSP();

                fprintf(fp, " %02X - ", i);
                WriteDSP((BYTE)i);

                j = 0;
                if (tblDSPC[i].R0)
                {
                    for (k = 0; k < tblDSPC[i].R0; k++)
                    {
                        fprintf(fp, "%02X ", ReadDSP());
                        if (++j > 23)
                        {
                            j = 0;
                            fprintf(fp, "\n      ");
                        }
                    }
                }

                if ((tblDSPC[i].R0) && (tblDSPC[i].R1))
                {
                    fprintf(fp, ":: ");
                    if (++j > 23)
                    {
                        j = 0;
                        fprintf(fp, "\n      ");
                    }
                }

                for (k = 0; k < tblDSPC[i].W0; k++)
                    WriteDSP(cSeed);

                if (tblDSPC[i].R1)
                {
                    for (k = 0; k < tblDSPC[i].R1; k++)
                    {
                        fprintf(fp, "%02X ", ReadDSP());
                        if (++j > 23)
                        {
                            j = 0;
                            fprintf(fp, "\n      ");
                        }
                    }
                }

                fputc('\n', fp);
                putslnc();
            }
        }

        fputs("\b\b\b\b\b Done\n", stderr);
	}

    if ((wFlags & (F_MIXS|F_MIXB|F_MIXD)) && (blMixer))
	{
        fprintf(stderr, "Evaluating MIXER ...     ");

		tblMIX[0x00].cFlags = M_CHANGED;

		for (i = (minMIX > 0x00) ? minMIX : 0x01; i <= maxMIX; i++)
			tblMIX[i].cOldData = ReadMIX((BYTE)i);

		ResetMIX();
		for (i = (minMIX > 0x00) ? minMIX : 0x01; i <= maxMIX; i++)
		{
            fprintf(stderr, "\b\b\b\b\b%c 0%02X", (i & 0x0001) ? 'þ' : 'ù', i);

			tblMIX[i].cResetData = ReadMIX((BYTE)i);
			WriteMIX((BYTE)i, (BYTE)((~tblMIX[i].cResetData) & 0x7F));

			tblMIX[i].cFlags = (ReadMIX(0x01) & 0x80) ? 0x00 : M_STATUS;

			if (ReadMIX((BYTE)i) != tblMIX[i].cResetData)
				tblMIX[i].cFlags |= M_CHANGED;
			WriteMIX((BYTE)i, (BYTE)tblMIX[i].cOldData);

			ReadMIX((BYTE)i);	  // prime status register
		}

		for (i = (minMIX > 0x00) ? minMIX : 0x01; i <= maxMIX; i++)
			WriteMIX((BYTE)i, tblMIX[i].cOldData);

        fputs("\b\b\b\b\b  Done\n", stderr);
	}

    if ((wFlags & F_MIXS) && (blMixer))
	{
		putslnb();
        fputs("MIXER INDEX SCAN\n\n", fp);

		for (i = minMIX, j = 0; i <= maxMIX; i++)
		{
			if (tblMIX[i].cFlags & (M_STATUS|M_CHANGED))
			{
                fprintf(fp, "%c%02X", (tblMIX[i].cFlags & M_STATUS) ? (tblMIX[i].cFlags & M_CHANGED) ? '+' : '*' : ' ', i);
				if (++j > 25)
				{
					j = 0;
                    fputc('\n', fp);
				}
			}
		}
		if (j != 0)
            fputc('\n', fp);

        fputs("\n* STATUS DETECTED   + BOTH DETECTED\n", fp);
	}

    if ((wFlags & F_MIXB) && (blMixer))
    {
        fprintf(stderr, "Bitscanning MIXER...      ");

		putslnb();
        fputs("MIXER BIT SCAN\n\n", fp);

		for (i = (minMIX > 0x00) ? minMIX : 0x01; i <= maxMIX; i++)
		{
            fprintf(stderr, "\b\b\b\b\b%c 0%02X", (i & 0x0001) ? 'þ' : 'ù', i);

			if (tblMIX[i].cFlags & (M_STATUS|M_CHANGED))
			{
                k = ReadMIX((BYTE)i);

                WriteMIX((BYTE)i, 0xFF);
                l = ReadMIX((BYTE)i);

                WriteMIX((BYTE)i, 0x00);
                m = ReadMIX((BYTE)i);

                WriteMIX((BYTE)i, (BYTE)k);

                fprintf(fp, " %02X:", i);
                for (n = 0; n < 8; n++, l <<= 1, m <<= 1)
                {
                    if (n == 4)
                        fputc('|', fp); // nibble separator

                    if ((!(l & 0x80)) && (m & 0x80))
                        fputc('?', fp);
                    else if (!(l & 0x80))
                        fputc('0', fp);
                    else if (m & 0x80)
                        fputc('1', fp);
                    else
                        fputc('*', fp);
                }
                fputc(' ', fp);

                if (++j > 4)
				{
					j = 0;
                    fputc('\n', fp);
				}
			}
		}
		if (j != 0)
            fputc('\n', fp);

        fputs("\b\b\b\b\b  Done\n", stderr);
    }

    if ((wFlags & F_MIXD) && (blMixer))
	{
		putslnb();
        fputs("MIXER DEFAULTS SCAN\n\n", fp);

		for (i = minMIX, j = 0; i <= maxMIX; i++)
		{
            fprintf(fp, " %02X%c%02X", i, (tblMIX[i].cFlags & (M_STATUS|M_CHANGED)) ? '=' : '+', tblMIX[i].cResetData);

			if (++j > 12)
			{
				j = 0;
                fputc('\n', fp);
			}
		}
		if (j != 0)
            fputc('\n', fp);

        fputs("\n= DETECTED   + UNDETECTED\n", fp);
	}

	putslna();
	exit(0);
}

BOOL ResetDSP(void)
{
	__asm
	{
		mov 	dx,wBaseAddr
		add 	dx,006h
		mov 	al,001h
		out 	dx,al
		in		al,dx
		in		al,dx
		in		al,dx
		in		al,dx
		xor 	al,al
		out 	dx,al
	}

	return (ReadDSP() == 0xAA) && (blIOError == FALSE);
}

BYTE ReadDSP(void)
{
	BYTE retr = 0xFF;

	blIOError = TRUE;
	__asm
	{
		mov 	dx,wBaseAddr
		add 	dx,00Eh
        mov     cx,nTimeout
    L0: in      al,dx
		and 	al,080h
		loopz	L0
		jz		L1
		sub 	dx,004h
		in		al,dx
		mov 	retr,al
		mov 	blIOError,FALSE
	L1:
	}

	return retr;
}

void WriteDSP(BYTE cData)
{
	blIOError = TRUE;
	__asm
	{
		mov 	dx,wBaseAddr
		add 	dx,00Ch
        mov     cx,nTimeout
	L0: in		al,dx
		and 	al,080h
		loopnz	L0
		jnz 	L1
		mov 	al,cData
		out 	dx,al
		mov 	blIOError,FALSE
	L1:
	}
}

void ResetMIX(void)
{
	__asm
	{
		mov 	dx,wBaseAddr
		add 	dx,004h
		xor 	al,al
		out 	dx,al
		inc 	dx
		inc 	ax
		out 	dx,al
		dec 	ax
		out 	dx,al
	}
}

BYTE ReadMIX(BYTE cIndex)
{
	BYTE retr;

	blIOError = FALSE;
	__asm
	{
		mov 	dx,wBaseAddr
		add 	dx,004h
		mov 	al,cIndex
		out 	dx,al
		inc 	dx
		in		al,dx
		mov 	retr,al
	}

	return retr;
}

void WriteMIX(BYTE cIndex, BYTE cData)
{
	blIOError = FALSE;
	__asm
	{
		mov 	dx,wBaseAddr
		add 	dx,004h
		mov 	al,cIndex
		out 	dx,al
		inc 	dx
		mov 	al,cData
		out 	dx,al
	}
}

BOOL isMixer(void)
{
    BYTE c0, c1;

    c0 = ReadMIX(0x04);
    WriteMIX(0x04, (BYTE)~c0);
    c1 = ReadMIX(0x04);
    WriteMIX(0x04, c0);

    return (c0 != c1);
}

BOOL isDSPxFD(void)
{
    BYTE c;

    ResetDSP();
    WriteDSP(0x10);
    WriteDSP(0x80);

    WriteDSP(0xFD);
    c = ReadDSP();

    ResetDSP();
    if (c != 0x10)
        return FALSE;


    WriteDSP(0x20);
    ReadDSP();

    WriteDSP(0xFD);
    c = ReadDSP();

    ResetDSP();
    return (c == 0x20);
}

BOOL TestDSPxE1(DSPC *pDSPC, int i)
{
    BYTE cDREQ8;
	BYTE cDDMA8Active, cDDMA8Inactive;
	int  j, k;

    pDSPC->wFlags = D_UNKNOWN;

    for (j = 0; (j < 0x10) && (!(pDSPC->wFlags & D_KNOWN)); j++)
    {
		ResetDSP();

		__asm
		{
			cli
			in		al,008h
			mov 	cDREQ8,al
            sti
        }

		WriteDSP((BYTE)i);

        if (blMixer)
            pDSPC->cX82  = ReadMIX(cMixerX);

		for (pDSPC->R0 = -1, blIOError = FALSE;
             (!blIOError) && (pDSPC->R0 != 0xFF); ReadDSP(), pDSPC->R0++)
            ;

		for (k = 0, pDSPC->W0 = 0, blIOError = FALSE;
			 (k < j) && (!blIOError); k++)
		{
			WriteDSP(cSeed);
			pDSPC->W0++;
		}

        if (blMixer)
            pDSPC->cX82 |= ReadMIX(cMixerX);

        for (pDSPC->R1 = -1, blIOError = FALSE;
             (!blIOError) && (pDSPC->R1 != 0xFF); ReadDSP(), pDSPC->R1++)
            ;

		__asm
		{
			cli

			in		al,008h
			mov 	ah,al
			xor 	al,cDREQ8
			and 	al,0B0h

			mov 	dh,al

			mov 	cl,cDMA8
			mov 	dl,001h
			add 	cl,004h
			shl 	dl,cl

			and 	al,ah
			and 	al,dl
			mov 	cDDMA8Active,al

			not 	ah
			and 	dh,ah
			and 	dh,dl
			mov 	cDDMA8Inactive,dh

            sti
        }

        WriteDSP(0xE1);
        if ((ReadDSP() == cMajorVersion) && (ReadDSP() == cMinorVersion) && (!blIOError))
        {
            pDSPC->wFlags |= D_KNOWN |
                             ((cDDMA8Active)    ? D_PDMA8HI  : 0x0000) |
                             ((cDDMA8Inactive)  ? D_PDMA8LO  : 0x0000);

            WriteDSP(0xD8);
            pDSPC->cFB |= ReadDSP() & 0x10;
            pDSPC->cFC  = 0x00;
		}
        else if ((pDSPC->R0 + pDSPC->R1) ||
                 (cDDMA8Active | cDDMA8Inactive))
        {
            pDSPC->wFlags |= D_KNOWN | D_WIERD |
                             ((cDDMA8Active)    ? D_PDMA8HI  : 0x0000) |
                             ((cDDMA8Inactive)  ? D_PDMA8LO  : 0x0000);

            pDSPC->cFB  = 0x00;
            pDSPC->cFC  = 0x00;
        }

        if ((pDSPC->R0 == 0xFF) && (pDSPC->R1 == 0xFF) && (!pDSPC->W0))
            pDSPC->R1 = 0x00;
    }

    return (pDSPC->wFlags & D_KNOWN != 0);
}

BOOL TestDSPxFD(DSPC *pDSPC, int i)
{
	BYTE cDREQ8, cDREQ16;
	BYTE cDDMA8Active, cDDMA8Inactive;
	BYTE cDDMA16Active, cDDMA16Inactive;
	BYTE cIRQ;
	int  j, k;

    pDSPC->wFlags = D_UNKNOWN;

    for (j = 0; (j < 0x10) && (!(pDSPC->wFlags & D_KNOWN)); j++)
	{
		ResetDSP();

		__asm
		{
			cli
			in		al,008h
			mov 	cDREQ8,al
			in		al,0D0h
			mov 	cDREQ16,al
			sti
		}

		WriteDSP((BYTE)i);

		Delay(3);
		cIRQ = ReadMIX(0x82);

		pDSPC->cX82 = ReadMIX(cMixerX);

        for (pDSPC->R0 = -1, blIOError = FALSE;
             (!blIOError) && (pDSPC->R0 != 0xFF); ReadDSP(), pDSPC->R0++)
            ;

		for (k = 0, pDSPC->W0 = 0, blIOError = FALSE;
			 (k < j) && (!blIOError); k++)
		{
			WriteDSP(cSeed);
			pDSPC->W0++;
		}

		Delay(3);
		cIRQ |= ReadMIX(0x82);

		pDSPC->cX82 |= ReadMIX(cMixerX);

        for (pDSPC->R1 = -1, blIOError = FALSE;
             (!blIOError) && (pDSPC->R1 != 0xFF); ReadDSP(), pDSPC->R1++)
            ;

		__asm
		{
			cli

			in		al,008h
			mov 	ah,al
			xor 	al,cDREQ8
			and 	al,0B0h

			mov 	dh,al

			mov 	cl,cDMA8
			mov 	dl,001h
			add 	cl,004h
			shl 	dl,cl

			and 	al,ah
			and 	al,dl
			mov 	cDDMA8Active,al

			not 	ah
			and 	dh,ah
			and 	dh,dl
			mov 	cDDMA8Inactive,dh


			in		al,0D0h
			mov 	ah,al
			xor 	al,cDREQ16
			and 	al,0E0h

			mov 	dh,al

			mov 	dl,001h
			mov 	cl,cDMA16
			shl 	dl,cl

			and 	al,ah
			and 	al,dl
			mov 	cDDMA16Active,al

			not 	ah
			and 	dh,ah
			and 	dh,dl
			mov 	cDDMA16Inactive,dh

			sti
		}

		WriteDSP(0xFD);
        if ((i == ReadDSP()) && (!blIOError))
        {
            // some of these flags may be invalid for some 0xFD DSPs
            pDSPC->wFlags |= D_KNOWN |
                             ((cIRQ & 0x0001)   ? D_IRQ8     : 0x0000) |
                             ((cIRQ & 0x0002)   ? D_IRQ16    : 0x0000) |
                             ((cIRQ & 0x0004)   ? D_IRQMPU   : 0x0000) |
                             ((cDDMA8Active)    ? D_PDMA8HI  : 0x0000) |
                             ((cDDMA8Inactive)  ? D_PDMA8LO  : 0x0000) |
                             ((cDDMA16Active)   ? D_PDMA16HI : 0x0000) |
                             ((cDDMA16Inactive) ? D_PDMA16LO : 0x0000);

            WriteDSP(0xFB);
            pDSPC->cFB  = ReadDSP();

            WriteDSP(0xFC);
            pDSPC->cFC  = ReadDSP();
		}
        else if ((pDSPC->R0 + pDSPC->R1) ||
                 (cDDMA8Active | cDDMA8Inactive | cDDMA16Active | cDDMA16Inactive))
        {
            // some of these flags may be invalid for some 0xFD DSPs
            pDSPC->wFlags |= D_KNOWN | D_WIERD |
                             ((cIRQ & 0x0001)   ? D_IRQ8     : 0x0000) |
                             ((cIRQ & 0x0002)   ? D_IRQ16    : 0x0000) |
                             ((cIRQ & 0x0004)   ? D_IRQMPU   : 0x0000) |
                             ((cDDMA8Active)    ? D_PDMA8HI  : 0x0000) |
                             ((cDDMA8Inactive)  ? D_PDMA8LO  : 0x0000) |
                             ((cDDMA16Active)   ? D_PDMA16HI : 0x0000) |
                             ((cDDMA16Inactive) ? D_PDMA16LO : 0x0000);

            pDSPC->cFB  = 0;
            pDSPC->cFC  = 0;
        }

        if ((pDSPC->R0 == 0xFF) && (pDSPC->R1 == 0xFF) && (!pDSPC->W0))
            pDSPC->R1 = 0x00;
	}

    return (pDSPC->wFlags & D_KNOWN != 0);
}

BYTE *PtrDSPC(int i, BYTE *pBuf)
{
    int   j;
    BYTE *p = pBuf;

    WriteDSP((BYTE)i);
    for (j = 0; j < tblDSPC[i].R0; *p = ReadDSP(), j++, p++)
        ;
    for (j = 0; j < tblDSPC[i].W0; WriteDSP(cSeed), j++)
        ;
    for (j = 0; j < tblDSPC[i].R1; *p = ReadDSP(), j++, p++)
        ;

    return pBuf;
}

void Delay(int nTicks)
{
	__asm
	{
		mov 	cx,nTicks
	L0: in		al,040h
		jmp 	L1
	L1: in		al,040h
		mov 	ah,al

	L2: in		al,040h
		jmp 	L3
	L3: in		al,040h
		cmp 	al,ah
		je		L2

		loop	L0
	}
}

void putslna(void)
{
	int i;

	for (i = 0; i < 79; i++)
        fputc(CHRDEF_LNA, fp);   // 'ÄÄÄ'
    fputc('\n', fp);
}

void putslnb(void)
{
	int i;

	for (i = 0; i < 79; i++)
        fputc(CHRDEF_LNB, fp);   // '---'
    fputc('\n', fp);
}

void putslnc(void)
{
	int i;

    fputc(' ', fp);
	for (i = 1; i < 79; i++)
        fputc(CHRDEF_LNC, fp);   // ''
    fputc('\n', fp);
}
