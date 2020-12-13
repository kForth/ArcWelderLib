// This code was taken from https://github.com/miloyip/dtoa-benchmark
// And was modified to output a rounded string in decimal format
// with fixed precision.
// This is the license as of the time of this writing
/*
Copyright (C) 2014 Milo Yip

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// The license for the fpconv algorithm was taken from Milo's github here
// https://github.com/miloyip/dtoa-benchmark/blob/master/src/fpconv/license
// The text of the license appears below, from the time of this writing.
/*
The MIT License
Copyright(c) 2013 Andreas Samoljuk

Permission is hereby granted, free of charge, to any person obtaining
a copy of this softwareand associated documentation files(the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and /or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions :

The above copyright noticeand this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#ifndef FPCONV_H
#define FPCONV_H

#include <stdint.h>
/* Fast and accurate double to string conversion based on Florian Loitsch's
 * Grisu-algorithm[1].
 *
 * Input:
 * fp -> the double to convert, dest -> destination buffer.
 * The generated string will never be longer than 24 characters.
 * Make sure to pass a pointer to at least 24 bytes of memory.
 * The emitted string will not be null terminated.
 *
 * Output:
 * The number of written characters.
 *
 * Exemplary usage:
 *
 * void print(double d)
 * {
 *      char buf[24 + 1] // plus null terminator
 *      int str_len = fpconv_dtoa(d, buf);
 *
 *      buf[str_len] = '\0';
 *      printf("%s", buf);
 * }
 *
 */

int fpconv_dtoa(double fp, char dest[24]);
int fpconv_dtos(double fp, char dest[24], unsigned char precision);

#endif

/* [1] http://florian.loitsch.com/publications/dtoa-pldi2010.pdf */


#define npowers     87
#define steppowers  8
#define firstpower -348 /* 10 ^ -348 */

#define expmax     -32
#define expmin     -60


typedef struct Fp {
  uint64_t frac;
  int exp;
} Fp;

static Fp powers_ten[] = {
    { 18054884314459144840U, -1220 }, { 13451937075301367670U, -1193 },
    { 10022474136428063862U, -1166 }, { 14934650266808366570U, -1140 },
    { 11127181549972568877U, -1113 }, { 16580792590934885855U, -1087 },
    { 12353653155963782858U, -1060 }, { 18408377700990114895U, -1034 },
    { 13715310171984221708U, -1007 }, { 10218702384817765436U, -980 },
    { 15227053142812498563U, -954 },  { 11345038669416679861U, -927 },
    { 16905424996341287883U, -901 },  { 12595523146049147757U, -874 },
    { 9384396036005875287U,  -847 },  { 13983839803942852151U, -821 },
    { 10418772551374772303U, -794 },  { 15525180923007089351U, -768 },
    { 11567161174868858868U, -741 },  { 17236413322193710309U, -715 },
    { 12842128665889583758U, -688 },  { 9568131466127621947U,  -661 },
    { 14257626930069360058U, -635 },  { 10622759856335341974U, -608 },
    { 15829145694278690180U, -582 },  { 11793632577567316726U, -555 },
    { 17573882009934360870U, -529 },  { 13093562431584567480U, -502 },
    { 9755464219737475723U,  -475 },  { 14536774485912137811U, -449 },
    { 10830740992659433045U, -422 },  { 16139061738043178685U, -396 },
    { 12024538023802026127U, -369 },  { 17917957937422433684U, -343 },
    { 13349918974505688015U, -316 },  { 9946464728195732843U,  -289 },
    { 14821387422376473014U, -263 },  { 11042794154864902060U, -236 },
    { 16455045573212060422U, -210 },  { 12259964326927110867U, -183 },
    { 18268770466636286478U, -157 },  { 13611294676837538539U, -130 },
    { 10141204801825835212U, -103 },  { 15111572745182864684U, -77 },
    { 11258999068426240000U, -50 },   { 16777216000000000000U, -24 },
    { 12500000000000000000U,   3 },   { 9313225746154785156U,   30 },
    { 13877787807814456755U,  56 },   { 10339757656912845936U,  83 },
    { 15407439555097886824U, 109 },   { 11479437019748901445U, 136 },
    { 17105694144590052135U, 162 },   { 12744735289059618216U, 189 },
    { 9495567745759798747U,  216 },   { 14149498560666738074U, 242 },
    { 10542197943230523224U, 269 },   { 15709099088952724970U, 295 },
    { 11704190886730495818U, 322 },   { 17440603504673385349U, 348 },
    { 12994262207056124023U, 375 },   { 9681479787123295682U,  402 },
    { 14426529090290212157U, 428 },   { 10748601772107342003U, 455 },
    { 16016664761464807395U, 481 },   { 11933345169920330789U, 508 },
    { 17782069995880619868U, 534 },   { 13248674568444952270U, 561 },
    { 9871031767461413346U,  588 },   { 14708983551653345445U, 614 },
    { 10959046745042015199U, 641 },   { 16330252207878254650U, 667 },
    { 12166986024289022870U, 694 },   { 18130221999122236476U, 720 },
    { 13508068024458167312U, 747 },   { 10064294952495520794U, 774 },
    { 14996968138956309548U, 800 },   { 11173611982879273257U, 827 },
    { 16649979327439178909U, 853 },   { 12405201291620119593U, 880 },
    { 9242595204427927429U,  907 },   { 13772540099066387757U, 933 },
    { 10261342003245940623U, 960 },   { 15290591125556738113U, 986 },
    { 11392378155556871081U, 1013 },  { 16975966327722178521U, 1039 },
    { 12648080533535911531U, 1066 }
};

static Fp find_cachedpow10(int exp, int* k)
{
  const double one_log_ten = 0.30102999566398114;

  int approx = -(exp + npowers) * one_log_ten;
  int idx = (approx - firstpower) / steppowers;

  while (1) {
    int current = exp + powers_ten[idx].exp + 64;

    if (current < expmin) {
      idx++;
      continue;
    }

    if (current > expmax) {
      idx--;
      continue;
    }

    *k = (firstpower + idx * steppowers);

    return powers_ten[idx];
  }
}