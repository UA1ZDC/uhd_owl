//
// Copyright 2014 Ettus Research
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef INCLUDED_AD9361_FILTER_TAPS_HPP
#define INCLUDED_AD9361_FILTER_TAPS_HPP

#include <boost/cstdint.hpp>

/* A default 128-tap filter that can be used for generic circumstances. */
/* static uint16_t default_128tap_coeffs[] = {
    0x0001,0xfff1,0xffcf,0xffc0,0xffe8,0x0020,0x001a,0xffe3,
    0xffe1,0x001f,0x0028,0xffdf,0xffcc,0x0024,0x0043,0xffdb,
    0xffac,0x0026,0x0068,0xffdb,0xff80,0x0022,0x009a,0xffe2,
    0xff47,0x0017,0x00db,0xfff3,0xfeff,0xffff,0x012b,0x0013,
    0xfea5,0xffd7,0x0190,0x0046,0xfe35,0xff97,0x020e,0x0095,
    0xfda7,0xff36,0x02ae,0x010d,0xfcf0,0xfea1,0x0383,0x01c6,
    0xfbf3,0xfdb6,0x04b7,0x02f8,0xfa6d,0xfc1a,0x06be,0x0541,
    0xf787,0xf898,0x0b60,0x0b6d,0xee88,0xea40,0x2786,0x7209
};
*/

/* The below pair of filters is from ADI and "optimized for a 10MHz LTE application". */
/*
static uint16_t lte10mhz_rx_coeffs[] = {
    0xffe2,0x0042,0x0024,0x0095,0x0056,0x004d,0xffcf,0xffb7,
    0xffb1,0x0019,0x0059,0x006a,0x0004,0xff9d,0xff72,0xffd4,
    0x0063,0x00b7,0x0062,0xffac,0xff21,0xff59,0x0032,0x0101,
    0x00f8,0x0008,0xfeea,0xfeac,0xffa3,0x0117,0x01b5,0x00d0,
    0xff05,0xfdea,0xfe9e,0x00ba,0x026f,0x0215,0xffb5,0xfd4a,
    0xfd18,0xffa0,0x02de,0x03dc,0x0155,0xfd2a,0xfb0d,0xfd54,
    0x0287,0x062f,0x048a,0xfe37,0xf862,0xf8c1,0x004d,0x0963,
    0x0b88,0x02a4,0xf3e7,0xebdd,0xf5f8,0x1366,0x3830,0x518b
};

static uint16_t lte10mhz_tx_coeffs[] = {
    0xfffb,0x0000,0x0004,0x0017,0x0024,0x0028,0x0013,0xfff3,
    0xffdc,0xffe5,0x000b,0x0030,0x002e,0xfffe,0xffc4,0xffb8,
    0xfff0,0x0045,0x0068,0x002b,0xffb6,0xff72,0xffad,0x0047,
    0x00b8,0x0088,0xffc8,0xff1c,0xff33,0x001a,0x0110,0x0124,
    0x0019,0xfec8,0xfe74,0xff9a,0x0156,0x0208,0x00d3,0xfe9b,
    0xfd68,0xfe96,0x015d,0x033f,0x0236,0xfecd,0xfc00,0xfcb5,
    0x00d7,0x04e5,0x04cc,0xffd5,0xf9fe,0xf8fb,0xfef2,0x078c,
    0x0aae,0x036d,0xf5c0,0xed89,0xf685,0x12af,0x36a4,0x4faa
};
*/

/************************************************************/
/* These filters suitable for decimation/interpolation by 2 */
/************************************************************/

/* 127 tap Halfband designed with: round(2^16 * halfgen4(0.9/4,32)) (center tap tweaked to 32767) */
static boost::int16_t hb127_coeffs[] = {
  -0,0,1,-0,-2,0,3,-0,-5,0,8,-0,-11,0,17,-0,-24,0,33,-0,-45,0,61,-0,-80,0,104,-0,-134,0,169,-0,
  -213,0,264,-0,-327,0,401,-0,-489,0,595,-0,-724,0,880,-0,-1075,0,1323,-0,-1652,0,2114,-0,-2819,0,4056,-0,-6883,0,20837,32767,
  20837,0,-6883,-0,4056,0,-2819,-0,2114,0,-1652,-0,1323,0,-1075,-0,880,0,-724,-0,595,0,-489,-0,401,0,-327,-0,264,0,-213,-0,
  169,0,-134,-0,104,0,-80,-0,61,0,-45,-0,33,0,-24,-0,17,0,-11,-0,8,0,-5,-0,3,0,-2,-0,1,0,-0, 0 };

/* 95 tap Halfband designed with: round(2^16 * halfgen4(0.9/4,24)) (center tap tweaked to 32767) */
static boost::int16_t hb95_coeffs[] = {
  -4,0,8,-0,-14,0,23,-0,-36,0,52,-0,-75,0,104,-0,-140,0,186,-0,-243,0,314,-0,-400,0,505,-0,-634,0,793,-0,
  -993,0,1247,-0,-1585,0,2056,-0,-2773,0,4022,-0,-6862,0,20830,32767,20830,0,-6862,-0,4022,0,-2773,-0,2056,0,-1585,-0,1247,0,-993,-0,
  793,0,-634,-0,505,0,-400,-0,314,0,-243,-0,186,0,-140,-0,104,0,-75,-0,52,0,-36,-0,23,0,-14,-0,8,0,-4,0};

/* 63 tap Halfband designed with: round(2^16 * halfgen4(0.9/4,16)) (center tap tweaked to 32767) */
static boost::int16_t hb63_coeffs[] = {
  -58,0,83,-0,-127,0,185,-0,-262,0,361,-0,-488,0,648,-0,-853,0,1117,-0,-1466,0,1954,-0,-2689,0,3960,-0,-6825,0,20818,32767,
  20818,0,-6825,-0,3960,0,-2689,-0,1954,0,-1466,-0,1117,0,-853,-0,648,0,-488,-0,361,0,-262,-0,185,0,-127,-0,83,0,-58,0};

/* 47 tap Halfband designed with: round(2^16 * halfgen4(0.85/4,12)) (center tap tweaked to 32767) */
static boost::int16_t hb47_coeffs[] = {
  -50,0,98,-0,-181,0,307,-0,-489,0,747,-0,-1109,0,1628,-0,-2413,0,3750,-0,-6693,0,20773,32767,20773,0,-6693,-0,3750,0,-2413,-0,
  1628,0,-1109,-0,747,0,-489,-0,307,0,-181,-0,98,0,-50,0};

/************************************************************/
/* These filters suitable for decimation/interpolation by 4 */
/* Designed for -3dB rolloff @ Fs/4                         */
/************************************************************/

/* 128 tap equiripple FIR low-pass designed with: round(2^16 * fir1(127,0.25)); */
static boost::int16_t fir_128_x4_coeffs[] = {
  -15,-27,-23,-6,17,33,31,9,-23,-47,-45,-13,34,69,67,21,-49,-102,-99,-32,69,146,143,48,-96,-204,-200,-69,129,278,275,97,-170,
  -372,-371,-135,222,494,497,187,-288,-654,-665,-258,376,875,902,363,-500,-1201,-1265,-530,699,1748,1906,845,-1089,-2922,-3424,
  -1697,2326,7714,12821,15921,15921,12821,7714,2326,-1697,-3424,-2922,-1089,845,1906,1748,699,-530,-1265,-1201,-500,363,902,875,
  376,-258,-665,-654,-288,187,497,494,222,-135,-371,-372,-170,97,275,278,129,-69,-200,-204,-96,48,143,146,69,-32,-99,-102,-49,21,
  67,69,34,-13,-45,-47,-23,9,31,33,17,-6,-23,-27,-15};

/* 96 tap equiripple FIR low-pass designed with: round(2^16 * fir1(95,0.25)); */
static boost::int16_t fir_96_x4_coeffs[] = {
  -18,-35,-33,-11,23,50,51,18,-37,-83,-86,-31,62,140,145,54,-98,-224,-232,-88,149,343,356,138,-218,-509,-530,-211,313,743,781,
  320,-447,-1089,-1163,-494,658,1663,1830,819,-1062,-2868,-3379,-1682,2314,7695,12812,15924,15924,12812,7695,2314,-1682,-3379,
  -2868,-1062,819,1830,1663,658,-494,-1163,-1089,-447,320,781,743,313,-211,-530,-509,-218,138,356,343,149,-88,-232,-224,-98,54,
  145,140,62,-31,-86,-83,-37,18,51,50,23,-11,-33,-35,-18};

/* 64 tap equiripple FIR low-pass designed with: round(2^16 * fir1(63,0.25)); */
static boost::int16_t fir_64_x4_coeffs[] = {
  -25,-54,-56,-22,41,102,117,50,-87,-223,-253,-109,174,443,496,215,-317,-809,-903,-398,550,1434,1623,744,-987,-2715,-3251,
  -1640,2279,7638,12782,15928,15928,12782,7638,2279,-1640,-3251,-2715,-987,744,1623,1434,550,-398,-903,-809,-317,215,496,
  443,174,-109,-253,-223,-87,50,117,102,41,-22,-56,-54,-25};

 /* 48 tap equiripple FIR low-pass designed with: round(2^16 * fir1(47,0.25)); */
static boost::int16_t fir_48_x4_coeffs[] = {
  -32,-74,-84,-39,68,191,237,114,-183,-508,-609,-287,419,1149,1358,647,-887,-2508,-3073,-1580,2230,7555,12736,15928,15928,
  12736,7555,2230,-1580,-3073,-2508,-887,647,1358,1149,419,-287,-609,-508,-183,114,237,191,68,-39,-84,-74,-32};

/* NOTE: To write coeffs directly from Octave in a format that's useful in C use: dlmwrite ("file.csv",bb,","); */

#endif // INCLUDED_AD9361_FILTER_TAPS_HPP