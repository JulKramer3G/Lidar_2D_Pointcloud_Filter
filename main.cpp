//
//  main.cpp
//  C++ Compiler Test
//
//  Created by Julius Kramer on 01.01.21.
//

#include <iostream>

#include "filter.hpp"

#include <list>

#define ELEMENTS 352

/*
 Angles are here *1000, because my lidar returns its angular data in that way. Angular Unit is Degrees. Distances are in mm.
 */

Point_Polar input[] = {
{3514, 976},
{3493, 1606},
{3509, 3055},
{3516, 4047},
{3539, 5134},
{3531, 6079},
{3533, 7182},
{3532, 8127},
{3523, 9103},
{3516, 10174},
{3511, 11119},
{3514, 12222},
{3519, 13167},
{3511, 14143},
{3510, 15214},
{3508, 16159},
{3511, 17262},
{3506, 18207},
{3511, 19183},
{3502, 20270},
{3513, 21215},
{3505, 22333},
{3484, 23294},
{3494, 24270},
{3491, 25373},
{3448, 25861},
{3516, 26964},
{3540, 27924},
{3468, 29043},
{3522, 30003},
{3533, 30980},
{3584, 32082},
{3556, 33043},
{3642, 34161},
{3698, 35138},
{3689, 36099},
{3778, 37217},
{3855, 38178},
{3907, 39170},
{3971, 40257},
{4069, 41218},
{4078, 42336},
{4024, 43297},
{4082, 44289},
{4108, 45376},
{4055, 46336},
{4280, 47455},
{4060, 48415},
{4160, 49408},
{4188, 50510},
{3819, 51471},
{4078, 52589},
{4490, 53550},
{1861, 54526},
{3727, 55629},
{1613, 56590},
{4413, 57708},
{4064, 58669},
{1131, 59645},
{2761, 60748},
{878, 61708},
{464, 62827},
{639, 63803},
{4525, 64764},
{4781, 65882},
{4408, 66843},
{4942, 67835},
{4938, 68922},
{4686, 69883},
{4813, 71001},
{824, 71962},
{1397, 72954},
{1515, 74529},
{1560, 75490},
{1574, 76482},
{1582, 77569},
{1579, 78514},
{1579, 79616},
{1579, 80561},
{1581, 81538},
{1584, 82609},
{1583, 83554},
{1581, 84656},
{1583, 85601},
{1590, 86578},
{1585, 87649},
{1589, 88594},
{1592, 89712},
{1592, 90657},
{1591, 91618},
{1590, 92705},
{1592, 93650},
{1595, 94752},
{1599, 95697},
{1602, 96658},
{1603, 97745},
{1612, 98690},
{1613, 99792},
{1617, 100737},
{1624, 101698},
{1628, 102785},
{1636, 103730},
{1642, 104832},
{1651, 105777},
{1659, 106738},
{1662, 107825},
{1672, 108770},
{1680, 109872},
{1683, 110817},
{1694, 111778},
{1703, 112865},
{1712, 113810},
{1720, 114912},
{1732, 115857},
{1745, 116818},
{1757, 117905},
{1768, 118865},
{1781, 119952},
{1798, 120913},
{1818, 121858},
{1836, 122961},
{1852, 123906},
{1877, 124882},
{1907, 125969},
{1942, 126930},
{1968, 128048},
{2005, 129009},
{2040, 130001},
{2074, 131088},
{2110, 132048},
{2161, 133167},
{2200, 134127},
{2250, 135120},
{2298, 136206},
{2355, 137167},
{2388, 138285},
{2440, 139246},
{2499, 140238},
{2546, 141325},
{2570, 142286},
{2572, 143404},
{2622, 144365},
{2621, 145357},
{2626, 146444},
{2641, 147405},
{2644, 148539},
{2630, 149484},
{2616, 150476},
{2615, 151578},
{2588, 152539},
{2552, 153657},
{2531, 154618},
{2521, 155595},
{2498, 156697},
{2489, 157658},
{2469, 158776},
{2464, 159737},
{2465, 160714},
{2444, 161816},
{2437, 162777},
{2424, 163895},
{2412, 164856},
{2413, 165832},
{2405, 166935},
{2394, 167896},
{2373, 168541},
{2325, 169975},
{2307, 170951},
{2299, 172054},
{2289, 173014},
{2280, 174133},
{2275, 175093},
{2265, 176070},
{2259, 177172},
{2246, 178133},
{2241, 179251},
{2245, 180228},
{2242, 181189},
{2247, 182307},
{2255, 183268},
{2263, 184260},
{2271, 185347},
{2279, 186307},
{2289, 187426},
{2292, 188386},
{2299, 189379},
{2291, 190465},
{2271, 191426},
{2273, 192072},
{2264, 193017},
{2269, 194151},
{2281, 195112},
{2278, 196088},
{2297, 197191},
{2427, 198152},
{2515, 199270},
{2506, 200231},
{2619, 201207},
{2592, 202310},
{2562, 203270},
{2636, 204389},
{2642, 205349},
{2653, 206326},
{2641, 207428},
{2642, 208389},
{2689, 209507},
{2681, 210484},
{2646, 211445},
{2659, 212563},
{2651, 213524},
{2667, 214973},
{2672, 216091},
{2665, 216563},
{2645, 217682},
{2653, 219131},
{2645, 220091},
{2617, 221194},
{2575, 222155},
{2534, 223147},
{2483, 224234},
{2462, 225194},
{2428, 226313},
{2410, 227273},
{2395, 228266},
{2373, 229352},
{2353, 230313},
{2328, 231431},
{2300, 232392},
{2269, 233384},
{2250, 234471},
{2231, 235432},
{2207, 236566},
{2192, 237527},
{2170, 238503},
{2147, 239606},
{2127, 240567},
{2108, 241685},
{2090, 242646},
{2075, 243622},
{2053, 244709},
{2036, 245654},
{2022, 246756},
{2006, 247701},
{1992, 248662},
{1983, 249749},
{1970, 250694},
{1955, 251796},
{1940, 252741},
{1938, 253702},
{1921, 254789},
{1911, 255734},
{1904, 256836},
{1898, 257781},
{1896, 258742},
{1887, 259829},
{1884, 260774},
{1877, 261876},
{1876, 262821},
{1873, 263782},
{1863, 264885},
{1857, 265830},
{1846, 266932},
{1828, 267909},
{1809, 268854},
{1790, 269956},
{1768, 270917},
{1751, 271909},
{1734, 272996},
{1710, 273957},
{1692, 275075},
{1677, 276036},
{1662, 277028},
{1648, 278115},
{1631, 279076},
{1616, 280194},
{1603, 281155},
{1594, 282147},
{1587, 283234},
{1585, 284194},
{1590, 285313},
{1600, 286273},
{1611, 287266},
{1633, 288352},
{1662, 289313},
{1691, 290431},
{1725, 291392},
{1762, 292384},
{1807, 293471},
{1850, 294432},
{1898, 295550},
{1943, 296511},
{1990, 297503},
{2033, 298606},
{2053, 299566},
{2092, 300685},
{2114, 301645},
{2137, 302622},
{2160, 303724},
{2170, 304685},
{2191, 305803},
{2205, 306764},
{2216, 307741},
{2229, 308843},
{2252, 309804},
{2272, 310922},
{2291, 311883},
{2332, 312859},
{2348, 313474},
{2443, 314450},
{2593, 315568},
{2934, 316513},
{2979, 317632},
{3023, 318608},
{3108, 319569},
{3023, 320687},
{3059, 321648},
{3506, 322640},
{3593, 323727},
{3654, 324688},
{3705, 325806},
{3716, 326767},
{3787, 327759},
{3834, 329334},
{3829, 329807},
{3867, 331287},
{3854, 332374},
{3849, 333335},
{3807, 334453},
{3793, 335414},
{3778, 336406},
{3737, 337493},
{3708, 338453},
{3697, 339572},
{3658, 340532},
{3632, 341525},
{3617, 342611},
{3591, 343572},
{3593, 344690},
{3573, 345651},
{3580, 346643},
{3529, 347730},
{3522, 348691},
{3494, 349825},
{3467, 350770},
{3425, 351762},
{3411, 352865},
{3405, 353825},
{3390, 354944},
{3401, 355416},
{3421, 356534},
{3433, 357495},
{3476, 358472},
{3477, 359574}
};

int main(int argc, const char * argv[]) {
    std::cout << "Filter test start! Filter algorithm created by Julius Kramer.\n";
    
    std::list<Point_Polar> points;
    for(uint32_t i = 0; i < ELEMENTS; i += 1) {
        input[i].angle /= 1000; // Convert angles to floats, because the Lidar module outputs everything multiplied by 1000
        points.push_back(input[i]);
    }
    Filter myFilter(20, 25); // Smoothing period 20, filter threshold 25
    
    std::list<Point_Cartesian> filtered = myFilter.filterDataset(points);
    
    for(Point_Cartesian point : filtered) {
        std::cout << "x: " << point.x << '\t' << "y: " << point.y << '\n';
    }
    
    std::cout << "Done. thank you.\n";
    
    return 0;
}