#ifndef DEFINE_H
#define DEFINE_H


//ceiling-map-2009-01-19_b.pgm PNM 657x343 657x343+0+0 PseudoClass 256c 8-bit 220.084kb 
#ifndef MAP // added 20110629 k-okada
//#define MAP      "ceiling-map-2009-01-19_b.pgm"//奈良先端地図
#define MAP      "rotRTCcenterCeiling.BMP"//再利用センター地図
#endif
#ifndef MAP_SHOW // added 20110629 k-okada
//#define MAP_SHOW "ceiling-map-2009-01-19_c.png"//奈良先端地図
#define MAP_SHOW "rotRTCcenterCeiling.JPG"//再利用センター地図
#endif
#define MAP_X (460)
#define MAP_Y (240)

#define SEARCH_PAD (8)

#define TEMPLATE_SIZE (96)

#define SAVE_TRACK  "map_tracking/"
#define LOAD_TRACK  "capture_data/"

#define USE_UNDISTORT

#endif
