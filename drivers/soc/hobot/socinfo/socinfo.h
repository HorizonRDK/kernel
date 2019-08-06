#ifndef SOCINFO_H
#define SOCINFO_H

enum hobot_board {
	X2_SVB = 100,
	J2_SVB = 200,
	J2SOM = 201,
	X2SOM1V8 = 102,
	X2SOM3V3 = 103,
	X2_DEV = 101,
	X2_SOMFULL = 104,
	X2_96BOARD = 105,
	J2_Mono = 202,
	J2_DEV = 203,
	J2_SK = 204,
	J2_SAM = 205,
	J2_QuadJ2A = 301,
	J2_QuadJ2B,
	J2_QuadJ2C,
	J2_QuadJ2D,
	J2_Quad = 300,
	J2_mm = 400,
	Unknown = 1000,
};

struct hobot_board_info {
	enum hobot_board generic_board_type;
	char *board_id_string;
};

#endif
