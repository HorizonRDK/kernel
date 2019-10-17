#ifndef ELPCURVE_H_
#define ELPCURVE_H_

#define CURVE_MAX_DATA_SIZE	256

typedef struct ec_curve_data_
{
   short size;
   unsigned char rr[CURVE_MAX_DATA_SIZE];
   unsigned char m[CURVE_MAX_DATA_SIZE];    //modulus m
   unsigned char mp[CURVE_MAX_DATA_SIZE];   // modular inverse of m (mod R)
   unsigned char x[CURVE_MAX_DATA_SIZE];    // Q_x  the x coordinate of the generator
   unsigned char y[CURVE_MAX_DATA_SIZE];    // Q_y  the y coordinate of the generator
   unsigned char z[CURVE_MAX_DATA_SIZE];    // Q_z   the z coordinate of the generator
   unsigned char r[CURVE_MAX_DATA_SIZE];    // R^2 mod m - used to convert an integer to an M-residue
   unsigned char a[CURVE_MAX_DATA_SIZE];    // domain parameter 'a'
   unsigned char b[CURVE_MAX_DATA_SIZE];    // domain parameter 'b'
   unsigned char order[CURVE_MAX_DATA_SIZE + 1];   // incase 160+1
   char *comment;                  // a short description of the curve
   unsigned char nr[CURVE_MAX_DATA_SIZE];   // R^2 mod n
   unsigned char np[CURVE_MAX_DATA_SIZE];   // inverse of n mod R
   unsigned char n[CURVE_MAX_DATA_SIZE];    // order of the curve
} EC_CURVE_DATA;

#endif
