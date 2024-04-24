/*
 * data_lut.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Jonas
 *
 * Credits to MetallicPriest: https://stackoverflow.com/questions/7091294/how-to-build-a-lookup-table-in-c-sdcc-compiler-with-linear-interpolation
 *
 */

typedef struct { double x; double y; } coord_t;
double interp(coord_t *c, double x, int n);


double interp(coord_t *c, double x, int n )
{
    int i;

    for( i = 0; i < n-1; i++ )
    {
        if ( c[i].x <= x && c[i+1].x >= x )
        {
            double diffx = x - c[i].x;
            double diffn = c[i+1].x - c[i].x;

            return c[i].y + ( c[i+1].y - c[i].y ) * diffx / diffn;
        }
    }

    return 0; // Not in Range
}
