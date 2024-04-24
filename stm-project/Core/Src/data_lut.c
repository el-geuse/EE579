/*
 * data_lut.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Jonas
 *
 * Credits to MetallicPriest: https://stackoverflow.com/questions/7091294/how-to-build-a-lookup-table-in-c-sdcc-compiler-with-linear-interpolation
 *
 */

typedef struct {float x; float y;} coord_t;

float interp(coord_t *c, float x, int n )
{
    int i;

    for( i = 0; i < n-1; i++ )
    {
        if ( c[i].x <= x && c[i+1].x >= x )
        {
        	float diffx = x - c[i].x;
        	float diffn = c[i+1].x - c[i].x;

            return c[i].y + ( c[i+1].y - c[i].y ) * diffx / diffn;
        }
    }

    return 0; // Not in Range
}
