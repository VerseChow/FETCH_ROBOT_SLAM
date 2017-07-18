


#include "stdlib.h"
#include "stdio.h"
#include "time.h" 

#include "cmath"

float sample(float sigma)
{
	float x1, x2, w, y1;
	float m = 0;
	static float y2;
	static int use_last = 0;

	if (use_last)		        /* use value from previous call */
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = (float)(rand() % 100)/100;
    		x2 = (float)(rand() % 100)/100;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	return( m + y1 * sigma );



}

int main(int argc, char **argv)
{


	for(int i=1;i<100;i++)
	{ //sample(1);

		printf("%f\n", sample(1));
	}
	while(1)
	{

	}

}

