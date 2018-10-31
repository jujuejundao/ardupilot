#include <stdio.h>
#include <math.h>

#define PI 3.1415926

int main(){
	double x, y, z;
	double deg_rad = 180/PI;

	z = atan(-0.1);
	z = z * deg_rad;
	printf("%lf\n", z);
	x = 30/deg_rad;
	y = sin(x);
	printf("%lf\n", y);
}
