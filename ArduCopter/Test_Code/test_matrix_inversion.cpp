#include <stdio.h>

bool inverse3x3(float m[], float invOut[])
{
    float inv[9];
    // computes the inverse of a matrix m
    float  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);


    float invdet = 1 / det;

    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

    for(int i = 0; i < 9; i++){
        invOut[i] = inv[i];
    }

    return true;
}

bool inverse4x4(float m[],float invOut[])
{
    float inv[16], det;
    int i;


    inv[0] = m[5]  * m[10] * m[15] -
    m[5]  * m[11] * m[14] -
    m[9]  * m[6]  * m[15] +
    m[9]  * m[7]  * m[14] +
    m[13] * m[6]  * m[11] -
    m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
    m[4]  * m[11] * m[14] +
    m[8]  * m[6]  * m[15] -
    m[8]  * m[7]  * m[14] -
    m[12] * m[6]  * m[11] +
    m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
    m[4]  * m[11] * m[13] -
    m[8]  * m[5] * m[15] +
    m[8]  * m[7] * m[13] +
    m[12] * m[5] * m[11] -
    m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
    m[4]  * m[10] * m[13] +
    m[8]  * m[5] * m[14] -
    m[8]  * m[6] * m[13] -
    m[12] * m[5] * m[10] +
    m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
    m[1]  * m[11] * m[14] +
    m[9]  * m[2] * m[15] -
    m[9]  * m[3] * m[14] -
    m[13] * m[2] * m[11] +
    m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
    m[0]  * m[11] * m[14] -
    m[8]  * m[2] * m[15] +
    m[8]  * m[3] * m[14] +
    m[12] * m[2] * m[11] -
    m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
    m[0]  * m[11] * m[13] +
    m[8]  * m[1] * m[15] -
    m[8]  * m[3] * m[13] -
    m[12] * m[1] * m[11] +
    m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
    m[0]  * m[10] * m[13] -
    m[8]  * m[1] * m[14] +
    m[8]  * m[2] * m[13] +
    m[12] * m[1] * m[10] -
    m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
    m[1]  * m[7] * m[14] -
    m[5]  * m[2] * m[15] +
    m[5]  * m[3] * m[14] +
    m[13] * m[2] * m[7] -
    m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
    m[0]  * m[7] * m[14] +
    m[4]  * m[2] * m[15] -
    m[4]  * m[3] * m[14] -
    m[12] * m[2] * m[7] +
    m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
    m[0]  * m[7] * m[13] -
    m[4]  * m[1] * m[15] +
    m[4]  * m[3] * m[13] +
    m[12] * m[1] * m[7] -
    m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
    m[0]  * m[6] * m[13] +
    m[4]  * m[1] * m[14] -
    m[4]  * m[2] * m[13] -
    m[12] * m[1] * m[6] +
    m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
    m[1] * m[7] * m[10] +
    m[5] * m[2] * m[11] -
    m[5] * m[3] * m[10] -
    m[9] * m[2] * m[7] +
    m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
    m[0] * m[7] * m[10] -
    m[4] * m[2] * m[11] +
    m[4] * m[3] * m[10] +
    m[8] * m[2] * m[7] -
    m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
    m[0] * m[7] * m[9] +
    m[4] * m[1] * m[11] -
    m[4] * m[3] * m[9] -
    m[8] * m[1] * m[7] +
    m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
    m[0] * m[6] * m[9] -
    m[4] * m[1] * m[10] +
    m[4] * m[2] * m[9] +
    m[8] * m[1] * m[6] -
    m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0) {
            return false;
        }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}


main()
{
    float test_input[16] = {1,2,4,4,5,6,7,8,9,10,11,1,13,14,5,16};
    float test_output[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    float Matrix_A[4][4]=
	{
	    {1, 0, 6, 0},
	    {0, 1, 0, 7},
	    {0, 0, 1, 0},
	    {0, 0, 0, 1}
	};

//    inverse4x4(test_input, test_output);
//
//    for(int i=0; i<16; i++)
//    {   
//        double printout = test_output[i];
//        printf("%f  ", printout);
//    }

    float test_input_3[9] = {1,2,4,4,5,1,13,14,5};
    float test_output_3[9] = {1,2,3,4,13,14,15,16,8};

    if(inverse3x3(test_input_3, test_output_3) == true)
    {
        for(int i=0; i<9; i++)
        {   
            double printout = test_output_3[i];
            printf("%f  ", printout);
        }
    }

}


