#include <stdio.h>
#include <math.h>

float dt_est = 0.03333333333333333;
float Mea_Distance_err = 0.1;
float Mea_Speed_err = 1.2;
float Mea_Angle_err = 5;
float Pro_Distance_err_x = 0.1;
float Pro_Distance_err_y = 0.1;
float Pro_Speed_err_x = 1.2;
float Pro_Speed_err_y = 1.2;
float sum = 0;
float err_t = 0.1;

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
            printf("matrix inversion failed `");
            return false;
        }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}

float X_now_1[4][1]=
{
    {10},
    {0},
    {0},
    {0}

};

float X_now[4][1]=
{
    {10},
    {0},
    {0},
    {0}
};

float X_now_kp[4][1]=
{
    {10},
    {0},
    {0},
    {0}
};


float Y_mea[4][1]=
{
    {100},
    {0},
    {0},
    {0}
};

float Matrix_A[4][4]=
{
    {1, 0, dt_est, 0},
    {0, 1, 0, dt_est},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};

float Matrix_A_T[4][4]=
{
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {dt_est, 0, 1, 0},
    {0, dt_est, 0, 1}
};

float Matrix_B[4][2]=
{
    {0.5*dt_est*dt_est, 0},
    {0, 0.5*dt_est*dt_est},
    {dt_est, 0},
    {0, dt_est}
};

//Measured Covariance Matrix
float R_now[4][4]=
{
    {Mea_Distance_err*Mea_Distance_err,0,0},
    {0, Mea_Speed_err*Mea_Speed_err, 0},
    {0, 0, Mea_Angle_err*Mea_Angle_err}
};

//Predicted Covariance matrix
float Q_now[4][4]=
{
    {err_t*err_t*dt_est*dt_est*dt_est*dt_est/4,0,err_t*err_t*dt_est*dt_est*dt_est/2,0},
    {0, err_t*err_t*dt_est*dt_est*dt_est*dt_est/4, 0, err_t*err_t*dt_est*dt_est*dt_est/2},
    {err_t*err_t*dt_est*dt_est/2, 0, err_t*err_t*dt_est, 0},
    {0, err_t*err_t*dt_est*dt_est/2, 0, err_t*err_t*dt_est}
};

float P_now_kp[4][4]=
{
    {Pro_Distance_err_x*Pro_Distance_err_x,0,0,0},
    {0, Pro_Distance_err_y*Pro_Distance_err_y, 0, 0},
    {0, 0, Pro_Speed_err_x*Pro_Speed_err_x, 0},
    {0, 0, 0, Pro_Speed_err_y*Pro_Speed_err_y}
};

float P_now[4][4]=
{
    {Pro_Distance_err_x*Pro_Distance_err_x,0,0,0},
    {0, Pro_Distance_err_y*Pro_Distance_err_y, 0, 0},
    {0, 0, Pro_Speed_err_x*Pro_Speed_err_x, 0},
    {0, 0, 0, Pro_Speed_err_y*Pro_Speed_err_y}
};

float P_now_1[4][4]=
{
    {Pro_Distance_err_x*Pro_Distance_err_x,0,0,0},
    {0, Pro_Distance_err_y*Pro_Distance_err_y, 0, 0},
    {0, 0, Pro_Speed_err_x*Pro_Speed_err_x, 0},
    {0, 0, 0, Pro_Speed_err_y*Pro_Speed_err_y}
};

float Matrix_H[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Matrix_H_T[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Kalman_Gain[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Kalman_Gain_T[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Four_Four_temp[4][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

float Four_Four_temp_1[4][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

float Four_One_temp[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};

float Four_One_temp_1[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};

float Four_Four_I[4][4]=
{
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
};

float Matrix_Acc[2][1]=
{
    {0},
    {0}
};

float Three_Three_temp_00[16];
float Three_Three_temp_11[16];

float rad_deg = 180/3.14159265;
float deg_rad = 3.14159265/180;

float Est_Target_Distance_x, Est_Target_Distance_y, Est_Target_Speed_x, Est_Target_Speed_y;
float debug_counter_1, debug_counter_2, debug_counter_3;
float Mea_Speed_0;


bool radar_estimator(float Mea_Distance, float Mea_Speed, float Mea_Angle, float Mea_Speed_0)
{


//Compute the predicted state: X_now_kp = Matrix_A * X_now_1 + Matrix_B*acc_cal;
    sum = 0;
    for(int c=0; c<4;c++){
         for(int d=0; d<1; d++){
            for(int k=0; k<4; k++){
                sum= sum + Matrix_A[c][k] * X_now_1[k][d];
            }
            X_now_kp[c][d]= sum;
            sum = 0;
        }

    }


//Compute Pkp covariance matrix: P_now_kp = Matrix_A*P_now_1*Matrix_A_T + Matrix_Q

    sum = 0;
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Matrix_A[c][k] * P_now_1[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }

    }

    sum = 0;
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Matrix_A_T[k][d];
            }
            P_now_kp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            P_now_kp[c][d] = P_now_kp[c][d] + Q_now[c][d];
        }
    }

/*
    P_now_kp[0][1] = 0;
    P_now_kp[0][2] = 0;
    P_now_kp[0][3] = 0;
    P_now_kp[1][0] = 0;
    P_now_kp[1][2] = 0;
    P_now_kp[1][3] = 0;
    P_now_kp[2][0] = 0;
    P_now_kp[2][1] = 0;
    P_now_kp[2][3] = 0;
    P_now_kp[3][0] = 0;
    P_now_kp[3][1] = 0;
    P_now_kp[3][2] = 0;
*/

//Compute for the matrix H

Matrix_H[0][0] = (X_now_kp[0][0])/(sqrt(X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[0][1] = (X_now_kp[1][0])/(sqrt(X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[0][2] = 0;
Matrix_H[0][3] = 0;

float tempx = X_now_kp[0][0];
float tempy = X_now_kp[1][0];
float tempa = X_now_kp[2][0];
float tempb = X_now_kp[3][0];
float temp0 = sqrt(tempy*tempy/tempx/tempx + 1);

Matrix_H[1][0] = -1*tempa*tempy*tempy/(tempx*tempx*tempx*temp0*temp0*temp0) - tempb*tempy/(tempx*tempx*temp0) + tempb*tempy*tempy*tempy/(tempx*tempx*tempx*tempx*temp0*temp0*temp0);
//Matrix_H[1][0] = (X_now_kp[2][0]) * (sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * (X_now_kp[1][0] / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0])) + (X_now_kp[3][0]) * (cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * (X_now_kp[1][0] / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[1][1] = (tempa*tempy + tempb*tempx)/(tempx*tempx+tempy*tempy)/temp0;
//Matrix_H[1][1] = (X_now_kp[2][0]) * (sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * ((-1*X_now_kp[0][0]) / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0])) + (X_now_kp[3][0]) * (cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * ((-1*X_now_kp[0][0]) / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[1][2] = -1/temp0;
//Matrix_H[1][2] = -1 * cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]));
Matrix_H[1][3] = tempy/tempx/temp0;
//Matrix_H[1][3] = -1 * sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]));
Matrix_H[2][0] = X_now_kp[1][0] / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]) * rad_deg;
Matrix_H[2][1] = (-1*X_now_kp[0][0]) / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]) * rad_deg;
Matrix_H[2][2] = 0;
Matrix_H[2][3] = 0;

Matrix_H[3][0] = -1*tempa*tempy/(tempx*tempx*temp0) + tempa*tempy*tempy*tempy/(tempx*tempx*tempx*tempx*temp0*temp0*temp0) - tempb*tempy*tempy/(tempx*tempx*tempx*temp0*temp0*temp0);
Matrix_H[3][1] = (tempa*tempx + tempb*tempy)/(tempx*tempx+tempy*tempy)/temp0;
Matrix_H[3][2] = tempy/tempx/temp0;
Matrix_H[3][3] = -1/temp0;

//Compute the kalman gain
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Matrix_H_T[d][c] = Matrix_H[c][d];
        }
    }

    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Matrix_H[c][k] * P_now_kp[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Matrix_H_T[k][d];
            }
            Four_Four_temp_1[c][d]= R_now[c][d] + sum;
            sum = 0;
        }
    }


    Three_Three_temp_00[0] = Four_Four_temp_1[0][0];
    Three_Three_temp_00[1] = Four_Four_temp_1[0][1];
    Three_Three_temp_00[2] = Four_Four_temp_1[0][2];
    Three_Three_temp_00[3] = Four_Four_temp_1[0][3];
    Three_Three_temp_00[4] = Four_Four_temp_1[1][0];
    Three_Three_temp_00[5] = Four_Four_temp_1[1][1];
    Three_Three_temp_00[6] = Four_Four_temp_1[1][2];
    Three_Three_temp_00[7] = Four_Four_temp_1[1][3];
    Three_Three_temp_00[8] = Four_Four_temp_1[2][02];
    Three_Three_temp_00[9] = Four_Four_temp_1[2][1];
    Three_Three_temp_00[10] = Four_Four_temp_1[2][2];
    Three_Three_temp_00[11] = Four_Four_temp_1[2][3];
    Three_Three_temp_00[12] = Four_Four_temp_1[3][0];
    Three_Three_temp_00[13] = Four_Four_temp_1[3][1];
    Three_Three_temp_00[14] = Four_Four_temp_1[3][2];
    Three_Three_temp_00[15] = Four_Four_temp_1[3][3];




    if(inverse4x4(Three_Three_temp_00, Three_Three_temp_11) == false)
        {
            debug_counter_1++;
            printf("%lf\n", debug_counter_1);
            Est_Target_Distance_x = X_now_kp[0][0];
            Est_Target_Distance_y = X_now_kp[1][0];
            Est_Target_Speed_x = X_now_kp[2][0];
            Est_Target_Speed_y = X_now_kp[3][0];
            return false;
        }

    Four_Four_temp_1[0][0] = Three_Three_temp_11[0];
    Four_Four_temp_1[0][1] = Three_Three_temp_11[1];
    Four_Four_temp_1[0][2] = Three_Three_temp_11[2];
    Four_Four_temp_1[0][3] = Three_Three_temp_11[3];
    Four_Four_temp_1[1][0] = Three_Three_temp_11[4];
    Four_Four_temp_1[1][1] = Three_Three_temp_11[5];
    Four_Four_temp_1[1][2] = Three_Three_temp_11[6];
    Four_Four_temp_1[1][3] = Three_Three_temp_11[7];
    Four_Four_temp_1[2][0] = Three_Three_temp_11[8];
    Four_Four_temp_1[2][1] = Three_Three_temp_11[9];
    Four_Four_temp_1[2][2] = Three_Three_temp_11[10];
    Four_Four_temp_1[2][3] = Three_Three_temp_11[11];
    Four_Four_temp_1[3][0] = Three_Three_temp_11[12];
    Four_Four_temp_1[3][1] = Three_Three_temp_11[13];
    Four_Four_temp_1[3][2] = Three_Three_temp_11[14];
    Four_Four_temp_1[3][3] = Three_Three_temp_11[15];

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + P_now_kp[c][k] * Matrix_H_T[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Four_Four_temp_1[k][d];
            }
            Kalman_Gain[c][d]= sum;
            sum = 0;
        }
    }
    

//compute the measured state:
    Y_mea[0][0] = Mea_Distance;
    Y_mea[1][0] = Mea_Speed;
    Y_mea[2][0] = Mea_Angle;
    Y_mea[3][0] = Mea_Speed_0;


//compute the filtered state

    Four_One_temp[0][0] = sqrt(X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]);
    Four_One_temp[1][0] = atan(-1*X_now_kp[1][0]/X_now_kp[0][0]) * rad_deg;
    Four_One_temp[2][0] = -1*X_now_kp[2][0]*cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0])) - X_now_kp[3][0]*sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]));
    Four_One_temp[3][0] = -1*X_now_kp[2][0]*sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0])) - X_now_kp[3][0]*cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]));

    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            Four_One_temp[c][d] = Y_mea[c][d] - Four_One_temp[c][d];
        }
    }


    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<1; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * Four_One_temp[k][d];
            }
            Four_One_temp_1[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            X_now[c][d] = X_now_kp[c][d] + Four_One_temp_1[c][d];
        }
    }

    Est_Target_Distance_x = X_now[0][0];
    Est_Target_Distance_y = X_now[1][0];
    Est_Target_Speed_x = X_now[2][0];
    Est_Target_Speed_y = X_now[3][0];

//update the covarience matrix wiki

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Kalman_Gain_T[c][d] = Kalman_Gain[d][c];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * Matrix_H[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = Four_Four_I[c][d] - Four_Four_temp[c][d];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * P_now_kp[k][d];
            }
            P_now[c][d]= sum;
            sum = 0;
        }
    }

/*

    for(int c=0; c<4; c++){
        for(int d=0; d<c; d++){
            float temp = Four_Four_temp[d][c];
            Four_Four_temp[d][c] = Four_Four_temp[c][d];
            Four_Four_temp[c][d] = temp;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + P_now[c][k] * Four_Four_temp[k][d];
            }
            Four_Four_temp_1[c][d]= sum;
            sum = 0;
        }
    }


    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<3; d++){
            for(int k=0; k<3; k++){
                sum= sum + Kalman_Gain[c][k] * R_now[k][d];
            }
            Four_Three_temp[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<3; k++){
                sum= sum + Four_Three_temp[c][k] * Kalman_Gain_T[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            P_now[c][d] = Four_Four_temp[c][d] + Four_Four_temp_1[c][d];
        }
    }
    
*/

//update the states
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            P_now_1[c][d] = P_now[c][d];
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            X_now_1[c][d] = X_now[c][d];
        }
    }
    return true;

}


int main()
{
    for(int i = 0; i < 30000; i ++)
    {
        if(radar_estimator(9,0,20,0) == true)
        {
            printf("%lf", Est_Target_Distance_x);
            printf("                        %lf\n",Est_Target_Distance_y);
        }
    }

    return 1;
}