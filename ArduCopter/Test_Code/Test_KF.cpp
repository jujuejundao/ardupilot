#include <stdio.h>
#include <math.h>

float debug_counter_1=0;
float Est_Target_Distance;
float Est_Target_Speed;
float Est_Target_Angle;
float Est_Target_Angle_Speed;

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
    		debug_counter_1++;
            printf("matrix inversion failed %lf\n", debug_counter_1);
            return false;
        }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}

/*

float dt_est = 0.03333333333333333;
float Mea_Distance_err = 0.1;
float Mea_Angle_err = 5;
float Mea_Speed_err = 1.2;
float Mea_Angle_Speed_err = 5;
float Pro_Distance_err = 0.1;
float Pro_Angle_err = 2;
float sum;

float X_now_1[4][1]=
{
    {100},
    {0},
    {0},
    {0}

};

float X_now[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};

float X_now_kp[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};


float Y_mea[4][1]=
{
    {0},
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

//Measured Covariance Matrix
float R_now[4][4]=
{
    {Mea_Distance_err*Mea_Distance_err, 0, 0, 0},
    {0, Mea_Angle_err*Mea_Angle_err, 0, 0},
    {0, 0,Mea_Speed_err*Mea_Speed_err, 0},
    {0, 0, 0, Mea_Angle_Speed_err*Mea_Angle_Speed_err}
};

//Predicted Covariance matrix
float Q_now[4][4]=
{
    {Pro_Distance_err*Pro_Distance_err*dt_est*dt_est*dt_est*dt_est/4,0,Pro_Distance_err*Pro_Distance_err*dt_est*dt_est*dt_est/2,0},
    {0, Pro_Angle_err*Pro_Angle_err*dt_est*dt_est*dt_est*dt_est/4, 0, Pro_Angle_err*Pro_Angle_err*dt_est*dt_est*dt_est/2},
    {Pro_Distance_err*Pro_Distance_err*dt_est*dt_est/2, 0, Pro_Distance_err*Pro_Distance_err*dt_est, 0},
    {0, Pro_Angle_err*Pro_Angle_err*dt_est*dt_est/2, 0, Pro_Angle_err*Pro_Angle_err*dt_est}
};

float P_now_kp[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float P_now_1[4][4]=
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

float Four_Four_I[4][4]=
{
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
};

float Four_Four_temp_00[16];
float Four_Four_temp_11[16];

bool radar_estimator(float Mea_Distance, float Mea_Speed, float Mea_Angle)
{

    //Compute the processed state
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

    //Compute Pkp Covariance Matrix
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


    //Compute the Kalman Gain
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = R_now[c][d] + P_now_kp[c][d];
        }
    }

    Four_Four_temp_00[0] = Four_Four_temp[0][0];
    Four_Four_temp_00[1] = Four_Four_temp[0][1];
    Four_Four_temp_00[2] = Four_Four_temp[0][2];
    Four_Four_temp_00[3] = Four_Four_temp[0][3];
    Four_Four_temp_00[4] = Four_Four_temp[1][0];
    Four_Four_temp_00[5] = Four_Four_temp[1][1];
    Four_Four_temp_00[6] = Four_Four_temp[1][2];
    Four_Four_temp_00[7] = Four_Four_temp[1][3];
    Four_Four_temp_00[8] = Four_Four_temp[2][02];
    Four_Four_temp_00[9] = Four_Four_temp[2][1];
    Four_Four_temp_00[10] = Four_Four_temp[2][2];
    Four_Four_temp_00[11] = Four_Four_temp[2][3];
    Four_Four_temp_00[12] = Four_Four_temp[3][0];
    Four_Four_temp_00[13] = Four_Four_temp[3][1];
    Four_Four_temp_00[14] = Four_Four_temp[3][2];
    Four_Four_temp_00[15] = Four_Four_temp[3][3];

    if(inverse4x4(Four_Four_temp_00, Four_Four_temp_11) == false)
        {
            debug_counter_1++;
            return false;
        }

    Four_Four_temp[0][0] = Four_Four_temp_11[0];
    Four_Four_temp[0][1] = Four_Four_temp_11[1];
    Four_Four_temp[0][2] = Four_Four_temp_11[2];
    Four_Four_temp[0][3] = Four_Four_temp_11[3];
    Four_Four_temp[1][0] = Four_Four_temp_11[4];
    Four_Four_temp[1][1] = Four_Four_temp_11[5];
    Four_Four_temp[1][2] = Four_Four_temp_11[6];
    Four_Four_temp[1][3] = Four_Four_temp_11[7];
    Four_Four_temp[2][0] = Four_Four_temp_11[8];
    Four_Four_temp[2][1] = Four_Four_temp_11[9];
    Four_Four_temp[2][2] = Four_Four_temp_11[10];
    Four_Four_temp[2][3] = Four_Four_temp_11[11];
    Four_Four_temp[3][0] = Four_Four_temp_11[12];
    Four_Four_temp[3][1] = Four_Four_temp_11[13];
    Four_Four_temp[3][2] = Four_Four_temp_11[14];
    Four_Four_temp[3][3] = Four_Four_temp_11[15];


    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + P_now_kp[c][k] * Four_Four_temp[k][d];
            }
            Kalman_Gain[c][d]= sum;
            sum = 0;
        }
    }

    //compute the filtered state
    Y_mea[0][0] = Mea_Distance;
    Y_mea[1][0] = Mea_Angle;
    Y_mea[2][0] = Mea_Speed;

    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            Four_One_temp[c][d] = Y_mea[c][d] - X_now_kp[c][d];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<1; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * Four_One_temp[k][d];
            }
            X_now[c][d]= sum + X_now_kp[c][d];
            sum = 0;
        }
    }

    Est_Target_Distance = X_now[0][0];
    Est_Target_Angle = X_now[1][0];
    Est_Target_Speed = X_now[2][0];
    Est_Target_Angle_Speed = X_now[3][0];

    //update covariance matrix
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = Four_Four_I[c][d] -  Kalman_Gain[c][d];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * P_now_kp[k][d];
            }
            P_now_1[c][d]= sum;
            sum = 0;
        }
    }

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
                sum= sum + P_now_1[c][k] * Four_Four_temp[k][d];
            }
            Four_Four_temp_1[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * R_now[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Kalman_Gain_T[c][d] = Kalman_Gain[d][c];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Kalman_Gain_T[k][d];
            }
            P_now_1[c][d]= sum + Four_Four_temp_1[c][d];
            sum = 0;
        }
    }

    //update the state matrix
    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            X_now_1[c][d] = X_now[c][d];
        }
    }
    return true;

}

*/

float dt_est = 0.03333333333333333;
float Mea_Distance_err = 0.1;
float Mea_Angle_err = 5;
float Mea_Speed_err = 1.2;
float Mea_Angle_Speed_err = 5;
float Pro_Distance_err = 0.1;
float Pro_Angle_err = 1.2;
float sum, data_abord;

float X_now_1[4][1]=
{
    {100},
    {0},
    {0},
    {0}

};

float X_now[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};

float X_now_kp[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};


float Y_mea[4][1]=
{
    {0},
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

//Measured Covariance Matrix
float R_now[4][4]=
{
    {Mea_Distance_err*Mea_Distance_err, 0, 0, 0},
    {0, Mea_Angle_err*Mea_Angle_err, 0, 0},
    {0, 0,Mea_Speed_err*Mea_Speed_err, 0},
    {0, 0, 0, Mea_Angle_Speed_err*Mea_Angle_Speed_err}
};

//Predicted Covariance matrix
float Q_now[4][4]=
{
    {Pro_Distance_err*Pro_Distance_err*dt_est*dt_est*dt_est*dt_est/4,0,Pro_Distance_err*Pro_Distance_err*dt_est*dt_est*dt_est/2,0},
    {0, Pro_Angle_err*Pro_Angle_err*dt_est*dt_est*dt_est*dt_est/4, 0, Pro_Angle_err*Pro_Angle_err*dt_est*dt_est*dt_est/2},
    {Pro_Distance_err*Pro_Distance_err*dt_est*dt_est/2, 0, Pro_Distance_err*Pro_Distance_err*dt_est, 0},
    {0, Pro_Angle_err*Pro_Angle_err*dt_est*dt_est/2, 0, Pro_Angle_err*Pro_Angle_err*dt_est}
};

float P_now_kp[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float P_now_1[4][4]=
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

float Four_Four_I[4][4]=
{
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
};

float Four_Four_temp_00[16];
float Four_Four_temp_11[16];
float Distance_t[10] = {100,100,100,100,100,100,100,100,100,100};

bool check_data(float);

bool radar_estimator(float Mea_Distance, float Mea_Speed, float Mea_Angle)
{

    //Compute the processed state
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

    //Compute Pkp Covariance Matrix
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

    //Compute the Kalman Gain
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = R_now[c][d] + P_now_kp[c][d];
        }
    }

    Four_Four_temp_00[0] = Four_Four_temp[0][0];
    Four_Four_temp_00[1] = Four_Four_temp[0][1];
    Four_Four_temp_00[2] = Four_Four_temp[0][2];
    Four_Four_temp_00[3] = Four_Four_temp[0][3];
    Four_Four_temp_00[4] = Four_Four_temp[1][0];
    Four_Four_temp_00[5] = Four_Four_temp[1][1];
    Four_Four_temp_00[6] = Four_Four_temp[1][2];
    Four_Four_temp_00[7] = Four_Four_temp[1][3];
    Four_Four_temp_00[8] = Four_Four_temp[2][02];
    Four_Four_temp_00[9] = Four_Four_temp[2][1];
    Four_Four_temp_00[10] = Four_Four_temp[2][2];
    Four_Four_temp_00[11] = Four_Four_temp[2][3];
    Four_Four_temp_00[12] = Four_Four_temp[3][0];
    Four_Four_temp_00[13] = Four_Four_temp[3][1];
    Four_Four_temp_00[14] = Four_Four_temp[3][2];
    Four_Four_temp_00[15] = Four_Four_temp[3][3];

    if(inverse4x4(Four_Four_temp_00, Four_Four_temp_11) == false)
        {
            debug_counter_1++;
            for(int i=0; i<4; i++)
                X_now[i][0] = X_now_kp[i][0];

            data_abord = 2;
            return false;
        }

    Four_Four_temp[0][0] = Four_Four_temp_11[0];
    Four_Four_temp[0][1] = Four_Four_temp_11[1];
    Four_Four_temp[0][2] = Four_Four_temp_11[2];
    Four_Four_temp[0][3] = Four_Four_temp_11[3];
    Four_Four_temp[1][0] = Four_Four_temp_11[4];
    Four_Four_temp[1][1] = Four_Four_temp_11[5];
    Four_Four_temp[1][2] = Four_Four_temp_11[6];
    Four_Four_temp[1][3] = Four_Four_temp_11[7];
    Four_Four_temp[2][0] = Four_Four_temp_11[8];
    Four_Four_temp[2][1] = Four_Four_temp_11[9];
    Four_Four_temp[2][2] = Four_Four_temp_11[10];
    Four_Four_temp[2][3] = Four_Four_temp_11[11];
    Four_Four_temp[3][0] = Four_Four_temp_11[12];
    Four_Four_temp[3][1] = Four_Four_temp_11[13];
    Four_Four_temp[3][2] = Four_Four_temp_11[14];
    Four_Four_temp[3][3] = Four_Four_temp_11[15];


    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + P_now_kp[c][k] * Four_Four_temp[k][d];
            }
            Kalman_Gain[c][d]= sum;
            sum = 0;
        }
    }


    //Process the measured data
    if(check_data(Mea_Distance) == false)
    {
        for(int i=0; i<4; i++)
            X_now[i][0] = X_now_kp[i][0];
        data_abord = 1;
    }

    else
    {
        //compute the filtered state
        Y_mea[0][0] = Mea_Distance;
        Y_mea[1][0] = Mea_Angle;
        Y_mea[2][0] = Mea_Speed;

        for(int c=0; c<4; c++){
            for(int d=0; d<1; d++){
                Four_One_temp[c][d] = Y_mea[c][d] - X_now_kp[c][d];
            }
        }

        sum = 0;
        for(int c=0; c<4;c++){
            for(int d=0; d<1; d++){
                for(int k=0; k<4; k++){
                    sum= sum + Kalman_Gain[c][k] * Four_One_temp[k][d];
                }
                X_now[c][d]= sum + X_now_kp[c][d];
                sum = 0;
            }
        }
        data_abord = 0;
    }

    

    Est_Target_Distance = X_now[0][0];
    Est_Target_Angle = X_now[1][0];
    Est_Target_Speed = X_now[2][0];
    Est_Target_Angle_Speed = X_now[3][0];

    //update covariance matrix
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = Four_Four_I[c][d] -  Kalman_Gain[c][d];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * P_now_kp[k][d];
            }
            P_now_1[c][d]= sum;
            sum = 0;
        }
    }

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
                sum= sum + P_now_1[c][k] * Four_Four_temp[k][d];
            }
            Four_Four_temp_1[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * R_now[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Kalman_Gain_T[c][d] = Kalman_Gain[d][c];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Kalman_Gain_T[k][d];
            }
            P_now_1[c][d]= sum + Four_Four_temp_1[c][d];
            sum = 0;
        }
    }

    //update the state matrix
    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            X_now_1[c][d] = X_now[c][d];
        }
    }

    for(int i=0; i<9; i++)
    {
        Distance_t[i] = Distance_t[i+1];
    }
    Distance_t[9] = X_now[0][0];

    return true;

}


int exceed_counter = 0;
int bypass_counter = 30;

bool check_data(float Current_Distance)
{
    if (bypass_counter != 30)
    {
        bypass_counter++;
        return true;
    }

    sum = 0;
    for(int i = 0; i<10; i++)
    {
        sum = sum  + Distance_t[i];
    }
    sum = sum/10;

    if((Current_Distance-sum > 2) || (Current_Distance-sum < -2))
    {
        exceed_counter++;
        if(exceed_counter < 20)
            return false;
        else
        {
            exceed_counter = 0;
            bypass_counter = 0;
            return true;
        }
    }
    exceed_counter = 0;
    return true;
}

int main()
{
    for(int i = 0; i < 500; i ++)
    {
        if(radar_estimator(i/30,1,20) == true)
        {
            printf("%lf", Est_Target_Distance);
            printf("      %lf",Est_Target_Angle);
            printf("           %lf",Est_Target_Speed);
            printf("              %lf\n",Est_Target_Angle_Speed);
      	}
    }

    return 1;
}