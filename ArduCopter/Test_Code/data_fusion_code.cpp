/*

//kalman filter for distance and speed only

float dt_est = 0.03333333333333333;
float Mea_Distance_err = 0.1;
float Mea_Speed_err = 1.2;
float Pro_Distance_err = 0.1;
float Pro_Speed_err = 1.2;

float acc_cal;
float acc_cal_vel;
float acc_cal_dis;

float X_now_1[2][1]=
{
    {100},
    {0}

};

float X_now_2[2][1]=
{
    {100},
    {0}
};

float X_now_3[2][1]=
{
    {100},
    {0}

};

float X_now_4[2][1]=
{
    {100},
    {0}
};

float X_now_5[2][1]=
{
    {100},
    {0}

};

float X_now_6[2][1]=
{
    {100},
    {0}
};

float Matrix_A[2][2]=
{
    {1, dt_est},
    {0, 1}
};

float Matrix_A_T[2][2]=
{
    {1, 0},
    {dt_est, 1}
};

float Matrix_B[2][1]=
{
    {0.5*dt_est*dt_est},
    {dt_est}
};

float P_now_1[2][2]=
{
    {Pro_Distance_err*Pro_Distance_err,0},
    {0,Pro_Speed_err*Pro_Speed_err}
};

float Y_mea[2][1]=
{
    {0},
    {0}
};

float X_now[2][1]=
{
    {0},
    {0}
};

float X_now_kp[2][1]=
{
    {0},
    {0}
};

float P_now_kp[2][2]=
{
    {0,0},
    {0,0}
};

float P_now_kp_temp[2][2]=
{
    {0,0},
    {0,0}
};

float P_now[2][2]=
{
    {0,0},
    {0,0}
};

float Kalman_Gain[2][2]=
{
    {0,0},
    {0,0}
};



void Copter::radar_estimator(float Mea_Distance, float Mea_Speed)
{

//compute the measured state:
    Y_mea[0][0] = Mea_Distance;
    Y_mea[1][0] = Mea_Speed;

//Compute the estimated accelration:
    acc_cal_vel = ((X_now_3[1][0] + X_now_2[1][0] + X_now_1[1][0]) - (X_now_6[1][0] + X_now_5[1][0] + X_now_4[1][0]))/ (3 * dt_est);
//    acc_cal_dis = (X_now_1[0][0] - X_now_2[0][0] - dt_est * (X_now_1[1][0] + X_now_2[1][0]) / 2) / (0.5*dt_est*dt_est);
    acc_cal = acc_cal_vel; //(acc_cal_dis + acc_cal_vel)/2;
  
//Compute the predicted state: X_now_kp = Matrix_A * X_now_1 + Matrix_B*acc_cal;
    X_now_kp[0][0] = Matrix_A[0][0] * X_now_1[0][0] + Matrix_A[0][1] * X_now_1[1][0] + Matrix_B[0][0] * acc_cal;
    X_now_kp[1][0] = Matrix_A[1][0] * X_now_1[0][0] + Matrix_A[1][1] * X_now_1[1][0] + Matrix_B[1][0] * acc_cal;

//Compute Pkp covariance matrix: P_now_kp = Matrix_A*P_now_1*Matrix_AT
    float sum = 0;
    for(int c=0; c<2;c++){
        for(int d=0; d<2; d++){
            for(int k=0; k<2; k++){
                sum= sum + Matrix_A[c][k] * P_now_1[k][d];
            }
            P_now_kp_temp[c][d]= sum;
            sum = 0;
        }

    }
    for(int c=0; c<2;c++){
        for(int d=0; d<2; d++){
            for(int k=0; k<2; k++){
                sum= sum + P_now_kp_temp[c][k] * Matrix_A_T[k][d];
            }
            P_now_kp[c][d]= sum;
            sum = 0;
        }

    }

    P_now_kp[0][1] = 0;
    P_now_kp[1][0] = 0;
    P_now_kp[0][0] = P_now_kp[0][0] + Pro_Distance_err*Pro_Distance_err;   //delete the error covariance
    P_now_kp[1][1] = P_now_kp[1][1] + Pro_Speed_err*Pro_Speed_err;         //delete the error covariance

//Compute the kalman gain
    Kalman_Gain[0][0] = P_now_kp[0][0] / (P_now_kp[0][0] + Mea_Distance_err * Mea_Distance_err);
    Kalman_Gain[1][1] = P_now_kp[1][1] / (P_now_kp[1][1] + Mea_Speed_err * Mea_Speed_err);

//compute the filtered state
    X_now[0][0] = X_now_kp[0][0] * (1-Kalman_Gain[0][0]) + Y_mea[0][0] * Kalman_Gain[0][0];
    X_now[1][0] = X_now_kp[1][0] * (1-Kalman_Gain[1][1]) + Y_mea[1][0] * Kalman_Gain[1][1];

    Est_Target_Distance = X_now[0][0];
    Est_Target_Speed = X_now[1][0];

//update the covarience matrix
//    P_now[0][0] = (1-Kalman_Gain[0][0]) * P_now_kp[0][0];
//    P_now[1][1] = (1-Kalman_Gain[1][1]) * P_now_kp[1][1];

//update the covarience matrix wiki
    P_now[0][0] = (1-Kalman_Gain[0][0]) * P_now_kp[0][0] * (1-Kalman_Gain[0][0]) + Kalman_Gain[0][0] * (Mea_Distance_err * Mea_Distance_err) * Kalman_Gain[0][0];
    P_now[1][1] = (1-Kalman_Gain[1][1]) * P_now_kp[1][1] * (1-Kalman_Gain[1][1]) + Kalman_Gain[1][1] * (Mea_Speed_err * Mea_Speed_err) * Kalman_Gain[1][1];

    P_now_1[0][0] = P_now[0][0];
    P_now_1[1][1] = P_now[1][1];

//update the states
    X_now_6[0][0] = X_now_5[0][0];
    X_now_6[1][0] = X_now_5[1][0];

    X_now_5[0][0] = X_now_4[0][0];
    X_now_5[1][0] = X_now_4[1][0];

    X_now_4[0][0] = X_now_3[0][0];
    X_now_4[1][0] = X_now_3[1][0];

    X_now_3[0][0] = X_now_2[0][0];
    X_now_3[1][0] = X_now_2[1][0];

    X_now_2[0][0] = X_now_1[0][0];
    X_now_2[1][0] = X_now_1[1][0];

    X_now_1[0][0] = X_now[0][0];
    X_now_1[1][0] = X_now[1][0];

//debug counter
    debug_counter_1 = acc_cal_vel;
    debug_counter_2 = acc_cal_dis;
    debug_counter_3 = acc_cal;

}

*/

/*

//low pass filter for distance and speed only
float Distance_t[10] = {100,100,100,100,100,100,100,100,100,100};
float Speed_t[10] = {0,0,0,0,0,0,0,0,0,0};


void record_data(float distance_now, float speed_now)
{
    for(int i = 10; i>=2; i--)
        Distance_t[i] = Distance_t[i-1];
    Distance_t[1] = distance_now;
    for(int j = 10; j>=2; j--)
        Speed_t[j] = Speed_t[j-1];
    Speed_t[1] = speed_now;
}



//radar estimator using low pass filter

int exceed_10_counter = 0;
int reduce_5_counter = 0;

void Copter::radar_estimator(float Distance_t0, float Speed_t0)
{
    //if distance suddenly exceed 10 times the difference    
    if((Distance_t0 - Distance_t[1]) > 10 * (Distance_t[1] - Distance_t[2]))
    {
        exceed_10_counter++;
        reduce_5_counter = 0; 
        if(exceed_10_counter >= 6)
        {
            exceed_10_counter = 0;
            Est_Target_Distance = 0.7 * Distance_t0 + 0.3 * Distance_t[1];
            Est_Target_Speed = 0.7 * Speed_t0 + 0.3 * Speed_t[1];
            record_data(Est_Target_Distance, Est_Target_Speed);

        }
        else
        {
            Est_Target_Distance = Distance_t[1] + (Distance_t[1] - Distance_t[2]);
            Est_Target_Speed = Speed_t[1];
            record_data(Est_Target_Distance, Est_Target_Speed);
        }
    }

    //if distance suddenly decrease
    else if((Distance_t[1] - Distance_t0) > 5 * (Distance_t[2] - Distance_t[1]))
    {
        reduce_5_counter ++;
        exceed_10_counter = 0; 
        if(reduce_5_counter >= 3)
        {
            reduce_5_counter = 0;
//            Est_Target_Distance = 0.7 * Distance_t0 + 0.3 * Distance_t[1];
//            Est_Target_Speed = 0.7 * Speed_t0 + 0.3 * Speed_t[1];
            Est_Target_Distance = 0.7 * Distance_t0 + 0.3 * Distance_t[1];
            Est_Target_Speed = 0.7 * Speed_t0 + 0.3 * Speed_t[1];
            record_data(Est_Target_Distance, Est_Target_Speed);

        }
        else
        {
            Est_Target_Distance = 0.2 * Distance_t0 + 0.8 * Distance_t[1];
            Est_Target_Speed = 0.2 * Speed_t0 + 0.8 * Speed_t[1];
            record_data(Est_Target_Distance, Est_Target_Speed);
        }
    }

    //if distance dont change too much
    else
    {
        exceed_10_counter = 0;
        reduce_5_counter = 0;
        Est_Target_Distance = 0.7 * Distance_t0 + 0.3 * Distance_t[1];
        Est_Target_Speed = 0.7 * Speed_t0 + 0.3 * Speed_t[1];
        record_data(Est_Target_Distance, Est_Target_Speed);
    }
    debug_counter_1 = exceed_10_counter;
    debug_counter_2 = reduce_5_counter;
 //   debug_counter_3 = acc_cal;

}

*/

/*
//lp 7/3 test
void Copter::radar_estimator(float Distance_t0, float Speed_t0)
{
    Est_Target_Distance = 0.7 * Distance_t0 + 0.3 * Distance_t[1];
    Est_Target_Speed = 0.7 * Speed_t0 + 0.3 * Speed_t[1];
    Distance_t[1] = Est_Target_Distance;
    Speed_t[1] = Est_Target_Speed;
}

*/

/*

//kalman filter for estimating the target position with reference to the copter


float dt_est = 0.03333333333333333;
float Mea_Distance_err_x = 0.1;
float Mea_Distance_err_y = 0.1;
float Mea_Speed_err_x = 1.2;
float Mea_Speed_err_y = 1.2;
float Pro_Distance_err_x = 0.1;
float Pro_Distance_err_y = 0.1;
float Pro_Speed_err_x = 1.2;
float Pro_Speed_err_y = 1.2;
float sum = 0;


float X_now_1[4][1]=
{
    {100},
    {0},
    {0},
    {0}

};

float X_now[4][1]=
{
    {100},
    {0},
    {0},
    {0}
};

float X_now_kp[4][1]=
{
    {100},
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
    {Mea_Distance_err_x*Mea_Distance_err_x,0,0,0},
    {0, Mea_Distance_err_y*Mea_Distance_err_y, 0, 0},
    {0, 0, Mea_Speed_err_x*Mea_Speed_err_x, 0},
    {0, 0, 0, Mea_Speed_err_y*Mea_Speed_err_y}
};

//Predicted Covariance matrix
float Q_now[4][4]=
{
    {Pro_Distance_err_x*Pro_Distance_err_x,0,0,0},
    {0, Pro_Distance_err_y*Pro_Distance_err_y, 0, 0},
    {0, 0, Pro_Speed_err_x*Pro_Speed_err_x, 0},
    {0, 0, 0, Pro_Speed_err_y*Pro_Speed_err_y}
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

float Kalman_Gain[4][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

float Kalman_Gain_T[4][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

float Four_Four_temp[4][4]=
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

float Matrix_Acc[2][1]=
{
    {0},
    {0}
};

float Four_Four_temp_00[16];
float Four_Four_temp_11[16];

bool Copter::radar_estimator(float Mea_Distance, float Mea_Speed, float Mea_Angle)
{

//compute the measured state:
//    Mea_Speed = Mea_Speed*(-1);
//    Mea_Angle = Mea_Angle*(-1);
    Y_mea[0][0] = Mea_Distance * cos(Mea_Angle);
    Y_mea[1][0] = Mea_Distance * sin(Mea_Angle);
    Y_mea[2][0] = Mea_Speed * cos(Mea_Angle);
    Y_mea[3][0] = Mea_Speed * sin(Mea_Angle);

    debug_counter_1 = Y_mea[0][0];
    debug_counter_2 = Y_mea[1][0];
    debug_counter_3 = Mea_Angle;

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

//    Matrix_Acc[0][0] = ((float)rand()/(float)(RAND_MAX)) * 6 -3;
//    Matrix_Acc[1][0] = ((float)rand()/(float)(RAND_MAX)) * 6 -3;



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

//Compute the kalman gain
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = P_now_kp[c][d] + R_now[c][d];
        }
    }


    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            if(Four_Four_temp[c][d] != 0)
                Kalman_Gain[c][d] = P_now_kp[c][d] / Four_Four_temp[c][d];
        }
    }

//compute the filtered state
    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<1; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * Y_mea[k][d];
            }
            X_now[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<1; d++){
            for(int k=0; k<4; k++){
                sum= sum + (1- Kalman_Gain[c][k]) * X_now_kp[k][d];
            }
            Four_One_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            X_now[c][d] = X_now[c][d] + Four_One_temp[c][d];
        }
    }

    Est_Target_Distance_x = X_now[0][0];
    Est_Target_Speed_x = X_now[2][0];
    Est_Target_Distance_y = X_now[1][0];
    Est_Target_Speed_y = X_now[3][0];

//update the covarience matrix wiki
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Kalman_Gain_T[c][d] = Kalman_Gain[c][d];
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<c; d++){
            float temp = Kalman_Gain_T[d][c];
            Kalman_Gain_T[d][c] = Kalman_Gain_T[c][d];
            Kalman_Gain_T[c][d] = temp;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + (1- Kalman_Gain[c][k]) * P_now_kp[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * (1 - Kalman_Gain_T[k][d]);
            }
            P_now[c][d]= sum;
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

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Kalman_Gain_T[k][d];
            }
            P_now[c][d]= P_now[c][d] + sum;
            sum = 0;
        }
    }


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

*/


/*
//matrix manipulation
c 1st matrix row
d 2nd matirx colume
k 2nd matrix row

    sum = 0;
    for(int c=0; c<2;c++){
        for(int d=0; d<2; d++){
            for(int k=0; k<2; k++){
                sum= sum + Matrix_A[c][k] * P_now_1[k][d];
            }
            P_now_kp_temp[c][d]= sum;
            sum = 0;
        }
    }

//addition of matrix

c row
d rolumn

for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            P_now_kp[c][d] = P_now_kp[c][d] + Q_now[c][d];
        }
    }

*/



//kalman filter for estimating the target position with reference to the copter


float dt_est = 0.03333333333333333;
float Mea_Distance_err = 0.1;
float Mea_Speed_err = 1.2;
float Mea_Angle_err = 5;
float Pro_Distance_err_x = 1;
float Pro_Distance_err_y = 1;
float Pro_Speed_err_x = 12;
float Pro_Speed_err_y = 12;
float sum = 0;


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


float Y_mea[3][1]=
{
    {100},
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
    {Pro_Distance_err_x*Pro_Distance_err_x,0,0,0},
    {0, Pro_Distance_err_y*Pro_Distance_err_y, 0, 0},
    {0, 0, Pro_Speed_err_x*Pro_Speed_err_x, 0},
    {0, 0, 0, Pro_Speed_err_y*Pro_Speed_err_y}
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

float Matrix_H[3][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Matrix_H_T[4][3]=
{
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};

float Kalman_Gain[4][3]=
{
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

float Kalman_Gain_T[3][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
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

float Three_Three_temp[3][3]=
{
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

float Three_Four_temp[3][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

float Four_Three_temp[4][3]=
{
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

float Three_One_temp[3][1]=
{
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

float Three_Three_temp_00[9];
float Three_Three_temp_11[9];

float rad_deg = 180/3.14159265;
float deg_rad = 3.14159265/180;

bool Copter::radar_estimator(float Mea_Distance, float Mea_Speed, float Mea_Angle)
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


//    P_now_kp[0][1] = 0;
//    P_now_kp[0][2] = 0;
//    P_now_kp[0][3] = 0;
//    P_now_kp[1][0] = 0;
//    P_now_kp[1][2] = 0;
//    P_now_kp[1][3] = 0;
//    P_now_kp[2][0] = 0;
//    P_now_kp[2][1] = 0;
//    P_now_kp[2][3] = 0;
//    P_now_kp[3][0] = 0;
//    P_now_kp[3][1] = 0;
//    P_now_kp[3][2] = 0;


//Compute for the matrix H

Matrix_H[0][0] = (X_now_kp[0][0])/(sqrt(X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[0][1] = (X_now_kp[1][0])/(sqrt(X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[0][2] = 0;
Matrix_H[0][3] = 0;
Matrix_H[1][0] = (X_now_kp[2][0]) * (sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * (X_now_kp[1][0] / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0])) + (X_now_kp[3][0]) * (cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * (X_now_kp[1][0] / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[1][1] = (X_now_kp[2][0]) * (sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * ((-1*X_now_kp[0][0]) / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0])) + (X_now_kp[3][0]) * (cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]))) * ((-1*X_now_kp[0][0]) / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]));
Matrix_H[1][2] = -1 * cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]));
Matrix_H[1][3] = -1 * sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]));
Matrix_H[2][0] = X_now_kp[1][0] / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]) * rad_deg;
Matrix_H[2][1] = (-1*X_now_kp[0][0]) / (X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]) * rad_deg;
Matrix_H[2][2] = 0;
Matrix_H[2][3] = 0;

//Compute the kalman gain
    for(int c=0; c<3; c++){
        for(int d=0; d<4; d++){
            Matrix_H_T[d][c] = Matrix_H[c][d];
        }
    }

    for(int c=0; c<3;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Matrix_H[c][k] * P_now_kp[k][d];
            }
            Three_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<3;c++){
        for(int d=0; d<3; d++){
            for(int k=0; k<4; k++){
                sum= sum + Three_Four_temp[c][k] * Matrix_H_T[k][d];
            }
            Three_Three_temp[c][d]= R_now[c][d] + sum;
            sum = 0;
        }
    }


    Three_Three_temp_00[0] = Three_Three_temp[0][0];
    Three_Three_temp_00[1] = Three_Three_temp[0][1];
    Three_Three_temp_00[2] = Three_Three_temp[0][2];
    Three_Three_temp_00[3] = Three_Three_temp[1][0];
    Three_Three_temp_00[4] = Three_Three_temp[1][1];
    Three_Three_temp_00[5] = Three_Three_temp[1][2];
    Three_Three_temp_00[6] = Three_Three_temp[2][0];
    Three_Three_temp_00[7] = Three_Three_temp[2][1];
    Three_Three_temp_00[8] = Three_Three_temp[2][2];




    if(inverse3x3(Three_Three_temp_00, Three_Three_temp_11) == false)
        {
            debug_counter_1++;
            Est_Target_Distance_x = X_now_kp[0][0];
            Est_Target_Distance_y = X_now_kp[1][0];
            Est_Target_Speed_x = X_now_kp[2][0];
            Est_Target_Speed_y = X_now_kp[3][0];
            return false;
        }

    Three_Three_temp[0][0] = Three_Three_temp_11[0];
    Three_Three_temp[0][1] = Three_Three_temp_11[1];
    Three_Three_temp[0][2] = Three_Three_temp_11[2];
    Three_Three_temp[1][0] = Three_Three_temp_11[3];
    Three_Three_temp[1][1] = Three_Three_temp_11[4];
    Three_Three_temp[1][2] = Three_Three_temp_11[5];
    Three_Three_temp[2][0] = Three_Three_temp_11[6];
    Three_Three_temp[2][1] = Three_Three_temp_11[7];
    Three_Three_temp[2][2] = Three_Three_temp_11[8];

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<3; d++){
            for(int k=0; k<4; k++){
                sum= sum + P_now_kp[c][k] * Matrix_H_T[k][d];
            }
            Four_Three_temp[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<3; d++){
            for(int k=0; k<3; k++){
                sum= sum + Four_Three_temp[c][k] * Three_Three_temp[k][d];
            }
            Kalman_Gain[c][d]= sum;
            sum = 0;
        }
    }
    

//compute the measured state:
    Y_mea[0][0] = Mea_Distance;
    Y_mea[1][0] = Mea_Speed;
    Y_mea[2][0] = Mea_Angle;


//compute the filtered state

    Three_One_temp[0][0] = sqrt(X_now_kp[0][0]*X_now_kp[0][0] + X_now_kp[1][0]*X_now_kp[1][0]);
    Three_One_temp[1][0] = atan(-1*X_now_kp[1][0]/X_now_kp[0][0]) * rad_deg;
    Three_One_temp[2][0] = -1*X_now_kp[2][0]*cos(atan(-1*X_now_kp[1][0]/X_now_kp[0][0])) - X_now_kp[3][0]*sin(atan(-1*X_now_kp[1][0]/X_now_kp[0][0]));

    for(int c=0; c<3; c++){
        for(int d=0; d<1; d++){
            Three_One_temp[c][d] = Y_mea[c][d] - Three_One_temp[c][d];
        }
    }

    debug_counter_1 = Y_mea[0][0]*cos(Y_mea[2][0]*deg_rad);
    debug_counter_2 = Y_mea[0][0]*sin(Y_mea[2][0]*deg_rad);
    debug_counter_3 = Three_One_temp[2][0];

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<1; d++){
            for(int k=0; k<3; k++){
                sum= sum + Kalman_Gain[c][k] * Three_One_temp[k][d];
            }
            Four_One_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            X_now[c][d] = X_now_kp[c][d] + Four_One_temp[c][d];
        }
    }

    Est_Target_Distance_x = X_now[0][0];
    Est_Target_Distance_y = X_now[1][0];
    Est_Target_Speed_x = X_now[2][0];
    Est_Target_Speed_y = X_now[3][0];

//update the covarience matrix wiki

    for(int c=0; c<3; c++){
        for(int d=0; d<4; d++){
            Kalman_Gain_T[c][d] = Kalman_Gain[d][c];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<3; k++){
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
