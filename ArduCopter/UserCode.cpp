#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    hal.uartE->begin(115200);   //zx

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    process_input_radar();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here

}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

float Rcs_1;
float Range_1;
float Azimuth_1;
float Vrel_1;
float SNR_1;
float Rcs_2;
float Range_2;
float Azimuth_2;
float Vrel_2;
float SNR_2;
float Rcs_3;
float Range_3;
float Azimuth_3;
float Vrel_3;
float SNR_3;
float Rcs_4;
float Range_4;
float Azimuth_4;
float Vrel_4;
float SNR_4;
float Rcs_5;
float Range_5;
float Azimuth_5;
float Vrel_5;
float SNR_5;
float Rcs_extra;
float Range_extra;
float Azimuth_extra;
float Vrel_extra;
float SNR_extra; 

bool Copter::process_input_radar()
{
    //main code of radar here
        uart_T = hal.uartE;
        int bytes_in_buff = uart_T->available();
        int read_bytes = 0;


        if (bytes_in_buff > 0)
        {
            while(read_bytes < bytes_in_buff)
            {
                uint8_t temp_byte;
                temp_byte = uart_T->read();

                for (int i = 0; i < 13; i++)
                    buffer[i] = buffer[i+1];
                
                buffer[13] = temp_byte;
                read_bytes++;
//              printf("0x%x\n", temp_byte);

                if (HEADER == HDRBYTES && ENDER == EDRBYTES)
                {

                    if(MESSAGE_ID == MSGID_TS)
                    {
                        No_of_targets = buffer[4];
                        Rollcount = buffer[5];
                        if(No_of_targets == 0)
                        {
                            Min_Target_Reflected_Area = 0;
                            Min_Target_Distance = 100;
                            Min_Target_Speed = 0;
                            Min_Target_Angle = 0;
                            Min_Target_SNR = 0;
                            radar_estimator(Min_Target_Distance, Min_Target_Speed, Min_Target_Angle);

                        }
                            
                    }
                        

                    else if(MESSAGE_ID == MSGID_TI)
                    {
                        for(int k = 0; k < 8; k++)
                            target_info[k] = buffer[k+4];

                        if(TI_Index == 1)
                        {
                            Rcs_1 = TI_Rcs*0.5 - 50;
                            Range_1 = (TI_RangeH*256 + TI_RangeL)*0.01;
                            Azimuth_1 = TI_Azimuth*2 - 90;
                            Vrel_1 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
                            SNR_1 = TI_SNR - 127;

                            range_list[TI_Index-1]= Range_1;
                            range_key[TI_Index-1] = TI_Index;

                            if(No_of_targets == TI_Index)
                            {
                                Min_Target_Reflected_Area = Rcs_1;
                                Min_Target_Distance = Range_1;
                                Min_Target_Angle = Azimuth_1;
                                Min_Target_Speed = Vrel_1;
                                Min_Target_SNR = SNR_1;
                                radar_estimator(Min_Target_Distance, Min_Target_Speed, Min_Target_Angle);
                            }

                        }
                        else if(TI_Index == 2)
                        {
                            Rcs_2 = TI_Rcs*0.5 - 50;
                            Range_2 = (TI_RangeH*256 + TI_RangeL)*0.01;
                            Azimuth_2 = TI_Azimuth*2 - 90;
                            Vrel_2 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
                            SNR_2 = TI_SNR - 127;

                            range_list[TI_Index-1]= Range_2;
                            range_key[TI_Index-1] = TI_Index;
                            
                            if(No_of_targets == TI_Index)
                            {
                                if(Range_1 > Range_2)
                                {
                                    Min_Target_Reflected_Area = Rcs_2;
                                    Min_Target_Distance = Range_2;
                                    Min_Target_Angle = Azimuth_2;
                                    Min_Target_Speed = Vrel_2;
                                    Min_Target_SNR = SNR_2;
                                }
                                else
                                {
                                    Min_Target_Reflected_Area = Rcs_1;
                                    Min_Target_Distance = Range_1;
                                    Min_Target_Angle = Azimuth_1;
                                    Min_Target_Speed = Vrel_1;
                                    Min_Target_SNR = SNR_1;
                                }
                                radar_estimator(Min_Target_Distance, Min_Target_Speed, Min_Target_Angle);
                            }
                        }
                        else if(TI_Index == 3)
                        {
                            Rcs_3 = TI_Rcs*0.5 - 50;
                            Range_3 = (TI_RangeH*256 + TI_RangeL)*0.01;
                            Azimuth_3 = TI_Azimuth*2 - 90;
                            Vrel_3 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
                            SNR_3 = TI_SNR - 127;

                            range_list[TI_Index-1]= Range_3;
                            range_key[TI_Index-1] = TI_Index;

                            if(No_of_targets == TI_Index)
                            {
                                for(int i = 0; i<No_of_targets-1;i++)
                                {
                                    for(int j = 0; j<No_of_targets-i-1; j++)
                                    {
                                        if(range_list[j] > range_list[j+1])
                                        {
                                            float temp_range = range_list[j];
                                            range_list[j] = range_list[j+1];
                                            range_list[j+1] = temp_range;
                                            float temp_key = range_key[j];
                                            range_key[j] = range_key[j+1];
                                            range_key[j+1] = temp_key;
                                        }
                                    }
                                }

                                switch(range_key[0])
                                {
                                    case 1: Min_Target_Reflected_Area = Rcs_1;
                                            Min_Target_Distance = Range_1;
                                            Min_Target_Angle = Azimuth_1;
                                            Min_Target_Speed = Vrel_1;
                                            Min_Target_SNR = SNR_1;
                                            break;

                                    case 2: Min_Target_Reflected_Area = Rcs_2;
                                            Min_Target_Distance = Range_2;
                                            Min_Target_Angle = Azimuth_2;
                                            Min_Target_Speed = Vrel_2;
                                            Min_Target_SNR = SNR_2;
                                            break;

                                    case 3: Min_Target_Reflected_Area = Rcs_3;
                                            Min_Target_Distance = Range_3;
                                            Min_Target_Angle = Azimuth_3;
                                            Min_Target_Speed = Vrel_3;
                                            Min_Target_SNR = SNR_3;
                                            break;

                                    case 4: Min_Target_Reflected_Area = Rcs_4;
                                            Min_Target_Distance = Range_4;
                                            Min_Target_Angle = Azimuth_4;
                                            Min_Target_Speed = Vrel_4;
                                            Min_Target_SNR = SNR_4;
                                            break;

                                    case 5: Min_Target_Reflected_Area = Rcs_5;
                                            Min_Target_Distance = Range_5;
                                            Min_Target_Angle = Azimuth_5;
                                            Min_Target_Speed = Vrel_5;
                                            Min_Target_SNR = SNR_5;
                                            break;

                                }

                                radar_estimator(Min_Target_Distance, Min_Target_Speed, Min_Target_Angle);

                            }
                            
                        }
                        else if(TI_Index == 4)
                        {
                            Rcs_4 = TI_Rcs*0.5 - 50;
                            Range_4 = (TI_RangeH*256 + TI_RangeL)*0.01;
                            Azimuth_4 = TI_Azimuth*2 - 90;
                            Vrel_4 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
                            SNR_4 = TI_SNR - 127;
                           
                            range_list[TI_Index-1]= Range_4;
                            range_key[TI_Index-1] = TI_Index;
                            
                            if(No_of_targets == TI_Index)
                            {
                                for(int i = 0; i<No_of_targets-1;i++)
                                {
                                    for(int j = 0; j<No_of_targets-i-1; j++)
                                    {
                                        if(range_list[j] > range_list[j+1])
                                        {
                                            float temp_range = range_list[j];
                                            range_list[j] = range_list[j+1];
                                            range_list[j+1] = temp_range;
                                            float temp_key = range_key[j];
                                            range_key[j] = range_key[j+1];
                                            range_key[j+1] = temp_key;
                                        }
                                    }
                                }

                                switch(range_key[0])
                                {
                                    case 1: Min_Target_Reflected_Area = Rcs_1;
                                            Min_Target_Distance = Range_1;
                                            Min_Target_Angle = Azimuth_1;
                                            Min_Target_Speed = Vrel_1;
                                            Min_Target_SNR = SNR_1;
                                            break;

                                    case 2: Min_Target_Reflected_Area = Rcs_2;
                                            Min_Target_Distance = Range_2;
                                            Min_Target_Angle = Azimuth_2;
                                            Min_Target_Speed = Vrel_2;
                                            Min_Target_SNR = SNR_2;
                                            break;

                                    case 3: Min_Target_Reflected_Area = Rcs_3;
                                            Min_Target_Distance = Range_3;
                                            Min_Target_Angle = Azimuth_3;
                                            Min_Target_Speed = Vrel_3;
                                            Min_Target_SNR = SNR_3;
                                            break;

                                    case 4: Min_Target_Reflected_Area = Rcs_4;
                                            Min_Target_Distance = Range_4;
                                            Min_Target_Angle = Azimuth_4;
                                            Min_Target_Speed = Vrel_4;
                                            Min_Target_SNR = SNR_4;
                                            break;

                                    case 5: Min_Target_Reflected_Area = Rcs_5;
                                            Min_Target_Distance = Range_5;
                                            Min_Target_Angle = Azimuth_5;
                                            Min_Target_Speed = Vrel_5;
                                            Min_Target_SNR = SNR_5;
                                            break;

                                }
                                radar_estimator(Min_Target_Distance, Min_Target_Speed, Min_Target_Angle);

                            }
                        }
                        else if(TI_Index == 5)
                        {
                            Rcs_5 = TI_Rcs*0.5 - 50;
                            Range_5 = (TI_RangeH*256 + TI_RangeL)*0.01;
                            Azimuth_5 = TI_Azimuth*2 - 90;
                            Vrel_5 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
                            SNR_5 = TI_SNR - 127;
                            
                            range_list[TI_Index-1]= Range_5;
                            range_key[TI_Index-1] = TI_Index;
                            
                            if(No_of_targets == TI_Index)
                            {
                                for(int i = 0; i<No_of_targets-1;i++)
                                {
                                    for(int j = 0; j<No_of_targets-i-1; j++)
                                    {
                                        if(range_list[j] > range_list[j+1])
                                        {
                                            float temp_range = range_list[j];
                                            range_list[j] = range_list[j+1];
                                            range_list[j+1] = temp_range;
                                            float temp_key = range_key[j];
                                            range_key[j] = range_key[j+1];
                                            range_key[j+1] = temp_key;
                                        }
                                    }
                                }

                                switch(range_key[0])
                                {
                                    case 1: Min_Target_Reflected_Area = Rcs_1;
                                            Min_Target_Distance = Range_1;
                                            Min_Target_Angle = Azimuth_1;
                                            Min_Target_Speed = Vrel_1;
                                            Min_Target_SNR = SNR_1;
                                            break;

                                    case 2: Min_Target_Reflected_Area = Rcs_2;
                                            Min_Target_Distance = Range_2;
                                            Min_Target_Angle = Azimuth_2;
                                            Min_Target_Speed = Vrel_2;
                                            Min_Target_SNR = SNR_2;
                                            break;

                                    case 3: Min_Target_Reflected_Area = Rcs_3;
                                            Min_Target_Distance = Range_3;
                                            Min_Target_Angle = Azimuth_3;
                                            Min_Target_Speed = Vrel_3;
                                            Min_Target_SNR = SNR_3;
                                            break;

                                    case 4: Min_Target_Reflected_Area = Rcs_4;
                                            Min_Target_Distance = Range_4;
                                            Min_Target_Angle = Azimuth_4;
                                            Min_Target_Speed = Vrel_4;
                                            Min_Target_SNR = SNR_4;
                                            break;

                                    case 5: Min_Target_Reflected_Area = Rcs_5;
                                            Min_Target_Distance = Range_5;
                                            Min_Target_Angle = Azimuth_5;
                                            Min_Target_Speed = Vrel_5;
                                            Min_Target_SNR = SNR_5;
                                            break;

                                }
                                radar_estimator(Min_Target_Distance, Min_Target_Speed, Min_Target_Angle);

                            }
                        }
                        else
                        {
                            Rcs_extra = TI_Rcs*0.5 - 50;
                            Range_extra = (TI_RangeH*256 + TI_RangeL)*0.01;
                            Azimuth_extra = TI_Azimuth*2 - 90;
                            Vrel_extra = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
                            SNR_extra = TI_SNR - 127;
                            
                            if(Min_Target_Distance > Range_extra)
                            {
                                Min_Target_Reflected_Area = Rcs_extra;
                                Min_Target_Distance = Range_extra;
                                Min_Target_Angle = Azimuth_extra;
                                Min_Target_Speed = Vrel_extra;
                                Min_Target_SNR = SNR_extra;
                            }
                            radar_estimator(Min_Target_Distance, Min_Target_Speed, Min_Target_Angle);
                        }

                    }

                }

            }
        }
        return true;
}

float dt_est = 0.03333333333333333;
float Mea_Distance_err = 0.1;
float Mea_Angle_err = 5;
float Mea_Speed_err = 1.2;
float Mea_Angle_Speed_err = 5;
float Pro_Distance_err = 0.1;
float Pro_Angle_err = 1.2;
float sum;
int exceed_counter = 0;
int bypass_counter = 10;
float Four_Four_temp_00[16];
float Four_Four_temp_11[16];
float Distance_t[10] = {100,100,100,100,100,100,100,100,100,100};

bool check_data(float);

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


bool Copter::radar_estimator(float Mea_Distance, float Mea_Speed, float Mea_Angle)
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
        if (Mea_Angle == 0)
        {
            Mea_Angle = Mea_Angle + 10;
        }
        Y_mea[1][0] = Mea_Angle - 10; // anlge off set = 10.  angle radar at right, target at left positive, left negative
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

        if (X_now[0][0] < 0)
        {
            X_now[0][0] = Mea_Distance;
            X_now[2][0] = Mea_Speed;
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

    for(int i=0; i<9; i++)
    {
        Distance_t[i] = Distance_t[i+1];
    }
    Distance_t[9] = X_now[0][0];

    return true;

}




bool check_data(float Current_D)
{
//    return true;
    if (bypass_counter != 10)
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

    if(Current_D-sum > 2)// || (Current_D-sum < -2)
    {
        exceed_counter++;
        if(exceed_counter < 10)
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