#ifndef __KALMAN_H
#define __KALMAN_H

typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP_t;

typedef struct
{
	float *buffer;
	unsigned int buffer_len;
}smooth_filter_t;

float kalmanFilter(KFP_t *kfp,float input);
float averageFilter(smooth_filter_t *filter,float in_data);
	
#endif