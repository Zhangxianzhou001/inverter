/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-21     RT-Thread    first version
 */

#include <rtthread.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

//pid所引用的
#include "qpid.h"
#include <string.h>
#include <stdlib.h>

//上云所引用的
#include <board.h>
#include "onenet.h"
#include "paho_mqtt.h"

//电压值DMA地址参数
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  400)   /* Size of array aADCxConvertedData[] */
static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

//ADC与DMA参数
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

//DS18B20参数
#include "sensor.h"
#include "sensor_dallas_ds18b20.h"
#define DS18B20_DATA_PIN    GET_PIN(A, 12)
struct rt_sensor_data sensor_data;

//上云所用参数
extern rt_bool_t init_ok ;
extern MQTTClient mq_client;
#define ONENET_TOPIC_DP "$sys/2EjA5sB9k7/device111/thing/property/post"
int16_t dianya;
float_t dianliu;
float gonglv;

//计算参数
float fin[200];//存放电压检测点
float liu[200];//存放电流检测点
int32_t num;
uint16_t spp;
uint32_t eee;
uint16_t spwmcnt=0;
float ture=220;
float ture1=0;
float n=0;

//功率因数计算参数
float v_rms;
float i_rms ;
float phase_difference ;
float power_factor ;
//调制比
float  MI=0.8;
float  TI=0.8;

//pid参数
qpid_t qpid;
float pid_out = 0;

//线程参数
rt_thread_t th1_ptr = NULL;
rt_thread_t th2_ptr = NULL;
rt_thread_t th3_ptr = NULL;
rt_thread_t th4_ptr = NULL;

//信号量参数
static rt_sem_t dynamic_sem1 = RT_NULL;
static rt_sem_t dynamic_sem2 = RT_NULL;
static rt_sem_t dynamic_sem3 = RT_NULL;

static rt_err_t result1;
static rt_err_t result2;
static rt_err_t result3;

// 采样点数量
#define N  200

int16_t t=40000;
float calculate_rms(float *data, int length);
float calculate_phase_difference(float *v, float *i, int length);

//滤波函数
int size = 200;

// 滤波后的输出数组
float filteredLiu[200];
float window[3];
// 设置窗口大小
int windowSize = 3;

// 比较函数用于qsort
int compare(const void *a, const void *b) {
    return (*(float *)a - *(float *)b);
}

void medianFilter(const float *input, float *output, int size, int windowSize) {
    int halfWindow = windowSize / 2;

    for (int i = 0; i < size; ++i) {
        int count = 0;

        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < size) {
                window[count++] = input[idx];
            }
        }

        // 对窗口内的值进行排序
        qsort(window, count, sizeof(float), compare);

        // 取中值
        output[i] = window[count / 2];
    }
}


       int spwm_group[400]={
           33,99,165,231,297,362,428,494,559,624,690,755,819,884,948,1013,1076,
       1140,1203,1266,1329,1392,1454,1515,1577,1638,1698,1758,1818,1877,1936,1994,2052,
       2110,2166,2223,2278,2333,2388,2442,2495,2548,2600,2652,2703,2753,2802,2851,2899,
       2946,2993,3039,3084,3129,3172,3215,3257,3298,3339,3378,3417,3455,3492,3528,3564,
       3598,3632,3664,3696,3727,3757,3786,3814,3841,3868,3893,3917,3940,3963,3984,4005,
       4024,4042,4060,4076,4092,4106,4119,4132,4143,4153,4163,4171,4178,4184,4190,4194,
       4197,4199,4200,4200,4199,4197,4194,4190,4184,4178,4171,4163,4153,4143,4132,4119,
       4106,4092,4076,4060,4042,4024,4005,3984,3963,3940,3917,3893,3868,3841,3814,3786,
       3757,3727,3696,3664,3632,3598,3564,3528,3492,3455,3417,3378,3339,3298,3257,3215,
       3172,3129,3084,3039,2993,2946,2899,2851,2802,2753,2703,2652,2600,2548,2495,2442,
       2388,2333,2278,2223,2166,2110,2052,1994,1936,1877,1818,1758,1698,1638,1577,1515,
       1454,1392,1329,1266,1203,1140,1076,1013,948,884,819,755,690,624,559,494,
       428,362,297,231,165,99,33,-33,-99,-165,-231,-297,-362,-428,-494,-559,
       -624,-690,-755,-819,-884,-948,-1013,-1076,-1140,-1203,-1266,-1329,-1392,-1454,-1515,-1577,
       -1638,-1698,-1758,-1818,-1877,-1936,-1994,-2052,-2110,-2166,-2223,-2278,-2333,-2388,-2442,-2495,
       -2548,-2600,-2652,-2703,-2753,-2802,-2851,-2899,-2946,-2993,-3039,-3084,-3129,-3172,-3215,-3257,
       -3298,-3339,-3378,-3417,-3455,-3492,-3528,-3564,-3598,-3632,-3664,-3696,-3727,-3757,-3786,-3814,
       -3841,-3868,-3893,-3917,-3940,-3963,-3984,-4005,-4024,-4042,-4060,-4076,-4092,-4106,-4119,-4132,
       -4143,-4153,-4163,-4171,-4178,-4184,-4190,-4194,-4197,-4199,-4200,-4200,-4199,-4197,-4194,-4190,
       -4184,-4178,-4171,-4163,-4153,-4143,-4132,-4119,-4106,-4092,-4076,-4060,-4042,-4024,-4005,-3984,
       -3963,-3940,-3917,-3893,-3868,-3841,-3814,-3786,-3757,-3727,-3696,-3664,-3632,-3598,-3564,-3528,
       -3492,-3455,-3417,-3378,-3339,-3298,-3257,-3215,-3172,-3129,-3084,-3039,-2993,-2946,-2899,-2851,
       -2802,-2753,-2703,-2652,-2600,-2548,-2495,-2442,-2388,-2333,-2278,-2223,-2166,-2110,-2052,-1994,
       -1936,-1877,-1818,-1758,-1698,-1638,-1577,-1515,-1454,-1392,-1329,-1266,-1203,-1140,-1076,-1013,
       -948,-884,-819,-755,-690,-624,-559,-494,-428,-362,-297,-231,-165,-99,-33,
       };
        /* USER CODE BEGIN PV */

       void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //中断回调函数 20k
        {
            if(htim==&htim2)
            {

               spp=4200.f + MI*spwm_group[spwmcnt];
               TIM1->CCR1 = spp;
               TIM1->CCR2 = spp;
               spwmcnt++;
          if(spwmcnt==400)spwmcnt=0;
          }
        }



void th1_entry(void *parameter)
{
    while(1){

        result1 = rt_sem_take(dynamic_sem1, RT_WAITING_FOREVER);
        if (result1 == RT_EOK)
        {

            int16_t j = 0;
            for(int16_t k=0;k<200;k++)
            {
                fin[k]=aADCxConvertedData[j]*3.3/4096;
                liu[k]=aADCxConvertedData[j+1]*3.3/4096;
                j+=2;
            }


            for(int16_t i=0;i<200;i++)
            {
               fin[i]=fin[i]-1.64;
               fin[i]=fin[i]*540;
               ture+=fin[i]*fin[i];
               if(i==199)
               {
                   ture=ture/200;
                   ture=sqrt(ture);
                   rt_kprintf("%f\n",ture);
                   //rt_sem_release(dynamic_sem2);
               }
            }

//            medianFilter(liu, filteredLiu, size, windowSize);
//            for(int16_t i=0;i<200;i++){
//                n=n+liu[i];
//            }
//            n=(n/200);
//            for(int16_t i=0;i<200;i++)
//            {
//                filteredLiu[i]=filteredLiu[i]-1.667;
//                //filteredLiu[i]=filteredLiu[i]-n;
//                filteredLiu[i]=(filteredLiu[i]/0.625) * 5;
//                rt_kprintf("%f,",n);
//                rt_kprintf("%f\n",filteredLiu[i]);
//               ture1+=filteredLiu[i]*filteredLiu[i];
//               if(i==199)
//               {
//                   ture1=ture1/200;
//                   ture1=sqrt(ture1);
//                   //rt_kprintf("%f,",ture1);
//                  rt_sem_release(dynamic_sem2);
//                  rt_sem_release(dynamic_sem3);
//               }
//            }
            for(int16_t i=0;i<200;i++)
                        {
                liu[i]=liu[i]-1.6675;
                //liu[i]=liu[i]-n;
                liu[i]=(liu[i]/0.625) * 5;
                //rt_kprintf("%f,",n);
                //rt_kprintf("%f\n",liu[i]);
               ture1+=liu[i]*liu[i];
               if(i==199)
               {
                   ture1=ture1/200;
                   ture1=sqrt(ture1);
                   //rt_kprintf("%f,",ture1);
                  rt_sem_release(dynamic_sem2);
                  rt_sem_release(dynamic_sem3);
               }
            }
        rt_thread_mdelay(50);

        }
    }

}
void th2_entry(void *parameter)
{
    result2 = rt_sem_take(dynamic_sem2, RT_WAITING_FOREVER);
    if (result2 == RT_EOK){
    while(1)
    {
    pid_out=qpid_cal_inc(&qpid, ture);
    //rt_kprintf("%f,",pid_out);
    MI += pid_out;
    if(MI >= 1) MI = 0.99f;
    else if(MI <= 0.3) MI = 0.3;
    //rt_kprintf("%f\n",MI);
    rt_thread_mdelay(50);
    }
    }
}

void th3_entry(void *parameter)
{
    char pub_msg[1024];
    MQTTMessage message;

    message.qos = QOS1;
    message.retained = 0;

    while(1)
    {
        dianya = ture;
        dianliu = ture1;
        gonglv = (ture * ture1) * power_factor;
        //rt_kprintf("%f\n",dianliu);
        //sprintf(pub_msg,"{\"id\":\"123\",\"params\":{\"DIANYA\":{\"value\":%d}}}",dianya);
         //rt_sprintf(pub_msg,"{\"id\":\"123\",\"params\":{\"DIANLIU\":{\"value\":%.2f}}}",dianliu);
        //rt_kprintf("ture: %f, ture1: %f, power_factor: %f, gonglv: %f\n", ture, ture1, power_factor, gonglv);

        rt_sprintf(pub_msg,"{\"id\":\"123\",\"params\":{\"DIANYA\":{\"value\":%d},\"DIANLIU\":{\"value\":%.4f},\"GONGLV\":{\"value\":%.3f},\"WENDU\":{\"value\":%.2f},\"YINSHU\":{\"value\":%.2f}}}",dianya,dianliu,gonglv,(float)sensor_data.data.temp/10,power_factor);

        message.payload = (void *)pub_msg;
        message.payloadlen = rt_strlen(message.payload);

        if (init_ok == RT_TRUE && mq_client.isconnected) {
             MQTTPublish(&mq_client , ONENET_TOPIC_DP, &message);
            rt_memset(pub_msg, 0, 1024);
        }

        rt_thread_mdelay(5000);
    }
    return RT_EOK;
}

void th4_entry(void *parameter)
{
    result3 = rt_sem_take(dynamic_sem3, RT_WAITING_FOREVER);
    if (result3 == RT_EOK){
            while(1){
//            v_rms = calculate_rms(fin, N);
//            i_rms = calculate_rms(liu, N);
              v_rms = ture;
              i_rms = ture1;
            phase_difference = calculate_phase_difference(fin, liu, N);
            power_factor = cos(phase_difference);
            if(power_factor<0.02)
                power_factor = 0 ;

            // 输出结果到终端
//            rt_kprintf("电压RMS: %.2f\n", v_rms);
//            rt_kprintf("电流RMS: %.2f\n", i_rms);
//            rt_kprintf("相位差: %.2f radians\n", phase_difference);
//            rt_kprintf("功率因数: %.2f\n", power_factor);


            rt_thread_mdelay(5000);
         }
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){

    rt_kprintf("NVIC_entry failed...\n");
    eee = HAL_ADC_GetError(&hadc1);
    rt_kprintf("%d\n",eee);
}

//DMA中断回调函数
void DMA2_Stream0_IRQHandler(void){


    for (int16_t i = 0 ; i < 400; i+=2)
                {
                        //rt_kprintf("%d,",i);
                        //rt_kprintf("%d,",aADCxConvertedData[i]);
                        //rt_kprintf("%d,",i+1);
                        //rt_kprintf("%d,\n",aADCxConvertedData[i+1]);
                }
    rt_sem_release(dynamic_sem1);
    HAL_DMA_IRQHandler(&hdma_adc1);
}

static void read_temp_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;

    rt_size_t res;

    dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't find device:%s\n", parameter);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open device failed!\n");
        return;
    }
    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)100);

    while (1)
    {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1)
        {
            rt_kprintf("read data failed!size is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            if (sensor_data.data.temp >= 0)
            {
//                rt_kprintf("temp:%3d.%dC, timestamp:%5d\n",
//                           sensor_data.data.temp / 10,
//                           sensor_data.data.temp % 10,
//                           sensor_data.timestamp);
            }
            else
            {
//                rt_kprintf("temp:-%2d.%dC, timestamp:%5d\n",
//                           abs(sensor_data.data.temp / 10),
//                           abs(sensor_data.data.temp % 10),
//                           sensor_data.timestamp);
            }
        }
        rt_thread_mdelay(5000);
    }
}


int main(void)
{
        HAL_Init();
        MX_GPIO_Init();
        MX_TIM1_Init();
        MX_TIM2_Init();
        MX_TIM3_Init();
        MX_DMA_Init();
        MX_ADC1_Init();



        //开启定时器2中断，开启四路spwm波，开启ADC和DMA
        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);
        HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);
        HAL_TIM_Base_Start(&htim3);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_Start_DMA(&hadc1,(uint32_t *)aADCxConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE);

        //pid启动
        qpid_init(&qpid);
        qpid_set_dst(&qpid, 220);
        qpid_set_ratio(&qpid, 0.0001, 0.00015, 0.002);
        qpid_set_lmt(&qpid, -0.8, 0.8);

        //创建信号量1
        dynamic_sem1 = rt_sem_create("sem1", 0, RT_IPC_FLAG_FIFO);
        if(dynamic_sem1 == RT_NULL){
            LOG_E("rt_sem1_create failed...\n");
            return -RT_ENOMEM;
        }
            LOG_D("rt_sem2_create SUCCESSED...\n");
        //创建信号量2
       dynamic_sem2 = rt_sem_create("sem2", 0, RT_IPC_FLAG_FIFO);
       if(dynamic_sem2 == RT_NULL){
           LOG_E("rt_sem2_create failed...\n");
           return -RT_ENOMEM;
       }
           LOG_D("rt_sem2_create SUCCESSED...\n");
        //创建信号量3
       dynamic_sem3 = rt_sem_create("sem3", 0, RT_IPC_FLAG_FIFO);
       if(dynamic_sem3 == RT_NULL){
           LOG_E("rt_sem3_create failed...\n");
           return -RT_ENOMEM;
       }
           LOG_D("rt_sem3_create SUCCESSED...\n");

        //启动任务一
        th1_ptr = rt_thread_create("th1_demo", th1_entry, NULL, 1024, 20, 5);
        if (th1_ptr==NULL) {
            LOG_E("RT_THREAD_1_CREATE FALLED...\n");
            return -RT_ENOMEM;
        }
            LOG_D("RT_THREAD_1_CREATE SUCCESSED...\n");
            rt_thread_startup(th1_ptr);

        //启动任务二
        th2_ptr = rt_thread_create("th2_demo", th2_entry, NULL, 1024, 20, 5);
        if (th2_ptr==NULL) {
            LOG_E("RT_THREAD_2_CREATE FALLED...\n");
            return -RT_ENOMEM;
        }
            LOG_D("RT_THREAD_2_CREATE SUCCESSED...\n");
            rt_thread_startup(th2_ptr);

       //启动任务三。上云任务
            th3_ptr = rt_thread_create("th3_demo", th3_entry, NULL, 2048, 20, 5);
            if (th3_ptr==NULL) {
                LOG_E("RT_THREAD_3_CREATE FALLED...\n");
                return -RT_ENOMEM;
            }
                LOG_D("RT_THREAD_3_CREATE SUCCESSED...\n");
                rt_thread_startup(th3_ptr);
       //启动任务四。计算功率因数
            th4_ptr = rt_thread_create("th4_demo", th4_entry, NULL, 2048, 20, 5);
            if (th4_ptr==NULL) {
                LOG_E("RT_THREAD_4_CREATE FALLED...\n");
                return -RT_ENOMEM;
            }
                LOG_D("RT_THREAD_4_CREATE SUCCESSED...\n");
                rt_thread_startup(th4_ptr);





            return RT_EOK;
}

static int ds18b20_read_temp_sample(void)
{
    rt_thread_t ds18b20_thread;

    ds18b20_thread = rt_thread_create("18b20tem",
                                      read_temp_entry,
                                      "temp_ds18b20",
                                      1024,
                                      RT_THREAD_PRIORITY_MAX / 2,
                                      20);
    if (ds18b20_thread != RT_NULL)
    {
        rt_thread_startup(ds18b20_thread);
    }

    return RT_EOK;
}
INIT_APP_EXPORT(ds18b20_read_temp_sample);

static int rt_hw_ds18b20_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.user_data = (void *)DS18B20_DATA_PIN;
    rt_hw_ds18b20_init("ds18b20", &cfg);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_ds18b20_port);

// 计算RMS值
float calculate_rms(float *data, int length) {
    float sum = 0.0;
    for (int i = 0; i < length; i++) {
        sum += data[i] * data[i];
    }
    return sqrt(sum / length);
}
//INIT_APP_EXPORT(calculate_rms);

// 计算两个数组之间的相位差（使用互相关法）
float calculate_phase_difference(float *v, float *i, int length) {
    float sum_v = 0.0, sum_i = 0.0, sum_vi = 0.0, sum_v2 = 0.0, sum_i2 = 0.0;
    for (int k = 0; k < length; k++) {
        sum_v += v[k];
        sum_i += i[k];
        sum_vi += v[k] * i[k];
        sum_v2 += v[k] * v[k];
        sum_i2 += i[k] * i[k];
    }

    float numerator = length * sum_vi - sum_v * sum_i;
    float denominator = sqrt((length * sum_v2 - sum_v * sum_v) * (length * sum_i2 - sum_i * sum_i));

    if (denominator == 0) {
        return 0;
    }

    float correlation = numerator / denominator;
    return acos(correlation);  // 返回相位差（弧度）
}
//INIT_APP_EXPORT(calculate_phase_difference);





