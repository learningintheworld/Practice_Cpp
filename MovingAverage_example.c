#include <stdio.h>
#include <stdlib.h>
#include "MovingAverageFilter.h"

double movmean_motor_angle[7];

// void movmean(double &motor_angle[0],double &movmean_motor_angle[0]){
// 	for(int i = 0; i < 7; i++)																			// Add new data points to the filter
// 	{
// 		enqueue(&filter_instance[i], motor_angle[i]);
// 	}
// 	for(int i = 0; i < 7; i++)																			// Obtain the mean result
// 	{
// 		movmean_motor_angle[i] = getMovingAverage(&filter_instance[i]);
// 	}
// }


/*

int main() {
	initMovingAverageFilter(&filter_instance[0], WINDOW_SIZE);
	initMovingAverageFilter(&filter_instance[1], WINDOW_SIZE);
	initMovingAverageFilter(&filter_instance[2], WINDOW_SIZE);
	initMovingAverageFilter(&filter_instance[3], WINDOW_SIZE);
	initMovingAverageFilter(&filter_instance[4], WINDOW_SIZE);
	initMovingAverageFilter(&filter_instance[5], WINDOW_SIZE);
	initMovingAverageFilter(&filter_instance[6], WINDOW_SIZE);

    double imu_data0[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double imu_data1[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double imu_data2[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double imu_data3[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double imu_data4[] = {89, 91, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 
    90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 
    90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 
    90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01, 90.01};
    double imu_data5[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double imu_data6[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int imu_data_size = sizeof(imu_data0) / sizeof(double);

    for (int i = 0; i < imu_data_size; i++)
    {
        enqueue(&filter_instance[0], imu_data0[i]);
        enqueue(&filter_instance[1], imu_data1[i]);
        enqueue(&filter_instance[2], imu_data2[i]);
        enqueue(&filter_instance[3], imu_data3[i]);
        enqueue(&filter_instance[4], imu_data4[i]);
        enqueue(&filter_instance[5], imu_data5[i]);
        enqueue(&filter_instance[6], imu_data6[i]);

        movmean_motor_angle[0] = getMovingAverage(&filter_instance[0]);
        movmean_motor_angle[1] = getMovingAverage(&filter_instance[1]);
        movmean_motor_angle[2] = getMovingAverage(&filter_instance[2]);
        movmean_motor_angle[3] = getMovingAverage(&filter_instance[3]);
        movmean_motor_angle[4] = getMovingAverage(&filter_instance[4]);
        movmean_motor_angle[5] = getMovingAverage(&filter_instance[5]);
        movmean_motor_angle[6] = getMovingAverage(&filter_instance[6]);

        for (int i = 0; i < 7; i++)
        {
            printf("%0.2f ", movmean_motor_angle[i]);
        }
        
        printf("\n");
    }






    // // 创建滑动窗口平均滤波器
    // initMovingAverageFilter(filter_instance, WINDOW_SIZE);

    // // 模拟IMU设备发出的数据
    // double imu_data[] = {96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 
    // 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 
    // 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 
    // 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05, 96.05,};
    // double imu_data1[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    // double imu_data2[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    // double imu_data3[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    // 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    // int imu_data_size = sizeof(imu_data) / sizeof(double);

    // // 模拟IMU数据每25ms发出一个数据点
    // for (int i = 0; i < imu_data_size; i++) {
    //     // 新数据点到达
    //     double new_data = imu_data[i];
        
    //     // 添加新数据点到滤波器中
    //     enqueue(filter_instance, new_data);

    //     // // 如果队列已满，则计算滑动窗口平均值并输出
    //     // if (i >= WINDOW_SIZE - 1) {
    //     //     double moving_average = getMovingAverage(filter);
    //     //     printf("Moving average at time %d: %.2f\n", i - WINDOW_SIZE + 1, moving_average);
            
    //     //     // 移除最旧的数据点，为下一个数据点腾出空间
    //     //     dequeue(filter);
    //     // }

    //     double moving_average = getMovingAverage(filter_instance);
    //     printf("Moving average at time %d: %.2f\n", i - WINDOW_SIZE + 1, moving_average);

    //     // // 移除最旧的数据点，为下一个数据点腾出空间
    //     // dequeue(filter);
    // }

    // // // 销毁滤波器
    // // destroyMovingAverageFilter(filter);

    // // 重置滤波器的状态，以便下次使用
    // resetMovingAverageFilter(filter_instance);

    return 0;
}



*/

