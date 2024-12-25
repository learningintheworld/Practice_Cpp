#include <stdio.h>
#include <stdlib.h>

#define WINDOW_SIZE 15

typedef struct {
    double *data;
    int size;
    int front;
    int rear;
    double sum;
    int count;
} MovingAverageFilter;

// 全局变量，作为滤波器
// static MovingAverageFilter filter_instance;
MovingAverageFilter filter_instance;

// MovingAverageFilter* createMovingAverageFilter(int size) {
//     MovingAverageFilter *filter = (MovingAverageFilter*)malloc(sizeof(MovingAverageFilter));
//     filter->data = (double*)malloc(size * sizeof(double));
//     filter->size = size;
//     filter->front = 0;
//     filter->rear = -1;
//     filter->sum = 0.0;
//     return filter;
// }
MovingAverageFilter* createMovingAverageFilter(int size) {
    filter_instance.data = (double*)malloc(size * sizeof(double));
    filter_instance.size = size;
    filter_instance.front = 0;
    filter_instance.rear = -1;
    filter_instance.sum = 0.0;
    filter_instance.count = 0;
    return &filter_instance;
}

void resetMovingAverageFilter() {
    filter_instance.front = 0;
    filter_instance.rear = -1;
    filter_instance.sum = 0.0;
    filter_instance.count = 0;
}

void destroyMovingAverageFilter(MovingAverageFilter *filter) {
    free(filter->data);
    // free(filter);
    filter->data = NULL;
}

void enqueue(MovingAverageFilter *filter, double value) {
    if (filter->rear == filter->size - 1) {
        filter->rear = -1;
    }
    filter->data[++filter->rear] = value;
    filter->sum += value;
    filter->count++;
}

double dequeue(MovingAverageFilter *filter) {
    double value = filter->data[filter->front++];
    if (filter->front == filter->size) {
        filter->front = 0;
    }
    filter->sum -= value;
    filter->count--;
    return value;
}

double getMovingAverage(MovingAverageFilter *filter) {
    // 检查队列中的元素数量是否等于窗口大小
    if (filter->count == filter->size) {
        double result =  filter->sum / filter->size;
        // 移除最旧的数据点，为下一个数据点腾出空间
        dequeue(filter);
        return result;
    } 
    else 
    {
        // 如果队列中的元素数量小于窗口大小，则只计算已有元素的平均值
        return filter->sum / filter->count;
        // return 0.0;
    }
}

/*

int main() {
    // 创建滑动窗口平均滤波器
    MovingAverageFilter *filter = createMovingAverageFilter(WINDOW_SIZE);

    // 模拟IMU设备发出的数据
    double imu_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    double imu_data1[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    double imu_data2[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    double imu_data3[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    int imu_data_size = sizeof(imu_data) / sizeof(double);

    // 模拟IMU数据每25ms发出一个数据点
    for (int i = 0; i < imu_data_size; i++) {
        // 新数据点到达
        double new_data = imu_data[i];
        
        // 添加新数据点到滤波器中
        enqueue(filter, new_data);

        // // 如果队列已满，则计算滑动窗口平均值并输出
        // if (i >= WINDOW_SIZE - 1) {
        //     double moving_average = getMovingAverage(filter);
        //     printf("Moving average at time %d: %.2f\n", i - WINDOW_SIZE + 1, moving_average);
            
        //     // 移除最旧的数据点，为下一个数据点腾出空间
        //     dequeue(filter);
        // }

        double moving_average = getMovingAverage(filter);
        printf("Moving average at time %d: %.2f\n", i - WINDOW_SIZE + 1, moving_average);

        // // 移除最旧的数据点，为下一个数据点腾出空间
        // dequeue(filter);

    }


    // // 销毁滤波器
    // destroyMovingAverageFilter(filter);

    // 重置滤波器的状态，以便下次使用
    resetMovingAverageFilter();

    return 0;
}

*/