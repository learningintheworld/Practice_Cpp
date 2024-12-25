#include <stdio.h>
#include <stdlib.h>

// 滑动均值滤波函数
void movmean(const double *A, int n, int k, double *M) {
    int i, j, half;
    double sum;

    half = k / 2;
    for (i = 0; i < n; i++) {
        sum = 0.0;
        // 计算滑动窗口内元素的和
        for (j = i - half; j <= i + half; j++) {
            if (j >= 0 && j < n) {
                sum += A[j];
            }
        }
        // 计算窗口内元素的平均值
        M[i] = sum / k;
    }
}

/*
int main() {
    int i;
    const int n = 10;
    const int k = 3;  // 窗口大小
    double A[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};  // 输入数组
    double M[n];  // 保存滤波结果的数组

    // 对输入数组进行滑动均值滤波
    movmean(A, n, k, M);

    // 输出滤波结果
    printf("Original Array: ");
    for (i = 0; i < n; i++) {
        printf("%.2f ", A[i]);
    }
    printf("\n");

    printf("Moving Average: ");
    for (i = 0; i < n; i++) {
        printf("%.2f ", M[i]);
    }
    printf("\n");

    return 0;
}

*/