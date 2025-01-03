#include <stdio.h>
#include <math.h>

// 定义四元数结构体
typedef struct {
    double w;
    double x;
    double y;
    double z;
} Quaternion;

// // 计算两个四元数之间的球面线性插值(Slerp)
// void quaternion_slerp(const Quaternion *q1, const Quaternion *q2, double t, Quaternion *result) {
//     // 计算点积
//     double dot = q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;

//     // 如果点积为负数，将其中一个四元数取反，确保选择最短的插值路径
//     if (dot < 0.0) {
//         dot = -dot;
//         result->w = -q2->w;
//         result->x = -q2->x;
//         result->y = -q2->y;
//         result->z = -q2->z;
//     } else {
//         result->w = q2->w;
//         result->x = q2->x;
//         result->y = q2->y;
//         result->z = q2->z;
//     }

//     // 插值
//     if (1.0 - dot > 0.0001) {
//         double angle = acos(dot);
//         double sin_angle = sin(angle);
//         double inv_sin_angle = 1.0 / sin_angle;
//         double coeff1 = sin((1.0 - t) * angle) * inv_sin_angle;
//         double coeff2 = sin(t * angle) * inv_sin_angle;

//         result->w = q1->w * coeff1 + result->w * coeff2;
//         result->x = q1->x * coeff1 + result->x * coeff2;
//         result->y = q1->y * coeff1 + result->y * coeff2;
//         result->z = q1->z * coeff1 + result->z * coeff2;
//     }
// }

void Quaternion_set(double w, double v1, double v2, double v3, Quaternion* output)
{
    // assert(output != NULL);
    output->w = w;
    output->x = v1;
    output->y = v2;
    output->z = v3;
}

void Quaternion_copy(Quaternion* q, Quaternion* output)
{
    Quaternion_set(q->w, q->x, q->y, q->z, output);
}

void quaternion_slerp(Quaternion* q1, Quaternion* q2, double t, Quaternion* output)
{
    // printf("%d\n", 111);

    Quaternion result, q2_new;

    // Based on http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    double cosHalfTheta = q1->w*q2->w + q1->x*q2->x + q1->y*q2->y + q1->z*q2->z;

    // // if q1=q2 or qa=-q2 then theta = 0 and we can return qa
    // if (fabs(cosHalfTheta) >= 1.0) {
    //     Quaternion_copy(q1, output);
    //     return;
    // }

    // 如果点积为负数，将其中一个四元数取反，确保选择最短的插值路径
    if (cosHalfTheta < 0.0) {
        cosHalfTheta = -cosHalfTheta;
        q2_new.w = -q2->w;
        q2_new.x = -q2->x;
        q2_new.y = -q2->y;
        q2_new.z = -q2->z;
    } else {
        q2_new.w = q2->w;
        q2_new.x = q2->x;
        q2_new.y = q2->y;
        q2_new.z = q2->z;
    }

    // // if q1=q2 or qa=-q2 then theta = 0 and we can return qa
    // if (fabs(cosHalfTheta) >= 1.0) {
    //     Quaternion_copy(q1, output);
    //     printf("%d\n", 222);
    //     return;
    // }

    double halfTheta = acos(cosHalfTheta);
    // double sinHalfTheta = sin(halfTheta);
    double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
    if (fabs(sinHalfTheta) < 0.0001) {
        printf("%d\n", 333);
        result.w = (q1->w * 0.5 + q2_new.w * 0.5);
        result.x = (q1->x * 0.5 + q2_new.x * 0.5);
        result.y = (q1->y * 0.5 + q2_new.y * 0.5);
        result.z = (q1->z * 0.5 + q2_new.z * 0.5);
    } else {
        printf("%d\n", 444);
        double ratioA = sin((1 - t) * halfTheta) * 1.0 / sinHalfTheta;
        double ratioB = sin(t * halfTheta) * 1.0 / sinHalfTheta;
        printf("%.3f, %.3f\n", ratioA, ratioB);
        result.w = (q1->w * ratioA + q2_new.w * ratioB);
        result.x = (q1->x * ratioA + q2_new.x * ratioB);
        result.y = (q1->y * ratioA + q2_new.y * ratioB);
        result.z = (q1->z * ratioA + q2_new.z * ratioB);
    }
    *output = result;
}

// 计算多个四元数的平均值
void quaternion_average(Quaternion *quaternions, int count, Quaternion *average) {
    average->w = 0.0;
    average->x = 0.0;
    average->y = 0.0;
    average->z = 0.0;

    for (int i = 0; i < count; i++) {
        Quaternion temp;
        quaternion_slerp(average, &quaternions[i], 1.0 / (i + 1), &temp);
        *average = temp;
    }
}

/*
int main() {
    // 示例：定义四个四元数
    Quaternion quaternions[200] = {
        // {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        // {0.9659, 0.0, 0.2588, 0.0},
        // {0.8660, 0.0, 0.5000, 0.0},
        // {0.7071, 0.0, 0.7071, 0.0},

        // {NAN, NAN, NAN, NAN},  
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},  // 单位四元数
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        // {0.707, 0.0, 0.707, 0.0},  // 45/90 度绕 y 轴旋转的四元数
        // {0.5, 0.5, 0.5, 0.5},  // 任意四元数
        // {0.0, 1.0, 0.0, 0.0}  // 180 度绕 x 轴旋转的四元数

    };

    // 计算四个四元数的平均值
    Quaternion average;
    quaternion_average(quaternions, 200, &average);

    // 输出平均值
    printf("Average Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", average.w, average.x, average.y, average.z);

    return 0;
}

*/