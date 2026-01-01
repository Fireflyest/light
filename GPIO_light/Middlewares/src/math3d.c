#include "math3d.h"
#include "gfx.h" // For screen center
#include "math.h"
#include "arm_math.h"


// Simple approximation to avoid linking libm
// static float arm_sin_f32(float x) {
//     const float PI = 3.14159265f;
//     const float TWO_PI = 6.2831853f;
//     const float HALF_PI = 1.57079632f;

//     // 1. Reduce to range [-PI, PI]
//     // Use integer division for large angles to avoid slow loops
//     int k = (int)(x / TWO_PI);
//     x -= k * TWO_PI;
    
//     if (x > PI) x -= TWO_PI;
//     if (x < -PI) x += TWO_PI;
    
//     // 2. Fold range to [-PI/2, PI/2] using symmetry
//     // sin(PI - x) = sin(x)
//     // sin(-PI - x) = -sin(PI + x) = -sin(x) -> handled by odd function property? 
//     // Actually: sin(x) for x in [PI/2, PI] -> sin(PI-x)
//     //           sin(x) for x in [-PI, -PI/2] -> sin(-PI-x)
    
//     if (x > HALF_PI) {
//         x = PI - x;
//     } else if (x < -HALF_PI) {
//         x = -PI - x;
//     }
    
//     // 3. Taylor series: x - x^3/6 + x^5/120
//     // This is very accurate within [-PI/2, PI/2]
//     float x2 = x * x;
//     return x * (1.0f - x2 / 6.0f + (x2 * x2) / 120.0f);
// }

// static float arm_cos_f32(float x) {
//     return arm_sin_f32(x + 1.57079632f);
// }

// static float sqrtf(float x) {
//     if (x <= 0.0f) return 0.0f;

//     union { uint32_t i; float f; } u;
//     u.f = x;

//     /* initial guess for 1/sqrt(x) */
//     u.i = 0x5f3759df - (u.i >> 1);
//     float y = u.f;

//     /* two Newton-Raphson iterations to refine y = 1/sqrt(x) */
//     y = y * (1.5f - 0.5f * x * y * y);
//     y = y * (1.5f - 0.5f * x * y * y);

//     /* sqrt(x) = x * (1/sqrt(x)) */
//     return x * y;
// }

static inline int round_to_int(float v) {
    return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

void Math3D_Project(Point3D* p, float focalLength, float cameraZ) {
    float scale = focalLength / (p->z + cameraZ);
    p->x = round_to_int(p->x * scale) + OLED_WIDTH / 2;
    p->y = round_to_int(p->y * scale) + OLED_HEIGHT / 2;
}

Quaternion Math3D_QuatFromEuler(float yaw, float pitch, float roll) {
    float cy = arm_cos_f32(yaw * 0.5f), sy = arm_sin_f32(yaw * 0.5f);
    float cp = arm_cos_f32(pitch * 0.5f), sp = arm_sin_f32(pitch * 0.5f);
    float cr = arm_cos_f32(roll * 0.5f), sr = arm_sin_f32(roll * 0.5f);
    Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

Quaternion Math3D_QuatFromAxisAngle(Vector3D axis, float angle) {
    float s = arm_sin_f32(angle * 0.5f);
    Quaternion q;
    q.w = arm_cos_f32(angle * 0.5f);
    q.x = axis.x * s;
    q.y = axis.y * s;
    q.z = axis.z * s;
    return q;
}

Quaternion Math3D_QuatNormalize(Quaternion q) {
    float n = 0;
    arm_sqrt_f32(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z, &n);
    if (n <= 0.0f) return (Quaternion){1,0,0,0};
    float inv = 1.0f / n;
    q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
    return q;
}

// Fast rotate vector p by quaternion q (uses optimized cross-product form)
Point3D Math3D_RotateByQuat(Point3D v, Quaternion q) {
    // t = 2 * cross(q_vec, v)
    float tx = 2.0f * (q.y * v.z - q.z * v.y);
    float ty = 2.0f * (q.z * v.x - q.x * v.z);
    float tz = 2.0f * (q.x * v.y - q.y * v.x);
    // v' = v + q.w * t + cross(q_vec, t)
    float cx = q.y * tz - q.z * ty;
    float cy = q.z * tx - q.x * tz;
    float cz = q.x * ty - q.y * tx;
    Point3D out;
    out.x = v.x + q.w * tx + cx;
    out.y = v.y + q.w * ty + cy;
    out.z = v.z + q.w * tz + cz;
    return out;
}

Point3D Math3D_RotateX(Point3D p, float angle) {
    Point3D newP;
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    newP.x = p.x;
    newP.y = p.y * c - p.z * s;
    newP.z = p.y * s + p.z * c;
    return newP;
}

Point3D Math3D_RotateY(Point3D p, float angle) {
    Point3D newP;
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    newP.x = p.x * c + p.z * s;
    newP.y = p.y;
    newP.z = -p.x * s + p.z * c;
    return newP;
}

Point3D Math3D_RotateZ(Point3D p, float angle) {
    Point3D newP;
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    newP.x = p.x * c - p.y * s;
    newP.y = p.x * s + p.y * c;
    newP.z = p.z;
    return newP;
}

