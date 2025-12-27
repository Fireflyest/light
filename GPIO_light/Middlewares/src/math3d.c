#include "math3d.h"
#include "gfx.h" // For screen center

// Simple approximation to avoid linking libm
static float sinf(float x) {
    const float PI = 3.14159265f;
    const float TWO_PI = 6.2831853f;
    const float HALF_PI = 1.57079632f;

    // 1. Reduce to range [-PI, PI]
    // Use integer division for large angles to avoid slow loops
    int k = (int)(x / TWO_PI);
    x -= k * TWO_PI;
    
    if (x > PI) x -= TWO_PI;
    if (x < -PI) x += TWO_PI;
    
    // 2. Fold range to [-PI/2, PI/2] using symmetry
    // sin(PI - x) = sin(x)
    // sin(-PI - x) = -sin(PI + x) = -sin(x) -> handled by odd function property? 
    // Actually: sin(x) for x in [PI/2, PI] -> sin(PI-x)
    //           sin(x) for x in [-PI, -PI/2] -> sin(-PI-x)
    
    if (x > HALF_PI) {
        x = PI - x;
    } else if (x < -HALF_PI) {
        x = -PI - x;
    }
    
    // 3. Taylor series: x - x^3/6 + x^5/120
    // This is very accurate within [-PI/2, PI/2]
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f + (x2 * x2) / 120.0f);
}

static float cosf(float x) {
    return sinf(x + 1.57079632f);
}

Point2D Math3D_Project(Point3D p, float focalLength, float cameraZ) {
    Point2D p2d;
    float scale = focalLength / (p.z + cameraZ);
    
    p2d.x = (int)(p.x * scale) + GFX_WIDTH / 2;
    p2d.y = (int)(p.y * scale) + GFX_HEIGHT / 2;
    
    return p2d;
}

Point3D Math3D_RotateX(Point3D p, float angle) {
    Point3D newP;
    float c = cosf(angle);
    float s = sinf(angle);
    newP.x = p.x;
    newP.y = p.y * c - p.z * s;
    newP.z = p.y * s + p.z * c;
    return newP;
}

Point3D Math3D_RotateY(Point3D p, float angle) {
    Point3D newP;
    float c = cosf(angle);
    float s = sinf(angle);
    newP.x = p.x * c + p.z * s;
    newP.y = p.y;
    newP.z = -p.x * s + p.z * c;
    return newP;
}

Point3D Math3D_RotateZ(Point3D p, float angle) {
    Point3D newP;
    float c = cosf(angle);
    float s = sinf(angle);
    newP.x = p.x * c - p.y * s;
    newP.y = p.x * s + p.y * c;
    newP.z = p.z;
    return newP;
}

