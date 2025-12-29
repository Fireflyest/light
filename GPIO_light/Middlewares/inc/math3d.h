#ifndef __MATH3D_H
#define __MATH3D_H

typedef struct {
    float x, y, z;
} Point3D;

typedef struct {
    float x, y, z;
} Vector3D;

typedef struct {
    float m[4][4];
} Matrix4x4;

typedef struct { float w, x, y, z; } Quaternion;


#define FOCAL_LENGTH_DEFAULT 64.0f
#define CAMERA_Z_DEFAULT    100.0f

// Project 3D point to 2D screen coordinates
// focalLength: Distance from eye to screen (e.g., 64)
// cameraZ: Distance from camera to object center (e.g., 100)
void Math3D_Project(Point3D* p, float focalLength, float cameraZ);

// build quaternion from Euler angles (radians), order ZYX (yaw, pitch, roll)
Quaternion Math3D_QuatFromEuler(float yaw, float pitch, float roll);
// normalize quaternion
Quaternion Math3D_QuatNormalize(Quaternion q);
// rotate point by quaternion (fast)
Point3D Math3D_RotateByQuat(Point3D p, Quaternion q);
// convenience: build quaternion from axis-angle (axis must be normalized)
Quaternion Math3D_QuatFromAxisAngle(Vector3D axis, float angle);

// Rotate point around axes
Point3D Math3D_RotateX(Point3D p, float angle);
Point3D Math3D_RotateY(Point3D p, float angle);
Point3D Math3D_RotateZ(Point3D p, float angle);

#endif /* __MATH3D_H */
