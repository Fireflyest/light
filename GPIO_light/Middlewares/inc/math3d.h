#ifndef __MATH3D_H
#define __MATH3D_H

typedef struct {
    float x, y, z;
} Point3D;

typedef struct {
    int x, y;
} Point2D;

// Project 3D point to 2D screen coordinates
// focalLength: Distance from eye to screen (e.g., 64)
// cameraZ: Distance from camera to object center (e.g., 100)
Point2D Math3D_Project(Point3D p, float focalLength, float cameraZ);

// Rotate point around axes
Point3D Math3D_RotateX(Point3D p, float angle);
Point3D Math3D_RotateY(Point3D p, float angle);
Point3D Math3D_RotateZ(Point3D p, float angle);

#endif /* __MATH3D_H */
