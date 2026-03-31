#pragma once

#include <cmath>
#include <array>

struct Mat4 
{
    double m[4][4];
    static Mat4 identity() 
    {
        Mat4 I{};
        for (int i = 0; i < 4; i++) I.m[i][i] = 1.0;
        return I;
    }
};

vec3 transformVec3(const vec3& v, const Mat4& M, double w) 
{
    double x = v.x() * M.m[0][0] + v.y() * M.m[1][0] + v.z() * M.m[2][0] + w * M.m[3][0];
    double y = v.x() * M.m[0][1] + v.y() * M.m[1][1] + v.z() * M.m[2][1] + w * M.m[3][1];
    double z = v.x() * M.m[0][2] + v.y() * M.m[1][2] + v.z() * M.m[2][2] + w * M.m[3][2];
    double w_out = v.x() * M.m[0][3] + v.y() * M.m[1][3] + v.z() * M.m[2][3] + w * M.m[3][3];

    if (w_out != 0.0 && w != 0.0)
    {
        x /= w_out; y /= w_out; z /= w_out;
    }
    return vec3(x, y, z);
}


Mat4 makeTranslation(double tx, double ty, double tz) 
{
    Mat4 T = Mat4::identity();
    T.m[3][0] = tx; T.m[3][1] = ty; T.m[3][2] = tz;
    return T;
}

Mat4 makeScale(double sx, double sy, double sz) 
{
    Mat4 S = Mat4::identity();
    S.m[0][0] = sx; S.m[1][1] = sy; S.m[2][2] = sz;
    return S;
}

Mat4 makeRotationY(double angle) 
{
    Mat4 R = Mat4::identity();
    double c = cos(angle), s = sin(angle);
    R.m[0][0] = c; R.m[0][2] = s;
    R.m[2][0] = -s; R.m[2][2] = c;
    return R;
}

Mat4 transpose(const Mat4& M) {
    Mat4 R;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            R.m[i][j] = M.m[j][i];
        }
    }
    return R;
}

Mat4 inverse(const Mat4& M) 
{
    Mat4 inv;
    double det;

    inv.m[0][0] = M.m[1][1] * M.m[2][2] * M.m[3][3] -
        M.m[1][1] * M.m[2][3] * M.m[3][2] -
        M.m[2][1] * M.m[1][2] * M.m[3][3] +
        M.m[2][1] * M.m[1][3] * M.m[3][2] +
        M.m[3][1] * M.m[1][2] * M.m[2][3] -
        M.m[3][1] * M.m[1][3] * M.m[2][2];

    inv.m[0][1] = -M.m[0][1] * M.m[2][2] * M.m[3][3] +
        M.m[0][1] * M.m[2][3] * M.m[3][2] +
        M.m[2][1] * M.m[0][2] * M.m[3][3] -
        M.m[2][1] * M.m[0][3] * M.m[3][2] -
        M.m[3][1] * M.m[0][2] * M.m[2][3] +
        M.m[3][1] * M.m[0][3] * M.m[2][2];

    inv.m[0][2] = M.m[0][1] * M.m[1][2] * M.m[3][3] -
        M.m[0][1] * M.m[1][3] * M.m[3][2] -
        M.m[1][1] * M.m[0][2] * M.m[3][3] +
        M.m[1][1] * M.m[0][3] * M.m[3][2] +
        M.m[3][1] * M.m[0][2] * M.m[1][3] -
        M.m[3][1] * M.m[0][3] * M.m[1][2];

    inv.m[0][3] = -M.m[0][1] * M.m[1][2] * M.m[2][3] +
        M.m[0][1] * M.m[1][3] * M.m[2][2] +
        M.m[1][1] * M.m[0][2] * M.m[2][3] -
        M.m[1][1] * M.m[0][3] * M.m[2][2] -
        M.m[2][1] * M.m[0][2] * M.m[1][3] +
        M.m[2][1] * M.m[0][3] * M.m[1][2];

    inv.m[1][0] = -M.m[1][0] * M.m[2][2] * M.m[3][3] +
        M.m[1][0] * M.m[2][3] * M.m[3][2] +
        M.m[2][0] * M.m[1][2] * M.m[3][3] -
        M.m[2][0] * M.m[1][3] * M.m[3][2] -
        M.m[3][0] * M.m[1][2] * M.m[2][3] +
        M.m[3][0] * M.m[1][3] * M.m[2][2];

    inv.m[1][1] = M.m[0][0] * M.m[2][2] * M.m[3][3] -
        M.m[0][0] * M.m[2][3] * M.m[3][2] -
        M.m[2][0] * M.m[0][2] * M.m[3][3] +
        M.m[2][0] * M.m[0][3] * M.m[3][2] +
        M.m[3][0] * M.m[0][2] * M.m[2][3] -
        M.m[3][0] * M.m[0][3] * M.m[2][2];

    inv.m[1][2] = -M.m[0][0] * M.m[1][2] * M.m[3][3] +
        M.m[0][0] * M.m[1][3] * M.m[3][2] +
        M.m[1][0] * M.m[0][2] * M.m[3][3] -
        M.m[1][0] * M.m[0][3] * M.m[3][2] -
        M.m[3][0] * M.m[0][2] * M.m[1][3] +
        M.m[3][0] * M.m[0][3] * M.m[1][2];

    inv.m[1][3] = M.m[0][0] * M.m[1][2] * M.m[2][3] -
        M.m[0][0] * M.m[1][3] * M.m[2][2] -
        M.m[1][0] * M.m[0][2] * M.m[2][3] +
        M.m[1][0] * M.m[0][3] * M.m[2][2] +
        M.m[2][0] * M.m[0][2] * M.m[1][3] -
        M.m[2][0] * M.m[0][3] * M.m[1][2];

    inv.m[2][0] = M.m[1][0] * M.m[2][1] * M.m[3][3] -
        M.m[1][0] * M.m[2][3] * M.m[3][1] -
        M.m[2][0] * M.m[1][1] * M.m[3][3] +
        M.m[2][0] * M.m[1][3] * M.m[3][1] +
        M.m[3][0] * M.m[1][1] * M.m[2][3] -
        M.m[3][0] * M.m[1][3] * M.m[2][1];

    inv.m[2][1] = -M.m[0][0] * M.m[2][1] * M.m[3][3] +
        M.m[0][0] * M.m[2][3] * M.m[3][1] +
        M.m[2][0] * M.m[0][1] * M.m[3][3] -
        M.m[2][0] * M.m[0][3] * M.m[3][1] -
        M.m[3][0] * M.m[0][1] * M.m[2][3] +
        M.m[3][0] * M.m[0][3] * M.m[2][1];

    inv.m[2][2] = M.m[0][0] * M.m[1][1] * M.m[3][3] -
        M.m[0][0] * M.m[1][3] * M.m[3][1] -
        M.m[1][0] * M.m[0][1] * M.m[3][3] +
        M.m[1][0] * M.m[0][3] * M.m[3][1] +
        M.m[3][0] * M.m[0][1] * M.m[1][3] -
        M.m[3][0] * M.m[0][3] * M.m[1][1];

    inv.m[2][3] = -M.m[0][0] * M.m[1][1] * M.m[2][3] +
        M.m[0][0] * M.m[1][3] * M.m[2][1] +
        M.m[1][0] * M.m[0][1] * M.m[2][3] -
        M.m[1][0] * M.m[0][3] * M.m[2][1] -
        M.m[2][0] * M.m[0][1] * M.m[1][3] +
        M.m[2][0] * M.m[0][3] * M.m[1][1];

    inv.m[3][0] = -M.m[1][0] * M.m[2][1] * M.m[3][2] +
        M.m[1][0] * M.m[2][2] * M.m[3][1] +
        M.m[2][0] * M.m[1][1] * M.m[3][2] -
        M.m[2][0] * M.m[1][2] * M.m[3][1] -
        M.m[3][0] * M.m[1][1] * M.m[2][2] +
        M.m[3][0] * M.m[1][2] * M.m[2][1];

    inv.m[3][1] = M.m[0][0] * M.m[2][1] * M.m[3][2] -
        M.m[0][0] * M.m[2][2] * M.m[3][1] -
        M.m[2][0] * M.m[0][1] * M.m[3][2] +
        M.m[2][0] * M.m[0][2] * M.m[3][1] +
        M.m[3][0] * M.m[0][1] * M.m[2][2] -
        M.m[3][0] * M.m[0][2] * M.m[2][1];

    inv.m[3][2] = -M.m[0][0] * M.m[1][1] * M.m[3][2] +
        M.m[0][0] * M.m[1][2] * M.m[3][1] +
        M.m[1][0] * M.m[0][1] * M.m[3][2] -
        M.m[1][0] * M.m[0][2] * M.m[3][1] -
        M.m[3][0] * M.m[0][1] * M.m[1][2] +
        M.m[3][0] * M.m[0][2] * M.m[1][1];

    inv.m[3][3] = M.m[0][0] * M.m[1][1] * M.m[2][2] -
        M.m[0][0] * M.m[1][2] * M.m[2][1] -
        M.m[1][0] * M.m[0][1] * M.m[2][2] +
        M.m[1][0] * M.m[0][2] * M.m[2][1] +
        M.m[2][0] * M.m[0][1] * M.m[1][2] -
        M.m[2][0] * M.m[0][2] * M.m[1][1];

    det = M.m[0][0] * inv.m[0][0] + M.m[0][1] * inv.m[1][0] + M.m[0][2] * inv.m[2][0] + M.m[0][3] * inv.m[3][0];

   // if (fabs(det) < 1e-9) 
 //   {
 //       throw std::runtime_error("Matrix is singular and cannot be inverted");
  //  }

    det = 1.0 / det;

    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++)
        {
            inv.m[i][j] *= det;
        }
    }

    return inv;
}

vec3 transformNormal(const vec3& n, const Mat4& M)
{
    Mat4 invM = inverse(M);
    Mat4 invTransM = transpose(invM);

    vec3 nTrans = transformVec3(n, invTransM, 0.0);

    return unit_vector(nTrans);
}

