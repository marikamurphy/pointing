#ifndef PROJECTIVE_UTIL_H
#define PROJECTIVE_UTIL_H

Eigen::MatrixXf getPIdent() {
    Eigen::MatrixXf pIdent = ArrayXXf::Zero(3, 4);
    for(int i = 0; i < 3; i++)
        pIdent(i,i) = 1.0f;
    return pIdent;
}

Eigen::MatrixXf getCameraIntrinsicMatrix(float fx, float fy, float cx, float cy) {
    Eigen::MatrixXf cameraIntrinsic = Eigen::MatrixXf::Identity(3, 3);
    cameraIntrinsic(0,0) = fx;
    cameraIntrinsic(1,1) = fy;
    cameraIntrinsic(0,3) = cx;
    cameraIntrinsic(1,3) = cy;
    return cameraIntrinsic;
}

#endif