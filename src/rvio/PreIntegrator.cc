/**
* This file is part of R-VIO.
*
* Copyright (C) 2019 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO> 
*
* R-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/core/core.hpp>

#include "PreIntegrator.h"
#include "../util/Numerics.h"


namespace RVIO
{
bool imuMotDet = true;
bool noMotion = false;
rclcpp::Time lastImuMotionTime;
PreIntegrator::PreIntegrator(const cv::FileStorage& fsSettings, rclcpp::Node::SharedPtr node)
    : mPreIntegratorNode(node)
{
    lastImuMotionTime = mPreIntegratorNode->now();
    mnGravity = fsSettings["IMU.nG"];
    mnSmallAngle = fsSettings["IMU.nSmallAngle"];

    mnGyroNoiseSigma = fsSettings["IMU.sigma_g"];
    mnGyroRandomWalkSigma = fsSettings["IMU.sigma_wg"];
    mnAccelNoiseSigma = fsSettings["IMU.sigma_a"];
    mnAccelRandomWalkSigma = fsSettings["IMU.sigma_wa"];
    wTH = fsSettings["ZUPT.w_th"];
    gmaxTH = fsSettings["ZUPT.g.max_th"];
    gminTH = fsSettings["ZUPT.g.min_th"];
    waitTime = fsSettings["ZUPT.wait_time"];

    // logfile.open("imudata.csv");
    // logfile << "w,a,v,roll,pitch,yaw" << std::endl;
    ImuNoiseMatrix.setIdentity();
    ImuNoiseMatrix.block<3,3>(0,0) *= pow(mnGyroNoiseSigma,2);
    ImuNoiseMatrix.block<3,3>(3,3) *= pow(mnGyroRandomWalkSigma,2);
    ImuNoiseMatrix.block<3,3>(6,6) *= pow(mnAccelNoiseSigma,2);
    ImuNoiseMatrix.block<3,3>(9,9) *= pow(mnAccelRandomWalkSigma,2);

    xk1k.setZero(26,1);
    Pk1k.setZero(24,24);
}


void PreIntegrator::propagate(Eigen::VectorXd& xkk,
                              Eigen::MatrixXd& Pkk,
                              std::list<ImuData*>& lImuData)
{
    Eigen::Vector3d gk = xkk.block(7,0,3,1);  // unit vector
    Eigen::Vector4d qk = xkk.block(10,0,4,1); // [0,0,0,1]
    Eigen::Vector3d pk = xkk.block(14,0,3,1); // [0,0,0]
    Eigen::Vector3d vk = xkk.block(17,0,3,1);
    Eigen::Vector3d bg = xkk.block(20,0,3,1);
    Eigen::Vector3d ba = xkk.block(23,0,3,1);

    // Gravity vector in {R}
    Eigen::Vector3d gR = gk;

    // Local velocity in {R}
    Eigen::Vector3d vR = vk;

    // Relative rotation term
    Eigen::Matrix3d Rk = QuatToRot(qk);
    Eigen::Matrix3d Rk_T = Rk.transpose();

    // Preintegrated terms
    Eigen::Vector3d dp;
    Eigen::Vector3d dv;
    dp.setZero();
    dv.setZero();

    // State transition matrix
    Eigen::Matrix<double,24,24> F;
    Eigen::Matrix<double,24,24> Phi;
    Eigen::Matrix<double,24,24> Psi;
    F.setZero();
    Phi.setZero();
    Psi.setIdentity();

    // Noise matrix
    Eigen::Matrix<double,24,12> G;
    Eigen::Matrix<double,24,24> Q;
    G.setZero();
    Q.setZero();

    Eigen::Matrix3d I;
    I.setIdentity();

    double Dt = 0;
    double w_avg = 0;
    double a_avg = 0;
    
    for (std::list<ImuData*>::const_iterator lit=lImuData.begin();
         lit!=lImuData.end(); ++lit)
    {
        Eigen::Vector3d wm = (*lit)->AngularVel;
        Eigen::Vector3d am = (*lit)->LinearAccel;

        double dt = (*lit)->TimeInterval;
        Dt += dt;

        Eigen::Vector3d w = wm-bg;
        Eigen::Vector3d a = am-ba;

        bool bIsSmallAngle = false;
        if (w.norm()<mnSmallAngle)
            bIsSmallAngle = true;

        double w1 = w.norm();
        double wdt = w1*dt;
        double wdt2 = wdt*wdt;
        double coswdt = cos(wdt);
        double sinwdt = sin(wdt);
        Eigen::Matrix3d wx = SkewSymm(w);
        Eigen::Matrix3d wx2 = wx*wx;
        Eigen::Matrix3d vx = SkewSymm(vk);

        // Covariance
        F.block<3,3>(9,9) = -wx;
        F.block<3,3>(9,18) = -I;
        F.block<3,3>(12,9) = -Rk_T*vx;
        F.block<3,3>(12,15) = Rk_T;
        F.block<3,3>(15,6) = -mnGravity*Rk;
        F.block<3,3>(15,9) = -mnGravity*SkewSymm(gk);
        F.block<3,3>(15,15) = -wx;
        F.block<3,3>(15,18) = -vx;
        F.block<3,3>(15,21) = -I;
        Phi = Eigen::Matrix<double,24,24>::Identity()+dt*F;
        Psi = Phi*Psi;

        G.block<3,3>(9,0) = -I;
        G.block<3,3>(15,0) = -vx;
        G.block<3,3>(15,6) = -I;
        G.block<3,3>(18,3) = I;
        G.block<3,3>(21,9) = I;
        Q = dt*G*ImuNoiseMatrix*(G.transpose());

        Pkk.block(0,0,24,24) = Phi*(Pkk.block(0,0,24,24))*(Phi.transpose())+Q;

        // State
        Eigen::Matrix3d deltaR;
        double f1, f2, f3, f4;
        if (bIsSmallAngle)
        {
            deltaR = I-dt*wx+(pow(dt,2)/2)*wx2;
            assert(std::isnan(deltaR.norm())!=true);

            f1 = -pow(dt,3)/3;
            f2 = pow(dt,4)/8;
            f3 = -pow(dt,2)/2;
            f4 = pow(dt,3)/6;
        }
        else
        {
            deltaR = I-(sinwdt/w1)*wx+((1-coswdt)/pow(w1,2))*wx2;
            assert(std::isnan(deltaR.norm())!=true);

            f1 = (wdt*coswdt-sinwdt)/pow(w1,3);
            f2 = .5*(wdt2-2*coswdt-2*wdt*sinwdt+2)/pow(w1,4);
            f3 = (coswdt-1)/pow(w1,2);
            f4 = (wdt-sinwdt)/pow(w1,3);
        }

        Rk = deltaR*Rk;
        Rk_T = Rk.transpose();

        dp += dv*dt;
        dp += Rk_T*(.5*pow(dt,2)*I+f1*wx+f2*wx2)*a;
        dv += Rk_T*(dt*I+f3*wx+f4*wx2)*a;

        pk = vR*Dt-.5*mnGravity*gR*pow(Dt,2)+dp;
        vk = Rk*(vR-mnGravity*gR*Dt+dv);
        gk = Rk*gR;
        gk.normalize();

        w_avg += wdt;
        a_avg += a.norm() * dt;
    }
    w_avg /= Dt;
    a_avg /= Dt;
    Eigen::Vector3d euler = Rk.eulerAngles(2, 1, 0); // yaw (Z), pitch (Y), roll (X)
    // logfile << w_avg << "," << a_avg << "," << vk.norm() << "," << euler[2]  << "," << euler[1] << "," << euler[0] << std::endl;
    if(w_avg < wTH && (a_avg < gmaxTH  && a_avg > gminTH ) ){
        imuMotDet = false;
    }else{
        imuMotDet = true;
        lastImuMotionTime = mPreIntegratorNode->now();
    }
    if(((mPreIntegratorNode->now() - lastImuMotionTime).seconds() > waitTime) && ((mPreIntegratorNode->now() - lastImgMotionTime).seconds() > waitTime)){
        noMotion = true;
        std::cout << "No motion mode!" << std::endl;
    }else{
        noMotion = false;
    }
    xk1k = xkk;
    xk1k.block(10,0,4,1) = RotToQuat(Rk);
    xk1k.block(14,0,3,1) = pk;
    if (noMotion)
        vk *= 0;
    xk1k.block(17,0,3,1) = vk;

    int nCloneStates = (xkk.rows()-26)/7;
    if (nCloneStates>0)
    {
        Pkk.block(0,24,24,6*nCloneStates) = Psi*Pkk.block(0,24,24,6*nCloneStates);
        Pkk.block(24,0,6*nCloneStates,24) = Pkk.block(0,24,24,6*nCloneStates).transpose();
    }
    Pkk = .5*(Pkk+Pkk.transpose());
    Pk1k = Pkk;
}

} // namespace RVIO
