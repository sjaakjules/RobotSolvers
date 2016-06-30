using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework; 
using CustomExtensions;

namespace RobotSolver
{
    class KinematicSolver
    {
        /// <summary>
        /// Returns a Pose for ABB 120 robot.
        /// </summary>
        /// <param name="theta"></param>
        /// <returns></returns>
        public Pose forwardKinimatics(double[] theta)
        {
            if (theta.Length == 6)
            {
                double[] robotConfig = new double[] { 290, 270, 70, 302, 72 };

                double s1 = Math.Sin(theta[0]);
                double c1 = Math.Cos(theta[0]);
                double s2 = Math.Sin(theta[1]);
                double c2 = Math.Cos(theta[1]);
                double s2p = Math.Sin(theta[1] - Math.PI / 2);
                double c2p = Math.Cos(theta[1] - Math.PI / 2);
                double s3 = Math.Sin(theta[2]);
                double c3 = Math.Cos(theta[2]);
                double s4 = Math.Sin(theta[3]);
                double c4 = Math.Cos(theta[3]);
                double s5 = Math.Sin(theta[4]);
                double c5 = Math.Cos(theta[4]);
                double s6 = Math.Sin(theta[5]);
                double c6 = Math.Cos(theta[5]);

                double s23 = Math.Sin(theta[1] - Math.PI / 2 + theta[2]);
                double c23 = Math.Cos(theta[1] - Math.PI / 2 + theta[2]);

                double l1 = robotConfig[0];
                double l2 = robotConfig[1];
                double l3 = robotConfig[2];
                double l4 = robotConfig[3];
                double l5 = robotConfig[4];

                // This has been copied from MatLab and transposed for row bias matrix, where as matlab uses Column (and all liturature online)
                double m11 = -s6 * (c4 * s1 + s4 * (c1 * s3 * s2p - c1 * c3 * c2p)) - c6 * (c5 * (s1 * s4 - c4 * (c1 * s3 * s2p - c1 * c3 * c2p)) - s5 * (c1 * c3 * s2p + c1 * c2p * s3));
                double m12 = s6 * (c1 * c4 + s4 * (c3 * c2p * s1 - s1 * s3 * s2p)) + c6 * (c5 * (c1 * s4 - c4 * (c3 * c2p * s1 - s1 * s3 * s2p)) + s5 * (c3 * s1 * s2p + c2p * s1 * s3));
                double m13 = c6 * (s5 * (c3 * c2p - s3 * s2p) + c4 * c5 * (c3 * s2p + c2p * s3)) - s4 * s6 * (c3 * s2p + c2p * s3);
                double m14 = 0;
                double m21 = s6 * (c5 * (s1 * s4 - c4 * (c1 * s3 * s2p - c1 * c3 * c2p)) - s5 * (c1 * c3 * s2p + c1 * c2p * s3)) - c6 * (c4 * s1 + s4 * (c1 * s3 * s2p - c1 * c3 * c2p));
                double m22 = c6 * (c1 * c4 + s4 * (c3 * c2p * s1 - s1 * s3 * s2p)) - s6 * (c5 * (c1 * s4 - c4 * (c3 * c2p * s1 - s1 * s3 * s2p)) + s5 * (c3 * s1 * s2p + c2p * s1 * s3));
                double m23 = -s6 * (s5 * (c3 * c2p - s3 * s2p) + c4 * c5 * (c3 * s2p + c2p * s3)) - c6 * s4 * (c3 * s2p + c2p * s3);
                double m24 = 0;
                double m31 = -s5 * (s1 * s4 - c4 * (c1 * s3 * s2p - c1 * c3 * c2p)) - c5 * (c1 * c3 * s2p + c1 * c2p * s3);
                double m32 = s5 * (c1 * s4 - c4 * (c3 * c2p * s1 - s1 * s3 * s2p)) - c5 * (c3 * s1 * s2p + c2p * s1 * s3);
                double m33 = c4 * s5 * (c3 * s2p + c2p * s3) - c5 * (c3 * c2p - s3 * s2p);
                double m34 = 0;
                double m41 = l2 * c1 * s2 + l4 * c1 * c2 * c3 + l3 * c1 * c2 * s3 + l3 * c1 * c3 * s2 - l4 * c1 * s2 * s3 - l5 * s1 * s4 * s5 + l5 * c1 * c2 * c3 * c5 - l5 * c1 * c5 * s2 * s3 - l5 * c1 * c2 * c4 * s3 * s5 - l5 * c1 * c3 * c4 * s2 * s5;
                double m42 = l2 * s1 * s2 + l4 * c2 * c3 * s1 + l3 * c2 * s1 * s3 + l3 * c3 * s1 * s2 + l5 * c1 * s4 * s5 - l4 * s1 * s2 * s3 + l5 * c2 * c3 * c5 * s1 - l5 * c5 * s1 * s2 * s3 - l5 * c2 * c4 * s1 * s3 * s5 - l5 * c3 * c4 * s1 * s2 * s5;
                double m43 = l1 - l4 * c23 - l3 * s23 + l2 * c2 - (l5 * Math.Sin(theta[3] - theta[4]) * s23) / 2 + (l5 * s23 * Math.Sin(theta[3] + theta[4])) / 2 - l5 * c23 * c5;
                double m44 = 1;

                Matrix transform = new Matrix((float)m11, (float)m12, (float)m13, (float)m14, (float)m21, (float)m22, (float)m23, (float)m24, (float)m31, (float)m32, (float)m33, (float)m34, (float)m41, (float)m42, (float)m43, (float)m44);

                return new Pose(transform);
            }
            return Pose.Zero;
        }

        public Pose forwardKinimatics(double[] theta, Pose endEffector)
        {
            Pose tool0 = forwardKinimatics(theta);
            return tool0 * endEffector;
        }

        /*
         * 
         * From Matlab, FK solution using DH values: 
         * robotDH = [0       0   t(1)      l1;
         *           -pi/2    0   t(2)-pi/2  0;
         *              0      l2  t(3)       0;
         *           -pi/2   l3  t(4)+pi    l4;
         *           -pi/2   0   t(5)       0;
         *           pi/2    0   t(6)       0;
         *             0     0    0        l5];
 
[ - s6*(c4*sin(t1) + s4*(cos(t1)*s3*sin(t2 - pi/2) - cos(t1)*c3*cos(t2 - pi/2))) - c6*(c5*(sin(t1)*s4 - c4*(cos(t1)*s3*sin(t2 - pi/2) - cos(t1)*c3*cos(t2 - pi/2))) - s5*(cos(t1)*c3*sin(t2 - pi/2) + cos(t1)*cos(t2 - pi/2)*s3)), s6*(c5*(sin(t1)*s4 - c4*(cos(t1)*s3*sin(t2 - pi/2) - cos(t1)*c3*cos(t2 - pi/2))) - s5*(cos(t1)*c3*sin(t2 - pi/2) + cos(t1)*cos(t2 - pi/2)*s3)) - c6*(c4*sin(t1) + s4*(cos(t1)*s3*sin(t2 - pi/2) - cos(t1)*c3*cos(t2 - pi/2))), - s5*(sin(t1)*s4 - c4*(cos(t1)*s3*sin(t2 - pi/2) - cos(t1)*c3*cos(t2 - pi/2))) - c5*(cos(t1)*c3*sin(t2 - pi/2) + cos(t1)*cos(t2 - pi/2)*s3), l2*cos(t1)*sin(t2) + l4*cos(t1)*cos(t2)*c3 + l3*cos(t1)*cos(t2)*s3 + l3*cos(t1)*c3*sin(t2) - l4*cos(t1)*sin(t2)*s3 - l5*sin(t1)*s4*s5 + l5*cos(t1)*cos(t2)*c3*c5 - l5*cos(t1)*c5*sin(t2)*s3 - l5*cos(t1)*cos(t2)*c4*s3*s5 - l5*cos(t1)*c3*c4*sin(t2)*s5]
[   s6*(cos(t1)*c4 + s4*(c3*cos(t2 - pi/2)*sin(t1) - sin(t1)*s3*sin(t2 - pi/2))) + c6*(c5*(cos(t1)*s4 - c4*(c3*cos(t2 - pi/2)*sin(t1) - sin(t1)*s3*sin(t2 - pi/2))) + s5*(c3*sin(t1)*sin(t2 - pi/2) + cos(t2 - pi/2)*sin(t1)*s3)), c6*(cos(t1)*c4 + s4*(c3*cos(t2 - pi/2)*sin(t1) - sin(t1)*s3*sin(t2 - pi/2))) - s6*(c5*(cos(t1)*s4 - c4*(c3*cos(t2 - pi/2)*sin(t1) - sin(t1)*s3*sin(t2 - pi/2))) + s5*(c3*sin(t1)*sin(t2 - pi/2) + cos(t2 - pi/2)*sin(t1)*s3)),   s5*(cos(t1)*s4 - c4*(c3*cos(t2 - pi/2)*sin(t1) - sin(t1)*s3*sin(t2 - pi/2))) - c5*(c3*sin(t1)*sin(t2 - pi/2) + cos(t2 - pi/2)*sin(t1)*s3), l2*sin(t1)*sin(t2) + l4*cos(t2)*c3*sin(t1) + l3*cos(t2)*sin(t1)*s3 + l3*c3*sin(t1)*sin(t2) + l5*cos(t1)*s4*s5 - l4*sin(t1)*sin(t2)*s3 + l5*cos(t2)*c3*c5*sin(t1) - l5*c5*sin(t1)*sin(t2)*s3 - l5*cos(t2)*c4*sin(t1)*s3*s5 - l5*c3*c4*sin(t1)*sin(t2)*s5]
[                                                                                           c6*(s5*(c3*cos(t2 - pi/2) - s3*sin(t2 - pi/2)) + c4*c5*(c3*sin(t2 - pi/2) + cos(t2 - pi/2)*s3)) - s4*s6*(c3*sin(t2 - pi/2) + cos(t2 - pi/2)*s3),                                                                                       - s6*(s5*(c3*cos(t2 - pi/2) - s3*sin(t2 - pi/2)) + c4*c5*(c3*sin(t2 - pi/2) + cos(t2 - pi/2)*s3)) - c6*s4*(c3*sin(t2 - pi/2) + cos(t2 - pi/2)*s3),                                                       c4*s5*(c3*sin(t2 - pi/2) + cos(t2 - pi/2)*s3) - c5*(c3*cos(t2 - pi/2) - s3*sin(t2 - pi/2)),                                                                                                                                                  l1 - l4*cos(t2 - pi/2 + t3) - l3*sin(t2 - pi/2 + t3) + l2*cos(t2) - (l5*sin(t4 - t5)*sin(t2 - pi/2 + t3))/2 + (l5*sin(t2 - pi/2 + t3)*sin(t4 + t5))/2 - l5*cos(t2 - pi/2 + t3)*c5]
[                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                       1]
 
         * 
         */


        /// <summary>
        /// IK given Pose, end effector size, last motor angles and reference elbow or base direction.
        /// The angles are in radians
        /// </summary>
        /// <param name="DesiredPose"></param>
        /// <param name="EE"></param>
        /// <param name="thetaLast"></param>
        /// <param name="elbow"></param>
        /// <param name="basePos"></param>
        /// <returns></returns>
        public double[] IKSolver(Pose DesiredPose, Pose EE, double[] thetaLast, ref ElbowPosition elbow, ref BasePosition basePos)
        {
            // Rotate the desired pose to alight tool tip with last link.
           // DesiredPose.Orientation = DesiredPose.Orientation * TaskspaceRotation;
            if (float.IsNaN( DesiredPose.Translation.X ) || float.IsNaN( DesiredPose.Translation.Y ) ||float.IsNaN( DesiredPose.Translation.Z ))
            {
                DesiredPose = forwardKinimatics(thetaLast, EE);
            }
            if (DesiredPose.Translation.Z < 0)
            {
                throw new Exception("Below table!");
            }
            double theta1, theta2, theta3, theta4, theta5, theta6;
            double[] angles1to3 = IK1to3(DesiredPose, EE, thetaLast, ref elbow, ref basePos);
            theta1 = angles1to3[0];
            theta2 = angles1to3[1];
            theta3 = angles1to3[2];

            double[,] r = DesiredPose.getMatrix;
            double[,] T30 = new double[,] { { Math.Sin(theta2 + theta3) * Math.Cos(theta1),     -Math.Sin(theta2 + theta3) * Math.Sin(theta1),  Math.Cos(theta2 + theta3),  -400 * Math.Cos(theta2 + theta3) - 25 * Math.Sin(theta2 + theta3) - 560 * Math.Sin(theta3) }, 
                                            { Math.Cos(theta2 + theta3) * Math.Cos(theta1),     -Math.Cos(theta2 + theta3) * Math.Sin(theta1),  -Math.Sin(theta2 + theta3), 400 * Math.Sin(theta2 + theta3) - 25 * Math.Cos(theta2 + theta3) - 560 * Math.Cos(theta3) }, 
                                            { Math.Sin(theta1), Math.Cos(theta1), 0, 0 }, { 0, 0, 0, 1 } };

            double[,] T3t = SF.multiplyMatrix(T30, r);

            if (Math.Abs(T3t[1, 1]) < 1e-6 && Math.Abs(T3t[1, 0]) < 1e-6)
            {
                // Singularity! set angles on last known theta4
                theta4 = thetaLast[3] - Math.Sign(thetaLast[3]) * 10e-5;
                theta5 = 0;
                theta6 = Math.Atan2(-T3t[0, 1], T3t[2, 1]) - thetaLast[3];
            }
            else
            {
                theta4 = Math.Atan2(-T3t[2, 2], -T3t[0, 2]);
                while (Math.Abs(thetaLast[3] - theta4) > 1.0 * Math.PI / 2)
                {
                    theta4 = (thetaLast[3] < theta4) ? theta4 - Math.PI : theta4 + Math.PI;
                }
                theta6 = Math.Atan2(-T3t[1, 1], -T3t[1, 0]);
                while (Math.Abs(thetaLast[5] - theta6) > 1.0 * Math.PI / 2)
                {
                    theta6 = (thetaLast[5] < theta6) ? (theta6 - Math.PI) : (theta6 + Math.PI);
                }
                theta5 = (Math.Abs(Math.Cos(theta4)) > Math.Abs(Math.Sin(theta4))) ? Math.Atan2((-1.0 * T3t[0, 2] / (Math.Cos(theta4))), T3t[1, 2]) : Math.Atan2((-1.0 * T3t[2, 2] / (Math.Sin(theta4))), T3t[1, 2]);
            }
            if (theta4 > (185.0 * Math.PI / 180) || theta4 < (-185.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (theta5 > (129.0 * Math.PI / 180) || theta5 < (-119.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (theta5 > (350.0 * Math.PI / 180) || theta5 < (-350.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            return new double[] { theta1, theta2, theta3, theta4, theta5, theta6 };
        }


        /// <summary>
        /// The uses geometry to calculate the potential solutions to the first three joints. 
        /// The output are the closest values to the last motor angles.
        /// Exceptions are thrown when the outcome is out of the workspace.
        /// all angles are in radians
        /// </summary>
        /// <param name="des"></param>
        /// <param name="EndEffectorTransform"></param> Transform from tool0 to endEffector frame
        /// <param name="lastVal"></param>
        /// <param name="elbow"></param>
        /// <param name="basePos"></param>
        /// <returns></returns>
        double[] IK1to3(Pose des, Pose EndEffectorTransform, Matrix[] T0)
        {

            double[] robotConfig = new double[] { 290, 270, 70, 302, 72 };
            double[][] AxisRange = new double[][] {   new double[] { 165, -165 }, 
                                                    new double[] { 110, -110 }, 
                                                    new double[] { 70, -110 },
                                                    new double[] { 160, -160 }, 
                                                    new double[] { 120, -120 }, 
                                                    new double[] { 400, -400 } };

            double[][] AxisOut;
            Pose tool0Transform = des * EndEffectorTransform.invert;

            double Zmin = -50;
            double wristOffset = Math.Atan2(robotConfig[2], robotConfig[3]);
            double link3Length = Math.Sqrt(robotConfig[2] * robotConfig[2] + robotConfig[3] * robotConfig[3]);

            double theta1a, theta1b, theta1, theta2u, theta2d, theta2, theta3u, theta3d, theta3;

            Vector3 Wrist = tool0Transform * (new Vector3(0, 0, (float)robotConfig[4]));

            if (Wrist.Z < Zmin)
            {
                throw new InverseKinematicsException("Out of workspace, under minZ");
            }

            // Forwards or backwawrds
            theta1a = Math.Atan2(Wrist.Y, Wrist.X);                                     // Forwards facing
            theta1b = (theta1a < 0) ? theta1a - Math.PI : theta1a + Math.PI;            // Backwards facing


            Vector3 Base = new Vector3(0, 0, (float)robotConfig[0]);
            Vector3 LinkBW = Wrist - Base;

            if (Math.Abs(LinkBW.Length() - (robotConfig[1] + link3Length)) < 1e-6)
            {
                throw new InverseKinematicsException("Out of workspace, Beyond arm reach");
            }

            if (Math.Abs(LinkBW.Length() - (robotConfig[1] + link3Length)) < 1e-3)
            {
                theta3 = -Math.PI / 2 + wristOffset;
            }

            // Hat variables are unit variables where xHat is the projected view onto the XY plane
            Vector3 xHat = new Vector3(LinkBW.X, LinkBW.Y, 0);
            xHat.Normalize();

            // angle from horizontal
            double beta = Math.Atan2(LinkBW.Z, Math.Sqrt(LinkBW.X * LinkBW.X + LinkBW.Y * LinkBW.Y));
            // Internal angle of arm configuration, link 2
            double gamma = Math.Acos((1.0 * LinkBW.LengthSquared() + robotConfig[1] * robotConfig[1] - link3Length * link3Length) / (2 * robotConfig[1] * LinkBW.Length()));
            // Internal angle of arm configuration, link 3
            double alpha = Math.Acos((1.0 * link3Length * link3Length + +robotConfig[1] * robotConfig[1] - LinkBW.LengthSquared()) / (2.0 * +robotConfig[1] * link3Length));

            if (double.IsNaN(gamma))
            {
                gamma = 0;
            }
            if (double.IsNaN(alpha))
            {
                alpha = Math.PI;
            }

                theta2u = Math.PI - (beta + gamma);
                theta2d = Math.PI - (beta - gamma);
                theta3u = Math.PI + wristOffset - alpha;
                theta3d = -(Math.PI - alpha - wristOffset);



                if (theta1a < AxisRange[0][1] * Math.PI / 180 || theta1a > AxisRange[0][0] * Math.PI / 180)
                {
                    theta1a = 0;
                }
                if (theta1b < AxisRange[0][1] * Math.PI / 180 || theta1b > AxisRange[0][0] * Math.PI / 180)
                {
                    theta1b = 0;
                }
                if (theta2d < AxisRange[1][1] * Math.PI / 180 || theta2d > AxisRange[1][0] * Math.PI / 180)
                {
                    theta2d = 0;
                }
                if (theta2u < AxisRange[1][1] * Math.PI / 180 || theta2u > AxisRange[1][0] * Math.PI / 180)
                {
                    theta2u = 0;
                }
                if (theta3u < AxisRange[2][1] * Math.PI / 180 || theta3u > AxisRange[2][0] * Math.PI / 180)
                {
                    theta3u = 0;
                }
                if (theta3d < AxisRange[2][1] * Math.PI / 180 || theta` 3d > AxisRange[2][0] * Math.PI / 180)
                {
                    theta3d = 0;
                }

                return new double[] { theta1a, theta1b, theta2u, theta2d, theta3u, theta3d };
        }

        
        /// <summary>
        /// The uses geometry to calculate the potential solutions to the first three joints. 
        /// The output are the closest values to the last motor angles.
        /// Exceptions are thrown when the outcome is out of the workspace.
        /// all angles are in radians
        /// </summary>
        /// <param name="des"></param>
        /// <param name="EndEffectorTransform"></param> Transform from tool0 to endEffector frame
        /// <param name="lastVal"></param>
        /// <param name="elbow"></param>
        /// <param name="basePos"></param>
        /// <returns></returns>
        double[] IK1to3(Pose des, Pose EndEffectorTransform, double[] lastVal, ref ElbowPosition elbow, ref BasePosition basePos)
        {

            double[] robotConfig = new double[] { 290, 270, 70, 302, 72 };
            Pose tool0Transform = des * EndEffectorTransform.invert;

            double Zmin = -50;
            double wristOffset = Math.Atan2(35, 515);
            double theta1a, theta1b, theta1, theta2u, theta2d, theta2, theta3u, theta3d, theta3;

            Vector3 Wrist = tool0Transform * (new Vector3(0, 0, (float)robotConfig[4]));

            if (Wrist.Z < Zmin)    
            {
                throw new InverseKinematicsException("Out of workspace, under minZ");
            }

            // Forwards or backwawrds
            theta1a = Math.Atan2(Wrist.Y, Wrist.X);
            theta1b = (theta1a < 0) ? theta1a - Math.PI : theta1a + Math.PI;
            if (Math.Abs(lastVal[0] - theta1a) < Math.Abs(lastVal[0] - theta1b))
            {
                theta1 = theta1a;
                basePos = BasePosition.front;
            }
            else
            {
                theta1 = theta1b;
                basePos = BasePosition.back;
            }
            if (theta1 < -170.0 * Math.PI / 180 || theta1 > 170.0 * Math.PI / 180)
            {
                throw new InverseKinematicsException("Out of workspace, Axis 1 error");
            }
            Vector3 Base = new Vector3(25 * (float)Math.Cos(-theta1), 25 * (float)Math.Sin(-theta1), 400f);
            Vector3 LinkBW = Wrist - Base;
            if (Math.Abs(LinkBW.Length() - (560 + Math.Sqrt(515 * 515 + 35 * 35))) < 1e-6)
            {
                throw new InverseKinematicsException("Out of workspace, Beyond arm reach");
            }
            if (LinkBW.Length() == (560 + Math.Sqrt(515 * 515 + 35 * 35)))
            {
                theta3 = -Math.Atan2(35, 515);
            }
            Vector3 xHat = new Vector3(LinkBW.X, LinkBW.Y, 0);
            xHat.Normalize();
            double beta = Math.Atan2(LinkBW.Z, Math.Sqrt(LinkBW.X * LinkBW.X + LinkBW.Y * LinkBW.Y));
            double gamma = Math.Acos((1.0 * LinkBW.LengthSquared() + 560 * 560 - 35 * 35 - 515 * 515) / (2 * 560 * LinkBW.Length()));
            double alpha = Math.Acos((1.0 * 560 * 560 + 35 * 35 + 515 * 515 - LinkBW.LengthSquared()) / (2.0 * 560 * Math.Sqrt(35 * 35 + 515 * 515)));
            if (double.IsNaN(gamma))
            {
                gamma = 0;
            }
            if (double.IsNaN(alpha))
            {
                alpha = Math.PI;
            }
            if (basePos == BasePosition.front)
            {
                theta2u = -(beta + gamma);
                theta2d = -(beta - gamma);
                theta3u = Math.PI + wristOffset - alpha;
                theta3d = -(Math.PI - alpha - wristOffset);
            }
            else
            {
                theta2u = -Math.PI + (beta - gamma);
                theta2d = -Math.PI + (beta + gamma);
                theta3u = Math.PI + wristOffset - alpha;
                theta3d = -Math.PI + alpha + wristOffset;
            }
            if (Math.Abs(lastVal[1] - theta2u) < Math.Abs(lastVal[1] - theta2d))
            {
                theta2 = theta2u;
                theta3 = theta3u;
                elbow = ElbowPosition.up;
            }
            else
            {
                theta2 = theta2d;
                theta3 = theta3d;
                elbow = ElbowPosition.down;
            }

            if (elbow == ElbowPosition.up)
            {
                if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    elbow = ElbowPosition.down;
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace, Axis 2 error");
                }
            }
            else if (elbow == ElbowPosition.down)
            {
                if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    elbow = ElbowPosition.up;
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace, Axis 3 Error");
                }
            }
            else
            {

                if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    elbow = ElbowPosition.up;
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    elbow = ElbowPosition.down;
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace, Axis 2/3 Error");
                }
            }
            return new double[] { theta1, theta2, theta3 };
        }


    }
}
