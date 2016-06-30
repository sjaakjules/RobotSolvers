using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Extensions;

namespace RobotSupport
{
    public enum ElbowPosition { up, down, stretched };
    public enum BasePosition { front, back, vertical };

    #region Kinamatic equations, FK/IK and transformation matricies

    public static class solvers
    {
        /// <summary>
        /// The uses geometry to calculate the potential solutions to the first three joints. 
        /// The output are the closest values to the last motor angles.
        /// Exceptions are thrown when the outcome is out of the workspace.
        /// all angles are in radians
        /// </summary>
        /// <param name="des"></param>
        /// <param name="EE"></param>
        /// <param name="lastVal"></param>
        /// <param name="elbow"></param>
        /// <param name="basePos"></param>
        /// <returns></returns>
        double[] IK1to3(Plane des, Vector3d EE, double[] lastVal, ref ElbowPosition elbow, ref BasePosition basePos)
        {
            Transform desiredPose = new Transform();
            Transform.PlaneToPlane(Plane.WorldXY, des);

            double Zmin = -50;
            double wristOffset = Math.Atan2(35, 515);
            double theta1a, theta1b, theta1, theta2u, theta2d, theta2, theta3u, theta3d, theta3;

            Vector3d Wrist = desiredPose * (-EE - new Vector3d(0, 0, 80));
            if (Wrist.Z < Zmin)
            {
                throw new InverseKinematicsException("Out of workspace, under minZ");
            }
            theta1a = -Math.Atan2(Wrist.Y, Wrist.X);
            theta1b = (theta1a > 0) ? theta1a - Math.PI : theta1a + Math.PI;
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
            Vector3d Base = new Vector3d(25 * (float)Math.Cos(-theta1), 25 * (float)Math.Sin(-theta1), 400f);
            Vector3d LinkBW = Wrist - Base;
            if (Math.Abs(LinkBW.Length - (560 + Math.Sqrt(515 * 515 + 35 * 35))) < 1e-6)
            {
                throw new InverseKinematicsException("Out of workspace, Beyond arm reach");
            }
            if (LinkBW.Length == (560 + Math.Sqrt(515 * 515 + 35 * 35)))
            {
                theta3 = -Math.Atan2(35, 515);
            }
            Vector3d xHat = new Vector3d(LinkBW.X, LinkBW.Y, 0);
            xHat.Unitize();
            double beta = Math.Atan2(LinkBW.Z, Math.Sqrt(LinkBW.X * LinkBW.X + LinkBW.Y * LinkBW.Y));
            double gamma = Math.Acos((1.0 * LinkBW.SquareLength + 560 * 560 - 35 * 35 - 515 * 515) / (2 * 560 * LinkBW.Length));
            double alpha = Math.Acos((1.0 * 560 * 560 + 35 * 35 + 515 * 515 - LinkBW.SquareLength) / (2.0 * 560 * Math.Sqrt(35 * 35 + 515 * 515)));
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
        public double[] IKSolver(Plane DesiredPose, Vector3d EE, double[] thetaLast, ref ElbowPosition elbow, ref BasePosition basePos)
        {
            // Rotate the desired pose to alight tool tip with last link.
            // DesiredPose.Orientation = DesiredPose.Orientation * TaskspaceRotation;
            if (double.IsNaN(DesiredPose.OriginX) || double.IsNaN(DesiredPose.OriginY) || double.IsNaN(DesiredPose.OriginZ))
            {
                DesiredPose = forwardKinimatics(thetaLast, EE);
            }
            if (DesiredPose.OriginZ < 0)
            {
                throw new Exception("Below table!");
            }
            double theta1, theta2, theta3, theta4, theta5, theta6;
            double[] angles1to3 = IK1to3(DesiredPose, EE, thetaLast, ref elbow, ref basePos);
            theta1 = angles1to3[0];
            theta2 = angles1to3[1];
            theta3 = angles1to3[2];

            double[,] r = DesiredPose.getMatrix();
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
        /// Computes the inverse jacobian to the WRIST where if singularity occures aproximates the jacobian.
        /// Angles are in radians!!!
        /// </summary>
        /// <param name="t1"></param>
        /// <param name="t2"></param>
        /// <param name="t3"></param>
        /// <param name="t4"></param>
        /// <param name="t5"></param>
        /// <param name="t6"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        static double[,] InverseJacobianWrist(double t1, double t2, double t3, double t4, double t5, double t6, double error)
        {
            double[,] inverseJoc = new double[6, 6];
            double s1 = Math.Sin(t1);
            double c1 = Math.Cos(t1);
            double s2 = Math.Sin(t2);
            double s2p3 = Math.Sin(t2 + t3);
            double c2p3 = Math.Cos(t2 + t3);
            double c2 = Math.Cos(t2);
            double s3 = Math.Sin(t3);
            double c3 = Math.Cos(t3);
            double s3p = Math.Sin(t3 - 1.0 * Math.PI / 2);
            double c3p = Math.Cos(t3 - 1.0 * Math.PI / 2);
            double s4 = Math.Sin(t4);
            double c4 = Math.Cos(t4);
            double s5 = Math.Sin(t5);
            double c5 = Math.Cos(t5);
            double s6 = Math.Sin(t6);
            double c6 = Math.Cos(t6);
            double c234 = Math.Cos(t2 + t3 + t4);
            // These are the denominators withing the inverse Jacobian, if they tend to zero singularity is reached and velocities will tend to inf!
            double cot5 = (Math.Abs(s5) < error) ? 1.0 * c5 / error : 1.0 * c5 / s5;
            double singularity1 = (Math.Abs(103 * c2p3 + 7 * s2p3 + 112 * c2 + 5) < error) ? Math.Sign(103 * c2p3 + 7 * s2p3 + 112 * c2 + 5) * error : (103 * c2p3 + 7 * s2p3 + 112 * c2 + 5);
            double singularity2 = (Math.Abs(1960 * c3 - 28840 * s3) < error) ? Math.Sign(1960 * c3 - 28840 * s3) * error : (1960 * c3 - 28840 * s3);
            double singularity3 = (Math.Abs(7 * c3 - 103 * s3) < error) ? Math.Sign(7 * c3 - 103 * s3) * error : (7 * c3 - 103 * s3);
            double singularity4 = (Math.Abs(3920 * c3 - 57680 * s3) < error) ? Math.Sign(3920 * c3 - 57680 * s3) * error : (3920 * c3 - 57680 * s3);
            double Sing5 = (3605 * c2 * s5 - 175 * c3 * s5 - 53045 * s2 * s5 + 2575 * s3 * s5 + 57680 * c2 * s3 * s5 - 7210 * c2 * c3 * c3 * s5 + 52800 * c3 * c3 * s2 * s5 - 3920 * c2 * c3 * s5 + 52800 * c2 * c3 * s3 * s5 + 7210 * c3 * s2 * s3 * s5);
            double singularity5 = (Math.Abs(Sing5) < error) ? Math.Sign(Sing5) * error : (Sing5);
            double Sing6 = (c2p3 * c2p3 * s5 - c2p3 * c2p3 * c4 * c4 * s5 + s2p3 * c2 * s3 * s5 + s2p3 * c3 * s2 * s5 - s2p3 * c4 * c4 * s2 * s5 * s3p + c2p3 * c2 * c4 * c5 * c3p - c2p3 * c4 * c5 * s2 * s3p + s2p3 * c2 * c4 * c5 * s3p + s2p3 * c4 * c5 * c3p * s2 - c2p3 * c2 * c4 * c4 * s5 * s3p - c2p3 * c4 * c4 * c3p * s2 * s5 + s2p3 * c2 * c4 * c4 * c3p * s5 - s2p3 * c2 * c4 * c4 * s3 * s5 - s2p3 * c3 * c4 * c4 * s2 * s5);
            double singularity6 = (Math.Abs(Sing6) < error) ? Math.Sign(Sing6) * error : (Sing6);
            double singularity7 = (Math.Abs(7 * c3 * s5 - 103 * s3 * s5) < error) ? Math.Sign(7 * c3 * s5 - 103 * s3 * s5) * error : (7 * c3 * s5 - 103 * s3 * s5);
            double singularity8 = ((Math.Abs(s5) < error) ? Math.Sign(s5) * error : s5);
            double Sing9 = (5 * s5 * (721 * c2 - 35 * c3 - 10609 * s2 + 515 * s3 - 784 * c2 * c3 + 11536 * c2 * s3 - 1442 * c2 * c3 * c3 + 10560 * c3 * c3 * s2 + 1442 * c3 * s2 * s3 + 10560 * c2 * c3 * s3));
            double singularity9 = ((Math.Abs(Sing9) < error) ? Math.Sign(Sing9) * error : Sing9);
            double Sing10 = (3605 * c2 - 175 * c3 - 53045 * s2 + 2575 * s3 - 3920 * c2 * c3 + 57680 * c2 * s3 - 7210 * c2 * c3 * c3 + 52800 * c3 * c3 * s2 + 7210 * c3 * s2 * s3 + 52800 * c2 * c3 * s3);
            double singularity10 = ((Math.Abs(Sing10) < error) ? Math.Sign(Sing10) * error : Sing10);

            inverseJoc[0, 0] = -1.0 * s1 / (5 * singularity1);
            inverseJoc[0, 1] = -1.0 * c1 / (5 * singularity1);
            inverseJoc[0, 2] = 0;
            inverseJoc[0, 3] = 0;
            inverseJoc[0, 4] = 0;
            inverseJoc[0, 5] = 0;
            inverseJoc[1, 0] = -1.0 * (103 * c1 * c2 * c3 - 103 * c1 * s2 * s3 + 7 * c1 * c2 * s3 + 7 * c1 * c3 * s2) / (2 * singularity2);
            inverseJoc[1, 1] = 1.0 * (7 * c2 * s1 * s3 + 7 * c3 * s1 * s2 - 103 * s1 * s2 * s3 + 103 * c2 * c3 * s1) / (2 * singularity2);
            inverseJoc[1, 2] = -1.0 * (7 * c2p3 - 103 * s2p3) / (560 * singularity3);
            inverseJoc[1, 3] = 0;
            inverseJoc[1, 4] = 0;
            inverseJoc[1, 5] = 0;
            inverseJoc[2, 0] = 1.0 * (112 * c1 * c2 - 103 * c1 * s2 * s3 + 103 * c1 * c2 * c3 + 7 * c1 * c2 * s3 + 7 * c1 * c3 * s2) / singularity4;
            inverseJoc[2, 1] = -1.0 * (112 * c2 * s1 + 7 * c2 * s1 * s3 + 7 * c3 * s1 * s2 - 103 * s1 * s2 * s3 + 103 * c2 * c3 * s1) / singularity4;
            inverseJoc[2, 2] = -1.0 * (103 * s2p3 - 7 * c2p3 + 112 * s2) / (560 * singularity3);
            inverseJoc[2, 3] = 0;
            inverseJoc[2, 4] = 0;
            inverseJoc[2, 5] = 0;
            inverseJoc[3, 0] = -1.0 * (103 * c2 * s1 * s5 + 112 * c1 * c2 * c2 * c5 * s4 - 103 * c2 * c3 * c3 * s1 * s5 - 7 * c3 * c3 * s1 * s2 * s5 + 5 * c1 * c2 * c5 * s4 + 103 * c4 * c5 * s1 * s2 + 103 * c1 * c2 * c2 * c3 * c5 * s4 + 7 * c2 * c3 * c3 * c4 * c5 * s1 + 7 * c1 * c2 * c2 * c5 * s3 * s4 - 103 * c3 * c3 * c4 * c5 * s1 * s2 - 7 * c2 * c3 * s1 * s3 * s5 + 103 * c3 * s1 * s2 * s3 * s5 + 7 * c1 * c2 * c3 * c5 * s2 * s4 - 103 * c2 * c3 * c4 * c5 * s1 * s3 - 103 * c1 * c2 * c5 * s2 * s3 * s4 - 7 * c3 * c4 * c5 * s1 * s2 * s3) / singularity5;
            inverseJoc[3, 1] = 1.0 * (103 * c1 * c2 * c3 * c3 * s5 - 103 * c1 * c2 * s5 + 7 * c1 * c3 * c3 * s2 * s5 + 112 * c2 * c2 * c5 * s1 * s4 - 103 * c1 * c4 * c5 * s2 + 5 * c2 * c5 * s1 * s4 + 103 * c1 * c3 * c3 * c4 * c5 * s2 + 103 * c2 * c2 * c3 * c5 * s1 * s4 + 7 * c2 * c2 * c5 * s1 * s3 * s4 + 7 * c1 * c2 * c3 * s3 * s5 - 103 * c1 * c3 * s2 * s3 * s5 - 7 * c1 * c2 * c3 * c3 * c4 * c5 + 103 * c1 * c2 * c3 * c4 * c5 * s3 + 7 * c1 * c3 * c4 * c5 * s2 * s3 + 7 * c2 * c3 * c5 * s1 * s2 * s4 - 103 * c2 * c5 * s1 * s2 * s3 * s4) / singularity5;
            inverseJoc[3, 2] = -1.0 * (c5 * s2 * s4) / (5 * singularity7);
            inverseJoc[3, 3] = 1.0 * (c2p3 * c1 * c4 * c4 * s5 - c2p3 * c1 * s5 + c1 * c2 * c4 * c4 * s5 * s3p + c1 * c4 * c4 * c3p * s2 * s5 - c1 * c2 * c4 * c5 * c3p + c1 * c4 * c5 * s2 * s3p + c3 * c5 * s1 * s4 * s3p - c2 * c2 * c3 * c5 * s1 * s4 * s3p - c2 * c2 * c5 * c3p * s1 * s3 * s4 + c2p3 * c2 * c5 * s1 * s4 * s3p + c2p3 * c5 * c3p * s1 * s2 * s4 + c3 * c4 * c3p * s1 * s4 * s5 + c2 * c2 * c4 * s1 * s3 * s4 * s5 * s3p + c2p3 * c2 * c4 * c3p * s1 * s4 * s5 - c2p3 * c4 * s1 * s2 * s4 * s5 * s3p - c2 * c3 * c5 * c3p * s1 * s2 * s4 + c2 * c5 * s1 * s2 * s3 * s4 * s3p - c2 * c2 * c3 * c4 * c3p * s1 * s4 * s5 + c2 * c3 * c4 * s1 * s2 * s4 * s5 * s3p + c2 * c4 * c3p * s1 * s2 * s3 * s4 * s5) / singularity6;
            inverseJoc[3, 4] = 1.0 * (c2p3 * s1 * s5 - c2p3 * c4 * c4 * s1 * s5 - c2 * c4 * c4 * s1 * s5 * s3p - c4 * c4 * c3p * s1 * s2 * s5 + c2 * c4 * c5 * c3p * s1 + c1 * c3 * c5 * s4 * s3p - c4 * c5 * s1 * s2 * s3p + c2p3 * c1 * c2 * c5 * s4 * s3p + c2p3 * c1 * c5 * c3p * s2 * s4 + c1 * c3 * c4 * c3p * s4 * s5 - c1 * c2 * c2 * c3 * c5 * s4 * s3p - c1 * c2 * c2 * c5 * c3p * s3 * s4 + c2p3 * c1 * c2 * c4 * c3p * s4 * s5 - c2p3 * c1 * c4 * s2 * s4 * s5 * s3p - c1 * c2 * c3 * c5 * c3p * s2 * s4 + c1 * c2 * c5 * s2 * s3 * s4 * s3p - c1 * c2 * c2 * c3 * c4 * c3p * s4 * s5 + c1 * c2 * c2 * c4 * s3 * s4 * s5 * s3p + c1 * c2 * c3 * c4 * s2 * s4 * s5 * s3p + c1 * c2 * c4 * c3p * s2 * s3 * s4 * s5) / singularity6;
            inverseJoc[3, 5] = s2p3 - 1.0 * (Math.Cos(t2 + t3 + t4) * cot5) / 2 - 1.0 * (Math.Cos(t2 + t3 - t4) * cot5) / 2;
            inverseJoc[4, 0] = 1.0 * (112 * c1 * c2 * c2 * c4 - 103 * s1 * s2 * s4 + 5 * c1 * c2 * c4 + 103 * c1 * c2 * c2 * c3 * c4 + 7 * c1 * c2 * c2 * c4 * s3 - 7 * c2 * c3 * c3 * s1 * s4 + 103 * c3 * c3 * s1 * s2 * s4 + 7 * c1 * c2 * c3 * c4 * s2 - 103 * c1 * c2 * c4 * s2 * s3 + 103 * c2 * c3 * s1 * s3 * s4 + 7 * c3 * s1 * s2 * s3 * s4) / singularity10;
            inverseJoc[4, 1] = -1.0 * (103 * c1 * s2 * s4 + 112 * c2 * c2 * c4 * s1 + 5 * c2 * c4 * s1 + 7 * c1 * c2 * c3 * c3 * s4 + 103 * c2 * c2 * c3 * c4 * s1 - 103 * c1 * c3 * c3 * s2 * s4 + 7 * c2 * c2 * c4 * s1 * s3 + 7 * c2 * c3 * c4 * s1 * s2 - 103 * c1 * c2 * c3 * s3 * s4 - 103 * c2 * c4 * s1 * s2 * s3 - 7 * c1 * c3 * s2 * s3 * s4) / singularity10;
            inverseJoc[4, 2] = 1.0 * (c4 * s2) / (5 * singularity3);
            inverseJoc[4, 3] = c4 * s1 - c1 * c2 * s3 * s4 - c1 * c3 * s2 * s4;
            inverseJoc[4, 4] = c1 * c4 + c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4;
            inverseJoc[4, 5] = -c2p3 * s4;
            inverseJoc[5, 0] = 1.0 * (103 * c4 * s1 * s2 + 112 * c1 * c2 * c2 * s4 + 5 * c1 * c2 * s4 + 103 * c1 * c2 * c2 * c3 * s4 + 7 * c2 * c3 * c3 * c4 * s1 + 7 * c1 * c2 * c2 * s3 * s4 - 103 * c3 * c3 * c4 * s1 * s2 + 7 * c1 * c2 * c3 * s2 * s4 - 103 * c2 * c3 * c4 * s1 * s3 - 103 * c1 * c2 * s2 * s3 * s4 - 7 * c3 * c4 * s1 * s2 * s3) / singularity9;
            inverseJoc[5, 1] = -1.0 * (5 * c2 * s1 * s4 + 112 * c2 * c2 * s1 * s4 - 103 * c1 * c4 * s2 - 7 * c1 * c2 * c3 * c3 * c4 + 103 * c1 * c3 * c3 * c4 * s2 + 103 * c2 * c2 * c3 * s1 * s4 + 7 * c2 * c2 * s1 * s3 * s4 + 103 * c1 * c2 * c3 * c4 * s3 + 7 * c1 * c3 * c4 * s2 * s3 + 7 * c2 * c3 * s1 * s2 * s4 - 103 * c2 * s1 * s2 * s3 * s4) / singularity9;
            inverseJoc[5, 2] = 1.0 * (s2 * s4) / (5 * singularity7);
            inverseJoc[5, 3] = 1.0 * (s1 * s4 + c1 * c2 * c4 * s3 + c1 * c3 * c4 * s2) / singularity8;
            inverseJoc[5, 4] = -1.0 * (c2 * c4 * s1 * s3 - c1 * s4 + c3 * c4 * s1 * s2) / singularity8;
            inverseJoc[5, 5] = 1.0 * (c2p3 * c4) / singularity8;
            return inverseJoc;
        }


        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in DEGREES, EE is in mm
        /// </summary>
        /// <param name="angles"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        public static Plane forwardKinimatics(double[] angles, Vector3d EE)
        {
            return forwardKinimatics(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], EE);
        }

        public void test()
        {
            Plane[] x = new Plane[2];
            Transform tr = new Transform();
            tr = Transform.PlaneToPlane(x[0], x[1]);
            List<double> output = new List<double>();
            output.Add(tr.M00);
            output.Add(tr.M01);
            output.Add(tr.M02);
            output.Add(tr.M03);

            output.Add(tr.M10);
            output.Add(tr.M11);
            output.Add(tr.M12);
            output.Add(tr.M13);

            output.Add(tr.M20);
            output.Add(tr.M21);
            output.Add(tr.M22);
            output.Add(tr.M23);

            output.Add(tr.M30);
            output.Add(tr.M31);
            output.Add(tr.M32);
            output.Add(tr.M33);
        }

        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in radians EE is in mm
        /// </summary>
        /// <param name="a1"></param>
        /// <param name="a2"></param>
        /// <param name="a3"></param>
        /// <param name="a4"></param>
        /// <param name="a5"></param>
        /// <param name="a6"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        static Plane forwardKinimatics(double a1, double a2, double a3, double a4, double a5, double a6, Vector3d EE)
        {
            // Angles of home position from virtical
            double beta1 = 0;
            double beta2 = 0;

            // Lengths of the different links on DH table
            double d1 = 152.4;
            double d2 = 61.9;
            double d3 = 177.25;
            double d4 = 161.5;
            double d5 = 4.5;
            double d6 = 75;

            double s1 = Math.Sin(a1);
            double c1 = Math.Cos(a1);
            double s2 = Math.Sin(Math.PI/2 + beta1 - a2);
            double c2 = Math.Cos(Math.PI/2 + beta1 - a2);
            double s3b = Math.Sin(beta2 - a3);
            double c3b = Math.Cos(beta2 - a3);
            double s3p = Math.Sin(a3 - 1.0 * Math.PI / 2);
            double c3p = Math.Cos(a3 - 1.0 * Math.PI / 2);
            double s4 = Math.Sin(a4);
            double c4 = Math.Cos(a4);
            double s5 = Math.Sin(a5);
            double c5 = Math.Cos(a5);
            double s5b = Math.Sin(a5 - Math.PI/2);
            double c5b = Math.Cos(a5 - Math.PI/2);
            double s6 = Math.Sin(a6);
            double c6 = Math.Cos(a6);
            double s234 = Math.Sin(Math.PI / 2 + beta1 - beta2 - a2 + a3 - a4);
            double c234 = Math.Cos(Math.PI / 2 + beta1 - beta2 - a2 + a3 - a4);

      

            double f1 = (s3b * c2 * c1 - s2 * c1 * c3b);
            double f2 = (s3b * c2 * c1 - s2 * c1 * c3b);
            double m1 = (c4 * f2 - s4 * f1);
            double m2 = (c4 * f1 + s4 * f2);
            double m3 = (s1 * s5b - c5b * m1);

            double m11 = s6 * m2 - c6 * m3;
            double m12 = s6 * m3 + c6 * m2;
            double m13 = -c5b * s1 - s5b * m1;
            double m14 = d2 * c1 + d5 * s1 - d6 * (c5b * s1 + s5b * m1) + d4 * f2 + d3 * c2 * c1;
            
            f1 = (s1 * s5b - c5b * m1);
            f2 = (s1 * s5b - c5b * m1);
            m1 = (c4 * f1 - s4 * f2);
            m2 = (c1 * s5b + c5b * m1);
            m3 = (c4 * f2 + s4 * f1);

            double m21 = -c6 * m2 - s6 * m3;
            double m22 = s6 * m2 - c6 * m3;
            double m23 = s5b * m1 - c1 * c5b;
            double m24 = d5 * c1 - d4 * f1 - d6 * (c1 * c5b - s5b * m1) - d2 * s1 - d3 * c2 * s1;

            double m31 = (Math.Cos(a5 - Math.PI / 2 + a6) * s234) / 2 + (Math.Cos(a5 - Math.PI / 2 - a6) * s234) / 2 + c234 * s6;
            double m32 = c234 * c6 - (Math.Sin(a5 - Math.PI / 2 + a6) * s234) / 2 + (s234 * Math.Sin(a5 - Math.PI / 2 - a6)) / 2;
            double m33 = Math.Cos(beta1 - beta2 - a2 + a3 - a4) * c5;
            double m34 = d1 + d4 * Math.Sin(Math.PI / 2 + beta1 - beta2 - a2 + a3) + d3 * Math.Cos(beta1 - a2) - d6 * s234 * s5b;

            double m41 = 0;
            double m42 = 0;
            double m43 = 0;
            double m44 = 1;

            Transform tram = makeTransform(m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, m41, m42, m43, m44);
            Plane plan = Plane.WorldXY;
            plan.Transform(tram);
            return plan;
        }

        static public Transform makeTransform(double m11, double m12, double m13, double m14, double m21, double m22, double m23, double m24, double m31, double m32, double m33, double m34, double m41, double m42, double m43, double m44)
        {
            Transform tram = new Transform();
            tram.M00 = m11;
            tram.M01 = m12;
            tram.M02 = m13;
            tram.M03 = m14;

            tram.M10 = m21;
            tram.M11 = m22;
            tram.M12 = m23;
            tram.M13 = m24;

            tram.M20 = m31;
            tram.M21 = m32;
            tram.M22 = m33;
            tram.M23 = m34;

            tram.M30 = m41;
            tram.M31 = m42;
            tram.M32 = m43;
            tram.M33 = m44;

            return tram;
        }


    #endregion
    }
}


    /*
    public struct Pose : IFormattable
    {
        double _x, _y, _z;
        float _angle;
        Quaternion _Orientation;
        Vector3d _axis;
        double[] _kukaValues;

        public static Pose Zero { get { return new Pose(Quaternion.Identity, Vector3d.Zero); } }

        /// <summary>
        /// Loads a new Pose using a string with the infomation of position, velocity and orientation. Can handle partial information.
        /// </summary>
        /// <param name="newPose"></param>
        /// <param name="LastPose"></param>
        public Pose(string[] newPose, Pose LastPose)
        {
            int shift = newPose.Length % 3;
            // The following If statements catch error while trying to load positions, use last position and keep loading.
            // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            if (!double.TryParse(newPose[0 + shift], out _x))
            {
                _x = LastPose._x;
            }
            if (!double.TryParse(newPose[1 + shift], out _y))
            {
                _y = LastPose._y;
            }
            if (!double.TryParse(newPose[2 + shift], out _z))
            {
                _z = LastPose._z;
            }
            double[] Axis = new double[newPose.Length - (3 + shift)];
            bool loadedAxis = true;
            for (int i = 3 + shift; i < newPose.Length; i++)
            {
                if (!double.TryParse(newPose[i], out Axis[i - (3 + shift)]))
                {
                    loadedAxis = false;
                    break;
                }
            }
            if (loadedAxis)
            {
                // only Z axis loaded
                if (Axis.Length == 3)
                {
                    this._Orientation = SF.QfromZaxis(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), LastPose.Orientation);
                    SF.getAxisAngle(_Orientation, out _axis, out _angle);
                    SF.getKukaAngles(_Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
                // Z axis and X axis loaded
                else if (Axis.Length == 6)
                {
                    this._Orientation = SF.QfromXZ(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), new Vector3((float)Axis[3], (float)Axis[4], (float)Axis[5]));
                    SF.getAxisAngle(_Orientation, out _axis, out _angle);
                    SF.getKukaAngles(_Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
                // No rotation loaded, position movement only
                else
                {
                    this._Orientation = LastPose.Orientation;
                    SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                    SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
            }
            // Failed while trying to load rotaition, use last orientation and keep moving.
            // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            else
            {
                this._Orientation = LastPose.Orientation;
                SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                _kukaValues[0] = (float)_x;
                _kukaValues[1] = (float)_y;
                _kukaValues[2] = (float)_z;
            }
        }


        /// <summary>
        /// Constructor used for commands from an external server.
        /// Loads a new Pose using a string with the infomation of position, velocity and orientation. Can handle partial information.
        /// </summary>
        /// <param name="PositionInfo"></param>
        /// <param name="LastPose"></param>
        /// <param name="TaskspaceRotation"></param> this is the constant rotation of the tool tip
        public Pose(string[] PositionInfo, string[] OrientationInfo, Pose LastPose, Quaternion TaskspaceRotation)
        {
            // The following If statements catch error while trying to load positions, use last position and keep loading.
            // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            if (PositionInfo == null || PositionInfo.Length != 3)
            {
                _x = LastPose._x;
                _y = LastPose._y;
                _z = LastPose._z;
            }
            else
            {
                if (!double.TryParse(PositionInfo[0], out _x))
                {
                    _x = LastPose._x;
                }
                if (!double.TryParse(PositionInfo[1], out _y))
                {
                    _y = LastPose._y;
                }
                if (!double.TryParse(PositionInfo[2], out _z))
                {
                    _z = LastPose._z;
                }
            }
            if (OrientationInfo == null)
            {
                this._Orientation = LastPose.Orientation;
                SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                _kukaValues[0] = (float)_x;
                _kukaValues[1] = (float)_y;
                _kukaValues[2] = (float)_z;
            }
            else
            {
                double[] Axis = new double[OrientationInfo.Length];
                bool loadedAxis = true;
                for (int i = 0; i < OrientationInfo.Length; i++)
                {
                    if (!double.TryParse(OrientationInfo[i], out Axis[i]))
                    {
                        loadedAxis = false;
                        break;
                    }
                }
                if (loadedAxis)
                {
                    // only Z axis loaded
                    if (Axis.Length == 3)
                    {
                        this._Orientation = SF.QfromZaxis(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), LastPose.Orientation) * TaskspaceRotation;
                        SF.getAxisAngle(_Orientation, out _axis, out _angle);
                        SF.getKukaAngles(_Orientation, out _kukaValues);
                        _kukaValues[0] = (float)_x;
                        _kukaValues[1] = (float)_y;
                        _kukaValues[2] = (float)_z;
                    }
                    // Z axis and X axis loaded
                    else if (Axis.Length == 6)
                    {
                        this._Orientation = SF.QfromXZ(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), new Vector3((float)Axis[3], (float)Axis[4], (float)Axis[5])) * TaskspaceRotation;
                        SF.getAxisAngle(_Orientation, out _axis, out _angle);
                        SF.getKukaAngles(_Orientation, out _kukaValues);
                        _kukaValues[0] = (float)_x;
                        _kukaValues[1] = (float)_y;
                        _kukaValues[2] = (float)_z;
                    }
                    // No rotation loaded, position movement only
                    else
                    {
                        this._Orientation = LastPose.Orientation;
                        SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                        SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                        _kukaValues[0] = (float)_x;
                        _kukaValues[1] = (float)_y;
                        _kukaValues[2] = (float)_z;
                    }

                }
                // Failed while trying to load rotaition, use last orientation and keep moving.
                // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
                else
                {
                    this._Orientation = LastPose.Orientation;
                    SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                    SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
            }
        }

        public Pose(double[] KukaValues)
        {
            _kukaValues = new double[6];
            KukaValues.CopyTo(_kukaValues, 0);
            this._x = KukaValues[0];
            this._y = KukaValues[1];
            this._z = KukaValues[2];
            _Orientation = SF.MakeQuaternionFromKuka(KukaValues);
            SF.getAxisAngle(_Orientation, out _axis, out _angle);
        }

        public Pose(Pose lastPose, Pose newPose, double elapsedTime)
        {
            if (elapsedTime == 0)
            {
                elapsedTime = 4;
            }
            Vector3d translation = newPose.Translation - lastPose.Translation;
            float angle;
            Vector3d axis;
            SF.getAxisAngle(Quaternion.Inverse(lastPose.Orientation) * newPose.Orientation, out axis, out angle);
            _axis = Vector3.Transform(axis, lastPose.Orientation);
            _angle = (float)(1.0 * angle / elapsedTime);
            _x = translation.X / elapsedTime;
            _y = translation.Y / elapsedTime;
            _z = translation.Z / elapsedTime;
            _Orientation = Quaternion.CreateFromAxisAngle(_axis, _angle);
            SF.getKukaAngles(_Orientation, out _kukaValues);
            _kukaValues[0] = (float)_x;
            _kukaValues[1] = (float)_y;
            _kukaValues[2] = (float)_z;
            for (int i = 0; i < _kukaValues.Length; i++)
            {
                if (double.IsNaN(_kukaValues[i]))
                {
                    _kukaValues[i] = 0;
                }
            }
        }


        public Pose(Matrix Pose) : this(Quaternion.CreateFromRotationMatrix(Pose), Pose.Translation) { }

        public Pose(Quaternion Orientation, Vector3 Position)
        {
            this._x = Position.X;
            this._y = Position.Y;
            this._z = Position.Z;
            this._Orientation = Orientation;
            SF.getAxisAngle(Orientation, out _axis, out _angle);
            SF.getKukaAngles(Orientation, out _kukaValues);
            _kukaValues[0] = (float)_x;
            _kukaValues[1] = (float)_y;
            _kukaValues[2] = (float)_z;
        }

        public Pose(TimeCoordinate Pose) : this(Pose.Orientation, Pose.Translation) { }


        public double[,] getMatrix
        {
            get
            {
                Matrix orientation = Matrix.CreateFromQuaternion(this.Orientation);
                double[,] matrix = new double[,] {  { orientation.M11, orientation.M21, orientation.M31, this.Translation.X }, 
                                                        { orientation.M12, orientation.M22, orientation.M32, this.Translation.Y }, 
                                                        { orientation.M13, orientation.M23, orientation.M33, this.Translation.Z }, { 0, 0, 0, 1 } };
                return matrix;
            }
        }


        public override int GetHashCode()
        {
            return ShiftAndWrap(_x.GetHashCode(), 2) ^ _y.GetHashCode();
        }

        public bool Equals(Pose pose2, double error)
        {
            if (Vector3.Distance(this.Translation, pose2.Translation) <= (error * 4) && SF.isOrientationAligned(this.Orientation, pose2.Orientation, error))
            {
                return true;
            }
            return false;
        }

        private int ShiftAndWrap(int value, int positions)
        {
            positions = positions & 0x1F;

            // Save the existing bit pattern, but interpret it as an unsigned integer.
            uint number = BitConverter.ToUInt32(BitConverter.GetBytes(value), 0);
            // Preserve the bits to be discarded.
            uint wrapped = number >> (32 - positions);
            // Shift and wrap the discarded bits.
            return BitConverter.ToInt32(BitConverter.GetBytes((number << positions) | wrapped), 0);
        }

        public override bool Equals(object o)
        {
            if (Vector3.Distance(this.Translation, ((Pose)o).Translation) == 1e-6 && SF.isOrientationAligned(this.Orientation, ((Pose)o).Orientation, 1e-6))
            {
                return true;
            }
            return false;
        }

        public static bool operator ==(Pose pose1, Pose pose2)
        {
            if (Vector3.Distance(pose1.Translation, pose2.Translation) == 1e-6 && pose1.Orientation.Equals(pose2.Orientation))
            {
                return true;
            }
            return false;
        }

        public static bool operator !=(Pose pose1, Pose pose2)
        {
            if (Vector3.Distance(pose1.Translation, pose2.Translation) == 1e-6 && pose1.Orientation.Equals(pose2.Orientation))
            {
                return false;
            }
            return true;
        }


        public Vector3 Translation
        {
            get { return new Vector3((float)_x, (float)_y, (float)_z); }
            set { _x = value.X; _y = value.Y; _z = value.Z; }
        }

        public Quaternion Orientation
        {
            get { return _Orientation; }
            set
            {
                _Orientation = value;
                SF.getAxisAngle(value, out _axis, out _angle);
            }
        }

        public double[] kukaValues
        {
            get { return _kukaValues; }
        }

        public Vector3 zAxis
        {
            get { return Vector3.Transform(new Vector3(0, 0, 1), _Orientation); }
        }


        public Vector3 yAxis
        {
            get { return Vector3.Transform(new Vector3(0, 1, 0), _Orientation); }
        }

        public Vector3 xAxis
        {
            get { return Vector3.Transform(new Vector3(1, 0, 0), _Orientation); }
        }

        public Vector3 Velocity
        {
            get { return _axis * _angle; }
        }

        public Vector3 axis
        {
            get { return _axis; }
        }

        public float angle
        {
            get { return _angle; }
        }

        public static Pose inverse(Pose pose)
        {
            return new Pose(Quaternion.Inverse(pose.Orientation), -Vector3.Transform(pose.Translation, Quaternion.Inverse(pose.Orientation)));
        }

        public string ToString(string format, IFormatProvider formatProvider)
        {

            if (format == null) format = "G";

            if (formatProvider != null)
            {
                ICustomFormatter formatter = formatProvider.GetFormat(this.GetType()) as ICustomFormatter;
                if (formatter != null)
                    return formatter.Format(format, this, formatProvider);
            }

            switch (format)
            {

                case "PosZ": return string.Format("{0},{1},{2},{3},{4},{5}", this._x, this._y, this._z, this.zAxis.X, this.zAxis.Y, this.zAxis.Z);

                case "data": return string.Format("{0},{1},{2},{3},{4},{5},{6}", this._x, this._y, this._z, this.angle, this.axis.X, this.axis.Y, this.axis.Z);

                case "Display": return string.Format("({0:0.000},{1:0.000},{2:0.000},{3:0.000},{4:0.000},{5:0.000})", this._x, this._y, this._z, this._kukaValues[3], this._kukaValues[4], this._kukaValues[5]);

                case "G":

                default: return String.Format("({0,5},{1,5},{2,5})", this._x, this._y, this._z);

            }

        }


        public override string ToString()
        {
            return ToString("G", null);
        }



        public string ToString(string format)
        {
            return ToString(format, null);
        }

        public string ToString(IFormatProvider formatProvider)
        {
            return ToString(null, formatProvider);
        }


        public Pose invert
        {
            get { return new Pose(Quaternion.Inverse(this.Orientation), -Vector3.Transform(this.Translation, Quaternion.Inverse(this.Orientation))); }
        }

        public static Vector3 operator *(Pose Pose, Vector3 Position)
        {
            Matrix PoseM = Matrix.Transpose(Matrix.CreateFromQuaternion(Pose._Orientation));
            return new Vector3((float)Pose._x + PoseM.M11 * Position.X + PoseM.M12 * Position.Y + PoseM.M13 * Position.Z,
                                (float)Pose._y + PoseM.M21 * Position.X + PoseM.M22 * Position.Y + PoseM.M23 * Position.Z,
                                (float)Pose._z + PoseM.M31 * Position.X + PoseM.M32 * Position.Y + PoseM.M33 * Position.Z);
        }

        public static Pose operator *(Pose Pose1, Pose Pose2)
        {
            return new Pose(Pose1.Orientation * Pose2.Orientation, Pose1 * Pose2.Translation);
        }


    }
     * 
     */
