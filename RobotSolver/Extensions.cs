using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Extensions
{

    #region Personalised Exceptions

    /// <summary>
    /// Exception class for Matrix class, derived from System.Exception
    /// </summary>
    public class MatrixException : Exception
    {
        public MatrixException()
            : base()
        { }

        public MatrixException(string Message)
            : base(Message)
        { }

        public MatrixException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for Kuka related functions, derived from System.Exception
    /// </summary>
    public class KukaException : Exception
    {
        public KukaException()
            : base()
        { }

        public KukaException(string Message)
            : base(Message)
        { }

        public KukaException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class OrientationException : Exception
    {
        public OrientationException()
            : base()
        { }

        public OrientationException(string Message)
            : base(Message)
        { }

        public OrientationException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class TrajectoryException : Exception
    {
        public TrajectoryException()
            : base()
        { }

        public TrajectoryException(string Message)
            : base(Message)
        { }

        public TrajectoryException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }



    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class InverseKinematicsException : Exception
    {
        public InverseKinematicsException()
            : base()
        { }

        public InverseKinematicsException(string Message)
            : base(Message)
        { }

        public InverseKinematicsException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    #endregion



    public static class doubleArrayExtensions
    {
        public static double[] truncate(this double[] array, int decimals)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = Math.Round(array[i], decimals);
            }
            return newArray;
        }

        public static double[] Copy(this double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = array[i];
            }
            return newArray;
        }

        public static double[] getRadian(this double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 1.0 * array[i] * Math.PI / 180;
            }
            return newArray;
        }

        public static double[] getDegree(this double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 180.0 * array[i] / Math.PI;
            }
            return newArray;
        }

        public static bool isClose(this double[] A1, double[] A2)
        {
            if (A1.Length != A2.Length)
                throw new Exception("Array length not same");
            for (int i = 0; i < A1.Length; i++)
            {
                if (Math.Abs(A1[i] - A2[i]) > 1e-1)
                {
                    return false;
                }
            }
            return true;
        }

        static public double[] createArray(this double value, int length)
        {
            double[] arrayout = new double[length];
            for (int i = 0; i < length; i++)
            {
                arrayout[i] = value;
            }
            return arrayout;
        }


        static public double[] add(this double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
                throw new Exception("Double arrays are not same length");
            double[] arrayout = new double[a2.Length];
            for (int i = 0; i < a1.Length; i++)
            {
                arrayout[i] = a1[i] + a2[i];
            }
            return arrayout;
        }


        static public double[] subtract(this double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
                throw new Exception("Double arrays are not same length");
            double[] arrayout = new double[a2.Length];
            for (int i = 0; i < a1.Length; i++)
            {
                arrayout[i] = a1[i] - a2[i];
            }
            return arrayout;
        }

        public static double[] multiply(this double[] M, double value)
        {
            double[] Mout = new double[M.Length];
            for (int i = 0; i < M.Length; i++)
            {
                Mout[i] = M[i] * value;
            }
            return Mout;
        }

        public static double multiply(this double[] V1, double[] V2)
        {
            int m_iRows = V1.Length;
            int m_iCols = V2.Length;
            if (m_iCols != m_iRows)
                throw new Exception("Invalid Matrix specified, not correct size");
            double product = 0;
            for (int i = 0; i < m_iRows; i++)
            {
                product += V1[i] * V2[i];
            }
            return product;
        }


        public static double dotProduct(this double[] V1, double[] V2)
        {
            int m_iRows = V1.Length;
            int m_iCols = V2.Length;
            if (m_iCols != m_iRows)
                throw new Exception("Invalid Matrix specified, not correct size");
            double product = 0;
            for (int i = 0; i < m_iRows; i++)
            {
                product += V1[i] * V2[i];
            }
            return product;
        }

        public static string toStringVector(this double[] Array)
        {
            StringBuilder textOut = new StringBuilder();
            textOut.Append("(");
            for (int i = 0; i < Array.Length; i++)
            {
                textOut.Append((180.0 * Array[i] / Math.PI) + ",");
            }
            textOut.Append(")");
            return textOut.ToString();
        }
    }

    public static class planeExtensions
    {

        public static double[,] getMatrix(this Plane matrix)
        {
            Transform rotation = Transform.PlaneToPlane(Plane.WorldXY, matrix);
            double[,] matOut = new double[4, 4];
            matOut[0, 0] = rotation.M00;
            matOut[0, 1] = rotation.M01;
            matOut[0, 2] = rotation.M02;
            matOut[0, 3] = rotation.M03;

            matOut[1, 0] = rotation.M10;
            matOut[1, 1] = rotation.M11;
            matOut[1, 2] = rotation.M12;
            matOut[1, 3] = rotation.M13;

            matOut[2, 0] = rotation.M20;
            matOut[2, 1] = rotation.M21;
            matOut[2, 2] = rotation.M22;
            matOut[2, 3] = rotation.M23;

            matOut[3, 0] = rotation.M30;
            matOut[3, 1] = rotation.M31;
            matOut[3, 2] = rotation.M32;
            matOut[3, 3] = rotation.M33;
            return matOut;
        }
    }

    public static class doubleMatrixExtensions
    {

        public static double[,] NullMatrix(double[,] mat, int row, int col)
        {
            double[,] result = new double[row, col];
            for (int i = 0; i < row; i++)
            {
                for (int j = 0; j < col; j++)
                {
                    result[i, j] = 0;
                }
            }
            return result;
        }

        /// <summary>
        /// Internal Fucntions for the above operators
        /// </summary>
        public static double[,] Negate(this double[,] mat)
        {
            return mat.Multiply(-1.0);
        }

        public static double[,] Add(this double[,] matrix1, double[,] matrix2)
        {
            if (matrix1.GetLength(0) != matrix2.GetLength(0) || matrix1.GetLength(1) != matrix2.GetLength(1))
                throw new Exception("Invalid Matrix specified, not correct size");
            double[,] result = new double[matrix1.GetLength(0), matrix1.GetLength(1)];
            for (int i = 0; i < result.GetLength(0); i++)
                for (int j = 0; j < result.GetLength(1); j++)
                    result[i, j] = matrix1[i, j] + matrix2[i, j];
            return result;
        }

        public static double[,] Multiply(this double[,] matrix1, double[,] matrix2)
        {
            if (matrix1.GetLength(1) != matrix2.GetLength(0))
                throw new Exception("Invalid Matrix specified, not correct size");
            double[,] result = new double[matrix1.GetLength(0), matrix2.GetLength(1)];
            for (int i = 0; i < result.GetLength(0); i++)
                for (int j = 0; j < result.GetLength(1); j++)
                    for (int k = 0; k < matrix1.GetLength(1); k++)
                        result[i, j] += matrix1[i, k] * matrix2[k, j];
            return result;
        }


        public static double[] Multiply(this double[,] matrix1, double[] colMatrix)
        {
            int m_iRows = matrix1.GetLength(0);
            int m_iCols = matrix1.GetLength(1);
            if (m_iCols != colMatrix.Length)
                throw new Exception("Invalid Matrix specified, not correct size");
            double[] result = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < m_iCols; j++)
                {
                    result[i] += matrix1[i, j] * colMatrix[j];
                }
            }
            return result;
        }

        public static double[,] Multiply(this double[,] matrix, int iNo)
        {
            double[,] result = new double[matrix.GetLength(0), matrix.GetLength(1)];
            for (int i = 0; i < matrix.GetLength(0); i++)
                for (int j = 0; j < matrix.GetLength(1); j++)
                    result[i, j] = matrix[i, j] * iNo;
            return result;
        }

        public static double[,] Multiply(this double[,] matrix, double frac)
        {
            double[,] result = new double[matrix.GetLength(0), matrix.GetLength(1)];
            for (int i = 0; i < matrix.GetLength(0); i++)
                for (int j = 0; j < matrix.GetLength(1); j++)
                    result[i, j] = matrix[i, j] * frac;
            return result;
        }



        /// <summary>
        /// The function return the Minor of element[Row,Col] of a Matrix object 
        /// </summary>
        public static double[,] Minor(this double[,] matrix, int iRow, int iCol)
        {
            double[,] minor = new double[matrix.GetLength(0) - 1, matrix.GetLength(1) - 1];
            int m = 0, n = 0;
            for (int i = 0; i < matrix.GetLength(0); i++)
            {
                if (i == iRow)
                    continue;
                n = 0;
                for (int j = 0; j < matrix.GetLength(1); j++)
                {
                    if (j == iCol)
                        continue;
                    minor[m, n] = matrix[i, j];
                    n++;
                }
                m++;
            }
            return minor;
        }

        /// <summary>
        /// The helper function for the above Determinent() method
        /// it calls itself recursively and computes determinent using minors
        /// </summary>
        public static double Determinent(this double[,] matrix)
        {
            double det = 0;
            if (matrix.GetLength(0) != matrix.GetLength(1))
                throw new Exception("Determinent of a non-square matrix doesn't exist");
            if (matrix.GetLength(0) == 1)
                return matrix[0, 0];
            for (int j = 0; j < matrix.GetLength(1); j++)
                det += (matrix[0, j] * Determinent(matrix.Minor(0, j)) * (int)System.Math.Pow(-1, 0 + j));
            return det;
        }


        /// <summary>
        /// The function returns the inverse of the current matrix in the traditional way(by adjoint method)
        /// It can be much slower if the given matrix has order greater than 6
        /// Try using InverseFast() function if the order of matrix is greater than 6
        /// </summary>
        public static double[,] Inverse(this double[,] matrix)
        {
            if (matrix.Determinent() == 0)
                throw new Exception("Inverse of a singular matrix is not possible");
            return (matrix.Adjoint().Multiply(1.0 / matrix.Determinent()));
        }


        public static double[,] PsuInverse(this double[,] matrix)
        {
            return matrix.Transpose().Multiply(matrix.Multiply(matrix.Transpose()).Inverse(1e3));
        }



        /// <summary>
        /// The function returns the inverse of the current matrix in the traditional way(by adjoint method)
        /// It can be much slower if the given matrix has order greater than 6
        /// Try using InverseFast() function if the order of matrix is greater than 6
        /// </summary>
        public static double[,] Inverse(this double[,] matrix, double error)
        {
            double det = matrix.Determinent();
            if (det < error)
            {
                return matrix.Adjoint().Multiply(1.0 / error);
            }
            return matrix.Adjoint().Multiply(1.0 / det);
        }

        /// <summary>
        /// The function returns the adjoint of the current matrix
        /// </summary>
        public static double[,] Adjoint(this double[,] matrix)
        {
            if (matrix.GetLength(0) != matrix.GetLength(1))
                throw new Exception("Adjoint of a non-square matrix does not exists");
            double[,] AdjointMatrix = new double[matrix.GetLength(0), matrix.GetLength(1)];
            for (int i = 0; i < matrix.GetLength(0); i++)
                for (int j = 0; j < matrix.GetLength(1); j++)
                    AdjointMatrix[i, j] = Math.Pow(-1, i + j) * (Minor(matrix, i, j).Determinent());
            AdjointMatrix = AdjointMatrix.Transpose();
            return AdjointMatrix;
        }

        /// <summary>
        /// The function returns the transpose of the current matrix
        /// </summary>
        public static double[,] Transpose(this double[,] matrix)
        {
            double[,] TransposeMatrix = new double[matrix.GetLength(1), matrix.GetLength(0)];
            for (int i = 0; i < TransposeMatrix.GetLength(0); i++)
                for (int j = 0; j < TransposeMatrix.GetLength(1); j++)
                    TransposeMatrix[i, j] = matrix[j, i];
            return TransposeMatrix;
        }

        public static double[] row(this double[,] M, int row)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iRows < row)
                throw new Exception("Invalid Matrix specified, not correct size");
            double[] Rout = new double[m_iCols];
            for (int i = 0; i < m_iCols; i++)
            {
                Rout[i] = M[row, i];
            }
            return Rout;
        }

        public static double[] column(this double[,] M, int col)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols < col)
                throw new Exception("Invalid Matrix specified, not correct size");
            double[] Cout = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                Cout[i] = M[i, col];
            }
            return Cout;
        }
    }

    public static class MatrixExtensions
    {
        public static string ToDataString(this Vector3d vec)
        {
            return string.Format("{0},{1},{2}", vec.X, vec.Y, vec.Z);
        }

        public static Vector3d getAxis(this Transform rotation, string column)
        {
            if (column.Equals("X", StringComparison.OrdinalIgnoreCase))
            {
                return new Vector3d(rotation.M11, rotation.M12, rotation.M13);
            }
            else if (column.Equals("Y", StringComparison.OrdinalIgnoreCase))
            {
                return new Vector3d(rotation.M21, rotation.M22, rotation.M23);
            }
            else if (column.Equals("Z", StringComparison.OrdinalIgnoreCase))
            {
                return new Vector3d(rotation.M31, rotation.M32, rotation.M33);
            }
            else
                throw new Exception("Matrix string did not match X, Y, Z");
        }


        ////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////// Update//////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////
        /*
        public static Matrix multiply(this Transform mat1, Transform mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

        public static Matrix multiply(this Matrix Rz, Matrix Ry, Matrix Rx)
        {
            return (Rz.multiply(Ry)).multiply(Rx);
        }
         * 
         

    }

    public static class VectorExtensions
    {

        public static Vector3d getOrientationError(this Vector3d error, Transform reference, Transform measured)
        {
            return Vector3d.Multiply((Vector3d.CrossProduct(measured.getAxis("x"), reference.getAxis("x")) + Vector3d.CrossProduct(measured.getAxis("y"), reference.getAxis("y")) + Vector3d.CrossProduct(measured.getAxis("z"), reference.getAxis("z"))), 0.5f);
        }


        public static Vector3d getOrientationError(this Vector3d error, Quaternion reference, Quaternion measured)
        {
            Vector3d changeAxis = Vector3d.Zero;
            float changeAngle = 0;
            Quaternion changeOrientation = measured.Inverse * reference;
            changeOrientation.toAxisAngle(out changeAxis, out changeAngle);
            return changeAxis.Transform( .Transform(changeAxis, measured) * changeAngle;
        }
    }

    public static class QuaternionExtensions
    {

        public static Quaternion createFromXaxisZaxis(this Quaternion Q, Vector3d xAxis, Vector3d zAxis)
        {
            xAxis.Normalize();
            zAxis.Normalize();
            Vector3 yAxis = Vector3.Cross(zAxis, xAxis);
            yAxis.Normalize();
            return Quaternion.CreateFromRotationMatrix(new Matrix(xAxis.X, xAxis.Y, xAxis.Z, 0, yAxis.X, yAxis.Y, yAxis.Z, 0, zAxis.X, zAxis.Y, zAxis.Z, 0, 0, 0, 0, 1));
        }

        public static Quaternion createFromZaxis(this Quaternion Q, Vector3 Zaxis, Quaternion _currentOrientation)
        {
            Matrix _currentPose = Matrix.CreateFromQuaternion(_currentOrientation);
            Vector3 axis = Vector3.Cross(Vector3.Normalize(_currentPose.Backward), Vector3.Normalize(Zaxis));
            float angle = (float)Math.Asin((double)axis.Length());
            if (Math.Abs(angle) < MathHelper.ToRadians(0.2f))
            {
                return _currentOrientation;
            }
            if (Vector3.Transform(Zaxis, Quaternion.Inverse(_currentOrientation)).Z < 0)
            {
                angle = Math.Sign(angle) * ((float)Math.PI - Math.Abs(angle));
            }
            return Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), angle) * _currentOrientation;
        }

        public static bool isOrientationAligned(this Quaternion Q1, Quaternion Q2, double error)
        {
            float angle;
            Vector3 axis;
            Quaternion change = Quaternion.Inverse(Q1) * Q2;
            change.toAxisAngle(out axis, out angle);
            if (Math.Abs(angle) <= error)
            {
                return true;
            }
            return false;
        }


        public static void toAxisAngle(this Quaternion quaternion, out Vector3 outAxis, out float outAngle)
        {
            quaternion.Normalize();
            if (quaternion.W < 0)
            {
                quaternion = Quaternion.Negate(quaternion);
            }
            float angle = 2 * (float)Math.Acos(quaternion.W);
            float s = (float)Math.Sin(1.0 * angle / 2);
            if (Math.Abs(s) < 1e-6)
            {
                outAxis.X = quaternion.X;
                outAxis.Y = quaternion.Y;
                outAxis.Z = quaternion.Z;
                if (Single.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                    outAxis = new Vector3(0, 0, 1);
                }

            }
            else
            {
                Vector3 axis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
                axis.Normalize();
                s = quaternion.X == 0 ? s : 1.0f * quaternion.X / axis.X;
                s = quaternion.Y == 0 ? s : 1.0f * quaternion.Y / axis.Y;
                s = quaternion.Z == 0 ? s : 1.0f * quaternion.Z / axis.Z;

                if (float.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
                outAxis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                }
            }
        }


    }

    public static class MiscExtensions
    {
        public static Vector3[] TranslationData(this Pose[] PoseArray)
        {
            return PoseArray.Select(p => p.Translation).ToArray();
        }

        public static double[] getLatest(this FilterButterworth[] filterValues)
        {
            double[] filtValues = new double[filterValues.Length];
            for (int i = 0; i < filterValues.Length; i++)
            {
                filtValues[i] = filterValues[i].Value;
            }
            return filtValues;
        }

        public static void updateValues(this FilterButterworth[] doubleFilter, double[] newValues)
        {
            if (newValues.Length == doubleFilter.Length)
            {
                for (int i = 0; i < doubleFilter.Length; i++)
                {
                    doubleFilter[i].Update((float)newValues[i]);
                }
            }
            else
            {
                throw new Exception("Filter length not equal to value array.");
            }
        }
         * 
         */
    }

    public static class SF
    {
        #region Matrix Functions

        public static void plotDoubles(double[] array, string Heading)
        {
            if (Heading.Length < 24)
            {
                string newheading = string.Concat(Heading, ":", new string(' ', 24 - Heading.Length));
            }
            Console.Write(String.Format("{0,-25}\t (", Heading));
            for (int i = 0; i < array.Length; i++)
            {
                Console.Write(String.Format(" {0:0.0} ,", array[i]));
            }
            Console.WriteLine(")");
        }

        public static string DoublesToString(double[] array)
        {
            StringBuilder strOut = new StringBuilder();
            strOut.Append("(");
            for (int i = 0; i < array.Length; i++)
            {
                strOut.Append(String.Format(" {0:0.0} ,", array[i]));
            }
            strOut.AppendLine(")");
            return strOut.ToString();
        }

        public static double[] getRadian(double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 1.0 * array[i] * Math.PI / 180;
            }
            return newArray;
        }

        public static double[] getDegree(double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 180.0 * array[i] / Math.PI;
            }
            return newArray;
        }

        public static bool IsClose(double[] A1, double[] A2)
        {
            for (int i = 0; i < A1.Length; i++)
            {
                if (Math.Abs(A1[i] - A2[i]) > 1e-1)
                {
                    return false;
                }
            }
            return true;
        }

        static public double[] createDouble(double value, int length)
        {
            double[] arrayout = new double[length];
            for (int i = 0; i < length; i++)
            {
                arrayout[i] = value;
            }
            return arrayout;
        }

        static public void addtoDoubles(double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
            {
                throw new MatrixException("Double arrays are not same length");
            }
            for (int i = 0; i < a1.Length; i++)
            {
                a1[i] += a2[i];
            }
        }

        static public double[] addDoubles(double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
            {
                throw new MatrixException("Double arrays are not same length");
            }
            double[] arrayout = new double[a2.Length];
            for (int i = 0; i < a1.Length; i++)
            {
                arrayout[i] = a1[i] + a2[i];
            }
            return arrayout;
        }

        public static double[] multiplyMatrix(double[] M, double value)
        {
            double[] Mout = new double[M.Length];
            for (int i = 0; i < M.Length; i++)
            {
                Mout[i] = M[i] * value;
            }
            return Mout;
        }

        public static double[] multiplyMatrix(double[,] M, double[] value)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols != value.Length)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Mout = new double[value.Length];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < m_iCols; j++)
                {
                    Mout[i] += M[i, j] * value[j];
                }
            }
            return Mout;
        }

        public static double[,] multiplyMatrix(double[,] M, double[,] value)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            int v_iRows = value.GetLength(0);
            int v_iCols = value.GetLength(1);
            if (m_iCols != v_iRows)
            {
                return multiplyMatrix(value, M);
                throw new MatrixException("Invalid Matrix specified, not correct size");
            }
            double[,] Mout = new double[m_iRows, v_iCols];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < v_iCols; j++)
                {
                    Mout[i, j] = dotProduct(getRow(M, i), getCol(value, j));
                }
            }
            return Mout;
        }

        public static double[] getRow(double[,] M, int row)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iRows < row)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Rout = new double[m_iCols];
            for (int i = 0; i < m_iCols; i++)
            {
                Rout[i] = M[row, i];
            }
            return Rout;
        }

        public static double[] getCol(double[,] M, int col)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols < col)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Cout = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                Cout[i] = M[i, col];
            }
            return Cout;
        }

        public static double dotProduct(double[] V1, double[] V2)
        {
            int m_iRows = V1.Length;
            int m_iCols = V2.Length;
            if (m_iCols != m_iRows)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double product = 0;
            for (int i = 0; i < m_iRows; i++)
            {
                product += V1[i] * V2[i];
            }
            return product;
        }

        #endregion



        public static double[] getAverage(double[][] arraylist)
        {
            int n = arraylist.Length;
            if (n < 1)
            {
                return new double[] { 0, 0, 0, 0, 0, 0 };
            }
            else
            {
                double[] average = new double[] { 0, 0, 0, 0, 0, 0 };
                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        average[j] += arraylist[i][j];
                    }
                }
                for (int i = 0; i < 6; i++)
                {
                    average[i] = 1.0 * average[i] / n;
                }
                return average;
            }
        }

    }


}
