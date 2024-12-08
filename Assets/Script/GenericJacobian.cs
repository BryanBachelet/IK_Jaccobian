using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using IK.Tools;
using System;
using System.IO;

public class GenericJacobian : MonoBehaviour
{
    public static JacobianMatrix CreateGenericJacobian(int jointCount, Transform[] joints)
    {
        JacobianMatrix jacobianMatrix = new JacobianMatrix(6, 6 * jointCount);

        // Create Adjoint Block
        for (int i = 0; i < jointCount; i++)
        {
            AdjointBlock adjointBlock = new AdjointBlock();
            Matrix4x4 rotationJoint = Matrix4x4.Rotate(joints[i].transform.localRotation);
            // Add the the two rotation bock
            adjointBlock.AddBlock(rotationJoint, 3, 3, 0, 0);
            adjointBlock.AddBlock(rotationJoint, 3, 3, 3, 3);

            // Calcul the cross rotation block and add it to the adjointBlock
            Matrix4x4 crossTranslation = Matrix4x4.zero;
            crossTranslation[0, 1] = -joints[i].transform.localPosition.z;
            crossTranslation[0, 2] = joints[i].transform.localPosition.y;
            crossTranslation[1, 2] = -joints[i].transform.localPosition.x;

            crossTranslation[1, 0] = joints[i].transform.localPosition.z;
            crossTranslation[2, 0] = -joints[i].transform.localPosition.y;
            crossTranslation[2, 1] = joints[i].transform.localPosition.x;

            Matrix4x4 crossRotation = crossTranslation * rotationJoint;
            adjointBlock.AddBlock(crossRotation, 3, 3, 0, 3);

            //adjointBlock.DebugMatrix();
            // Add the adjoin block to the jaccobian matrix
            jacobianMatrix.AddData(adjointBlock.values, adjointBlock.rows, adjointBlock.column, i * adjointBlock.column);
        }
        double[][] Ai = MatrixInverseSVD.MatrixInverseSVDProgram.MatInverseSVD(jacobianMatrix.ConvertMatInArray());

        jacobianMatrix.AddData(Ai, jacobianMatrix.column, jacobianMatrix.rows, 0, 0);
        // jacobianMatrix.Transpose();
        return jacobianMatrix;
    }

    /// <summary>
    /// Calcul the delta values of orientation and position
    /// </summary>
    /// <param name="linearError"></param>
    /// <param name="angularError"></param>
    /// <param name="jacobianMatrix"></param>
    /// <returns></returns>
    public static double[] GetDeltaValues(Vector3 linearError, Vector3 angularError, JacobianMatrix jacobianMatrix)
    {
        Vector6 velocity = new Vector6(linearError, angularError);

        return jacobianMatrix * velocity;
    }
}



namespace MatrixInverseSVD
{

    /// <summary>
    /// 
    /// All the code below has been get from James MacCaffey's article :  https://visualstudiomagazine.com/Articles/2024/02/01/matrix-inverse-ml-tutorial.aspx?Page=1
    ///
    /// </summary>
    public class MatrixInverseSVDProgram
    {
        //static void Main(string[] args)
        //{
        //    Console.WriteLine("\nMatrix inverse using SVD ");

        //    double[][] A = new double[4][];
        //    A[0] = new double[] { 4.0, 7.0, 1.0, 2.0 };
        //    A[1] = new double[] { 6.0, 0.0, 3.0, 5.0 };
        //    A[2] = new double[] { 8.0, 1.0, 9.0, 2.0 };
        //    A[3] = new double[] { 2.0, 5.0, 6.0, -3.0 };

        //    Console.WriteLine("\nSource matrix A: ");
        //    MatShow(A, 2, 6);

        //    double[][] Ai = MatInverseSVD(A);
        //    Console.WriteLine("\nInverse matrix Ai: ");
        //    MatShow(Ai, 4, 9);

        //    double[][] AiA = MatProd(Ai, A);
        //    Console.WriteLine("\nAi * A: ");
        //    MatShow(AiA, 4, 9);

        //    Console.WriteLine("\nEnd demo ");
        //    Console.ReadLine();
        //} // Main

        static double[][] MatProd(double[][] matA,
            double[][] matB)
        {
            int aRows = matA.Length;
            int aCols = matA[0].Length;
            int bRows = matB.Length;
            int bCols = matB[0].Length;
            if (aCols != bRows)
                throw new Exception("Non-conformable matrices");

            //double[][] result = MatMake(aRows, bCols);
            double[][] result = new double[aRows][];
            for (int i = 0; i < aRows; ++i)
                result[i] = new double[bCols];

            for (int i = 0; i < aRows; ++i) // each row of A
                for (int j = 0; j < bCols; ++j) // each col of B
                    for (int k = 0; k < aCols; ++k)
                        result[i][j] += matA[i][k] * matB[k][j];

            return result;
        }

        // ======================================================

        public static double[][] MatInverseSVD(double[][] M)
        {
            // SVD Jacobi algorithm
            // A = U * S * Vh
            // inv(A) = tr(Vh) * inv(S) * tr(U)
            double[][] U;
            double[][] Vh;
            double[] s;
            MatDecomposeSVD(M, out U, out Vh, out s);

            // TODO: check if determinant is zero(no inverse)
            double absDet = 1.0;
            for (int i = 0; i < M.Length; ++i)
                absDet *= s[i];
            Console.WriteLine(absDet); Console.ReadLine();

            // convert s vector to Sinv matrix
            double[][] Sinv = MatInvFromVec(s);

            double[][] V = MatTranspose(Vh);
            double[][] Utrans = MatTranspose(U);
            double[][] resultTemp = MatProduct(Sinv, Utrans);
            double[][] result = MatProduct(V, resultTemp);
            return result;

            // ----------------------------------------------------
            // 11 helpers: MatDecomposeSVD, MatMake, MatCopy,
            // MatIdentity, MatGetColumn, MatTranspose,
            // MatInvFromVec, MatProduct, VecNorm, VecDot, Hypot
            // ----------------------------------------------------

            static void MatDecomposeSVD(double[][] mat,
              out double[][] U, out double[][] Vh,
              out double[] s)
            {
                // assumes source matrix is square
                double EPSILON = 1.0e-15;

                double[][] A = MatCopy(mat); // working U
                int m = A.Length;
                int n = A[0].Length;  // check m == n
                double[][] Q = MatIdentity(n); // working V
                double[] t = new double[n];  // working s

                int ct = 1;  // rotation counter
                int pass = 0;
                double tol = 10 * n * EPSILON; // heuristic

                int passMax = 5 * n;
                if (passMax < 15) passMax = 15; // heuristic

                // save the column error estimates
                for (int j = 0; j < n; ++j)
                {
                    double[] cj = MatGetColumn(A, j);
                    double sj = VecNorm(cj);
                    t[j] = EPSILON * sj;
                }

                while (ct > 0 && pass <= passMax)
                {
                    ct = n * (n - 1) / 2;  // rotation counter
                    for (int j = 0; j < n - 1; ++j)
                    {
                        for (int k = j + 1; k < n; ++k)
                        {
                            double sin; double cos;

                            double[] cj = MatGetColumn(A, j);
                            double[] ck = MatGetColumn(A, k);

                            double p = 2.0 * VecDot(cj, ck);
                            double a = VecNorm(cj);
                            double b = VecNorm(ck);

                            double q = a * a - b * b;
                            double v = Hypot(p, q);

                            double errorA = t[j];
                            double errorB = t[k];

                            bool sorted = false;
                            if (a >= b) sorted = true;

                            bool orthog = false;
                            if (Math.Abs(p) <=
                              tol * (a * b)) orthog = true;

                            bool badA = false;
                            if (a < errorA) badA = true;
                            bool badB = false;
                            if (b < errorB) badB = true;

                            if (sorted == true && (orthog == true ||
                              badA == true || badB == true))
                            {
                                --ct;
                                continue;
                            }

                            // compute rotation angles
                            if (v == 0 || sorted == false)
                            {
                                cos = 0.0;
                                sin = 1.0;
                            }
                            else
                            {
                                cos = Math.Sqrt((v + q) / (2.0 * v));
                                sin = p / (2.0 * v * cos);
                            }

                            // apply rotation to A (U)
                            for (int i = 0; i < m; ++i)
                            {
                                double Aik = A[i][k];
                                double Aij = A[i][j];
                                A[i][j] = Aij * cos + Aik * sin;
                                A[i][k] = -Aij * sin + Aik * cos;
                            }

                            // update singular values
                            t[j] = Math.Abs(cos) * errorA +
                              Math.Abs(sin) * errorB;
                            t[k] = Math.Abs(sin) * errorA +
                              Math.Abs(cos) * errorB;

                            // apply rotation to Q (V)
                            for (int i = 0; i < n; ++i)
                            {
                                double Qij = Q[i][j];
                                double Qik = Q[i][k];
                                Q[i][j] = Qij * cos + Qik * sin;
                                Q[i][k] = -Qij * sin + Qik * cos;
                            } // i
                        } // k
                    } // j

                    ++pass;
                } // while

                //  compute singular values
                double prevNorm = -1.0;
                for (int j = 0; j < n; ++j)
                {
                    double[] column = MatGetColumn(A, j);
                    double norm = VecNorm(column);

                    // check if singular value is zero
                    if (norm == 0.0 || prevNorm == 0.0
                      || (j > 0 && norm <= tol * prevNorm))
                    {
                        t[j] = 0.0;
                        for (int i = 0; i < m; ++i)
                            A[i][j] = 0.0;
                        prevNorm = 0.0;
                    }
                    else
                    {
                        t[j] = norm;
                        for (int i = 0; i < m; ++i)
                            A[i][j] = A[i][j] * 1.0 / norm;
                        prevNorm = norm;
                    }
                }

                if (ct > 0)
                {
                    Console.WriteLine("Jacobi iterations did not" +
                      " converge");
                }

                U = A;
                Vh = MatTranspose(Q);
                s = t;

                // to sync with default np.linalg.svd() shapes:
                // if m < n, extract 1st m columns of U
                //   extract 1st m values of s
                //   extract 1st m rows of Vh
                // not applicable for matrix inverse.

                // if (m < n)
                // {
                //   U = MatExtractFirstColumns(U, m);
                //   s = VecExtractFirst(s, m);
                //   Vh = MatExtractFirstRows(Vh, m);
                // }

            } // MatDecomposeSVD

            static double[][] MatMake(int r, int c)
            {
                double[][] result = new double[r][];
                for (int i = 0; i < r; ++i)
                    result[i] = new double[c];
                return result;
            }

            static double[][] MatCopy(double[][] m)
            {
                int r = m.Length; int c = m[0].Length;
                double[][] result = MatMake(r, c);
                for (int i = 0; i < r; ++i)
                    for (int j = 0; j < c; ++j)
                        result[i][j] = m[i][j];
                return result;
            }

            static double[][] MatIdentity(int n)
            {
                double[][] result = MatMake(n, n);
                for (int i = 0; i < n; ++i)
                    result[i][i] = 1.0;
                return result;
            }

            static double[] MatGetColumn(double[][] m, int j)
            {
                int rows = m.Length;
                double[] result = new double[rows];
                for (int i = 0; i < rows; ++i)
                    result[i] = m[i][j];
                return result;
            }

            static double[][] MatTranspose(double[][] m)
            {
                int r = m.Length;
                int c = m[0].Length;
                double[][] result = MatMake(c, r);
                for (int i = 0; i < r; ++i)
                    for (int j = 0; j < c; ++j)
                        result[j][i] = m[i][j];
                return result;
            }

            static double[][] MatInvFromVec(double[] s)
            {
                // Sinv from s
                int n = s.Length;
                double[][] result = MatMake(n, n);
                for (int i = 0; i < n; ++i)
                {
                    if (s[i] != 0)
                        result[i][i] = 1.0 / s[i];
                    else
                        result[i][i] = 0;
                }

                return result;
            }

            static double[][] MatProduct(double[][] matA,
              double[][] matB)
            {
                int aRows = matA.Length;
                int aCols = matA[0].Length;
                int bRows = matB.Length;
                int bCols = matB[0].Length;
                if (aCols != bRows)
                    throw new Exception("Non-conformable matrices");

                double[][] result = MatMake(aRows, bCols);

                for (int i = 0; i < aRows; ++i)
                    for (int j = 0; j < bCols; ++j)
                        for (int k = 0; k < aCols; ++k)
                            result[i][j] += matA[i][k] * matB[k][j];

                return result;
            }

            static double VecNorm(double[] vec)
            {
                double sum = 0.0;
                int n = vec.Length;
                for (int i = 0; i < n; ++i)
                    sum += vec[i] * vec[i];
                return Math.Sqrt(sum);
            }

            static double VecDot(double[] v1, double[] v2)
            {
                int n = v1.Length;
                double sum = 0.0;
                for (int i = 0; i < n; ++i)
                    sum += v1[i] * v2[i];
                return sum;
            }

            static double Hypot(double x, double y)
            {
                // fancy sqrt(x^2 + y^2)
                double xabs = Math.Abs(x);
                double yabs = Math.Abs(y);
                double min, max;

                if (xabs < yabs)
                {
                    min = xabs; max = yabs;
                }
                else
                {
                    min = yabs; max = xabs;
                }

                if (min == 0)
                    return max;

                double u = min / max;
                return max * Math.Sqrt(1 + u * u);
            }

        } // MatInverseSVD

        // ======================================================

        static void MatShow(double[][] M, int dec, int wid)
        {
            for (int i = 0; i < M.Length; ++i)
            {
                for (int j = 0; j < M[0].Length; ++j)
                {
                    double v = M[i][j];
                    Console.Write(v.ToString("F" + dec).
                      PadLeft(wid));
                }
                Console.WriteLine("");
            }
        }

        static double[][] MatLoad(string fn, int[] usecols,
          char sep, string comment)
        {
            // self-contained version
            // first, count number of non-comment lines
            int nRows = 0;
            string line = "";
            FileStream ifs = new FileStream(fn, FileMode.Open);
            StreamReader sr = new StreamReader(ifs);
            while ((line = sr.ReadLine()) != null)
                if (line.StartsWith(comment) == false)
                    ++nRows;
            sr.Close(); ifs.Close();

            int nCols = usecols.Length;
            double[][] result = new double[nRows][];
            for (int r = 0; r < nRows; ++r)
                result[r] = new double[nCols];

            line = "";
            string[] tokens = null;
            ifs = new FileStream(fn, FileMode.Open);
            sr = new StreamReader(ifs);

            int i = 0;
            while ((line = sr.ReadLine()) != null)
            {
                if (line.StartsWith(comment) == true)
                    continue;
                tokens = line.Split(sep);
                for (int j = 0; j < nCols; ++j)
                {
                    int k = usecols[j];  // into tokens
                    result[i][j] = double.Parse(tokens[k]);
                }
                ++i;
            }
            sr.Close(); ifs.Close();
            return result;
        }

    } // Program

} // ns