using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace IK.Tools
{

    public class SimpleJacobianMatrix
    {
        public float[,] value;
        public int rows = 3;
        public int column;

        public SimpleJacobianMatrix()
        {
            rows = 3;
            column = 0;
        }

        public void AddColumn(Vector3 a)
        {

            if (column == 0)
            {
                column += 1;
                value = new float[rows, column];
                value[0, 0] = a.x;
                value[1, 0] = a.y;
                value[2, 0] = a.z;
                return;
            }
            column += 1;
            float[,] tempValue = value;

            value = new float[rows, column];
            for (int i = 0; i < column - 1; i++)
            {

                for (int j = 0; j < rows || j < 3; j++)
                {
                    value[j, i] = tempValue[j, i];
                }
            }


            value[0, column - 1] = a.x;
            value[1, column - 1] = a.y;
            value[2, column - 1] = a.z;
            // DebugMatrix();
        }

        public void DebugMatrix()
        {
            string debugString = "Jacobian Matrix value :";
            for (int i = 0; i < column; i++)
            {
                debugString += "\n";
                for (int j = 0; j < rows; j++)
                {
                    debugString += " " + value[j, i].ToString() + " ,";
                }
            }

            Debug.Log(debugString);
        }

        public SimpleJacobianMatrix Transpose()
        {
            float[,] tempValue = new float[rows, column];
            tempValue = value;
            value = new float[column, rows];
            for (int i = 0; i < column; i++)
            {
                for (int j = 0; j < rows; j++)
                {
                    value[i, j] = tempValue[j, i];
                }
            }
            int tempRows = column;
            column = rows;
            rows = tempRows;
            //   DebugMatrix();
            return this;
        }

        public static float[] operator *(SimpleJacobianMatrix a, Vector3 b)
        {
            float[] values = new float[a.rows];
            for (int i = 0; i < a.rows; i++)
            {

                values[i] += a.value[i, 0] * b.x;
                values[i] += a.value[i, 1] * b.y;
                values[i] += a.value[i, 2] * b.z;
            }

            return values;
        }

        public static float[] MultiplyLocationDirectoin(SimpleJacobianMatrix a, Vector3 location, Vector3 direction)
        {
            float[] values = new float[a.rows];
            for (int i = 0; i < a.rows; i++)
            {

                values[i] += a.value[i, 0] * location.x;
                values[i] += a.value[i, 1] * location.y;
                values[i] += a.value[i, 2] * location.z;

                values[i] += a.value[i, 0] * direction.x;
                values[i] += a.value[i, 1] * direction.y;
                values[i] += a.value[i, 2] * direction.z;
            }

            return values;
        }

    }

    /// <summary>
    /// Adjoint Matrix Block 6x6;
    /// </summary>
    public class AdjointBlock
    {
        /// <summary>
        /// Row Major -> [Row][Column]
        /// </summary>
        public float[,] values;
        public int column
        {
            get { return 6; }
            private set { }
        }
        public int rows
        {
            get { return 6; }
            private set { }
        }

        public AdjointBlock()
        {
            values = new float[rows, column];
        }


        public void DebugMatrix()
        {
            string debugString = "Jacobian Matrix value :";
            for (int i = 0; i < column; i++)
            {
                debugString += "\n";
                for (int j = 0; j < rows; j++)
                {
                    debugString += " " + values[j, i].ToString() + " ,";
                }
            }

            Debug.Log(debugString);
        }

        public void AddBlock(Matrix4x4 mat, int sizeRow, int sizeColumn, int indexRow, int indexColumn)
        {
            for (int i = 0; i < sizeRow; i++)
            {
                for (int j = 0; j < sizeColumn; j++)
                {
                    values[indexRow + i, indexColumn + j] = mat[i, j];
                }
            }
        }
    }


    public class Vector6
    {
        public float[] values = new float[6];

        public Vector6(Vector3 firstPart, Vector3 secondPart)
        {
            values[0] = firstPart.x;
            values[1] = firstPart.y;
            values[2] = firstPart.z;

            values[3] = secondPart.x;
            values[4] = secondPart.y;
            values[5] = secondPart.z;
        }
    }


    public class JacobianMatrix
    {

        public double[,] value;
        public int rows;
        public int column;


        public JacobianMatrix(int rowSize, int columnSize)
        {
            rows = rowSize;
            column = columnSize;
            value = new double[rowSize, columnSize];
        }



        public void AddData(float[,] values, int rowSize, int columnSize, int indexColunm, int indexRow = 0)
        {
            for (int i = 0; i < rowSize; i++)
            {
                for (int j = 0; j < columnSize; j++)
                {
                    value[indexRow + i, indexColunm + j] = values[i, j];
                }
            }
        }

        public void AddData(double[][]values, int rowSize, int columnSize, int indexColunm, int indexRow = 0)
        {
            rows = rowSize;
            column = columnSize;
            value = new double[rowSize, columnSize];
            for (int i = 0; i < rowSize; i++)
            {
                for (int j = 0; j < columnSize; j++)
                {
                    value[indexRow + i, indexColunm + j] = values[i][j];
                }
            }
        }

        public void DebugMatrix()
        {
            string debugString = "Jacobian Matrix value :";
            for (int i = 0; i < column; i++)
            {
                debugString += "\n";
                for (int j = 0; j < rows; j++)
                {
                    debugString += " " + value[j, i].ToString() + " ,";
                }
            }

            Debug.Log(debugString);
        }

        public void Transpose()
        {
            double[,] tempValue = new double[rows, column];
            tempValue = value;
            value = new double[column, rows];
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < column; j++)
                {
                    value[j, i] = tempValue[i, j];
                }
            }
            int tempRows = column;
            column = rows;
            rows = tempRows;
        }


        public static double[] operator *(JacobianMatrix a, Vector6 b)
        {
            double[] values = new double[a.rows];
            for (int i = 0; i < a.rows; i++)
            {
                for (int j = 0; j < b.values.Length; j++)
                {
                    values[i] += a.value[i, j] * b.values[j];
                }

            }

            return values;
        }



        public static JacobianMatrix operator *(JacobianMatrix a, JacobianMatrix b)
        {

            if (a.column != b.rows)
            {
                Debug.LogError("JacobianMatrix not matching");
                JacobianMatrix jacobianErrorMatrix = new JacobianMatrix(0, 0);
                return jacobianErrorMatrix;
            }

            JacobianMatrix jacobianMatrix = new JacobianMatrix(b.rows, b.column);

            for (int i = 0; i < b.rows; i++)
            {
                for (int j = 0; j < b.rows; j++)
                {
                    for (int k = 0; k < b.column; k++)
                    {

                        jacobianMatrix.value[i, j] += a.value[i, k] * b.value[k, j];
                    }
                }

            }

            return jacobianMatrix;
        }

        public double[][] ConvertMatInArray()
        {
            double[][] newMat = new double[rows][];
            for (int i = 0; i < rows; i++)
            {
                newMat[i] = new double[column];

                for (int j = 0; j < column; j++)
                {
                    newMat[i][j] = value[i, j];
                }

            }

            return newMat;
        }

        //public JacobianMatrix Inverse()
        //{
        //    JacobianMatrix jacobianMatrix = new JacobianMatrix(this.rows, this.column);
        //    return jacobianMatrix;
        //}


    }
}
