using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

namespace dsg_wbm
{
    public class Analysis
    {
        /// <summary>
        /// Base stiffness matrix for truss without transformation
        /// </summary>
        /// <param name="E"></param>
        /// <param name="A"></param>
        /// <param name="L"></param>
        /// <returns></returns>
        public Matrix<double> kElementalTruss(double E, double A, double L)
        {
            Matrix<double> k = Matrix<double>.Build.Dense(4, 4);
            k[0, 0] = E * A / L;
            k[0, 1] = - E * A / L;
            k[1, 0] = - E * A / L;
            k[1, 1] = E * A / L;
            return k;
        }

        /// <summary>
        /// Base stiffness matrix for frame without transformation
        /// </summary>
        /// <param name="E"></param>
        /// <param name="A"></param>
        /// <param name="G"></param>
        /// <param name="L"></param>
        /// <param name="Iz"></param>
        /// <param name="Iy"></param>
        /// <param name="J"></param>
        /// <returns></returns>
        public Matrix<double> kElementalFrame(double E, double A, double G, double L, double Iz, double Iy, double J)
        {
            Matrix<double> k = Matrix<double>.Build.Dense(12, 12);

            return k;
        }
    }
}
