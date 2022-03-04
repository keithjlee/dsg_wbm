using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace dsg_wbm
{
    public class Element
    {
        // Fields
        public Tuple<int, int>[]? NodeIndex { get; set; } // {(start node index, end node index)}
        public int Dim { get; } // 2 or 3
        public int Type { get; } // 0 = 2D truss, 1 = 3D truss, 2 = 2D frame, 3 = 3D frame
        public int NDOFS { get; } // 4, 6, or 12
        public double[] PosStart { get; } = Array.Empty<double>(); // [x, y,;z] position of start node
        public double[] PosEnd { get; } = Array.Empty<double>(); // [x, y; z] position of end node
        public double Length { get; } // length of element
        public double[,] K { get; } // stiffness matrix in GCS USE AS DENSE MATRIX
        public int[] DOFIndex { get; } = Array.Empty<int>(); // [DOF indices in global ordering system] 
        public double[] LocalForces { get; } = Array.Empty<double>(); // [member forces in LCS]
        public double[] GlobalForces { get; } = Array.Empty<double>(); // [member forces in GCS]
        public double AxialForce { get; } // Axial force (- compression, + tension)
        public double A { get; set; } // Area
        public double E { get; set; } // Young's Modulus
        public double G { get; set; } // Shear Modulus
        public double Iz { get; set; } // Strong axis moment of inertia
        public double Iy { get; set; } // weak axis moment of inertia
        public double J { get; set; } // Torsional constant 
        public double Psi { get; set; } // LCS reference angle from local x axis
        public double[,]? R { get; } // Rotation matrix
        public int[][]? LCS { get; } // [[local x], [local y], [local z]]

        public Element(Node[] nodes, int[] nodeIndices)
        {
            this.Dim = nodes[0].Position.Length;

            // Add starting and ending positions
            int iStart = nodeIndices[0];
            int iEnd = nodeIndices[1];
            this.PosStart = nodes[iStart].Position;
            this.PosEnd = nodes[iEnd].Position;

            this.NDOFS = nodes[iStart].DOFS.Length + nodes[iEnd].DOFS.Length; // total number of DOFs

            // Element length --> use ramons vector type, and Math.Norm()
            double sqrlen = 0;
            for (int i = 0; i < this.Dim; i++)
            {
                sqrlen += Math.Pow((this.PosEnd[i] - this.PosStart[i]), 2);
            }

            this.Length = Math.Sqrt(sqrlen);


        }
    }
}
