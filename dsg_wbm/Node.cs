using System.Drawing;
using static System.Math;
using MathNet.Numerics.LinearAlgebra;


namespace dsg_wbm
{
    public enum NodeType
    {
        Truss,
        Frame
    }
    public class Node
    {
        // Fields
        public Vector<double> Position { get; set; } // [x,y,z] position of node
        public double X { get; } // x position of node
        public double Y { get; } // y position of node
        public double Z { get; } // z position of node
        public List<bool> DOFS { get; set; } // [dx, dy; dz, rx, ry, rz]
        public int nDOFS { get; } // number of DOFs
        public Vector<double> Load { get; set; } // [px, py; pz, mx, my, mz]
        public Vector<double> Reaction { get; } // [rpx, rpy; rpz, rmx, rmy, rmz]
        public Vector<double> Disp { get; } // [dx, dy; dz, mx, my, mz]
        public Tuple<int, int>[]? Elements { get; } // [(elementIndex, 0 = start/1 = end), ...] of all attached elements
        public int[] GlobalIndex { get; } = Array.Empty<int>(); // [positions of DOF in global index order]

        public Node(Vector<double> position, List<bool> dofs)
        {
            this.Position = position;
            this.DOFS = dofs;


            if (position.Count == 3)
            {
                this.X = position[0];
                this.Y = position[1];
                this.Z = position[2];
            }
            else
            {
                this.X = position[0];
                this.Y = position[1];
            }
        }

        }
    }
}