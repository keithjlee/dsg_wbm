using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace dsg_wbm
{
    public class Structure
    {
        public List<Node> Nodes; // List of nodes in structure
        public List<Element> Elements; // List of elements in structure
        public List<Load>? Loads; // List of loads on structure
        public Matrix<double>? K; // global stiffness matrix
        //public List<Node>? DisplacedNodes; // list of displaced nodes
        //public List<Element>? DisplacedElements; // list of displaced elements
        public List<bool> DOFs { get
            {
                List<bool> dofs = new List<bool>();
                if (Nodes != null)
                {
                    foreach (Node node in Nodes)
                    {
                        dofs.AddRange(node.DOFS);
                    }
                }
                return dofs;
            } } // list of global DOFs
        public Vector<double>? F; // external force vectors converted from Loads
        public Matrix<double>? U; // Nodal displacements
        public Matrix<double>? Reactions; // Reactions at fixed DOFs
        public double? Compliance; // Performance proxy of structure
        public int nNodes { get
            {
                if (Nodes != null)
                {
                    return Nodes.Count;
                }
                else
                {
                    return 0;
                }
            }
        } // number of nodes
        public int nElements
        {
            get
            {
                if (Elements != null)
                {
                    return Elements.Count;
                }
                else
                {
                    return 0;
                }
            }
        } // number of elements
        public int nDOFs
        {
            get
            {
                if (this.DOFs != null)
                {
                    return this.DOFs.Count;
                }
                else
                {
                    return 0;
                }
            }
        } // number of DOFs
        public List<int> freeDOFs
        {
            get
            {
                List<int> dofIndices = new List<int>();
                if (this.DOFs.Count != 0 && this.nDOFs != 0)
                {
                    for (int i = 0; i < this.nDOFs; i++)
                    {
                        if (this.DOFs[i] == true)
                        {
                            dofIndices.Add(i);
                        }
                    }
                }

                return dofIndices;
            }
        }

        /// <summary>
        /// Base constructor with no loads yet defined
        /// </summary>
        /// <param name="nodes"></param>
        /// <param name="elements"></param>
        public Structure(List<Node> nodes, List<Element> elements)
        {
            // Populate basic fields
            this.Nodes = nodes;
            this.Elements = elements;

            // Preprocessing
            AddNodeElements(); // Association between nodes and attached elements
            NodeGlobalIndex(); // global DOF ordering of node DOFs
            DofExpander(); // global DOF ordering of element DOFs (indices for global K)
        }

        /// <summary>
        /// Full constructor with nodes, elements, loads
        /// </summary>
        /// <param name="nodes"></param>
        /// <param name="elements"></param>
        /// <param name="loads"></param>
        public Structure(List<Node> nodes, List<Element> elements, List<Load> loads)
        {
            // Populate basic fields
            this.Nodes = nodes;
            this.Elements = elements;
            this.Loads = loads;

            // Preprocessing
            AddNodeElements(); // Association between nodes and attached elements
            NodeGlobalIndex(); // global DOF ordering of node DOFs
            DofExpander(); // global DOF ordering of element DOFs (indices for global K)

            // Perform pre-processing: local DOF to global DOF indexing
            // Element/node linking
        }

        //Methods
        /// <summary>
        /// For each node in Structure.Nodes, adds a tuple (element index, start/end) for each element associated to that node
        /// </summary>
        /// <exception cref="Exception"></exception>
        private void AddNodeElements()
        {
            if (this.Elements != null && this.Nodes != null)
            {
                for (int i = 0; i < this.nElements; i++)
                {
                    Element e = this.Elements[i];

                    if (e.NodeIndex != null)
                    {
                        int j = e.NodeIndex.Item1;
                        int k = e.NodeIndex.Item2;

                        // Add starting node association + remove duplicates
                        this.Nodes[j].Elements.Add(new Tuple<int, int>(j, -1));
                        this.Nodes[j].Elements = this.Nodes[j].Elements.Distinct().ToList();

                        // Add ending node assocation + remove duplicates
                        this.Nodes[k].Elements.Add(new Tuple<int, int>(k, 1));
                        this.Nodes[k].Elements = this.Nodes[k].Elements.Distinct().ToList();
                    }
                    else
                    {
                        throw new Exception("Node index not populated in element.");
                    }
                   
                }
            }

            else
            {
                throw new Exception("Elements not populated.");
            }
        }

        private void AddNodeLoads()
        {
            if (this.Loads != null && this.Nodes != null)
            {
                foreach(Load load in this.Loads)
                {
                    if (load.LoadValues != null)
                    {
                        this.Nodes[load.Index].Load = load.LoadValues;
                    }
                }
            }
            else
            {
                throw new Exception("Loads or Nodes undefined.");
            }
        }

        // Assigns to each node in nodes the positions of its DOF in the global ordering scheme
        private void NodeGlobalIndex()
        {
            int dofCount = 0;
            //if (this.Nodes != null)
            //{
            foreach (Node node in this.Nodes)
            {
                node.GlobalIndex.Clear(); // reset
                for (int i = dofCount; i < dofCount + node.nDOFS; i++)
                {
                    node.GlobalIndex.Add(i);
                }
                dofCount += node.nDOFS;
            }
            //}
            //else
            //{
            //    throw new Exception("Nodes field is null.");
            //}
        }

        // Assigns the 2 * nDOF
        private void DofExpander()
        {
            //if (this.Elements != null && this.Nodes != null)
            //{
            foreach (Element element in this.Elements)
            {
                if (element.NodeIndex != null)
                {
                    List<int> indices = new List<int>();
                    indices.AddRange(this.Nodes[element.NodeIndex.Item1].GlobalIndex);
                    indices.AddRange(this.Nodes[element.NodeIndex.Item2].GlobalIndex);
                    element.DOFIndex = indices;
                }
                else
                {
                    throw new Exception("An element is not associated with any nodes in the structure.");
                }
                    
            }
            //}
            //else
            //{
            //    throw new Exception("Nodes or Elements field is not yet populated in this structure.");
            //}
        }

        private void MakeElementalK()
        {
            foreach (Element element in this.Elements)
            {
                if (element.R == null)
                {
                    throw new Exception("Element does not have rotation matrix.");
                }

                switch (element.Type)
                {
                    case AnalysisType.Truss:
                        {
                            Matrix<double> kLocal = Analysis.kElementalTruss(element.E, element.A, element.Length);

                            element.K = element.R.Transpose() * kLocal * element.R;
                            break;
                        }

                    case AnalysisType.Frame:
                        {
                            Matrix<double> kLocal = Analysis.kElementalFrame(element.E, element.A, element.G, element.Length, element.Iz, element.Iy, element.J);

                            element.K = element.R.Transpose() * kLocal * element.R;
                            break;
                        }

                    default:
                        {
                            throw new Exception("Analysis type not set for element.");
                        }
                }
            }
            
        }
        // Makes global F vector
        private void MakeGlobalF()
        {
            Vector<double> F = Vector<double>.Build.Dense(this.nDOFs);

            if (this.Nodes != null && this.Loads != null)
            {
                foreach(Load load in this.Loads)
                {
                    List<int> indices = this.Nodes[load.Index].GlobalIndex;

                    if (load.LoadValues != null && indices.Count == load.LoadValues.Count)
                    {
                        for (int i = 0; i < indices.Count; i++)
                        {
                            F[indices[i]] = load.LoadValues[i];
                        }
                    }
                    else
                    {
                        throw new Exception("Load values do not have same dimensions as associated node's DOFs.");
                    }
                }
                this.F = F;
            }
            else
            {
                throw new Exception("Nodes and/or Loads not defined");
            }
        }
        public void Analyze()
        {
            // Populate empty fields: F, DOFs...
            AddNodeLoads();
            MakeElementalK();

            MakeGlobalF();
            DofExpander();

            // build stiffness matrix (if not yet built)

            // Perform Ax = b (either cholesky or LU)

            // Populate empty fields: U, Reactions, DisplacedNodes, Displaced Elements
            
        }
    }
}