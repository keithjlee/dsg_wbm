using dsg_wbm;
using System.Drawing;
// See https://aka.ms/new-console-template for more information
Console.WriteLine("Tests:");


// Creating a node
double[] pos = new double[3] { 3.2, 1.3, 5.5 };
bool[] dofs = new bool[6] { true, false, true, true, false, false };
int[]testindices = new int[2] {1, 0};

// extracting nonlinearly from array
var rearrangedPositions = new ArraySegment<double>(pos, 2, 1);

Node mynode = new(pos, dofs);

// assigning after class creation
mynode.Load = new double[] { 4.3, 1.2, 1.1, 7.6, 4.4, 3.3 };

//
int testIndex = (int)pos[testindices[0]];

Console.WriteLine(mynode.Position[0]);
Console.WriteLine(rearrangedPositions[0]);
Console.WriteLine(testIndex);