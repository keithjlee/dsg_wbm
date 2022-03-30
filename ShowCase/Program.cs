﻿using dsg_wbm;
using System.Drawing;
using MathNet.Numerics.LinearAlgebra;
// See https://aka.ms/new-console-template for more information
Console.WriteLine("Tests:");


// Creating Nodes
Vector<double> n1position = Vector<double>.Build.DenseOfArray( new double[3] { 0, 0, 0 });
List<bool> n1dofs = new List<bool> { false, false, false, false, false, false };
Node n1 = new Node(n1position, n1dofs);

Vector<double> n2position = Vector<double>.Build.DenseOfArray(new double[3] { 3, 0, 0 });
List<bool> n2dofs = new List<bool> { true, true, true, true, true, true };
Node n2 = new Node(n2position, n2dofs);

Vector<double> n3position = Vector<double>.Build.DenseOfArray(new double[3] { 0, 0, -3 });
List<bool> n3dofs = new List<bool> { false, false, false, false, false, false };
Node n3 = new Node(n3position, n3dofs);

Vector<double> n4position = Vector<double>.Build.DenseOfArray(new double[3] { 0, -4, 0 });
List<bool> n4dofs = new List<bool> { false, false, false, false, false, false };
Node n4 = new Node(n4position, n4dofs);

// Material properties
double E = 210e6;
double A = 0.02;
double Iy = 10e-5;
double Iz = 20e-5;
double J = 5e-5;
double G = 84e6;

// list of nodes
List<Node> nodes = new List<Node>();
nodes.Add(n1);
nodes.Add(n2);
nodes.Add(n3);
nodes.Add(n4);



