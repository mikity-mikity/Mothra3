﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

using My = MyMath.Symbolic;

using Mothra.UI;

using ShoNS.Array;

namespace mikity.ghComponents
{
    public partial class Mothra3 : Grasshopper.Kernel.GH_Component
    {
        public class slice
        {
            public Plane pl;
            public int varOffset;
            public slice(Plane _pl)
            {
                pl = _pl;
            }
            public void update(Plane _pl)
            {
                pl=_pl;
            }
        }
        public class branch
        {
            public NurbsCurve crv;
            public NurbsCurve airyCrv;
            public leaf target = null;
            public leaf left=null,right=null;
            public int varOffset;
            public int conOffset;
            public int N;
            public slice slice;
            public enum type
            {
                reinforce,open,kink,fix
            }
            public type branchType;
        }
        public class leaf
        {
            public int varOffset;
            public int conOffset;
            //public List<Mesher.Geometry.Edge>[] bbOut;
            //public List<Line>[] triEdges;
            //public List<Line> result;
            public Minilla3D.Objects.masonry myMasonry;

            public NurbsSurface srf;
            public NurbsSurface[] airySrf=new NurbsSurface[4];
            public NurbsSurface airySrfCombined;
            public branch[] branch=new branch[4];
            public bool[] flip = new bool[4] { false, false, false, false };
            public SparseDoubleArray Laplacian;
            public SparseDoubleArray shiftArray;
            public int n, m, r;  //Number of vertices, edges and triangles.
            public int nU, nV;
            public int uDim, vDim;
            public int uDdim, vDdim;
            public int nUelem;
            public int nVelem;
            public int NN;
            public double scaleU, scaleV, originU, originV;
            public Interval domU, domV;
            //public Mesher.Data.Vertex[] vertices;
            //public Mesher.Geometry.Edge[] edges;
            //public Mesher.Data.Triangle[] triangles;
            public tuple_ex[] tuples;
            public List<int> fixedPoints;
            public DoubleArray[] baseFunction; //right-hand side
            public DoubleArray[] coeff;        // coefficients
            public Func<double, double, double>[] Function;
            public Action<double, double, double[]>[] dFunction;
            public Action<double, double, double[,]>[] ddFunction;
        }
        public class tuple_ex:Minilla3D.Elements.nurbsElement.tuple
        {
            public tuple_ex(int _N, double _ou, double _ov, double _u, double _v, int _index, double _loU, double _loV, double _area):base(_N, _ou, _ov, _u,_v,  _index, _loU, _loV, _area)
            {}
            
            public void init(NurbsSurface S,double scaleU,double scaleV)
            {
                Point3d P;
                Vector3d[] V;
                S.Evaluate(u, v, 1, out P, out V);
                x = P.X;
                y = P.Y;
                gi2[0][0] = V[0].X * scaleU;
                gi2[0][1] = V[0].Y * scaleU;
                gi2[0][2] = 0;
                gi2[1][0] = V[1].X * scaleV;
                gi2[1][1] = V[1].Y * scaleV;
                gi2[1][2] = 0;
                gij2[0, 0] = gi2[0][0] * gi2[0][0] + gi2[0][1] * gi2[0][1];
                gij2[1, 0] = gi2[1][0] * gi2[0][0] + gi2[1][1] * gi2[0][1];
                gij2[0, 1] = gi2[0][0] * gi2[1][0] + gi2[0][1] * gi2[1][1];
                gij2[1, 1] = gi2[1][0] * gi2[1][0] + gi2[1][1] * gi2[1][1];
                double det = gij2[0, 0] * gij2[1, 1] - gij2[0, 1] * gij2[1, 0];
                Gij2[0, 0] = gij2[1, 1] / det;
                Gij2[1, 1] = gij2[0, 0] / det;
                Gij2[0, 1] = -gij2[0, 1] / det;
                Gij2[1, 0] = -gij2[1, 0] / det;
                Gi2[0][0]=Gij2[0,0]*gi2[0][0]+Gij2[1,0]*gi2[1][0];
                Gi2[0][1]=Gij2[0,0]*gi2[0][1]+Gij2[1,0]*gi2[1][1];
                Gi2[0][2]=0;
                Gi2[1][0]=Gij2[0,1]*gi2[0][0]+Gij2[1,1]*gi2[1][0];
                Gi2[1][1]=Gij2[0,1]*gi2[0][1]+Gij2[1,1]*gi2[1][1];
                Gi2[1][2]=0;
                S.Evaluate(u, v, 2, out P, out V);
                second2[0, 0][0] = V[2][0] * scaleU * scaleU;
                second2[0, 0][1] = V[2][1] * scaleU * scaleU;
                second2[0, 0][2] = 0;
                second2[1, 1][0] = V[4][0] * scaleV * scaleV;
                second2[1, 1][1] = V[4][1] * scaleV * scaleV;
                second2[1, 1][2] = 0;
                second2[0, 1][0] = V[3][0] * scaleU * scaleV;
                second2[0, 1][1] = V[3][1] * scaleU * scaleV;
                second2[0, 1][2] = 0;
                second2[1, 0][0] = V[3][0] * scaleV * scaleU;
                second2[1, 0][1] = V[3][1] * scaleV * scaleU;
                second2[1, 0][2] = 0;
                Gammaijk2[0, 0, 0] = second2[0, 0][0] * Gi2[0][0] + second2[0, 0][1] * Gi2[0][1];
                Gammaijk2[0, 0, 1] = second2[0, 0][0] * Gi2[1][0] + second2[0, 0][1] * Gi2[1][1];
                Gammaijk2[0, 1, 0] = second2[0, 1][0] * Gi2[0][0] + second2[0, 1][1] * Gi2[0][1];
                Gammaijk2[0, 1, 1] = second2[0, 1][0] * Gi2[1][0] + second2[0, 1][1] * Gi2[1][1];
                Gammaijk2[1, 0, 0] = second2[1, 0][0] * Gi2[0][0] + second2[1, 0][1] * Gi2[0][1];
                Gammaijk2[1, 0, 1] = second2[1, 0][0] * Gi2[1][0] + second2[1, 0][1] * Gi2[1][1];
                Gammaijk2[1, 1, 0] = second2[1, 1][0] * Gi2[0][0] + second2[1, 1][1] * Gi2[0][1];
                Gammaijk2[1, 1, 1] = second2[1, 1][0] * Gi2[1][0] + second2[1, 1][1] * Gi2[1][1];
            }
        }
        ControlBox myControlBox = new ControlBox();
        List<Surface> _listSrf;
        List<Curve> _listCrv;
        List<leaf> listLeaf;
        List<branch> listBranch;
        List<Point3d> a;
        List<Point3d> a2;
        List<Line> crossCyan;
        List<Line> crossMagenta;
        List<slice> listSlice;
        int lastComputed = -1;
        int currentAiry = 0;
        private void init()
        {
            a = new List<Point3d>();
            a2 = new List<Point3d>();
            lastComputed = -1;
            crossCyan = new List<Line>();
            crossMagenta = new List<Line>();
            listSlice = new List<slice>();
        }
        public Mothra3()
            : base("Mothra3", "Mothra3", "Mothra3", "Kapybara3D", "Computation")
        {
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("191d64c1-2ed5-4bfc-96e4-800fe372ad0a"); }
        }
        protected override void RegisterInputParams(Grasshopper.Kernel.GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddSurfaceParameter("listSurface", "lstSrf", "list of surfaces", Grasshopper.Kernel.GH_ParamAccess.list);
            pManager.AddCurveParameter("listCurve", "lstCrv", "list of curves", Grasshopper.Kernel.GH_ParamAccess.list);
            pManager.AddTextParameter("listType", "lstType", "list of types of edge curves", Grasshopper.Kernel.GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(Grasshopper.Kernel.GH_Component.GH_OutputParamManager pManager)
        {
        }
        public override void AddedToDocument(Grasshopper.Kernel.GH_Document document)
        {
            base.AddedToDocument(document);
            myControlBox.Show();
            myControlBox.setFunctionToCompute2(() => { computeF(); });
        }
        void computeF()
        {
            if (lastComputed == 3) {
                foreach (var leaf in listLeaf)
                {
                    leaf.NN = 4;
                    double area = 1d / ((double)leaf.NN) / ((double)leaf.NN);
                    //setup functions
                    /*bottom*/
                    var _f1 = new My.Lambda((u, v,u0,v0) => u * (u - u0) * (v + v0) * (v - v0) / Math.Sqrt(u * (u - u0) * (v + v0) * (v - v0) * u * (u - u0) * (v + v0) * (v - v0) + u * (u - u0) * (v) * (v - 2 * v0) * u * (u - u0) * (v) * (v - 2 * v0) + (u + u0) * (u - u0) * (v) * (v - v0) * (u + u0) * (u - u0) * (v) * (v - v0) + u * (u - 2 * u0) * (v) * (v - v0) * u * (u - 2 * u0) * (v) * (v - v0)));
                    /*right*/
                    var _f2 = new My.Lambda((u, v, u0, v0) => u * (u - 2 * u0) * (v) * (v - v0) / Math.Sqrt(u * (u - u0) * (v + v0) * (v - v0) * u * (u - u0) * (v + v0) * (v - v0) + u * (u - u0) * (v) * (v - 2 * v0) * u * (u - u0) * (v) * (v - 2 * v0) + (u + u0) * (u - u0) * (v) * (v - v0) * (u + u0) * (u - u0) * (v) * (v - v0) + u * (u - 2 * u0) * (v) * (v - v0) * u * (u - 2 * u0) * (v) * (v - v0)));
                    /*top*/
                    var _f3 = new My.Lambda((u, v, u0, v0) => u * (u - u0) * (v) * (v - 2 * v0) / Math.Sqrt(u * (u - u0) * (v + v0) * (v - v0) * u * (u - u0) * (v + v0) * (v - v0) + u * (u - u0) * (v) * (v - 2 * v0) * u * (u - u0) * (v) * (v - 2 * v0) + (u + u0) * (u - u0) * (v) * (v - v0) * (u + u0) * (u - u0) * (v) * (v - v0) + u * (u - 2 * u0) * (v) * (v - v0) * u * (u - 2 * u0) * (v) * (v - v0)));
                    /*left*/
                    var _f4 = new My.Lambda((u, v, u0, v0) => (u + u0) * (u - u0) * (v) * (v - v0) / Math.Sqrt(u * (u - u0) * (v + v0) * (v - v0) * u * (u - u0) * (v + v0) * (v - v0) + u * (u - u0) * (v) * (v - 2 * v0) * u * (u - u0) * (v) * (v - 2 * v0) + (u + u0) * (u - u0) * (v) * (v - v0) * (u + u0) * (u - u0) * (v) * (v - v0) + u * (u - 2 * u0) * (v) * (v - v0) * u * (u - 2 * u0) * (v) * (v - v0)));
                    var f1 = (Func<double, double, double, double, double>)_f1.Compile();
                    var f2 = (Func<double, double, double, double, double>)_f2.Compile();
                    var f3 = (Func<double, double, double, double, double>)_f3.Compile();
                    var f4 = (Func<double, double, double, double, double>)_f4.Compile();
                    leaf.Function[0] = new Func<double, double, double>((u, v) => { return f1(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.Function[1] = new Func<double, double, double>((u, v) => { return f2(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.Function[2] = new Func<double, double, double>((u, v) => { return f3(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.Function[3] = new Func<double, double, double>((u, v) => { return f4(u, v, leaf.nUelem, leaf.nVelem); });
                    var _df1du = _f1.Derive("u");
                    var _df1dv = _f1.Derive("v");
                    var _df2du = _f2.Derive("u");
                    var _df2dv = _f2.Derive("v");
                    var _df3du = _f3.Derive("u");
                    var _df3dv = _f3.Derive("v");
                    var _df4du = _f4.Derive("u");
                    var _df4dv = _f4.Derive("v");
                    var df1du = (Func<double, double, double, double, double>)_df1du.Compile();
                    var df1dv = (Func<double, double, double, double, double>)_df1dv.Compile();
                    var df2du = (Func<double, double, double, double, double>)_df2du.Compile();
                    var df2dv = (Func<double, double, double, double, double>)_df2dv.Compile();
                    var df3du = (Func<double, double, double, double, double>)_df3du.Compile();
                    var df3dv = (Func<double, double, double, double, double>)_df3dv.Compile();
                    var df4du = (Func<double, double, double, double, double>)_df4du.Compile();
                    var df4dv = (Func<double, double, double, double, double>)_df4dv.Compile();
                    leaf.dFunction[0] = new Action<double, double, double[]>((u, v, res) => { res[0] = df1du(u, v, leaf.nUelem, leaf.nVelem); res[1] = df1dv(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.dFunction[1] = new Action<double, double, double[]>((u, v, res) => { res[0] = df2du(u, v, leaf.nUelem, leaf.nVelem); res[1] = df2dv(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.dFunction[2] = new Action<double, double, double[]>((u, v, res) => { res[0] = df3du(u, v, leaf.nUelem, leaf.nVelem); res[1] = df3dv(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.dFunction[3] = new Action<double, double, double[]>((u, v, res) => { res[0] = df4du(u, v, leaf.nUelem, leaf.nVelem); res[1] = df4dv(u, v, leaf.nUelem, leaf.nVelem); });

                    var _d2f1dudu = _df1du.Derive("u");
                    var _d2f1dudv = _df1du.Derive("v");
                    var _d2f1dvdv = _df1dv.Derive("v");
                    var _d2f2dudu = _df2du.Derive("u");
                    var _d2f2dudv = _df2du.Derive("v");
                    var _d2f2dvdv = _df2dv.Derive("v");
                    var _d2f3dudu = _df3du.Derive("u");
                    var _d2f3dudv = _df3du.Derive("v");
                    var _d2f3dvdv = _df3dv.Derive("v");
                    var _d2f4dudu = _df4du.Derive("u");
                    var _d2f4dudv = _df4du.Derive("v");
                    var _d2f4dvdv = _df4dv.Derive("v");
                    var d2f1dudu = (Func<double, double, double, double, double>)_d2f1dudu.Compile();
                    var d2f1dudv = (Func<double, double, double, double, double>)_d2f1dudv.Compile();
                    var d2f1dvdv = (Func<double, double, double, double, double>)_d2f1dvdv.Compile();
                    var d2f2dudu = (Func<double, double, double, double, double>)_d2f2dudu.Compile();
                    var d2f2dudv = (Func<double, double, double, double, double>)_d2f2dudv.Compile();
                    var d2f2dvdv = (Func<double, double, double, double, double>)_d2f2dvdv.Compile();
                    var d2f3dudu = (Func<double, double, double, double, double>)_d2f3dudu.Compile();
                    var d2f3dudv = (Func<double, double, double, double, double>)_d2f3dudv.Compile();
                    var d2f3dvdv = (Func<double, double, double, double, double>)_d2f3dvdv.Compile();
                    var d2f4dudu = (Func<double, double, double, double, double>)_d2f4dudu.Compile();
                    var d2f4dudv = (Func<double, double, double, double, double>)_d2f4dudv.Compile();
                    var d2f4dvdv = (Func<double, double, double, double, double>)_d2f4dvdv.Compile();
                    leaf.ddFunction[0] = new Action<double, double, double[,]>((u, v, res) => { res[0, 0] = d2f1dudu(u, v, leaf.nUelem, leaf.nVelem); res[0, 1] = d2f1dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 0] = d2f1dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 1] = d2f1dvdv(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.ddFunction[1] = new Action<double, double, double[,]>((u, v, res) => { res[0, 0] = d2f2dudu(u, v, leaf.nUelem, leaf.nVelem); res[0, 1] = d2f2dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 0] = d2f2dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 1] = d2f2dvdv(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.ddFunction[2] = new Action<double, double, double[,]>((u, v, res) => { res[0, 0] = d2f3dudu(u, v, leaf.nUelem, leaf.nVelem); res[0, 1] = d2f3dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 0] = d2f3dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 1] = d2f3dvdv(u, v, leaf.nUelem, leaf.nVelem); });
                    leaf.ddFunction[3] = new Action<double, double, double[,]>((u, v, res) => { res[0, 0] = d2f4dudu(u, v, leaf.nUelem, leaf.nVelem); res[0, 1] = d2f4dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 0] = d2f4dudv(u, v, leaf.nUelem, leaf.nVelem); res[1, 1] = d2f4dvdv(u, v, leaf.nUelem, leaf.nVelem); });
                    //setup tuples
                    leaf.r = leaf.nUelem * leaf.nVelem * leaf.NN * leaf.NN;
                    leaf.tuples = new tuple_ex[leaf.r];
                    for (int vv = 0; vv < leaf.NN * leaf.nVelem; vv++)
                    {
                        for (int uu = 0; uu < leaf.NN*leaf.nUelem; uu++)
                        {
                            int num = uu + vv * (leaf.NN * leaf.nUelem);
                            double centerU = (uu + 0.5) / leaf.NN;
                            double centerV = (vv + 0.5) / leaf.NN;
                            //element index
                            int uNum = (int)centerU;
                            int vNum = (int)centerV;
                            int index = uNum + vNum * leaf.nUelem;
                            //local coordinates
                            double localU = centerU - uNum;
                            double localV = centerV - vNum;
                            leaf.tuples[num] = new tuple_ex(4, centerU, centerV, centerU * leaf.scaleU + leaf.originU, centerV * leaf.scaleV + leaf.originV, index, localU, localV, area);
                            leaf.tuples[num].init(leaf.srf, leaf.scaleU, leaf.scaleV);
                            
                            for (int s = 0; s < 4; s++)
                            {
                                leaf.tuples[num].nf[s]=leaf.Function[s](centerU, centerV);
                                leaf.dFunction[s](centerU, centerV, leaf.tuples[num].ndf[s]);
                                leaf.ddFunction[s](centerU, centerV, leaf.tuples[num].nddf[s]);                                
                            }
                        }
                    }
                    createNurbsElements(leaf);
                    double[,] x;
                    x = new double[leaf.nU * leaf.nV, 3];
                    Nurbs2x(leaf, x);
                    leaf.myMasonry.setupNodesFromList(x);
                    leaf.myMasonry.computeGlobalCoord();
                    foreach (var e in leaf.myMasonry.elemList)
                    {
                        e.precompute();
                        e.computeBaseVectors();
                    }
                    foreach (var tup in leaf.tuples)
                    {
                        leaf.myMasonry.elemList[tup.index].precompute(tup);
                    }
                }
                //call mosek
                mosek1(listLeaf,listBranch);
                //For preview
                crossMagenta.Clear();
                crossCyan.Clear();
                foreach (var leaf in listLeaf)
                {
                    foreach (var tuple in leaf.tuples)
                    {
                        for (int i = 0; i < 2; i++)
                        {
                            if (tuple.eigenValues[i] < 0)
                            {
                                double s = tuple.eigenValues[i]*0.05;
                                //double s = 0.1;
                                Point3d S = new Point3d(tuple.x - tuple.eigenVectors[i][0]*s, tuple.y - tuple.eigenVectors[i][1]*s, tuple.z - tuple.eigenVectors[i][2]*s);
                                Point3d E = new Point3d(tuple.x + tuple.eigenVectors[i][0]*s, tuple.y + tuple.eigenVectors[i][1]*s, tuple.z + tuple.eigenVectors[i][2]*s);
                                Line line = new Line(S, E);
                                crossCyan.Add(line);
                            }
                            else
                            {
                                double s = tuple.eigenValues[i]*0.05;
                                //double s = 0.1;
                                Point3d S = new Point3d(tuple.x - tuple.eigenVectors[i][0] * s, tuple.y - tuple.eigenVectors[i][1] * s, tuple.z - tuple.eigenVectors[i][2] * s);
                                Point3d E = new Point3d(tuple.x + tuple.eigenVectors[i][0]*s, tuple.y + tuple.eigenVectors[i][1]*s, tuple.z + tuple.eigenVectors[i][2]*s);
                                Line line = new Line(S, E);
                                crossMagenta.Add(line);
                            }
                        }
                    }
                }

            } else { System.Windows.Forms.MessageBox.Show("Not Ready.");}
        }

        public bool findCurve(leaf leaf,ref branch target, List<branch> listBranch, NurbsCurve curve)
        {
            var Points = curve.Points;
            var rPoints = curve.Points.Reverse().ToList();

            foreach (var branch in listBranch)
            {
                if (branch.crv.Points[0].Location.DistanceTo(Points[0].Location) < 0.0001)
                {
                    if (branch.crv.Points[1].Location.DistanceTo(Points[1].Location) < 0.0001)
                    {
                        target = branch;
                        if (branch.branchType != branch.type.kink)
                        {
                            branch.target = leaf;
                        }
                        else
                        {
                            if (branch.left == null) branch.left = leaf; else branch.right = leaf;
                        }
                        return false;
                    }
                }
                else if (branch.crv.Points[0].Location.DistanceTo(rPoints[0].Location) < 0.0001)
                {
                    if (branch.crv.Points[1].Location.DistanceTo(rPoints[1].Location) < 0.0001)
                    {
                        target = branch;
                        if (branch.branchType != branch.type.kink)
                        {
                            branch.target = leaf;
                        }
                        else
                        {
                            if (branch.left == null) branch.left = leaf; else branch.right = leaf;
                        }
                        return true;
                    }
                }
            }
            AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "cannnot find");
            return false;
        }
        public void resultToPreview(int i)
        {
            a2.Clear();
            foreach(var leaf in listLeaf)
            {
                foreach (var tuple in leaf.tuples)
                {
                    double x, y, z;
                    x = tuple.ou+30;
                    y = tuple.ov;
                    z = tuple.nf[i];
                    var P = new Point3d(x, y, z);
                    a2.Add(P);
                }
            }
        }
        protected override void SolveInstance(Grasshopper.Kernel.IGH_DataAccess DA)
        {
            init();
            _listSrf = new List<Surface>();
            _listCrv = new List<Curve>();
            List<string> types = new List<string>();
            if (!DA.GetDataList(0, _listSrf)) { return; }
            if (!DA.GetDataList(1, _listCrv)) { return; }
            if (!DA.GetDataList(2, types)) { return;}
            
            if(_listCrv.Count!=types.Count){AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "need types for curves"); return; }
            listLeaf = new List<leaf>();
            listBranch = new List<branch>();
            for (int i = 0; i < _listCrv.Count; i++)
            {
                var branch = new branch();
                branch.crv = _listCrv[i] as NurbsCurve;
                branch.N = branch.crv.Points.Count;
                switch (types[i])
                {
                    case "reinforce":
                        branch.branchType = branch.type.reinforce;
                        break;
                    case "kink":
                        branch.branchType = branch.type.kink;
                        break;
                    case "open":
                        branch.branchType = branch.type.open;
                        break;
                    case "fix":
                        branch.branchType = branch.type.fix;
                        break;
                    default:
                        AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "type should be either of reinforce, kink, fix, or open");
                        return;
                }
                listBranch.Add(branch);
            }
            myControlBox.setNumF(4);
            lastComputed = 3;
            for (int s = 0; s < 4; s++)
            {
                myControlBox.EnableRadio(s, (i) => { currentAiry = i; resultToPreview(i); this.ExpirePreview(true); });
            }
/*            myControlBox.setFunctionToCompute(() =>
            {
                if (lastComputed == 3) return;
                lastComputed++;
                computeBaseFunction(lastComputed);
                this.ExpirePreview(true);
                myControlBox.EnableRadio(lastComputed, (i) => { currentAiry = i; resultToPreview(i); this.ExpirePreview(true); });
            }
                );*/
            var pl1 = new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1));
            listSlice.Add(new slice(pl1));
            myControlBox.clearSliders();
            var slider1=myControlBox.addSlider(0, 1, 100, 62);
            slider1.Converter = (val) => {
                pl1 = new Plane(new Point3d(0, 0, val / 10d), new Vector3d(0, 0, 1));
                listSlice[0].update(pl1);
                this.ExpirePreview(true);
                return val / 10d;
            };
            int ss = 1;
            
            foreach (var branch in listBranch)
            {
                if (branch.branchType == branch.type.reinforce) branch.slice = listSlice[0];
                if (branch.branchType == branch.type.open)
                {
                    var plnew = new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1));
                    listSlice.Add(new slice(plnew));
                    branch.slice = listSlice[ss];
                    var slider = myControlBox.addSlider(0, 1, 100, 40);
                    slider.Converter = (val) =>
                    {
                        var O = (branch.crv.Points[0].Location + branch.crv.Points[branch.N - 1].Location) / 2;
                        var V = (branch.crv.Points[1].Location - branch.crv.Points[0].Location);
                        var X = branch.crv.Points[branch.N - 1].Location; ;
                        var Z = new Vector3d(0, 0, 1);
                        var W = Vector3d.CrossProduct(Z, X - O);
                        if (V * W < 0) W.Reverse();
                        Z.Unitize();
                        W.Unitize();
                        var theta = val / 100d * Math.PI / 2d;
                        var Y = O + Z * Math.Cos(theta) + W * Math.Sin(theta);
                        plnew = new Plane(O, X, Y);
                        branch.slice.update(plnew);
                        this.ExpirePreview(true);
                        return val / 100d*Math.PI/2d;
                    };
                    ss++;
                }
            }
            foreach (var srf in _listSrf)
            {
                var leaf=new leaf();
                listLeaf.Add(leaf);
                leaf.srf = srf as NurbsSurface;
                leaf.nU = leaf.srf.Points.CountU;
                leaf.nV = leaf.srf.Points.CountV;
                leaf.domU = leaf.srf.Domain(0);
                leaf.domV = leaf.srf.Domain(1);
                leaf.uDim = leaf.srf.OrderU;
                leaf.vDim = leaf.srf.OrderV;
                leaf.uDdim = leaf.srf.OrderU - 1;
                leaf.vDdim = leaf.srf.OrderV - 1;
                leaf.nUelem = leaf.nU - leaf.uDdim;
                leaf.nVelem = leaf.nV - leaf.vDdim;
                leaf.scaleU = (leaf.domU.T1 - leaf.domU.T0) / leaf.nUelem;
                leaf.scaleV = (leaf.domV.T1 - leaf.domV.T0) / leaf.nVelem;
                leaf.originU = leaf.domU.T0;
                leaf.originV = leaf.domV.T0;
                leaf.baseFunction = new DoubleArray[4];
                leaf.coeff = new DoubleArray[4];
                leaf.Function = new Func<double, double, double>[4];
                leaf.dFunction = new Action<double, double, double[]>[4];
                leaf.ddFunction = new Action<double, double, double[,]>[4];
                //InputGeometry input = new InputGeometry();  //temporary
                //int nNode = 11;
                //int _N = 0;
                var domainU = leaf.srf.Domain(0);
                var domainV = leaf.srf.Domain(1);
                //Find corresponding curve
                //(0,0)->(1,0)
                var curve = leaf.srf.IsoCurve(0, domainV.T0) as NurbsCurve;
                leaf.flip[0] = findCurve(leaf,ref leaf.branch[0], listBranch, curve);//bottom
                //(1,0)->(1,1)
                curve = leaf.srf.IsoCurve(1, domainU.T1) as NurbsCurve;
                leaf.flip[1] = findCurve(leaf, ref leaf.branch[1], listBranch, curve);//right
                //(1,1)->(0,1)
                curve = leaf.srf.IsoCurve(0, domainV.T1) as NurbsCurve;
                leaf.flip[2] = findCurve(leaf, ref leaf.branch[2], listBranch, curve);//top
                //(0,1)->(0,0)
                curve = leaf.srf.IsoCurve(1, domainU.T0) as NurbsCurve;
                leaf.flip[3] = findCurve(leaf, ref leaf.branch[3], listBranch, curve);//left
                
                /*
                //(0,0)->(1,0)
                double u=0, v=0;
                for (int i = 0; i < nNode-1; i++)
                {
                    u = domainU.T0 + (domainU.T1 - domainU.T0) / (nNode - 1) * i;
                    v = domainV.T0;
                    input.AddPoint(u / leaf.scaleU - leaf.originU / leaf.scaleU, v / leaf.scaleV - leaf.originV / leaf.scaleV);
                    _N++;
                    input.AddSegment(_N - 1, _N, 1);
                }
                //(1,0)->(1,1)
                for (int i = 0; i < nNode-1; i++)
                {
                    u = domainU.T1;
                    v = domainV.T0 + (domainV.T1 - domainV.T0) / (nNode - 1) * i;
                    input.AddPoint(u / leaf.scaleU - leaf.originU / leaf.scaleU, v / leaf.scaleV - leaf.originV / leaf.scaleV);
                    _N++;
                    input.AddSegment(_N - 1, _N, 2);
                }
                //(1,1)->(0,1)
                for (int i = 0; i < nNode-1; i++)
                {
                    u = domainU.T1 + (domainU.T0 - domainU.T1) / (nNode - 1) * i;
                    v = domainV.T1;
                    input.AddPoint(u / leaf.scaleU - leaf.originU / leaf.scaleU, v / leaf.scaleV - leaf.originV / leaf.scaleV);
                    _N++;
                    input.AddSegment(_N - 1, _N, 3);
                }
                //(0,1)->(0,0)
                for (int i = 0; i < nNode-1; i++)
                {
                    u = domainU.T0;
                    v = domainV.T1 + (domainV.T0 - domainV.T1) / (nNode - 1) * i;
                    input.AddPoint(u / leaf.scaleU - leaf.originU / leaf.scaleU, v / leaf.scaleV - leaf.originV / leaf.scaleV);
                    _N++;
                    if (i == nNode - 2)
                    {
                        input.AddSegment(_N - 1, 0, 4);
                    }
                    else
                    {
                        input.AddSegment(_N - 1, _N, 4);
                    }
                }

                Mesher.Mesh mesh = new Mesher.Mesh();
                mesh.Behavior.UseBoundaryMarkers = true;
                mesh.Behavior.MaxArea = Math.Pow(Math.Min(input.Bounds.Width, input.Bounds.Height) / 10d, 2);
                mesh.Behavior.Convex = false;
                mesh.Behavior.Algorithm = TriangulationAlgorithm.SweepLine;
                mesh.Behavior.ConformingDelaunay = true;
                mesh.Triangulate(input);

                mesh.Behavior.Quality = true;
                mesh.Behavior.MinAngle = 30;
                mesh.Behavior.MaxAngle = 100;

                mesh.Refine();

                mesh.Smooth();
                mesh.Smooth();
                foreach (var P in mesh.Vertices)
                {
                    if (P.Attributes == null)
                    {
                        leaf.gmesh.Vertices.Add(new Point3d(P.X, P.Y, 0));
                    }
                    else
                    {
                        leaf.gmesh.Vertices.Add(new Point3d(P.X, P.Y, P.Attributes[0]));
                    }
                }
                foreach (var tri in mesh.Triangles)
                {
                    leaf.gmesh.Faces.AddFace(tri.P0, tri.P1, tri.P2);
                }

                leaf.bbOut = new List<Edge>[4];
                leaf.triEdges = new List<Line>[4];
                foreach (var edge in mesh.Edges)
                {
                    var f = edge.Boundary;
                    if (f == 0) continue;
                    if (f < 5)
                    {
                        if (leaf.bbOut[f - 1] == null)
                        {
                            leaf.bbOut[f - 1] = new List<Edge>();
                            leaf.triEdges[f - 1] = new List<Line>();
                        }
                        leaf.bbOut[f - 1].Add(new Edge(edge.P0, edge.P1));
                        var P = mesh.Vertices.ElementAt(edge.P0);
                        var Q = mesh.Vertices.ElementAt(edge.P1);
                        leaf.triEdges[f - 1].Add(new Line(new Point3d(P.X, P.Y, 0), new Point3d(Q.X, Q.Y, 0)));
                    }
                }
                leaf.n = mesh.Vertices.Count();
                leaf.m = mesh.Edges.Count();
                leaf.r = mesh.Triangles.Count();
                int[,] lines = new int[leaf.m, 2];
                int _i = 0;
                foreach (var edge in mesh.Edges)
                {
                    lines[_i, 0] = edge.P0;
                    lines[_i, 1] = edge.P1;
                    _i++;
                }
                leaf.fixedPoints = new List<int>();
                leaf.vertices = mesh.Vertices.ToArray();
                leaf.edges = mesh.Edges.ToArray();
                leaf.triangles = mesh.Triangles.ToArray();
                foreach (var V in leaf.vertices)
                {
                    if (V.Boundary > 0) leaf.fixedPoints.Add(Array.IndexOf(leaf.vertices, V));
                }
                leaf.Laplacian = computeLaplacian(lines, leaf.n);
                leaf.shiftArray = computeShiftArray(leaf.fixedPoints, leaf.n);
                //Arranging bb
                foreach (var bbb in leaf.bbOut)
                {
                    //Look for the first end
                    Mesher.Geometry.Edge first = null;
                    bool reverse = false;
                    foreach (var bbbb in bbb)
                    {
                        int P = bbbb.P0;
                        int count = 0;
                        foreach (var bbbbb in bbb)
                        {
                            if (bbbbb.P0 == P || bbbbb.P1 == P) count++;
                        }
                        if (count == 1) { first = bbbb; break; }
                        P = bbbb.P1;
                        count = 0;
                        foreach (var bbbbb in bbb)
                        {
                            if (bbbbb.P0 == P || bbbbb.P1 == P) count++;
                        }
                        if (count == 1) { first = bbbb; reverse = true; break; }
                    }
                    bbb.Remove(first);
                    if (reverse)
                    {
                        first = new Edge(first.P1, first.P0);
                    }
                    bbb.Insert(0, first);
                    for (int i = 0; i < bbb.Count - 1; i++)
                    {
                        Mesher.Geometry.Edge next = null;
                        reverse = false;
                        int P = bbb[i].P1;
                        for (int j = i + 1; j < bbb.Count; j++)
                        {
                            if (bbb[j].P0 == P) { next = bbb[j]; break; }
                            if (bbb[j].P1 == P) { next = bbb[j]; reverse = true; break; }
                        }
                        bbb.Remove(next);
                        if (reverse) next = new Edge(next.P1, next.P0);
                        bbb.Insert(i + 1, next);
                    }
                }*/
            }
        }
    }
}
