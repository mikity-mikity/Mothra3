using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

using Mesher;
using Mesher.Geometry;
using Mesher.Tools;

using Mothra.UI;

using ShoNS.Array;

namespace mikity.ghComponents
{
    public partial class Mothra3 : Grasshopper.Kernel.GH_Component
    {
        public class leaf
        {
            public List<Mesher.Geometry.Edge>[] bbOut;
            public List<Line>[] triEdges;
            public List<Line> result;
            public Minilla3D.Objects.masonry myMasonry;

            public NurbsSurface srf;
            public NurbsSurface airySrf;
            public Rhino.Geometry.Mesh gmesh = new Rhino.Geometry.Mesh();
            public SparseDoubleArray Laplacian;
            public SparseDoubleArray shiftArray;
            public int n, m, r;  //Number of vertices, edges and triangles.
            public int nU, nV;
            public int uDim, vDim;
            public int uDdim, vDdim;
            public int nUelem;
            public int nVelem;
            public double scaleU, scaleV, originU, originV;
            public Interval domU, domV;
            public Mesher.Data.Vertex[] vertices;
            public Mesher.Geometry.Edge[] edges;
            public Mesher.Data.Triangle[] triangles;
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
        List<leaf> listLeaf;
        List<Point3d> a;
        List<Line> crossCyan;
        List<Line> crossMagenta;
        List<Line> f;

        int lastComputed = -1;        

        private void init()
        {
            a = new List<Point3d>();
            f = new List<Line>();
            lastComputed = -1;
            crossCyan = new List<Line>();
            crossMagenta = new List<Line>();
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
                int N=4;

                foreach (var leaf in listLeaf)
                {
                    leaf.tuples = new tuple_ex[leaf.r];
                    for (int i = 0; i < leaf.r; i++)
                    {
                        var tri = leaf.triangles[i];
                        var A = tri.GetVertex(0);
                        var B = tri.GetVertex(1);
                        var C = tri.GetVertex(2);
                        double centerU = (A.X + B.X + C.X) / 3d;
                        double centerV = (A.Y + B.Y + C.Y) / 3d;
                        //element index
                        int uNum = (int)centerU;
                        int vNum = (int)centerV;
                        int index = uNum + vNum * leaf.nUelem;
                        //local coordinates
                        double localU = centerU - uNum;
                        double localV = centerV - vNum;

                        leaf.tuples[i] = new tuple_ex(4, centerU, centerV, centerU * leaf.scaleU + leaf.originU, centerV * leaf.scaleV + leaf.originV, index, localU, localV, tri.Area);
                        leaf.tuples[i].init(leaf.srf, leaf.scaleU, leaf.scaleV);
                        for (int s = 0; s < N; s++)
                        {
                            leaf.tuples[i].f[s] = leaf.Function[s](centerU, centerV);
                            leaf.dFunction[s](centerU, centerV, leaf.tuples[i].df[s]);
                            leaf.ddFunction[s](centerU, centerV, leaf.tuples[i].ddf[s]);
                        }
                        //computes kernel...
                        double squaredLength = 0;
                        for (int s = 0; s < N; s++)
                        {
                            squaredLength += leaf.tuples[i].f[s] * leaf.tuples[i].f[s];
                        }
                        for (int s = 0; s < N; s++)
                        {
                            for (int t = 0; t < N; t++)
                            {
                                leaf.tuples[i].kernel[s, t] = -leaf.tuples[i].f[s] * leaf.tuples[i].f[t] / squaredLength;
                                if (s == t) leaf.tuples[i].kernel[s, t] += 1d;
                            }
                        }
                        double norm = Math.Sqrt(squaredLength);
                        //compute normalized base function
                        for (int s = 0; s < N; s++)
                        {
                            leaf.tuples[i].nf[s] = leaf.tuples[i].f[s] / norm;
                        }
                        // compute normalized first derivative
                        for (int v = 0; v < 2; v++)
                        {
                            for (int s = 0; s < N; s++)
                            {
                                double val = 0;
                                for (int t = 0; t < N; t++)
                                {
                                    val += leaf.tuples[i].df[s][v] * leaf.tuples[i].kernel[s, t] / norm;
                                }
                                leaf.tuples[i].ndf[s][v] = val;
                            }
                        }

                        //compute normalized second derivative
                        for (int v = 0; v < 2; v++)
                        {
                            for (int w = 0; w < 2; w++)
                            {
                                for (int s = 0; s < N; s++)
                                {
                                    double val = 0;
                                    for (int t = 0; t < N; t++)
                                    {
                                        val += leaf.tuples[i].ddf[s][v, w] * leaf.tuples[i].kernel[s, t] / norm;
                                        val -= 3 * leaf.tuples[i].df[s][v] * leaf.tuples[i].df[s][w] * leaf.tuples[i].kernel[s, t] / norm / squaredLength;
                                    }
                                    leaf.tuples[i].nddf[s][v, w] = val;
                                }
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
                    //call mosek
                    mosek1(leaf);
                }
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
        protected override void SolveInstance(Grasshopper.Kernel.IGH_DataAccess DA)
        {
            init();
            _listSrf = new List<Surface>();
            if (!DA.GetDataList(0, _listSrf)) { return; }
            listLeaf = new List<leaf>();
            myControlBox.setNumF(4);
            myControlBox.setFunctionToCompute(() =>
            {
                if (lastComputed == 3) return;
                lastComputed++;
                computeBaseFunction(lastComputed);
                this.ExpirePreview(true);
                myControlBox.EnableRadio(lastComputed, (i) => { resultToPreview(i); this.ExpirePreview(true); });
            }
                );

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
                InputGeometry input = new InputGeometry();  //temporary
                int nNode = 11;
                int _N = 0;
                var domainU = leaf.srf.Domain(0);
                var domainV = leaf.srf.Domain(1);
                //(0,0)->(1,0)
                //var curve = leaf.srf.IsoCurve(0, domainV.T0);
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
                //curve = leaf.srf.IsoCurve(1, domainU.T1);
                for (int i = 0; i < nNode-1; i++)
                {
                    u = domainU.T1;
                    v = domainV.T0 + (domainV.T1 - domainV.T0) / (nNode - 1) * i;
                    input.AddPoint(u / leaf.scaleU - leaf.originU / leaf.scaleU, v / leaf.scaleV - leaf.originV / leaf.scaleV);
                    _N++;
                    input.AddSegment(_N - 1, _N, 2);
                }
                //(1,1)->(0,1)
                //curve = leaf.srf.IsoCurve(0, domainV.T1);
                for (int i = 0; i < nNode-1; i++)
                {
                    u = domainU.T1 + (domainU.T0 - domainU.T1) / (nNode - 1) * i;
                    v = domainV.T1;
                    input.AddPoint(u / leaf.scaleU - leaf.originU / leaf.scaleU, v / leaf.scaleV - leaf.originV / leaf.scaleV);
                    _N++;
                    input.AddSegment(_N - 1, _N, 3);
                }
                //(0,1)->(0,0)
                //curve = leaf.srf.IsoCurve(1, domainU.T0);
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
                foreach (var l in input.Segments)
                {
                    var P = input.Points.ElementAt(l.P0);
                    var Q = input.Points.ElementAt(l.P1);
                    f.Add(new Line(new Point3d(P.X, P.Y, 0), new Point3d(Q.X, Q.Y, 0)));
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
                }
            }
/*          for (int i = 0; i < nUelem; i++)
            {
                for (int j = 0; j < nVelem; j++)
                {
                    Point3d P;
                    double u = domU.T0 + (domU.T1 - domU.T0) / nUelem * (i + 0.3);
                    double v = domV.T0 + (domV.T1 - domV.T0) / nVelem * (j + 0.3);
                    P = face.PointAt(u, v);
                    dd2.Add(P);
                    u = domU.T0 + (domU.T1 - domU.T0) / nUelem * (i + 0.7);
                    v = domV.T0 + (domV.T1 - domV.T0) / nVelem * (j + 0.3);
                    P = face.PointAt(u, v);
                    dd2.Add(P);
                    u = domU.T0 + (domU.T1 - domU.T0) / nUelem * (i + 0.3);
                    v = domV.T0 + (domV.T1 - domV.T0) / nVelem * (j + 0.7);
                    P = face.PointAt(u, v);
                    dd2.Add(P);
                    u = domU.T0 + (domU.T1 - domU.T0) / nUelem * (i + 0.7);
                    v = domV.T0 + (domV.T1 - domV.T0) / nVelem * (j + 0.7);
                    P = face.PointAt(u, v);
                    dd2.Add(P);
                }
            }
            int nPt = 50;
            int nPt2 = 30;
            for (int i = 0; i <= nPt; i++)
            {
                double u = domU[0] + (domU[1] - domU[0]) / ((double)nPt) * i;
                var C = face.TrimAwareIsoCurve(1, u);
                foreach (var curve in C)
                {
                    c.Add(curve);
                }
            }
            for (int i = 0; i <= nPt; i++)
            {
                double v = domV[0] + (domV[1] - domV[0]) / ((double)nPt) * i;
                var D = face.TrimAwareIsoCurve(0, v);
                foreach (var curve in D)
                {
                    c.Add(curve);
                }
            }
            for (int i = 0; i <= nPt; i++)
            {
                for (int j = 0; j <= nPt; j++)
                {
                    double u = domU[0] + (domU[1] - domU[0]) / ((double)nPt) * i;
                    double v = domV[0] + (domV[1] - domV[0]) / ((double)nPt) * j;
                    Point3d P;
                    Vector3d[] tmp;
                    var flag=face.Evaluate(u,v,0,out P,out tmp);
                    var C = face.TrimAwareIsoCurve(0, v);
                    var D = face.TrimAwareIsoCurve(1, u);
                    bool flagU=false,flagV=false;
                    if (C.Length > 0)
                    {
                        foreach (var s in C)
                        {
                            var domC = s.Domain;
                            double f;
                            s.ClosestPoint(P, out f);
                            var P2=s.PointAt(f);
                            if((P-P2).Length<0.00000001) flagU = true;
                        }
                    }
                    if (D.Length > 0)
                    {
                        foreach (var s in D)
                        {
                            var domD = s.Domain;
                            double f;
                            s.ClosestPoint(P, out f);
                            var P2 = s.PointAt(f);
                            if ((P - P2).Length < 0.00000001) flagV = true;
                        }
                    }
                    if (flagU && flagV)
                    {
                        a.Add(P);
                        a2.Add(new Point3d(u / scaleU - originU / scaleU, v / scaleV - originV / scaleV, 0));
                    }
                    else
                    {
                        b.Add(P);
                        b2.Add(new Point3d(u / scaleU - originU / scaleU, v / scaleV - originV / scaleV, 0));
                    }
                }
            }
*/
        }
    }
}
