using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;


using Mothra.UI;

using ShoNS.Array;

namespace mikity.ghComponents
{
    public partial class Mothra3 : Grasshopper.Kernel.GH_Component
    {
        public void tieBranch2(branch branch, leaf leaf)
        {
            int T0 = 0, T1 = 0;

            for (int s = 0; s < 4; s++)
            {
                if (s == 0)
                {
                    T0 = 0; T1 = (leaf.nUelem*leaf.NN) - 1;
                }
                if (s == 1)
                {
                    T0 = (leaf.nUelem*leaf.NN); T1 = (leaf.nUelem + leaf.nVelem)*leaf.NN - 1;
                }
                if (s == 2)
                {
                    T0 = (leaf.nUelem + leaf.nVelem)*leaf.NN; T1 = (leaf.nUelem * 2 + leaf.nVelem)*leaf.NN - 1;
                }
                if (s == 3)
                {
                    T0 = (leaf.nUelem * 2 + leaf.nVelem)*leaf.NN; T1 = (leaf.nUelem * 2 + leaf.nVelem * 2)*leaf.NN - 1;
                }
                if (leaf.branch[s] == branch)//s=0:bottom, s=1:right, s=2:top, s=3:left 
                {
                    int N = 0;
                    if (s == 0 || s == 2) N = leaf.nU; else N = leaf.nV;
                    if (N == branch.N)
                    {
                        for (int i = 0; i < branch.nElem*branch.NN; i++)
                        {
                            if (leaf.flip[s])
                            {
                                if (branch.left == leaf)
                                    branch.tuples[i].left = leaf.edgeTuples[T1 - i];
                                if (branch.right == leaf)
                                    branch.tuples[i].right = leaf.edgeTuples[T1 - i];
                                if (branch.target == leaf)
                                    branch.tuples[i].target = leaf.edgeTuples[T1 - i];
                            }
                            else
                            {
                                if (branch.left == leaf)
                                    branch.tuples[i].left = leaf.edgeTuples[T0 + i];
                                if (branch.right == leaf)
                                    branch.tuples[i].right = leaf.edgeTuples[T0 + i];
                                if (branch.target == leaf)
                                    branch.tuples[i].target = leaf.edgeTuples[T0 + i];
                            }
                        }
                    }
                    else { AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "cannot tie"); }
                }
            }
        }

        public class node
        {
            public double x,y,z;
            public int N;
            public List<branch> share=new List<branch>();
            public List<int> number = new List<int>();
            public int varOffset;
            public int conOffset;
            public bool compare(Point3d P)
            {
                double dx = P.X - x;
                double dy = P.Y - y;
                double dz = P.Z - z;
                if ((dx * dx + dy * dy + dz * dz) < 0.0000001) return true;
                return false;
            }
            public node()
            {
                N = 0;
                share.Clear();
                number.Clear();
            }
        }
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
            public NurbsCurve shellCrv;
            public Minilla3D.Objects.arch myArch;
            public leaf target = null;
            public leaf left=null,right=null;
            public int varOffset;
            public int conOffset;
            public int N;  //number of nodes
            public int r;  //Number of tuples.
            public int Dim;
            public int dDim;
            public int nElem;
            public int NN;  //NN tuples in one element 
            public double scaleT,originT;
            public Interval dom;
            public dl_ex[] tuples;
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
            public Minilla3D.Objects.masonry myMasonry;
            public NurbsSurface srf;
            public NurbsSurface airySrf;
            public NurbsSurface shellSrf;
            public branch[] branch = new branch[4];
            public bool[] flip = new bool[4] { false, false, false, false };
            public int  r;  //Number of tuples.
            public int nU, nV;
            public int uDim, vDim;
            public int uDdim, vDdim;
            public int nUelem;
            public int nVelem;
            public int NN;  //NN*NN tuples in one element 
            public double scaleU, scaleV, originU, originV;
            public Interval domU, domV;
            public tuple_ex[] tuples;
            public tuple_ex[] edgeTuples;
            public enum type
            {
                flat,convex
            }
            public type leafType;
        }
        public class dl_ex : Minilla3D.Elements.nurbsCurve.dl
        {
            public dl_ex(double _ot, double _t, int _index, double _lo, double _area)
                : base(_ot, _t, _index, _lo, _area)
            {
            }
            public void init(NurbsCurve C, double scaleT)
            {
                Point3d P;
                P=C.PointAt(t);
                x = P.X;
                y = P.Y;
                z = 0;
            }
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
        List<node> listNode;
        List<Point3d> a;
        List<Point3d> a2;
        List<Line> crossCyan;
        List<Line> crossMagenta;
        List<slice> listSlice;
        List<NurbsCurve> listError;
        private void init()
        {
            ready = false;
            a = new List<Point3d>();
            a2 = new List<Point3d>();
            crossCyan = new List<Line>();
            crossMagenta = new List<Line>();
            listError = new List<NurbsCurve>();
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
            pManager.AddTextParameter("listType", "lstSrfType", "list of types of surfaces", Grasshopper.Kernel.GH_ParamAccess.list);
            pManager.AddTextParameter("listType", "lstCrvType", "list of types of edge curves", Grasshopper.Kernel.GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(Grasshopper.Kernel.GH_Component.GH_OutputParamManager pManager)
        {
        }
        public override void AddedToDocument(Grasshopper.Kernel.GH_Document document)
        {
            base.AddedToDocument(document);
            myControlBox.Show();
            myControlBox.setFunctionToCompute(() => { computeF(); });
            myControlBox.setFunctionToCompute2(() => { computeG(); });
        }
        void computeG()
        {
            if (!ready) { System.Windows.Forms.MessageBox.Show("not ready"); return; }
            mosek2(listLeaf, listBranch, listNode, myControlBox.force);
        }
        bool ready = false;
        void computeF()
        {
            foreach (var leaf in listLeaf)
            {
                leaf.NN = 4;
                double area = 1d / ((double)leaf.NN) / ((double)leaf.NN);
                //setup tuples
                //internal tuples
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
                    }
                }
                //edge tuples
                leaf.edgeTuples = new tuple_ex[2 * (leaf.nUelem * leaf.NN + leaf.nVelem * leaf.NN)];
                //bottom
                for (int i = 0; i < leaf.nUelem * leaf.NN; i++)
                {
                    int uu = i, vv = 0;
                    int num = i;
                    double centerU = (uu + 0.5) / leaf.NN;
                    double centerV = (vv) / leaf.NN;
                    //element index
                    int uNum = (int)centerU;
                    int vNum = (int)centerV;
                    int index = uNum + vNum * leaf.nUelem;                            
                    //local coordinates
                    double localU = centerU - uNum;
                    double localV = centerV - vNum;
                    leaf.edgeTuples[num] = new tuple_ex(4, centerU, centerV, centerU * leaf.scaleU + leaf.originU, centerV * leaf.scaleV + leaf.originV, index, localU, localV, area);
                    leaf.edgeTuples[num].init(leaf.srf, leaf.scaleU, leaf.scaleV);
                    leaf.edgeTuples[num].dcdt = new double[2] {1,0};
                }
                //right
                for (int i = 0; i < leaf.nVelem * leaf.NN; i++)
                {
                    int uu = leaf.nUelem*leaf.NN, vv = i;
                    int num = i+leaf.nUelem*leaf.NN;
                    double centerU = (uu) / leaf.NN;
                    double centerV = (vv + 0.5) / leaf.NN;
                    //element index
                    int uNum = (int)(centerU-0.001);
                    int vNum = (int)centerV;
                    int index = uNum + vNum * leaf.nUelem;
                    //local coordinates
                    double localU = centerU - uNum;
                    double localV = centerV - vNum;
                    leaf.edgeTuples[num] = new tuple_ex(4, centerU, centerV, centerU * leaf.scaleU + leaf.originU, centerV * leaf.scaleV + leaf.originV, index, localU, localV, area);
                    leaf.edgeTuples[num].init(leaf.srf, leaf.scaleU, leaf.scaleV);
                    leaf.edgeTuples[num].dcdt = new double[2] { 0, 1 };
                }
                //top
                for (int i = 0; i < leaf.nUelem * leaf.NN; i++)
                {
                    int uu = i, vv = leaf.nVelem * leaf.NN;
                    int num = i + leaf.nUelem * leaf.NN + leaf.nVelem * leaf.NN;
                    double centerU = (uu+0.5) / leaf.NN;
                    double centerV = (vv) / leaf.NN;
                    //element index
                    int uNum = (int)centerU;
                    int vNum = (int)(centerV-0.0001);
                    int index = uNum + vNum * leaf.nUelem;
                    //local coordinates
                    double localU = centerU - uNum;
                    double localV = centerV - vNum;
                    leaf.edgeTuples[num] = new tuple_ex(4, centerU, centerV, centerU * leaf.scaleU + leaf.originU, centerV * leaf.scaleV + leaf.originV, index, localU, localV, area);
                    leaf.edgeTuples[num].init(leaf.srf, leaf.scaleU, leaf.scaleV);
                    leaf.edgeTuples[num].dcdt = new double[2] { -1, 0 };
                }
                //left
                for (int i = 0; i < leaf.nVelem * leaf.NN; i++)
                {
                    int uu = 0, vv = i;
                    int num = i + leaf.nUelem * leaf.NN*2 + leaf.nVelem * leaf.NN;
                    double centerU = (uu) / leaf.NN;
                    double centerV = (vv + 0.5) / leaf.NN;
                    //element index
                    int uNum = (int)centerU;
                    int vNum = (int)centerV;
                    int index = uNum + vNum * leaf.nUelem;
                    //local coordinates
                    double localU = centerU - uNum;
                    double localV = centerV - vNum;
                    leaf.edgeTuples[num] = new tuple_ex(4, centerU, centerV, centerU * leaf.scaleU + leaf.originU, centerV * leaf.scaleV + leaf.originV, index, localU, localV, area);
                    leaf.edgeTuples[num].init(leaf.srf, leaf.scaleU, leaf.scaleV);
                    leaf.edgeTuples[num].dcdt = new double[2] { 0, -1 };
                }
                createNurbsElements(leaf);
                double[,] x;
                x = new double[leaf.nU * leaf.nV, 3];
                Nurbs2x(leaf.srf, x);
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
                foreach (var tup in leaf.edgeTuples)
                {
                    leaf.myMasonry.elemList[tup.index].precompute(tup);
                }
            }
            foreach (var branch in listBranch)
            {
                branch.NN = 4;
                createNurbsElements(branch);
                double[,] x;
                x = new double[branch.N, 3];
                Nurbs2x(branch.crv, x);
                branch.myArch.setupNodesFromList(x);
                branch.myArch.computeGlobalCoord();
                foreach (var e in branch.myArch.elemList)
                {
                    e.precompute();
                    e.computeBaseVectors();
                }
                //branch.type
                //branch.left,right,target
                branch.tuples = new dl_ex[branch.nElem * branch.NN];
                for (int i = 0; i < branch.nElem * branch.NN; i++)
                {
                    int tt = i;
                    int num = i;
                    double centerT = (tt + 0.5) / branch.NN;
                    //element index
                    int tNum = (int)centerT;
                    int index = tNum;
                    //local coordinates
                    double localT = centerT - tNum;
                    branch.tuples[num] = new dl_ex(centerT, centerT * branch.scaleT + branch.originT, index, localT, 1d / ((double)branch.NN));
                    branch.tuples[num].init(branch.crv, branch.scaleT);
                }
                if (branch.branchType == branch.type.kink)
                {
                    tieBranch2(branch, branch.left);
                    tieBranch2(branch, branch.right);
                }
                else
                {
                    tieBranch2(branch, branch.target);
                }
                foreach (var tup in branch.tuples)
                {
                    branch.myArch.elemList[tup.index].precompute(tup);
                    if (branch.branchType == branch.type.kink)
                    {
                        branch.left.myMasonry.elemList[tup.left.index].computeGradTangent(tup.left);
                        branch.right.myMasonry.elemList[tup.right.index].computeGradTangent(tup.right);
                    }
                    else if(branch.branchType==branch.type.fix)
                    {
                        branch.target.myMasonry.elemList[tup.target.index].computeGradTangent(tup.target);
                    }
                    else
                    {
                        if (branch.slice == null) AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "need a plane");
                        else
                        {
                            var vars = branch.slice.pl.GetPlaneEquation();
                            branch.target.myMasonry.elemList[tup.target.index].computeTangent(tup.target,vars[0],vars[1],vars[2],vars[3]);
                            branch.target.myMasonry.elemList[tup.target.index].computeGradTangent(tup.target);
                        }
                    }
                }
            }
            //call mosek
            mosek1(listLeaf,listBranch,listNode,myControlBox.coeff);
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
                            double s = tuple.eigenValues[i];
                            //double s = 0.1;
                            Point3d S = new Point3d(tuple.x - tuple.eigenVectors[i][0] * s, tuple.y - tuple.eigenVectors[i][1] * s, tuple.z - tuple.eigenVectors[i][2] * s);
                            Point3d E = new Point3d(tuple.x + tuple.eigenVectors[i][0] * s, tuple.y + tuple.eigenVectors[i][1] * s, tuple.z + tuple.eigenVectors[i][2] * s);
                            Line line = new Line(S, E);
                            line.Transform(zDown);
                            crossCyan.Add(line);
                        }
                        else
                        {
                            double s = tuple.eigenValues[i];
                            //double s = 0.1;
                            Point3d S = new Point3d(tuple.x - tuple.eigenVectors[i][0] * s, tuple.y - tuple.eigenVectors[i][1] * s, tuple.z - tuple.eigenVectors[i][2] * s);
                            Point3d E = new Point3d(tuple.x + tuple.eigenVectors[i][0] * s, tuple.y + tuple.eigenVectors[i][1] * s, tuple.z + tuple.eigenVectors[i][2] * s);
                            Line line = new Line(S, E);
                            line.Transform(zDown);
                            crossMagenta.Add(line);
                        }
                    }
                }
            }
            ready = true;
            this.ExpirePreview(true);
        }
        public int ttt = 0;
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
            ttt++;
            AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "cannot find:"+ttt.ToString());
            listError.Add(curve);
            return false;
        }
        protected override void SolveInstance(Grasshopper.Kernel.IGH_DataAccess DA)
        {
            init();
            _listSrf = new List<Surface>();
            _listCrv = new List<Curve>();
            List<string> srfTypes = new List<string>();
            List<string> crvTypes = new List<string>();
            if (!DA.GetDataList(0, _listSrf)) { return; }
            if (!DA.GetDataList(1, _listCrv)) { return; }
            if (!DA.GetDataList(2, srfTypes)) { return; }
            if (!DA.GetDataList(3, crvTypes)) { return; }

            if (_listSrf.Count != srfTypes.Count) { AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "need types for surfaces"); return; }
            if (_listCrv.Count != crvTypes.Count) { AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "need types for curves"); return; }
            listSlice = new List<slice>();
            listLeaf = new List<leaf>();
            listBranch = new List<branch>();
            listNode=new List<node>();
            for (int i = 0; i < _listCrv.Count; i++)
            {
                var branch = new branch();
                branch.crv = _listCrv[i] as NurbsCurve;
                branch.N = branch.crv.Points.Count;
                branch.dom = branch.crv.Domain;
                branch.Dim= branch.crv.Order;
                branch.dDim = branch.crv.Order - 1;
                branch.nElem = branch.N - branch.dDim;
                branch.scaleT = (branch.dom.T1 - branch.dom.T0) / branch.nElem;
                branch.originT = branch.dom.T0;
                switch (crvTypes[i])
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
            myControlBox.clearSliders();

            var pl1 = new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1));
            listSlice.Add(new slice(pl1));
            int ss = 1;

            foreach (var branch in listBranch)
            {
                if (branch.branchType == branch.type.reinforce)
                {
                    //var plnew = new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1));
                    //listSliceI.Add(new slice(plnew));
                    branch.slice = listSlice[0];
                    //ssI++;
                }
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
                        return theta;
                    };
                    ss++;
                }
            }
            // Connect nodes
            foreach (var node in listNode)
            {
                node.N = 0;
                node.share.Clear();
                node.number.Clear();
            }
            foreach (var branch in listBranch)
            {
                var P = branch.crv.Points[0].Location;
                bool flag = false;
                foreach (var node in listNode)
                {
                    if (node.compare(P))
                    {
                        flag = true;
                        node.N++;
                        node.share.Add(branch);
                        node.number.Add(0);
                        break;
                    }
                }
                if (!flag)
                {
                    var newNode=new node();
                    listNode.Add(newNode);
                    newNode.N++;
                    newNode.share.Add(branch);
                    newNode.number.Add(0);
                    newNode.x = P.X;
                    newNode.y = P.Y;
                    newNode.z = P.Z;
                }
                var Q = branch.crv.Points[branch.N - 1].Location;
                flag = false;
                foreach (var node in listNode)
                {
                    if (node.compare(Q))
                    {
                        flag = true;
                        node.N++;
                        node.share.Add(branch);
                        node.number.Add(branch.N-1);
                        break;
                    }
                }
                if (!flag)
                {
                    var newNode = new node();
                    listNode.Add(newNode);
                    newNode.N++;
                    newNode.share.Add(branch);
                    newNode.number.Add(branch.N-1);
                    newNode.x = Q.X;
                    newNode.y = Q.Y;
                    newNode.z = Q.Z;
                }
            }
            for(int i=0;i<_listSrf.Count;i++)
            {
                var srf = _listSrf[i];
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
                switch (srfTypes[i])
                {
                    case "convex":
                        leaf.leafType = leaf.type.convex;
                        break;
                    case "flat":
                        leaf.leafType = leaf.type.flat;
                        break;
                    default:
                        AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "type should be either of convex or flat");
                        return;
                }

            }
        }
    }
}
