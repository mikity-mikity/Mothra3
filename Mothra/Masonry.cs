﻿using ShoNS.Array;
using Rhino.Geometry;
using System.IO;
using System.Reflection;
using Minilla3D.Elements;
using Mothra.UI;
namespace mikity.ghComponents
{


    //public class Mothra : Grasshopper.Kernel.GH_Component
    //{
/*        Transform[] XZ;
        Transform[] XZ2;
        ControlBox myControlBox = new ControlBox();
        
        Rhino.Geometry.NurbsSurface inputNurbs, xyNurbs;
        NurbsSurface[] airyNurbs;
        NurbsSurface[] outputNurbs;
        Minilla3D.Objects.masonry[] myMasonry;
        double[,] x;
        double[,] Force=null;
        bool initialized = false;
        int nU, nV;
        Rhino.Geometry.Mesh alternativeMesh = null;
        List<int> boundaryIndex=null;
        bool __update = false;
        System.Windows.Forms.Timer timer=null;
 * */

        /*        protected override System.Drawing.Bitmap Icon
                {
                    get
                    {
                        //現在のコードを実行しているAssemblyを取得
                        System.Reflection.Assembly myAssembly =
                            System.Reflection.Assembly.GetExecutingAssembly();

                        System.IO.Stream st = myAssembly.GetManifestResourceStream("mikity.ghComponents.icons.icon46.bmp");
                        //指定されたマニフェストリソースを読み込む
                        System.Drawing.Bitmap bmp = new System.Drawing.Bitmap(st);
                        return bmp;
                    }
                }
        */
        /*public Mothra()
            : base("Mothra", "Mothra", "Mothra", "Kapybara3D", "Computation")
        {
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("13a84560-1b45-401b-bad8-b8487a55d51e"); }
        }

        protected override void RegisterInputParams(Grasshopper.Kernel.GH_Component.GH_InputParamManager pManager)
        {
            //pManager.AddGenericParameter("Surface", "S", "inputSurface", Grasshopper.Kernel.GH_ParamAccess.item);
            //pManager.AddBooleanParameter("Boundary", "b", "Boundary", Grasshopper.Kernel.GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(Grasshopper.Kernel.GH_Component.GH_OutputParamManager pManager)
        {
        }
        public override void AddedToDocument(Grasshopper.Kernel.GH_Document document)
        {
            base.AddedToDocument(document);
            myControlBox.Show();
            myControlBox.setCompute(() =>
            {
                if (__update)
                {
                    __update = false;
                    update();
                    this.ExpirePreview(true);
                }
            });
        }
*/
        /*bool isBoundary(int n)
        {
            for (int i = 0; i < boundaryIndex.Count(); i++)
            {
                if (boundaryIndex[i] == n)
                {
                    return true;
                }
            }
            return false;
        }*/
        //public Rhino.Geometry.Plane[] cuttingPlane=new Plane[4];
        /*public void update()
        {
            Nurbs2x(airyNurbs[0], x);
            double norm = 0;
            double minZ = Double.MaxValue;
            for (int i = 0; i < nU * nV; i++)
            {
                if (x[i, 2] < minZ) minZ = x[i, 2];
            }
            for (int i = 0; i < nU * nV; i++)
            {
                x[i, 2]-=minZ;
            }
            for (int i = 0; i < nU * nV; i++)
            {
                norm += x[i, 2] * x[i, 2];
            }
            norm = Math.Sqrt(norm);
            for (int i = 0; i < nU * nV; i++)
            {
                x[i, 2] /= norm/500;
            }            
            x2Nurbs(x, airyNurbs[0]);
            int nConstraints = 0;
            int nConstraints2 = 0;
            ShoNS.Array.SparseDoubleArray jacobian=null;
            ShoNS.Array.DoubleArray residual=null;
            ShoNS.Array.SparseDoubleArray jacobianH=null;
            foreach (var v in myMasonry)
            {
                v.setupNodesFromList(x);
                v.precompute();
                v.computeGlobalCoord();
                nConstraints2 = v.totalNumberOfIconst(boundary);
                nConstraints = v.totalNumberOfBconst();
                jacobian = new SparseDoubleArray(nConstraints, nU * nV);
                residual = new DoubleArray(nConstraints, 1);
            
                jacobianH = new SparseDoubleArray(nConstraints2, nU * nV);
                v.GetJacobianOfCurvature(jacobianH, boundary);
                v.getJacobian(jacobian);
            }
            try
            {
                ////////////////////////////GUROBI///////////////////
                GRBEnv env = new GRBEnv("c:\\qcp.log");
                env.Set(GRB.IntParam.Threads, 4);
                //env.Set(GRB.IntParam.DualReductions, 0);
                GRBModel model = new GRBModel(env);
                double[] lb = new double[nU * nV];
                double[] ub = new double[nU * nV];
                char[] type = new char[nU * nV];
                string[] name = new string[nU * nV];
                int count = 0;
                for (int j = 0; j < nV; j++)
                {
                    for (int i = 0; i < nU; i++, count++)
                    {
                        lb[count] = -GRB.INFINITY;
                        ub[count] = GRB.INFINITY;
                        type[count] = GRB.CONTINUOUS;
                        name[count] = "N[" + i.ToString("g") + "," + j.ToString("g") + "]";
                    }
                }
                lb[0] = 500;
                ub[0] = 500;
                lb[nU - 1] = 500;
                ub[nU - 1] = 500;
                lb[nU * nV - nU] = 500;
                ub[nU * nV - nU] = 500;
                lb[nU * nV - 1] = 500;
                ub[nU * nV - 1] = 500;

                GRBVar[] vars = model.AddVars(lb, ub, null, type, name);
                model.Update();
                GRBVar[][] planeVars=null;
                
                
                lb = new double[nConstraints2];
                ub = new double[nConstraints2];
                type = new char[nConstraints2];
                name = new string[nConstraints2];

                for (int j = 0; j < nConstraints2; j+=3)
                {
                    lb[j] = 0;
                    ub[j] = GRB.INFINITY;
                    type[j] = GRB.CONTINUOUS;
                    name[j] = "P" + j.ToString("g") + "-" + "phi_{0,0}";
                    lb[j + 1] = 0;
                    ub[j+1] = GRB.INFINITY;
                    type[j+1] = GRB.CONTINUOUS;
                    name[j+1] = "P" + j.ToString("g") + "-" + "phi_{1,1}";
                    lb[j + 2] = -GRB.INFINITY;
                    ub[j + 2] = GRB.INFINITY;
                    type[j+2] = GRB.CONTINUOUS;
                    name[j + 2] = "P" + j.ToString("g") + "-" + "phi_{0,1}";
                }
                GRBVar[] phis = model.AddVars(lb, ub, null, type, name);

                model.Update();
                

                //Add Hessian entries to variables
                GRBLinExpr[] exprs = new GRBLinExpr[nConstraints2];
                double[] rhs = new double[nConstraints2];
                char[] senses = new char[nConstraints2];

                for (int i = 0; i < nConstraints2; i++)
                {
                    exprs[i] = new GRBLinExpr();
                    rhs[i] = 0;
                    senses[i] = GRB.EQUAL;
                }
                foreach (var e in jacobianH.Elements)
                {
                    exprs[e.Row].AddTerm(e.Value, vars[e.Col]);
                }
                for (int i = 0; i < nConstraints2; i++)
                {
                    exprs[i].AddTerm(-1, phis[i]);
                }
                model.AddConstrs(exprs, senses, rhs, null);

                GRBQuadExpr obj = new GRBQuadExpr();

                model.Update();
                //Quadratic constraints  detH>0
                for (int i = 0; i < nConstraints2; i+=3)
                {
                    GRBQuadExpr Qlhs = new GRBQuadExpr(phis[i +2] * phis[i+ 2]);
                    GRBQuadExpr Qrhs = new GRBQuadExpr(phis[i +0] * phis[i + 1]);
                    model.AddQConstr(Qlhs, GRB.LESS_EQUAL, Qrhs, null);
                }
                
                if (!boundary)
                {
                    //Boundary variable
                    lb = new double[nConstraints];
                    ub = new double[nConstraints];
                    type = new char[nConstraints];
                    name = new string[nConstraints];
                    for (int j = 0; j < nConstraints; j++)
                    {
                        lb[j] = -100;// -GRB.INFINITY;
                        ub[j] = 100;// GRB.INFINITY;
                        type[j] = GRB.CONTINUOUS;
                        name[j] = "B" + j.ToString("g");
                    }
                    GRBVar[] bVars = model.AddVars(lb, ub, null, type, name);
                    model.Update();
                    GRBLinExpr[] bConds = new GRBLinExpr[nConstraints];
                    double[] bRhs = new double[nConstraints];
                    char[] bSenses = new char[nConstraints];
                    for (int i = 0; i < nConstraints; i++)
                    {
                        bConds[i] = new GRBLinExpr();
                        bRhs[i] = 0;
                        bSenses[i] = GRB.EQUAL;
                    }
                    for (int i = 0; i < nConstraints; i++)
                    {
                        for (int j = 0; j < nU * nV; j++)
                        {
                            if (jacobian[i, j] != 0)
                            {
                                bConds[i].AddTerm(jacobian[i, j], vars[j]);
                            }
                        }
                        bConds[i].AddTerm(-1, bVars[i]);
                    }
                    model.AddConstrs(bConds, bSenses, bRhs, null);
                    //Objective
                    for (int i = 0; i < nConstraints; i++)
                    {
                        obj.AddTerm(1, bVars[i], bVars[i]);
                    }
                    model.SetObjective(obj);
                    model.Optimize();
                    switch (model.Get(GRB.IntAttr.Status))
                    {
                        case GRB.Status.OPTIMAL:
                            //System.Windows.Forms.MessageBox.Show("Congraturation");
                            break;
                        case GRB.Status.UNBOUNDED:
                            System.Windows.Forms.MessageBox.Show("UNBOUNDED");
                            break;
                        case GRB.Status.CUTOFF:
                            System.Windows.Forms.MessageBox.Show("CUTOFF");
                            break;
                        case GRB.Status.INF_OR_UNBD:
                            System.Windows.Forms.MessageBox.Show("INF_OR_UNBD");
                            break;
                        case GRB.Status.INFEASIBLE:
                            System.Windows.Forms.MessageBox.Show("INFEASIBLE");
                            break;
                        case GRB.Status.INPROGRESS:
                            System.Windows.Forms.MessageBox.Show("INPROGRESS");
                            break;
                        case GRB.Status.ITERATION_LIMIT:
                            System.Windows.Forms.MessageBox.Show("ITERATION_LIMIT");
                            break;
                        case GRB.Status.INTERRUPTED:
                            System.Windows.Forms.MessageBox.Show("INTERRUPTED");
                            break;
                        case GRB.Status.SUBOPTIMAL:
                            System.Windows.Forms.MessageBox.Show("SUBOPTIMAL");
                            break;
                        case GRB.Status.SOLUTION_LIMIT:
                            System.Windows.Forms.MessageBox.Show("SOLUTION_LIMIT");
                            break;
                        case GRB.Status.NODE_LIMIT:
                            System.Windows.Forms.MessageBox.Show("NODE_LIMIT");
                            break;
                        default:
                            System.Windows.Forms.MessageBox.Show("Something elese");
                            break;

                    }
                    for (int j = 0; j < nU * nV; j++)
                    {
                        x[j, 2] = vars[j].Get(GRB.DoubleAttr.X);
                    }
                }
                else
                {
                    //Plane variables
                    planeVars = new GRBVar[4][];
                    for (int n = 0; n < 4; n++)
                    {
                        //z(x,y)=-D-Ax-By
                        type = new char[3];
                        name = new string[3];
                        lb = new double[3];
                        ub=new double[3];
                        for (int i = 0; i < 3; i++)
                        {
                            type[i] = GRB.CONTINUOUS;
                            lb[i] = -GRB.INFINITY;
                            ub[i] = GRB.INFINITY;
                        }
                        name[0] = "A" + n.ToString("g");
                        name[1] = "B" + n.ToString("g");
                        name[2] = "D" + n.ToString("g");
                        planeVars[n] = model.AddVars(lb,ub, null, type, name);
                    }
                    model.Update();
                    //GRBQuadExpr obj = new GRBQuadExpr();

                    //Coordinate variables of border integratingpoints
                    var borderVars = new List<GRBVar>[4];
                    var slackVars = new List<GRBVar>[4];
                    borderVars[0] = new List<GRBVar>();
                    borderVars[1] = new List<GRBVar>();
                    borderVars[2] = new List<GRBVar>();
                    borderVars[3] = new List<GRBVar>();
                    slackVars[0] = new List<GRBVar>();
                    slackVars[1] = new List<GRBVar>();
                    slackVars[2] = new List<GRBVar>();
                    slackVars[3] = new List<GRBVar>();
                    var tE = myMasonry[0].topEdge;
                    var lE = myMasonry[0].leftEdge;
                    var rE = myMasonry[0].rightEdge;
                    var bE = myMasonry[0].bottomEdge;
                    nConstraints=myMasonry[0].totalNumbrOfTopEdgeIntPoint();
                    jacobian = new SparseDoubleArray(nConstraints, nU * nV);
                    foreach (var v in myMasonry)
                    {
                        v.GetJacobianOfTopEdge(jacobian);
                    }
                    count = 0;
                    for (int i = 0; i < tE.Count; i++)
                    {
                        var e = tE[i];
                        for (int j = 0; j < e.nIntPoint; j++,count++)
                        {
                            borderVars[0].Add(model.AddVar(-GRB.INFINITY, GRB.INFINITY, 0, GRB.CONTINUOUS, "topZ" + i.ToString() + "," + j.ToString()));
                            slackVars[0].Add(model.AddVar(-GRB.INFINITY,GRB.INFINITY,0,GRB.CONTINUOUS,"slack"));
                            model.Update();
                            var con = new GRBLinExpr();
                            var node=e.getIntPoint(j);
                            con.AddTerm(1, borderVars[0][count]);   //z
                            con.AddTerm(1, planeVars[0][2]);        //D
                            con.AddTerm(node[0], planeVars[0][0]);  //Ax
                            con.AddTerm(node[1], planeVars[0][1]);  //By
                            model.AddConstr(con, GRB.EQUAL, 0, null);
                            
                            var con2 = new GRBLinExpr();
                            con2.AddTerm(-1, slackVars[0][count]);
                            for (int k = 0; k < nU * nV; k++)
                            {
                                if (jacobian[count, k] != 0)
                                {
                                    con2.AddTerm(jacobian[count, k], vars[k]);
                                }
                            }
                            model.AddConstr(con2, GRB.EQUAL, 0, null);
                            obj.AddTerm(1, slackVars[0][count], slackVars[0][count]);
                            obj.AddTerm(-2, slackVars[0][count], borderVars[0][count]);
                            obj.AddTerm(1, borderVars[0][count], borderVars[0][count]);
                        }
                    }
                    
                    nConstraints = myMasonry[0].totalNumbrOfBottomEdgeIntPoint();
                    jacobian = new SparseDoubleArray(nConstraints, nU * nV);
                    foreach (var v in myMasonry)
                    {
                        v.GetJacobianOfBottomEdge(jacobian);
                    }
                    count = 0;
                    for (int i = 0; i < bE.Count; i++)
                    {
                        var e = bE[i];
                        for (int j = 0; j < e.nIntPoint; j++, count++)
                        {
                            borderVars[1].Add(model.AddVar(-GRB.INFINITY, GRB.INFINITY, 0, GRB.CONTINUOUS, "topZ" + i.ToString() + "," + j.ToString()));
                            slackVars[1].Add(model.AddVar(-GRB.INFINITY,GRB.INFINITY,0,GRB.CONTINUOUS,"slack"));
                            model.Update();
                            var con = new GRBLinExpr();
                            var node=e.getIntPoint(j);
                            con.AddTerm(1, borderVars[1][count]);   //z
                            con.AddTerm(1, planeVars[1][2]);        //D
                            con.AddTerm(node[0], planeVars[1][0]);  //Ax
                            con.AddTerm(node[1], planeVars[1][1]);  //By
                            model.AddConstr(con, GRB.EQUAL, 0, null);
                            
                            var con2 = new GRBLinExpr();
                            con2.AddTerm(-1, slackVars[1][count]);
                            for (int k = 0; k < nU * nV; k++)
                            {
                                if (jacobian[count, k] != 0)
                                {
                                    con2.AddTerm(jacobian[count, k], vars[k]);
                                }
                            }
                            model.AddConstr(con2, GRB.EQUAL, 0, null);
                            obj.AddTerm(1, slackVars[1][count], slackVars[1][count]);
                            obj.AddTerm(-2, slackVars[1][count], borderVars[1][count]);
                            obj.AddTerm(1, borderVars[1][count], borderVars[1][count]);
                        }
                    }
                    
                    nConstraints = myMasonry[0].totalNumbrOfLeftEdgeIntPoint();
                    jacobian = new SparseDoubleArray(nConstraints, nU * nV);
                    foreach (var v in myMasonry)
                    {
                        v.GetJacobianOfLeftEdge(jacobian);
                    }
                    count = 0;
                    for (int i = 0; i < lE.Count; i++)
                    {
                        var e = lE[i];
                        for (int j = 0; j < e.nIntPoint; j++, count++)
                        {
                            borderVars[2].Add(model.AddVar(-GRB.INFINITY, GRB.INFINITY, 0, GRB.CONTINUOUS, "topZ" + i.ToString() + "," + j.ToString()));
                            slackVars[2].Add(model.AddVar(-GRB.INFINITY,GRB.INFINITY,0,GRB.CONTINUOUS,"slack"));
                            model.Update();
                            var con = new GRBLinExpr();
                            var node=e.getIntPoint(j);
                            con.AddTerm(1, borderVars[2][count]);   //z
                            con.AddTerm(1, planeVars[2][2]);        //D
                            con.AddTerm(node[0], planeVars[2][0]);  //Ax
                            con.AddTerm(node[1], planeVars[2][1]);  //By
                            model.AddConstr(con, GRB.EQUAL, 0, null);
                            
                            var con2 = new GRBLinExpr();
                            con2.AddTerm(-1, slackVars[2][count]);
                            for (int k = 0; k < nU * nV; k++)
                            {
                                if (jacobian[count, k] != 0)
                                {
                                    con2.AddTerm(jacobian[count, k], vars[k]);
                                }
                            }
                            model.AddConstr(con2, GRB.EQUAL, 0, null);
                            obj.AddTerm(1, slackVars[2][count], slackVars[2][count]);
                            obj.AddTerm(-2, slackVars[2][count], borderVars[2][count]);
                            obj.AddTerm(1, borderVars[2][count], borderVars[2][count]);
                        }
                    }
                    
                    
                    nConstraints = myMasonry[0].totalNumbrOfRightEdgeIntPoint();
                    jacobian = new SparseDoubleArray(nConstraints, nU * nV);
                    foreach (var v in myMasonry)
                    {
                        v.GetJacobianOfRightEdge(jacobian);
                    }
                    count = 0;
                    for (int i = 0; i < rE.Count; i++)
                    {
                        var e = rE[i];
                        for (int j = 0; j < e.nIntPoint; j++, count++)
                        {
                            borderVars[3].Add(model.AddVar(-GRB.INFINITY, GRB.INFINITY, 0, GRB.CONTINUOUS, "topZ" + i.ToString() + "," + j.ToString()));
                            slackVars[3].Add(model.AddVar(-GRB.INFINITY,GRB.INFINITY,0,GRB.CONTINUOUS,"slack"));
                            model.Update();
                            var con = new GRBLinExpr();
                            var node=e.getIntPoint(j);
                            con.AddTerm(1, borderVars[3][count]);   //z
                            con.AddTerm(1, planeVars[3][2]);        //D
                            con.AddTerm(node[0], planeVars[3][0]);  //Ax
                            con.AddTerm(node[1], planeVars[3][1]);  //By
                            model.AddConstr(con, GRB.EQUAL, 0, null);
                            
                            var con2 = new GRBLinExpr();
                            con2.AddTerm(-1, slackVars[3][count]);
                            for (int k = 0; k < nU * nV; k++)
                            {
                                if (jacobian[count, k] != 0)
                                {
                                    con2.AddTerm(jacobian[count, k], vars[k]);
                                }
                            }
                            model.AddConstr(con2, GRB.EQUAL, 0, null);
                            obj.AddTerm(1, slackVars[3][count], slackVars[3][count]);
                            obj.AddTerm(-2, slackVars[3][count], borderVars[3][count]);
                            obj.AddTerm(1, borderVars[3][count], borderVars[3][count]);

                        }
                    }
                    
                    //Quadratic constraints  detH>0
                    
                    model.SetObjective(obj);
                    
                    model.Optimize();
                    switch (model.Get(GRB.IntAttr.Status))
                    {
                        case GRB.Status.OPTIMAL:
                            //
                            break;
                        case GRB.Status.UNBOUNDED:
                            System.Windows.Forms.MessageBox.Show("UNBOUNDED");
                            break;
                        case GRB.Status.CUTOFF:
                            System.Windows.Forms.MessageBox.Show("CUTOFF");
                            break;
                        case GRB.Status.INF_OR_UNBD:
                            System.Windows.Forms.MessageBox.Show("INF_OR_UNBD");
                            break;
                        case GRB.Status.INFEASIBLE:
                            System.Windows.Forms.MessageBox.Show("INFEASIBLE");
                            break;
                        case GRB.Status.INPROGRESS:
                            System.Windows.Forms.MessageBox.Show("INPROGRESS");
                            break;
                        case GRB.Status.ITERATION_LIMIT:
                            System.Windows.Forms.MessageBox.Show("ITERATION_LIMIT");
                            break;
                        case GRB.Status.INTERRUPTED:
                            System.Windows.Forms.MessageBox.Show("INTERRUPTED");
                            break;
                        case GRB.Status.SUBOPTIMAL:
                            System.Windows.Forms.MessageBox.Show("SUBOPTIMAL");
                            break;
                        case GRB.Status.SOLUTION_LIMIT:
                            System.Windows.Forms.MessageBox.Show("SOLUTION_LIMIT");
                            break;
                        case GRB.Status.NODE_LIMIT:
                            System.Windows.Forms.MessageBox.Show("NODE_LIMIT");
                            break;
                        default:
                            System.Windows.Forms.MessageBox.Show("Something elese");
                            break;

                    }
                }
                int count2 = 0;
                for (int s = 1; s < nV - 1; s++)
                {
                    for (int t = 1; t < nU - 1; t++,count2++)
                    {
                        lb = new double[nU * nV];
                        ub = new double[nU * nV];
                        for (int i = 0; i < nV; i++)
                        {
                            for (int j = 0; j < nU; j++)
                            {
                                ub[j + i * nU] = -GRB.INFINITY;
                                ub[j + i * nU] = GRB.INFINITY;
                            }
                        }
                        lb[0] = 500;
                        ub[0] = 500;
                        lb[nU - 1] = 500;
                        ub[nU - 1] = 500;
                        lb[nU * nV - nU] = 500;
                        ub[nU * nV - nU] = 500;
                        lb[nU * nV - 1] = 500;
                        ub[nU * nV - 1] = 500;

                        lb[t + s * nU] = 0;
                        ub[t + s * nU] = 0;
                        model.Set(GRB.DoubleAttr.UB, vars, ub);
                        model.Set(GRB.DoubleAttr.LB, vars, lb);
                        model.Optimize();
                        var f=model.Get(GRB.DoubleAttr.X, vars);
                        for (int j = 0; j < nU * nV; j++)
                        {
                            x[j, 2] = f[j];
                        }
                        x2Nurbs(x, airyNurbs[count2]);
                        if (boundary)
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                double AA = planeVars[i][0].Get(GRB.DoubleAttr.X);
                                double BB = planeVars[i][1].Get(GRB.DoubleAttr.X);
                                double DD = planeVars[i][2].Get(GRB.DoubleAttr.X);
                                double Norm = Math.Sqrt(AA * AA + BB * BB + 1);
                                cuttingPlane[i] = new Plane(AA / Norm, BB / Norm, 1.0 / Norm, DD / Norm);
                                switch (i)
                                {
                                    case 0:
                                        foreach (var e in myMasonry[count2].topEdge)
                                        {
                                            e.setPlane(AA, BB, 1, DD);
                                        }
                                        break;
                                    case 1:
                                        foreach (var e in myMasonry[count2].bottomEdge)
                                        {
                                            e.setPlane(AA, BB, 1, DD);
                                        }
                                        break;
                                    case 2:
                                        foreach (var e in myMasonry[count2].leftEdge)
                                        {
                                            e.setPlane(AA, BB, 1, DD);
                                        }
                                        break;
                                    case 3:
                                        foreach (var e in myMasonry[count2].rightEdge)
                                        {
                                            e.setPlane(AA, BB, 1, DD);
                                        }
                                        break;
                                }

                            }
                        }
                    }
                }
                model.Dispose();
                env.Dispose();

            }
            catch (GRBException e)
            {
                AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "GUROBI ERROR!" + e.ErrorCode + ". " + e.Message);
                return;
            }
            for (int s = 0; s < (nU - 2) * (nV - 2); s++)
            {
                Nurbs2x(airyNurbs[s],x);
                myMasonry[s].setupNodesFromList(x);
                myMasonry[s].precompute();
                myMasonry[s].computeAiryFunction();
                myMasonry[s].giveEdgeTension(0);
                if (boundary)
                {
                    myMasonry[s].computeAngle();
                }
                Force = new double[nU * nV, 3];

                //Solve
                myMasonry[s].computeEigenVectors();
                Nurbs2x(xyNurbs, x);
                myMasonry[s].setupNodesFromList(x);
                myMasonry[s].computeGlobalCoord();
                ShoNS.Array.SparseDoubleArray hess = new SparseDoubleArray(nU * nV * 3, nU * nV * 3);
            
                myMasonry[s].computeHessian();
                myMasonry[s].getHessian(hess);


                Nurbs2x(xyNurbs, x);
                int nParticles = nU * nV;
                //Current configuration
                var origX = DoubleArray.Zeros(nParticles*3, 1);
                for (int i = 0; i < nParticles; i++)
                {
                    origX[i * 3 + 0, 0] = x[i, 0];
                    origX[i * 3 + 1, 0] = x[i, 1];
                    origX[i * 3 + 2, 0] = x[i, 2];
                }
                List<int> shift = new List<int>();
                int T1 = 0;
                int T2 = 0;
                for (int i = 0; i < nParticles; i++)
                {
                    shift.Add(i);
                }
                int C1 = 0,C2=0;
                if (boundary)
                {
                    T1 = (nParticles - 4) * 3 - 1;
                    T2 = nParticles * 3 - 1;
                    C1 = 0;
                    C2 = nParticles - 4;
                    for (int i = 0; i < nParticles; i++)
                    {
                        if (i == 0 || i == nU - 1 || i == nU * nV - nU || i == nU * nV - 1)
                        {
                            shift[i] = C2;
                            C2++;
                        }
                        else
                        {
                            shift[i] = C1;
                            C1++;
                        }
                    }

                }
                else
                {
                    T1 = (nParticles - boundaryIndex.Count()) * 3 - 1;
                    T2 = nParticles * 3 - 1;
                    C1 = 0;
                    C2 = nParticles - boundaryIndex.Count();
                    for (int i = 0; i < nParticles; i++)
                    {
                        if (isBoundary(i))
                        {
                            shift[i] = C2;
                            C2++;
                        }
                        else
                        {
                            shift[i] = C1;
                            C1++;
                        }
                    }
                }
                var shiftArray = new SparseDoubleArray(nParticles * 3, nParticles * 3);
                for (int i = 0; i < nParticles; i++)
                {
                    shiftArray[i * 3, shift[i] * 3] = 1;
                    shiftArray[i * 3 + 1, shift[i] * 3 + 1] = 1;
                    shiftArray[i * 3 + 2, shift[i] * 3 + 2] = 1;
                }
                var ED = shiftArray.T.Multiply(hess) as SparseDoubleArray;
                ED = ED.Multiply(shiftArray) as SparseDoubleArray;
                var slice1 = new SparseDoubleArray(T1 + 1, T2 + 1);
                var slice2 = new SparseDoubleArray(T2 + 1, T2 - T1);
                for (int i = 0; i < T1 + 1; i++)
                {
                    slice1[i, i] = 1;
                }
                for (int i = 0; i < T2 - T1; i++)
                {
                    slice2[i + T1 + 1, i] = 1;
                }
                var DIB = (slice1.Multiply(ED) as SparseDoubleArray).Multiply(slice2) as SparseDoubleArray;
                var DII = (slice1.Multiply(ED) as SparseDoubleArray).Multiply(slice1.T) as SparseDoubleArray;
                var solver = new SparseLU(DII);
                origX = shiftArray.T * origX;
                var fixX = origX.GetSlice(T1 + 1, T2, 0, 0);
                var B = -DIB * fixX;
                var force = DoubleArray.Zeros(nParticles * 3, 1);
                for(int i=0;i<nParticles;i++)
                {
                    force[i * 3 + 0, 0] = 0;
                    force[i * 3 + 1, 0] = 0;
                    force[i * 3 + 2, 0] = 0;
                }
                force = (shiftArray.T*force).GetSlice(0, T1, 0, 0);
                B = B + force;
                var dx = solver.Solve(B);

                var ret = DoubleArray.Zeros(nParticles *3, 1);
                for (int i = 0; i < T1 + 1; i++)
                {
                    ret[i, 0] = dx[i, 0];
                }
                for (int i = T1 + 1; i <= T2; i++)
                {
                    ret[i, 0] = fixX[i - T1 - 1, 0];
                }
                var xx = shiftArray * ret;
                var F=hess* xx;
                for (int i = 0; i < nParticles; i++)
                {
                    double Fx = F[i * 3 + 0, 0];
                    double Fy = F[i * 3 + 1, 0];
                    double Fz = F[i * 3 + 2, 0];

                    if ((Fx * Fx + Fy * Fy + Fz * Fz) >0.01)
                    {
                        F[i * 3 + 0, 0] = Fx;
                        F[i * 3 + 1, 0] = Fy;
                        F[i * 3 + 2, 0] = Fz+1;
                        Force[i, 0] = Fx;
                        Force[i, 1] = Fy;
                        Force[i, 2] = Fz+1;
                    }
                    else
                    {
                        F[i * 3 + 0, 0] = 0;
                        F[i * 3 + 1, 0] = 0;
                        F[i * 3 + 2, 0] = 0+1;
                        Force[i, 0] = 0;
                        Force[i, 1] = 0;
                        Force[i, 2] = 0+1;
                    }
                }
            
            
                origX = DoubleArray.Zeros(nParticles * 3, 1);
                for (int i = 0; i < nParticles; i++)
                {
                    origX[i * 3 + 0, 0] = x[i, 0];
                    origX[i * 3 + 1, 0] = x[i, 1];
                    origX[i * 3 + 2, 0] = x[i, 2];
                }
                shift = new List<int>();
                for (int i = 0; i < nParticles; i++)
                {
                    shift.Add(i);
                }
                T1 = (nParticles - 4) * 3 - 1;
                T2 = nParticles * 3 - 1;
                C1 = 0;
                C2 = nParticles - 4;
                for (int i = 0; i < nParticles; i++)
                {
                    if (i==0 || i==nU-1|| i==nU*nV-nU || i==nU*nV-1)
                    {
                        shift[i] = C2;
                        C2++;
                    }
                    else
                    {
                        shift[i] = C1;
                        C1++;
                    }
                }
                shiftArray = new SparseDoubleArray(nParticles * 3, nParticles * 3);
                for (int i = 0; i < nParticles; i++)
                {
                    shiftArray[i * 3, shift[i] * 3] = 1;
                    shiftArray[i * 3 + 1, shift[i] * 3 + 1] = 1;
                    shiftArray[i * 3 + 2, shift[i] * 3 + 2] = 1;
                }


                ED = shiftArray.T.Multiply(hess) as SparseDoubleArray;
                ED = ED.Multiply(shiftArray) as SparseDoubleArray;
                slice1 = new SparseDoubleArray(T1 + 1, T2 + 1);
                slice2 = new SparseDoubleArray(T2 + 1, T2 - T1);
                for (int i = 0; i < T1 + 1; i++)
                {
                    slice1[i, i] = 1;
                }
                for (int i = 0; i < T2 - T1; i++)
                {
                    slice2[i + T1 + 1, i] = 1;
                }
                DIB = (slice1.Multiply(ED) as SparseDoubleArray).Multiply(slice2) as SparseDoubleArray;
                DII = (slice1.Multiply(ED) as SparseDoubleArray).Multiply(slice1.T) as SparseDoubleArray;
                solver = new SparseLU(DII);
                origX = shiftArray.T * origX;
                fixX = origX.GetSlice(T1 + 1, T2, 0, 0);
                B = -DIB * fixX;
                force = (shiftArray.T * F).GetSlice(0, T1, 0, 0);
                B = B + force;
                dx = solver.Solve(B);

                ret = DoubleArray.Zeros(nParticles * 3, 1);
                for (int i = 0; i < T1 + 1; i++)
                {
                    ret[i, 0] = dx[i, 0];
                }
                for (int i = T1 + 1; i <= T2; i++)
                {
                    ret[i, 0] = fixX[i - T1 - 1, 0];
                }
                xx = shiftArray * ret;
            

            
            
                for (int i = 0; i < nParticles; i++)
                {   
                    x[i, 0] = xx[i * 3, 0];
                    x[i, 1] = xx[i * 3 + 1, 0];
                    x[i, 2] = xx[i * 3 + 2, 0];
                }
                norm = 0;
                for (int i = 0; i < nU * nV; i++)
                {
                    norm += x[i, 2] * x[i, 2];
                }
                norm = Math.Sqrt(norm);
                if (boundary)
                {
                    norm = norm * 3;
                }
                for (int i = 0; i < nU * nV; i++)
                {
                    x[i, 2] /= norm / 750;
                }            

                x2Nurbs(x, outputNurbs[s]);
            }

        }
         * */
        //public override void BakeGeometry(Rhino.RhinoDoc doc, Rhino.DocObjects.ObjectAttributes att, List<Guid> obj_ids)
        //{
/*            Rhino.DocObjects.ObjectAttributes a2 = att.Duplicate();
            a2.LayerIndex = 1;
            Rhino.DocObjects.ObjectAttributes a3 = att.Duplicate();
            a3.LayerIndex = 2;
            for (int s = (nU - 2) * (nV - 2) - 1; s >= 0; s--)
            {
                var aN = airyNurbs[s].Duplicate() as Surface;
                aN.Transform(XZ2[s]);
                var oN = outputNurbs[s].Duplicate() as Surface;
                oN.Transform(XZ[s]);
                Guid id = doc.Objects.AddSurface(aN, a2);
                obj_ids.Add(id);
                Guid id2 = doc.Objects.AddSurface(oN, a3);
                obj_ids.Add(id2);            
            }*/
        //}
        //public override void DrawViewportWires(Grasshopper.Kernel.IGH_PreviewArgs args)
        //{
            /*if (Hidden)
            {
                return;
            }*/
            /*for (int i = 0; i < 4; i++)
            {
                double[] f = null;
                if (cuttingPlane[i] != null)
                {
                    f = cuttingPlane[i].GetPlaneEquation();
                    //System.Windows.MessageBox.Show(f[0].ToString()+","+f[1].ToString()+","+f[2].ToString()+","+f[3].ToString());
                    var P1 = cuttingPlane[i].PointAt(-100, -100);
                    var P2 = cuttingPlane[i].PointAt(100, -100);
                    var P3 = cuttingPlane[i].PointAt(100, 100);
                    var P4 = cuttingPlane[i].PointAt(-100, 100);
                    args.Display.DrawLine(P1, P2, System.Drawing.Color.LightCoral, 2);
                    args.Display.DrawLine(P2, P3, System.Drawing.Color.LightCoral, 2);
                    args.Display.DrawLine(P3, P4, System.Drawing.Color.LightCoral, 2);
                    args.Display.DrawLine(P4, P1, System.Drawing.Color.LightCoral, 2);
                }
            }*/
            /*
            if (boundary)
            {
                for (int s = (nU - 2) * (nV - 2) - 1; s >= 0; s--)
                {
                    var f = outputNurbs[s].IsoCurve(0, 0);
                    f.Transform(XZ[s]);
                    var g = outputNurbs[s].IsoCurve(1, 0);
                    g.Transform(XZ[s]);
                    var h = outputNurbs[s].IsoCurve(1, outputNurbs[s].Domain(0).Max);
                    h.Transform(XZ[s]);
                    var c = outputNurbs[s].IsoCurve(0, outputNurbs[s].Domain(1).Max);
                    c.Transform(XZ[s]);
                    args.Display.DrawCurve(f, System.Drawing.Color.Orange, 4);
                    args.Display.DrawCurve(g, System.Drawing.Color.Orange, 4);
                    args.Display.DrawCurve(h, System.Drawing.Color.Orange, 4);
                    args.Display.DrawCurve(c, System.Drawing.Color.Orange, 4);
                }
            }
            if (XZ != null)
            {
                for (int s = (nU - 2) * (nV - 2) - 1; s >= 0; s--)
                {
                    var aN = airyNurbs[s].Duplicate() as Surface;
                    aN.Transform(XZ2[s]);
                    var oN = outputNurbs[s].Duplicate() as Surface;
                    oN.Transform(XZ[s]);
                    args.Display.DrawSurface(aN, System.Drawing.Color.Blue, 1);
                    args.Display.DrawSurface(oN, System.Drawing.Color.Red, 1);
                }
            }
//            args.Display.DrawSurface(outputNurbs[1], System.Drawing.Color.Blue, 1);
//            args.Display.DrawSurface(xyNurbs, System.Drawing.Color.Green, 1);
            double[][] vec = new double[2][] { new double[3], new double[3] };
            double[] val = new double[2];
            double[] node=null;
            double S = 50;
            Nurbs2x(xyNurbs, x);
            for (int s = (nU - 2) * (nV - 2) - 1; s >= 0; s--)
            {
                var v=myMasonry[s];
                v.setupNodesFromList(x);
                v.computeGlobalCoord();

                foreach (var e in v.elemList)
                {
                    for (int i = 0; i < e.nIntPoint; i++)
                    {
                        e.getEigenVectors(vec, val, i);
                        node = e.getIntPoint(i);
                        System.Drawing.Color color;
                        double S1 = S * val[0];
                        double S2 = S * val[1];
                        if (val[0] > 0)
                        { color = System.Drawing.Color.Cyan; }
                        else { color = System.Drawing.Color.Magenta; }
                        var P1=new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        var P2=new Rhino.Geometry.Point3d(node[0] + vec[0][0] * S1, node[1] + vec[0][1] * S1, node[2] + vec[0][2] * S1);
                        P2.Transform(XZ[s]);                        
                        args.Display.DrawLine(P1,P2 , color, 1);
                        P1=new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        P2=new Rhino.Geometry.Point3d(node[0] - vec[0][0] * S1, node[1] - vec[0][1] * S1, node[2] - vec[0][2] * S1);
                        P2.Transform(XZ[s]);
                        args.Display.DrawLine(P1, P2, color, 1);
                        if (val[1] > 0)
                        { color = System.Drawing.Color.Cyan; }
                        else { color = System.Drawing.Color.Magenta; }
                        P1 = new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        P2 = new Rhino.Geometry.Point3d(node[0] + vec[1][0] * S2, node[1] + vec[1][1] * S2, node[2] + vec[1][2] * S2);
                        P2.Transform(XZ[s]);
                        args.Display.DrawLine(P1, P2, color, 1);
                        P1 = new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        P2 = new Rhino.Geometry.Point3d(node[0] - vec[1][0] * S2, node[1] - vec[1][1] * S2, node[2] - vec[1][2] * S2);
                        P2.Transform(XZ[s]);
                        args.Display.DrawLine(P1, P2, color, 1);

                    }
                    for (int i = 0; i < e.nBIntPoint; i++)
                    {
                        e.getBEigenVectors(vec, val, i);
                        node = e.getBIntPoint(i);
                        System.Drawing.Color color;
                        double S1 = S * val[0];
                        double S2 = S * val[1];
                        if (val[0] > 0)
                        { color = System.Drawing.Color.Cyan; }
                        else { color = System.Drawing.Color.Magenta; }
                        var P1 = new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        var P2 = new Rhino.Geometry.Point3d(node[0] + vec[0][0] * S1, node[1] + vec[0][1] * S1, node[2] + vec[0][2] * S1);
                        P2.Transform(XZ[s]);
                        args.Display.DrawLine(P1, P2, color, 1);
                        P1 = new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        P2 = new Rhino.Geometry.Point3d(node[0] - vec[0][0] * S1, node[1] - vec[0][1] * S1, node[2] - vec[0][2] * S1);
                        P2.Transform(XZ[s]);
                        args.Display.DrawLine(P1, P2, color, 1);
                        if (val[1] > 0)
                        { color = System.Drawing.Color.Cyan; }
                        else { color = System.Drawing.Color.Magenta; }
                        P1 = new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        P2 = new Rhino.Geometry.Point3d(node[0] + vec[1][0] * S2, node[1] + vec[1][1] * S2, node[2] + vec[1][2] * S2);
                        P2.Transform(XZ[s]);
                        args.Display.DrawLine(P1, P2, color, 1);
                        P1 = new Rhino.Geometry.Point3d(node[0], node[1], node[2]);
                        P1.Transform(XZ[s]);
                        P2 = new Rhino.Geometry.Point3d(node[0] - vec[1][0] * S2, node[1] - vec[1][1] * S2, node[2] - vec[1][2] * S2);
                        P2.Transform(XZ[s]);
                        args.Display.DrawLine(P1, P2, color, 1);
                    }
                }
            }
            List<Rhino.Geometry.Point3d> xyP = new List<Point3d>();
            List<Rhino.Geometry.Point3d> outputP = new List<Point3d>();
            Nurbs2x(xyNurbs, x);
            foreach (var v in myMasonry)
            {
                v.setupNodesFromList(x);
                v.computeGlobalCoord();
                foreach (var e in v.elemList)
                {
                    for (int i = 0; i < e.nIntPoint; i++)
                    {
                        var d = e.getIntPoint(i);
                        var d2 = new Rhino.Geometry.Point3d(d[0], d[1], d[2]);
                        xyP.Add(d2);
                        args.Display.DrawPoint(d2, Rhino.Display.PointStyle.ControlPoint, 2, System.Drawing.Color.Red);
                    }
                    if (!boundary)
                    {
                        for (int i = 0; i < e.nBIntPoint; i++)
                        {
                            var d = e.getBIntPoint(i);
                            var d2 = new Rhino.Geometry.Point3d(d[0], d[1], d[2]);
                            args.Display.DrawPoint(d2, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Red);
                        }
                    }
                }
            }
            */
/*            for (int s = (nU - 2) * (nV - 2) - 1; s >= 0; s--)
            {
                Nurbs2x(outputNurbs[s], x);
                myMasonry.setupNodesFromList(x);
                myMasonry.computeGlobalCoord();
                foreach (var e in myMasonry.elemList)
                {
                    for (int i = 0; i < e.nIntPoint; i++)
                    {
                        var d = e.getIntPoint(i);
                        var d2 = new Rhino.Geometry.Point3d(d[0], d[1], d[2]);
                        outputP.Add(d2);
                        args.Display.DrawPoint(d2, Rhino.Display.PointStyle.ControlPoint, 2, System.Drawing.Color.Red);
                    }
                }
                if (boundary)
                {
                    foreach (var e in myMasonry.edgeList)
                    {
                        for (int i = 0; i < e.nIntPoint; i++)
                        {
                            var d = e.getIntPoint(i);
                            var d2 = new Rhino.Geometry.Point3d(d[0], d[1], d[2]);
                            outputP.Add(d2);
                            args.Display.DrawPoint(d2, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Green);
                        }
                    }
                }
                args.Display.DrawPoint(outputNurbs[s].Points.GetControlPoint(0, 0).Location, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Red);
                args.Display.DrawPoint(outputNurbs[s].Points.GetControlPoint(nU - 1, 0).Location, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Red);
                args.Display.DrawPoint(outputNurbs[s].Points.GetControlPoint(0, nV - 1).Location, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Red);
                args.Display.DrawPoint(outputNurbs[s].Points.GetControlPoint(nU - 1, nV - 1).Location, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Red);
            }*/
            /*double S3 = 50;
            if (Force != null)
            {
                for (int i = 0; i < nU * nV; i++)
                {
                    double Fx = Force[i, 0] * S3;
                    double Fy = Force[i, 1] * S3;
                    double Fz = Force[i, 2] * S3;
                    if ((Fx * Fx + Fy * Fy + Fz * Fz) !=0)
                        args.Display.DrawArrow(new Line(x[i, 0], x[i, 1], x[i, 2], x[i, 0] + Fx, x[i, 1] + Fy, x[i, 2] + Fz), System.Drawing.Color.Orange);
                }
            }*/
            //base.DrawViewportWires(args);
        //}
        //bool boundary = false;
        //protected override void SolveInstance(Grasshopper.Kernel.IGH_DataAccess DA)
        //{
/*            Object inputGeometry = null;
            if (!DA.GetData(0, ref inputGeometry)) { return; }
            if (!DA.GetData(1, ref boundary)) { boundary = false; }
            if (inputGeometry is Grasshopper.Kernel.Types.GH_Surface)
            {
                inputNurbs = (inputGeometry as Grasshopper.Kernel.Types.GH_Surface).Value.Surfaces[0].ToNurbsSurface();
                initialized=initializeNurbs(inputNurbs);
            }
            __update = true;}*/
        //}
        /*
        void createAiryInitialSurface(Rhino.Geometry.NurbsSurface S)
        {
            //find the center point
            for (int i = 0; i < nV; i++)
            {
                for (int j = 0; j < nU; j++)
                {
                    var P = S.Points.GetControlPoint(j, i);
                    double Z = (i - ((nV-1) / 2d)) * (i - ((nV-1) / 2d)) + (j - ((nU-1) / 2d)) * (j - ((nU-1) / 2d));
                    S.Points.SetControlPoint(j, i, new Rhino.Geometry.ControlPoint(P.Location.X, P.Location.Y, -Z));
                }
            }
        }
        void x2Nurbs(double[,] _x,Rhino.Geometry.NurbsSurface S)
        {
            for (int i = 0; i < nV; i++)
            {
                for (int j = 0; j < nU; j++)
                {
                    var P=new Rhino.Geometry.Point3d(x[(i * nU + j) , 0],x[(i * nU + j) , 1] ,x[(i * nU + j) ,2] );
                    S.Points.SetControlPoint(j, i, P);
                }
            }

        }
        void Nurbs2x(Rhino.Geometry.NurbsSurface S,double[,] _x)
        {
            for (int i = 0; i < nV; i++)
            {
                for (int j = 0; j < nU; j++)
                {
                    x[(i*nU+j),0]=S.Points.GetControlPoint(j, i).Location.X;
                    x[(i*nU+j),1]=S.Points.GetControlPoint(j, i).Location.Y;
                    x[(i*nU+j),2]=S.Points.GetControlPoint(j, i).Location.Z;
                }
            }
        }
        bool initializeNurbs(Rhino.Geometry.NurbsSurface S)
        {
            var X = Rhino.Geometry.Transform.Translation(250, 0, 0);
            alternativeMesh = new Rhino.Geometry.Mesh();
            nU = S.Points.CountU;
            nV = S.Points.CountV;
            airyNurbs = new NurbsSurface[(nU - 2) * (nV - 2)];
            outputNurbs = new NurbsSurface[(nU - 2) * (nV - 2)];
            XZ = new Transform[(nU - 2) * (nV - 2)];
            XZ2 = new Transform[(nU - 2) * (nV - 2)];

            int count = 0;
            for (int s = 1; s < nU - 1; s++)
            {
                for (int t = 1; t < nV - 1; t++, count++)
                {
                    XZ[count] = Transform.Translation(250 * s, 250 * t, 0);
                    XZ2[count] = Transform.Translation(250 * s, 250 * t, 400);
                }
            }

            for (int j = 0; j < nV; j++)
            {
                for (int i = 0; i < nU; i++)
                {
                    Rhino.Geometry.ControlPoint cPoint = S.Points.GetControlPoint(i, j);
                    alternativeMesh.Vertices.Add(cPoint.Location);
                }
            }
            for (int j = 0; j < nV - 1; j++)
            {
                for (int i = 0; i < nU - 1; i++)
                {
                    int D1 = j * nU + i;
                    int D2 = j * nU + i + 1;
                    int D3 = (j + 1) * nU + i + 1;
                    int D4 = (j + 1) * nU + i;
                    alternativeMesh.Faces.AddFace(new Rhino.Geometry.MeshFace(D1, D2, D3, D4));
                }
            }
            //Get Boundary
            boundaryIndex = new List<int>();
            Rhino.Geometry.Polyline[] boundary = alternativeMesh.GetNakedEdges();
            
            foreach (var p in boundary)
            {
                for (int i = 0; i < p.Count - 1; i++)
                {
                    var f = alternativeMesh.Vertices.Select((e,index)=>new{e,index}).Where(e=>e.e==p[i]).Select(e=>e.index).First();
                    boundaryIndex.Add(f);
                }
            }
            for (int s = 0;s<(nU-2)*(nV-2);s++)
            {
                airyNurbs[s]=S.Duplicate() as Rhino.Geometry.NurbsSurface;
                outputNurbs[s]=S.Duplicate() as Rhino.Geometry.NurbsSurface;
            }
            xyNurbs = S.Duplicate() as Rhino.Geometry.NurbsSurface;
            foreach (var s in airyNurbs)
            {
                s.Transform(X);
                createAiryInitialSurface(s);
            }
            foreach (var s in outputNurbs)
            {
                s.Transform(X);
            }
            xyNurbs.Transform(X);
            createNurbsElements(inputNurbs);

            x = new double[nU * nV , 3];
            Nurbs2x(xyNurbs, x);
            foreach (var v in myMasonry)
            {
                v.setupNodesFromList(x);
                v.computeGlobalCoord();
            }
            return true;
        }
        void createNurbsElements(Rhino.Geometry.NurbsSurface S)
        {
            double[] uKnot;
            double[] vKnot;

            int N = nU * nV;
            int uDim = S.OrderU;
            int vDim = S.OrderV;
            int uDdim = S.OrderU - 1;
            int vDdim = S.OrderV - 1;


            uKnot = new double[nU - uDdim + 1 + uDdim * 2];
            vKnot = new double[nV - vDdim + 1 + vDdim * 2];
            for (int i = 0; i < uDdim; i++)
            {
                uKnot[i] = 0;
            }
            for (int i = 0; i < vDdim; i++)
            {
                vKnot[i] = 0;
            }
            for (int i = 0; i < nU - uDdim + 1; i++)
            {
                uKnot[i + uDdim] = i;
            }
            for (int i = 0; i < nV - vDdim + 1; i++)
            {
                vKnot[i + vDdim] = i;
            }
            for (int i = 0; i < uDdim; i++)
            {
                uKnot[i + nU + 1] = nU - uDdim;
            }
            for (int i = 0; i < vDdim; i++)
            {
                vKnot[i + nV + 1] = nV - vDdim;
            }
            myMasonry = new Minilla3D.Objects.masonry[(nU - 2) * (nV - 2)];
            for (int i = 0; i < (nU - 2) * (nV - 2); i++)
            {
                myMasonry[i] = new Minilla3D.Objects.masonry();
            }
            foreach(var v in myMasonry)
            {
                for (int j = 1; j < nV - vDdim + 1; j++)
                {
                    for (int i = 1; i < nU - uDdim + 1; i++)
                    {
                        int[] index = new int[uDim * vDim];
                        for (int k = 0; k < vDim; k++)
                        {
                            for (int l = 0; l < uDim; l++)
                            {
                                index[k * uDim + l] = (j - 1 + k) * nU + i - 1 + l;
                            }
                        }
                        //judge if on the border
                        Minilla3D.Elements.nurbsElement.border _border = Minilla3D.Elements.nurbsElement.border.None;
                        if (j == 1)
                        {
                            _border = _border | Minilla3D.Elements.nurbsElement.border.Top;
                            {
                                int[] index2 = new int[uDim];
                                for (int l = 0; l < uDim; l++)
                                {
                                    index2[l] = (j - 1) * nU + i - 1 + l;
                                }
                                var c = new Minilla3D.Elements.nurbsCurve(uDim, index2, i, uKnot);
                                v.edgeList.Add(c);
                                v.topEdge.Add(c);
                            }
                        }
                        if (i == 1)
                        {
                            _border = _border | Minilla3D.Elements.nurbsElement.border.Left;
                            {
                                int[] index2 = new int[vDim];
                                for (int k = 0; k < vDim; k++)
                                {
                                    index2[k] = (j - 1 + k) * nU + i - 1;
                                }
                                var c = new Minilla3D.Elements.nurbsCurve(vDim, index2, j, vKnot);
                                v.edgeList.Add(c);
                                v.leftEdge.Add(c);
                            }
                        }
                        if (j == nV - vDdim)
                        {
                            _border = _border | Minilla3D.Elements.nurbsElement.border.Bottom;
                            {
                                int[] index2 = new int[uDim];
                                for (int l = 0; l < uDim; l++)
                                {
                                    index2[l] = (j - 1 + (vDim - 1)) * nU + i - 1 + l;
                                }
                                var c = new Minilla3D.Elements.nurbsCurve(uDim, index2, i, uKnot);
                                v.edgeList.Add(c);
                                v.bottomEdge.Add(c);
                            }
                        }
                        if (i == nU - uDdim)
                        {
                            _border = _border | Minilla3D.Elements.nurbsElement.border.Right;
                            {
                                int[] index2 = new int[vDim];
                                for (int k = 0; k < vDim; k++)
                                {
                                    index2[k] = (j - 1 + k) * nU + i - 1 + uDim - 1;

                                }
                                var c = new Minilla3D.Elements.nurbsCurve(vDim, index2, j, vKnot);
                                v.edgeList.Add(c);
                                v.rightEdge.Add(c);
                            }
                        }
                        v.elemList.Add(new Minilla3D.Elements.nurbsElement(uDim, vDim, index, i, j, uKnot, vKnot, _border));
                        switch (_border)
                        {
                            case nurbsElement.border.Left | nurbsElement.border.Top | nurbsElement.border.Right:
                                v.elemList.Last().stitch(v.leftEdge.Last(), v.topEdge.Last(), v.rightEdge.Last());
                                break;
                            case nurbsElement.border.Left | nurbsElement.border.Bottom | nurbsElement.border.Right:
                                v.elemList.Last().stitch(v.leftEdge.Last(), v.bottomEdge.Last(), v.rightEdge.Last());
                                break;
                            case nurbsElement.border.Left | nurbsElement.border.Top | nurbsElement.border.Bottom:
                                v.elemList.Last().stitch(v.leftEdge.Last(), v.topEdge.Last(), v.bottomEdge.Last());
                                break;
                            case nurbsElement.border.Right | nurbsElement.border.Top | nurbsElement.border.Bottom:
                                v.elemList.Last().stitch(v.rightEdge.Last(), v.topEdge.Last(), v.bottomEdge.Last());
                                break;
                            case nurbsElement.border.Left | nurbsElement.border.Right:
                                v.elemList.Last().stitch(v.leftEdge.Last(), v.rightEdge.Last());
                                break;
                            case nurbsElement.border.Top | nurbsElement.border.Bottom:
                                v.elemList.Last().stitch(v.topEdge.Last(), v.bottomEdge.Last());
                                break;
                            case nurbsElement.border.Left | nurbsElement.border.Top:
                                v.elemList.Last().stitch(v.leftEdge.Last(), v.topEdge.Last());
                                break;
                            case nurbsElement.border.Left | nurbsElement.border.Bottom:
                                v.elemList.Last().stitch(v.leftEdge.Last(), v.bottomEdge.Last());
                                break;
                            case nurbsElement.border.Right | nurbsElement.border.Top:
                                v.elemList.Last().stitch(v.rightEdge.Last(), v.topEdge.Last());
                                break;
                            case nurbsElement.border.Right | nurbsElement.border.Bottom:
                                v.elemList.Last().stitch(v.rightEdge.Last(), v.bottomEdge.Last());
                                break;
                            case nurbsElement.border.Left:
                                v.elemList.Last().stitch(v.leftEdge.Last());
                                break;
                            case nurbsElement.border.Right:
                                v.elemList.Last().stitch(v.rightEdge.Last());
                                break;
                            case nurbsElement.border.Top:
                                v.elemList.Last().stitch(v.topEdge.Last());
                                break;
                            case nurbsElement.border.Bottom:
                                v.elemList.Last().stitch(v.bottomEdge.Last());
                                break;
                        }
                    }
                }
            }
        }*/
    //}

}
