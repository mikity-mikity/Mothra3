using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ShoNS.Array;
using Rhino.Geometry;
namespace mikity.ghComponents
{
    class msgclass : mosek.Stream
    {
        string prefix;
        public msgclass(string prfx)
        {
            prefix = prfx;
        }

        public override void streamCB(string msg)
        {
            Console.Write("{0}{1}", prefix, msg);
        }
    }

    public partial class Mothra3 : Grasshopper.Kernel.GH_Component
    {
        public void defineKinkAngle(branch branch,leaf leaf,mosek.Task task, int numCon, int numVar)
        {
            for (int t= 0; t < branch.tuples.Count(); t++)
            {
                task.putaij(numCon + t, numVar + t, -1);
                task.putconbound(numCon + t, mosek.boundkey.fx, 0, 0);
                var tup = branch.tuples[t];
                var target = tup.target;
                target.dcdtstar[0] = target.dcdt[1];
                target.dcdtstar[1] = -target.dcdt[0];
                double gamma = 0;
                for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        gamma += target.dcdt[i] * target.gij[i, j] * target.dcdt[j];
                    }
                }
                for (int k = 0; k < target.nNode; k++)
                {
                    for (int i = 0; i < 2; i++)
                    {
                        target.s[i] =target.d1[i][k];
                    }
                    var val = 0d;
                    for (int i = 0; i < 2; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            val += target.s[i] * target.Gij[i, j] * target.dcdtstar[j];
                        }
                    }
                    val *= target.refDv;
                    val /= Math.Sqrt(gamma);
                    task.putaij(numCon + t, target.internalIndex[k] + leaf.varOffset, val);
                }
            }
        }
        public void defineKinkAngle2(branch branch, leaf leaf1, leaf leaf2, mosek.Task task, int numCon, int numVar)
        {
            for (int t = 0; t < branch.tuples.Count(); t++)
            {
                task.putaij(numCon + t, numVar + t, -1);
                task.putconbound(numCon + t, mosek.boundkey.fx, 0, 0);
                var tup = branch.tuples[t];
                for (int h = 0; h < 2; h++)
                {
                    Minilla3D.Elements.nurbsElement.tuple target = null;
                    leaf leaf = null;
                    if (h == 0) { target = tup.left; leaf = leaf1; }
                    if (h == 1) { target = tup.right; leaf = leaf2; }
                    target.dcdtstar[0] = target.dcdt[1];
                    target.dcdtstar[1] = -target.dcdt[0];
                    double gamma = 0;
                    for (int i = 0; i < 2; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            gamma += target.dcdt[i] * target.gij[i, j] * target.dcdt[j];
                            gamma += target.dcdt[i] * target.gij[i, j] * target.dcdt[j];
                        }
                    }
                    for (int k = 0; k < target.nNode; k++)
                    {
                        for (int i = 0; i < 2; i++)
                        {
                            target.s[i] = target.d1[i][k];
                        }
                        var val = 0d;
                        for (int i = 0; i < 2; i++)
                        {
                            for (int j = 0; j < 2; j++)
                            {
                                val += target.s[i] * target.Gij[i, j] * target.dcdtstar[j];
                            }
                        }
                        val *= target.refDv;
                        val /= Math.Sqrt(gamma);
                        task.putaij(numCon + t, target.internalIndex[k] + leaf.varOffset, val);
                    }
                }
            }
        }

        public void tieBranchD1(branch branch,leaf leaf,mosek.Task task,int num0/*1 or 2*/,int num1/*0 or 1*/)
        {
            if (leaf.branch[0] == branch)
            {
                if (leaf.nU == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        task.putconbound(i * num0 + num1 + branch.conOffset , mosek.boundkey.fx, 0, 0);
                        if (leaf.flip[0])
                        {
                            task.putaij(branch.conOffset + i * num0 + num1 , i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1 , (branch.N - 1 - i) + leaf.varOffset, -1);
                        }
                        else
                        {
                            task.putaij(branch.conOffset + i * num0 + num1 , i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1 , i + leaf.varOffset, -1);
                        }
                    }
                }
            }
            if (leaf.branch[1] == branch)
            {
                if (leaf.nV == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        task.putconbound(i * num0 + num1 + branch.conOffset, mosek.boundkey.fx, 0, 0);
                        if (leaf.flip[1])
                        {
                            task.putaij(branch.conOffset + i * num0 + num1, i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1, leaf.nU * (branch.N - i) - 1 + leaf.varOffset, -1);
                        }
                        else
                        {
                            task.putaij(branch.conOffset + i * num0 + num1, i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1, leaf.nU * (i + 1) - 1 + leaf.varOffset, -1);
                        }
                    }
                }
            }
            if (leaf.branch[2] == branch)
            {
                if (leaf.nU == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        task.putconbound(i * num0 + num1 + branch.conOffset, mosek.boundkey.fx, 0, 0);
                        if (leaf.flip[2])
                        {
                            task.putaij(branch.conOffset + i * num0 + num1, i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1, leaf.nU * (leaf.nV - 1) + (branch.N - 1 - i) + leaf.varOffset, -1);
                        }
                        else
                        {
                            task.putaij(branch.conOffset + i * num0 + num1, i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1, leaf.nU * (leaf.nV - 1) + i + leaf.varOffset, -1);
                        }
                    }
                }
            }

            if (leaf.branch[3] == branch)
            {
                if (leaf.nV == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        task.putconbound(i * num0 + num1 + branch.conOffset, mosek.boundkey.fx, 0, 0);
                        if (leaf.flip[3])
                        {
                            task.putaij(branch.conOffset + i * num0 + num1, i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1, leaf.nU * (branch.N - 1 - i) + leaf.varOffset, -1);
                        }
                        else
                        {
                            task.putaij(branch.conOffset + i * num0 + num1, i + branch.varOffset, 1);
                            task.putaij(branch.conOffset + i * num0 + num1, leaf.nU * i + leaf.varOffset, -1);
                        }
                    }
                }
            }

        }
        public void tieBranchD3(branch branch, leaf leaf, mosek.Task task, int num0/*1 or 2*/, int num1/*0 or 1*/)
        {
            if (leaf.branch[0] == branch)
            {
                if (leaf.nU == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            task.putconbound((i * num0 + num1) * 3 + k + branch.conOffset, mosek.boundkey.fx, 0, 0);
                            if (leaf.flip[0])
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, ((branch.N - 1 - i) * 3 + k) + leaf.varOffset, -1);
                            }
                            else
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + leaf.varOffset, -1);
                            }
                        }
                    }
                }
            }
            if (leaf.branch[1] == branch)
            {
                if (leaf.nV == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            task.putconbound((i * num0 + num1) * 3 + k + branch.conOffset, mosek.boundkey.fx, 0, 0);
                            if (leaf.flip[1])
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (leaf.nU * (branch.N - i) - 1) * 3 + k + leaf.varOffset, -1);
                            }
                            else
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (leaf.nU * (i + 1) - 1) * 3 + k + leaf.varOffset, -1);
                            }
                        }
                    }
                }
            }
            if (leaf.branch[2] == branch)
            {
                if (leaf.nU == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            task.putconbound((i * num0 + num1) * 3 + k + branch.conOffset, mosek.boundkey.fx, 0, 0);
                            if (leaf.flip[2])
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (leaf.nU * (leaf.nV - 1) + (branch.N - 1 - i)) * 3 + k + leaf.varOffset, -1);
                            }
                            else
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (leaf.nU * (leaf.nV - 1) + i) * 3 + k + leaf.varOffset, -1);
                            }
                        }
                    }
                }
            }

            if (leaf.branch[3] == branch)
            {
                if (leaf.nV == branch.N)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            task.putconbound((i * num0 + num1)*3+k + branch.conOffset, mosek.boundkey.fx, 0, 0);
                            if (leaf.flip[3])
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (leaf.nU * (branch.N - 1 - i)) * 3 + k + leaf.varOffset, -1);
                            }
                            else
                            {
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (i) * 3 + k + branch.varOffset, 1);
                                task.putaij(branch.conOffset + (i * num0 + num1) * 3 + k, (leaf.nU * i) * 3 + k + leaf.varOffset, -1);
                            }
                        }
                    }
                }
            }

        }
        public void mosek2(List<leaf> _listLeaf, List<branch> _listBranch, List<node> _listNode,double force)
        {
            double infinity = 0;
            int numvar = 0;
            int numcon = 0;
            foreach (var leaf in _listLeaf)
            {
                leaf.varOffset = numvar;
                leaf.conOffset = numcon;  //no conditions
                numvar += leaf.nU * leaf.nV * 3;  //x,y,z coordinates
            }
            foreach (var branch in _listBranch)
            {
                branch.varOffset = numvar;
                branch.conOffset = numcon;
                numvar += branch.N*3;   //x,y,z coodinates
                if (branch.branchType == branch.type.kink)
                {
                    numcon += 2 * branch.N * 3;
                }
                else
                {
                    numcon += branch.N * 3;
                }
            }
            foreach (var node in _listNode)
            {
                node.varOffset = numvar;
                node.conOffset = numcon;
                numvar+=3;  //always 3
                numcon += node.N*3;  //usually 3
            }
            //variable settings
            mosek.boundkey[] bkx = new mosek.boundkey[numvar];
            double[] blx = new double[numvar];
            double[] bux = new double[numvar];
            foreach (var leaf in _listLeaf)
            {
                //x,y,z
                for (int i = 0; i < leaf.nU * leaf.nV; i++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        bkx[i * 3 + k + leaf.varOffset] = mosek.boundkey.fr;
                        blx[i * 3 + k + leaf.varOffset] = -infinity;
                        bux[i * 3 + k + leaf.varOffset] = infinity;
                    }
                }
            }
            foreach (var branch in _listBranch)
            {
                for (int i = 0; i < branch.N; i++)
                {
                    if (branch.branchType == branch.type.fix)
                    {
                        bkx[i * 3 + 0 + branch.varOffset] = mosek.boundkey.fx;
                        blx[i * 3 + 0 + branch.varOffset] = branch.crv.Points[i].Location.X;
                        bux[i * 3 + 0 + branch.varOffset] = branch.crv.Points[i].Location.X;
                        bkx[i * 3 + 1 + branch.varOffset] = mosek.boundkey.fx;
                        blx[i * 3 + 1 + branch.varOffset] = branch.crv.Points[i].Location.Y;
                        bux[i * 3 + 1 + branch.varOffset] = branch.crv.Points[i].Location.Y;
                        bkx[i * 3 + 2 + branch.varOffset] = mosek.boundkey.fx;
                        blx[i * 3 + 2 + branch.varOffset] = 0;
                        bux[i * 3 + 2 + branch.varOffset] = 0;
                    }
                    else
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            bkx[i * 3 + k + branch.varOffset] = mosek.boundkey.fr;
                            blx[i * 3 + k + branch.varOffset] = -infinity;
                            bux[i * 3 + k + branch.varOffset] = infinity;
                        }
                    }
                }
            }
            foreach (var node in _listNode)
            {
                for (int k = 0; k < 3; k++)
                {
                    bkx[k + node.varOffset] = mosek.boundkey.fr;
                    blx[k + node.varOffset] = -infinity;
                    bux[k + node.varOffset] = infinity;
                }
            }
            // Make mosek environment.
            using (mosek.Env env = new mosek.Env())
            {
                // Create a task object.
                using (mosek.Task task = new mosek.Task(env, 0, 0))
                {
                    // Directs the log task stream to the user specified
                    // method msgclass.streamCB
                    task.set_Stream(mosek.streamtype.log, new msgclass(""));

                    /* Append 'numcon' empty constraints.
                       The constraints will initially have no bounds. */
                    task.appendcons(numcon);

                    /* Append 'numvar' variables.
                       The variables will initially be fixed at zero (x=0). */
                    task.appendvars(numvar);

                    for (int j = 0; j < numvar; ++j)
                    {
                        task.putvarbound(j, bkx[j], blx[j], bux[j]);
                    }
                    foreach (var leaf in _listLeaf)
                    {
                        for (int i = 0; i < leaf.nU * leaf.nV; i++)
                        {
                            task.putcj(i*3+2+leaf.varOffset, -force);//force
                        }
                    }
                    ShoNS.Array.SparseDoubleArray mat = new SparseDoubleArray(numvar, numvar);
                    foreach (var leaf in _listLeaf)
                    {
                        foreach (var tup in leaf.tuples)
                        {
                            var det = tup.SPK[0, 0] * tup.SPK[1, 1] - tup.SPK[0, 1] * tup.SPK[0, 1];
                            if (det > 0)
                            {
                                for (int i = 0; i < tup.nNode; i++)
                                {
                                    for (int j = 0; j < tup.nNode; j++)
                                    {
                                        for (int k = 0; k < 3; k++)
                                        {
                                            for (int l = 0; l < 2; l++)
                                            {
                                                for (int m = 0; m < 2; m++)
                                                {
                                                    var val = tup.B[l, m, i * 3 + k, j * 3 + k] * tup.SPK[l, m] * tup.refDv * tup.area;
                                                    if (leaf.varOffset + tup.internalIndex[j] * 3 + k > leaf.varOffset + tup.internalIndex[i] * 3 + k) continue;
                                                    mat[leaf.varOffset + tup.internalIndex[i] * 3 + k, leaf.varOffset + tup.internalIndex[j] * 3 + k] += val;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    foreach (var branch in _listBranch)
                    {
                        if (branch.branchType != branch.type.open)
                        {
                            foreach (var tup in branch.tuples)
                            {
                                for (int i = 0; i < tup.nNode; i++)
                                {
                                    for (int j = 0; j < tup.nNode; j++)
                                    {
                                        for (int k = 0; k < 3; k++)
                                        {
                                            if (tup.SPK[0, 0] > 0)
                                            {
                                                var val = tup.B[0, 0, i * 3 + k, j * 3 + k] * tup.SPK[0, 0] * tup.refDv * tup.area;
                                                for (int l = 0; l < 1; l++)
                                                {
                                                    for (int m = 0; m < 1; m++)
                                                    {
                                                        var val2 = tup.B[l, m, i * 3 + k, j * 3 + k] * tup.SPK[l, m] * tup.refDv * tup.area;
                                                        if (branch.varOffset + tup.internalIndex[j] * 3 + k > branch.varOffset + tup.internalIndex[i] * 3 + k) continue;
                                                        mat[branch.varOffset + tup.internalIndex[i] * 3 + k, branch.varOffset + tup.internalIndex[j] * 3 + k] += val2;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    List<int> _qosubi = new List<int>();
                    List<int> _qosubj = new List<int>();
                    List<double> _qoval = new List<double>();
                    for (int i = 0; i < numvar; i++)
                    {
                        for (int j = 0; j < numvar; j++)
                        {
                            if (j > i) continue;
                            if (mat[i, j] != 0)
                            {
                                _qosubi.Add(i);
                                _qosubj.Add(j);
                                _qoval.Add(mat[i, j]);
                            }
                        }
                    }


                    var qosubi = _qosubi.ToArray();
                    var qosubj = _qosubj.ToArray();
                    var qoval = _qoval.ToArray();
                    task.putqobj(qosubi, qosubj, qoval);
                    foreach (var branch in _listBranch)
                    {
                        if (branch.branchType == branch.type.kink)
                        {
                            tieBranchD3(branch, branch.left, task, 2, 0);
                            tieBranchD3(branch, branch.right, task, 2, 1);
                        }
                        else
                        {
                            tieBranchD3(branch, branch.target, task, 1, 0);
                        }
                    }
                    foreach (var node in _listNode)
                    {
                        for (int i = 0; i < node.N; i++)
                        {
                            for (int k = 0; k < 3; k++)
                            {
                                task.putconbound(node.conOffset + i * 3 + k, mosek.boundkey.fx, 0, 0);
                                task.putaij(node.conOffset + i * 3 + k, node.varOffset + k, -1);
                                task.putaij(node.conOffset + i * 3 + k, node.share[i].varOffset + (node.number[i]) * 3 + k, 1);
                            }
                        }
                    }
                    task.putobjsense(mosek.objsense.minimize);
                    task.optimize();
                    // Print a summary containing information
                    //   about the solution for debugging purposes
                    task.solutionsummary(mosek.streamtype.msg);

                    mosek.solsta solsta;
                    /* Get status information about the solution */
                    task.getsolsta(mosek.soltype.itr, out solsta);

                    double[] xx = new double[numvar];

                    task.getxx(mosek.soltype.itr, // Basic solution.     
                                 xx);

                    switch (solsta)
                    {
                        case mosek.solsta.optimal:
                            System.Windows.Forms.MessageBox.Show("Optimal primal solution\n");
                            break;
                        case mosek.solsta.near_optimal:
                            System.Windows.Forms.MessageBox.Show("Near Optimal primal solution\n");
                            break;
                        case mosek.solsta.dual_infeas_cer:
                        case mosek.solsta.prim_infeas_cer:
                        case mosek.solsta.near_dual_infeas_cer:
                        case mosek.solsta.near_prim_infeas_cer:
                            Console.WriteLine("Primal or dual infeasibility.\n");
                            break;
                        case mosek.solsta.unknown:
                            System.Windows.Forms.MessageBox.Show("Unknown solution status\n");
                            break;
                        default:
                            System.Windows.Forms.MessageBox.Show("Other solution status\n");
                            break;

                    }
                    foreach (var branch in _listBranch)
                    {
                        branch.shellCrv = branch.crv.Duplicate() as NurbsCurve;
                        for (int i = 0; i < branch.N; i++)
                        {
                            branch.shellCrv.Points.SetPoint(i, new Point3d(xx[branch.varOffset + (i) * 3 + 0], xx[branch.varOffset + (i) * 3 + 1], xx[branch.varOffset + (i) * 3 + 2]));
                        }
                    }
                    foreach (var leaf in _listLeaf)
                    {
                        leaf.shellSrf = leaf.srf.Duplicate() as NurbsSurface;
                        for (int i = 0; i < leaf.nU; i++)
                        {
                            for (int j = 0; j < leaf.nV; j++)
                            {
                                leaf.shellSrf.Points.SetControlPoint(i, j, new ControlPoint(xx[leaf.varOffset + (i + leaf.nU * j) * 3 + 0], xx[leaf.varOffset + (i + leaf.nU * j) * 3 + 1], xx[leaf.varOffset + (i + leaf.nU * j) * 3 + 2]));
                            }
                        }
                    }

                }
            }
        }
        public void mosek1(List<leaf> _listLeaf,List<branch> _listBranch,List<node> _listNode,bool obj)
        {
            // Since the value infinity is never used, we define
            // 'infinity' symbolic purposes only
            double infinity = 0;
            int[] csub = new int[3];// for cones
            int numvar = 0;
            int numcon = 0;
            foreach (var leaf in _listLeaf)
            {
                leaf.varOffset = numvar;
                leaf.conOffset = numcon;
                numvar += (leaf.nU * leaf.nV) + leaf.r * 3;
                numcon += leaf.r * 3 ;// H11,H22,H12
            }
            foreach (var branch in _listBranch)
            {
                branch.varOffset = numvar;
                branch.conOffset = numcon;
                numvar += branch.N + branch.tuples.Count(); //added kink angle variables. can be set inequality constraints.
                if (branch.branchType == branch.type.kink)
                {
                    numcon += 2 * branch.N;
                }
                else if (branch.branchType == branch.type.reinforce)
                {
                    numvar++;               //z height variable
                    numcon += 1 * branch.N; //branch->z height
                    numcon += 1 * branch.N; //branch->edge 
                }
                else
                {
                    numcon += 1 * branch.N; //branch->edge 
                }
                numcon += branch.tuples.Count();// kink angle definitions.
            }
            foreach (var node in _listNode)
            {
                node.varOffset = numvar;
                node.conOffset = numcon;
                numvar++;  //always 1
                numcon += node.N;  //usually 3
            }
            //variable settings
            mosek.boundkey[] bkx = new mosek.boundkey[numvar];
            double[] blx = new double[numvar];
            double[] bux = new double[numvar];
            foreach (var leaf in _listLeaf)
            {
                //z
                for (int i = 0; i < leaf.nU * leaf.nV; i++)
                {
                    bkx[i+leaf.varOffset] = mosek.boundkey.fr;
                    blx[i + leaf.varOffset] = -infinity;
                    bux[i + leaf.varOffset] = infinity;
                }
                //H11,H22,H12
                if (leaf.leafType == leaf.type.convex)
                {
                    for (int i = 0; i < leaf.r; i++)
                    {
                        int n = i * 3 + (leaf.nU * leaf.nV);
                        bkx[n + leaf.varOffset] = mosek.boundkey.fr;
                        blx[n + leaf.varOffset] = -infinity;
                        bux[n + leaf.varOffset] = infinity;
                        bkx[n + 1 + leaf.varOffset] = mosek.boundkey.fr;
                        blx[n + 1 + leaf.varOffset] = -infinity;
                        bux[n + 1 + leaf.varOffset] = infinity;
                        bkx[n + 2 + leaf.varOffset] = mosek.boundkey.fr;
                        blx[n + 2 + leaf.varOffset] = -infinity;
                        bux[n + 2 + leaf.varOffset] = infinity;
                    }
                }
                else
                {
                    for (int i = 0; i < leaf.r; i++)
                    {
                        int n = i * 3 + (leaf.nU * leaf.nV);
                        bkx[n + leaf.varOffset] = mosek.boundkey.fr;
                        blx[n + leaf.varOffset] = -infinity;
                        bux[n + leaf.varOffset] = infinity;
                        bkx[n + 1 + leaf.varOffset] = mosek.boundkey.fr;
                        blx[n + 1 + leaf.varOffset] = -infinity;
                        bux[n + 1 + leaf.varOffset] = infinity;
                        bkx[n + 2 + leaf.varOffset] = mosek.boundkey.fr;
                        blx[n + 2 + leaf.varOffset] = -infinity;
                        bux[n + 2 + leaf.varOffset] = infinity;
                    }
                }
            }
            foreach(var branch in _listBranch)
            {
                if (branch.branchType == branch.type.reinforce )
                {
                    double A, B, C, D;
                    var vars = branch.slice.pl.GetPlaneEquation();
                    A = vars[0];
                    B = vars[1];
                    C = vars[2];
                    D = vars[3];
                    //plane is Ax+By+Cz+D=0, the norm of [A,B,C] is always 1
                    for (int i = 0; i < branch.N; i++)
                    {
                        /*if (i == 0 || i == branch.N - 1)
                        {
                            bkx[i + branch.varOffset] = mosek.boundkey.fr;
                            blx[i + branch.varOffset] = 0;
                            bux[i + branch.varOffset] = 0;
                        }
                        else*/
                        {
                            bkx[i + branch.varOffset] = mosek.boundkey.fr;
                            blx[i + branch.varOffset] = -infinity;
                            bux[i + branch.varOffset] = infinity;
                        }
                    }
                    //kink angle parameter
                    for (int i = 0; i < branch.tuples.Count(); i++)
                    {
                        bkx[branch.N + i + branch.varOffset] = mosek.boundkey.fr;
                        blx[branch.N + i + branch.varOffset] = 0;
                        bux[branch.N + i + branch.varOffset] = 0;
                    }
                    //Z height parameter
                    bkx[branch.N + branch.tuples.Count() + branch.varOffset] = mosek.boundkey.fr;
                    blx[branch.N + branch.tuples.Count() + branch.varOffset] = -infinity;
                    bux[branch.N + branch.tuples.Count() + branch.varOffset] = infinity;
                }
                else if (branch.branchType == branch.type.open)
                {
                    double A, B, C, D;
                    var vars = branch.slice.pl.GetPlaneEquation();
                    A = vars[0];
                    B = vars[1];
                    C = vars[2];
                    D = vars[3];
                    //plane is Ax+By+Cz+D=0, the norm of [A,B,C] is always 1
                    for (int i = 0; i < branch.N; i++)
                    {
                        bkx[i + branch.varOffset] = mosek.boundkey.fx;
                        double x = branch.crv.Points[i].Location.X;
                        double y = branch.crv.Points[i].Location.Y;
                        double z = (-D - A * x - B * y) / C;
                        blx[i + branch.varOffset] = z;
                        bux[i + branch.varOffset] = z;
                    }
                    //kink angle parameter
                    for (int i = 0; i < branch.tuples.Count(); i++)
                    {
                        bkx[branch.N + i + branch.varOffset] = mosek.boundkey.fx;
                        blx[branch.N + i + branch.varOffset] = branch.tuples[i].target.valDc;
                        bux[branch.N + i + branch.varOffset] = branch.tuples[i].target.valDc;
                    }
                }
                else if (branch.branchType == branch.type.kink)
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        bkx[i + branch.varOffset] = mosek.boundkey.fr;
                        blx[i + branch.varOffset] = -infinity;
                        bux[i + branch.varOffset] = infinity;
                    }
                    for (int i = 0; i < branch.tuples.Count(); i++)
                    {
                        if (obj)
                        {
                            bkx[branch.N + i + branch.varOffset] = mosek.boundkey.ra;
                            blx[branch.N + i + branch.varOffset] = 0;
                            bux[branch.N + i + branch.varOffset] = 0.2;
                        }
                        else
                        {
                            bkx[branch.N + i + branch.varOffset] = mosek.boundkey.lo;
                            blx[branch.N + i + branch.varOffset] = 0;
                            bux[branch.N + i + branch.varOffset] = infinity;
                        }

                    }
                }
                else
                {
                    for (int i = 0; i < branch.N; i++)
                    {
                        bkx[i + branch.varOffset] = mosek.boundkey.fr;
                        blx[i + branch.varOffset] = -infinity;
                        bux[i + branch.varOffset] = infinity;
                    }
                    //kink angle parameter
                    for (int i = 0; i < branch.tuples.Count(); i++)
                    {
                        bkx[branch.N + i + branch.varOffset] = mosek.boundkey.fr;
                        blx[branch.N + i + branch.varOffset] = -infinity;
                        bux[branch.N + i + branch.varOffset] = infinity;
                    }
                }
            }
            foreach(var node in _listNode)
            {
                bkx[node.varOffset]=mosek.boundkey.fr;
                blx[node.varOffset] = -infinity;
                bux[node.varOffset] = infinity;
            }
            // Make mosek environment.
            using (mosek.Env env = new mosek.Env())
            {
                // Create a task object.
                using (mosek.Task task = new mosek.Task(env, 0, 0))
                {
                    // Directs the log task stream to the user specified
                    // method msgclass.streamCB
                    task.set_Stream(mosek.streamtype.log, new msgclass(""));

                    /* Append 'numcon' empty constraints.
                       The constraints will initially have no bounds. */
                    task.appendcons(numcon);

                    /* Append 'numvar' variables.
                       The variables will initially be fixed at zero (x=0). */
                    task.appendvars(numvar);

                    for (int j = 0; j < numvar; ++j)
                    {
                        /* Set the linear term c_j in the objective.*/
                        //task.putcj(j, 1);
                        /* Set the bounds on variable j.
                               blx[j] <= x_j <= bux[j] */
                        task.putvarbound(j, bkx[j], blx[j], bux[j]);
                    }
                    //task.putqobjij
                    double root2 = Math.Sqrt(2);
                    foreach (var leaf in listLeaf)
                    {

                        double[] grad = new double[leaf.tuples[0].nNode];
                        double[] grad0 = new double[leaf.tuples[0].nNode];
                        double[] grad1i = new double[leaf.tuples[0].nNode];
                        double[] grad1j = new double[leaf.tuples[0].nNode];
                        //define H11,H12,H22
                        for (int i = 0; i < leaf.r; i++)
                        {
                            int N11 = i * 3; //condition number
                            int N22 = i * 3 + 1;
                            int N12 = i * 3 + 2;
                            int target = i * 3 + (leaf.nU * leaf.nV)+leaf.varOffset;   //variable number
                            task.putaij(N11+leaf.conOffset, target, -1);
                            task.putconbound(N11 + leaf.conOffset, mosek.boundkey.fx, 0, 0);
                            task.putaij(N22 + leaf.conOffset, target + 1, -1);
                            task.putconbound(N22 + leaf.conOffset, mosek.boundkey.fx, 0, 0);
                            task.putaij(N12 + leaf.conOffset, target + 2, -1);
                            task.putconbound(N12 + leaf.conOffset, mosek.boundkey.fx, 0, 0);
                            //N11
                            leaf.tuples[i].d2[0, 0].CopyTo(grad, 0);
                            leaf.tuples[i].d0.CopyTo(grad0, 0);
                            leaf.tuples[i].d1[0].CopyTo(grad1i, 0);
                            leaf.tuples[i].d1[0].CopyTo(grad1j, 0);
                            for (int k = 0; k < leaf.tuples[i].nNode; k++)
                            {
                                for (int j = 0; j < leaf.tuples[i].elemDim; j++)
                                {
                                    grad[k] -= leaf.tuples[i].Gammaijk[0, 0, j] * leaf.tuples[i].d1[j][k];
                                }
                                double val = 0;
                                val += grad[k];
                                task.putaij(N11 + leaf.conOffset, leaf.tuples[i].internalIndex[k] + leaf.varOffset, -val / root2);
                            }
                            //N22
                            leaf.tuples[i].d2[1, 1].CopyTo(grad, 0);
                            leaf.tuples[i].d0.CopyTo(grad0, 0);
                            leaf.tuples[i].d1[1].CopyTo(grad1i, 0);
                            leaf.tuples[i].d1[1].CopyTo(grad1j, 0);
                            for (int k = 0; k < leaf.tuples[i].nNode; k++)
                            {
                                for (int j = 0; j < leaf.tuples[i].elemDim; j++)
                                {
                                    grad[k] -= leaf.tuples[i].Gammaijk[1, 1, j] * leaf.tuples[i].d1[j][k];
                                }
                                double val = 0;
                                val += grad[k];
                                task.putaij(N22 + leaf.conOffset, leaf.tuples[i].internalIndex[k] + leaf.varOffset, -val / root2);
                            }
                            //N12
                            leaf.tuples[i].d2[0, 1].CopyTo(grad, 0);
                            leaf.tuples[i].d0.CopyTo(grad0, 0);
                            leaf.tuples[i].d1[0].CopyTo(grad1i, 0);
                            leaf.tuples[i].d1[1].CopyTo(grad1j, 0);
                            for (int k = 0; k < leaf.tuples[i].nNode; k++)
                            {
                                for (int j = 0; j < leaf.tuples[i].elemDim; j++)
                                {
                                    grad[k] -= leaf.tuples[i].Gammaijk[0, 1, j] * leaf.tuples[i].d1[j][k];
                                }
                                double val = 0;
                                val += grad[k];
                                task.putaij(N12 + leaf.conOffset, leaf.tuples[i].internalIndex[k] + leaf.varOffset, -val);
                            }

                        }
                        //if (leaf.leafType == leaf.type.convex)
                        {
                            for (int i = 0; i < leaf.r; i++)
                            {
                                int N11 = i * 3 + (leaf.nU * leaf.nV); //variable number
                                int N22 = i * 3 + 1 + (leaf.nU * leaf.nV);
                                int N12 = i * 3 + 2 + (leaf.nU * leaf.nV);

                                csub[0] = N11 + leaf.varOffset;
                                csub[1] = N22 + leaf.varOffset;
                                csub[2] = N12 + leaf.varOffset;
                                task.appendcone(mosek.conetype.rquad,
                                                0.0, // For future use only, can be set to 0.0 
                                                csub);
                            }
                        }
                    }
                    
                    
                    foreach (var branch in _listBranch)
                    {
                        if (branch.branchType == branch.type.kink)
                        {
                            tieBranchD1(branch, branch.left, task, 2, 0);
                            tieBranchD1(branch, branch.right, task, 2, 1);
                            defineKinkAngle2(branch,branch.left,branch.right,task, branch.conOffset + branch.N*2, branch.varOffset + branch.N);
                            if (obj)
                            {
                                for (int i = 0; i < branch.tuples.Count(); i++)
                                {
                                    task.putcj(branch.N + branch.varOffset + i,1);
                                }
                            }
                        }
                        else
                        {
                            tieBranchD1(branch, branch.target, task, 1, 0);
                            defineKinkAngle(branch,branch.target, task, branch.conOffset + branch.N, branch.varOffset + branch.N);
                        }
                        if (branch.branchType == branch.type.reinforce)
                        {
                            //height parameter
                            for (int i = 0; i < branch.N; i++)
                            {
                                task.putconbound(branch.conOffset + branch.N + branch.tuples.Count() + i, mosek.boundkey.fx, 0, 0);
                                task.putaij(branch.conOffset + branch.N + branch.tuples.Count() + i, branch.varOffset + branch.N + branch.tuples.Count(), -1);
                                task.putaij(branch.conOffset + branch.N + branch.tuples.Count() + i, branch.varOffset + i, 1);
                            }
                        }
                    }
                    foreach (var node in _listNode)
                    {
                        for (int i = 0; i < node.N; i++)
                        {
                            task.putconbound(node.conOffset + i, mosek.boundkey.fx, 0, 0);
                            task.putaij(node.conOffset + i, node.varOffset, -1);
                            task.putaij(node.conOffset + i, node.share[i].varOffset+node.number[i], 1);
                        }
                    }
                    task.putobjsense(mosek.objsense.maximize);
                    task.optimize();
                    // Print a summary containing information
                    //   about the solution for debugging purposes
                    task.solutionsummary(mosek.streamtype.msg);

                    mosek.solsta solsta;
                    /* Get status information about the solution */
                    task.getsolsta(mosek.soltype.itr, out solsta);

                    double[] xx = new double[numvar];

                    task.getxx(mosek.soltype.itr, // Basic solution.     
                                 xx);

                    switch (solsta)
                    {
                        case mosek.solsta.optimal:
                            System.Windows.Forms.MessageBox.Show("Optimal primal solution\n");
                            break;
                        case mosek.solsta.near_optimal:
                            System.Windows.Forms.MessageBox.Show("Near Optimal primal solution\n");
                            break;
                        case mosek.solsta.dual_infeas_cer:
                        case mosek.solsta.prim_infeas_cer:
                        case mosek.solsta.near_dual_infeas_cer:
                        case mosek.solsta.near_prim_infeas_cer:
                            Console.WriteLine("Primal or dual infeasibility.\n");
                            break;
                        case mosek.solsta.unknown:
                            System.Windows.Forms.MessageBox.Show("Unknown solution status\n");
                            break;
                        default:
                            System.Windows.Forms.MessageBox.Show("Other solution status\n");
                            break;

                    }
                    //store airy potential
                    foreach (var leaf in listLeaf)
                    {
                        double[] x = new double[leaf.nU * leaf.nV];
                        for (int j = 0; j < leaf.nV; j++)
                        {
                            for (int i = 0; i < leaf.nU; i++)
                            {
                                x[i + j * leaf.nU] = xx[i + j * leaf.nU + leaf.varOffset];
                            }
                        }
                        leaf.myMasonry.setupAiryPotentialFromList(x);
                    }
                    foreach (var leaf in listLeaf)
                    {
                        foreach (var tup in leaf.tuples)
                        {
                            leaf.myMasonry.elemList[tup.index].computeStressFunction(tup);
                        }
                    }
                    foreach (var branch in listBranch)
                    {
                        double[] x = new double[branch.N];
                        for (int i = 0; i < branch.N; i++)
                        {
                            x[i] = xx[i + branch.varOffset];
                        }
                        branch.myArch.setupAiryPotentialFromList(x);
                    }
                    foreach (var branch in listBranch)
                    {
                        foreach (var tup in branch.tuples)
                        {
                            if (branch.branchType == branch.type.kink)
                            {
                                branch.left.myMasonry.elemList[tup.left.index].computeTangent(tup.left);
                                branch.right.myMasonry.elemList[tup.right.index].computeTangent(tup.right);
                            }
                            else if (branch.branchType == branch.type.fix)
                            {
                                branch.target.myMasonry.elemList[tup.target.index].computeTangent(tup.target);
                            }
                            else
                            {

                                branch.target.myMasonry.elemList[tup.target.index].computeTangent(tup.target);
                            }
                        }
                    }
                  
                    foreach (var branch in _listBranch)
                    {
                        branch.airyCrv = branch.crv.Duplicate() as NurbsCurve;
                        for (int j = 0; j < branch.N; j++)
                        {
                            var P = branch.crv.Points[j];
                            branch.airyCrv.Points.SetPoint(j,new Point3d(P.Location.X, P.Location.Y, xx[j + branch.varOffset]));
                        }
                        if (branch.branchType == branch.type.reinforce)
                        {
                            var plane = new Plane(new Point3d(0, 0,xx[branch.N+branch.tuples.Count()+branch.varOffset]), new Vector3d(0, 0, 1));
                            listSlice[0].update(plane);
                        }
                        for(int i=0;i<branch.tuples.Count();i++)
                        {
                            //branch.tuples[i].z = branch.airyCrv.PointAt(branch.tuples[i].t).Z;
                            //int D = i + branch.N;
                            if (branch.branchType == branch.type.open)
                            {
                                branch.tuples[i].H[0, 0] = branch.tuples[i].target.valD - branch.tuples[i].target.valDc;
                            }
                            else if (branch.branchType==branch.type.reinforce)
                            {
                                branch.tuples[i].H[0, 0] = branch.tuples[i].target.valD;
                            }
                            else if (branch.branchType == branch.type.fix)
                            {
                                branch.tuples[i].H[0, 0] = 0;
                            }
                            else
                            {
                                branch.tuples[i].H[0, 0] = branch.tuples[i].left.valD + branch.tuples[i].right.valD;
                            }
                        }
                    }
                    foreach (var leaf in _listLeaf)
                    {
                        leaf.airySrf = leaf.srf.Duplicate() as NurbsSurface;
                        for (int j = 0; j < leaf.nV; j++)
                        {
                            for (int i = 0; i < leaf.nU; i++)
                            {
                                var P = leaf.srf.Points.GetControlPoint(i, j);
                                leaf.airySrf.Points.SetControlPoint(i, j, new ControlPoint(P.Location.X, P.Location.Y, xx[i + j * leaf.nU  + leaf.varOffset]));
                            }
                        }
                    }
                }
            }
        }
        void hodgeStar(List<leaf> _listLeaf, List<branch> _listBranch, List<node> _listNode, Func<double, double> coeff)
        {
            foreach (var branch in _listBranch)
            {
                for (int i = 0; i < branch.tuples.Count(); i++)
                {
                    double g = branch.tuples[i].gij[0, 0];
                    double val = coeff(g);
                    branch.tuples[i].SPK[0, 0] = branch.tuples[i].H[0, 0] * val;
                }
            }
            foreach (var leaf in _listLeaf)
            {
                for (int j = 0; j < leaf.r; j++)
                {
                    //Hodge star
                    double g = leaf.tuples[j].refDv * leaf.tuples[j].refDv;

                    leaf.tuples[j].SPK[0, 0] = leaf.tuples[j].H[1, 1] / g;//xx[N22 + leaf.varOffset] / g * root2;
                    leaf.tuples[j].SPK[1, 1] = leaf.tuples[j].H[0, 0] / g;//xx[N11 + leaf.varOffset] / g * root2;
                    leaf.tuples[j].SPK[0, 1] = -leaf.tuples[j].H[0, 1] / g;//-xx[N12 + leaf.varOffset] / g;
                    leaf.tuples[j].SPK[1, 0] = -leaf.tuples[j].H[1, 0] / g;//-xx[N12 + leaf.varOffset] / g;

                    leaf.tuples[j].computeEigenVectors();
                }
            }
            //For visualization
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

        }
    }
}
