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
        public void mosek1(leaf leaf)
        {
            int numvar = leaf.nU * leaf.nV;
            int numcon = 0;// leaf.r;
            
            // Since the value infinity is never used, we define
            // 'infinity' symbolic purposes only
            double infinity = 0;

            //mosek.boundkey[] bkc = { mosek.boundkey.fx, mosek.boundkey.fx, mosek.boundkey.fx, mosek.boundkey.fx, mosek.boundkey.fx };  //fix: fix to lower bound
            //double[] blc = { 0, 0, 0, 0, 10 };
            //double[] buc = { 0, 0, 0, 0, 10 };

            mosek.boundkey[] bkx = new mosek.boundkey[numvar];
            double[] blx = new double[numvar];
            double[] bux = new double[numvar];
            for (int i = 0; i < numvar; i++)
            {
                bkx[i] = mosek.boundkey.ra;
                blx[i] = 2;
                bux[i] = 8;
            }
            int[] fxP = { 0, leaf.nU - 1, leaf.nU * leaf.nV - leaf.nU, leaf.nU * leaf.nV - 1 };
            int centerP = leaf.nU / 2 + (leaf.nV / 2) * leaf.nU;
            foreach (int i in fxP)
            {
                bkx[i] = mosek.boundkey.fx;
                blx[i] = 0;
                bux[i] = 0;
            }
            bkx[centerP] = mosek.boundkey.fx;
            blx[centerP] = 10;
            bux[centerP] = 10;
            /*mosek.boundkey[] bkx = {mosek.boundkey.lo,
                            mosek.boundkey.lo,
                            mosek.boundkey.lo,
                            mosek.boundkey.fr,          
                            mosek.boundkey.fr,
                            mosek.boundkey.fr};
            double[] blx = { 0.0,
                     0.0,
                     0.0,
                     -infinity,
                     -infinity,
                     -infinity};
            double[] bux = { +infinity,
                     +infinity,
                     +infinity,
                     +infinity,
                     +infinity,
                     +infinity};
            */
/*            double[] c = { 0.0,
                     0.0,
                     0.0,
                     1.0,
                     1.0,
                     1.0};

            double[][] aval = {new double[] {1.0},
                         new double[] {1.0},
                         new double[] {2.0}};

            int[][] asub = {new int[] {0},
                         new int[] {0},
                         new int[] {0}};

            int[] csub = new int[3];
            */
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
                        task.putcj(j, 1);
                        /* Set the bounds on variable j.
                               blx[j] <= x_j <= bux[j] */
                        task.putvarbound(j, bkx[j], blx[j], bux[j]);
                    }
                    /*
                    for (int j = 0; j < aval.Length; ++j)
                        // Input column j of A 
                        task.putacol(j,          // Variable (column) index.
                                     asub[j],     // Row index of non-zeros in column j.
                                     aval[j]);    // Non-zero Values of column j. 
                    */
                    /* Set the bounds on constraints.
                         for i=1, ...,numcon : blc[i] <= constraint i <= buc[i] */
                    //for (int i = 0; i < numcon; ++i)
                    //    task.putconbound(i, bkc[i], blc[i], buc[i]);

                    /*CONE*/
                    /*
                    csub[0] = 3;
                    csub[1] = 0;
                    csub[2] = 1;
                    task.appendcone(mosek.conetype.quad,
                                    0.0, // For future use only, can be set to 0.0 
                                    csub);
                    csub[0] = 4;
                    csub[1] = 5;
                    csub[2] = 2;
                    task.appendcone(mosek.conetype.rquad, 0.0, csub);
                    */
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
                        case mosek.solsta.near_optimal:
                            Console.WriteLine("Optimal primal solution\n");
                            for (int j = 0; j < numvar; ++j)
                                Console.WriteLine("x[{0}]: {1}", j, xx[j]);
                            break;
                        case mosek.solsta.dual_infeas_cer:
                        case mosek.solsta.prim_infeas_cer:
                        case mosek.solsta.near_dual_infeas_cer:
                        case mosek.solsta.near_prim_infeas_cer:
                            Console.WriteLine("Primal or dual infeasibility.\n");
                            break;
                        case mosek.solsta.unknown:
                            Console.WriteLine("Unknown solution status.\n");
                            break;
                        default:
                            Console.WriteLine("Other solution status");
                            break;

                    }
                }
            }
        }

    
        private static double epsilon = 0.2;
        Func<double, double, double, double, double> F = (x1, x2, y1, y2) => { return Math.Sqrt(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)))); };
        Action<double, double, double, double, double[]> dF = (x1, x2, y1, y2, grad) =>
        {
            grad[0] = epsilon * (x1 - x2) / Math.Sqrt(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2))));
            grad[1] = epsilon * (y1 - y2) / Math.Sqrt(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2))));
        };
        Action<double, double, double, double, double[,]> ddF = (x1, x2, y1, y2, grad) =>
        {
            grad[0, 0] = epsilon / Math.Sqrt(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)))) - epsilon * epsilon * (x1 - x2) * (x1 - x2) * Math.Pow(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2))), -3 / 2d);
            grad[0, 1] = -epsilon * epsilon * (x1 - x2) * (y1 - y2) * Math.Pow(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2))), -3 / 2d);
            grad[1, 0] = -epsilon * epsilon * (x1 - x2) * (y1 - y2) * Math.Pow(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2))), -3 / 2d);
            grad[1, 1] = epsilon / Math.Sqrt(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)))) - epsilon * epsilon * (y1 - y2) * (y1 - y2) * Math.Pow(1 + epsilon * (((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2))), -3 / 2d);
        };
        public void computeBaseFunction(int lastComputed)
        {
            if (lastComputed <4)
            {
                computeBaseFunction1(lastComputed);
            }
        }

        private void computeBaseFunctionCommon(int lastComputed, DoubleArray origX, leaf leaf)
        {
            var M = (leaf.shiftArray.T.Multiply(leaf.Laplacian) as SparseDoubleArray) * leaf.shiftArray as SparseDoubleArray;
            int T1 = leaf.n - leaf.fixedPoints.Count;
            int T2 = leaf.n;
            var slice1 = new SparseDoubleArray(T1, T2);
            var slice2 = new SparseDoubleArray(T2, T2 - T1);
            for (int i = 0; i < T1; i++)
            {
                slice1[i, i] = 1;
            }
            for (int i = 0; i < T2 - T1; i++)
            {
                slice2[i + T1, i] = 1;
            }
            var DIB = (slice1.Multiply(M) as SparseDoubleArray).Multiply(slice2) as SparseDoubleArray;
            var DII = (slice1.Multiply(M) as SparseDoubleArray).Multiply(slice1.T) as SparseDoubleArray;
            var solver = new SparseLU(DII);
            origX = leaf.shiftArray.T * origX;
            var fixX = origX.GetSlice(T1, T2 - 1, 0, 0);
            var B = -DIB * fixX;
            var dx = solver.Solve(B);
            var ret = DoubleArray.Zeros(leaf.n, 1);
            for (int i = 0; i < T1; i++)
            {
                ret[i, 0] = dx[i, 0];
            }
            for (int i = T1; i < T2; i++)
            {
                ret[i, 0] = fixX[i - T1, 0];
            }
            if (lastComputed < leaf.baseFunction.Length)
            {
                leaf.baseFunction[lastComputed] = leaf.shiftArray * ret;
            }
            //vertices...x,y
            //baseFunction[]...z
            var MM = DoubleArray.Zeros(leaf.n, leaf.n);
            for (int i = 0; i < leaf.n; i++)
            {
                for (int j = 0; j < leaf.n; j++)
                {
                    MM[i, j] = F(leaf.vertices[i].X, leaf.vertices[j].X, leaf.vertices[i].Y, leaf.vertices[j].Y);
                }
            }
            var V = DoubleArray.Zeros(leaf.n, 1);
            for (int i = 0; i < leaf.n; i++)
            {
                V[i, 0] = leaf.baseFunction[lastComputed][i, 0];
            }
            //var solver2 = new LU(MM);
            var solver2 = new SVD(MM);
            leaf.coeff[lastComputed] = solver2.Solve(V);
            leaf.Function[lastComputed] = (x, y) =>
            {
                double val = 0;
                for (int j = 0; j < leaf.n; j++)
                {
                    val += leaf.coeff[lastComputed][j, 0] * F(x, leaf.vertices[j].X, y, leaf.vertices[j].Y);
                }
                return val;
            };
            leaf.dFunction[lastComputed] = (x, y, val) =>
            {
                val[0] = 0;
                val[1] = 0;
                double[] r = new double[2] { 0, 0 };
                for (int j = 0; j < leaf.n; j++)
                {
                    dF(x, leaf.vertices[j].X, y, leaf.vertices[j].Y, r);
                    val[0] += leaf.coeff[lastComputed][j, 0] * r[0];
                    val[1] += leaf.coeff[lastComputed][j, 0] * r[1];
                }
            };
            leaf.ddFunction[lastComputed] = (x, y, val) =>
            {
                val[0, 0] = 0;
                val[0, 1] = 0;
                val[1, 0] = 0;
                val[1, 1] = 0;
                double[,] r = new double[2, 2] { { 0, 0 }, { 0, 0 } };
                for (int j = 0; j < leaf.n; j++)
                {
                    ddF(x, leaf.vertices[j].X, y, leaf.vertices[j].Y, r);
                    val[0, 0] += leaf.coeff[lastComputed][j, 0] * r[0, 0];
                    val[0, 1] += leaf.coeff[lastComputed][j, 0] * r[0, 1];
                    val[1, 0] += leaf.coeff[lastComputed][j, 0] * r[1, 0];
                    val[1, 1] += leaf.coeff[lastComputed][j, 0] * r[1, 1];
                }
            };
        }
        private void computeBaseFunction1(int lastComputed)
        {
            foreach (var leaf in listLeaf)
            {
                DoubleArray origX = new DoubleArray(leaf.n, 1);
                for (int i = 0; i < leaf.n; i++)
                {
                    origX[i, 0] = 0;
                }
                var b = leaf.bbOut[lastComputed];
                var T0 = leaf.vertices[b[0].P0];
                var T1 = leaf.vertices[b.Last().P1];
                double Length = Math.Sqrt((T1.X - T0.X) * (T1.X - T0.X) + (T1.Y - T0.Y) * (T1.Y - T0.Y));
                for (int i = 0; i < b.Count; i++)
                {
                    var TS = leaf.vertices[b[i].P0];
                    double L = Math.Sqrt((TS.X - T0.X) * (TS.X - T0.X) + (TS.Y - T0.Y) * (TS.Y - T0.Y));
                    double y = Math.Pow((Length/2d-L)/(Length/2d), 2) - 1d;
                    origX[b[i].P0, 0] = -3d * y;
                    if (i == b.Count - 1)
                    {
                        y = Math.Pow(((i + 1d) - ((double)b.Count / 2d)) / ((double)b.Count / 2d), 2) - 1d;
                        origX[b[i].P1, 0] = -3d * y;
                    }
                }
                computeBaseFunctionCommon(lastComputed, origX,leaf);
            }
            if (lastComputed == 0) resultToPreview(0);
        }
        public void resultToPreview(int num)
        {
            a = new List<Point3d>();
            foreach (var leaf in listLeaf)
            {
                if (leaf.baseFunction[num] == null) { return; }
                var xx = leaf.baseFunction[num];
                leaf.result = new List<Line>();
                foreach (var edge in leaf.edges)
                {
                    Rhino.Geometry.Point3d P = new Rhino.Geometry.Point3d(leaf.vertices[edge.P0].X, leaf.vertices[edge.P0].Y, xx[edge.P0]);
                    Rhino.Geometry.Point3d Q = new Rhino.Geometry.Point3d(leaf.vertices[edge.P1].X, leaf.vertices[edge.P1].Y, xx[edge.P1]);
                    leaf.result.Add(new Rhino.Geometry.Line(P, Q));
                }
                foreach (var V in leaf.vertices)
                {
                    var P = leaf.srf.PointAt(V.X * leaf.scaleU + leaf.originU, V.Y * leaf.scaleV + leaf.originV);
                    a.Add(new Point3d(P.X,P.Y,leaf.Function[num](V.X,V.Y)));
                }
            }
        }
        public SparseDoubleArray computeLaplacian(int[,] lines, int nP)
        {
            if (lines.GetLength(1) != 2) return null;
            SparseDoubleArray C = new SparseDoubleArray(lines.GetLength(0), nP);
            int m = lines.GetLength(0);
            for (int i = 0; i < m; i++)
            {
                int P = lines[i, 0];
                int Q = lines[i, 1];
                C[i, P] = 1;
                C[i, Q] = -1;
            }
            SparseDoubleArray D = (C.T * C) as SparseDoubleArray;
            return D;
        }
        public SparseDoubleArray computeShiftArray(List<int> fixedPoints,int n)
        {
            int[] shift = new int[n];

            int NN = fixedPoints.Count();
            int DD = n - NN; //number of free nodes
            for (int i = 0; i < n; i++)
            {
                shift[i] = -100;
            }
            for (int i = 0; i < NN; i++)
            {
                shift[fixedPoints[i]] = DD + i;
            }
            int S = 0;
            for (int i = 0; i < n; i++)
            {
                if (shift[i] == -100)
                {
                    shift[i] = S;
                    S++;
                }
            }
            if (S != DD) AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, "contradiction!");
            var shiftArray = new SparseDoubleArray(n, n);
            for (int i = 0; i < n; i++)
            {
                shiftArray[i, shift[i]] = 1;
            }
            return shiftArray;
        }
    }
}
