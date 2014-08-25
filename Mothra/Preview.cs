using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace mikity.ghComponents
{
    public partial class Mothra3 : Grasshopper.Kernel.GH_Component
    {

        public override void DrawViewportWires(Grasshopper.Kernel.IGH_PreviewArgs args)
        {
            if (Hidden)
            {
                return;
            }
            //eigenvectors
            if (crossCyan != null)
            {
                args.Display.DrawLines(crossCyan, System.Drawing.Color.Cyan);
            }
            if (crossMagenta != null)
            {
                args.Display.DrawLines(crossMagenta, System.Drawing.Color.Magenta);
            }
            /*if (listSlice != null)
            {
                foreach (var slice in listSlice)
                {
                    if (slice != listSlice.First())
                    {
                        var pl = slice.pl;
                        args.Display.DrawPolygon(new Rhino.Geometry.Point3d[] { pl.PointAt(-5, -5), pl.PointAt(-5, 5), pl.PointAt(5, 5), pl.PointAt(5, -5) }, System.Drawing.Color.Azure, true);
                    }
                }
            }*/
            foreach (var branch in listBranch)
            {
                switch (branch.branchType)
                {
                    case Mothra3.branch.type.fix:
                        args.Display.DrawCurve(branch.crv, System.Drawing.Color.Brown, 2);
                        break;
                    case Mothra3.branch.type.reinforce:
                        args.Display.DrawCurve(branch.crv, System.Drawing.Color.Pink, 2);
                        break;
                    case Mothra3.branch.type.kink:
                        args.Display.DrawCurve(branch.crv, System.Drawing.Color.Purple, 2);
                        break;
                    case Mothra3.branch.type.open:
                        args.Display.DrawCurve(branch.crv, System.Drawing.Color.Green, 2);
                        break;
                }
            }
            foreach (var leaf in listLeaf)
            {
                if (currentAiry < 4)
                {
                    if (leaf.airySrf[currentAiry] != null)
                    {
                        args.Display.DrawSurface(leaf.airySrf[currentAiry], System.Drawing.Color.Brown, 3);
                    }
                }
                else
                {
                    if(leaf.airySrf!=null)
                        args.Display.DrawSurface(leaf.airySrfCombined, System.Drawing.Color.Brown, 3);
                }
            }
            foreach (var leaf in listLeaf)
            {
                foreach (var tup in leaf.edgeTuples)
                {
                    args.Display.DrawLine(new Rhino.Geometry.Line(new Rhino.Geometry.Point3d(tup.x, tup.y, tup.z), new Rhino.Geometry.Point3d(tup.x + tup.gi[0][0], tup.y + tup.gi[0][1], tup.z + tup.gi[0][2])), System.Drawing.Color.DarkCyan);
                    args.Display.DrawLine(new Rhino.Geometry.Line(new Rhino.Geometry.Point3d(tup.x, tup.y, tup.z), new Rhino.Geometry.Point3d(tup.x + tup.gi[1][0], tup.y + tup.gi[1][1], tup.z + tup.gi[1][2])), System.Drawing.Color.DarkCyan);
                }
            }
            foreach(var branch in listBranch)
            {
                if (branch.airyCrv != null)
                {
                    args.Display.DrawCurve(branch.airyCrv, System.Drawing.Color.SeaGreen, 4);
                }
            }
            if (listBranch != null)
            {
                foreach (var branch in listBranch)
                {
                    if (branch.branchType == branch.type.kink)
                    {
                        if (branch.tuples != null)
                        {
                            foreach (var tup in branch.tuples)
                            {
                                var D = (tup.left.valD + tup.right.valD)/50d;
                                if (D > 0)
                                    args.Display.DrawCircle(new Rhino.Geometry.Circle(branch.airyCrv.PointAt(tup.t), D), System.Drawing.Color.Pink);
                                else
                                    args.Display.DrawCircle(new Rhino.Geometry.Circle(branch.airyCrv.PointAt(tup.t), D), System.Drawing.Color.Yellow);
                            }
                        }
                    }
                    else /*if (branch.branchType != branch.type.fix)*/
                    {
                        if (branch.tuples != null)
                        {
                            foreach (var tup in branch.tuples)
                            {
                                var D = (tup.target.valD - tup.target.valDc)/50d;
                                if (D > 0)
                                    args.Display.DrawCircle(new Rhino.Geometry.Circle(branch.airyCrv.PointAt(tup.t), D), System.Drawing.Color.Pink);
                                else
                                    args.Display.DrawCircle(new Rhino.Geometry.Circle(branch.airyCrv.PointAt(tup.t), D), System.Drawing.Color.Yellow);
                            }
                        }
                    }
                }
            }
            /*
            if (a != null)
            {
                args.Display.DrawPoints(a, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Blue);

            }*/
            if (a2 != null)
            {
                args.Display.DrawPoints(a2, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Blue);

            }
        }
    }
}
