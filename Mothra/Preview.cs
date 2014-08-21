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
            if (listSlice != null)
            {
                foreach (var slice in listSlice)
                {
                    var pl = slice.pl;
                    args.Display.DrawPolygon(new Rhino.Geometry.Point3d[]{pl.PointAt(-5, -5), pl.PointAt(-5, 5), pl.PointAt(5, 5), pl.PointAt(5, -5)},System.Drawing.Color.Azure,true);
                }
            }
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
                if (leaf.gmesh != null)
                {
                    args.Display.DrawMeshWires(leaf.gmesh, System.Drawing.Color.Gray);
                }
                if (leaf.triEdges != null)
                {
                    for (int i = 0; i < leaf.triEdges.Count(); i++)
                    {
                        var boundary = leaf.triEdges[i];
                        if (boundary != null)
                        {
                            System.Drawing.Color color = System.Drawing.Color.Red;
                            if (i == 0) color = System.Drawing.Color.Red;
                            if (i == 1) color = System.Drawing.Color.Blue;
                            if (i == 2) color = System.Drawing.Color.Green;
                            if (i == 3) color = System.Drawing.Color.Yellow;
                            if (i == 4) color = System.Drawing.Color.Orange;
                            if (i == 5) color = System.Drawing.Color.Purple;
                            if (i == 6) color = System.Drawing.Color.Brown;
                            foreach (var bb in boundary)
                            {
                                args.Display.DrawLine(bb, color, 5);
                            }
                        }
                    }
                }
                /*
                if (leaf.result != null)
                {
                    args.Display.DrawLines(leaf.result, System.Drawing.Color.Yellow);
                }*/
                if (leaf.airySrf[currentAiry] != null)
                {
                    args.Display.DrawSurface(leaf.airySrf[currentAiry], System.Drawing.Color.Brown, 3);
                }
            }
            foreach(var branch in listBranch)
            {
                if (branch.airyCrv != null)
                {
                    args.Display.DrawCurve(branch.airyCrv, System.Drawing.Color.SeaGreen, 4);
                }
            }/*
            if (a != null)
            {
                args.Display.DrawPoints(a, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Blue);

            }*/
        }
    }
}
