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
            //Mesher edges
            if (f != null)
            {
                foreach (var l in f)
                {
                    args.Display.DrawLine(l, System.Drawing.Color.Bisque);
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
                if (leaf.result != null)
                {
                    args.Display.DrawLines(leaf.result, System.Drawing.Color.Yellow);
                }
                if (leaf.airySrf != null)
                {
                    args.Display.DrawSurface(leaf.airySrf, System.Drawing.Color.Violet, 2);
                }
            }
            /*if (a != null)
            {
                args.Display.DrawPoints(a, Rhino.Display.PointStyle.X, 2, System.Drawing.Color.Blue);

            }*/
        }
    }
}
