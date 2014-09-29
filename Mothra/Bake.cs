using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace mikity.ghComponents
{
    public partial class Mothra3 : Grasshopper.Kernel.GH_Component
    {
        public override void BakeGeometry(Rhino.RhinoDoc doc, Rhino.DocObjects.ObjectAttributes att, List<Guid> obj_ids)
        {
            Rhino.DocObjects.ObjectAttributes a2 = att.Duplicate();
            a2.LayerIndex = 2;
            Rhino.DocObjects.ObjectAttributes a3 = att.Duplicate();
            a3.LayerIndex = 3;
            Rhino.DocObjects.ObjectAttributes a4 = att.Duplicate();
            a4.LayerIndex = 4;
            Rhino.DocObjects.ObjectAttributes a5 = att.Duplicate();
            a5.LayerIndex = 5;
            foreach (var leaf in listLeaf)
            {
                Guid id = doc.Objects.AddSurface(leaf.airySrf, a2);
                obj_ids.Add(id);
                var srf = leaf.shellSrf.Duplicate() as Rhino.Geometry.NurbsSurface;
                srf.Transform(zDown);
                id = doc.Objects.AddSurface(srf, a3);
                obj_ids.Add(id);
            }
            foreach (var branch in listBranch)
            {
                Guid id = doc.Objects.AddCurve(branch.airyCrv, a2);
                obj_ids.Add(id);
                var crv = branch.shellCrv.Duplicate() as Rhino.Geometry.NurbsCurve;
                crv.Transform(zDown);
                id = doc.Objects.AddCurve(crv, a3);
                obj_ids.Add(id);

            }
            if (crossMagenta != null)
            {
                foreach (var line in crossMagenta)
                {
                    Guid id = doc.Objects.AddLine(line, a4);
                    obj_ids.Add(id);
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
                                //var D = (tup.left.valD + tup.right.valD)/50d;
                                var D = tup.H[0, 0] / 5d;
                                if (D > 0)
                                {
                                    var circle = new Rhino.Geometry.Circle(new Rhino.Geometry.Point3d(tup.x, tup.y, tup.z), D);
                                    circle.Transform(zDown);
                                    Guid id = doc.Objects.AddCircle(circle, a5);
                                    obj_ids.Add(id);
                                }
                            }
                        }
                    }
                    else /*if (branch.branchType != branch.type.fix)*/
                    {
                        if (branch.tuples != null)
                        {
                            foreach (var tup in branch.tuples)
                            {
                                //var D = (tup.target.valD - tup.target.valDc)/50d;
                                var D = tup.H[0, 0] / 5d;
                                if (D > 0)
                                {
                                    var circle = new Rhino.Geometry.Circle(new Rhino.Geometry.Point3d(tup.x, tup.y, tup.z), D);
                                    circle.Transform(zDown);
                                    Guid id = doc.Objects.AddCircle(circle, a5);
                                    obj_ids.Add(id);
                                }
                            }
                        }
                    }
                }
            }

        }
    }
}
