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
        }
    }
}
