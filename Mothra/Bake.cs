﻿using System;
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
            a2.LayerIndex = 1;
            foreach(var leaf in listLeaf)
            {
                Guid id = doc.Objects.AddSurface(leaf.airySrf, a2);
                obj_ids.Add(id);
            }
        }
    }
}
