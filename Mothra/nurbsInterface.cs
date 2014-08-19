using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace mikity.ghComponents
{
    public partial class Mothra3 : Grasshopper.Kernel.GH_Component
    {
        void Nurbs2x(leaf leaf, double[,] _x)
        {
            for (int i = 0; i < leaf.nV; i++)
            {
                for (int j = 0; j < leaf.nU; j++)
                {
                    _x[(i * leaf.nU + j), 0] = leaf.srf.Points.GetControlPoint(j, i).Location.X;
                    _x[(i * leaf.nU + j), 1] = leaf.srf.Points.GetControlPoint(j, i).Location.Y;
                    _x[(i * leaf.nU + j), 2] = leaf.srf.Points.GetControlPoint(j, i).Location.Z;
                }
            }
        }
        void createNurbsElements(leaf leaf)
        {
            double[] uKnot;
            double[] vKnot;

            int N = leaf.nU * leaf.nV;
            int uDim = leaf.srf.OrderU;
            int vDim = leaf.srf.OrderV;
            int uDdim = leaf.srf.OrderU - 1;
            int vDdim = leaf.srf.OrderV - 1;


            uKnot = new double[leaf.nU - uDdim + 1 + uDdim * 2];
            vKnot = new double[leaf.nV - vDdim + 1 + vDdim * 2];
            for (int i = 0; i < uDdim; i++)
            {
                uKnot[i] = 0;
            }
            for (int i = 0; i < vDdim; i++)
            {
                vKnot[i] = 0;
            }
            for (int i = 0; i < leaf.nU - uDdim + 1; i++)
            {
                uKnot[i + uDdim] = i;
            }
            for (int i = 0; i < leaf.nV - vDdim + 1; i++)
            {
                vKnot[i + vDdim] = i;
            }
            for (int i = 0; i < uDdim; i++)
            {
                uKnot[i + leaf.nU + 1] = leaf.nU - uDdim;
            }
            for (int i = 0; i < vDdim; i++)
            {
                vKnot[i + leaf.nV + 1] = leaf.nV - vDdim;
            }
            leaf.myMasonry = new Minilla3D.Objects.masonry();
            for (int j = 1; j < leaf.nV - vDdim + 1; j++)
            {
                for (int i = 1; i < leaf.nU - uDdim + 1; i++)
                {
                    int[] index = new int[uDim * vDim];
                    for (int k = 0; k < vDim; k++)
                    {
                        for (int l = 0; l < uDim; l++)
                        {
                            index[k * uDim + l] = (j - 1 + k) * leaf.nU + i - 1 + l;
                        }
                    }
                    Minilla3D.Elements.nurbsElement.border _border = Minilla3D.Elements.nurbsElement.border.None;
                    /*
                    //judge if on the border
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
                    }*/
                    leaf.myMasonry.elemList.Add(new Minilla3D.Elements.nurbsElement(uDim, vDim, index, i, j, uKnot, vKnot, _border));
                    /*switch (_border)
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
                    }*/
                }
            }
        }
    }
}
