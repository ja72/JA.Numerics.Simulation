using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial
{

    public class Element
    {
        public Element(Color color, params int[] face)
        {
            Color = color;
            Face = face;
        }

        public int[] Face { get; }
        public Color Color { get; set; }

        public void Flip()
        {
            Array.Copy(Face.Reverse().ToArray(), Face, Face.Length);
        }
    }

    public class Mesh : IHasUnits<Mesh>
    {
        public Mesh(UnitSystem units = UnitSystem.MMKS)
        {
            Units = units;
            Nodes = new List<Vector3>();
            Elements = new List<Element>();
        }

        public Mesh(UnitSystem units, IEnumerable<Vector3> nodes, IEnumerable<Element> elements) : this(units)
        {
            Nodes = new List<Vector3>(nodes);
            Elements = new List<Element>(elements);
        }

        public UnitSystem Units { get; }
        public List<Vector3> Nodes { get; }
        public List<Element> Elements { get; }

        public void GetVolumeProperties(out float volume, out Vector3 center, out Matrix3 specificMmoi)
        {
            volume = 0;
            center = Vector3.Zero;
            specificMmoi = Matrix3.Zero;
            float dV;
            Vector3 dc;
            Matrix3 dI;
            for (int index = 0; index < Elements.Count; index++)
            {
                foreach (var trig in GetTriangles(index, Pose.Origin))
                {
                    dV = Vector3.Dot(trig.A, Vector3.Cross(trig.B, trig.C)) / 6;
                    dc = (trig.A + trig.B + trig.C) / 4;

                    dI = 6f * (
                        LinearAlgebra.Mmoi(trig.A + trig.B)
                        + LinearAlgebra.Mmoi(trig.B + trig.C)
                        + LinearAlgebra.Mmoi(trig.C + trig.A)) / 120;

                    volume += dV;
                    center += dV * dc;
                    specificMmoi += dV * dI;
                }
            }
            center /= volume;
            specificMmoi /= volume;

            specificMmoi -= LinearAlgebra.Mmoi(center);
        }
        public MassProperties GetMmoiFromMass(float mass)
        {
            GetVolumeProperties(out var volume, out var c, out var I);
            I *= mass;
            return new MassProperties(Units, mass, I, c);
        }
        public MassProperties GetMmoiFromDensity(float density)
        {
            GetVolumeProperties(out var volume, out var c, out var I);
            float mass = density * volume;
            I *= mass;
            return new MassProperties(Units, mass, I, c);
        }

        public void ApplyTranform(Pose origin)
        {
            for (int i = 0; i < Nodes.Count; i++)
            {
                Nodes[i] = Pose.FromLocal(origin, Nodes[i]);
            }
        }
        public void ReverseTranform(Pose origin)
        {
            for (int i = 0; i < Nodes.Count; i++)
            {
                Nodes[i] = Pose.ToLocal(Nodes[i], origin);
            }
        }

        /// <summary>
        /// Gets the coordinates of the nodes of a face, applying the mesh transformation.
        /// </summary>
        /// <param name="index">The face index.</param>
        public Vector3[] GetNodes(int index, Pose pose)
        {
            var R = Matrix4x4.CreateFromQuaternion(pose.Orientation);
            return Elements[index].Face.Select(ni => pose.Position + Vector3.Transform(Nodes[ni], R)).ToArray();
        }
        public Triangle[] GetTriangles(int index, Pose pose) => GetPolygon(index, pose).GetTriangles();
        public Polygon GetPolygon(int index, Pose pose) => new Polygon(GetNodes(index, pose));

        /// <summary>
        /// Gets the normal vector of a face, applying the mesh transformation.
        /// </summary>
        /// <param name="index">The face index.</param>
        public Vector3[] GetNormals(int index, Pose pose)
        {
            return GetNormals(GetNodes(index, pose));
        }
        /// <summary>
        /// Gets the normal vectors of a face at each node, applying the mesh transformation.
        /// </summary>
        /// <param name="nodes">The nodes of the face.</param>
        public Vector3[] GetNormals(Vector3[] nodes)
        {
            var normals = new Vector3[nodes.Length];
            for (int i = 0; i < nodes.Length; i++)
            {
                int j = (i + 1) % nodes.Length;
                int k = (i - 1 + nodes.Length) % nodes.Length;

                Vector3 A = nodes[i], B = nodes[j], C = nodes[k];

                normals[i] = (
                    Vector3.Cross(A, B)
                    + Vector3.Cross(B, C)
                    + Vector3.Cross(C, A)
                    ).Unit();
            }

            return normals;
        }
        /// <summary>
        /// Gets the average normal vector of a face, applying the mesh transformation.
        /// </summary>
        /// <param name="nodes">The nodes of the face.</param>
        public Vector3 GetNormal(Vector3[] nodes)
        {
            var list = GetNormals(nodes);
            Vector3 n = Vector3.Zero;
            for (int i = 0; i < list.Length; i++)
            {
                n += list[i];
            }
            return n.Unit();
        }

        /// <summary>
        /// Adds the face from a list of nodes.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="nodes">The local face nodes.</param>
        public void AddFace(Color color,
            params Vector3[] nodes)
        {
            var elemIndex = new int[nodes.Length];
            for (int i = 0; i < nodes.Length; i++)
            {
                if (Nodes.Contains(nodes[i]))
                {
                    elemIndex[i] = Nodes.IndexOf(nodes[i]);
                }
                else
                {
                    elemIndex[i] = Nodes.Count;
                    Nodes.Add(nodes[i]);
                }
            }
            Elements.Add(new Element(color, elemIndex));
        }
        /// <summary>
        /// Adds a square panel as a face.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="center">The center of the panel.</param>
        /// <param name="x_axis">The x-axis defining the direction of length.</param>
        /// <param name="length">The panel length.</param>
        /// <param name="width">The panel width.</param>
        public void AddPanel(Color color,
            Vector3 center,
            Vector3 x_axis,
            float length,
            float width)
        {
            x_axis = x_axis.Unit();
            Vector3 z_axis = center == Vector3.Zero ? Vector3.UnitZ : center.Unit();
            Vector3 y_axis = Vector3.Cross(z_axis, x_axis);

            AddFace(color,
                center - length / 2 * x_axis - width / 2 * y_axis,
                center + length / 2 * x_axis - width / 2 * y_axis,
                center + length / 2 * x_axis + width / 2 * y_axis,
                center - length / 2 * x_axis + width / 2 * y_axis);
        }

        /// <summary>
        /// Creates a cube mesh from 6 panels.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="sizeX">The size of the cube in the x-axis.</param>
        /// <param name="sizeY">The size of the cube in the y-axis.</param>
        /// <param name="sizeZ">The size of the cube in the z-axis.</param>
        public static Mesh CreateCube(Color color, float sizeX, float sizeY, float sizeZ)
        {
            var mesh = new Mesh();
            mesh.AddPanel(
                color,
                new Vector3(0, sizeY/2, 0),
                Vector3.UnitX,
                sizeX, sizeZ);
            mesh.AddPanel(
                color,
                new Vector3(0, -sizeY/2, 0),
                Vector3.UnitX,
                sizeX, sizeZ);
            mesh.AddPanel(
                color,
                new Vector3(sizeX/2, 0, 0),
                Vector3.UnitZ,
                sizeZ, sizeY);
            mesh.AddPanel(
                color,
                new Vector3(-sizeX/2, 0, 0),
                Vector3.UnitZ,
                sizeZ, sizeY);
            mesh.AddPanel(
                color,
                new Vector3(0, 0, sizeZ/2),
                Vector3.UnitX,
                sizeX, sizeY);
            mesh.AddPanel(
                color,
                new Vector3(0, 0, -sizeZ/2),
                Vector3.UnitX,
                sizeX, sizeY);
            return mesh;
        }

        public static Mesh CreateCube(Color color, float size)
            => CreateCube(color, size, size, size);

        /// <summary>
        /// Creates a square pyramid mesh from 5 panels.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="base">The size of the base.</param>
        /// <param name="height">The height of the pyramid.</param>
        public static Mesh CreatePyramid(Color color, float @base, float height)
        {
            var mesh = new Mesh();
            mesh.Nodes.Add(new Vector3(-@base/2, -@base/2, 0));
            mesh.Nodes.Add(new Vector3(@base/2, -@base/2, 0));
            mesh.Nodes.Add(new Vector3(@base/2, @base/2, 0));
            mesh.Nodes.Add(new Vector3(-@base/2, @base/2, 0));
            mesh.Elements.Add(new Element(color, 3, 2, 1, 0));
            mesh.Nodes.Add(height*Vector3.UnitZ);
            mesh.Elements.Add(new Element(color, 4, 0, 1));
            mesh.Elements.Add(new Element(color, 4, 1, 2));
            mesh.Elements.Add(new Element(color, 4, 2, 3));
            mesh.Elements.Add(new Element(color, 4, 3, 0));

            return mesh;
        }
        public Mesh ConvertTo(UnitSystem target)
        {
            float fl = UnitFactors.Length(Units, target);
            return new Mesh(target,
                Nodes.Select((n) => fl * n),
                Elements);
        }
    }
}
