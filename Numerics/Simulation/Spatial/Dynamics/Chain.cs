using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using System.Drawing;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial.Solvers
{
    using Geometry;
    using JA.UI;

    /// <summary>
    /// Simulation chain consisting of a open tree structure of links joined together.
    /// </summary>
    /// <seealso cref="Spatial.Object" />
    /// <seealso cref="Spatial.Solvers.Link3}" />
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Chain : Object,
        INotifyPropertyChanged
    {
        readonly List<Link3> linkList;

        public Chain(Chain copy) : this(copy.World, copy.linkList)
        {
            ContactNormal = copy.ContactNormal;
            ContactPoint = copy.ContactPoint;
        }
        public Chain(World3 world, params Link3[] links) 
            : this(world, links.AsEnumerable())
        { }
        public Chain(World3 world, IEnumerable<Link3> links) 
            : base(world)
        {
            linkList = new List<Link3>();
            linkList.AddRange(links);
            ContactNormal = Vector3.Zero;
            ContactPoint = Vector3.Zero;
        }
        public Chain(int count, Body body, Pose onParent, JointProperties joint)
            : base(body.World)
        {
            linkList = new List<Link3>(count);
            for (int i = 0; i < count; i++)
            {
                AddLink(body, onParent, joint);
            }
            ContactNormal = Vector3.UnitY;
            ContactPoint = count*onParent.Position;
        }

        public Link3 AddLink(Body body, Pose onParent, JointProperties joint)
        {
            return linkList.Count > 0 ? 
                new Link3(body, linkList[linkList.Count-1], onParent, joint) : 
                new Link3(body, this, Pose.Origin, joint);
        }
        public Link3 AddChild(Mesh mesh, Pose meshPosition, float mass, Pose onWorld, JointProperties type, float initialPosition = 0)
            => AddChild(mesh, meshPosition, mesh.GetMmoiFromMass(mass), onWorld, type, initialPosition);
        public Link3 AddChild(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onWorld, JointProperties type, float initialPosition = 0)
            => new Link3(mesh, this, meshPosition, mass, onWorld, type, initialPosition);
        public Link3 AddChild(Body body, Pose onParent, JointProperties type, float initialDisplacement = 0)
            => new Link3(body, this, onParent, type, initialDisplacement);

        public Pose Pivot
        {
            get => linkList.First().LocationOnParent;
            set
            {
                ContactPoint -= linkList.First().LocationOnParent.Position;
                linkList.First().LocationOnParent = value;
                ContactPoint += value.Position;
            }
        }

        #region Notify Property
        public event PropertyChangedEventHandler PropertyChanged;
        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        #endregion

        #region Property
        [Browsable(false)] public Link3 this[int index] { get => linkList[index]; }
        [Browsable(false)] public IReadOnlyList<Link3> Links { get => linkList; }
        [Browsable(false)] public int IndexOf(Link3 link) => linkList.IndexOf(link); 
        [Browsable(false)] public IList<Link3> ChildrenOf(Link3 link) => linkList.Where( f=>f.Parent == link).ToList();
        /// <summary>
        /// Direction of Contact
        /// </summary>
        [Category("Simulation")] public Vector3 ContactNormal { get; set; }
        /// <summary>
        /// Location of Contact
        /// </summary>
        [Category("Simulation")] public Vector3 ContactPoint { get; set; }

        #endregion

        #region Methods
        internal void AddLink(Link3 frame)
        {
            linkList.Add(frame);
        }
        public Link3 SlideAlongX(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0)
            => AddChild(mesh, localPosition, mass, onWorld, JointType.SlideAlongX, initialDisplacement);
        public Link3 SlideAlongY(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0)
            => AddChild(mesh, localPosition, mass, onWorld, JointType.SlideAlongY, initialDisplacement);
        public Link3 AddRevolute(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialAngle = 0)
            => AddChild(mesh, localPosition, mass, onWorld, JointType.RotateAboutZ, initialAngle);
        public Link3 AddCollarAlongX(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(null, localPosition, MassProperties.Zero, onWorld, JointType.SlideAlongX, initialDisplacement)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);
        public Link3 AddCollarAlongY(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(null, localPosition, MassProperties.Zero, onWorld, JointType.SlideAlongY, initialDisplacement)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);
        public Link3 AddFree(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialX, float intialY, float initialAngle)
            => AddChild(null, localPosition, MassProperties.Zero, onWorld, JointType.SlideAlongX, initialX)
                    .AddChild(null, Pose.Origin, MassProperties.Zero, Pose.Origin, JointType.SlideAlongY, intialY)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);

        public void RemoveChild(Link3 child)
        {
            foreach (var item in linkList.Where(f => f.Parent == child))
            {
                item.SetParent(null);
            }
            linkList.Remove(child);
        }

        public override void ConvertTo(UnitSystem target)
        {
            for (int i = 0; i < linkList.Count; i++)
            {
                linkList[i].ConvertTo(target);
            }
            ContactPoint *= Unit.Length.Convert(Units, target);
            base.ConvertTo(target);
        }
        public override string ToString()
        {
            return $"Chain(Count={linkList.Count})";
        }
        #endregion
        public override void Render(Graphics g, Camera camera, Pose pose, float scale = 1)
        {
            for (int i = 0; i < linkList.Count; i++)
            {
                pose = Pose.FromLocal(pose, linkList[i].LocationOnParent);
                pose = Pose.FromLocal(pose, linkList[i].JointProperties.GetLocalStep(linkList[i].InitialDisplacement));
                linkList[i].Render(g, camera, pose, scale);
            }
        }
    }
}
