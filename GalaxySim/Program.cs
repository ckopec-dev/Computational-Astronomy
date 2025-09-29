using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using System.Numerics;

namespace GalaxySim
{
    public class Particle(Vector2 position, Vector2 velocity, float mass, bool isStar = false)
    {
        public Vector2 Position = position;
        public Vector2 Velocity = velocity;
        public float Mass = mass;
        public bool IsStar = isStar;
    }

    public class BoundingBox(Vector2 center, float size)
    {
        public Vector2 Center = center;
        public float Size = size;

        public bool Contains(Vector2 point)
        {
            return Math.Abs(point.X - Center.X) <= Size / 2 && Math.Abs(point.Y - Center.Y) <= Size / 2;
        }
    }

    public class BHNode(BoundingBox bounds)
    {
        public BoundingBox Bounds = bounds;
        public BHNode[] Children = new BHNode[4];
        public Particle? Particle;
        public Vector2 CenterOfMass;
        public float Mass;
        public bool HasChildren => Children[0] != null;

        public void Insert(Particle p)
        {
            // Ignore particles outside bounds
            if (!Bounds.Contains(p.Position))
                return;

            // If this node has no particle and no children, store particle here
            if (Particle == null && !HasChildren)
            {
                Particle = p;
                Mass = p.Mass;
                CenterOfMass = p.Position;
                return;
            }

            // If this node already has one particle and no children, subdivide
            if (!HasChildren)
                Subdivide();

            // If a particle already exists, push it down
            if (Particle != null)
            {
                InsertIntoChild(Particle);
                Particle = null;
            }

            // Insert new particle
            InsertIntoChild(p);

            // Update mass and COM
            Mass += p.Mass;
            CenterOfMass = (CenterOfMass * (Mass - p.Mass) + p.Position * p.Mass) / Mass;
        }

        private void Subdivide()
        {
            float half = Bounds.Size / 2f;
            float quarter = half / 2f;
            Vector2 c = Bounds.Center;

            Children[0] = new BHNode(new BoundingBox(new Vector2(c.X - quarter, c.Y - quarter), half)); // bottom-left
            Children[1] = new BHNode(new BoundingBox(new Vector2(c.X + quarter, c.Y - quarter), half)); // bottom-right
            Children[2] = new BHNode(new BoundingBox(new Vector2(c.X - quarter, c.Y + quarter), half)); // top-left
            Children[3] = new BHNode(new BoundingBox(new Vector2(c.X + quarter, c.Y + quarter), half)); // top-right
        }

        private void InsertIntoChild(Particle p)
        {
            foreach (var child in Children)
            {
                if (child.Bounds.Contains(p.Position))
                {
                    child.Insert(p);
                    return;
                }
            }
        }

        public Vector2 ComputeForce(Particle p, float theta, float G)
        {
            if (Mass == 0 || (Particle != null && Particle == p)) return Vector2.Zero;

            Vector2 dir = CenterOfMass - p.Position;
            float dist = dir.Length() + 1e-5f;

            // If sufficiently far away, approximate as one mass
            if (!HasChildren || (Bounds.Size / dist) < theta)
            {
                return G * Mass / (dist * dist) * Vector2.Normalize(dir);
            }
            else
            {
                Vector2 total = Vector2.Zero;
                foreach (var child in Children)
                {
                    if (child != null)
                        total += child.ComputeForce(p, theta, G);
                }
                return total;
            }
        }
    }

    public static class GalaxySimulation
    {
        const float G = 0.1f;
        const float dt = 0.1f;
        static readonly Random rng = new();

        public static void Run()
        {
            int n = 500;
            List<Particle> particles = InitializeDisk(n, 1000f);
            int steps = 5000;

            Directory.CreateDirectory("snapshots");

            for (int step = 0; step < steps; step++)
            {
                // Build Barnes-Hut tree
                BHNode root = new(new BoundingBox(Vector2.Zero, 4000f));
                foreach (var p in particles)
                    root.Insert(p);

                // Compute forces
                List<Vector2> forces = new(particles.Count);
                foreach (var p in particles)
                    forces.Add(root.ComputeForce(p, 0.5f, G));

                // Integrate
                for (int i = 0; i < particles.Count; i++)
                {
                    var p = particles[i];
                    p.Velocity += forces[i] * dt;
                    p.Position += p.Velocity * dt;
                }

                if (step % 10 == 0)
                {
                    SaveSnapshotImage(particles, step);
                    Console.WriteLine($"Step {step}/{steps}");
                }
            }
        }

        static List<Particle> InitializeDisk(int n, float radius)
        {
            var list = new List<Particle>();
            for (int i = 0; i < n; i++)
            {
                double r = Math.Sqrt(rng.NextDouble()) * radius;
                double theta = rng.NextDouble() * 2 * Math.PI;
                Vector2 pos = new((float)(r * Math.Cos(theta)), (float)(r * Math.Sin(theta)));
                Vector2 vel = new Vector2((float)(-Math.Sin(theta)), (float)(Math.Cos(theta))) * (float)Math.Sqrt(G * n / (r + 1));
                list.Add(new Particle(pos, vel, 1f));
            }
            return list;
        }

        static void SaveSnapshotImage(List<Particle> particles, int step)
        {
            int size = 800;
            using var image = new Image<Rgba32>(size, size, Color.Black);
            float scale = size / 4000f;
            Vector2 center = new(size / 2f);

            foreach (var p in particles)
            {
                int x = (int)(center.X + p.Position.X * scale);
                int y = (int)(center.Y + p.Position.Y * scale);
                if (x >= 0 && y >= 0 && x < size && y < size)
                    image[x, y] = Color.White;
            }

            string path = Path.Combine("snapshots", $"snap_{step:D4}.png");
            image.Save(path);
        }
    }

    class Program
    {
        static void Main()
        {
            GalaxySimulation.Run();
        }
    }
}
