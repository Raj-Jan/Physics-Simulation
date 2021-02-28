using System;
using System.Drawing;

namespace Project1.MBS
{
    public partial class Body : IBody
    {
        private bool changed = false;
        private Complex dir = new Complex(1, 0);
        private Vector pos = new Vector(0, 0);

        public Vector Pos
        {
            get => pos;
            set
            {
                changed = value.A != pos.A;

                pos = value;
            }
        }
        public Vector Vel { get; set; }
        public Vector Acc { get; set; }
        public Vector Force { get; set; }
        public Vector Mass { get; set; }

        public Complex Dir
        {
            get
            {
                if (changed)
                {
                    dir = new Complex
                    {
                        X = (float)Math.Cos(Pos.A),
                        Y = (float)Math.Sin(Pos.A),
                    };

                    changed = false;
                }

                return dir;
            }
        }

        public void Draw(Pen pen, Graphics graphics)
        {
            pen.Color = Color.White;

            graphics.FillEllipse(pen.Brush, Pos.X - 3, Pos.Y - 3, 6, 6);

            var s = 100 * Dir;

            pen.Width = 2;

            graphics.DrawLine(pen, Pos.X, Pos.Y, Pos.X + s.X, Pos.Y + s.Y);

            pen.Color = Color.Red;

            graphics.DrawLine(pen, Pos.X - s.X, Pos.Y - s.Y, Pos.X, Pos.Y);
        }
    }

    public interface ISystem
    {
        float this[int c] { set; }
        Vector this[int c, IBody body] { set; }
    }

    public interface IJoint
    {
        int Count { get; }

        void Evaluate(int c, ISystem system);
    }

    public interface IBody
    {
        Complex Dir { get; }

        Vector Mass { get; }
        Vector Force { get; }

        Vector Acc { get; set; }
        Vector Vel { get; set; }
        Vector Pos { get; set; }
    }

    public class DynamicSystem : ISystem
    {           
        public DynamicSystem(IJoint[] joints, IBody[] bodies)
        {
            blen = bodies.Length;

            foreach (var joint in joints)
                clen += joint.Count;

            if (clen > 0)
            {
                l = new float[clen];
                y = new float[clen];
                n = new float[clen, clen];
                k = new float[clen, clen];
                g = new Vector[clen, blen];
                h = new Vector[clen, blen];

                for (int j = 0; j < clen; j++)
                    n[j, j] = 1;
            }

            this.bodies = bodies;
            this.joints = joints;
        }

        private readonly int blen;
        private readonly int clen;
        
        private readonly float[] l;
        private readonly float[] y;
        private readonly float[,] n;
        private readonly float[,] k;
        private readonly Vector[,] g;
        private readonly Vector[,] h;
           
        private readonly IBody[] bodies;
        private readonly IJoint[] joints;

        public float this[int c]
        {
            get => y[c];
            set => y[c] = value;
        }
        public Vector this[int c, IBody body]
        {
            get
            {
                int b = Array.IndexOf(bodies, body);
                return g[c, b];
            }
            set
            {
                int b = Array.IndexOf(bodies, body);
                g[c, b] = value;
            }
        }

        public void Update(float step)
        {
            if (clen > 0)
            {
                float temp;

                /*********** c = 1 ***********/

                Array.Copy(n, k, n.Length);

                /*********** d, a, b, l, y = 0 ***********/

                Array.Clear(g, 0, g.Length);
                Array.Clear(n, 0, n.Length);
                Array.Clear(l, 0, l.Length);
                Array.Clear(y, 0, y.Length);

                /*********** eval d, y ***********/

                for (int i = 0, j = 0; j < clen; i++)
                {
                    joints[i].Evaluate(j, this);
                    j += joints[i].Count;
                }

                /*********** a = d / m ***********/

                for (int i = 0; i < clen; i++)
                    for (int j = 0; j < blen; j++)
                        h[i, j] = g[i, j] / bodies[j].Mass;

                /*********** b = a * d ^ T ***********/

                for (int i = 0; i < clen; i++)
                    for (int j = 0; j < clen; j++)
                        for (int k = 0; k < blen; k++)
                            n[i, j] += h[i, k] * g[j, k];

                /*********** c = b ^ -1 ***********/

                for (int i = 0; i < clen; i++)
                {
                    temp = n[i, i];

                    for (int j = 0; j < clen; j++)
                    {
                        n[i, j] /= temp;
                        k[i, j] /= temp;
                    }

                    for (int j = 0; j < clen; j++)
                    {
                        if (j != i)
                        {
                            temp = n[j, i] / n[i, i];

                            for (int k = 0; k < clen; k++)
                            {
                                if (k == i) n[j, k] = 0;
                                else n[j, k] -= n[i, k] * temp;

                                this.k[j, k] -= this.k[i, k] * temp;
                            }
                        }
                    }
                }

                /*********** l = c * (a * g - y) ***********/

                for (int i = 0; i < clen; i++)
                {
                    for (int j = 0; j < clen; j++)
                    {
                        temp = 0;

                        for (int k = 0; k < blen; k++)
                            temp += h[j, k] * bodies[k].Force;

                        l[i] += k[i, j] * (temp - y[j]);
                    }
                }
            }

            for (int i = 0; i < blen; i++)
            {
                /*********** q'' = g / m - a * l ***********/

                bodies[i].Acc = bodies[i].Force / bodies[i].Mass;

                for (int j = 0; j < clen; j++)
                    bodies[i].Acc += -h[j, i] * l[j];

                /*********** intergrate v, q ***********/

                bodies[i].Vel += bodies[i].Acc * step;
                bodies[i].Pos += bodies[i].Vel * step;
            }
        }
        public void Update(int integration, float step)
        {
            if (clen > 0)
            {
                float temp;

                /*********** c = 1 ***********/

                Array.Copy(n, k, n.Length);

                /*********** d, a, b, l, y = 0 ***********/

                Array.Clear(g, 0, g.Length);
                Array.Clear(n, 0, n.Length);
                Array.Clear(l, 0, l.Length);
                Array.Clear(y, 0, y.Length);

                /*********** eval d, y ***********/

                for (int i = 0, j = 0; j < clen; i++)
                {
                    joints[i].Evaluate(j, this);
                    j += joints[i].Count;
                }

                /*********** a = d / m ***********/

                for (int i = 0; i < clen; i++)
                    for (int j = 0; j < blen; j++)
                        h[i, j] = g[i, j] / bodies[j].Mass;

                /*********** b = a * d ^ T ***********/

                for (int i = 0; i < clen; i++)
                    for (int j = 0; j < clen; j++)
                        for (int k = 0; k < blen; k++)
                            n[i, j] += h[i, k] * g[j, k];

                /*********** c = b ^ -1 ***********/

                for (int i = 0; i < clen; i++)
                {
                    temp = n[i, i];

                    for (int j = 0; j < clen; j++)
                    {
                        n[i, j] /= temp;
                        k[i, j] /= temp;
                    }

                    for (int j = 0; j < clen; j++)
                    {
                        if (j != i)
                        {
                            temp = n[j, i] / n[i, i];

                            for (int k = 0; k < clen; k++)
                            {
                                if (k == i)
                                    n[j, k] = 0;
                                else
                                    n[j, k] -= n[i, k] * temp;

                                this.k[j, k] -= this.k[i, k] * temp;
                            }
                        }
                    }
                }

                /*********** l = c * (a * g - y) ***********/

                for (int i = 0; i < clen; i++)
                {
                    for (int j = 0; j < clen; j++)
                    {
                        temp = 0;

                        for (int k = 0; k < blen; k++)
                            temp += h[j, k] * bodies[k].Force;

                        l[i] += k[i, j] * (temp - y[j]);
                    }
                }
            }

            for (int i = 0; i < blen; i++)
            {                    
                /*********** q'' = g / m - a * l ***********/

                bodies[i].Acc = bodies[i].Force / bodies[i].Mass;

                for (int j = 0; j < clen; j++)
                    bodies[i].Acc += -h[j, i] * l[j];

                /*********** intergrate v, q ***********/

                for (int j = 0; j < integration; j++)
                {
                    bodies[i].Vel += bodies[i].Acc * step;
                    bodies[i].Pos += bodies[i].Vel * step;
                }
            }
        }
        public void Update(int steps, int integration, float step)
        {
            step /= steps * integration;
            for (int i = 0; i < steps; i++)
                Update(integration, step);
        }
    }

    public class RevoulteJoint : IJoint
    {
        public RevoulteJoint(float xi, float yi, float xj, float yj, IBody i, IBody j)
        {
            si = new Complex(xi, yi);
            sj = new Complex(xj, yj);

            bi = i;
            bj = j;
        }
        public RevoulteJoint(float x, float y, IBody i, IBody j)
        {
            var s = new Complex(x, y);

            si = (s - i.Pos.Pos) * ~i.Dir;
            sj = (s - j.Pos.Pos) * ~j.Dir;

            bi = i;
            bj = j;
        }

        private Complex si, sj;
        private IBody bi, bj;

        public float Alpha { get; set; } = 10;
        public float Beta { get; set; } = 25;

        public int Count => 2;

        public void Evaluate(int c, ISystem system)
        {
            var dsi = bi.Dir.Normal * this.si;
            var dsj = bj.Dir.Normal * this.sj;

            var si = bi.Dir * this.si;
            var sj = bj.Dir * this.sj;

            var v = (bi.Pos.Pos - bj.Pos.Pos) + (si - sj);
            var dv = (bi.Vel.Pos + dsi * bi.Vel.A) - (bj.Vel.Pos + dsj * bj.Vel.A);
            var ddv = (-si * bi.Vel.A * bi.Vel.A) - (-sj * bj.Vel.A * bj.Vel.A);

            system[c, bi] = new Vector(1, 0, dsi.X);
            system[c, bj] = -new Vector(1, 0, dsj.X);
            system[c++] = -ddv.X - Alpha * dv.X - Beta * v.X;

            system[c, bi] = new Vector(0, 1, dsi.Y);
            system[c, bj] = -new Vector(0, 1, dsj.Y);
            system[c] = -ddv.Y - Alpha * dv.Y - Beta * v.Y;
        }
    }

    public class Anchor : IJoint
    {
        public Anchor(float xi, float yi, float xj, float yj, IBody i)
        {
            si = new Complex(xi, yi);
            sj = new Complex(xj, yj);

            bi = i;
        }
        public Anchor(float x, float y, IBody i)
        {
            var s = new Complex(x, y);

            si = (s - i.Pos.Pos) * ~i.Dir;
            sj = s;

            bi = i;
        }

        private Complex si, sj;
        private IBody bi;

        public float Alpha { get; set; } = 10;
        public float Beta { get; set; } = 25;

        public int Count => 2;

        public void Evaluate(int c, ISystem system)
        {
            var si = bi.Dir * this.si;
            var dsi = bi.Dir.Normal * this.si;

            var v = (bi.Pos.Pos + si) - sj;
            var dv = (bi.Vel.Pos + dsi * bi.Vel.A);
            var ddv = (-si * bi.Vel.A * bi.Vel.A);

            system[c, bi] = new Vector(1, 0, dsi.X);
            system[c++] = -ddv.X - Alpha * dv.X - Beta * v.X;

            system[c, bi] = new Vector(0, 1, dsi.Y);
            system[c] = -ddv.Y - Alpha * dv.Y - Beta * v.Y;
        }
    }
}
