using Project1.Naudet;
using System;
using System.Drawing;
using System.Windows.Forms;

namespace Project1
{
    public class Window : Form
    {
        private const int step = 1000 / 20;

        public static readonly Window window = new Window();

        private static void Main()
        {
            Application.Run(window);
            window.Dispose();
        }

        public Window()
        {
            StartPosition = FormStartPosition.Manual;
            FormBorderStyle = FormBorderStyle.None;
            Bounds = Screen.PrimaryScreen.Bounds;

            timer = new Timer()
            {
                Enabled = true,
                Interval = step
            };

            timer.Tick += (o, args) => Update(0.001f * step);

            Initialize();
        }

        private Timer timer;

        protected override CreateParams CreateParams
        {
            get
            {
                CreateParams handleParam = base.CreateParams;
                handleParam.ExStyle |= 0x02000000;
                return handleParam;
            }
        }

        protected override void OnKeyDown(KeyEventArgs e)
        {
            base.OnKeyDown(e);

            if (e.KeyCode == Keys.Escape)
                Application.Exit();
        }
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            e.Graphics.Clear(Color.Black);

            Draw();
        }

        //private Joint[] joints;
        //private RigidBody[] bodies;

        //private void Initialize()
        //{
        //    joints = new Joint[]
        //    {
        //        new Joint()
        //        {
        //            Axis = new SVector(new Vector(), new Vector(0, 0, 1)),
        //            Motor = 0.1f
        //        },
        //        new Joint()
        //        {
        //            Axis = new SVector(new Vector(), new Vector(0, 0, 1)),
        //            Motor = 0
        //        }
        //    };

        //    bodies = new RigidBody[]
        //    {
        //        new RigidBody()
        //        {
        //            Mass = Solid.Rod(1, 200),
        //            Position = new Transform(300, 300, 0),

        //            Force = new SVector(new Vector(), new Vector())
        //        },
        //        new RigidBody()
        //        {
        //            Mass = Solid.Rod(1, 200),
        //            Position = new Transform(500, 300, 0),
        //            Force = new SVector(new Vector(0, 10, 0), new Vector())
        //        },
        //        new RigidBody()
        //        {
        //            Mass = Solid.Rod(1, 200),
        //            Position = new Transform(700, 300, 0),
        //            Force = new SVector(new Vector(0, 10, 0), new Vector())
        //        }
        //    };

        //    bodies[0].Mass.Output = new Vector(100, 0, 0);
        //    bodies[1].Mass.Input = new Vector(-100, 0, 0);
        //    bodies[1].Mass.Output = new Vector(100, 0, 0);
        //    bodies[2].Mass.Input = new Vector(-100, 0, 0);
        //}

        //private void Update(float step)
        //{
        //    StateVector vec = new StateVector()
        //    {
        //        Acceleration = new SVector(new Vector(0, 0, 0), new Vector(0, 0, 0)),
        //    };

        //    bodies[0].Propagate(ref vec);

        //    for (int i = 0; i < joints.Length;)
        //    {
        //        joints[i].Propagate(ref vec);
        //        bodies[++i].Propagate(ref vec);
        //    }

        //    bodies[0].Integrate(step);

        //    for (int i = 1; i < bodies.Length; i++)
        //    {
        //        bodies[i].Integrate(step, joints[i - 1]);
        //    }

        //    Vector start = bodies[0].Position.Lin;

        //    for (int i = 0; i < bodies.Length; i++)
        //        bodies[i].ResolvePosition(ref start);

        //    Invalidate();
        //}
        //private void Draw()
        //{
        //    foreach (var body in bodies)
        //        body.Draw();
        //}

        private Joint[] joints;
        private Body[] bodies;
        private Body ground;

        private void Initialize()
        {
            Vector force = new Vector(0, 10, 0);

            ground = new Body(0, 0, 0, 0);

            bodies = new Body[]
            {
                new Body(500, 300, 0, 1)
                {
                    Force = new SVector(force, new Vector())
                },
                new Body(700, 300, 0, 1)
                {
                    Force = new SVector(force, new Vector())
                },
                //new Body(800, 200, 0, 1)
                //{
                //    Force = new SVector(force, new Vector())
                //},
                //new Body(900, 300, -1, 1)
                //{
                //    Force = new SVector(force, new Vector())
                //},
                //new Body(1000, 400, 0, 1)
                //{
                //    Force = new SVector(force, new Vector())
                //},
            };

            joints = new Joint[]
            {
                Joint.Revoulte(ground, bodies[0], new Vector(400, 300, 0), new Vector(0, 0, 1)),
                Joint.Revoulte(bodies[0], bodies[1], new Vector(600, 300, 0), new Vector(0, 0, 1)),
                //Joint.Revoulte(bodies[2], bodies[3], new Vector(700, 200, 0), new Vector(0, 0, 1)),
                //Joint.Revoulte(bodies[3], bodies[4], new Vector(900, 200, 0), new Vector(0, 0, 1)),
                //Joint.Revoulte(bodies[4], bodies[5], new Vector(900, 400, 0), new Vector(0, 0, 1)),
            };
        }

        private void Update(float step)
        {
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);
            Joint.Solve(joints, step / 3);

            Invalidate();
        }
        private void Draw()
        {
            int i = 0;

            foreach (var body in bodies)
            {
                body.Draw();
            }
            foreach (var joint in joints)
            {
                joint.Draw(i++);
            }

        }
    }

    public static class Utils
    {
        private static Pen pen = new Pen(Color.White);

        public static void Width(float size)
        {
            pen.Width = size;
        }
        public static void Stroke(int r = 0, int g = 0, int b = 0)
        {
            var color = Color.FromArgb(r, g, b);

            pen.Color = color;
        }

        public static void Line(float x1, float y1, float x2, float y2)
        {
            using (var graphics = Window.window.CreateGraphics())
            {
                graphics.DrawLine(pen, x1, y1, x2, y2);
            }
        }
        public static void Point(float x, float y)
        {
            using (var graphics = Window.window.CreateGraphics())
            {
                float r = pen.Width / 2;

                graphics.FillEllipse(pen.Brush, x - r, y - r, 2 * r, 2 * r);
            }
        }

        public static void Text(float x, float y, string text)
        {
            using (var graphics = Window.window.CreateGraphics())
            {
                graphics.DrawString(text, SystemFonts.DefaultFont, pen.Brush, x, y);
            }
        }
    }
}
