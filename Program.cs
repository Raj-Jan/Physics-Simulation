using Project1.MBS;
using System;
using System.Drawing;
using System.Windows.Forms;

namespace Project1
{
    public class Window : Form
    {
        private const int step = 50;

        private static void Main()
        {
            using (var window = new Window())
            {
                Application.Run(window);
            }
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

            Draw(e.Graphics);
        }

        private DynamicSystem system;
        private Body[] bodies;
        private IJoint[] joints;

        private void Initialize()
        {
            bodies = new Body[]
            {
                new Body
                {
                    Mass = new Vector(1, 1, 1),
                    Pos = new Vector(600, 300, 0),
                    Force = new Vector(0, 20, 0)
                },
                new Body
                {
                    Mass = new Vector(1, 1, 1),
                    Pos = new Vector(800, 300, 0),
                    Force = new Vector(0, 20, 0)
                },
                new Body
                {
                    Mass = new Vector(1, 1, 1),
                    Pos = new Vector(1000, 300, 0),
                    Force = new Vector(0, 20, 0)
                },
            };
            joints = new IJoint[]
            {
                new RevoulteJoint(700, 300, bodies[0], bodies[1]),
                new RevoulteJoint(900, 300, bodies[1], bodies[2]),
                new Anchor(500, 300, bodies[0]),
            };

            system = new DynamicSystem(joints, bodies);
        }

        private void Update(float step)
        {
            system.Update(96, 1 ,step);

            Invalidate();
        }
        private void Draw(Graphics graphics)
        {
            graphics.Clear(Color.Black);

            using (var pen = new Pen(Color.White))
            {
                foreach (var body in bodies)
                {
                    body.Draw(pen, graphics);
                }
            }
        }
    }
}
