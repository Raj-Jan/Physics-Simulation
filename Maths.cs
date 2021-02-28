using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Project1
{
    public struct Complex
    {
        public Complex(float x, float y)
        {
            X = x;
            Y = y;
        }

        public float X { get; set; }
        public float Y { get; set; }

        public float Len2
        {
            get => X * X + Y * Y;
        }
        public float Lenght
        {
            get => (float)Math.Sqrt(X * X + Y * Y);
        }

        public Complex Normal
        {
            get => new Complex(-Y, X);
        }

        public float Cross(Complex other)
        {
            return X * other.Y - Y * other.X;
        }

        public static Complex operator ~(Complex p)
        {
            return new Complex(p.X, -p.Y);
        }
        public static Complex operator -(Complex p)
        {
            return new Complex(-p.X, -p.Y);
        }

        public static Complex operator +(Complex p, Complex q)
        {
            return new Complex(p.X + q.X, p.Y + q.Y);
        }
        public static Complex operator -(Complex p, Complex q)
        {
            return new Complex(p.X - q.X, p.Y - q.Y);
        }
        public static Complex operator *(Complex p, Complex q)
        {
            var x = p.X * q.X - p.Y * q.Y;
            var y = p.X * q.Y + p.Y * q.X;

            return new Complex(x, y);
        }
        public static Complex operator /(Complex p, Complex q)
        {
            return p * (1 / q);
        }


        public static Complex operator *(Complex p, float q)
        {
            return new Complex(p.X * q, p.Y * q);
        }
        public static Complex operator /(Complex p, float q)
        {
            return new Complex(p.X / q, p.Y / q);
        }

        public static Complex operator /(float p, Complex q)
        {
            var len = p * q.Len2;

            return new Complex(q.X / len, -q.Y / len);
        }
        public static Complex operator *(float p, Complex q)
        {
            return new Complex(p * q.X, p * q.Y);
        }

        public static Complex operator +(Vector p, Complex q)
        {
            return p.Pos + q;
        }

        public override string ToString()
        {
            return $"{X}, {Y}";
        }
    }

    public struct Vector
    {
        public Vector(Complex pos, float a)
        {
            this.pos = pos;
            A = a;
        }
        public Vector(Complex pos)
        {
            this.pos = pos;
            A = 0;
        }
        public Vector(float x, float y, float a)
        {
            pos = new Complex(x, y);
            A = a;
        }
        public Vector(float x, float y)
        {
            pos = new Complex(x, y);
            A = 0;
        }

        private Complex pos;

        public Complex Pos
        {
            get => pos;
            set => pos = value;
        }
        public float X
        {
            get => pos.X;
            set => pos.X = value;
        }
        public float Y
        {
            get => pos.Y;
            set => pos.Y = value;
        }
        public float A { get; set; }

        public float LenghtSq
        {
            get => pos.Len2;
        }
        public float Lenght
        {
            get => pos.Lenght;
        }

        public static Vector operator -(Vector p)
        {
            return new Vector(-p.pos, -p.A);
        }
        public static Vector operator +(Vector p, Vector q)
        {
            return new Vector(p.pos + q.pos, p.A + q.A);
        }
        public static Vector operator -(Vector p, Vector q)
        {
            return new Vector(p.pos - q.pos, p.A - q.A);
        }
        public static Vector operator *(float p, Vector q)
        {
            return new Vector(p * q.pos, p * q.A);
        }
        public static Vector operator *(Vector p, float q)
        {
            return new Vector(p.pos * q, p.A * q);
        }
        public static Vector operator /(Vector p, float q)
        {
            return new Vector(p.pos / q, p.A / q);
        }

        public static float operator *(Vector p, Vector q)
        {
            return p.X * q.X + p.Y * q.Y + p.A * q.A;
        }
        public static Vector operator /(Vector p, Vector q)
        {
            return new Vector(p.X / q.X, p.Y / q.Y, p.A / q.A);
        }

        public override string ToString()
        {
            return $"{{{pos}}}, {A}";
        }
    }
}
