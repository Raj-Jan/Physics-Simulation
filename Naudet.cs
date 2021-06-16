using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Project1.Naudet
{
    public static class Utils
    {
	public static void Line(Vector x1, Vector x2)
	{
	    Project1.Utils.Line(x1.X, x1.Y, x2.X, x2.Y);
	}
	public static void Point(Vector x)
	{
	    Project1.Utils.Point(x.X, x.Y);
	}
    }

    public struct Vector
    {
	public Vector(float x, float y, float z)
	{
	    X = x;
	    Y = y;
	    Z = z;
	}

	public float X { get; set; }
	public float Y { get; set; }
	public float Z { get; set; }

	public float Length
	{
	    get => (float)Math.Sqrt(X * X + Y * Y + Z * Z);
	}

	public static Vector operator -(Vector p)
	{
	    return new Vector(-p.X , -p.Y, -p.Z);
	}
	public static Vector operator +(Vector p, Vector q)
	{
	    return new Vector(p.X + q.X, p.Y + q.Y, p.Z + q.Z);
	}
	public static Vector operator -(Vector p, Vector q)
	{
	    return new Vector(p.X - q.X, p.Y - q.Y, p.Z - q.Z);
	}
	public static Vector operator *(Vector v, float f)
	{
	    return new Vector(v.X * f, v.Y * f, v.Z * f);
	}
	public static Vector operator /(Vector v, float f)
	{
	    return new Vector(v.X / f, v.Y / f, v.Z / f);
	}
	public static Vector operator ^(Vector p, Vector q)
	{
	    return new Vector(p.Y * q.Z - p.Z * q.Y, p.Z * q.X - p.X * q.Z, p.X * q.Y - p.Y * q.X);
	}

	public static Matrix operator *(Vector p, Vector q)
	{
	    return new Matrix
	    (
		p.X * q.X, p.X * q.Y, p.X * q.Z,
		p.Y * q.X, p.Y * q.Y, p.Y * q.Z,
		p.Z * q.X, p.Z * q.Y, p.Z * q.Z
	    );
	}

	public static float operator |(Vector p, Vector q)
	{
	    return p.X * q.X + p.Y * q.Y + p.Z * q.Z;
	}

	public override string ToString()
	{
	    return $"{X}, {Y}, {Z}";
	}
    }

    public struct Matrix
    {
	public static Matrix One
	{
	    get => new Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
	}
	public static Matrix Zero
	{
	    get => new Matrix(0, 0, 0, 0, 0, 0, 0, 0, 0);
	}

	public Matrix(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
	{
	    this.m00 = m00;
	    this.m01 = m01;
	    this.m02 = m02;
	    this.m10 = m10;
	    this.m11 = m11;
	    this.m12 = m12;
	    this.m20 = m20;
	    this.m21 = m21;
	    this.m22 = m22;
	}
	public Matrix(float m00, float m11, float m22)
	{
	    this.m00 = m00;
	    this.m01 = 0;
	    this.m02 = 0;
	    this.m10 = 0;
	    this.m11 = m11;
	    this.m12 = 0;
	    this.m20 = 0;
	    this.m21 = 0;
	    this.m22 = m22;
	}

	private float m00, m01, m02;
	private float m10, m11, m12;
	private float m20, m21, m22;

	public static Matrix CreateRotation(Vector axis, float angle)
	{
	    if (angle == 0) return One;

	    var x = axis.X;
	    var y = axis.Y;
	    var z = axis.Z;
	    var sin = (float)Math.Sin(-angle);
	    var cos = (float)Math.Cos(-angle);
	    var x2 = x * x;
	    var y2 = y * y;
	    var z2 = z * z;
	    var xy = x * y;
	    var xz = x * z;
	    var yz = y * z;

	    return new Matrix
	    (
		x2 + (cos * (1 - x2)), xy - (cos * xy) + (sin * z), xz - (cos * xz) - (sin * y),
		xy - (cos * xy) - (sin * z), y2 + (cos * (1 - y2)), yz - (cos * yz) + (sin * x),
		xz - (cos * xz) + (sin * y), yz - (cos * yz) - (sin * x), z2 + (cos * (1 - z2))
	    );
	}
	public static Matrix CreateRotation(Vector axisAngle)
	{
	    float angle = axisAngle.Length;

	    return CreateRotation(axisAngle / angle, angle);
	}
	public static Matrix CreateSkew(Vector vec)
	{
	    return new Matrix
	    (
		0, -vec.Z, vec.Y,
		vec.Z, 0, -vec.X,
		-vec.Y, vec.X, 0
	    );
	}

	public static Matrix operator ~(Matrix p)
	{
	    return new Matrix(p.m00, p.m10, p.m20, p.m01, p.m11, p.m21, p.m02, p.m12, p.m22);
	}
	public static Matrix operator -(Matrix p)
	{
	    return new Matrix(-p.m00, -p.m01, -p.m02, -p.m10, -p.m11, -p.m12, -p.m20, -p.m21, -p.m22);
	}
	public static Matrix operator +(Matrix p, Matrix q)
	{
	    return new Matrix
	    (
		p.m00 + q.m00, p.m01 + q.m01, p.m02 + q.m02,
		p.m10 + q.m10, p.m11 + q.m11, p.m12 + q.m12,
		p.m20 + q.m20, p.m21 + q.m21, p.m22 + q.m22
	    );
	}
	public static Matrix operator -(Matrix p, Matrix q)
	{
	    return new Matrix
	    (
		p.m00 - q.m00, p.m01 - q.m01, p.m02 - q.m02,
		p.m10 - q.m10, p.m11 - q.m11, p.m12 - q.m12,
		p.m20 - q.m20, p.m21 - q.m21, p.m22 - q.m22
	    );
	}
	public static Matrix operator *(Matrix p, Matrix q)
	{
	    return new Matrix
	    (
		p.m00 * q.m00 + p.m01 * q.m10 + p.m02 * q.m20,
		p.m00 * q.m01 + p.m01 * q.m11 + p.m02 * q.m21,
		p.m00 * q.m02 + p.m01 * q.m12 + p.m02 * q.m22,

		p.m10 * q.m00 + p.m11 * q.m10 + p.m12 * q.m20,
		p.m10 * q.m01 + p.m11 * q.m11 + p.m12 * q.m21,
		p.m10 * q.m02 + p.m11 * q.m12 + p.m12 * q.m22,

		p.m20 * q.m00 + p.m21 * q.m10 + p.m22 * q.m20,
		p.m20 * q.m01 + p.m21 * q.m11 + p.m22 * q.m21,
		p.m20 * q.m02 + p.m21 * q.m12 + p.m22 * q.m22
	    );
	}
	public static Matrix operator *(Matrix p, float q)
	{
	    return new Matrix
	    (
		p.m00 * q, p.m01 * q, p.m02 * q,
		p.m10 * q, p.m11 * q, p.m12 * q,
		p.m20 * q, p.m21 * q, p.m22 * q
	    );
	}
	public static Matrix operator /(Matrix p, float q)
	{
	    return new Matrix
	    (
		p.m00 / q, p.m01 / q, p.m02 / q,
		p.m10 / q, p.m11 / q, p.m12 / q,
		p.m20 / q, p.m21 / q, p.m22 / q
	    );
	}

	public static Matrix operator ^(Matrix p, Vector q)
	{
	    Vector x = new Vector(p.m00, p.m01, p.m02) ^ q;
	    Vector y = new Vector(p.m10, p.m11, p.m12) ^ q;
	    Vector z = new Vector(p.m20, p.m21, p.m22) ^ q;

	    return new Matrix(x.X, x.Y, x.Z, y.X, y.Y, y.Z, z.X, z.Y, z.Z);
	}

	public static Vector operator *(Matrix m, Vector v)
	{
	    float x = m.m00 * v.X + m.m01 * v.Y + m.m02 * v.Z;
	    float y = m.m10 * v.X + m.m11 * v.Y + m.m12 * v.Z;
	    float z = m.m20 * v.X + m.m21 * v.Y + m.m22 * v.Z;

	    return new Vector(x, y, z);
	}

	public override string ToString()
	{
	    return $"{m00}, {m01}, {m02} | {m10}, {m11}, {m12} | {m20}, {m21}, {m22} ";
	}

	public static implicit operator SymmMatrix(Matrix mat)
	{
	    return new SymmMatrix(mat.m00, mat.m01, mat.m02, mat.m11, mat.m12, mat.m22);
	}
    }

    public struct SymmMatrix
    {
	public static SymmMatrix One
	{
	    get => new SymmMatrix(1, 0, 0, 1, 0, 1);
	}
	public static SymmMatrix Zero
	{
	    get => new SymmMatrix(0, 0, 0, 0, 0, 0);
	}

	public SymmMatrix(float m00, float m01, float m02, float m11, float m12, float m22)
	{
	    this.m00 = m00;
	    this.m01 = m01;
	    this.m02 = m02;
	    this.m11 = m11;
	    this.m12 = m12;
	    this.m22 = m22;
	}

	private float m00, m01, m02;
	private float         m11, m12;
	private float                  m22;

	public static SymmMatrix operator -(SymmMatrix p)
	{
	    return new SymmMatrix(-p.m00, -p.m01, -p.m02, -p.m11, -p.m12, -p.m22);
	}
	public static SymmMatrix operator +(SymmMatrix p, SymmMatrix q)
	{
	    return new SymmMatrix
	    (
		p.m00 + q.m00, p.m01 + q.m01, p.m02 + q.m02,
			       p.m11 + q.m11, p.m12 + q.m12,
					      p.m22 + q.m22
	    );
	}
	public static SymmMatrix operator -(SymmMatrix p, SymmMatrix q)
	{
	    return new SymmMatrix
	    (
		p.m00 - q.m00, p.m01 - q.m01, p.m02 - q.m02,
			       p.m11 - q.m11, p.m12 - q.m12,
					      p.m22 - q.m22
	    );
	}

	public static SymmMatrix operator *(SymmMatrix p, float q)
	{
	    return new SymmMatrix
	    (
		p.m00 * q, p.m01 * q, p.m02 * q,
			   p.m11 * q, p.m12 * q,
				      p.m22 * q
	    );
	}
	public static SymmMatrix operator /(SymmMatrix p, float q)
	{
	    return new SymmMatrix
	    (
		p.m00 / q, p.m01 / q, p.m02 / q,
			   p.m11 / q, p.m12 / q,
				      p.m22 / q
	    );
	}

	public static Vector operator *(SymmMatrix m, Vector v)
	{
	    float x = m.m00 * v.X + m.m01 * v.Y + m.m02 * v.Z;
	    float y = m.m01 * v.X + m.m11 * v.Y + m.m12 * v.Z;
	    float z = m.m02 * v.X + m.m12 * v.Y + m.m22 * v.Z;

	    return new Vector(x, y, z);
	}

	public override string ToString()
	{
	    return $"{m00}, {m01}, {m02} | {m01}, {m11}, {m12} | {m02}, {m12}, {m22} ";
	}

	public static implicit operator Matrix(SymmMatrix mat)
	{
	    return new Matrix(mat.m00, mat.m01, mat.m02, mat.m01, mat.m11, mat.m12, mat.m01, mat.m12, mat.m22);
	}
    }

    public struct SMatrix
    {
	public static SMatrix One
	{
	    get => new SMatrix(SymmMatrix.One, Matrix.Zero, SymmMatrix.One);
	}

	public SMatrix(SymmMatrix m00, Matrix m01, SymmMatrix m11)
	{
	    this.m00 = m00;
	    this.m01 = m01;
	    this.m11 = m11;
	}

	public SymmMatrix m00; public Matrix m01;
					    public SymmMatrix m11;

	public Matrix m10
	{
	    get => ~m01;
	}

	public static SMatrix operator +(SMatrix p, SMatrix q)
	{
	    var m00 = p.m00 + q.m00;
	    var m01 = p.m01 + q.m01;
	    var m11 = p.m11 + q.m11;

	    return new SMatrix(m00, m01, m11);
	}
	public static SMatrix operator -(SMatrix p, SMatrix q)
	{
	    var m00 = p.m00 - q.m00;
	    var m01 = p.m01 - q.m01;
	    var m11 = p.m11 - q.m11;

	    return new SMatrix(m00, m01, m11);
	}
	public static SMatrix operator *(SMatrix p, SMatrix q)
	{
	    var m00 = (Matrix)p.m00 * q.m00 + p.m01 * q.m10;
	    var m01 = p.m00 * q.m01 + p.m01 * q.m11;
	    var m11 = p.m10 * q.m01 + (Matrix)p.m11 * q.m11;

	    return new SMatrix(m00, m01, m11);
	}
	public static SVector operator *(SMatrix p, SVector q)
	{
	    return new SVector(p.m00 * q.Lin + p.m01 * q.Ang, p.m10 * q.Lin + p.m11 * q.Ang);
	}
	public static SMatrix operator *(SMatrix p, float q)
	{
	    return new SMatrix
	    (
		p.m00 * q, p.m01 * q,
			   p.m11 * q
	    );
	}
	public static SMatrix operator /(SMatrix p, float q)
	{
	    return new SMatrix
	    (
		p.m00 / q, p.m01 / q,
			   p.m11 / q
	    );
	}
	public static SMatrix operator *(float p, SMatrix q)
	{
	    return new SMatrix
	    (
		q.m00 * p, q.m01 * p,
			   q.m11 * p
	    );
	}
    }

    public struct Transform
    {
	public Transform(Matrix ang, Vector lin)
	{
	    Ang = ang;
	    Lin = lin;
	}

	public Matrix Ang { get; set; }
	public Vector Lin { get; set; }

	public static Transform operator +(Transform p, SVector q)
	{
	    Matrix rot = Matrix.CreateRotation(q.Ang);

	    p.Ang = rot * p.Ang;
	    p.Lin = q.Lin + p.Lin;

	    return p;
	}

	public static Transform operator +(Transform p, Matrix q)
	{
	    return new Transform(q * p.Ang, p.Lin);
	}
	public static Transform operator +(Transform p, Vector q)
	{
	    return new Transform(p.Ang, p.Lin + q);
	}
    }

    public struct SVector
    {
	public SVector(Vector lin, Vector ang)
	{
	    Lin = lin;
	    Ang = ang;
	}

	public Vector Lin { get; set; }
	public Vector Ang { get; set; }

	public float Length
	{
	    get => (float)Math.Sqrt((Lin | Lin) + (Ang | Ang));
	}

	public static SVector operator -(SVector p)
	{
	    return new SVector(-p.Lin, -p.Ang);
	}
	public static SVector operator +(SVector p, SVector q)
	{
	    return new SVector(p.Lin + q.Lin, p.Ang + q.Ang);
	}
	public static SVector operator -(SVector p, SVector q)
	{
	    return new SVector(p.Lin - q.Lin, p.Ang - q.Ang);
	}
	public static SVector operator *(SVector p, float q)
	{
	    return new SVector(p.Lin * q, p.Ang * q);
	}
	public static SVector operator *(Matrix p, SVector q)
	{
	    return new SVector(p * q.Lin, p * q.Ang);
	}
	public static SVector operator /(SVector p, float q)
	{
	    return new SVector(p.Lin / q, p.Ang / q);
	}

	public static SMatrix operator *(SVector p, SVector q)
	{
	    return new SMatrix
	    (
		p.Lin * q.Lin, p.Lin * q.Ang,
			       p.Ang * q.Ang
	    );
	}
	public static SVector operator ^(SVector p, SVector q)
	{
	    return new SVector(p.Ang ^ q.Lin, (p.Lin ^ q.Lin) + (p.Ang ^ q.Ang));
	}

	public static float operator |(SVector p, SVector q)
	{
	    return (p.Lin | q.Lin) + (p.Ang | q.Ang);
	}

	public override string ToString()
	{
	    return $"{Lin} | {Ang}";
	}
    }

    public struct Solid
    {
	public static Solid Rod(float mass, float length)
	{
	    float inertia = mass * length * length * mass / 12;

	    return new Solid()
	    {
		mass = mass,
		inertia = new SymmMatrix(0, 0, 0, inertia, 0, inertia),
	    };
	}

	private float mass;
	private SymmMatrix inertia;

	public void Rotate(Matrix rot)
	{
	    inertia = rot * inertia * ~rot;
	}

	public static SVector operator *(Solid p, SVector q)
	{
	    return new SVector(q.Lin * p.mass, p * q.Ang);
	}
	public static   Vector operator *(Solid p,   Vector q)
	{
	    return p.inertia * q;
	}

	public static implicit operator SMatrix(Solid solid)
	{
	    return new SMatrix
	    (
		Matrix.One * solid.mass, Matrix.Zero,
					 solid.inertia
	    );
	}
    }

	public class Body
	{
		public Body(float x, float y, float a, float mass, float len = 200)
		{
			a *= (float)Math.PI / 2;

			Position = new Transform(Matrix.CreateRotation(new Vector(0, 0, 1), a), new Vector(x, y, 0));

			Mass = Solid.Rod(mass, len);
		}

		public Solid Mass { get; set; }

		public Transform Position { get; set; }
		public SVector Velocity { get; set; }
		public SVector Force { get; set; }

		public SVector Momentum
		{
			get => Mass * Velocity;
		}

		public void Draw()
		{
			Vector d0 = Position.Ang * new Vector(120, 20, 0);
			Vector d1 = Position.Ang * new Vector(-120, 20, 0);

			Project1.Utils.Stroke(255, 255, 255);
			Project1.Utils.Width(5);
			Utils.Point(Position.Lin);

			Project1.Utils.Width(2);
			Project1.Utils.Stroke(122, 122, 122);
			Utils.Line(Position.Lin + d0, Position.Lin + d1);
			Utils.Line(Position.Lin + d1, Position.Lin - d0);
			Utils.Line(Position.Lin - d0, Position.Lin - d1);
			Utils.Line(Position.Lin - d1, Position.Lin + d0);
			//Utils.Stroke(255, 112, 112);
			//Utils.Line(Position.Lin, Position.Lin - d);
		}
	}

	public class Joint
	{
		public static Joint Revoulte(Body prev, Body next, Vector pin, Vector axis)
		{
			Vector r = pin - prev.Position.Lin;
			Vector d = next.Position.Lin - pin;

			Joint result = new Joint()
			{
				prev = prev,
				next = next,
				r = r,
				d = d
			};

			result.axis = new SVector(axis ^ d, axis);

			return result;
		}
		public static Joint Prismatic(Body prev, Body next, Vector pin, Vector axis)
		{
			Vector r = pin - prev.Position.Lin;
			Vector d = pin - next.Position.Lin;

			Joint result = new Joint()
			{
				prev = prev,
				next = next,
				r = r,
				d = d
			};

			result.axis = new SVector(axis, new Vector());

			return result;
		}

		private Body prev;
		private Body next;

		private Vector r;
		private Vector d;

		private SVector axis;
		public float motor;
		private float moment;
		private float vel;
		private float pos;

		private float p;
		private float b;
		private SVector a;

		private void Propagate(Vector delta, ref SVector fstar, ref SVector pstar)
		{
			pstar += next.Momentum;
			fstar += next.Force;

			p = axis | (fstar - (next.Velocity ^ pstar));

			pstar = XT(delta, pstar);
			fstar = XT(delta, fstar);
		}
		private void Propagate(Vector delta, ref SMatrix mstar, ref SVector zstar)
		{
			mstar += next.Mass;

			var temp = mstar * axis;
			var a = XT(delta, temp);
			var gamma = 1 / (axis | temp);
			var at = a * gamma;
			var b = gamma * (moment - (axis | zstar));

			this.a = at;
			this.b = b;

			mstar = XT(delta, mstar) - a * at;
			zstar = XT(delta, zstar) + a * b;
		}
		private void Propagate(ref SVector fstar, ref SVector pstar, ref SMatrix mstar, ref SVector zstar)
		{
			var delta = next.Position.Lin - prev.Position.Lin;

			Propagate(delta, ref fstar, ref pstar);
			Propagate(delta, ref mstar, ref zstar);
		}

		private void Integrate(float dt)
		{
			pos += vel * dt;
			vel = b - (a | prev.Velocity);
			moment += (p + motor) * dt;
		}

		private void ResolveVelocity()
		{
			Vector delta = next.Position.Lin - prev.Position.Lin;

			next.Velocity = X(delta, prev.Velocity) + axis * vel;
		}
		private void ResolvePosition(float dt, ref Matrix rotPrev)
		{
			var rotNext = Matrix.CreateRotation(next.Velocity.Ang * dt);

			r = rotPrev * r;
			d = rotNext * d;

			next.Mass.Rotate(rotNext);

			Matrix rot = rotNext * next.Position.Ang;
			Vector pos = prev.Position.Lin + r + d;

			next.Position = new Transform(rot, pos);

			axis = rotNext * axis;

			rotPrev = rotNext;
		}

		public void Draw(int i)
		{
			Project1.Utils.Width(2);

			Project1.Utils.Stroke(112, 255, 112);
			Utils.Line(prev.Position.Lin, prev.Position.Lin + r);

			Project1.Utils.Stroke(255, 112, 112);
			Utils.Line(next.Position.Lin, next.Position.Lin - d);

			Project1.Utils.Stroke(255, 255, 255);
			Utils.Line(next.Position.Lin, next.Position.Lin + axis.Lin);

			Project1.Utils.Stroke(255, 255, 255);
			Project1.Utils.Text(20, 20 + 20 * i, $"momentum : {moment}");
			Project1.Utils.Text(170, 20 + 20 * i, $"vel : {vel}");
		}

		public override string ToString()
		{
			return $"Axis: {axis}\n Momentum: {moment}\n Vel: {vel}\n Pos: {pos}\n";
		}

		public static void Solve(Joint[] joints, float dt)
		{
			SVector pstar = new SVector();
			SVector fstar = new SVector();
			SVector astar = new SVector();
			SMatrix mstar = new SMatrix();

			for (int i = joints.Length; i --> 0;)
			{
				Joint joint = joints[i];

				joint.Propagate(ref fstar, ref pstar, ref mstar, ref astar);
			}

			Matrix rot = joints[0].prev.Position.Ang;

			for (int i = 0; i < joints.Length; i++)
			{
				Joint joint = joints[i];

				joint.Integrate(dt);

				joint.ResolveVelocity();
				joint.ResolvePosition(dt, ref rot);
			}
		}

		private static SVector X(Vector delta, SVector vec)
		{
			return new SVector(vec.Lin - (delta ^ vec.Ang), vec.Ang);
		}
		private static SVector XT(Vector delta, SVector vec)
		{
			return new SVector(vec.Lin, vec.Ang + (delta ^ vec.Lin));
		}
		private static SMatrix XT(Vector delta, SMatrix mat)
		{
			Matrix x = Matrix.CreateSkew(delta);

			SMatrix result = new SMatrix();
			result.m00 = mat.m00;
			result.m01 = mat.m01 - mat.m00 * x;
			result.m11 = mat.m11 + (SymmMatrix)(x * result.m01 - mat.m10 * x);

			return result;
		}
	}
}
