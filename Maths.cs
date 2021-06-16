using System;

namespace Project1.Maths
{
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
			return new Vector(-p.X, -p.Y, -p.Z);
		}
		public static Vector operator +(Vector p, Vector q)
		{
			return new Vector(p.X + q.X, p.Y + q.Y, p.Z + q.Z);
		}
		public static Vector operator -(Vector p, Vector q)
		{
			return new Vector(p.X - q.X, p.Y - q.Y, p.Z - q.Z);
		}
		public static Vector operator *(Vector p, Vector q)
                {
			return new Vector(p.Y * q.Z - p.Z * q.Y, p.Z * q.X - p.X * q.Z, p.X * q.Y - p.Y * q.X);
		}
		public static Vector operator *(Vector p, float q)
		{
			return new Vector(p.X * q, p.Y * q, p.Z * q);
		}
		public static Vector operator /(Vector p, float q)
		{
			return new Vector(p.X / q, p.Y / q, p.Z / q);
		}

		public static float Dot(Vector p, Vector q)
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

		private readonly float m00, m01, m02;
		private readonly float m10, m11, m12;
		private readonly float m20, m21, m22;

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

		public static Vector operator *(Matrix m, Vector v)
		{
			float x = m.m00 * v.X + m.m01 * v.Y + m.m02 * v.Z;
			float y = m.m10 * v.X + m.m11 * v.Y + m.m12 * v.Z;
			float z = m.m20 * v.X + m.m21 * v.Y + m.m22 * v.Z;

			return new Vector(x, y, z);
		}

		public static SymMatrix operator *(SkewMatrix p, Matrix q)
                {
			return new SymMatrix
			(
				p.Y * q.m20 - p.Z * q.m10,
				p.Y * q.m21 - p.Z * q.m11,
				p.Y * q.m22 - p.Z * q.m12,
				
				p.Z * q.m01 - p.X * q.m21,
				p.Z * q.m02 - p.X * q.m22,
				
				p.X * q.m12 - p.Y * q.m02
			);
		}

		public override string ToString()
		{
			return $"{m00}, {m01}, {m02} | {m10}, {m11}, {m12} | {m20}, {m21}, {m22} ";
		}
	}

	public struct SkewMatrix
        {
                public SkewMatrix(float x, float y, float z)
                {
                        X = x;
                        Y = y;
                        Z = z;
                }
		public SkewMatrix(Vector vec)
                {
			X = vec.X;
			Y = vec.Y;
			Z = vec.Z;
                }

                public float X { get; set; }
		public float Y { get; set; }
		public float Z { get; set; }

		public static SkewMatrix operator -(SkewMatrix p)
		{
			return new SkewMatrix(-p.X, -p.Y, -p.Z);
		}
		public static SkewMatrix operator +(SkewMatrix p, SkewMatrix q)
		{
			return new SkewMatrix(p.X + q.X, p.Y + q.Y, p.Z + q.Z);
		}
		public static SkewMatrix operator -(SkewMatrix p, SkewMatrix q)
		{
			return new SkewMatrix(p.X - q.X, p.Y - q.Y, p.Z - q.Z);
		}
		public static SkewMatrix operator *(SkewMatrix p, SkewMatrix q)
		{
			return new SkewMatrix(p.Y * q.Z - p.Z * q.Y, p.Z * q.X - p.X * q.Z, p.X * q.Y - p.Y * q.X);
		}
		public static SkewMatrix operator *(SkewMatrix p, float q)
		{
			return new SkewMatrix(p.X * q, p.Y * q, p.Z * q);
		}
		public static SkewMatrix operator /(SkewMatrix p, float q)
		{
			return new SkewMatrix(p.X / q, p.Y / q, p.Z / q);
		}
	}

	public struct SymMatrix
	{
		public SymMatrix(float m00, float m01, float m02, float m11, float m12, float m22)
		{
			this.m00 = m00;
			this.m01 = m01;
			this.m02 = m02;
			this.m11 = m11;
			this.m12 = m12;
			this.m22 = m22;
		}

		private readonly float	m00, m01, m02;
		private readonly float		m11, m12;
		private readonly float			m22;

		public static SymMatrix operator -(SymMatrix p)
		{
			return new SymMatrix
			(
				-p.m00, -p.m01, -p.m02,
					    -p.m11, -p.m12,
							-p.m22
			);
		}
		public static SymMatrix operator +(SymMatrix p, SymMatrix q)
		{
			return new SymMatrix
			(
				p.m00 + q.m00, p.m01 + q.m01, p.m02 + q.m02,
						      p.m11 + q.m11, p.m12 + q.m12,
									     p.m22 + q.m22
			);
		}
		public static SymMatrix operator -(SymMatrix p, SymMatrix q)
		{
			return new SymMatrix
			(
				p.m00 - q.m00, p.m01 - q.m01, p.m02 - q.m02,
						      p.m11 - q.m11, p.m12 - q.m12,
									     p.m22 - q.m22
			);
		}
		public static SymMatrix operator *(SymMatrix p, float q)
		{
			return new SymMatrix
			(
				p.m00 * q, p.m01 * q, p.m02 * q,
					       p.m11 * q, p.m12 * q,
							      p.m22 * q
			);
		}
		public static SymMatrix operator /(SymMatrix p, float q)
		{
			return new SymMatrix
			(
				p.m00 / q, p.m01 / q, p.m02 / q,
					       p.m11 / q, p.m12 / q,
							      p.m22 / q
			);
		}

		public static Matrix operator *(SymMatrix p, SkewMatrix q)
		{
			var x = p.m12 * q.X;
			var y = p.m02 * q.Y;
			var z = p.m01 * q.Z;

			return new Matrix
			(
						z - y,			p.m02 * q.X - p.m00 * q.Z,	p.m00 * q.Y - p.m01 * q.X,
				p.m11 * q.Z - p.m12 * q.Y,			x - z,		p.m01 * q.Y - p.m11 * q.X,
				p.m12 * q.Z - p.m22 * q.Y,	p.m22 * q.X - p.m02 * q.Z,			y - x
			);
                }

		public override string ToString()
		{
			return $"{m00} {m01} {m02} | {m01} {m11} {m12} | {m02} {m12} {m22}";
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

		public static Transform operator -(Transform p)
                {
			return new Transform(-p.Ang, -p.Lin);
		}
		public static Transform operator +(Transform p, Transform q)
                {
			return new Transform(p.Ang + q.Ang, p.Lin + q.Lin);
		}
		public static Transform operator -(Transform p, Transform q)
		{
			return new Transform(p.Ang - q.Ang, p.Lin - q.Lin);
		}
		public static Transform operator *(Transform p, Transform q)
                {
			return new Transform(p.Ang * q.Ang, p.Lin + q.Lin);
                }
		public static Transform operator *(Transform p, float q)
                {
			return new Transform(p.Ang * q, p.Lin * q);
		}
		public static Transform operator /(Transform p, float q)
		{
			return new Transform(p.Ang / q, p.Lin / q);
		}

                public override string ToString()
                {
			return $"{Ang} || {Lin}";
                }
        }

	public struct SVector
        {
                public SVector(Vector ang, Vector lin)
                {
                        Ang = ang;
                        Lin = lin;
                }

                public Vector Ang { get; set; }
		public Vector Lin { get; set; }

		public static SVector operator -(SVector p)
                {
			return new SVector(-p.Ang, -p.Lin);
                }
		public static SVector operator +(SVector p, SVector q)
                {
			return new SVector(p.Ang + q.Ang, p.Lin + q.Lin);
		}
		public static SVector operator -(SVector p, SVector q)
		{
			return new SVector(p.Ang - q.Ang, p.Lin - q.Lin);
		}
		public static SVector operator *(SVector p, SVector q)
                {
			return new SVector(p.Lin * q.Lin + p.Ang * q.Ang, p.Ang * q.Lin);
                }
		public static SVector operator *(SVector p, float q)
                {
			return new SVector(p.Ang * q, p.Lin * q);
		}
		public static SVector operator /(SVector p, float q)
		{
			return new SVector(p.Ang / q, p.Lin / q);
		}

		public static float	 Dot(SVector p, SVector q)
                {
			return Vector.Dot(p.Ang, q.Ang) + Vector.Dot(p.Lin, q.Lin);
                }

		public static SVector   X(Vector p, SVector q)
		{
			var ang = q.Ang;
			var lin = q.Lin + q.Ang *p;

			return new SVector(ang, lin);
		}
		public static SVector XT(Vector p, SVector q)
		{
			var ang = q.Ang + q.Lin * p;
			var lin = q.Lin;

			return new SVector(ang, lin);
		}

		public override string ToString()
                {
			return $"{Ang} | {Lin}";
                }
        }

	public struct Inertia
	{
		public SymMatrix Ang { get; set; }
		public float Lin { get; set; }
		public Vector Center { get; set; }

		public void Rotate(Matrix rot)
                {
			Center = rot * Center;
                }
        }
}
