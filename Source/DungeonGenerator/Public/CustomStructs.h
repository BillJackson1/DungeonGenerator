#pragma once

#include "MathUtil.h"

struct Triangle;

struct FCoordinates
{
	int32 MinX;
	int32 MinY;
	int32 MaxX;
	int32 MaxY;

	inline bool operator==(const FCoordinates& other) const
	{
		return (MinX == other.MinX && MinY == other.MinY && MaxX == other.MaxX && MaxY == other.MaxY);
	}
};

struct IntVector
{
	int32 X = 0;
	int32 Y = 0;

	inline bool operator==(const IntVector& other) const
	{
		return (X == other.X && Y == other.Y);
	}
	inline bool operator!=(const IntVector& other) const 
	{
		return(X != other.X || Y != other.Y);
	}
};

struct Vertex
{
	float X;
	float Y;
	float Z;

	Vertex(float x = 0.f, float y = 0.f, float z = 0.f)
	{
		X = x;
		Y = y;
		Z = z;
	};
	Vertex(const FVector& vector)
	{
		*this = vector;
	}

	FVector ToVector()
	{
		FVector vector(X, Y, Z);
		return vector;
	}

	inline Vertex& operator=(const FVector& vector)
	{
		X = vector.X;
		Y = vector.Y;
		Z = vector.Z;

		return *this;
	}
	inline bool operator==(const Vertex& vertex) const
	{
		FLT_EPSILON;

		float diffX = abs(X - vertex.X);
		float diffY = abs(Y - vertex.Y);
		float diffZ = abs(Z - vertex.Z);

		return (diffX < FLT_EPSILON && diffY < FLT_EPSILON && diffZ < FLT_EPSILON);
	}
	//2D version
	inline bool operator<(const Vertex& vertex) const
	{
		float diffX = abs(X - vertex.X);
		float diffY = abs(Y - vertex.Y);
		float diffZ = abs(Z - vertex.Z);

		if (diffX < FLT_EPSILON)
		{
			return Y < vertex.Y;
		}

		return (X < vertex.X);
	}
};

struct Edge
{
	Vertex A;
	Vertex B;

	Edge(Vertex a = Vertex(), Vertex b = Vertex()) { A = a; B = b; };

	inline bool operator==(const Edge& other) const
	{
		return (A == other.A && B == other.B);
	}
};

struct Triangle
{
	Vertex A;
	Vertex B;
	Vertex C;

	Edge AB;
	Edge AC;
	Edge BC;

	Triangle(Vertex a = Vertex(), Vertex b = Vertex(), Vertex c = Vertex(), Edge ab = Edge(), Edge ac = Edge(), Edge bc = Edge())
	{
		A = a;
		B = b;
		C = c;
		AB = ab;
		AC = ac;
		BC = bc;
	}

	bool ShareVertexWithSuperTriangle(const Triangle& b)
	{
		return (this->A == b.A || this->A == b.B || this->A == b.C ||
				this->B == b.A || this->B == b.B || this->B == b.C ||
				this->C == b.A || this->C == b.B || this->C == b.C);
	}

	//This allows to check if an edge is part of a triangle
	inline const bool operator==(const Edge& other) const
	{
		return (AB == other || AC == other || BC == other);
	}

	inline const bool operator==(const Triangle& other) const 
	{
		return (A == other.A && B == other.B && C == other.C);
	}
};

struct GridNode
{
	FCoordinates nodeCoords;
	IntVector nodeIndex;
	int32 nodeWeight;
};