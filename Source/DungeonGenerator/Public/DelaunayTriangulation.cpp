
#include "DelaunayTriangulation.h"
#include "Math/Vector.h"
#include "DrawDebugHelpers.h"

DelaunayTriangulation::DelaunayTriangulation()
{
}

DelaunayTriangulation::DelaunayTriangulation(UWorld* world)
{
	//m_InputVertices = Vertices;
	this->world = world;
}

DelaunayTriangulation::~DelaunayTriangulation()
{
}

TArray<Triangle> DelaunayTriangulation::Triangulation(int32 FieldSizeX, int32 FieldSizeY, const TArray<Vertex>& InVertices)
{
	//This is an implementation of the Bowyer-Watson algorithm, it ensures all rooms are connected
	Triangle superTriangle = MakeSuperTriangle(FieldSizeX, FieldSizeY);
	TArray<Triangle> triangles;
	TArray<int32> indicesToRemove;
	triangles.Add(superTriangle);

	//Here begins the triangulation
	for (int32 i = 0; i < InVertices.Num(); i++)
	{
		TArray<Triangle> badTriangles;
		FVector currentVertex(InVertices[i].X, InVertices[i].Y, InVertices[i].Z);

		//Determine bad triangles ie: if a point is in a triangle's circumcircle, it is a bad triangle
		for (int32 j = 0; j < triangles.Num(); j++)
		{
			//Find circumcenter of current triangle
			FVector circumcenter = FindCircumcenter(triangles[j]);

			//Determine if point is in circumcircle		
			if (IsPointInCircle(triangles[j], circumcenter, currentVertex))
			{
				badTriangles.Add(triangles[j]);
			}
		}

		TArray<Edge> polygon;

		//Look for edges that are unique to one triangle
		for (int32 j = 0; j < badTriangles.Num(); j++)
		{
			TArray<Edge> triangleEdges{ badTriangles[j].AB, badTriangles[j].AC, badTriangles[j].BC };

			for (Edge edge : triangleEdges)
			{
				if (!IsNotUniqueEdge(badTriangles, edge, j))
				{
					polygon.Add(edge);
				}
			}
		}

		//Remove bad triangles from triangulation
		for (Triangle triangle : badTriangles)
		{
			int32 indexToRemove = triangles.IndexOfByKey(triangle);
			
			if (indexToRemove != INDEX_NONE)
			{
				triangles.RemoveAt(indexToRemove);
			}
		}

		//Make new triangles out of edges
		for (Edge edge : polygon)
		{
			triangles.Add(MakeNewTriangle(edge, InVertices[i]));
		}
	}

	//Remove any triangle that is connected to super triangle to eliminate the super triangle
	for (Triangle tri : triangles)
	{
		if (tri.ShareVertexWithSuperTriangle(superTriangle))
		{
			int32 indexToRemove = triangles.IndexOfByKey(tri);

			if (indexToRemove != INDEX_NONE)
			{
				indicesToRemove.Add(indexToRemove);
			}
		}
	}

	for (int32 j = 0; j < indicesToRemove.Num(); j++)
	{
		
		triangles.RemoveAt(indicesToRemove[j] - j);
	}

	return triangles;
}

Triangle DelaunayTriangulation::MakeSuperTriangle(int32 FieldSizeX, int32 FieldSizeY)
{
	float triangleScale = 2.f;
	//Make super triangle
	Vertex v_A{FieldSizeX / 2.f + FieldSizeX * triangleScale, FieldSizeY / 2.f, 0.f};
	Vertex v_B{ FieldSizeX / 2.f - FieldSizeX * triangleScale, FieldSizeY / 2.f - FieldSizeY * triangleScale, 0.f};
	Vertex v_C{FieldSizeX / 2.f - FieldSizeX * triangleScale , FieldSizeY / 2.f + FieldSizeY * triangleScale, 0.f };

	Edge e_A{ v_A, v_B };
	Edge e_B{ v_A, v_C };
	Edge e_C{ v_B, v_C };

	Triangle A{ v_A, v_B, v_C, e_A, e_B, e_C };

	//DrawDebugLine(world, FVector(v_A.X, v_A.Y, 0.f), FVector(v_B.X, v_B.Y, 0.f), FColor::Green, true, -1.f, 0, 20.f);
	//DrawDebugLine(world, FVector(v_A.X, v_A.Y, 0.f), FVector(v_C.X, v_C.Y, 0.f), FColor::Green, true, -1.f, 0, 20.f);
	//DrawDebugLine(world, FVector(v_B.X, v_B.Y, 0.f), FVector(v_C.X, v_C.Y, 0.f), FColor::Green, true, -1.f, 0, 20.f);

	return A;
}

FVector DelaunayTriangulation::FindCircumcenter(const Triangle& InTriangle)
{
	FVector A(InTriangle.A.X, InTriangle.A.Y, InTriangle.A.Z);
	FVector B(InTriangle.B.X, InTriangle.B.Y, InTriangle.B.Z);
	FVector C(InTriangle.C.X, InTriangle.C.Y, InTriangle.C.Z);

	float d = 2 * (A.X * (B.Y - C.Y) + B.X * (C.Y - A.Y) + C.X * (A.Y - B.Y));
	float centerX = ((A.X * A.X + A.Y * A.Y) * (B.Y - C.Y) + (B.X * B.X + B.Y * B.Y) * (C.Y - A.Y) + (C.X * C.X + C.Y * C.Y) * (A.Y -B.Y)) / d;
	float centerY = ((A.X * A.X + A.Y * A.Y) * (C.X - B.X) + (B.X * B.X + B.Y * B.Y) * (A.X - C.X) + (C.X * C.X + C.Y * C.Y) * (B.X - A.X)) / d;

	FVector center(centerX, centerY, 0.f);

	//DrawDebugPoint(world, center, 20.f ,FColor::Green, true, -1.f);

	return center;
}

Triangle DelaunayTriangulation::MakeNewTriangle(const Edge& edge, const Vertex& vertex)
{
	Triangle newTri(edge.A, edge.B, vertex, edge, Edge(edge.A, vertex), Edge(edge.B, vertex));
	return newTri;
}

bool DelaunayTriangulation::IsPointInCircle(const Triangle& triangle, const FVector& center, const FVector& vertex)
{
	FVector triangleVertex(triangle.A.X, triangle.A.Y, triangle.A.Z);

	//DrawDebugCircle(world, center, FVector::Dist(center , triangleVertex), 100, FColor::Green, true, -1.f, 0, 25.f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), false);

	return (FVector::Dist(vertex, center) < FVector::Dist( triangleVertex, center));
}

bool DelaunayTriangulation::IsNotUniqueEdge(const TArray<Triangle>& triangles, const Edge& edge, const int32 index)
{
	for (int32 i = 0; i < triangles.Num(); i++)
	{
		if (i != index)
		{
			if (triangles[i] == edge)
			{
				return true;
			}
		}
	}

	return false;
}
