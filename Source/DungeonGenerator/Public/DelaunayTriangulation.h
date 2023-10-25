#pragma once

#include "CustomStructs.h"
#include "Engine/World.h"

class DelaunayTriangulation
{
public:
	DelaunayTriangulation();

	//Only for debugging purposes
	DelaunayTriangulation(UWorld* world);

	~DelaunayTriangulation();

	TArray<Triangle> Triangulation(int32 FieldSizeX, int32 FieldSizeY,const TArray<Vertex>&);
	Triangle MakeSuperTriangle(int32, int32);
	FVector FindCircumcenter(const Triangle&);

private:
	TArray<Vertex> m_InputVertices;

	//Only for debugging purposes
	UWorld* world;

	
	Triangle MakeNewTriangle(const Edge&, const Vertex&);
	bool IsPointInCircle(const Triangle&, const FVector&, const FVector&);
	bool IsNotUniqueEdge(const TArray<Triangle>&, const Edge&, const int32);
};