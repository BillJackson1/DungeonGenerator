// Fill out your copyright notice in the Description page of Project Settings.


#include "Generator.h"
#include <vector>
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

// Sets default values
AGenerator::AGenerator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	m_PlayAreaCoords = {0};
}

// Called when the game starts or when spawned
void AGenerator::BeginPlay()
{
	Super::BeginPlay();

	DefinePlayArea();

	uint32 currentRoomIndex = 0;

	//Create rooms
	for (int32 i = 0; i < m_NbRooms; i++)
	{
		AddRoom(currentRoomIndex);
	}

	DelaunayTriangulation delaunay(GetWorld());

	TArray<Vertex> roomVertices = GetRoomCenter();

	//Create the connections to each room
	TArray<Triangle> triangulation = delaunay.Triangulation(m_PlayAreaSizeX, m_PlayAreaSizeY, roomVertices);
	//DrawTriangulation(triangulation, delaunay );
	TArray<Edge> MST = GetMST(triangulation);
	//DrawMST(MST);

	//Create World grid and set node weights
	TArray<TArray<GridNode>> grid; 
	grid = MakeGrid();
	SetGridNodeWeights(grid);
	//DrawGrid(grid);

	//Create path between rooms
	TArray<TArray<IntVector>> hallways = GenerateHallways(grid, MST);

	DressRooms();
	ApplyHallVisuals(hallways, grid);

	int32 startRoomIndex = FMath::RandRange(0, m_Rooms.Num() - 1);
	FVector startPos = m_Rooms[startRoomIndex]->GetCenter();
	startPos.Z += 200.f;
	UGameplayStatics::GetPlayerPawn(GetWorld(), 0)->SetActorLocation(startPos);
}

// Called every frame
void AGenerator::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AGenerator::DefinePlayArea()
{
	m_PlayAreaCoords.MinX = 0;
	m_PlayAreaCoords.MinY = 0;
	m_PlayAreaCoords.MaxX = m_PlayAreaSizeX * m_GridCellSize;
	m_PlayAreaCoords.MaxY = m_PlayAreaSizeY * m_GridCellSize;

	UE_LOG(LogTemp, Warning, TEXT("Size x: %d  Size Y: %d"), m_PlayAreaSizeX * m_GridCellSize, m_PlayAreaSizeY * m_GridCellSize);

	FVector center = FVector(m_PlayAreaSizeX / 2, m_PlayAreaSizeY / 2, 0.f);

	DrawSquare(center, center);
}

void AGenerator::AddRoom(uint32& currentRoomIndex)
{
	IntVector roomSize;
	IntVector roomExtent;
	FVector roomLocation;
	FCoordinates cornerCoordinates;
	int32 maxPlacementAttempts = 0;
	do 
	{
		maxPlacementAttempts++;

		roomSize = DefineRoomSize();
		roomExtent = FindRoomExtent(roomSize);
		roomLocation = SelectRoomLocation(roomExtent, cornerCoordinates);

	} while (maxPlacementAttempts < 100 && CheckOverlap(m_Rooms, cornerCoordinates, currentRoomIndex));

	if (maxPlacementAttempts >= 99)
	{
		return;
	}

	FVector extent = FVector(roomExtent.X, roomExtent.Y, 0.f);
	DrawSquare(roomLocation, extent);

	//DrawDebugCircle(GetWorld(), roomLocation, 200.f, 100, FColor::Green, true, -1.f, 0, 20.f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), true);
	m_Rooms.Add(GetWorld()->SpawnActor<ARoom>(roomLocation, FRotator()));
	m_Rooms[currentRoomIndex]->SetRoomCoordinates(cornerCoordinates, roomLocation, roomSize);

	currentRoomIndex++;
}

IntVector AGenerator::DefineRoomSize()
{
	IntVector roomSize;

	roomSize.X = FMath::RandRange(5, m_RoomSizeX) * m_GridCellSize;
	roomSize.Y = FMath::RandRange(5, m_RoomSizeY) * m_GridCellSize;

	return roomSize;
}

IntVector AGenerator::FindRoomExtent(const IntVector roomSize)
{
	IntVector roomExtent;

	roomExtent.X = roomSize.X / 2;
	roomExtent.Y = roomSize.Y / 2;

	return roomExtent;
}

FVector AGenerator::SelectRoomLocation(const IntVector roomExtent, FCoordinates& coords)
{
	FVector roomLocation;

	int32 x = m_PlayAreaSizeX / m_GridCellSize;
	int32 y = m_PlayAreaSizeY / m_GridCellSize;

	int32 minX = FMath::RandRange(0, x) * m_GridCellSize - roomExtent.X * 2;
	int32 minY = FMath::RandRange(0, y) * m_GridCellSize - roomExtent.Y * 2;

	if (minX < 0)
	{
		minX += abs(minX);
	}
	if (minY < 0)
	{
		minY += abs(minY);
	}

	coords.MinX = minX;
	coords.MinY = minY;
	coords.MaxX = minX + roomExtent.X * 2;
	coords.MaxY = minY + roomExtent.Y * 2;

	float locationX = minX + roomExtent.X;
	float locationY = minY + roomExtent.Y;

	roomLocation = FVector(locationX, locationY, 0.f);

	return roomLocation;
}

TArray<Edge> AGenerator::GetMST(const TArray<Triangle>& triangles)
{
	//This is an implementation of Prim's algorithm. It removes all unecessary edges, ensuring there is no cycles in the graph
	TArray<Edge> edges;

	//Here I sorted all the edges. It was a personal preference but I don't think its necessary to do so
	for (Triangle tri : triangles)
	{
		TArray<Edge> triEdges{ tri.AB, tri.AC, tri.BC };

		for (Edge e : triEdges)
		{
			Edge sortedEdge;
			if (e.A< e.B)
			{
				sortedEdge.A = e.A;
				sortedEdge.B = e.B;
			}
			else
			{
				sortedEdge.A = e.B;
				sortedEdge.B = e.A;
			}


			edges.AddUnique(sortedEdge);
		}
	}

	edges.Sort([](const Edge& a, const Edge b)
		{
			return (a.A < b.A);
		}
	);


	//Add all room verices to sorted array, again probably not necessary, it just made sense to me to sort everything
	TArray<Vertex> nodes;

	for (ARoom* room : m_Rooms)
	{
		nodes.Add(room->GetCenter());
	}

	nodes.Sort([](const Vertex& a, const Vertex& b)
		{
			return a < b;
		}
	);

	//Here we create an adjacency list for faster navigation of the graph
	TArray<TArray<int32>> AdjList;

	for (int32 i = 0; i < nodes.Num(); i++)
	{
		AdjList.Add(TArray<int32>());
		for (Edge e : edges)
		{
			if (nodes[i] == e.A)
			{
				AdjList[i].Add(nodes.IndexOfByKey(e.B));
			}
			else if (nodes[i] == e.B)
			{
				AdjList[i].Add(nodes.IndexOfByKey(e.A));
			}			
		}
	}

	//Start of MST Algo
	std::vector<int> parents(nodes.Num());
	std::vector<int> weights(nodes.Num(), INT_MAX);
	std::vector<bool> visited(nodes.Num(), false);
	TArray<Edge> discardedEdges;
	parents[0] = -1;
	weights[0] = 0;

	//Find the next lowest cost, non-visited vertex
	for (int32 i = 0; i < nodes.Num(); i++)
	{
		int32 currentNode = SelectMinVertex(weights, visited);
		visited[currentNode] = true;

		//Here we keep only the shortest connection between two adjacent vertices, discarding the others
		for (int32 j = 0; j < AdjList[i].Num(); j++)
		{
			if (AdjList[i][j] != 0 && !visited[AdjList[i][j]] && AdjList[i][j] < weights[AdjList[i][j]])
			{
				//Add reference to shortest adjacent node 
				weights[AdjList[i][j]] = AdjList[i][j];
				parents[AdjList[i][j]] = currentNode;
			}
			else
			{
				//Add discarded reference to new structure for later use

				Edge edge(nodes[i], nodes[AdjList[i][j]]);
				discardedEdges.Add(edge);
			}
		}
	}

	TArray<Edge> MST;

	//Convert references to actual edges
	for (int32 i = 0; i < parents.size(); i++)
	{
		if (parents[i] >= 0)
		{
			MST.Add(Edge(nodes[parents[i]], nodes[i]));
		}
	}

	//Re-add some of the discarded references as edges to create cycles to make things more interesting
	for(Edge edge : discardedEdges)
	{
		float diceRoll = FMath::FRandRange(0.f, 100.f);

		if (diceRoll <= m_DungeonCycleChance)
		{
			MST.Add(edge);
		}
		
	}

	return MST;
}

int32 AGenerator::SelectMinVertex(const std::vector<int>& weights, const std::vector<bool>& visited)
{
	//Returns the next non-visited vertex with the lowest weight

	int32 min = INT_MAX;
	int32 vertIndex;

	for (int32 i = 0; i < weights.size(); i++)
	{
		if (!visited[i] && weights[i] < min)
		{
			vertIndex = i;
			min = weights[i];
		}
	}

	return vertIndex;
}

TArray<TArray<GridNode>> AGenerator::MakeGrid()
{
	TArray<TArray<GridNode>> grid;
	int32 iLoop = m_PlayAreaSizeX / m_GridCellSize;
	int32 jLoop = m_PlayAreaSizeY / m_GridCellSize;

	UE_LOG(LogTemp, Warning, TEXT("iLoop size: %d  jLoopSize: %d"), iLoop, jLoop);

	for (int32 i = 0; i < iLoop; i++)
	{
		grid.Add(TArray<GridNode>());
		for (int32 j = 0; j < jLoop; j++)
		{
			FCoordinates coords;
			coords.MinX = i * m_GridCellSize;
			coords.MinY = j * m_GridCellSize;
			coords.MaxX = coords.MinX + m_GridCellSize;
			coords.MaxY = coords.MinY + m_GridCellSize;

			GridNode cell;
			cell.nodeCoords = coords;
			cell.nodeWeight = 1;
			cell.nodeIndex = {i, j};
			grid[i].Add(cell);
		}
	}

	return grid;
}

void AGenerator::SetGridNodeWeights(TArray<TArray<GridNode>>& grid)
{
	for (int32 i = 0; i < grid.Num(); i++)
	{
		for (int32 j = 0; j < grid[0].Num(); j++)
		{
			IntVector currentIndex{ i, j };
			for (ARoom* r : m_Rooms)
			{
				if (CheckCellIsRoom(*r, grid[i][j]))
				{
					grid[i][j].nodeWeight = 1000;
				}
			}
		}
	}

	for (int i = 0; i < grid.Num(); i++)
	{
		for (int j = 0; j < grid[0].Num(); j++)
		{
			if (IsNodeRoomNeighbor(grid, IntVector{ i, j }))
			{
				grid[i][j].nodeWeight = 500;
			}
		}
	}
}

bool AGenerator::IsNodeRoomNeighbor(const TArray<TArray<GridNode>>& grid, const IntVector& index)
{
	for (int i = -1; i < 2; i++)
	{
		if (index.X + i < 0 || index.X + i >= grid.Num())
		{
			continue;
		}
		for (int j = -1; j < 2; j++)
		{
			if (index.Y + j < 0 || index.Y + j >= grid[0].Num())
			{
				continue;
			}

			if ( grid[index.X][index.Y].nodeWeight < 1000 && grid[index.X + i][index.Y + j].nodeWeight >= 1000)
			{
				return true;
			}
		}
	}
	return false;
}

TArray<TArray<IntVector>> AGenerator::GenerateHallways(TArray<TArray<GridNode>>& grid, const TArray<Edge>& connections)
{
	TArray<TArray<IntVector>> hallways;
	//Cycle through each connections to make halls from one room to another
	for (Edge e : connections)
	{
		FCoordinates start;
		FCoordinates end;

		//Find out which room is the start and which one is the end
		for (const ARoom* r : m_Rooms)
		{
			Vertex room(r->GetCenter().X, r->GetCenter().Y, r->GetCenter().Z);
			if (room == e.A)
			{
				start.MinX = r->GetCoordinates().MinX;
				start.MinY = r->GetCoordinates().MinY;
				start.MaxX = start.MinX + m_GridCellSize;
				start.MaxY = start.MinY + m_GridCellSize;

			}
			else if (room == e.B)
			{
				end.MinX = r->GetCoordinates().MinX;
				end.MinY = r->GetCoordinates().MinY;
				end.MaxX = end.MinX + m_GridCellSize;
				end.MaxY = end.MinY + m_GridCellSize;
			}
		}

		IntVector startIndex;
		IntVector endIndex;

		//Find the node index in the graph
		for (int32 i = 0; i < grid.Num(); i++)
		{
			for (int32 j = 0; j < grid[i].Num(); j++)
			{
				if (grid[i][j].nodeCoords == start)
				{
					startIndex = IntVector{ i,j };
				}
				else if (grid[i][j].nodeCoords == end)
				{
					endIndex = IntVector{ i,j };
				}
			}
		}

		//Find a starting node outside the room 
		//Find which direction we are going
		EDirection dir = GetPathDirection(grid, startIndex, endIndex);
		IntVector startNodeIndex = GetPathStartingNode(startIndex, dir, grid);

		//Find end node outside target room and direction from start node
		dir = GetPathDirection(grid, startNodeIndex, endIndex);
		IntVector endNodeIndex = GetPathEndNode(endIndex, dir, grid);

		//Find path
		TArray<IntVector> path = GetPath(startNodeIndex, endNodeIndex, grid, dir);
		hallways.Add(path);
		//for (IntVector index : path)
		//{
		//	grid[index.X][index.Y].nodeWeight = 0;
		//	FVector center(grid[index.X][index.Y].nodeCoords.MinX + m_GridCellSize / 2, grid[index.X][index.Y].nodeCoords.MinY + m_GridCellSize / 2, 0.f);
		//	FVector extent(m_GridCellSize / 2, m_GridCellSize / 2, 0.f);

		//	DrawDebugBox(GetWorld(), center, extent, FColor::White, true, -1.f, -2, 25.f);
		//}
	}
	return hallways;
}

AGenerator::EDirection  AGenerator::GetPathDirection(const TArray<TArray<GridNode>>& grid, const IntVector& startIndex, const IntVector& endIndex)
{
	EDirection direction;
	FCoordinates startCoords = grid[startIndex.X][startIndex.Y].nodeCoords;
	FCoordinates endCoords = grid[endIndex.X][endIndex.Y].nodeCoords;
	FVector dir = FVector(endCoords.MinX, endCoords.MinY, 0.f) - FVector(startCoords.MinX, startCoords.MinY, 0.f);
	dir.Normalize();
	//UE_LOG(LogTemp, Warning, TEXT("Dir vector is:  X: %f Y: %f"), dir.X, dir.Y);
	float dotX = (FVector::DotProduct(FVector::ForwardVector, dir));
	float dotY = (FVector::DotProduct(FVector::RightVector, dir));

	if (dotX > 0.5f)
	{
		direction = EDirection::NORTH;
	}
	else if (dotX < -0.5f)
	{
		direction = EDirection::SOUTH;
	}
	else if (dotY > 0.f)
	{
		direction = EDirection::EAST;
	}
	else
	{
		direction = EDirection::WEST;
	}

	return direction;
}

IntVector AGenerator::GetPathStartingNode(const IntVector& startRoomIndex, const EDirection& targetDirection, const TArray<TArray<GridNode>>& grid)
{
	//In here, the idea is to find the starting node outside of the starting room, on the proper side of the room

	IntVector startNodeIndex;
	GridNode currentNode = grid[startRoomIndex.X][startRoomIndex.Y];
	int indexIncrement = 1;

	//Need to get reference to actual room object to center the start node
	ARoom* room = nullptr;
	for (int i = 0; i < m_Rooms.Num(); i++)
	{
		if (grid[startRoomIndex.X][startRoomIndex.Y].nodeCoords.MinX == m_Rooms[i]->GetCoordinates().MinX && grid[startRoomIndex.X][startRoomIndex.Y].nodeCoords.MinY == m_Rooms[i]->GetCoordinates().MinY)
		{
			room = m_Rooms[i];
			break;
		}
	}

	switch (targetDirection)
	{
	case EDirection::NORTH:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if ((startRoomIndex.X + indexIncrement) >= grid.Num())
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}
			currentNode = grid[startRoomIndex.X + indexIncrement][startRoomIndex.Y];
			startNodeIndex.X = startRoomIndex.X + indexIncrement;
			startNodeIndex.Y = startRoomIndex.Y;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			startNodeIndex = { startNodeIndex.X, startNodeIndex.Y + room->GetRoomSize().Y / m_GridCellSize / 2 };
		}
	}
	break;
	case EDirection::SOUTH:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if ((startRoomIndex.X - indexIncrement) < 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}
			currentNode = grid[startRoomIndex.X - indexIncrement][startRoomIndex.Y];
			startNodeIndex.X = startRoomIndex.X - indexIncrement;
			startNodeIndex.Y = startRoomIndex.Y;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			 startNodeIndex = { startNodeIndex.X, startNodeIndex.Y + room->GetRoomSize().Y / m_GridCellSize / 2 };
		}
	}
		break;
	case EDirection::EAST:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if (startRoomIndex.Y + indexIncrement >= grid[startRoomIndex.X].Num())
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}

			currentNode = grid[startRoomIndex.X][startRoomIndex.Y + indexIncrement];
			startNodeIndex.X = startRoomIndex.X;
			startNodeIndex.Y = startRoomIndex.Y + indexIncrement;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			startNodeIndex = { startNodeIndex.X + room->GetRoomSize().X / m_GridCellSize / 2 , startNodeIndex.Y };
		}
	}
		break;
	case EDirection::WEST:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if (startRoomIndex.Y - indexIncrement < 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}

			currentNode = grid[startRoomIndex.X][startRoomIndex.Y - indexIncrement];
			startNodeIndex.X = startRoomIndex.X;
			startNodeIndex.Y = startRoomIndex.Y - indexIncrement;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			startNodeIndex = { startNodeIndex.X + room->GetRoomSize().X / m_GridCellSize / 2 , startNodeIndex.Y };
		}
	}
		break;
	default:
		break;
	}


	

	return startNodeIndex;
}

IntVector AGenerator::GetPathEndNode(const IntVector& endRoomIndex, const EDirection& fromDirection, const TArray<TArray<GridNode>>& grid)
{
	//Same as GetPathStartingNode but for the end room.
	IntVector endNodeIndex;
	GridNode currentNode = grid[endRoomIndex.X][endRoomIndex.Y];
	int indexIncrement = 1;
	
	//Find actual room ref to center end node
	ARoom* room = nullptr;
	for (int i = 0; i < m_Rooms.Num(); i++)
	{
		if (grid[endRoomIndex.X][endRoomIndex.Y].nodeCoords.MinX == m_Rooms[i]->GetCoordinates().MinX && grid[endRoomIndex.X][endRoomIndex.Y].nodeCoords.MinY == m_Rooms[i]->GetCoordinates().MinY)
		{
			room = m_Rooms[i];
			break;
		}
	}

	switch (fromDirection)
	{
	case EDirection::NORTH:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if ((endRoomIndex.X - indexIncrement) < 0 )
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}
			currentNode = grid[endRoomIndex.X - indexIncrement][endRoomIndex.Y];
			endNodeIndex.X = endRoomIndex.X - indexIncrement;
			endNodeIndex.Y = endRoomIndex.Y;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			endNodeIndex = { endNodeIndex.X, endNodeIndex.Y + room->GetRoomSize().Y / m_GridCellSize / 2 };
		}
	}
	break;
	case EDirection::SOUTH:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if ((endRoomIndex.X + indexIncrement) >= grid.Num())
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}
			currentNode = grid[endRoomIndex.X + indexIncrement][endRoomIndex.Y];
			endNodeIndex.X = endRoomIndex.X + indexIncrement;
			endNodeIndex.Y = endRoomIndex.Y;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			endNodeIndex = { endNodeIndex.X, endNodeIndex.Y + room->GetRoomSize().Y / m_GridCellSize / 2 };
		}
	}
	break;
	case EDirection::EAST:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if (endRoomIndex.Y - indexIncrement < 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}

			currentNode = grid[endRoomIndex.X][endRoomIndex.Y - indexIncrement];
			endNodeIndex.X = endRoomIndex.X;
			endNodeIndex.Y = endRoomIndex.Y - indexIncrement;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			endNodeIndex = { endNodeIndex.X + room->GetRoomSize().X / m_GridCellSize / 2, endNodeIndex.Y };
		}
	}
	break;
	case EDirection::WEST:
	{
		while (currentNode.nodeWeight >= 1000)
		{
			if (endRoomIndex.Y + indexIncrement >= grid[endRoomIndex.X].Num())
			{
				UE_LOG(LogTemp, Warning, TEXT("Index out of bounds"));
				break;
			}

			currentNode = grid[endRoomIndex.X][endRoomIndex.Y + indexIncrement];
			endNodeIndex.X = endRoomIndex.X;
			endNodeIndex.Y = endRoomIndex.Y + indexIncrement;
			indexIncrement++;
		}

		if (room != nullptr)
		{
			endNodeIndex = { endNodeIndex.X + room->GetRoomSize().X / m_GridCellSize / 2, endNodeIndex.Y};
		}
	}
	break;
	default:
		break;
	}

	UE_LOG(LogTemp, Warning, TEXT("Node weight is: %d"), grid[endNodeIndex.X][endNodeIndex.Y].nodeWeight);
	return endNodeIndex;
}

TArray<IntVector> AGenerator::GetPath(const IntVector& startIndex, const IntVector& endIndex, TArray<TArray<GridNode>>& grid, const EDirection& direction)
{
	//This is an implmentation of the A* algorithm
	TArray<IntVector> open;
	
	std::vector<std::vector<IntVector>> cameFrom(grid.Num(), std::vector<IntVector>(grid[0].Num()));
	std::vector<std::vector<int>> f_Score(grid.Num(), std::vector<int>(grid[0].Num(), INT_MAX));
	std::vector<std::vector<int>> g_Score(grid.Num(), std::vector<int>(grid[0].Num(), INT_MAX));
	
	open.Add(startIndex);

	g_Score[startIndex.X][startIndex.Y] = 0;
	f_Score[startIndex.X][startIndex.Y] = GetHCost(startIndex, endIndex);

	while (!open.IsEmpty())
	{
		int32 currentIndex;
		IntVector current;

		//Find node with lowest f_cost and remove it from open list
		currentIndex = GetNodeWithSmallestWeight(open, f_Score);
		current = open[currentIndex];
		
		if (current == endIndex)
		{			
			TArray<IntVector> path = ConstructPath(cameFrom, current, startIndex);
			return path;
		}

		open.RemoveAt(currentIndex);

		//Find neighbors
		for (int i = -1; i < 2; i++)
		{
			//OOB check
			if (current.X + i < 0 || current.X + i >= grid.Num())
			{
				continue;
			}

			for (int j = -1; j < 2; j++)
			{
				//OOB check
				if (current.Y + j < 0 || current.Y + j >= grid[0].Num())
				{
					continue;
				}
				//Ignore diagonal neighbors
				if (abs(i) == abs(j))
				{
					continue;
				}

				IntVector neighbor{current.X + i, current.Y + j};
				int32 temp_g_Score = g_Score[current.X][current.Y] + 1 + grid[neighbor.X][neighbor.Y].nodeWeight;

				//Check if current path to neighbor is cheaper than any path stored
				if (temp_g_Score < g_Score[neighbor.X][neighbor.Y] && grid[neighbor.X][neighbor.Y].nodeWeight < 1000)
				{					
					cameFrom[neighbor.X][neighbor.Y] = current;
					g_Score[neighbor.X][neighbor.Y] = temp_g_Score;
					f_Score[neighbor.X][neighbor.Y] = temp_g_Score + GetHCost(neighbor, endIndex);

					//We need to evaluate this neighbor's neighbors later, so add it to open list if it isn't present
					if (!open.Contains(current))
					{						
						open.Add(neighbor);
					}
				}
			}
		}

	}

	//Empty Array if no path is found
 	return TArray<IntVector>();
}

TArray<IntVector> AGenerator::ConstructPath(const std::vector<std::vector<IntVector>>& cameFrom, IntVector& current, const IntVector& start)
{
	//Here we build the path by backtracking from end to start
	TArray<IntVector> path;
	path.Add(current);
	
	while (current != start)
	{
		current = cameFrom[current.X][current.Y];
		path.Insert(current, 0);
	}

	return path;
}

void AGenerator::DressHallways(const TArray<TArray<IntVector>>& hallways, const TArray<TArray<GridNode>>& grid)
{
	for (TArray<IntVector> hallway : hallways)
	{
		int32 segmentCounter = 0;
		FTransform transform(FRotator(0.f, 0.f, 0.f), FVector(0.f, 0.f, 0.f), FVector(1.f, 1.f, 1.f));
		UProceduralMeshComponent* mesh = Cast<UProceduralMeshComponent>(AddComponentByClass(UProceduralMeshComponent::StaticClass(), false, transform, false));
		
		for (IntVector tile : hallway)
		{
			FVector v1(grid[tile.X][tile.Y].nodeCoords.MinX, grid[tile.X][tile.Y].nodeCoords.MinY, 0.f);
			FVector v2(grid[tile.X][tile.Y].nodeCoords.MaxX, grid[tile.X][tile.Y].nodeCoords.MinY, 0.f);
			FVector v3(grid[tile.X][tile.Y].nodeCoords.MaxX, grid[tile.X][tile.Y].nodeCoords.MaxY, 0.f);
			FVector v4(grid[tile.X][tile.Y].nodeCoords.MinX, grid[tile.X][tile.Y].nodeCoords.MaxY, 0.f);

			TArray<FVector> vertices{ v1, v2, v3, v4 };
			TArray<int32> triangles{ 0, 2, 1,
									 0, 3, 2 };

			TArray<FVector> normals{ FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f) };
			TArray<FVector2D> UV0{ FVector2D(0, 0), FVector2D(0, 1), FVector2D(1, 1), FVector2D(1, 0) };
			TArray<FProcMeshTangent> tangents{ FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f) };

			mesh->CreateMeshSection(segmentCounter, vertices, triangles, normals, UV0, TArray<FColor>(), tangents, true);
			mesh->SetMaterial(segmentCounter	, m_HallwayMaterial);
			segmentCounter++;
		}

		
		mesh->RegisterComponent();
		//m_HallwayMeshes.Add(mesh);
	}


}

int32 AGenerator::GetNodeWithSmallestWeight(const TArray<IntVector>& openList, const std::vector<std::vector<int>>& f_Score )
{
	int32 index = 0;;
	int32 lowest_f_Score = INT_MAX;
	for (int32 i = 0; i < openList.Num(); i++)
	{
		if (f_Score[openList[i].X][openList[i].Y] < lowest_f_Score)
		{
			lowest_f_Score = f_Score[openList[i].X][openList[i].Y];
			index = i;
		}
	}

	return index;
}

int32 AGenerator::GetHCost(const IntVector& current, const IntVector& end)
{
	int32 h = abs(current.X - end.X) + abs(current.Y - end.Y);
	return h;
}

bool AGenerator::CheckOverlap(const TArray<ARoom*>& rooms, const FCoordinates& roomCoordinates, const uint32 currentRoomIndex)
{
	FCoordinates B = roomCoordinates;

	for(uint32 i = 0; i < currentRoomIndex; i++)
	{
		FCoordinates A = rooms[i]->GetCoordinates();

		if (A.MinX - m_MinSpacing * m_GridCellSize < B.MaxX && A.MaxX + m_MinSpacing * m_GridCellSize > B.MinX && A.MinY - m_MinSpacing * m_GridCellSize < B.MaxY && A.MaxY + m_MinSpacing * m_GridCellSize > B.MinY)
		{
			return true;
		}
	} 

	return false;
}

bool AGenerator::CheckCellIsRoom(const ARoom& room, const GridNode& cell)
{
	FCoordinates roomCoords(room.GetCoordinates());
	FCoordinates cellCoords(cell.nodeCoords);

	if (cellCoords.MinX >= roomCoords.MinX && cellCoords.MinY >= roomCoords.MinY && cellCoords.MaxX <= roomCoords.MaxX && cellCoords.MaxY <= roomCoords.MaxY)
	{
		return true;
	}

	return false;
}

void AGenerator::DrawSquare( const FVector& center, const FVector& extent)
{
	DrawDebugBox(GetWorld(), center, extent, FColor::Yellow, true, -1.f, 0, 10.f);
}

void AGenerator::DrawTriangulation(const TArray<Triangle>& triangulation, DelaunayTriangulation& delaunay)
{
	for (Triangle tri : triangulation)
	{
		FVector A = tri.A.ToVector();
		FVector B = tri.B.ToVector();
		FVector C = tri.C.ToVector();

		DrawDebugLine(GetWorld(), A, B, FColor::Green, true, -1.f, 0, 20.f);
		DrawDebugLine(GetWorld(), A, C, FColor::Green, true, -1.f, 0, 20.f);
		DrawDebugLine(GetWorld(), B, C, FColor::Green, true, -1.f, 0, 20.f);

		//FVector center = delaunay.FindCircumcenter(tri);
		//DrawDebugCircle(GetWorld(), center, FVector::Dist(center, tri.A.ToVector()), 100.f, FColor::Red, true, -1.f, 0, 20.f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), false);
	}
}

void AGenerator::DrawMST(const TArray<Edge>& MST)
{
	for (Edge e : MST)
	{
		DrawDebugLine(GetWorld(), e.A.ToVector(), e.B.ToVector(), FColor::Blue, true, -1.f, 0, 20.f);	
	}
}

void AGenerator::DrawGrid(const TArray<TArray<GridNode>>& grid)
{
	FVector extent((float)m_GridCellSize / 2.f, (float)m_GridCellSize / 2.f, 0.f);
	
	for (int32 i = 0; i < grid.Num(); i++)
	{
		for (int32 j = 0; j < grid[i].Num(); j++)
		{
			FColor gridColor(FColor::Green);
			if (grid[i][j].nodeWeight >= 1000)
			{
				gridColor = FColor::Red;
			}
			else if (grid[i][j].nodeWeight == 500)
			{
				gridColor = FColor::Blue;
			}
			FVector center(grid[i][j].nodeCoords.MinX + (m_GridCellSize / 2), grid[i][j].nodeCoords.MinY + (m_GridCellSize / 2), 0.f);
			DrawDebugBox(GetWorld(), center, extent, gridColor, true, -1.f, 0, 15.f);
		}
	}
}

TArray<Vertex> AGenerator::GetRoomVertices()
{
	TArray<Vertex> roomVertices;

	for (int32 i = 0; i < m_Rooms.Num(); i++)
	{
		FCoordinates coords = m_Rooms[i]->GetCoordinates();
		//Get each room's 4 vertices and add them to array
		Vertex A{ (float)coords.MinX, (float)coords.MinY};
		Vertex B{ (float)coords.MinX, (float)coords.MaxY};
		Vertex C{ (float)coords.MaxX, (float)coords.MinY};
		Vertex D{ (float)coords.MaxX, (float)coords.MaxY};
		
		roomVertices.Add(A);
		roomVertices.Add(B);
		roomVertices.Add(C);
		roomVertices.Add(D);
	}
	return roomVertices;
}

TArray<Vertex> AGenerator::GetRoomCenter()
{
	TArray<Vertex> roomCenterCoords;

	for (int32 i = 0; i < m_Rooms.Num(); i++)
	{
		//Converting vectors to vertices	
		Vertex centerCoords = m_Rooms[i]->GetCenter();

		roomCenterCoords.Add(centerCoords);
	}
	return roomCenterCoords;
}

void AGenerator::DressRooms()
{
	for (ARoom* room : m_Rooms)
	{
		room->SetFloorMaterial(m_FloorMaterial);
		room->SetWalls(*m_WallMesh, m_GridCellSize);
	}


}

void AGenerator::MakeDoors(const TArray<TArray<IntVector>>& halls, const TArray<TArray<GridNode>>& grid)
{
	//Replacing walls with doors
	for (int i = 0; i < halls.Num(); i++)
	{
		GridNode currentNode = grid[halls[i][0].X][halls[i][0].Y];
		//Find direction opposite to next hallway tile from start tile
		IntVector current{ halls[i][0] };
		IntVector next{ halls[i][1] };

		UE_LOG(LogTemp, Warning, TEXT("Current X: %d Y: %d   Next X: %d Y:%d"), current.X, current.Y, next.X, next.Y);

		FVector dir;

		//Needed cause door asset doesn't have the same pivot point as the walls(Should learn to change pivot points maybe??)
		FVector shiftDir;

		if (next.X > current.X)
		{
			dir = FVector::BackwardVector;
			shiftDir = FVector::LeftVector;
		}
		else if (next.X < current.X)
		{
			dir = FVector::ForwardVector;
			shiftDir = FVector::RightVector;
		}
		else if (next.Y > current.Y)
		{
			dir = FVector::LeftVector;
			shiftDir = FVector::ForwardVector;
		}
		else if (next.Y < current.Y)
		{
			dir = FVector::RightVector;
			shiftDir = FVector::BackwardVector;
		}

		shiftDir *= m_GridCellSize / 2;

		FVector start(currentNode.nodeCoords.MinX + m_GridCellSize / 2.f, currentNode.nodeCoords.MinY + m_GridCellSize / 2.f, 0.f);
		FVector end = start + dir * m_GridCellSize;
		FHitResult hit;
		
		DrawDebugLine(GetWorld(), start, end, FColor::Blue, true, -1.f, -2, 25.f);
		DrawDebugPoint(GetWorld(), start, 25.f, FColor::Yellow, true, -1.f, -3);

		if (GetWorld()->LineTraceSingleByChannel(hit, start, end, ECollisionChannel::ECC_Visibility, FCollisionQueryParams::DefaultQueryParam, FCollisionResponseParams::DefaultResponseParam))
		{
			if (Cast<ARoom>(hit.GetActor()))
			{
				UStaticMeshComponent* sm = Cast<UStaticMeshComponent>(hit.Component);

				if (sm->GetStaticMesh() != m_DoorMesh)
				{
					sm->SetStaticMesh(m_DoorMesh);
					sm->SetRelativeLocation(sm->GetRelativeLocation() + shiftDir);
				}
			}
		}
		

		//Find direction opposite to next hallway tile from end tile
		currentNode = grid[halls[i].Last().X][halls[i].Last().Y];
		current = { halls[i].Last()};
		IntVector previous{ halls[i].Last(1) };

		if (previous.X > current.X)
		{
			dir = FVector::BackwardVector;
			shiftDir = FVector::LeftVector;
		}
		else if (previous.X < current.X)
		{
			dir = FVector::ForwardVector;
			shiftDir = FVector::RightVector;
		}
		else if (previous.Y > current.Y)
		{
			dir = FVector::LeftVector;
			shiftDir = FVector::ForwardVector;
		}
		else if (previous.Y < current.Y)
		{
			dir = FVector::RightVector;
			shiftDir = FVector::BackwardVector;
		}

		start = FVector(currentNode.nodeCoords.MinX + m_GridCellSize / 2.f, currentNode.nodeCoords.MinY + m_GridCellSize / 2.f, 0.f);
		end = start + dir * m_GridCellSize;

		DrawDebugLine(GetWorld(), start, end, FColor::Blue, true, -1.f, -2, 25.f);
		DrawDebugPoint(GetWorld(), start, 25.f, FColor::Red, true, -1.f, -3);

		shiftDir *= m_GridCellSize / 2;

		if (GetWorld()->LineTraceSingleByChannel(hit, start, end, ECollisionChannel::ECC_Visibility, FCollisionQueryParams::DefaultQueryParam, FCollisionResponseParams::DefaultResponseParam))
		{
			if (Cast<ARoom>(hit.GetActor()) && Cast<UStaticMeshComponent>(hit.Component)->GetStaticMesh() != m_DoorMesh)
			{
				UStaticMeshComponent* sm = Cast<UStaticMeshComponent>(hit.Component);

				if (sm->GetStaticMesh() != m_DoorMesh)
				{
					sm->SetStaticMesh(m_DoorMesh);
					sm->SetRelativeLocation(sm->GetRelativeLocation() + shiftDir);
				}
			}
		}
	}
	
}

void AGenerator::MergeHallways( TArray<TArray<IntVector>>& halls)
{
	TArray<TArray<IntVector>> mergedHallways{};
	
	for (int i = 0; i < halls.Num(); i++)
	{
		bool isMerged = false;
		TArray<IntVector> mergedHall{};
		for (int j = 0; j < halls[i].Num(); j++)
		{
			for (int k = i + 1; k < halls.Num(); k++)
			{
				for (int h = 0; h < halls[k].Num(); h++)
				{
					if (halls[i][j] == halls[k][h])
					{
						mergedHall.Append(halls[i]);
						for (IntVector tile : halls[k])
						{
							mergedHall.AddUnique(tile);
						}

						mergedHallways.Add(mergedHall);

						isMerged = true;

						break;
					}
				}


			}
		}

		if (!isMerged)
		{
			mergedHallways.Add(halls[i]);
		}
	}
	halls = mergedHallways;
}

void AGenerator::MakeHallFloors(const GridNode& node, const int32 sectionCounter)
{
	UProceduralMeshComponent* mesh = Cast<UProceduralMeshComponent>(AddComponentByClass(UProceduralMeshComponent::StaticClass(), false, FTransform(), false));

	int32 minX = node.nodeCoords.MinX;
	int32 minY = node.nodeCoords.MinY;
	int32 maxX = node.nodeCoords.MaxX;
	int32 maxY = node.nodeCoords.MaxY;

	TArray<FVector> vertices{ FVector(minX, minY, 0.f),
							  FVector(maxX, minY, 0.f),
							  FVector(maxX, maxY, 0.f),
							  FVector(minX, maxY, 0.f) };

	TArray<int32> triangles{ 0, 2, 1,
							0, 3, 2 };

	TArray<FVector> normals{ FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f) };
	TArray<FVector2D> UV0{ FVector2D(0, 0), FVector2D(0, 1), FVector2D(1, 1), FVector2D(1, 0) };
	TArray<FProcMeshTangent> tangents{ FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f) };

	mesh->CreateMeshSection(sectionCounter, vertices, triangles, normals, UV0, TArray<FColor>(), tangents, true);
	mesh->SetMaterial(sectionCounter, m_HallwayMaterial);
}

void AGenerator::ApplyHallVisuals( TArray<TArray<IntVector>>& halls, const TArray<TArray<GridNode>>& grid)
{
	//We need to place doors where halls meet rooms
	MakeDoors(halls, grid);

	//We need to merge halls that are interconnected to avoid multiple rendering, and it will help with wall placement at junctions.
	MergeHallways(halls);

	for (int i = 0; i < halls.Num(); i++)
	{
		int32 sectionCounter = 0;
		for (int j = 0; j < halls[i].Num(); j++)
		{
			MakeHallFloors(grid[halls[i][j].X][halls[i][j].Y], sectionCounter);

			sectionCounter++;
		}
	}
}

void AGenerator::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.GetMemberPropertyName() == "m_RoomSizeX")
	{
		m_RoomSizeX = FMath::Clamp(m_RoomSizeX, 0, m_PlayAreaSizeX < 1 ? 1 : m_PlayAreaSizeX);
	}
	else if (PropertyChangedEvent.GetMemberPropertyName() == "m_RoomSizeY")
	{
		m_RoomSizeY = FMath::Clamp(m_RoomSizeY, 0, m_PlayAreaSizeY < 1 ? 1 : m_PlayAreaSizeY);
	}
}


