// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CustomStructs.h"
#include "Room.h"
#include "DelaunayTriangulation.h"
#include "Generator.generated.h"

UCLASS()
class DUNGEONGENERATOR_API AGenerator : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGenerator();
	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	

	//PlayArea values must be dividable by gridcellsize
	UPROPERTY(EditAnywhere, Category= GameWorld, meta= (ClampMin = "0"))
	int32 m_PlayAreaSizeX;

	UPROPERTY(EditAnywhere, Category= GameWorld, meta = (ClampMin = "0"))
	int32 m_PlayAreaSizeY;

	UPROPERTY(EditAnywhere, Category= GameWorld, meta= (ClampMin = "100"))
	int32 m_GridCellSize;

	UPROPERTY(EditAnywhere, Category = "RoomParams", meta = (ClampMin = "0"))
	int32 m_NbRooms;

	//This parameter determines how likely loops will be generated in the dungeon
	UPROPERTY(EditAnywhere, Category = GameWorld, meta = (ClampMin = "0"))
	float m_DungeonCycleChance;

	UPROPERTY(EditAnywhere, Category = "RoomParams")
	int32 m_RoomSizeX;

	UPROPERTY(EditAnywhere, Category = "RoomParams")
	int32 m_RoomSizeY;

	UPROPERTY(EditAnywhere, Category= "RoomParams", meta = (ClampMin = "0"))
	int32 m_MinSpacing;

	UPROPERTY(EditAnywhere, Category= "RoomMaterials")
	UMaterialInterface* m_FloorMaterial;

	UPROPERTY(EditAnywhere, Category= "HallMaterials")
	UMaterialInterface* m_HallwayMaterial;

	UPROPERTY(EditAnywhere, Category= "RoomParams")
	UStaticMesh* m_WallMesh;

	UPROPERTY(EditAnywhere, Category= "RoomParams")
	UStaticMesh* m_DoorMesh;
private:
	enum class EDirection
	{
		NORTH,
		WEST,
		SOUTH,
		EAST
	};

	FCoordinates m_PlayAreaCoords;
	TArray<ARoom*> m_Rooms;
	TArray<UProceduralMeshComponent*> m_HallMeshes;
	
	

	void DefinePlayArea();

	void AddRoom(uint32&);
	IntVector DefineRoomSize();
	IntVector FindRoomExtent(const IntVector);
	FVector SelectRoomLocation(const IntVector, FCoordinates&);
	TArray<Vertex> GetRoomVertices();
	TArray<Vertex> GetRoomCenter();
	void DressRooms();
	void MakeDoors(const TArray<TArray<IntVector>>&, const TArray<TArray<GridNode>>&);
	void MergeHallways(TArray<TArray<IntVector>>&);
	void ApplyHallVisuals(TArray<TArray<IntVector>>&, const TArray<TArray<GridNode>>&);

	TArray<Edge> GetMST(const TArray<Triangle>&);
	int32 SelectMinVertex(const std::vector<int>&, const std::vector<bool>&);

	TArray<TArray<GridNode>> MakeGrid();
	void SetGridNodeWeights(TArray<TArray<GridNode>>&);
	bool IsNodeRoomNeighbor(const TArray<TArray<GridNode>>&, const IntVector&);
	TArray<TArray<IntVector>> GenerateHallways(TArray<TArray<GridNode>>&, const TArray<Edge>&);
	EDirection GetPathDirection(const TArray<TArray<GridNode>>&, const IntVector&, const IntVector&);
	IntVector GetPathStartingNode(const IntVector&, const EDirection&, const TArray<TArray<GridNode>>&);
	IntVector GetPathEndNode(const IntVector&, const EDirection&, const TArray<TArray<GridNode>>&);
	TArray<IntVector> GetPath(const IntVector&, const IntVector&, TArray<TArray<GridNode>>&, const EDirection&);
	TArray<IntVector> ConstructPath(const std::vector<std::vector<IntVector>>&, IntVector&, const IntVector&);
	void DressHallways(const TArray<TArray<IntVector>>&, const TArray<TArray<GridNode>>&);

	int32 GetNodeWithSmallestWeight(const TArray<IntVector>&, const std::vector<std::vector<int>>&);
	int32 GetHCost(const IntVector&, const IntVector&);

	bool CheckOverlap(const TArray<ARoom*>&, const FCoordinates&, const uint32);
	bool CheckCellIsRoom(const ARoom&, const GridNode&);

	void DrawSquare(const FVector&, const FVector&);
	void DrawTriangulation(const TArray<Triangle>&, DelaunayTriangulation&);
	void DrawMST(const TArray<Edge>&);
	void DrawGrid(const TArray<TArray<GridNode>>&);





#if WITH_EDITOR

	virtual void PostEditChangeProperty(FPropertyChangedEvent&) override;

#endif

};
