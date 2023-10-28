// Fill out your copyright notice in the Description page of Project Settings.

#include "Room.h"
#include "CustomStructs.h"

// Sets default values
ARoom::ARoom()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;	
}

// Called when the game starts or when spawned
void ARoom::BeginPlay()
{
	Super::BeginPlay();

	m_ProcMesh = Cast<UProceduralMeshComponent>(AddComponentByClass(UProceduralMeshComponent::StaticClass(), false, FTransform(), false));
}

// Called every frame
void ARoom::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ARoom::SetRoomCoordinates(const FCoordinates& cornerCoordinates, const FVector& centerCoordinates, const IntVector& roomSize)
{
	m_CornerCoordinates = cornerCoordinates;
	m_CenterCoordinates = centerCoordinates;
	m_RoomSize = roomSize;
}

void ARoom::SetFloorMaterial(UMaterialInterface* material)
{
	FVector v1(m_CornerCoordinates.MinX, m_CornerCoordinates.MinY, 0.f);
	FVector v2(m_CornerCoordinates.MaxX, m_CornerCoordinates.MinY, 0.f);
	FVector v3(m_CornerCoordinates.MaxX, m_CornerCoordinates.MaxY, 0.f);
	FVector v4(m_CornerCoordinates.MinX, m_CornerCoordinates.MaxY, 0.f);

	TArray<FVector> vertices{ v1, v2, v3, v4 };
	TArray<int32> triangles{ 0, 2, 1,
							0, 3, 2 };
	TArray<FVector> normals{ FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f), FVector(0.f, 0.f, 1.0f) };
	TArray<FVector2D> UV0{ FVector2D(0, 0), FVector2D(0, 10), FVector2D(10, 10), FVector2D(10, 0) };
	TArray<FProcMeshTangent> tangents{ FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f), FProcMeshTangent(0.f, 1.f, 0.f) };

	m_ProcMesh->CreateMeshSection(0, vertices, triangles, normals, UV0, TArray<FColor>(), tangents, true);
	m_ProcMesh->SetMaterial(0, material);
	m_ProcMesh->RegisterComponent();
}

void ARoom::SetWalls(UStaticMesh& statmesh, const int32 cellSize)
{
	//Size of the room in cell amount
	int32 NbCellX = m_RoomSize.X / cellSize;
	int32 NbCellY = m_RoomSize.Y / cellSize;
	int32 roomCoordX = m_CornerCoordinates.MinX;
	int32 roomCoordY = m_CornerCoordinates.MinY;

	for (int i = 0; i <= NbCellX; i++)
	{
		for (int j = 0; j <= NbCellY; j++)
		{
			if ((i == 0 || i == NbCellX) || (j == 0 || j == NbCellY))
			{
				FVector wallCoord(roomCoordX + i * cellSize, roomCoordY + j * cellSize, 0.f);

				UStaticMeshComponent* mesh = Cast<UStaticMeshComponent>(AddComponentByClass(UStaticMeshComponent::StaticClass(), true, FTransform(), true));

				mesh->RegisterComponent();

				if (mesh != nullptr)
				{
					mesh->SetStaticMesh(&statmesh);
				}

				mesh->SetRelativeLocation(wallCoord);

				if (i > 0 && i < NbCellX)
				{
					if (j == 0)
					{
						mesh->SetRelativeRotation(FRotator(0.f, 90.f, 0.f));
					}
					else if (j == NbCellY)
					{
						mesh->SetRelativeRotation(FRotator(0.f, -90.f, 0.f));
					}
				}

				if (i == 0 && j == NbCellY)
				{
					mesh->SetRelativeRotation(FRotator(0.f, -90.f, 0.f));
				}

				if (i == NbCellX)
				{
					if (j == 0)
					{
						mesh->SetRelativeRotation(FRotator(0.f, 90.f, 0.f));
					}
					else
					{
						mesh->SetRelativeRotation(FRotator(0.f, 180.f, 0.f));
					}
				}

				m_RoomMeshes.Add(mesh);
			}
		}
	}
}

void ARoom::SwapWallToDoor(const int32 wallIndex, UStaticMesh& statmesh)
{
	m_RoomMeshes[wallIndex]->SetStaticMesh(&statmesh);
}



