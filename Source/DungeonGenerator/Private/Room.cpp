// Fill out your copyright notice in the Description page of Project Settings.

#include "Room.h"
#include "CustomStructs.h"

// Sets default values
ARoom::ARoom()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	m_ProcMesh = CreateAbstractDefaultSubobject<UProceduralMeshComponent>("MeshComponent");
	SetRootComponent(m_ProcMesh);
	
	
}

// Called when the game starts or when spawned
void ARoom::BeginPlay()
{
	Super::BeginPlay();
	m_ProcMesh->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
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


	m_ProcMesh->CreateMeshSection(0, vertices, triangles, normals, UV0,TArray<FColor>(), tangents, true);

	
	m_ProcMesh->SetMaterial(0, material);
}

