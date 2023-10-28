// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CustomStructs.h"
#include "ProceduralMeshComponent.h"
#include "Room.generated.h"

UCLASS()
class DUNGEONGENERATOR_API ARoom : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARoom();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void SetRoomCoordinates(const FCoordinates&, const FVector&, const IntVector&);
	void SetFloorMaterial(UMaterialInterface*);
	void SetWalls(UStaticMesh&, const int32);
	void SwapWallToDoor(const int32, UStaticMesh&);

	FORCEINLINE const FCoordinates& GetCoordinates() const { return m_CornerCoordinates; }
	FORCEINLINE const FVector& GetCenter() const { return m_CenterCoordinates; }
	FORCEINLINE const IntVector& GetRoomSize() const { return m_RoomSize; }
	FORCEINLINE const TArray<UStaticMeshComponent*>& GetWalls() const { return m_RoomMeshes; }
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	FCoordinates m_CornerCoordinates;
	FVector m_CenterCoordinates;
	IntVector m_RoomSize;
	UProceduralMeshComponent* m_ProcMesh;
	TArray<UStaticMeshComponent*> m_RoomMeshes;
};
