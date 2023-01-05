// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Units/RigUnit.h"
#include "RigUnit_Skeleposer.generated.h"

/** Each Directory contains other directories and poses as indices*/
struct FDirectory
{
	/** Not used currently */
	float Weight;

	/** 0 by default */
	int ParentIndex;

	/** Positive index is Pose, Negative index is Directory */
	TArray<int> ChildrenIndices;
};

/** List of directories by index. Indices got from Maya */
class FDirectoryList
{
public:
	FDirectoryList() {}

	/** Directory accessor */
	FDirectory& operator[](int Idx) { return _Items.FindOrAdd(Idx); }

	/** Get poses in a correct order. 
	* @param DirectoryIdx Index of a directory
	* @param PoseIndices Output ordered indices	
	*/
	void GetPosesOrder(int DirectoryIdx, TArray<int32>& PoseIndices) const
	{
		const FDirectory *Found = _Items.Find(DirectoryIdx);
		if (Found)
		{
			for (auto& Idx : Found->ChildrenIndices)
			{
				if (Idx >= 0)
					PoseIndices.Add(Idx);
				else
					GetPosesOrder(-Idx, PoseIndices);
			}
		}
	}

private:
	TMap<int32, FDirectory> _Items; /** Directory per index*/
};

typedef enum
{
	ADDITIVE, /** Add the pose to the current transformations */
	REPLACE /** Replace current transformations by the pose */
} FPoseBlendMode;

/** Pose got from Skeleposer in Maya */
struct FPose
{
	/** Pose name */
	FString Name;

	/** Blending type, ADDITIVE or REPLACE */
	FPoseBlendMode BlendMode;

	/** Matrices per bone index */
	TMap<int32, FTransform> Deltas;

	/** Indices of the correct poses */
	TArray<int32> Corrects;
};

/** Poses per bone. Used for perfomance reasons */
struct FBonePose
{
	/** Pose name */
	FString Name;

	/** Blending type, ADDITIVE or REPLACE */
	FPoseBlendMode BlendMode;

	/** Pose delta matrix */
	FTransform Delta;

	/** Correct poses names */
	TArray<FString> Corrects;
};

USTRUCT()
struct FRigUnit_Skeleposer_WorkData
{
	GENERATED_BODY()

	/** Poses be bone name */
	TMap<FString, TArray<FBonePose>> BonePoses;
	FString FilePathCache;

	void ReadJsonFile(const FString &FilePath, TArray<FString> &PoseNameList);
};

/** Poses set in Unreal by a user */
USTRUCT()
struct FUserPose
{
	GENERATED_BODY()

	UPROPERTY(meta = (Input))
	FString PoseName;

	UPROPERTY(meta = (Input, ClampMin = 0.f, ClampMax = 1.f))
	float PoseWeight;
};

/**
 * 
 */
USTRUCT(meta = (DisplayName = "Skeleposer", Category = "Others", NodeColor = "1.0, 0.0, 1.0"))
struct SKELEPOSER_API FRigUnit_Skeleposer : public FRigUnitMutable
{
	GENERATED_BODY()

	FRigUnit_Skeleposer();

	RIGVM_METHOD()
	virtual void Execute(const FRigUnitContext& Context) override;

	RIGVM_METHOD()
	virtual FRigVMStructUpgradeInfo GetUpgradeInfo() const override;	

	/** Relative to Contents directory json file path */
	UPROPERTY(meta = (Input, DetailsOnly))
	FString FilePath;

	UPROPERTY(meta = (Input))
	bool bUseMorphTargets;

	UPROPERTY(meta = (Input, DisplayName="Poses", TitleProperty="PoseName"))
	TArray<FUserPose> UserPoses; // TMap is better here, but not supported by ControlRig

	// List of all poses, read only
	UPROPERTY(meta = (Output, DetailsOnly))
	TArray<FString> PoseNameList;

	// Cache
	UPROPERTY(Transient)
	FRigUnit_Skeleposer_WorkData WorkData;
};
