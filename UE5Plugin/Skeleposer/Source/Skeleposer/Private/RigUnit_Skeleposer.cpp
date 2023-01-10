// Fill out your copyright notice in the Description page of Project Settings.


#include "RigUnit_Skeleposer.h"

#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Units/RigUnitContext.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(RigUnit_Skeleposer)

#define LOCTEXT_NAMESPACE "Skeleposer"

inline void DRAW_TEXT(const FString& s, const FColor &Color=FColor::Yellow)
{
	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 2.0f, Color, *s);
	UE_LOG(LogTemp, Warning, TEXT("%s"), *s);
}

inline FMatrix ConvertMatrixFromMayaToUE(const FMatrix& Mat)
{
	FMatrix Out;
	Out.M[0][0] = Mat.M[0][0];
	Out.M[0][1] = Mat.M[0][2];
	Out.M[0][2] = Mat.M[0][1];
	Out.M[0][3] = Mat.M[0][3];

	Out.M[1][0] = Mat.M[2][0];
	Out.M[1][1] = Mat.M[2][2];
	Out.M[1][2] = Mat.M[2][1];
	Out.M[1][3] = Mat.M[2][3];

	Out.M[2][0] = Mat.M[1][0];
	Out.M[2][1] = Mat.M[1][2];
	Out.M[2][2] = Mat.M[1][1];
	Out.M[2][3] = Mat.M[1][3];

	Out.M[3][0] = Mat.M[3][0];
	Out.M[3][1] = Mat.M[3][2];
	Out.M[3][2] = Mat.M[3][1];
	Out.M[3][3] = Mat.M[3][3];
	return Out;
}

FRigUnit_Skeleposer_Execute()
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_RIGUNIT()
	URigHierarchy* Hierarchy = Context.Hierarchy;

	if (!Hierarchy)
		return;

	if (Context.State == EControlRigState::Init)
	{
		WorkData.ReadJsonFile(FilePath, PoseNameList);
		return;
	}
	else if (Context.State == EControlRigState::Update)
	{
		TMap<FString, float> PoseWeights;
		for (const auto &UserPose : UserPoses)
			PoseWeights.Add(UserPose.PoseName, UserPose.PoseWeight);

		for (auto& BoneItem : WorkData.BonePoses) // go through bones
		{
			const FRigElementKey BoneKey(FName(*BoneItem.Key), ERigElementType::Bone);

			if (Hierarchy->Find(BoneKey))
			{
				FTransform Local = Hierarchy->GetLocalTransform(BoneKey, false);

				FVector Translation = Local.GetTranslation();
				FQuat Rotation = Local.GetRotation();
				FVector Scale = Local.GetScale3D();

				for (const auto& Pose : BoneItem.Value)
				{
					float PoseWeight = PoseWeights.FindOrAdd(Pose.Name, 0);
					
					if (!Pose.Corrects.IsEmpty()) // if corrects found
					{
						PoseWeight = FLT_MAX; // find lowest weight
						for (const FString& CorrectName : Pose.Corrects)
						{
							const float w = PoseWeights.FindOrAdd(CorrectName, 0);
							if (w < PoseWeight)
								PoseWeight = w;
						}	

						PoseWeights[Pose.Name] = PoseWeight;
					}

					if (!Pose.Inbetween.Value.IsEmpty())
					{
						const float SourcePoseWeight = PoseWeights.FindOrAdd(Pose.Inbetween.Key, 0);
						PoseWeight = Pose.Inbetween.Value.EvaluateFromX(SourcePoseWeight).Y;
						PoseWeights[Pose.Name] = PoseWeight;
					}

					if (PoseWeight > 1e-3)
					{	
						if (Pose.BlendMode == FPoseBlendMode::ADDITIVE)
						{
							Translation += Pose.Delta.GetTranslation() * PoseWeight;
							Rotation = FQuat::Slerp(Rotation, Rotation * Pose.Delta.GetRotation(), PoseWeight);
							Scale = FMath::Lerp(Scale, Pose.Delta.GetScale3D() * Scale, PoseWeight);
						}

						else if (Pose.BlendMode == FPoseBlendMode::REPLACE)
						{
							Translation = FMath::Lerp(Translation, Pose.Delta.GetTranslation(), PoseWeight);
							Rotation = FQuat::Slerp(Rotation, Pose.Delta.GetRotation(), PoseWeight);
							Scale = FMath::Lerp(Scale, Pose.Delta.GetScale3D(), PoseWeight);
						}
					}
				}

				Local.SetTranslation(Translation);
				Local.SetRotation(Rotation);
				Local.SetScale3D(Scale);

				Hierarchy->SetLocalTransform(BoneKey, Local);
			}
		}

		// activate poses' morph targets
		if (bUseMorphTargets)
		{
			for (const auto& PoseWeightItem : PoseWeights)
			{
				FRigElementKey CurveKey(FName(PoseWeightItem.Key), ERigElementType::Curve);
				
				if (Hierarchy->Find(CurveKey))
					Hierarchy->SetCurveValue(CurveKey, PoseWeightItem.Value);
			}
		}
	}
}

void FRigUnit_Skeleposer_WorkData::ReadJsonFile(const FString& FilePath, TArray<FString>& PoseNameList)
{
	if (FilePath == FilePathCache) 
		return;
	FilePathCache = FilePath; // cache file path

	DRAW_TEXT("Read json file");

	const FString FullFilePath = FPaths::ProjectContentDir() + "/" + FilePath;

	if (!FPaths::FileExists(FullFilePath))
	{
		DRAW_TEXT("Cannot find '" + FullFilePath + "'. Path must be relative to the project's content directory");
		return;
	}

	BonePoses.Reset();
	PoseNameList.Reset();

	const FMatrix RootMatrix = FQuat(-0.707106769, 0, 0, 0.707106709).ToMatrix(); // Q = FQuat::MakeFromEuler(FVector(90, 0, 0));
	const FMatrix RootMatrixInverse = FQuat(0.707106769, 0, 0, 0.707106709).ToMatrix();  // Q.Inverse

	FString JsonContent;
	FFileHelper::LoadFileToString(JsonContent, *FullFilePath);

	TSharedPtr<FJsonObject> JsonObject;
	auto Reader = TJsonReaderFactory<>::Create(JsonContent);
	if (FJsonSerializer::Deserialize(Reader, JsonObject))
	{
		// get joints and indices used in poseDeltaMatrices
		const TSharedPtr<FJsonObject> JointsItem = JsonObject->Values["joints"]->AsObject();
		if (!JointsItem)
			return;

		TMap<int32, FString> BoneNames;
		for (const auto& Item : JointsItem->Values)
		{
			const int32 Idx = FCString::Atoi(*Item.Key);

			const FString BoneName = Item.Value->AsString();
			BoneNames.Add(Idx, BoneName);
			BonePoses.Add(BoneName);
		}

		// get directories
		FDirectoryList DirectoryList;

		const TSharedPtr<FJsonObject> DirectoriesItem = JsonObject->Values["directories"]->AsObject();
		if (!DirectoriesItem)
			return;

		for (const auto& Item : DirectoriesItem->Values)
		{
			const int32 Idx = FCString::Atoi(*Item.Key);
			const auto& DirectoryValues = Item.Value->AsObject()->Values;

			FDirectory& Directory = DirectoryList[Idx];

			Directory.Weight = 1;
			Directory.ParentIndex = DirectoryValues["directoryParentIndex"]->AsNumber();;

			for (const auto& ChildIndex : DirectoryValues["directoryChildrenIndices"]->AsArray())
				Directory.ChildrenIndices.Add(ChildIndex->AsNumber());
		}

		// get poses
		const TSharedPtr<FJsonObject> PosesItem = JsonObject->Values["poses"]->AsObject();
		if (!PosesItem)
			return;

		TMap<int32, FPose> PoseList; // pose per index
		for (const auto& Item : PosesItem->Values)
		{
			const int32 PoseIdx = FCString::Atoi(*Item.Key);
			const auto& PoseValues = Item.Value->AsObject()->Values;

			FPose& Pose = PoseList.FindOrAdd(PoseIdx);

			Pose.Name = PoseValues["poseName"]->AsString();
			Pose.BlendMode = (FPoseBlendMode)PoseValues["poseBlendMode"]->AsNumber();

			PoseNameList.Add(Pose.Name);

			if (PoseValues.Contains("corrects"))
			{
				for (const auto& CorrectItem : PoseValues["corrects"]->AsArray())
					Pose.Corrects.Add(CorrectItem->AsNumber());
			}

			if (PoseValues.Contains("inbetween")) // idx, [(x,y), (x,y)]
			{
				const TArray<TSharedPtr<FJsonValue>> &InbetweenArray = PoseValues["inbetween"]->AsArray();

				Pose.Inbetween.Key = InbetweenArray[0]->AsNumber(); // source pose index

				for (const auto& PointItem : InbetweenArray[1]->AsArray())
				{
					const TArray<TSharedPtr<FJsonValue>>& PointArray = PointItem->AsArray();
					const double X = PointArray[0]->AsNumber();
					const double Y = PointArray[1]->AsNumber();
					Pose.Inbetween.Value.Points.Add(FVector2D(X,Y));
				}
			}

			for (const auto& DeltaMatricesItem : PoseValues["poseDeltaMatrices"]->AsObject()->Values)
			{
				const int32 BoneIdx = FCString::Atoi(*DeltaMatricesItem.Key);

				FMatrix Mat;
				auto& DeltaMatrixValue = DeltaMatricesItem.Value->AsArray();
				for (int i = 0; i < 4; i++)
					for (int j = 0; j < 4; j++)
						Mat.M[i][j] = DeltaMatrixValue[i * 4 + j]->AsNumber();

				// convert axes from Maya to UE
				FTransform PoseDelta(RootMatrix * ConvertMatrixFromMayaToUE(Mat) * RootMatrixInverse);

				// revert scale axes
				FMatrix NoScaleMatrix = PoseDelta.ToMatrixNoScale();
				FVector Scale = PoseDelta.GetScale3D();
				PoseDelta.SetFromMatrix(NoScaleMatrix);
				PoseDelta.SetScale3D(FVector(Scale.X, Scale.Z, Scale.Y));

				Pose.Deltas.Add(BoneIdx, PoseDelta);
			}
		}

		// get all poses indices in a correct order
		TArray<int32> PoseIndices;
		DirectoryList.GetPosesOrder(0, PoseIndices);

		// get pose per bone list, like bone1:[pose1, pose2, pose3], bone2:[pose1, pose4]
		for (int32 PoseIdx : PoseIndices)
		{
			const FPose& Pose = PoseList[PoseIdx];

			for (const auto& DeltaItem : Pose.Deltas)
			{
				FBonePose BonePose;
				BonePose.Name = Pose.Name;
				BonePose.BlendMode = Pose.BlendMode;
				BonePose.Delta = DeltaItem.Value;

				for (int32 CorrectIdx : Pose.Corrects)
					BonePose.Corrects.Add(PoseList[CorrectIdx].Name);

				if (!Pose.Inbetween.Value.IsEmpty())
				{
					BonePose.Inbetween.Key = PoseList[Pose.Inbetween.Key].Name;
					BonePose.Inbetween.Value = Pose.Inbetween.Value;
				}

				const FString& BoneName = BoneNames[DeltaItem.Key];
				BonePoses[BoneName].Add(BonePose);
			}
		}
	}
}

FRigUnit_Skeleposer::FRigUnit_Skeleposer()
{
	bUseMorphTargets = false;
}

FRigVMStructUpgradeInfo FRigUnit_Skeleposer::GetUpgradeInfo() const
{
	return FRigVMStructUpgradeInfo();
}

#undef LOCTEXT_NAMESPACE