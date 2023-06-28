// Created by Alexander Zagoruyko. Published in 2023

#include "RigUnit_Skeleposer.h"

#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Units/RigUnitContext.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(RigUnit_Skeleposer)

#define LOCTEXT_NAMESPACE "Skeleposer"

inline void DisplayError(const FString& s, const FColor &Color=FColor::Red)
{
#if WITH_EDITOR	
	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.0, Color, *s);
	UE_LOG(LogTemp, Warning, TEXT("%s"), *s);
#endif
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

FString StripNamespace(const FString& Name)
{
	// Find the last occurrence of ':'
	int32 LastColonIndex;
	if (Name.FindLastChar(':', LastColonIndex))
	{
		// Extract the substring starting from the character after the last ':'
		return Name.Mid(LastColonIndex + 1);
	}
	return Name;
}

void GetPosesOrder(const TMap<int32, FDirectory>& Directories, int32 DirectoryIdx, TArray<int32>& PoseIndices)
{
	const FDirectory* Found = Directories.Find(DirectoryIdx);
	if (Found)
	{
		for (auto& Idx : Found->ChildrenIndices)
		{
			if (Idx >= 0)
				PoseIndices.Add(Idx);
			else
				GetPosesOrder(Directories, -Idx, PoseIndices);
		}
	}
}

FRigUnit_Skeleposer_Execute()
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_RIGUNIT()
	URigHierarchy* Hierarchy = Context.Hierarchy;

	if (!Hierarchy)
		return;

	if (Context.State == EControlRigState::Init)
	{
		WorkData.Reset();
		return;
	}

	else if (Context.State == EControlRigState::Update)
	{
		// Read JSON file only once unless the path is changed
		if (FilePath != WorkData.FilePathCache) 
		{
			WorkData.FilePathCache = FilePath; // cache file path

			const FString FullFilePath = FPaths::Combine(FPaths::ProjectContentDir(), FilePath);
			if (!FPaths::FileExists(FullFilePath))
			{
				DisplayError("Cannot find '" + FilePath + "'. FilePath must be relative to the project's content directory", FColor::Red);
				return;
			}

			WorkData.ReadJsonFile(FullFilePath, PoseNameList);
		}

		if (WorkData.BonePoses.IsEmpty())
		{
			UE_CONTROLRIG_RIGUNIT_REPORT_WARNING(TEXT("No poses loaded from the file"));
			return;
		}

		TMap<FString, float> PosesWeights; // user poses' weights and weights that are calculated during the computation
		for (const auto &UserPose : UserPoses)
			PosesWeights.Add(UserPose.PoseName, UserPose.PoseWeight);

		for (auto& [BoneKey, BonePoses] : WorkData.BonePoses) // go through bones
		{
			if (!Hierarchy->Find(BoneKey))
				continue;

			FTransform Local = Hierarchy->GetLocalTransform(BoneKey, false);

			FVector Translation = Local.GetTranslation();
			FQuat Rotation = Local.GetRotation();
			FVector Scale = Local.GetScale3D();

			for (const auto& BonePose : BonePoses)
			{
				float PoseWeight = PosesWeights.FindOrAdd(BonePose.PoseName, 0);
					
				if (!BonePose.Corrects.IsEmpty()) // if corrects found
				{
					PoseWeight = FLT_MAX; // find lowest weight
					for (const FString& CorrectName : BonePose.Corrects)
					{
						const float w = PosesWeights.FindOrAdd(CorrectName, 0);
						if (w < PoseWeight)
							PoseWeight = w;
					}
				}

				if (!BonePose.Inbetween.Value.IsEmpty())
				{
					const float SourcePoseWeight = PosesWeights.FindOrAdd(BonePose.Inbetween.Key, 0);
					PoseWeight = BonePose.Inbetween.Value.EvaluateFromX(SourcePoseWeight).Y;
				}

				PosesWeights[BonePose.PoseName] = PoseWeight; // save weight for the pose

				if (PoseWeight > 1e-3)
				{	
					if (BonePose.BlendMode == FPoseBlendMode::ADDITIVE)
					{
						Translation += BonePose.Delta.GetTranslation() * PoseWeight;
						Rotation = FQuat::Slerp(Rotation, Rotation * BonePose.Delta.GetRotation(), PoseWeight);
						Scale = FMath::Lerp(Scale, BonePose.Delta.GetScale3D() * Scale, PoseWeight);
					}

					else if (BonePose.BlendMode == FPoseBlendMode::REPLACE)
					{
						Translation = FMath::Lerp(Translation, BonePose.Delta.GetTranslation(), PoseWeight);
						Rotation = FQuat::Slerp(Rotation, BonePose.Delta.GetRotation(), PoseWeight);
						Scale = FMath::Lerp(Scale, BonePose.Delta.GetScale3D(), PoseWeight);
					}
				}
			}

			Local.SetTranslation(Translation);
			Local.SetRotation(Rotation);
			Local.SetScale3D(Scale);

			Hierarchy->SetLocalTransform(BoneKey, Local);
		}

		// activate poses' morph targets
		if (bUseMorphTargets)
		{
			for (const auto& [PoseName, PoseWeight] : PosesWeights)
			{
				FRigElementKey CurveKey(FName(PoseName), ERigElementType::Curve);
				
				if (Hierarchy->Find(CurveKey))
					Hierarchy->SetCurveValue(CurveKey, PoseWeight);
			}
		}
	}
}

void FRigUnit_Skeleposer_WorkData::ReadJsonFile(const FString& FilePath, TArray<FString>& PoseNameList)
{
	BonePoses.Reset();
	PoseNameList.Reset();

	const FMatrix RootMatrix = FQuat(-0.707106769, 0, 0, 0.707106709).ToMatrix(); // Q = FQuat::MakeFromEuler(FVector(90, 0, 0));
	const FMatrix RootMatrixInverse = FQuat(0.707106769, 0, 0, 0.707106709).ToMatrix();  // Q.Inverse

	FString JsonContent;
	FFileHelper::LoadFileToString(JsonContent, *FilePath);

	TSharedPtr<FJsonObject> JsonObject;
	auto Reader = TJsonReaderFactory<>::Create(JsonContent);
	if (FJsonSerializer::Deserialize(Reader, JsonObject))
	{
		// get joints and indices used in poseDeltaMatrices
		const TSharedPtr<FJsonObject> JointsObject = JsonObject->Values["joints"]->AsObject();

		TMap<int32, FRigElementKey> Bones; // per index		
		for (const auto& [JointIdxStr, JointValue] : JointsObject->Values)
		{
			const int32 Idx = FCString::Atoi(*JointIdxStr);

			const FString BoneName = StripNamespace(JointValue->AsString()); // UE remove namespace for bones automatically

			const FRigElementKey BoneKey(FName(*BoneName), ERigElementType::Bone);
			Bones.Add(Idx, BoneKey);
			BonePoses.Add(BoneKey);
		}

		// get directories
		TMap<int32, FDirectory> Directories;

		const TSharedPtr<FJsonObject> DirectoriesObject = JsonObject->Values["directories"]->AsObject();

		for (const auto& [DirIdxStr, DirObject] : DirectoriesObject->Values)
		{
			const int32 Idx = FCString::Atoi(*DirIdxStr);
			const auto& DirValue = DirObject->AsObject()->Values;

			FDirectory& Directory = Directories.Add(Idx);
			Directory.ParentIndex = DirValue["directoryParentIndex"]->AsNumber();

			for (const auto& ChildIndex : DirValue["directoryChildrenIndices"]->AsArray())
				Directory.ChildrenIndices.Add(ChildIndex->AsNumber());
		}

		// get poses
		const TSharedPtr<FJsonObject> PosesObject = JsonObject->Values["poses"]->AsObject();

		TMap<int32, FPose> Poses; // pose per index
		for (const auto& [PoseName, PoseObject] : PosesObject->Values)
		{
			const int32 PoseIdx = FCString::Atoi(*PoseName);
			const auto& PoseValue = PoseObject->AsObject()->Values;

			FPose& Pose = Poses.FindOrAdd(PoseIdx);

			Pose.Name = PoseValue["poseName"]->AsString();
			Pose.BlendMode = (FPoseBlendMode)PoseValue["poseBlendMode"]->AsNumber();

			PoseNameList.Add(Pose.Name);

			if (PoseValue.Contains("corrects"))
			{
				for (const auto& CorrectValue : PoseValue["corrects"]->AsArray())
					Pose.Corrects.Add(CorrectValue->AsNumber());
			}

			if (PoseValue.Contains("inbetween")) // idx, [(x,y), (x,y)]
			{
				const TArray<TSharedPtr<FJsonValue>> &InbetweenArray = PoseValue["inbetween"]->AsArray();

				Pose.Inbetween.Key = InbetweenArray[0]->AsNumber(); // source pose index

				for (const auto& PointItem : InbetweenArray[1]->AsArray())
				{
					const TArray<TSharedPtr<FJsonValue>>& PointArray = PointItem->AsArray();
					const double X = PointArray[0]->AsNumber();
					const double Y = PointArray[1]->AsNumber();
					Pose.Inbetween.Value.Points.Add(FVector2D(X,Y));
				}

				// pin both ends
				FVector2D FirstPoint = Pose.Inbetween.Value.Points[0];
				FVector2D LastPoint = Pose.Inbetween.Value.Points.Top();
				FirstPoint.X = 0;
				LastPoint.X = 1;

				Pose.Inbetween.Value.Points.Insert(FirstPoint, 0);
				Pose.Inbetween.Value.Points.Add(LastPoint);
			}

			for (const auto& [DeltaMatrixIdxStr, DeltaMatrixArrayValue] : PoseValue["poseDeltaMatrices"]->AsObject()->Values)
			{
				const int32 BoneIdx = FCString::Atoi(*DeltaMatrixIdxStr);

				FMatrix Mat;
				auto& DeltaMatrixValue = DeltaMatrixArrayValue->AsArray();
				for (int32 i = 0; i < 4; i++)
					for (int32 j = 0; j < 4; j++)
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
		GetPosesOrder(Directories, 0, PoseIndices);

		// get pose per bone list, like bone1:[pose1, pose2, pose3], bone2:[pose1, pose4]
		for (int32 PoseIdx : PoseIndices)
		{
			const FPose& Pose = Poses[PoseIdx];

			for (const auto& [DeltaIndex, DeltaTransform] : Pose.Deltas)
			{
				FBonePose BonePose;
				BonePose.PoseName = Pose.Name;
				BonePose.BlendMode = Pose.BlendMode;
				BonePose.Delta = DeltaTransform;

				for (int32 CorrectIdx : Pose.Corrects)
					BonePose.Corrects.Add(Poses[CorrectIdx].Name);

				if (!Pose.Inbetween.Value.IsEmpty())
				{
					BonePose.Inbetween.Key = Poses[Pose.Inbetween.Key].Name;
					BonePose.Inbetween.Value = Pose.Inbetween.Value;
				}

				const FRigElementKey *BoneKey = Bones.Find(DeltaIndex);
				if (BoneKey)
					BonePoses[*BoneKey].Add(BonePose);
				else
					DisplayError("Cannot find bone at index "+FString::FromInt(DeltaIndex), FColor::Red);
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