// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class Skeleposer : ModuleRules
{
    public Skeleposer(ReadOnlyTargetRules Target) : base(Target)
    {
        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                // ... add other public dependencies that you statically link with here ...
                "AnimGraphRuntime", "ControlRig", "RigVM", "Json", "JsonUtilities"
            }
            );


        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject"
                // ... add private dependencies that you statically link with here ...
            }
            );

        if (Target.bBuildEditor)
        {
            PrivateDependencyModuleNames.AddRange(
                new string[]
                    {
                        "Engine"
                    }
                );
         }        
    }

}
