#include <algorithm>

#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MVector.h> 
#include <maya/MMatrix.h> 
#include <maya/MAngle.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MEvaluationNode.h>

#include <maya/MFnIntArrayData.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>

#include "skeleposer.h"
#include "utils.hpp"

using namespace std;

MTypeId Skeleposer::typeId(1274440);

MObject Skeleposer::attr_joints;
MObject Skeleposer::attr_baseMatrices;
MObject Skeleposer::attr_jointOrientX;
MObject Skeleposer::attr_jointOrientY;
MObject Skeleposer::attr_jointOrientZ;
MObject Skeleposer::attr_jointOrients;
MObject Skeleposer::attr_poses;
MObject Skeleposer::attr_poseName;
MObject Skeleposer::attr_poseWeight;
MObject Skeleposer::attr_poseEnabled;
MObject Skeleposer::attr_poseBlendMode;
MObject Skeleposer::attr_poseDirectoryIndex;
MObject Skeleposer::attr_poseDeltaMatrices;
MObject Skeleposer::attr_directories;
MObject Skeleposer::attr_directoryName;
MObject Skeleposer::attr_directoryWeight;
MObject Skeleposer::attr_directoryParentIndex;
MObject Skeleposer::attr_directoryChildrenIndices;
MObject Skeleposer::attr_outputTranslates;
MObject Skeleposer::attr_outputRotateX;
MObject Skeleposer::attr_outputRotateY;
MObject Skeleposer::attr_outputRotateZ;
MObject Skeleposer::attr_outputRotates;
MObject Skeleposer::attr_outputScales;

MStatus Skeleposer::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    if (plug != attr_outputTranslates && plug != attr_outputRotates && plug != attr_outputScales)
        return MS::kUnknownParameter;
    
    MArrayDataHandle posesHandle = dataBlock.outputArrayValue(attr_poses);
    const int N_POSES = posesHandle.elementCount();

    MArrayDataHandle jointsHandle = dataBlock.outputArrayValue(attr_joints);
    MArrayDataHandle baseMatricesHandle = dataBlock.outputArrayValue(attr_baseMatrices);
    MArrayDataHandle jointOrientsHandle = dataBlock.outputArrayValue(attr_jointOrients);

    map<int, Joint> joints;
    for (int i = 0; i < jointsHandle.elementCount(); i++)
    {
        jointsHandle.jumpToArrayElement(i);
        const int idx = jointsHandle.elementIndex();

        Joint& jnt = joints[idx];
        jnt.poses.reserve(N_POSES);

        if (baseMatricesHandle.jumpToElement(idx) == MS::kSuccess) // if there is an element exists in the idx, get the matrix
            jnt.baseMatrix = baseMatricesHandle.inputValue().asMatrix();

        if (jointOrientsHandle.jumpToElement(idx) == MS::kSuccess) // if there is an element exists in the idx, get the matrix
        {
            MDataHandle joh = jointOrientsHandle.inputValue();
            const MVector jo(
                joh.child(attr_jointOrientX).asAngle().asRadians(),
                joh.child(attr_jointOrientY).asAngle().asRadians(),
                joh.child(attr_jointOrientZ).asAngle().asRadians());

            jnt.jointOrient = MEulerRotation(jo).asQuaternion();
        }
    }

    Directory directory;
    
    MArrayDataHandle directoriesHandle = dataBlock.outputArrayValue(attr_directories);
    for (int i = 0; i < directoriesHandle.elementCount(); i++)
    {
        directoriesHandle.jumpToArrayElement(i);
        const int idx = directoriesHandle.elementIndex();

        DirectoryItem& item = directory[idx];
        item.weight = directoriesHandle.inputValue().child(attr_directoryWeight).asFloat();
        item.parentIndex = directoriesHandle.inputValue().child(attr_directoryParentIndex).asInt();

        const MFnIntArrayData intArrayData(directoriesHandle.inputValue().child(attr_directoryChildrenIndices).data());
        item.childrenIndices.resize(intArrayData.length());
        for (int k = 0; k < intArrayData.length(); k++)
            item.childrenIndices[k] = intArrayData[k];
    }

    vector<int> posesIndicesInOrder;
    posesIndicesInOrder.reserve(N_POSES);
    directory.getPosesOrder(0, posesIndicesInOrder);

    for (auto &idx : posesIndicesInOrder)
    {
        if (posesHandle.jumpToElement(idx) != MS::kSuccess)
        {
            MGlobal::displayWarning(".poses["+MSTR(idx)+"] doesn't exist");
            continue;
        }

        const int poseDirectoryIndex = posesHandle.inputValue().child(attr_poseDirectoryIndex).asInt();

        const bool poseEnabled = posesHandle.inputValue().child(attr_poseEnabled).asBool();

        const float poseWeight = posesHandle.inputValue().child(attr_poseWeight).asFloat();
        const short poseBlendMode = posesHandle.inputValue().child(attr_poseBlendMode).asShort();
        const float directoryWeight = directory.getRecursiveWeight(poseDirectoryIndex);
        const float fullPoseWeight = poseWeight * directoryWeight;

        if (fullPoseWeight > EPSILON && poseEnabled)
        {
            MArrayDataHandle poseDeltaMatrices(posesHandle.outputValue().child(attr_poseDeltaMatrices));
            for (int k = 0; k < poseDeltaMatrices.elementCount(); k++)
            {
                poseDeltaMatrices.jumpToArrayElement(k);
                const int jointIndex = poseDeltaMatrices.elementIndex();

                Pose p;
                p.weight = fullPoseWeight;
                p.blendMode = (PoseBlendMode)poseBlendMode;
                p.deltaMatrix = poseDeltaMatrices.inputValue().asMatrix();
                joints[jointIndex].poses.push_back(std::move(p));
            }
        }
    }

    MArrayDataHandle outputTranslatesHandle = dataBlock.outputArrayValue(attr_outputTranslates);
    MArrayDataHandle outputRotatesHandle = dataBlock.outputArrayValue(attr_outputRotates);
    MArrayDataHandle outputScalesHandle = dataBlock.outputArrayValue(attr_outputScales);

    MArrayDataBuilder outputTranslatesBuilder(&dataBlock, attr_outputTranslates, joints.size());
    MArrayDataBuilder outputRotatesBuilder(&dataBlock, attr_outputRotates, joints.size());
    MArrayDataBuilder outputScalesBuilder(&dataBlock, attr_outputScales, joints.size());

    // apply poses
    for (auto &item : joints)
    {       
        MVector translation = taxis(item.second.baseMatrix);
        MQuaternion rotation = mat2quat(item.second.baseMatrix);
        MVector scale = mscale(item.second.baseMatrix);

        for (const auto& p : item.second.poses)
        {
            if (p.weight > EPSILON)
            {
                MMatrix delta = p.deltaMatrix;
                const MVector deltaScale(mscale(delta));
                set_mscale(delta, MVector(1, 1, 1));

                if (p.blendMode == PoseBlendMode::ADDIVITE)
                {
                    translation += taxis(delta) * p.weight;
                    rotation = slerp(rotation, mat2quat(delta) * rotation, p.weight);
                    scale = vectorLerp(scale, vectorMult(deltaScale, scale), p.weight);
                }
                
                else if (p.blendMode == PoseBlendMode::REPLACE)
                {
                    translation = vectorLerp(translation, taxis(delta), p.weight);
                    rotation = slerp(rotation, mat2quat(delta), p.weight);
                    scale = vectorLerp(scale, deltaScale, p.weight);
                }
            }
        }

        const int& idx = item.first;

        auto tHandle = outputTranslatesBuilder.addElement(idx);
        auto rHandle = outputRotatesBuilder.addElement(idx);
        auto sHandle = outputScalesBuilder.addElement(idx);
        
        const MEulerRotation euler = mat2quat(rotation * item.second.jointOrient.inverse()).asEulerRotation();

        tHandle.setMVector(translation);

        rHandle.child(attr_outputRotateX).setMAngle(MAngle(euler.x));
        rHandle.child(attr_outputRotateY).setMAngle(MAngle(euler.y));
        rHandle.child(attr_outputRotateZ).setMAngle(MAngle(euler.z));

        sHandle.setMVector(scale);
    }

    outputTranslatesHandle.set(outputTranslatesBuilder);
    outputRotatesHandle.set(outputRotatesBuilder);
    outputScalesHandle.set(outputScalesBuilder);

    outputTranslatesHandle.setAllClean();
    outputRotatesHandle.setAllClean();
    outputScalesHandle.setAllClean();

    dataBlock.setClean(attr_outputTranslates);
    dataBlock.setClean(attr_outputRotates);
    dataBlock.setClean(attr_outputScales);
    
	return MS::kSuccess;
}

MStatus Skeleposer::initialize()
{
    MFnNumericAttribute nAttr;
    MFnMessageAttribute msgAttr;
    MFnCompoundAttribute cAttr;
    MFnMatrixAttribute mAttr;
    MFnTypedAttribute tAttr;
    MFnUnitAttribute uAttr;
    MFnEnumAttribute eAttr;
    
    attr_joints = msgAttr.create("joints", "joints");
    msgAttr.setArray(true);
    msgAttr.setHidden(true);
    addAttribute(attr_joints);


    attr_baseMatrices = mAttr.create("baseMatrices", "baseMatrices");
    mAttr.setArray(true);
    mAttr.setHidden(true);
    addAttribute(attr_baseMatrices);


    attr_jointOrientX = uAttr.create("jointOrientX", "jointOrientX", MFnUnitAttribute::kAngle);
    attr_jointOrientY = uAttr.create("jointOrientY", "jointOrientY", MFnUnitAttribute::kAngle);
    attr_jointOrientZ = uAttr.create("jointOrientZ", "jointOrientZ", MFnUnitAttribute::kAngle);

    attr_jointOrients = nAttr.create("jointOrients", "jointOrients", attr_jointOrientX, attr_jointOrientY, attr_jointOrientZ);
    nAttr.setArray(true);
    nAttr.setHidden(true);
    addAttribute(attr_jointOrients);

    
    attr_poseName = tAttr.create("poseName", "poseName", MFnData::kString);
    attr_poseWeight = nAttr.create("poseWeight", "poseWeight", MFnNumericData::kFloat, 1);
    nAttr.setMin(0);
    nAttr.setMax(1);
    nAttr.setKeyable(true);
    
    attr_poseEnabled = nAttr.create("poseEnabled", "poseEnabled", MFnNumericData::kBoolean, true);

    attr_poseBlendMode = eAttr.create("poseBlendMode", "poseBlendMode", 0);
    eAttr.addField("Additive", 0);
    eAttr.addField("Replace", 1);

    attr_poseDirectoryIndex = nAttr.create("poseDirectoryIndex", "poseDirectoryIndex", MFnNumericData::kInt, 0);
    nAttr.setMin(0);
    nAttr.setHidden(true);

    attr_poseDeltaMatrices = mAttr.create("poseDeltaMatrices", "poseDeltaMatrices");
    mAttr.setHidden(true);
    mAttr.setArray(true);
    
    attr_poses = cAttr.create("poses", "poses");
    cAttr.addChild(attr_poseName);
    cAttr.addChild(attr_poseWeight);
    cAttr.addChild(attr_poseEnabled);
    cAttr.addChild(attr_poseDirectoryIndex);
    cAttr.addChild(attr_poseBlendMode);
    cAttr.addChild(attr_poseDeltaMatrices);
    cAttr.setArray(true);      
    addAttribute(attr_poses);


    attr_directoryName = tAttr.create("directoryName", "directoryName", MFnData::kString);
    attr_directoryWeight = nAttr.create("directoryWeight", "directoryWeight", MFnNumericData::kFloat, 1);
    nAttr.setMin(0);
    nAttr.setMax(1);
    nAttr.setKeyable(true);

    attr_directoryParentIndex = nAttr.create("directoryParentIndex", "directoryParentIndex", MFnNumericData::kInt, 0);
    nAttr.setMin(0);
    nAttr.setHidden(true);

    attr_directoryChildrenIndices = tAttr.create("directoryChildrenIndices", "directoryChildrenIndices", MFnData::kIntArray);
    tAttr.setHidden(true);

    attr_directories = cAttr.create("directories", "directories");
    cAttr.addChild(attr_directoryName);
    cAttr.addChild(attr_directoryWeight);
    cAttr.addChild(attr_directoryParentIndex);
    cAttr.addChild(attr_directoryChildrenIndices);
    cAttr.setArray(true);
    addAttribute(attr_directories);


    attr_outputTranslates = nAttr.create("outputTranslates", "outputTranslates", MFnNumericData::k3Double);
    nAttr.setArray(true);
    nAttr.setHidden(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(attr_outputTranslates);


    attr_outputRotateX = uAttr.create("outputRotateX", "outputRotateX", MFnUnitAttribute::kAngle);
    attr_outputRotateY = uAttr.create("outputRotateY", "outputRotateY", MFnUnitAttribute::kAngle);
    attr_outputRotateZ = uAttr.create("outputRotateZ", "outputRotateZ", MFnUnitAttribute::kAngle);

    attr_outputRotates = nAttr.create("outputRotates", "outputRotates", attr_outputRotateX, attr_outputRotateY, attr_outputRotateZ);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);
    nAttr.setHidden(true);
    addAttribute(attr_outputRotates);


    attr_outputScales = nAttr.create("outputScales", "outputScales", MFnNumericData::k3Double);
    nAttr.setArray(true);
    nAttr.setHidden(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(attr_outputScales);


    attributeAffects(attr_baseMatrices, attr_outputTranslates);
    attributeAffects(attr_jointOrients, attr_outputTranslates);
    attributeAffects(attr_poses, attr_outputTranslates);
    attributeAffects(attr_poseWeight, attr_outputTranslates);
    attributeAffects(attr_poseBlendMode, attr_outputTranslates);
    attributeAffects(attr_poseDirectoryIndex, attr_outputTranslates);
    attributeAffects(attr_poseDeltaMatrices, attr_outputTranslates);
    attributeAffects(attr_directories, attr_outputTranslates);
    attributeAffects(attr_directoryWeight, attr_outputTranslates);
    attributeAffects(attr_directoryChildrenIndices, attr_outputTranslates);

    attributeAffects(attr_baseMatrices, attr_outputRotates);
    attributeAffects(attr_jointOrients, attr_outputRotates);
    attributeAffects(attr_poses, attr_outputRotates);
    attributeAffects(attr_poseWeight, attr_outputRotates);
    attributeAffects(attr_poseBlendMode, attr_outputRotates);
    attributeAffects(attr_poseDirectoryIndex, attr_outputRotates);
    attributeAffects(attr_poseDeltaMatrices, attr_outputRotates);
    attributeAffects(attr_directories, attr_outputRotates);
    attributeAffects(attr_directoryWeight, attr_outputRotates);
    attributeAffects(attr_directoryChildrenIndices, attr_outputRotates);

    attributeAffects(attr_baseMatrices, attr_outputScales);
    attributeAffects(attr_jointOrients, attr_outputScales);
    attributeAffects(attr_poses, attr_outputScales);
    attributeAffects(attr_poseWeight, attr_outputScales);
    attributeAffects(attr_poseBlendMode, attr_outputScales);
    attributeAffects(attr_poseDirectoryIndex, attr_outputScales);
    attributeAffects(attr_poseDeltaMatrices, attr_outputScales);
    attributeAffects(attr_directories, attr_outputScales);
    attributeAffects(attr_directoryWeight, attr_outputScales);
    attributeAffects(attr_directoryChildrenIndices, attr_outputScales);
    
    return MS::kSuccess;
}
