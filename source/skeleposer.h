#include <vector>
#include <string>
#include <map>

#include <maya/MPxNode.h>
#include <maya/MMatrix.h>
#include <maya/MQuaternion.h>

using namespace std;

struct DirectoryItem
{
    float weight;
    int parentIndex;
    vector<int> childrenIndices; // positive - poses, negative - directories
};

class Directory
{
public:
	Directory() {}

	void clear() { _items.clear(); }
	bool empty() const { return _items.empty(); }

	DirectoryItem& operator[](int idx) { return _items[idx]; }

	float getRecursiveWeight(int idx) const
	{
		auto& found = _items.find(idx);
		if (found == _items.end())
			return 1;

		auto& item = found->second;

		if (idx == item.parentIndex)
			return item.weight;

		return item.weight * getRecursiveWeight(item.parentIndex);
	}

	void getPosesOrder(int idx, vector<int> &poseIndices) const
	{
		auto& found = _items.find(idx);
		if (found == _items.end())
			return;

		for (auto& idx : found->second.childrenIndices)
		{
			if (idx >= 0)
				poseIndices.push_back(idx);
			else
				getPosesOrder(-idx, poseIndices);
		}
	}

private:
	map<int, DirectoryItem> _items;
};

typedef enum
{
	ADDIVITE,
	REPLACE
} PoseBlendMode;

struct Pose
{
	float weight;
	PoseBlendMode blendMode;
	MMatrix deltaMatrix;
};

struct Joint
{
	MMatrix baseMatrix;
	MQuaternion jointOrient;
	vector<Pose> poses;	
};

class Skeleposer : public MPxNode
{
public:
	static MTypeId typeId;

	static MObject attr_joints;
	static MObject attr_baseMatrices;
	static MObject attr_jointOrientX;
	static MObject attr_jointOrientY;
	static MObject attr_jointOrientZ;
	static MObject attr_jointOrients;
	static MObject attr_poses;
	static MObject attr_poseName;
	static MObject attr_poseWeight;
	static MObject attr_poseEnabled;
	static MObject attr_poseBlendMode;
	static MObject attr_poseDirectoryIndex;
	static MObject attr_poseDeltaMatrices;

	static MObject attr_directories;
	static MObject attr_directoryName;
	static MObject attr_directoryWeight;
	static MObject attr_directoryParentIndex;
	static MObject attr_directoryChildrenIndices; // positive - poses, negative - directories

	static MObject attr_outputTranslates;

	static MObject attr_outputRotateX;
	static MObject attr_outputRotateY;
	static MObject attr_outputRotateZ;
	static MObject attr_outputRotates;

	static MObject attr_outputScales;

	static void* creator() { return new Skeleposer(); }
	static MStatus initialize();

	MStatus compute(const MPlug&, MDataBlock&);
	
	MStatus shouldSave(const MPlug& plug, bool& isSaving)
	{
		isSaving = true; // save everything
		return MS::kSuccess;
	}

	void postConstructor()
	{
		setExistWithoutInConnections(true);
		setExistWithoutOutConnections(true);
	}

private:	
};
