#include <vector>
#include <string>
#include <map>

#include <maya/MPxNode.h>
#include <maya/MMatrix.h>

using namespace std;

class BlendMatrix : public MPxNode
{
public:
	static MTypeId typeId;

	static MObject attr_matrixA;
	static MObject attr_matrixB;
	static MObject attr_weight;
	static MObject attr_outMatrix;

	static void* creator() { return new BlendMatrix(); }
	static MStatus initialize();

	MStatus compute(const MPlug&, MDataBlock&);
};
