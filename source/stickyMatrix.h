#include <vector>
#include <string>
#include <map>

#include <maya/MPxNode.h>
#include <maya/MMatrix.h>

using namespace std;

class StickyMatrix : public MPxNode
{
public:
	static MTypeId typeId;

	static MObject attr_parent1;	
	static MObject attr_translate1;
	static MObject attr_rotate1X;
	static MObject attr_rotate1Y;
	static MObject attr_rotate1Z;
	static MObject attr_rotate1;
	static MObject attr_scale1;
	static MObject attr_jointOrient1X;
	static MObject attr_jointOrient1Y;
	static MObject attr_jointOrient1Z;
	static MObject attr_jointOrient1;
	static MObject attr_offset1;

	static MObject attr_parent2;
	static MObject attr_translate2;
	static MObject attr_rotate2X;
	static MObject attr_rotate2Y;
	static MObject attr_rotate2Z;
	static MObject attr_rotate2;
	static MObject attr_scale2;
	static MObject attr_jointOrient2X;
	static MObject attr_jointOrient2Y;
	static MObject attr_jointOrient2Z;
	static MObject attr_jointOrient2;
	static MObject attr_offset2;

	static MObject attr_sticky;
	static MObject attr_blendAt;
	static MObject attr_spherical;
	static MObject attr_sphereCenter;
	static MObject attr_sphericalLengthFactor;

	static MObject attr_outMatrix1;
	static MObject attr_outMatrix2;

	static void* creator() { return new StickyMatrix(); }
	static MStatus initialize();

	MStatus compute(const MPlug&, MDataBlock&);
};
