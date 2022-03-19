#include <algorithm>

#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MQuaternion.h>

#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>

#include "blendMatrix.h"
#include "utils.hpp"

using namespace std;

MTypeId BlendMatrix::typeId(1274441);

MObject BlendMatrix::attr_matrixA;
MObject BlendMatrix::attr_matrixB;
MObject BlendMatrix::attr_weight;
MObject BlendMatrix::attr_outMatrix;


MStatus BlendMatrix::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    if (plug != attr_outMatrix)
        return MS::kFailure;

    const MMatrix matrixA = dataBlock.inputValue(attr_matrixA).asMatrix();
    const MMatrix matrixB = dataBlock.inputValue(attr_matrixB).asMatrix();
    const float weight = dataBlock.inputValue(attr_weight).asFloat();

    const MMatrix m = blendMatrices(matrixA, matrixB, weight);
    dataBlock.outputValue(attr_outMatrix).setMMatrix(m);

	return MS::kSuccess;
}

MStatus BlendMatrix::initialize()
{
    MFnNumericAttribute nAttr;
    MFnMatrixAttribute mAttr;
    
    attr_matrixA = mAttr.create("matrixA", "matrixA");
    addAttribute(attr_matrixA);

    attr_matrixB = mAttr.create("matrixB", "matrixB");
    addAttribute(attr_matrixB);

    attr_weight = nAttr.create("weight", "weight", MFnNumericData::kFloat, 0.5);
    nAttr.setMin(0);
    nAttr.setMax(1);
    nAttr.setKeyable(true);
    addAttribute(attr_weight);

    attr_outMatrix = mAttr.create("outMatrix", "outMatrix");
    addAttribute(attr_outMatrix);
    
    attributeAffects(attr_matrixA, attr_outMatrix);
    attributeAffects(attr_matrixB, attr_outMatrix);
    attributeAffects(attr_weight, attr_outMatrix);

    return MS::kSuccess;
}
