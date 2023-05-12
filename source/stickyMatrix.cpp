#include <algorithm>

#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>

#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>

#include "stickyMatrix.h"
#include "utils.hpp"

using namespace std;

MTypeId StickyMatrix::typeId(1274442);

MObject StickyMatrix::attr_parent1;
MObject StickyMatrix::attr_parent1WithoutScale;
MObject StickyMatrix::attr_parent2;
MObject StickyMatrix::attr_parent2WithoutScale;
MObject StickyMatrix::attr_translate1;
MObject StickyMatrix::attr_rotate1X;
MObject StickyMatrix::attr_rotate1Y;
MObject StickyMatrix::attr_rotate1Z;
MObject StickyMatrix::attr_rotate1;
MObject StickyMatrix::attr_scale1;
MObject StickyMatrix::attr_jointOrient1X;
MObject StickyMatrix::attr_jointOrient1Y;
MObject StickyMatrix::attr_jointOrient1Z;
MObject StickyMatrix::attr_jointOrient1;
MObject StickyMatrix::attr_offset1;

MObject StickyMatrix::attr_translate2;
MObject StickyMatrix::attr_rotate2X;
MObject StickyMatrix::attr_rotate2Y;
MObject StickyMatrix::attr_rotate2Z;
MObject StickyMatrix::attr_rotate2;
MObject StickyMatrix::attr_scale2;
MObject StickyMatrix::attr_jointOrient2X;
MObject StickyMatrix::attr_jointOrient2Y;
MObject StickyMatrix::attr_jointOrient2Z;
MObject StickyMatrix::attr_jointOrient2;
MObject StickyMatrix::attr_offset2;

MObject StickyMatrix::attr_sticky;
MObject StickyMatrix::attr_blendAt;
MObject StickyMatrix::attr_spherical;
MObject StickyMatrix::attr_sphereCenter;
MObject StickyMatrix::attr_sphericalLengthFactor;

MObject StickyMatrix::attr_outMatrix1;
MObject StickyMatrix::attr_outMatrix2;

MStatus StickyMatrix::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    if (plug != attr_outMatrix1 && plug != attr_outMatrix2)
        return MS::kUnknownParameter;
    
    const bool parent1WithoutScale = dataBlock.inputValue(attr_parent1WithoutScale).asBool();
    const bool parent2WithoutScale = dataBlock.inputValue(attr_parent2WithoutScale).asBool();

    MMatrix parent1 = dataBlock.inputValue(attr_parent1).asMatrix();
    MMatrix parent2 = dataBlock.inputValue(attr_parent2).asMatrix();

    if (parent1WithoutScale)
        set_mscale(parent1, MVector(1, 1, 1));

    if (parent2WithoutScale)
        set_mscale(parent2, MVector(1, 1, 1));

    const MVector translate1 = dataBlock.inputValue(attr_translate1).asVector();
    const MVector translate2 = dataBlock.inputValue(attr_translate2).asVector();
    const MVector rotate1 = dataBlock.inputValue(attr_rotate1).asVector();
    const MVector rotate2 = dataBlock.inputValue(attr_rotate2).asVector();
    const MVector scale1 = dataBlock.inputValue(attr_scale1).asVector();
    const MVector scale2 = dataBlock.inputValue(attr_scale2).asVector();
    const MVector jointOrient1 = dataBlock.inputValue(attr_jointOrient1).asVector();
    const MVector jointOrient2 = dataBlock.inputValue(attr_jointOrient2).asVector();
    const MMatrix offset1 = dataBlock.inputValue(attr_offset1).asMatrix();
    const MMatrix offset2 = dataBlock.inputValue(attr_offset2).asMatrix();

    const float blendAt = dataBlock.inputValue(attr_blendAt).asFloat();
    const float sticky = dataBlock.inputValue(attr_sticky).asFloat();

    const float spherical = dataBlock.inputValue(attr_spherical).asFloat();
    const MVector sphereCenter = dataBlock.inputValue(attr_sphereCenter).asVector();
    const float sphericalLengthFactor = dataBlock.inputValue(attr_sphericalLengthFactor).asFloat();

    const MMatrix joMat1(MEulerRotation(jointOrient1).asMatrix());
    const MMatrix joMat2(MEulerRotation(jointOrient2).asMatrix());

    MMatrix inputMatrix1(MEulerRotation(rotate1).asMatrix() * joMat1);
    set_maxis(inputMatrix1, 3, translate1);    
    set_mscale(inputMatrix1, scale1);

    MMatrix inputMatrix2(MEulerRotation(rotate2).asMatrix() * joMat2);
    set_maxis(inputMatrix2, 3, translate2);
    set_mscale(inputMatrix2, scale2);
    
    const MMatrix inputMatrixWorld1 = inputMatrix1 * parent1;
    const MMatrix inputMatrixWorld2 = inputMatrix2 * parent2;

    const MMatrix avg = blendMatrices(inputMatrixWorld1, inputMatrixWorld2, blendAt);

    MMatrix avgBlend1 = blendMatrices(inputMatrixWorld1, offset1*avg, sticky);
    MMatrix avgBlend2 = blendMatrices(inputMatrixWorld2, offset2*avg, sticky);

    /*
tm = loc_m * j_pm.inverse()
rm = loc_m * (jo * j_pm).inverse()

m = pm.dt.Matrix(rm)
m.a30 = tm.a30
m.a31 = tm.a31
m.a32 = tm.a32
    */

    // spherical
    const double d1 = (taxis(inputMatrixWorld1) - sphereCenter).length() * sphericalLengthFactor;
    const double d2 = (taxis(inputMatrixWorld2) - sphereCenter).length() * sphericalLengthFactor;
    
    const MPoint sp1 = sphereCenter + (taxis(avgBlend1) - sphereCenter).normal() * d1;
    const MPoint sp2 = sphereCenter + (taxis(avgBlend2) - sphereCenter).normal() * d2;

    const MVector p1 = taxis(avgBlend1) * (1 - spherical * sticky) + sp1 * spherical * sticky;
    const MVector p2 = taxis(avgBlend2) * (1 - spherical * sticky) + sp2 * spherical * sticky;

    set_maxis(avgBlend1, 3, p1);
    set_maxis(avgBlend2, 3, p2);
    
    // output
    const MMatrix tm1 = avgBlend1 * parent1.inverse();
    const MMatrix tm2 = avgBlend2 * parent2.inverse();

    const MMatrix rm1 = avgBlend1 * (joMat1 * parent1).inverse();
    const MMatrix rm2 = avgBlend2 * (joMat2 * parent2).inverse();

    MMatrix m1(rm1);
    MMatrix m2(rm2);
    set_maxis(m1, 3, taxis(tm1));
    set_maxis(m2, 3, taxis(tm2));

    dataBlock.outputValue(attr_outMatrix1).setMMatrix(m1);
    dataBlock.outputValue(attr_outMatrix2).setMMatrix(m2);

    dataBlock.setClean(attr_outMatrix1);
    dataBlock.setClean(attr_outMatrix2);

	return MS::kSuccess;
}

MStatus StickyMatrix::initialize()
{
    MFnNumericAttribute nAttr;
    MFnMatrixAttribute mAttr;
    MFnUnitAttribute uAttr;
    
    attr_parent1 = mAttr.create("parent1", "p1");
    mAttr.setHidden(true);
    addAttribute(attr_parent1);

    attr_parent1WithoutScale = nAttr.create("parent1WithoutScale", "p1wos", MFnNumericData::kBoolean, false);
    addAttribute(attr_parent1WithoutScale);

    attr_translate1 = nAttr.create("translate1", "t1", MFnNumericData::k3Double);
    nAttr.setKeyable(true);
    addAttribute(attr_translate1);

    attr_rotate1X = uAttr.create("rotate1X", "r1x", MFnUnitAttribute::kAngle);
    attr_rotate1Y = uAttr.create("rotate1Y", "r1y", MFnUnitAttribute::kAngle);
    attr_rotate1Z = uAttr.create("rotate1Z", "r1z", MFnUnitAttribute::kAngle);

    attr_rotate1 = nAttr.create("rotate1", "r1", attr_rotate1X, attr_rotate1Y, attr_rotate1Z);
    nAttr.setKeyable(true);
    addAttribute(attr_rotate1);

    attr_scale1 = nAttr.create("scale1", "s1", MFnNumericData::k3Double, 1);
    nAttr.setKeyable(true);
    addAttribute(attr_scale1);

    attr_jointOrient1X = uAttr.create("jointOrient1X", "jo1x", MFnUnitAttribute::kAngle);
    attr_jointOrient1Y = uAttr.create("jointOrient1Y", "jo1y", MFnUnitAttribute::kAngle);
    attr_jointOrient1Z = uAttr.create("jointOrient1Z", "jo1z", MFnUnitAttribute::kAngle);

    attr_jointOrient1 = nAttr.create("jointOrient1", "jo1", attr_jointOrient1X, attr_jointOrient1Y, attr_jointOrient1Z);
    nAttr.setKeyable(true);
    addAttribute(attr_jointOrient1);

    attr_offset1 = mAttr.create("offset1", "o1");
    mAttr.setHidden(true);
    addAttribute(attr_offset1);


    attr_parent2 = mAttr.create("parent2", "p2");
    mAttr.setHidden(true);
    addAttribute(attr_parent2);

    attr_parent2WithoutScale = nAttr.create("parent2WithoutScale", "p2wos", MFnNumericData::kBoolean, false);
    addAttribute(attr_parent2WithoutScale);

    attr_translate2 = nAttr.create("translate2", "t2", MFnNumericData::k3Double);
    nAttr.setKeyable(true);
    addAttribute(attr_translate2);


    attr_rotate2X = uAttr.create("rotate2X", "r2x", MFnUnitAttribute::kAngle);
    attr_rotate2Y = uAttr.create("rotate2Y", "r2y", MFnUnitAttribute::kAngle);
    attr_rotate2Z = uAttr.create("rotate2Z", "r2z", MFnUnitAttribute::kAngle);

    attr_rotate2 = nAttr.create("rotate2", "r2", attr_rotate2X, attr_rotate2Y, attr_rotate2Z);
    nAttr.setKeyable(true);
    addAttribute(attr_rotate2);


    attr_scale2 = nAttr.create("scale2", "s2", MFnNumericData::k3Double, 1);
    nAttr.setKeyable(true);
    addAttribute(attr_scale2);

    attr_jointOrient2X = uAttr.create("jointOrient2X", "jo2x", MFnUnitAttribute::kAngle);
    attr_jointOrient2Y = uAttr.create("jointOrient2Y", "jo2y", MFnUnitAttribute::kAngle);
    attr_jointOrient2Z = uAttr.create("jointOrient2Z", "jo2z", MFnUnitAttribute::kAngle);

    attr_jointOrient2 = nAttr.create("jointOrient2", "jo2", attr_jointOrient2X, attr_jointOrient2Y, attr_jointOrient2Z);
    nAttr.setKeyable(true);
    addAttribute(attr_jointOrient2);

    attr_offset2 = mAttr.create("offset2", "o2");
    mAttr.setHidden(true);
    addAttribute(attr_offset2);


    attr_blendAt = nAttr.create("blendAt", "bl", MFnNumericData::kFloat, 0.5);
    nAttr.setMin(0);
    nAttr.setMax(1);
    nAttr.setKeyable(true);
    addAttribute(attr_blendAt);

    attr_sticky = nAttr.create("sticky", "st", MFnNumericData::kFloat, 0);
    nAttr.setMin(0);
    nAttr.setMax(1);
    nAttr.setKeyable(true);
    addAttribute(attr_sticky);

    attr_spherical = nAttr.create("spherical", "sph", MFnNumericData::kFloat, 0);
    nAttr.setMin(0);
    nAttr.setMax(1);
    nAttr.setKeyable(true);
    addAttribute(attr_spherical);

    attr_sphereCenter = nAttr.create("sphereCenter", "sphc", MFnNumericData::k3Double);
    nAttr.setKeyable(true);
    addAttribute(attr_sphereCenter);

    attr_sphericalLengthFactor = nAttr.create("sphericalLengthFactor", "sphlf", MFnNumericData::kFloat, 1);
    nAttr.setMin(0);
    nAttr.setMax(10);
    nAttr.setKeyable(true);
    addAttribute(attr_sphericalLengthFactor);


    attr_outMatrix1 = mAttr.create("outMatrix1", "om1");
    mAttr.setHidden(true);
    addAttribute(attr_outMatrix1);

    attr_outMatrix2 = mAttr.create("outMatrix2", "om2");
    mAttr.setHidden(true);
    addAttribute(attr_outMatrix2);
    
    attributeAffects(attr_parent1, attr_outMatrix1);
    attributeAffects(attr_parent1WithoutScale, attr_outMatrix1);
    attributeAffects(attr_parent2, attr_outMatrix1);
    attributeAffects(attr_parent2WithoutScale, attr_outMatrix1);
    attributeAffects(attr_translate1, attr_outMatrix1);
    attributeAffects(attr_translate2, attr_outMatrix1);
    attributeAffects(attr_rotate1, attr_outMatrix1);
    attributeAffects(attr_rotate2, attr_outMatrix1);
    attributeAffects(attr_scale1, attr_outMatrix1);
    attributeAffects(attr_scale2, attr_outMatrix1);
    attributeAffects(attr_jointOrient1, attr_outMatrix1);
    attributeAffects(attr_jointOrient2, attr_outMatrix1);
    attributeAffects(attr_offset1, attr_outMatrix1);
    attributeAffects(attr_offset2, attr_outMatrix1);
    attributeAffects(attr_blendAt, attr_outMatrix1);
    attributeAffects(attr_sticky, attr_outMatrix1);
    attributeAffects(attr_spherical, attr_outMatrix1);
    attributeAffects(attr_sphereCenter, attr_outMatrix1);
    attributeAffects(attr_sphericalLengthFactor, attr_outMatrix1);

    attributeAffects(attr_parent1, attr_outMatrix2);
    attributeAffects(attr_parent1WithoutScale, attr_outMatrix2);
    attributeAffects(attr_parent2, attr_outMatrix2);
    attributeAffects(attr_parent2WithoutScale, attr_outMatrix2);
    attributeAffects(attr_translate1, attr_outMatrix2);
    attributeAffects(attr_translate2, attr_outMatrix2);
    attributeAffects(attr_rotate1, attr_outMatrix2);
    attributeAffects(attr_rotate2, attr_outMatrix2);
    attributeAffects(attr_scale1, attr_outMatrix2);
    attributeAffects(attr_scale2, attr_outMatrix2);
    attributeAffects(attr_jointOrient1, attr_outMatrix2);
    attributeAffects(attr_jointOrient2, attr_outMatrix2);
    attributeAffects(attr_offset1, attr_outMatrix2);
    attributeAffects(attr_offset2, attr_outMatrix2);
    attributeAffects(attr_blendAt, attr_outMatrix2);
    attributeAffects(attr_sticky, attr_outMatrix2);
    attributeAffects(attr_spherical, attr_outMatrix2);
    attributeAffects(attr_sphereCenter, attr_outMatrix2);
    attributeAffects(attr_sphericalLengthFactor, attr_outMatrix2);

    return MS::kSuccess;
}
