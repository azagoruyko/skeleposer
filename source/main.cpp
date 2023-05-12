#include <maya/MFnPlugin.h>

#include "skeleposer.h"
#include "blendMatrix.h" // blendMatrix already defined in Maya 2022+
#include "stickyMatrix.h"

#if MAYA_APP_VERSION >= 2020 // blendMatrix node is defined in Maya 2020+
#define BLEND_MATRIX_NAME "sblendMatrix"
#else
#define BLEND_MATRIX_NAME "blendMatrix"
#endif

MStatus initializePlugin(MObject plugin)
{
	MStatus stat;
	MFnPlugin pluginFn(plugin);

	stat = pluginFn.registerNode("skeleposer", Skeleposer::typeId, Skeleposer::creator, Skeleposer::initialize);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	
	stat = pluginFn.registerNode(MString(BLEND_MATRIX_NAME), BlendMatrix::typeId, BlendMatrix::creator, BlendMatrix::initialize);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = pluginFn.registerNode("stickyMatrix", StickyMatrix::typeId, StickyMatrix::creator, StickyMatrix::initialize);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	return MS::kSuccess;
}

MStatus uninitializePlugin(MObject plugin)
{
	MStatus stat;
	MFnPlugin pluginFn(plugin);

	stat = pluginFn.deregisterNode(Skeleposer::typeId);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = pluginFn.deregisterNode(BlendMatrix::typeId);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = pluginFn.deregisterNode(StickyMatrix::typeId);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	return MS::kSuccess;
}