#include <maya/MFnPlugin.h>

#include "skeleposer.h"
#include "blendMatrix.h"
#include "stickyMatrix.h"

MStatus initializePlugin(MObject plugin)
{
	MStatus stat;
	MFnPlugin pluginFn(plugin);

	stat = pluginFn.registerNode("skeleposer", Skeleposer::typeId, Skeleposer::creator, Skeleposer::initialize);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = pluginFn.registerNode("blendMatrix", BlendMatrix::typeId, BlendMatrix::creator, BlendMatrix::initialize);
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