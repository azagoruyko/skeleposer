import maya.api.OpenMaya as om
import maya.cmds as cmds
import pymel.core as pm

NamingScheme = {
    "LeftStart": {"L_": "R_", "l_": "r_", "Left":"Right", "left_": "right_"},
    "LeftEnd": {"_L": "_R", "_l": "_r", "Left": "Right", "_left":"_right"},
    "RightStart": {"R_": "L_", "r_": "l_", "Right":"Left", "right_":"left_"},
    "RightEnd": {"_R": "_L", "_r": "_l", "Right":"Left", "_right":"_left"},
    "LeftMiddle": {"Left":"Right", "_L_":"_R_", "_l_":"_r_"},
    "RightMiddle": {"Right":"Left", "_R_":"_L_", "_r_":"_l_"},
}

def findSymmetricName(name: str, left: bool = True, right: bool = True) -> str:
    """Find the symmetric equivalent of a given name based on naming conventions."""
    leftData = (left, NamingScheme["LeftStart"], NamingScheme["LeftMiddle"], NamingScheme["LeftEnd"])
    rightData = (right, NamingScheme["RightStart"], NamingScheme["RightMiddle"], NamingScheme["RightEnd"])

    for enable, starts, mids, ends in [leftData, rightData]:
        if enable:
            for s in starts:
                if name.startswith(s):
                    return starts[s] + name[len(s):]

            for s in ends:
                if name.endswith(s):
                    return name[:-len(s)] + ends[s]

            for s in mids:
                if s in name:
                    idx = name.index(s)
                    return name[:idx] + mids[s] + name[idx+len(s):]

def isLeftSide(name: str) -> bool:
    """Check if a name belongs to the left side based on naming conventions."""
    for s in NamingScheme["LeftStart"]:
        if name.startswith(s):
            return True

    for s in NamingScheme["LeftEnd"]:
        if name.endswith(s):
            return True

    for s in NamingScheme["LeftMiddle"]:
        if s in name:
            return True

def isRightSide(name: str) -> bool:
    """Check if a name belongs to the right side based on naming conventions."""
    for s in NamingScheme["RightStart"]:
        if name.startswith(s):
            return True

    for s in NamingScheme["RightEnd"]:
        if name.endswith(s):
            return True

    for s in NamingScheme["RightMiddle"]:
        if s in name:
            return True

def undoBlock(f):
    """Decorator that wraps a function in Maya's undo block for atomic undo operations."""
    def inner(*args,**kwargs):
        pm.undoInfo(ock=True, cn=f.__name__)
        try:
            out = f(*args, **kwargs)
        finally:
            pm.undoInfo(cck=True)
        return out
    return inner

def clamp(v: float, mn: float = 0.0, mx: float = 1.0) -> float:
    """Clamp a value between minimum and maximum bounds."""
    if v > mx:
        return mx
    elif v < mn:
        return mn
    return v

def shortenValue(v: float, epsilon: float = 1e-5) -> float:
    """Round a value to the nearest integer if within epsilon tolerance."""
    roundedValue = round(v)
    return roundedValue if abs(v - roundedValue) < epsilon else v

def maxis(m: om.MMatrix, a: int) -> om.MVector:
    """Extract axis vector from a 4x4 matrix (0=X, 1=Y, 2=Z, 3=Translation)."""
    return om.MVector(m[a*4+0], m[a*4+1], m[a*4+2])

def set_maxis(m: om.MMatrix, a: int, v: om.MVector):
    """Set axis vector in a 4x4 matrix (0=X, 1=Y, 2=Z, 3=Translation)."""
    m[a*4+0] = v[0]
    m[a*4+1] = v[1]
    m[a*4+2] = v[2]

def mscale(m: om.MMatrix) -> om.MVector:
    """Extract scale values from a transformation matrix."""
    return om.MVector(maxis(m,0).length(), maxis(m,1).length(), maxis(m,2).length())

def set_mscale(m: om.MMatrix, s: om.MVector):
    """Apply scale values to a transformation matrix preserving rotation."""
    set_maxis(m, 0, maxis(m, 0).normal()*s[0])
    set_maxis(m, 1, maxis(m, 1).normal()*s[1])
    set_maxis(m, 2, maxis(m, 2).normal()*s[2])

def mscaled(m: om.MMatrix, s: om.MVector = om.MVector(1,1,1)) -> om.MMatrix:
    """Return a copy of the matrix with modified scale values."""
    m = om.MMatrix(m)
    set_maxis(m, 0, maxis(m, 0).normal()*s[0])
    set_maxis(m, 1, maxis(m, 1).normal()*s[1])
    set_maxis(m, 2, maxis(m, 2).normal()*s[2])
    return m

def slerp(q1: tuple, q2: tuple, w: float) -> om.MQuaternion:
    """Perform spherical linear interpolation between two quaternions."""
    q1 = om.MQuaternion(q1[0], q1[1], q1[2], q1[3])
    q2 = om.MQuaternion(q2[0], q2[1], q2[2], q2[3])
    return om.MQuaternion.slerp(q1, q2, w)

def blendMatrices(m1: om.MMatrix, m2: om.MMatrix, w: float) -> om.MMatrix:
    """Blend two transformation matrices using linear and spherical interpolation."""
    m1 = om.MMatrix(m1)
    m2 = om.MMatrix(m2)

    q1 = om.MTransformationMatrix(mscaled(m1)).rotation().asQuaternion()
    q2 = om.MTransformationMatrix(mscaled(m2)).rotation().asQuaternion()

    s = mscale(m1) * (1-w) + mscale(m2) * w
    m = om.MMatrix(mscaled(slerp(q1, q2, w).asMatrix(), s))

    set_maxis(m, 3, maxis(m1, 3)*(1-w) + maxis(m2, 3)*w)
    return m

def getLocalMatrix(joint: pm.PyNode) -> om.MMatrix:
    """Get joint's local transformation matrix including joint orient."""
    q = om.MQuaternion(joint.getRotation().asQuaternion())

    if cmds.objectType(str(joint)) == "joint":
        q *= om.MQuaternion(joint.getOrientation())

    t = cmds.getAttr(joint+".t")[0]
    s = cmds.getAttr(joint+".s")[0]

    qm = q.asMatrix()

    sm = om.MMatrix()
    sm[0] = s[0]
    sm[5] = s[1]
    sm[10] = s[2]

    m = sm * qm
    m[12] = t[0]
    m[13] = t[1]
    m[14] = t[2]

    return om.MMatrix(m)

def getDelta(m: om.MMatrix, bm: om.MMatrix) -> om.MMatrix:
    """Calculate delta transformation between pose matrix and base matrix."""
    m = om.MMatrix(m)
    bm = om.MMatrix(bm)

    s = mscale(m)
    bs = mscale(bm)

    d = m * bm.inverse()

    # translation is simple as well as scale
    d[12] = m[12]-bm[12]
    d[13] = m[13]-bm[13]
    d[14] = m[14]-bm[14]

    sx = s[0]/bs[0]
    sy = s[1]/bs[1]
    sz = s[2]/bs[2]
    set_mscale(d, [sx,sy,sz])
    return d

def applyDelta(dm: om.MMatrix, bm: om.MMatrix) -> om.MMatrix:
    """Apply delta transformation to a base matrix."""
    dm = om.MMatrix(dm)
    bm = om.MMatrix(bm)

    ds = mscale(dm)
    bms = mscale(bm)

    m = dm * bm # get rotation matrix

    # translation is simple as well as scale
    m[12] = bm[12]+dm[12]
    m[13] = bm[13]+dm[13]
    m[14] = bm[14]+dm[14]

    sx = ds[0]*bms[0]
    sy = ds[1]*bms[1]
    sz = ds[2]*bms[2]

    set_mscale(m, [sx, sy, sz])
    return m

def symmat(m: om.MMatrix) -> om.MMatrix:
    """Create a symmetric matrix by flipping the X axis."""
    out = om.MMatrix(m)
    out[0] *= -1
    out[4] *= -1
    out[8] *= -1
    out[12] *= -1
    return out

def parentConstraintMatrix(destBase: om.MMatrix, srcBase: om.MMatrix, src: om.MMatrix) -> om.MMatrix:
    """Calculate resulting matrix as in Maya's parentConstraint transformation."""
    return destBase * srcBase.inverse() * src

def mirrorMatrix(base: om.MMatrix, srcBase: om.MMatrix, src: om.MMatrix) -> om.MMatrix:
    """Mirror a transformation matrix across the YZ plane."""
    return parentConstraintMatrix(base, symmat(srcBase), symmat(src))

def mirrorMatrixByDelta(srcBase: om.MMatrix, src: om.MMatrix, destBase: om.MMatrix) -> om.MMatrix:
    """Mirror matrix using delta transformation between source and destination."""
    mirroredSrcBase = mirrorMatrix(om.MMatrix(), om.MMatrix(), srcBase)
    mirroredSrc = mirrorMatrix(om.MMatrix(), om.MMatrix(), src)

    # set translation the same, used for rotation
    dt = maxis(mirroredSrcBase,3) - maxis(destBase,3)
    set_maxis(mirroredSrc, 3, maxis(mirroredSrc, 3) - dt)
    set_maxis(mirroredSrcBase, 3, maxis(mirroredSrcBase, 3) - dt)

    return parentConstraintMatrix(destBase, mirroredSrcBase, mirroredSrc)

def dagPose_findIndex(dagPose: pm.PyNode, j: pm.PyNode) -> int:
    """Find the index of a joint in a dagPose node."""
    for m in dagPose.members:
        inputs = m.inputs(sh=True)
        if inputs and inputs[0] == j:
            return m.index()

def dagPose_getWorldMatrix(dagPose: pm.PyNode, j: pm.PyNode) -> om.MMatrix:
    """Get world matrix of a joint from a dagPose node."""
    idx = dagPose_findIndex(dagPose, j)
    if idx is not None:
        return om.MMatrix(dagPose.worldMatrix[idx].get())

def dagPose_getParentMatrix(dagPose: pm.PyNode, j: pm.PyNode) -> om.MMatrix:
    """Get parent world matrix of a joint from a dagPose node."""
    idx = dagPose_findIndex(dagPose, j)
    if idx is not None:
        parent = dagPose.parents[idx].inputs(p=True, sh=True)
        if parent and parent[0] != dagPose.world:
            return om.MMatrix(dagPose.worldMatrix[parent[0].index()].get())
    return om.MMatrix()

def getRemapInputPlug(remap: pm.PyNode) -> pm.Attribute:
    """Get the actual input plug connected to a remapValue node."""
    inputs = remap.inputValue.inputs(p=True)
    if inputs:
        inputPlug = inputs[0]
        if pm.objectType(inputPlug.node()) == "unitConversion":
            inputs = inputPlug.node().input.inputs(p=True)
            if inputs:
                return inputs[0]
        else:
            return inputPlug

def getRemapActualWeightInput(plug: pm.Attribute) -> pm.Attribute:
    """Get the actual weight input plug bypassing remapValue and unitConversion nodes."""
    inputs = plug.inputs(p=True)
    if inputs:
        inputPlug = inputs[0]
        if pm.objectType(inputPlug.node()) == "remapValue":
            return getRemapInputPlug(inputPlug.node())

        elif pm.objectType(inputPlug.node()) == "unitConversion":
            inputs = inputPlug.node().input.inputs(p=True)
            if inputs:
                return inputs[0]

        else:
            return inputPlug

def clearUnusedRemapValue():
    """Delete all remapValue nodes that have no output connections."""
    pm.delete([n for n in pm.ls(type="remapValue") if not n.outValue.isConnected() and not n.outColor.isConnected()])

def getBlendShapeTargetDelta(blendShape: pm.PyNode, targetIndex: int) -> tuple:
    """Get vertex indices and delta positions for a blend shape target."""
    targetDeltas = blendShape.inputTarget[0].inputTargetGroup[targetIndex].inputTargetItem[6000].inputPointsTarget.get()
    targetComponentsPlug = blendShape.inputTarget[0].inputTargetGroup[targetIndex].inputTargetItem[6000].inputComponentsTarget.__apimplug__()

    targetIndices = []
    componentList = pm.api.MFnComponentListData(targetComponentsPlug.asMObject())
    for i in range(componentList.length()):
        compTargetIndices = pm.api.MIntArray()
        singleIndexFn = pm.api.MFnSingleIndexedComponent(componentList[i])
        singleIndexFn.getElements(compTargetIndices)
        targetIndices += compTargetIndices

    return targetIndices, targetDeltas

def matchJoint(j: pm.PyNode, name: str = None) -> pm.PyNode:
    """Create a new joint matching the world transformation of an existing joint."""
    newj = pm.createNode("joint", n=name or j.name())
    pm.xform(newj, ws=True, m=pm.xform(j, q=True, ws=True, m=True))
    newj.setOrientation(newj.getOrientation()*newj.getRotation().asQuaternion()) # freeze
    newj.setRotation([0,0,0])
    return newj

def transferSkin(src: pm.PyNode, dest: pm.PyNode):
    """Transfer skin cluster connections from source joint to destination joint."""
    for p in src.wm.outputs(p=True, type="skinCluster"):
        dest.wm >> p

        if not dest.hasAttr("lockInfluenceWeights"):
            dest.addAttr("lockInfluenceWeights", at="bool", dv=False)

        dest.lockInfluenceWeights >> p.node().lockWeights[p.index()]
        #p.node().bindPreMatrix[p.index()].set(dest.wim.get())