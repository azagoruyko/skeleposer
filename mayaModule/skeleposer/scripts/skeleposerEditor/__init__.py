import re
import os
import json
from contextlib import contextmanager

from PySide2.QtGui import *
from PySide2.QtCore import *
from PySide2.QtWidgets import *

import maya.api.OpenMaya as om
import pymel.core as pm
import maya.cmds as cmds

from shiboken2 import wrapInstance
mayaMainWindow = wrapInstance(int(pm.api.MQtUtil.mainWindow()), QMainWindow)

RootDirectory = os.path.dirname(__file__)

NamingScheme = {
    "LeftStart": {"L_": "R_", "l_": "r_", "Left":"Right", "left_": "right_"},
    "LeftEnd": {"_L": "_R", "_l": "_r", "Left": "Right", "_left":"_right"},
    "RightStart": {"R_": "L_", "r_": "l_", "Right":"Left", "right_":"left_"},
    "RightEnd": {"_R": "_L", "_r": "_l", "Right":"Left", "_right":"_left"},
    "LeftMiddle": {"Left":"Right", "_L_":"_R_", "_l_":"_r_"},
    "RightMiddle": {"Right":"Left", "_R_":"_L_", "_r_":"_l_"},
}

def getQWidgetFromMelControl(ctrl):
    ptr = pm.api.MQtUtil.findControl(ctrl)
    return wrapInstance(int(ptr), QWidget)

def findSymmetricName(name, left=True, right=True):
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

def isLeftSide(name):
    for s in NamingScheme["LeftStart"]:
        if name.startswith(s):
            return True

    for s in NamingScheme["LeftEnd"]:
        if name.endswith(s):
            return True

    for s in NamingScheme["LeftMiddle"]:
        if s in name:
            return True

def isRightSide(name):
    for s in NamingScheme["RightStart"]:
        if name.startswith(s):
            return True

    for s in NamingScheme["RightEnd"]:
        if name.endswith(s):
            return True

    for s in NamingScheme["RightMiddle"]:
        if s in name:
            return True

def clamp(v, mn=0.0, mx=1.0):
    if v > mx:
        return mx
    elif v < mn:
        return mn
    return v

def shortenValue(v, epsilon=1e-5):
    roundedValue = round(v)
    return roundedValue if abs(v - roundedValue) < epsilon else v

def maxis(m, a):
    return om.MVector(m[a*4+0], m[a*4+1], m[a*4+2])

def set_maxis(m, a, v):
    m[a*4+0] = v[0]
    m[a*4+1] = v[1]
    m[a*4+2] = v[2]

def mscale(m):
    return om.MVector(maxis(m,0).length(), maxis(m,1).length(), maxis(m,2).length())

def set_mscale(m, s):
    set_maxis(m, 0, maxis(m, 0).normal()*s[0])
    set_maxis(m, 1, maxis(m, 1).normal()*s[1])
    set_maxis(m, 2, maxis(m, 2).normal()*s[2])

def mscaled(m, s=om.MVector(1,1,1)):
    m = om.MMatrix(m)
    set_maxis(m, 0, maxis(m, 0).normal()*s[0])
    set_maxis(m, 1, maxis(m, 1).normal()*s[1])
    set_maxis(m, 2, maxis(m, 2).normal()*s[2])
    return m

def slerp(q1, q2, w):
    q1 = om.MQuaternion(q1[0], q1[1], q1[2], q1[3])
    q2 = om.MQuaternion(q2[0], q2[1], q2[2], q2[3])
    return om.MQuaternion.slerp(q1, q2, w)

def blendMatrices(m1, m2, w):
    m1 = om.MMatrix(m1)
    m2 = om.MMatrix(m2)

    q1 = om.MTransformationMatrix(mscaled(m1)).rotation().asQuaternion()
    q2 = om.MTransformationMatrix(mscaled(m2)).rotation().asQuaternion()

    s = mscale(m1) * (1-w) + mscale(m2) * w
    m = om.MMatrix(mscaled(slerp(q1, q2, w).asMatrix(), s))

    set_maxis(m, 3, maxis(m1, 3)*(1-w) + maxis(m2, 3)*w)
    return m

def getLocalMatrix(joint):
    '''
    Get joint local matrix: t, r*jo, s
    '''
    q = om.MQuaternion(joint.getRotation().asQuaternion())

    if isinstance(joint, pm.nt.Joint):
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

def getDelta(m, bm): # get delta matrix from pose world matrix
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

def applyDelta(dm, bm):
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

def symmat(m): # flip x axis
    out = om.MMatrix(m)
    out[0] *= -1
    out[4] *= -1
    out[8] *= -1
    out[12] *= -1
    return out

def parentConstraintMatrix(destBase, srcBase, src):
    return destBase * srcBase.inverse() * src

def mirrorMatrix(base, srcBase, src):
    return parentConstraintMatrix(base, symmat(srcBase), symmat(src))

def mirrorMatrixByDelta(srcBase, src, destBase):
    mirroredSrcBase = mirrorMatrix(om.MMatrix(), om.MMatrix(), srcBase)
    mirroredSrc = mirrorMatrix(om.MMatrix(), om.MMatrix(), src)

    # set translation the same, used for rotation
    dt = maxis(mirroredSrcBase,3) - maxis(destBase,3)
    set_maxis(mirroredSrc, 3, maxis(mirroredSrc, 3) - dt)
    set_maxis(mirroredSrcBase, 3, maxis(mirroredSrcBase, 3) - dt)

    return parentConstraintMatrix(destBase, mirroredSrcBase, mirroredSrc)

def dagPose_findIndex(dagPose, j):
    for m in dagPose.members:
        inputs = m.inputs(sh=True)
        if inputs and inputs[0] == j:
            return m.index()

def dagPose_getWorldMatrix(dagPose, j):
    idx = dagPose_findIndex(dagPose, j)
    if idx is not None:
        return om.MMatrix(dagPose.worldMatrix[idx].get())

def dagPose_getParentMatrix(dagPose, j):
    idx = dagPose_findIndex(dagPose, j)
    if idx is not None:
        parent = dagPose.parents[idx].inputs(p=True, sh=True)
        if parent and parent[0] != dagPose.world:
            return om.MMatrix(dagPose.worldMatrix[parent[0].index()].get())
    return om.MMatrix()

def getRemapInputPlug(remap):
    inputs = remap.inputValue.inputs(p=True)
    if inputs:
        inputPlug = inputs[0]
        if pm.objectType(inputPlug.node()) == "unitConversion":
            inputs = inputPlug.node().input.inputs(p=True)
            if inputs:
                return inputs[0]
        else:
            return inputPlug

def getActualWeightInput(plug):
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
    pm.delete([n for n in pm.ls(type="remapValue") if not n.outValue.isConnected() and not n.outColor.isConnected()])

def undoBlock(f):
    def inner(*args,**kwargs):
        pm.undoInfo(ock=True, cn=f.__name__)
        try:
            out = f(*args, **kwargs)
        finally:
            pm.undoInfo(cck=True)
        return out
    return inner

def findTargetIndexByName(blend, name):
    for aw in blend.w:
        if pm.aliasAttr(aw, q=True)==name:
            return aw.index()

def findAvailableTargetIndex(blend):
    idx = 0
    while blend.w[idx].exists():
        idx += 1
    return idx

def getBlendShapeTargetDelta(blendShape, targetIndex):
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

def matchJoint(j, name=None):
    newj = pm.createNode("joint", n=name or j.name())
    pm.xform(newj, ws=True, m=pm.xform(j, q=True, ws=True, m=True))
    newj.setOrientation(newj.getOrientation()*newj.getRotation().asQuaternion()) # freeze
    newj.setRotation([0,0,0])
    return newj

def transferSkin(src, dest):
    for p in src.wm.outputs(p=True, type="skinCluster"):
        dest.wm >> p

        if not dest.hasAttr("lockInfluenceWeights"):
            dest.addAttr("lockInfluenceWeights", at="bool", dv=False)

        dest.lockInfluenceWeights >> p.node().lockWeights[p.index()]
        #p.node().bindPreMatrix[p.index()].set(dest.wim.get())

class Skeleposer(object):
    TrackAttrs = ["t","tx","ty","tz","r","rx","ry","rz","s","sx","sy","sz"]

    def __init__(self, node=None):
        self._editPoseData = {}

        if pm.objExists(node):
            self.node = pm.PyNode(node)
            self.removeEmptyJoints()
        else:
            self.node = pm.createNode("skeleposer", n=node)

        self.addInternalAttributes()

    def addInternalAttributes(self):
        if not self.node.hasAttr("connectionData"):
            self.node.addAttr("connectionData", dt="string")
            self.node.connectionData.set("{}")

        if not self.node.hasAttr("dagPose"):
            self.node.addAttr("dagPose", at="message")

    def findAvailableDirectoryIndex(self):
        idx = 0
        while self.node.directories[idx].exists():
            idx += 1
        return idx

    def findAvailablePoseIndex(self):
        idx = 0
        while self.node.poses[idx].exists():
            idx += 1
        return idx

    def findAvailableJointIndex(self):
        idx = 0
        while self.node.joints[idx].exists() and self.node.joints[idx].isConnected():
            idx += 1
        return idx

    def getJointIndex(self, joint):
        plugs = [p for p in joint.message.outputs(p=True) if p.node() == self.node]
        if plugs:
            return plugs[0].index()

    def getJointByIndex(self, idx):
        if self.node.joints[idx].exists():
            inputs = self.node.joints[idx].inputs()
            if inputs:
                return inputs[0]

    def clearAll(self):
        for a in self.node.joints:
            pm.removeMultiInstance(a, b=True)

        for a in self.node.jointOrients:
            pm.removeMultiInstance(a, b=True)

        for a in self.node.baseMatrices:
            pm.removeMultiInstance(a, b=True)

        for a in self.node.directories:
            pm.removeMultiInstance(a, b=True)

        for a in self.node.poses:
            for aa in a.poseDeltaMatrices:
                pm.removeMultiInstance(aa, b=True)

            pm.removeMultiInstance(a, b=True)

    def resetToBase(self, joints):
        for jnt in joints:
            idx = self.getJointIndex(jnt)
            if idx is not None:
                jnt.setMatrix(self.node.baseMatrices[idx].get())

    def resetDelta(self, poseIndex, joints):
        for j in joints:
            idx = self.getJointIndex(j)
            if idx is not None:
                pm.removeMultiInstance(self.node.poses[poseIndex].poseDeltaMatrices[idx], b=True)
                j.setMatrix(self.node.baseMatrices[idx].get())

    @undoBlock
    def updateBaseMatrices(self):
        for ja in self.node.joints:
            inputs = ja.inputs()
            bm = self.node.baseMatrices[ja.index()]
            if inputs and bm.isSettable():
                bm.set(getLocalMatrix(inputs[0]))
            else:
                pm.warning("updateBaseMatrices: %s is not writable. Skipped"%bm.name())

        self.updateDagPose()

    def makeCorrectNode(self, drivenIndex, driverIndexList):
        c = pm.createNode("combinationShape", n=self.node.name()+"_"+str(drivenIndex)+"_combinationShape")
        c.combinationMethod.set(1) # lowest weighting
        for i, idx in enumerate(driverIndexList):
            self.node.poses[idx].poseWeight >> c.inputWeight[i]
        c.outputWeight >> self.node.poses[drivenIndex].poseWeight
        return c

    def makeInbetweenNode(self, drivenIndex, driverIndex):
        rv = pm.createNode("remapValue", n=self.node.name()+"_"+str(drivenIndex)+"_remapValue")
        self.node.poses[driverIndex].poseWeight >> rv.inputValue
        rv.outValue >> self.node.poses[drivenIndex].poseWeight
        return rv

    def addJoints(self, joints):
        for j in joints:
            if self.getJointIndex(j) is None:
                idx = self.findAvailableJointIndex()
                j.message >> self.node.joints[idx]

                if isinstance(j, pm.nt.Joint):
                    j.jo >> self.node.jointOrients[idx]
                else:
                    self.node.jointOrients[idx].set([0,0,0])

                self.node.baseMatrices[idx].set(getLocalMatrix(j))

                self.node.outputTranslates[idx] >> j.t
                self.node.outputRotates[idx] >> j.r
                self.node.outputScales[idx] >> j.s
            else:
                pm.warning("addJoints: %s is already connected"%j)

        self.updateDagPose()

    def removeJoints(self, joints):
        for jnt in joints:
            idx = self.getJointIndex(jnt)
            if idx is not None:
                self.removeJointByIndex(idx)

                for a in Skeleposer.TrackAttrs:
                    inp = jnt.attr(a).inputs(p=True)
                    if inp:
                        pm.disconnectAttr(inp[0], jnt.attr(a))

        self.updateDagPose()

    def removeJointByIndex(self, jointIndex):
        pm.removeMultiInstance(self.node.joints[jointIndex], b=True)
        pm.removeMultiInstance(self.node.baseMatrices[jointIndex], b=True)
        pm.removeMultiInstance(self.node.jointOrients[jointIndex], b=True)

        # remove joint's matrices in all poses
        for p in self.node.poses:
            for m in p.poseDeltaMatrices:
                if m.index() == jointIndex:
                    pm.removeMultiInstance(m, b=True)
                    break

    def removeEmptyJoints(self):
        for ja in self.node.joints:
            inputs = ja.inputs()
            if not inputs:
                self.removeJointByIndex(ja.index())
                pm.warning("removeEmptyJoints: removing %s as empty"%ja.name())

    def updateDagPose(self):
        if self.node.dagPose.inputs():
            pm.delete(self.node.dagPose.inputs())

        joints = self.getJoints()
        if joints:
            dp = pm.dagPose(joints, s=True, sl=True, n=self.node.name()+"_world_dagPose")
            dp.message >> self.node.dagPose
        else:
            pm.warning("updateDagPose: no joints found attached")

    def getJoints(self):
        joints = []
        for ja in self.node.joints:
            inputs = ja.inputs(type=["joint", "transform"])
            if inputs:
                joints.append(inputs[0])
            else:
                pm.warning("getJoints: %s is not connected"%ja.name())
        return joints

    def getPoseJoints(self, poseIndex):
        joints = []
        for m in self.node.poses[poseIndex].poseDeltaMatrices:
            ja = self.node.joints[m.index()]
            inputs = ja.inputs()
            if inputs:
                joints.append(inputs[0])
            else:
                pm.warning("getPoseJoints: %s is not connected"%ja.name())
        return joints

    def findPoseIndexByName(self, poseName):
        for p in self.node.poses:
            if p.poseName.get() == poseName:
                return p.index()

    def makePose(self, name):
        idx = self.findAvailablePoseIndex()
        self.node.poses[idx].poseName.set(name)

        indices = self.node.directories[0].directoryChildrenIndices.get() or []
        indices.append(idx)
        self.node.directories[0].directoryChildrenIndices.set(indices, type="Int32Array")
        return idx

    def makeDirectory(self, name, parentIndex=0):
        idx = self.findAvailableDirectoryIndex()
        directory = self.node.directories[idx]
        directory.directoryName.set(name)
        directory.directoryParentIndex.set(parentIndex)

        indices = self.node.directories[parentIndex].directoryChildrenIndices.get() or []
        indices.append(-idx) # negative indices are directories
        self.node.directories[parentIndex].directoryChildrenIndices.set(indices, type="Int32Array")

        return idx

    def removePose(self, poseIndex):
        directoryIndex = self.node.poses[poseIndex].poseDirectoryIndex.get()

        indices = self.node.directories[directoryIndex].directoryChildrenIndices.get() or []
        if poseIndex in indices:
            indices.remove(poseIndex)
        self.node.directories[directoryIndex].directoryChildrenIndices.set(indices, type="Int32Array")

        for m in self.node.poses[poseIndex].poseDeltaMatrices:
            pm.removeMultiInstance(m, b=True)

        pm.removeMultiInstance(self.node.poses[poseIndex], b=True)

    def removeDirectory(self, directoryIndex):
        for ch in self.node.directories[directoryIndex].directoryChildrenIndices.get() or []:
            if ch >= 0:
                self.removePose(ch)
            else:
                self.removeDirectory(-ch)

        parentIndex = self.node.directories[directoryIndex].directoryParentIndex.get()

        indices = self.node.directories[parentIndex].directoryChildrenIndices.get() or []
        if -directoryIndex in indices: # negative indices are directories
            indices.remove(-directoryIndex)
        self.node.directories[parentIndex].directoryChildrenIndices.set(indices, type="Int32Array")

        pm.removeMultiInstance(self.node.directories[directoryIndex], b=True)

    def parentDirectory(self, directoryIndex, newParentIndex, insertIndex=None):
        oldParentIndex = self.node.directories[directoryIndex].directoryParentIndex.get()
        self.node.directories[directoryIndex].directoryParentIndex.set(newParentIndex)

        oldIndices = self.node.directories[oldParentIndex].directoryChildrenIndices.get() or []
        if -directoryIndex in oldIndices: # negative indices are directories
            oldIndices.remove(-directoryIndex)
            self.node.directories[oldParentIndex].directoryChildrenIndices.set(oldIndices, type="Int32Array")

        newIndices = self.node.directories[newParentIndex].directoryChildrenIndices.get() or []
        if insertIndex is None:
            newIndices.append(-directoryIndex)
        else:
            newIndices.insert(insertIndex, -directoryIndex)

        self.node.directories[newParentIndex].directoryChildrenIndices.set(newIndices, type="Int32Array")

    def parentPose(self, poseIndex, newDirectoryIndex, insertIndex=None):
        oldDirectoryIndex = self.node.poses[poseIndex].poseDirectoryIndex.get()
        self.node.poses[poseIndex].poseDirectoryIndex.set(newDirectoryIndex)

        oldIndices = self.node.directories[oldDirectoryIndex].directoryChildrenIndices.get() or []
        if poseIndex in oldIndices:
            oldIndices.remove(poseIndex)
            self.node.directories[oldDirectoryIndex].directoryChildrenIndices.set(oldIndices, type="Int32Array")

        newIndices = self.node.directories[newDirectoryIndex].directoryChildrenIndices.get() or []
        if insertIndex is None:
            newIndices.append(poseIndex)
        else:
            newIndices.insert(insertIndex, poseIndex)

        self.node.directories[newDirectoryIndex].directoryChildrenIndices.set(newIndices, type="Int32Array")

    def dagPose(self):
        dagPoseInputs = self.node.dagPose.inputs(type="dagPose")
        if dagPoseInputs:
            return dagPoseInputs[0]
        else:
            pm.warning("dagPose: no dagPose found attached")

    def removeEmptyDeltas(self, poseIndex):
        for m in self.node.poses[poseIndex].poseDeltaMatrices:
            if m.get().isEquivalent(pm.dt.Matrix(), 1e-4):
                pm.removeMultiInstance(m, b=True)

    def copyPose(self, fromIndex, toIndex, joints=None):
        self.resetDelta(toIndex, joints or self.getPoseJoints(toIndex))

        srcPose = self.node.poses[fromIndex]
        srcBlendMode = srcPose.poseBlendMode.get()

        joints = joints or self.getPoseJoints(fromIndex)
        indices = set([self.getJointIndex(j) for j in joints])

        destPose = self.node.poses[toIndex]
        destPose.poseBlendMode.set(srcBlendMode)

        for mattr in srcPose.poseDeltaMatrices:
            if mattr.index() in indices:
                destPose.poseDeltaMatrices[mattr.index()].set(mattr.get())

    def mirrorPose(self, poseIndex):
        dagPose = self.dagPose()

        blendMode = self.node.poses[poseIndex].poseBlendMode.get()

        joints = sorted(self.getPoseJoints(poseIndex), key=lambda j: len(j.getAllParents())) # sort by parents number, process parents first
        for j in joints:
            idx = self.getJointIndex(j)

            j_mirrored = findSymmetricName(str(j), right=False) # find right side joint
            if not j_mirrored or not cmds.objExists(j_mirrored):
                continue

            j_mirrored = pm.PyNode(j_mirrored)

            mirror_idx = self.getJointIndex(j_mirrored)
            if mirror_idx is None: # if mirror joint is not connected, skip
                continue

            j_mbase = dagPose_getWorldMatrix(dagPose, j) # get base world matrices
            mirrored_mbase = dagPose_getWorldMatrix(dagPose, j_mirrored)

            j_pmat = dagPose_getParentMatrix(dagPose, j)
            mirrored_pmat = dagPose_getParentMatrix(dagPose, j_mirrored)

            delta = self.node.poses[poseIndex].poseDeltaMatrices[idx].get()
            jm = applyDelta(delta, j_mbase) if blendMode == 0 else om.MMatrix(delta) * j_pmat

            mirrored_m = mirrorMatrixByDelta(j_mbase, jm, mirrored_mbase)

            if j == j_mirrored:
                mirrored_m = blendMatrices(jm, mirrored_m, 0.5)

            self.node.poses[poseIndex].poseDeltaMatrices[mirror_idx].set(getDelta(mirrored_m, mirrored_mbase) if blendMode == 0 else mirrored_m * mirrored_pmat.inverse())

    def flipPose(self, poseIndex):
        dagPose = self.dagPose()

        blendMode = self.node.poses[poseIndex].poseBlendMode.get()

        output = {}
        for j in self.getPoseJoints(poseIndex):
            idx = self.getJointIndex(j)

            j_mirrored = findSymmetricName(str(j))
            if not j_mirrored or not cmds.objExists(j_mirrored):
                continue

            j_mirrored = pm.PyNode(j_mirrored)

            mirror_idx = self.getJointIndex(j_mirrored)
            if mirror_idx is None: # if mirror joint is not connected, skip
                continue

            j_mbase = dagPose_getWorldMatrix(dagPose, j)
            mirrored_mbase = dagPose_getWorldMatrix(dagPose, j_mirrored)

            j_pmat = dagPose_getParentMatrix(dagPose, j)
            mirrored_pmat = dagPose_getParentMatrix(dagPose, j_mirrored)

            j_delta = self.node.poses[poseIndex].poseDeltaMatrices[idx].get()
            mirrored_delta = self.node.poses[poseIndex].poseDeltaMatrices[mirror_idx].get()

            jm = applyDelta(j_delta, j_mbase) if blendMode == 0 else om.MMatrix(j_delta) * j_pmat
            mirrored_jm = applyDelta(mirrored_delta, mirrored_mbase) if blendMode == 0 else om.MMatrix(mirrored_delta) * mirrored_pmat

            j_pmat = dagPose_getParentMatrix(dagPose, j)
            mirrored_pmat = dagPose_getParentMatrix(dagPose, j_mirrored)

            m = mirrorMatrixByDelta(mirrored_mbase, mirrored_jm, j_mbase)
            mirrored_m = mirrorMatrixByDelta(j_mbase, jm, mirrored_mbase)

            output[idx] = getDelta(m, j_mbase) if blendMode == 0 else m * j_pmat.inverse()
            output[mirror_idx] = getDelta(mirrored_m, mirrored_mbase) if blendMode == 0 else mirrored_m * mirrored_pmat.inverse()

        for idx in output:
            self.node.poses[poseIndex].poseDeltaMatrices[idx].set(output[idx])

        self.removeEmptyDeltas(poseIndex)

    def changePoseBlendMode(self, poseIndex, blend):
        dagPose = self.dagPose()

        pose = self.node.poses[poseIndex]
        poseBlend = pose.poseBlendMode.get()

        for j in self.getPoseJoints(poseIndex):
            idx = self.getJointIndex(j)

            delta = om.MMatrix(pose.poseDeltaMatrices[idx].get())
            bmat = dagPose_getWorldMatrix(dagPose, j)
            pmat = dagPose_getParentMatrix(dagPose, j)
            wm = applyDelta(delta, bmat) if poseBlend == 0 else delta * pmat
            pose.poseDeltaMatrices[idx].set(getDelta(wm, bmat) if blend == 0 else wm * pmat.inverse())

        pose.poseBlendMode.set(blend)

    @undoBlock
    def disconnectOutputs(self):
        connectionData = json.loads(self.node.connectionData.get())

        if connectionData:
            pm.warning("Disconnection is skipped")
            return

        for ja in self.node.joints:
            j = ja.inputs()[0]

            connections = {}
            for a in Skeleposer.TrackAttrs:
                inp = j.attr(a).inputs(p=True)
                if inp:
                    connections[a] = inp[0].name()
                    pm.disconnectAttr(connections[a], j.attr(a))

            connectionData[ja.index()] = connections

        self.node.connectionData.set(json.dumps(connectionData))

    @undoBlock
    def reconnectOutputs(self):
        connectionData = json.loads(self.node.connectionData.get())

        if not connectionData:
            pm.warning("Connection is skipped")
            return

        for idx in connectionData:
            for a in connectionData[idx]:
                j = self.getJointByIndex(idx)
                pm.connectAttr(connectionData[idx][a], j+"."+a, f=True)

        self.node.connectionData.set("{}")

    @undoBlock
    def beginEditPose(self, idx):
        if self._editPoseData:
            pm.warning("Already in edit mode")
            return

        self._editPoseData = {"joints":{}, "poseIndex":idx, "input": None}

        inputs = self.node.poses[idx].poseWeight.inputs(p=True)
        if inputs:
            pm.disconnectAttr(inputs[0], self.node.poses[idx].poseWeight)
            self._editPoseData["input"] = inputs[0]

        self.node.poses[idx].poseWeight.set(1)

        poseEnabled = self.node.poses[idx].poseEnabled.get()
        self.node.poses[idx].poseEnabled.set(False) # disable pose

        for j in self.getJoints():
            self._editPoseData["joints"][j.name()] = getLocalMatrix(j)

        self.node.poses[idx].poseEnabled.set(poseEnabled) # restore pose state

        self.disconnectOutputs()

    @undoBlock
    def endEditPose(self):
        if not self._editPoseData:
            pm.warning("Not in edit mode")
            return

        pose = self.node.poses[self._editPoseData["poseIndex"]]

        for j in self.getJoints():
            jointIndex = self.getJointIndex(j)

            bmat = self._editPoseData["joints"][j.name()]

            mat = getLocalMatrix(j)
            if not mat.isEquivalent(bmat, 1e-4):
                poseBlendMode = pose.poseBlendMode.get()

                if poseBlendMode == 0: # additive
                    pose.poseDeltaMatrices[jointIndex].set(getDelta(mat, bmat))

                elif poseBlendMode == 1: # replace
                    pose.poseDeltaMatrices[jointIndex].set(mat)

            else:
                pm.removeMultiInstance(pose.poseDeltaMatrices[jointIndex], b=True)

        if self._editPoseData["input"]:
            self._editPoseData["input"] >> pose.poseWeight

        self.reconnectOutputs()
        self._editPoseData = {}

    def findActivePoseIndex(self, value=0.01):
        return [p.index() for p in self.node.poses if p.poseWeight.get() > value]

    def getDirectoryData(self, idx=0):
        data = {"directoryIndex":idx, "children":[]}
        for chIdx in self.node.directories[idx].directoryChildrenIndices.get() or []:
            if chIdx >= 0:
                data["children"].append(chIdx)
            else:
                data["children"].append(self.getDirectoryData(-chIdx))
        return data

    @undoBlock
    def addSplitPose(self, srcPoseName, destPoseName, **kwargs): # addSplitPose("brows_up", "L_brow_up_inner", R_=0, M_=0.5, L_brow_2=0.3, L_brow_3=0, L_brow_4=0)
        srcPose = None
        destPose = None
        for p in self.node.poses:
            if p.poseName.get() == srcPoseName:
                srcPose = p

            if p.poseName.get() == destPoseName:
                destPose = p

        if not srcPose:
            pm.warning("Cannot find source pose: "+srcPoseName)
            return

        if not destPose:
            idx = self.makePose(destPoseName)
            destPose = self.node.poses[idx]

        self.copyPose(srcPose.index(), destPose.index())
        if destPose.poseWeight.isSettable():
            destPose.poseWeight.set(0)

        for j in self.getPoseJoints(destPose.index()):
            j_idx = self.getJointIndex(j)
            bm = self.node.baseMatrices[j_idx].get()

            for pattern in kwargs:
                if re.search(pattern, j.name()):
                    w = kwargs[pattern]
                    pdm = destPose.poseDeltaMatrices[j_idx]
                    if w > 1e-3:
                        pdm.set( blendMatrices(om.MMatrix(), om.MMatrix(pdm.get()), w) )
                    else:
                        pm.removeMultiInstance(pdm, b=True)

    @undoBlock
    def addSplitBlends(self, blendShape, targetName, poses):
        blendShape = pm.PyNode(blendShape)

        targetIndex = findTargetIndexByName(blendShape, targetName)
        if targetIndex is None:
            pm.warning("Cannot find '{}' target in {}".format(targetName, blendShape))
            return

        mesh = blendShape.getOutputGeometry()[0]

        blendShape.envelope.set(0) # turn off blendShapes

        basePoints = pm.api.MPointArray()
        meshFn = pm.api.MFnMesh(mesh.__apimdagpath__())
        meshFn.getPoints(basePoints)

        offsetsList = []
        sumOffsets = [1e-5] * basePoints.length()
        for poseName in poses:
            poseIndex = self.findPoseIndexByName(poseName)
            if poseIndex is not None:
                pose = self.node.poses[poseIndex]

                inputs = pose.poseWeight.inputs(p=True)
                if inputs:
                    pm.disconnectAttr(inputs[0], pose.poseWeight)
                pose.poseWeight.set(1)

                points = pm.api.MPointArray()
                meshFn.getPoints(points)

                offsets = [0]*points.length()
                for i in range(points.length()):
                    offsets[i] = (points[i] - basePoints[i]).length()
                    sumOffsets[i] += offsets[i]**2

                offsetsList.append(offsets)

                if inputs:
                    inputs[0] >> pose.poseWeight
                else:
                    pose.poseWeight.set(0)

            else:
                pm.warning("Cannot find '{}' pose".format(poseName))

        blendShape.envelope.set(1)

        targetGeo = pm.PyNode(pm.sculptTarget(blendShape, e=True, regenerate=True, target=targetIndex)[0])
        targetIndices, targetDeltas = getBlendShapeTargetDelta(blendShape, targetIndex)
        targetComponents = ["vtx[%d]"%v for v in targetIndices]

        targetDeltaList = []
        for poseName in poses: # per pose
            poseTargetIndex = findTargetIndexByName(blendShape, poseName)
            if poseTargetIndex is None:
                poseTargetIndex = findAvailableTargetIndex(blendShape)
                tmp = pm.duplicate(targetGeo)[0]
                tmp.rename(poseName)
                pm.blendShape(blendShape, e=True, t=[mesh, poseTargetIndex, tmp, 1])
                pm.delete(tmp)

            poseTargetDeltas = [pm.dt.Point(p) for p in targetDeltas] # copy delta for each pose target, indices won't be changed
            targetDeltaList.append((poseTargetIndex, poseTargetDeltas))

            poseIndex = self.findPoseIndexByName(poseName)
            if poseIndex is not None:
                self.node.poses[poseIndex].poseWeight >> blendShape.w[poseTargetIndex]

        pm.delete(targetGeo)

        for i, (poseTargetIndex, targetDeltas) in enumerate(targetDeltaList): # i - 0..len(poses)
            for k, idx in enumerate(targetIndices):
                w = offsetsList[i][idx]**2 / sumOffsets[idx]
                targetDeltas[k] *= w

            blendShape.inputTarget[0].inputTargetGroup[poseTargetIndex].inputTargetItem[6000].inputPointsTarget.set(len(targetDeltas), *targetDeltas, type="pointArray")
            blendShape.inputTarget[0].inputTargetGroup[poseTargetIndex].inputTargetItem[6000].inputComponentsTarget.set(len(targetComponents), *targetComponents, type="componentList")

    @undoBlock
    def addJointsAsLayer(self, rootJoint, shouldTransferSkin=True):
        rootJoint = pm.PyNode(rootJoint)
        joints = [rootJoint] + rootJoint.listRelatives(type="joint", ad=True, c=True)

        skelJoints = {j: matchJoint(j) for j in joints}

        # set corresponding parents
        for j in skelJoints:
            parent = j.getParent()
            if parent in skelJoints:
                skelJoints[parent] | skelJoints[j]

        if rootJoint.getParent():
            rootLocalName = rootJoint.name().split("|")[-1]
            grp = pm.createNode("transform", n=rootLocalName + "_parent_transform")
            pm.parentConstraint(rootJoint.getParent(), grp)
            grp | skelJoints[rootJoint]

        self.addJoints(skelJoints.values())

        # set base matrices
        for old, new in skelJoints.items():
            idx = self.getJointIndex(new)
            old.m >> self.node.baseMatrices[idx]

            if shouldTransferSkin:
                transferSkin(old, new)

        # update skin clusters
        if shouldTransferSkin:
            skinClusters = pm.ls(type="skinCluster")
            if skinClusters:
                pm.dgdirty(skinClusters)

        return skelJoints[rootJoint]

    def toJson(self):
        data = {"joints":{}, "baseMatrices":{}, "poses": {}, "directories": {}}

        for j in self.node.joints:
            inputs = j.inputs()
            if inputs:
                data["joints"][j.index()] = inputs[0].name()

        for bm in self.node.baseMatrices:
            a = "{}.baseMatrices[{}]".format(self.node, bm.index())
            data["baseMatrices"][bm.index()] = [shortenValue(v) for v in cmds.getAttr(a)]

        for d in self.node.directories:
            data["directories"][d.index()] = {}
            directoryData = data["directories"][d.index()]

            directoryData["directoryName"] = d.directoryName.get() or ""
            directoryData["directoryWeight"] = d.directoryWeight.get()
            directoryData["directoryParentIndex"] = d.directoryParentIndex.get()
            directoryData["directoryChildrenIndices"] = d.directoryChildrenIndices.get()

        for p in self.node.poses:
            data["poses"][p.index()] = {}
            poseData = data["poses"][p.index()]

            poseData["poseName"] = p.poseName.get()
            poseData["poseWeight"] = p.poseWeight.get()
            poseData["poseDirectoryIndex"] = p.poseDirectoryIndex.get()
            poseData["poseBlendMode"] = p.poseBlendMode.get()

            # corrects
            poseWeightInputs = p.poseWeight.inputs(type="combinationShape")
            if poseWeightInputs:
                combinationShapeNode = poseWeightInputs[0]
                poseData["corrects"] = [iw.getParent().index() for iw in combinationShapeNode.inputWeight.inputs(p=True) if iw.getParent()]

            # inbetween
            poseWeightInputs = p.poseWeight.inputs(type="remapValue")
            if poseWeightInputs:
                remapNode = poseWeightInputs[0]
                inputValueInputs = remapNode.inputValue.inputs(p=True)
                if inputValueInputs and inputValueInputs[0].getParent() and inputValueInputs[0].node() == self.node:
                    sourcePoseIndex = inputValueInputs[0].getParent().index()

                    points = []
                    for va in remapNode.value:
                        x, y, _ = va.get()
                        points.append((x,y))
                    points = sorted(points, key=lambda p: p[0]) # sort by X
                    poseData["inbetween"] = [sourcePoseIndex, points]

            poseData["poseDeltaMatrices"] = {}

            for m in p.poseDeltaMatrices:
                a = "{}.poses[{}].poseDeltaMatrices[{}]".format(self.node, p.index(), m.index())
                poseData["poseDeltaMatrices"][m.index()] = [shortenValue(v) for v in cmds.getAttr(a)]

        return data

    def fromJson(self, data):
        self.clearAll()

        for idx in data["joints"]:
            j = data["joints"][idx]
            if pm.objExists(j):
                j = pm.PyNode(j)
                j.message >> self.node.joints[idx]
                j.jo >> self.node.jointOrients[idx]
            else:
                pm.warning("fromJson: cannot find "+j)

        for idx, m in data["baseMatrices"].items():
            a = "{}.baseMatrices[{}]".format(self.node, idx)
            cmds.setAttr(a, m, type="matrix")

        for idx, d in data["directories"].items():
            a = self.node.directories[idx]
            a.directoryName.set(str(d["directoryName"]))
            a.directoryWeight.set(d["directoryWeight"])
            a.directoryParentIndex.set(d["directoryParentIndex"])
            a.directoryChildrenIndices.set(d["directoryChildrenIndices"], type="Int32Array")

        for idx, p in data["poses"].items():
            a = self.node.poses[idx]
            a.poseName.set(str(p["poseName"]))
            a.poseWeight.set(p["poseWeight"])
            a.poseDirectoryIndex.set(p["poseDirectoryIndex"])
            a.poseBlendMode.set(p["poseBlendMode"])

            for m_idx, m in p["poseDeltaMatrices"].items():
                a = "{}.poses[{}].poseDeltaMatrices[{}]".format(self.node, idx, m_idx)
                cmds.setAttr(a, m, type="matrix")

            if "corrects" in p: # when corrects found
                self.makeCorrectNode(idx, p["corrects"])

            if "inbetween" in p: # setup inbetween
                sourcePoseIndex, points = p["inbetween"]
                remapValue = self.makeInbetweenNode(idx, sourcePoseIndex)
                for i, pnt in enumerate(points):
                    remapValue.value[i].set(pnt[0], pnt[1], 1) # linear interpolation

    def getWorldPoses(self, joints=None):
        dagPose = self.dagPose()

        # cache joints matrices
        jointsData = {}
        for j in self.getJoints():
            idx = self.getJointIndex(j)

            bmat = dagPose_getWorldMatrix(dagPose, j)
            pmat = dagPose_getParentMatrix(dagPose, j)
            jointsData[idx] = {"joint":j, "baseMatrix":bmat, "parentMatrix":pmat}

        data = {}
        for pose in self.node.poses:
            blendMode = pose.poseBlendMode.get()

            deltas = {}
            for delta in pose.poseDeltaMatrices:
                jdata = jointsData[delta.index()]

                if not joints or jdata["joint"] in joints:
                    dm = delta.get()
                    wm = applyDelta(dm, jdata["baseMatrix"]) if blendMode == 0 else om.MMatrix(dm) * jdata["parentMatrix"]
                    deltas[delta.index()] = wm.tolist()

            if deltas:
                data[pose.index()] = deltas

        return data

    @undoBlock
    def setWorldPoses(self, poses):
        dagPose = self.dagPose()

        # cache joints matrices
        jointsData = {}
        for j in self.getJoints():
            idx = self.getJointIndex(j)

            bmat = dagPose_getWorldMatrix(dagPose, j)
            pmat = dagPose_getParentMatrix(dagPose, j)
            jointsData[idx] = {"joint":j, "baseMatrix":bmat, "parentInverseMatrix":pmat.inverse()}

        for pi in poses:
            blendMode = self.node.poses[pi].poseBlendMode.get()

            for di in poses[pi]:
                jdata = jointsData[di]
                delta = getDelta(poses[pi][di], jdata["baseMatrix"]) if blendMode == 0 else om.MMatrix(poses[pi][di]) * jdata["parentInverseMatrix"]
                self.node.poses[pi].poseDeltaMatrices[di].set(delta)

####################################################################################

@undoBlock
def editButtonClicked(btn, item):
    global editPoseIndex

    w = skel.node.poses[item.poseIndex].poseWeight.get()

    if editPoseIndex is None:
        skel.beginEditPose(item.poseIndex)
        btn.setStyleSheet("background-color: #aaaa55")
        mainWindow.toolsWidget.show()

        editPoseIndex = item.poseIndex

    elif editPoseIndex == item.poseIndex:
        skel.endEditPose()
        btn.setStyleSheet("")
        mainWindow.toolsWidget.hide()

        editPoseIndex = None

def setItemWidgets(item):
    tw = item.treeWidget()

    if item.directoryIndex is not None:
        attrWidget = getQWidgetFromMelControl(cmds.attrFieldSliderGrp(at=skel.node.directories[item.directoryIndex].directoryWeight.name(), min=0, max=1, l="", pre=2, cw3=[0,40,100]))
        attrWidget.children()[3].setStyleSheet("background-color: #333333; border: 1px solid #555555")
        tw.setItemWidget(item, 1, attrWidget)

    elif item.poseIndex is not None:
        attrWidget = getQWidgetFromMelControl(cmds.attrFieldSliderGrp(at=skel.node.poses[item.poseIndex].poseWeight.name(),min=0, max=2, smn=0, smx=1, l="", pre=2, cw3=[0,40,100]))
        attrWidget.children()[3].setStyleSheet("background-color: #333333; border: 1px solid #555555")
        tw.setItemWidget(item, 1, attrWidget)

        editBtn = QPushButton("Edit", parent=tw)
        editBtn.setFixedWidth(50)
        editBtn.clicked.connect(lambda btn=editBtn, item=item: editButtonClicked(btn, item))
        tw.setItemWidget(item, 2, editBtn)

        driver = getActualWeightInput(skel.node.poses[item.poseIndex].poseWeight)
        if driver:
            if pm.objectType(driver) == "combinationShape":
                names = [p.parent().poseName.get() for p in driver.node().inputWeight.inputs(p=True, type="skeleposer")]
                label = "correct: " + ", ".join(names)

            elif pm.objectType(driver) == "skeleposer":
                if driver.longName().endswith(".poseWeight"):
                    label = "inbetween: "+driver.parent().poseName.get()
                else:
                    label = driver.longName()
            else:
                label = driver.name()
        else:
            label = ""

        changeDriverBtn = ChangeButtonWidget(item, label, parent=tw)
        tw.setItemWidget(item, 3, changeDriverBtn)

def getAllParents(item):
    allParents = []

    parent = item.parent()
    if parent:
        allParents.append(parent)
        allParents += getAllParents(parent)

    return allParents[::-1]

def centerWindow(w):
    # center the window on the screen
    qr = w.frameGeometry()
    cp = QDesktopWidget().availableGeometry().center()
    qr.moveCenter(cp)
    w.move(qr.topLeft())

def updateItemVisuals(item):
    if item.poseIndex is not None:
        enabled = skel.node.poses[item.poseIndex].poseEnabled.get()
        blendMode = skel.node.poses[item.poseIndex].poseBlendMode.get()
        if blendMode == 0: # relative
            item.setForeground(0, QColor(200, 200, 200) if enabled else QColor(110, 110,110))

        elif blendMode == 1: # replace
            item.setForeground(0, QColor(128,128,255) if enabled else QColor(110, 110, 110))

        font = item.font(0)
        font.setStrikeOut(False if enabled else True)
        item.setFont(0,font)

    elif item.directoryIndex is not None:
        font = item.font(0)
        font.setBold(True)
        item.setFont(0,font)

def makePoseItem(poseIndex):
    item = QTreeWidgetItem([skel.node.poses[poseIndex].poseName.get() or ""])
    item.setIcon(0, QIcon(RootDirectory+"/icons/pose.png"))
    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEditable | Qt.ItemIsEnabled | Qt.ItemIsDragEnabled)
    item.setToolTip(0, ".poses[%d]"%poseIndex)
    item.poseIndex = poseIndex
    item.directoryIndex = None

    updateItemVisuals(item)
    return item

def makeDirectoryItem(directoryIndex):
    item = QTreeWidgetItem([skel.node.directories[directoryIndex].directoryName.get() or ""])
    item.setIcon(0, QIcon(RootDirectory+"/icons/directory.png"))
    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEditable | Qt.ItemIsEnabled | Qt.ItemIsDragEnabled | Qt.ItemIsDropEnabled)
    item.setToolTip(0, ".directories[%d]"%directoryIndex)
    item.poseIndex = None
    item.directoryIndex = directoryIndex

    updateItemVisuals(item)
    return item

class ChangeButtonWidget(QWidget):
    def __init__(self, item, label=" ", **kwargs):
        super(ChangeButtonWidget, self).__init__(**kwargs)

        self.item = item

        layout = QHBoxLayout()
        layout.setContentsMargins(5,0,0,0)
        self.setLayout(layout)

        self.labelWidget = QLabel(label)

        changeBtn = QPushButton("Change")
        changeBtn.clicked.connect(self.changeDriver)

        layout.addWidget(changeBtn)
        layout.addWidget(self.labelWidget)
        layout.addStretch()

    def changeDriver(self):
        driver = getActualWeightInput(skel.node.poses[self.item.poseIndex].poseWeight)

        remapNode = skel.node.poses[self.item.poseIndex].poseWeight.inputs(type="remapValue")
        limit = remapNode[0].inputMax.get() if remapNode else 1

        changeDialog = ChangeDriverDialog(driver, limit, parent=mayaMainWindow)
        changeDialog.accepted.connect(self.updateDriver)
        changeDialog.cleared.connect(self.clearDriver)
        changeDialog.show()

    def clearDriver(self):
        inputs = skel.node.poses[self.item.poseIndex].poseWeight.inputs(p=True)
        if inputs:
            driver = inputs[0]

            if pm.objectType(driver.node()) in ["remapValue", "unitConversion", "combinationShape"]:
                pm.delete(driver.node())
            else:
                pm.disconnectAttr(driver, skel.node.poses[self.item.poseIndex].poseWeight)

        self.labelWidget.setText("")

    def updateDriver(self, newDriver):
        self.clearDriver()
        newDriver >> skel.node.poses[self.item.poseIndex].poseWeight
        self.labelWidget.setText(getActualWeightInput(skel.node.poses[self.item.poseIndex].poseWeight).name())

class SearchReplaceWindow(QDialog):
    replaceClicked = Signal(str, str)

    def __init__(self, **kwargs):
        super(SearchReplaceWindow, self).__init__(**kwargs)
        self.setWindowTitle("Search/Replace")
        layout = QGridLayout()
        layout.setDefaultPositioning(2, Qt.Horizontal)
        self.setLayout(layout)

        self.searchWidget = QLineEdit("L_")
        self.replaceWidget = QLineEdit("R_")

        btn = QPushButton("Replace")
        btn.clicked.connect(self.btnClicked)

        layout.addWidget(QLabel("Search"))
        layout.addWidget(self.searchWidget)
        layout.addWidget(QLabel("Replace"))
        layout.addWidget(self.replaceWidget)
        layout.addWidget(QLabel(""))
        layout.addWidget(btn)

    def btnClicked(self):
        self.replaceClicked.emit(self.searchWidget.text(), self.replaceWidget.text())
        self.accept()

class TreeWidget(QTreeWidget):
    def __init__(self, **kwargs):
        super(TreeWidget, self).__init__(**kwargs)

        self.clipboard = []

        self.searchWindow = SearchReplaceWindow(parent=self)
        self.searchWindow.replaceClicked.connect(self.searchAndReplace)

        self.setHeaderLabels(["Name", "Value", "Edit", "Driver"])
        self.header().setSectionResizeMode(QHeaderView.ResizeToContents) # Qt5

        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.setDragEnabled(True)
        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setDropIndicatorShown(True)
        self.setAcceptDrops(True)

        self.itemChanged.connect(lambda item, idx=None:self.treeItemChanged(item))

    def checkSkeleposer(self):
        if not skel or not skel.node.exists():
            pm.warning("Select skeleposer node")
            return False
        return True

    def addItemsFromSkeleposerData(self, parentItem, skelData):
        for ch in skelData["children"]:
            if isinstance(ch, dict):
                item = makeDirectoryItem(ch["directoryIndex"])
                parentItem.addChild(item)
                self.addItemsFromSkeleposerData(item, ch)

            else:
                item = makePoseItem(ch)
                parentItem.addChild(item)

    def updateTree(self):
        self.clear()
        self.addItemsFromSkeleposerData(self.invisibleRootItem(), skel.getDirectoryData())
        for ch in self.getChildrenRecursively(self.invisibleRootItem()):
            setItemWidgets(ch)

    def getChildrenRecursively(self, item, pose=True, directory=True):
        children = []
        for i in range(item.childCount()):
            ch = item.child(i)

            if ch.poseIndex is not None and not pose:
                continue

            if ch.directoryIndex is not None and not directory:
                continue

            children.append(ch)
            children += self.getChildrenRecursively(ch, pose, directory)

        return children
    
    def getValidParent(self):
        selectedItems = self.selectedItems()
        if selectedItems:
            last = selectedItems[-1]
            return last if last.directoryIndex is not None else last.parent()
            
    @contextmanager
    def keepState(self):
        selectedIndices = [] # poses > 0, directories < 0
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                selectedIndices.append(sel.poseIndex)
            elif sel.directoryIndex is not None:
                selectedIndices.append(-sel.directoryIndex)

        expanded = {}
        for ch in self.getChildrenRecursively(self.invisibleRootItem(), pose=False):
            expanded[ch.directoryIndex] = ch.isExpanded()

        yield

        for ch in self.getChildrenRecursively(self.invisibleRootItem()):
            setItemWidgets(ch)

            if ch.directoryIndex in expanded:
                ch.setExpanded(expanded[ch.directoryIndex])

            if (ch.poseIndex is not None and ch.poseIndex in selectedIndices) or\
            (ch.directoryIndex is not None and -ch.directoryIndex in selectedIndices):
                ch.setSelected(True)

    def loadSkeleposer(self):
        if not self.checkSkeleposer():
            return
        
        path, _ = QFileDialog.getOpenFileName(self, "Import skeleposer", "", "*.json")
        if path:
            with open(path, "r") as f:
                data = json.load(f)
            skel.fromJson(data)
            self.updateTree()

    def saveSkeleposer(self):
        if not self.checkSkeleposer():
            return

        path, _ = QFileDialog.getSaveFileName(self, "Export skeleposer", "", "*.json")
        if path:
            with open(path, "w") as f:
                json.dump(skel.toJson(), f)

    @undoBlock
    def muteItems(self):
        if not self.checkSkeleposer():
            return
                
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                a = skel.node.poses[sel.poseIndex].poseEnabled
                a.set(not a.get())
                updateItemVisuals(sel)

    def searchAndReplace(self, searchText, replaceText):
        if not self.checkSkeleposer():
            return
                
        for sel in self.selectedItems():
            sel.setText(0, sel.text(0).replace(searchText, replaceText))

    @undoBlock
    def addInbetweenPose(self):
        if not self.checkSkeleposer():
            return
                
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                item = self.makePose(sel.text(0)+"_inbtw", self.getValidParent())
                skel.makeInbetweenNode(item.poseIndex, sel.poseIndex)
                setItemWidgets(item)

    @undoBlock
    def setPoseBlendMode(self, blend):
        if not self.checkSkeleposer():
            return
                
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                skel.changePoseBlendMode(sel.poseIndex, blend)
                updateItemVisuals(sel)

    def collapseOthers(self):
        selectedItems = self.selectedItems()
        if not selectedItems:
            return

        allParents = []
        for sel in selectedItems:
            allParents += getAllParents(sel)

        allParents = set(allParents)
        for ch in self.getChildrenRecursively(self.invisibleRootItem()):
            if ch not in allParents:
                ch.setExpanded(False)

    @undoBlock
    def groupSelected(self):
        if not self.checkSkeleposer():
            return
                
        dirItem = self.makeDirectory(parent=self.getValidParent())

        for sel in self.selectedItems():
            (sel.parent() or self.invisibleRootItem()).removeChild(sel)
            dirItem.addChild(sel)
            self.treeItemChanged(sel)

        dirItem.setSelected(True)

    def copyPoseJointsDelta(self, joints=None):
        if not self.checkSkeleposer():
            return
                
        currentItem = self.currentItem()
        if currentItem and currentItem.poseIndex is not None:
            self.clipboard = {"poseIndex": currentItem.poseIndex, "joints":joints}

    @undoBlock
    def pastePoseDelta(self):
        if not self.checkSkeleposer():
            return
                
        if self.clipboard:
            currentItem = self.currentItem()
            if currentItem and currentItem.poseIndex is not None:
                skel.copyPose(self.clipboard["poseIndex"], currentItem.poseIndex, self.clipboard["joints"])

    @undoBlock
    def flipItemsOnOppositePose(self, items=None):
        if not self.checkSkeleposer():
            return
                
        selectedItems = self.selectedItems()
        if not selectedItems and not items:
            return

        doUpdateUI = False

        for sel in items or selectedItems:
            if sel.poseIndex is not None:
                sourcePoseIndex = sel.poseIndex
                sourcePoseName = sel.text(0)

                destPoseName = findSymmetricName(sourcePoseName)
                if destPoseName != sourcePoseName:
                    destPoseIndex = skel.findPoseIndexByName(destPoseName)
                    if not destPoseIndex:
                        destPoseIndex = skel.makePose(destPoseName)
                        doUpdateUI = True

                    skel.copyPose(sourcePoseIndex, destPoseIndex)
                    skel.flipPose(destPoseIndex)

            elif sel.directoryIndex is not None:
                self.flipItemsOnOppositePose(self.getChildrenRecursively(sel))

        if doUpdateUI:
            mainWindow.treeWidget.updateTree()

    @undoBlock
    def mirrorItems(self, items=None):
        if not self.checkSkeleposer():
            return
                
        for sel in items or self.selectedItems():
            if sel.poseIndex is not None:
                skel.mirrorPose(sel.poseIndex)

            elif sel.directoryIndex is not None:
                self.mirrorItems(self.getChildrenRecursively(sel))

    @undoBlock
    def flipItems(self, items=None):
        if not self.checkSkeleposer():
            return
                
        for sel in items or self.selectedItems():
            if sel.poseIndex is not None:
                skel.flipPose(sel.poseIndex)

            elif sel.directoryIndex is not None:
                self.flipItems(self.getChildrenRecursively(sel))

    @undoBlock
    def resetWeights(self):
        if not self.checkSkeleposer():
            return
                
        for p in skel.node.poses:
            if p.poseWeight.isSettable():
                p.poseWeight.set(0)

    @undoBlock
    def resetJoints(self):
        if not self.checkSkeleposer():
            return
                
        joints = pm.ls(sl=True, type=["joint", "transform"])
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                skel.resetDelta(sel.poseIndex, joints)

    @undoBlock
    def duplicateItems(self, items=None, parent=None):
        if not self.checkSkeleposer():
            return
                
        parent = parent or self.getValidParent()
        for item in items or self.selectedItems():
            if item.poseIndex is not None:
                newItem = self.makePose(item.text(0), parent)
                skel.copyPose(item.poseIndex, newItem.poseIndex)

            elif item.directoryIndex is not None:
                newItem = self.makeDirectory(item.text(0), parent)

                for i in range(item.childCount()):
                    self.duplicateItems([item.child(i)], newItem)

    @undoBlock
    def setupWeightFromSelection(self):
        if not self.checkSkeleposer():
            return
                
        currentItem = self.currentItem()
        if currentItem and currentItem.poseIndex is not None:
            indices = [item.poseIndex for item in self.selectedItems() if item.poseIndex is not None and item is not currentItem]
            skel.makeCorrectNode(currentItem.poseIndex, indices)
            setItemWidgets(currentItem)

    @undoBlock
    def createInbetweenFromSelection(self):
        if not self.checkSkeleposer():
            return
                
        currentItem = self.currentItem()
        if currentItem and currentItem.poseIndex is not None:
            indices = [item.poseIndex for item in self.selectedItems() if item.poseIndex is not None and item is not currentItem]
            skel.makeInbetweenNode(currentItem.poseIndex, indices[-1])
            setItemWidgets(currentItem)

    @undoBlock
    def addCorrectivePose(self):
        if not self.checkSkeleposer():
            return
                
        selectedItems = self.selectedItems()
        if selectedItems:
            indices = [item.poseIndex for item in selectedItems if item.poseIndex is not None]
            names = [item.text(0) for item in selectedItems if item.poseIndex is not None]

            item = self.makePose("_".join(names)+"_correct", self.getValidParent())
            skel.makeCorrectNode(item.poseIndex, indices)
            setItemWidgets(item)

    @undoBlock
    def removeItems(self):
        if not self.checkSkeleposer():
            return

        for item in self.selectedItems():
            if item.directoryIndex is not None: # remove directory
                skel.removeDirectory(item.directoryIndex)
                (item.parent() or self.invisibleRootItem()).removeChild(item)

            elif item.poseIndex is not None:
                skel.removePose(item.poseIndex)
                (item.parent() or self.invisibleRootItem()).removeChild(item)

    def makePose(self, name="Pose", parent=None):
        if not self.checkSkeleposer():
            return

        idx = skel.makePose(name)
        item = makePoseItem(idx)

        if parent:
            parent.addChild(item)
            skel.parentPose(idx, parent.directoryIndex)
        else:
            self.invisibleRootItem().addChild(item)

        setItemWidgets(item)
        return item

    def makeDirectory(self, name="Group", parent=None):
        if not self.checkSkeleposer():
            return
                
        parentIndex = parent.directoryIndex if parent else 0
        idx = skel.makeDirectory(name, parentIndex)

        item = makeDirectoryItem(idx)
        (parent or self.invisibleRootItem()).addChild(item)

        setItemWidgets(item)
        return item
    
    @undoBlock
    def addJoints(self):
        if not self.checkSkeleposer():
            return
                
        ls = pm.ls(sl=True, type=["joint", "transform"])
        if ls:
            skel.addJoints(ls)
        else:
            pm.warning("Select joints to add")

    @undoBlock
    def removeJoints(self):
        if not self.checkSkeleposer():
            return
        
        ls = pm.ls(sl=True, type=["joint", "transform"])
        if ls:
            skel.removeJoints(ls)
        else:
            pm.warning("Select joints to remove")    

    def addJointsAsLayer(self):
        if not self.checkSkeleposer():
            return
                
        ls = pm.ls(sl=True, type=["joint", "transform"])
        if ls:
            skel.addJointsAsLayer(ls[0])
        else:
            pm.warning("Select root joint to add as a layer")                
    
    def reconnectOutputs(self):
        if not self.checkSkeleposer():
            return
                
        skel.reconnectOutputs()

    def disconnectOutputs(self):
        if not self.checkSkeleposer():
            return
                
        skel.disconnectOutputs()

    def updateBaseMatrices(self):
        if not self.checkSkeleposer():
            return
                
        skel.updateBaseMatrices()

    @undoBlock
    def treeItemChanged(self, item):
        if item.directoryIndex is not None: # directory
            skel.node.directories[item.directoryIndex].directoryName.set(item.text(0).strip())

            parent = item.parent()
            realParent = parent or self.invisibleRootItem()
            skel.parentDirectory(item.directoryIndex, parent.directoryIndex if parent else 0, realParent.indexOfChild(item))

        elif item.poseIndex is not None:
            skel.node.poses[item.poseIndex].poseName.set(item.text(0).strip())

            parent = item.parent()
            realParent = parent or self.invisibleRootItem()
            skel.parentPose(item.poseIndex, parent.directoryIndex if parent else 0, realParent.indexOfChild(item))

        setItemWidgets(item)

    def dragEnterEvent(self, event):
        if event.mouseButtons() == Qt.MiddleButton:
            QTreeWidget.dragEnterEvent(self, event)
            self.dragItems = self.selectedItems()

    def dragMoveEvent(self, event):
        QTreeWidget.dragMoveEvent(self, event)

    @undoBlock
    def dropEvent(self, event):
        QTreeWidget.dropEvent(self, event)

        for item in sorted(self.dragItems, key=lambda x: -(x.parent() or self.invisibleRootItem()).indexOfChild(x)): # greater index first
            self.treeItemChanged(item)

            if item.directoryIndex is not None: # update widgets for all children
                for ch in self.getChildrenRecursively(item):
                    setItemWidgets(ch)

class BlendSliderWidget(QWidget):
    valueChanged = Signal(float)

    def __init__(self, **kwargs):
        super(BlendSliderWidget, self).__init__(**kwargs)

        layout = QHBoxLayout()
        layout.setMargin(0)
        self.setLayout(layout)

        self.textWidget = QLineEdit("1")
        self.textWidget.setFixedWidth(40)
        self.textWidget.setValidator(QDoubleValidator())
        self.textWidget.editingFinished.connect(self.textChanged)

        self.sliderWidget = QSlider(Qt.Horizontal)
        self.sliderWidget.setValue(100)
        self.sliderWidget.setMinimum(0)
        self.sliderWidget.setMaximum(100)
        self.sliderWidget.setTracking(True)
        self.sliderWidget.sliderReleased.connect(self.sliderValueChanged)

        layout.addWidget(self.textWidget)
        layout.addWidget(self.sliderWidget)
        layout.addStretch()

    def textChanged(self):
        value = float(self.textWidget.text())
        self.sliderWidget.setValue(value*100)
        self.valueChanged.emit(value)

    def sliderValueChanged(self):
        value = self.sliderWidget.value()/100.0
        self.textWidget.setText(str(value))
        self.valueChanged.emit(value)

class ToolsWidget(QWidget):
    def __init__(self, **kwargs):
        super(ToolsWidget, self).__init__(**kwargs)

        layout = QHBoxLayout()
        layout.setMargin(0)
        self.setLayout(layout)

        mirrorJointsBtn = QToolButton()
        mirrorJointsBtn.setToolTip("Mirror joints")
        mirrorJointsBtn.setAutoRaise(True)
        mirrorJointsBtn.clicked.connect(self.mirrorJoints)
        mirrorJointsBtn.setIcon(QIcon(RootDirectory+"/icons/mirror.png"))

        resetJointsBtn = QToolButton()
        resetJointsBtn.setToolTip("Reset to default")
        resetJointsBtn.setAutoRaise(True)
        resetJointsBtn.clicked.connect(lambda: skel.resetToBase(pm.ls(sl=True, type=["joint", "transform"])))
        resetJointsBtn.setIcon(QIcon(RootDirectory+"/icons/reset.png"))

        self.blendSliderWidget = BlendSliderWidget()
        self.blendSliderWidget.valueChanged.connect(self.blendValueChanged)

        layout.addWidget(mirrorJointsBtn)
        layout.addWidget(resetJointsBtn)
        layout.addWidget(self.blendSliderWidget)
        layout.addStretch()

    @undoBlock
    def blendValueChanged(self, v):
        for j in pm.ls(sl=True, type="transform"):
            if j in self.matrices:
                bm, m = self.matrices[j]
                j.setMatrix(blendMatrices(bm, m, v))

    def showEvent(self, event):
        self.matrices = {}
        poseIndex = skel._editPoseData["poseIndex"]
        for j in skel.getPoseJoints(poseIndex):
            self.matrices[j] = (skel.node.baseMatrices[skel.getJointIndex(j)].get(), getLocalMatrix(j))

    def mirrorJoints(self):
        dagPose = skel.dagPose()

        joints = sorted(pm.ls(sl=True, type=["joint", "transform"]), key=lambda j: len(j.getAllParents())) # sort by parents number, process parents first

        for L_joint in joints:
            R_joint = findSymmetricName(str(L_joint), right=False) # skip right joints
            if not R_joint or not cmds.objExists(R_joint):
                continue

            R_joint = pm.PyNode(R_joint)

            L_m = om.MMatrix(L_joint.wm.get())

            L_base = dagPose_getWorldMatrix(dagPose, L_joint)
            R_base = dagPose_getWorldMatrix(dagPose, R_joint)

            R_m = mirrorMatrixByDelta(L_base, L_m, R_base)
            pm.xform(R_joint, ws=True, m=R_m)

class NodeSelectorWidget(QWidget):
    nodeChanged = Signal(object)

    def __init__(self, **kwargs):
        super(NodeSelectorWidget, self).__init__(**kwargs)

        layout = QHBoxLayout()
        layout.setMargin(0)
        self.setLayout(layout)

        self.lineEditWidget = QLineEdit()
        self.lineEditWidget.editingFinished.connect(lambda: self.nodeChanged.emit(self.getNode()))

        btn = QPushButton("<<")
        btn.setFixedWidth(30)
        btn.clicked.connect(self.getSelectedNode)

        layout.addWidget(self.lineEditWidget)
        layout.addWidget(btn)
        layout.setStretch(0,1)
        layout.setStretch(1,0)

    def getSelectedNode(self):
        ls = pm.ls(sl=True)
        if ls:
            self.lineEditWidget.setText(ls[0].name())
            self.nodeChanged.emit(self.getNode())

    def setNode(self, node):
        self.lineEditWidget.setText(str(node))

    def getNode(self):
        n = self.lineEditWidget.text()
        return pm.PyNode(n) if pm.objExists(n) else ""

class ChangeDriverDialog(QDialog):
    accepted = Signal(object)
    cleared = Signal()

    def __init__(self, plug=None, limit="1", **kwargs):
        super(ChangeDriverDialog, self).__init__(**kwargs)

        self.setWindowTitle("Change driver")

        layout = QVBoxLayout()
        self.setLayout(layout)

        gridLayout = QGridLayout()
        gridLayout.setDefaultPositioning(2, Qt.Horizontal)

        self.nodeWidget = NodeSelectorWidget()
        self.nodeWidget.nodeChanged.connect(self.updateAttributes)

        self.attrsWidget = QComboBox()
        self.attrsWidget.setEditable(True)

        self.limitWidget = QLineEdit(str(limit))
        self.limitWidget.setValidator(QDoubleValidator())

        okBtn = QPushButton("Ok")
        okBtn.clicked.connect(self.createNode)

        clearBtn = QPushButton("Clear")
        clearBtn.clicked.connect(self.clearNode)

        gridLayout.addWidget(QLabel("Node"))
        gridLayout.addWidget(self.nodeWidget)

        gridLayout.addWidget(QLabel("Attribute"))
        gridLayout.addWidget(self.attrsWidget)

        gridLayout.addWidget(QLabel("Limit"))
        gridLayout.addWidget(self.limitWidget)

        hlayout = QHBoxLayout()
        hlayout.addWidget(okBtn)
        hlayout.addWidget(clearBtn)

        layout.addLayout(gridLayout)
        layout.addLayout(hlayout)

        self.updateAttributes()

        if plug:
            self.nodeWidget.setNode(plug.node().name())
            self.attrsWidget.setCurrentText(plug.longName())

    def clearNode(self):
        self.cleared.emit()
        self.close()

    def createNode(self):
        node = self.nodeWidget.getNode()
        attr = self.attrsWidget.currentText()

        if node and pm.objExists(node+"."+attr):
            ls = pm.ls(sl=True)

            limit = float(self.limitWidget.text())
            suffix = "pos" if limit > 0 else "neg"

            n = pm.createNode("remapValue", n=node+"_"+attr+"_"+suffix+"_remapValue")
            node.attr(attr) >> n.inputValue
            n.inputMax.set(limit)
            self.accepted.emit(n.outValue)
            self.accept()

            pm.select(ls)
        else:
            pm.warning("createNode: "+node+"."+attr+" doesn't exist")

    def updateAttributes(self, node=None):
        currentText = self.attrsWidget.currentText()
        self.attrsWidget.clear()
        if node:
            attrs = ["translateX","translateY","translateZ","rotateX","rotateY","rotateZ","scaleX","scaleY","scaleZ"]
            attrs += [a.longName() for a in node.listAttr(s=True, se=True, ud=True)]
            self.attrsWidget.addItems(attrs)
            self.attrsWidget.setCurrentText(currentText)

class WideSplitterHandle(QSplitterHandle):
    def __init__(self, orientation, parent, **kwargs):
        super(WideSplitterHandle, self).__init__(orientation, parent, **kwargs)

    def paintEvent(self, event):
        painter = QPainter(self)
        brush = QBrush()
        brush.setStyle(Qt.Dense6Pattern)
        brush.setColor(QColor(150, 150, 150))
        painter.fillRect(event.rect(), QBrush(brush))

class WideSplitter(QSplitter):
    def __init__(self, orientation, **kwargs):
        super(WideSplitter, self).__init__(orientation, **kwargs)
        self.setHandleWidth(7)

    def createHandle(self):
        return WideSplitterHandle(self.orientation(), self)

class ListWithFilterWidget(QWidget):
    def __init__(self, **kwargs):
        super(ListWithFilterWidget, self).__init__(**kwargs)

        layout = QVBoxLayout()
        layout.setContentsMargins(0,0,0,0)
        self.setLayout(layout)

        self.filterWidget = QLineEdit()
        self.filterWidget.textChanged.connect(self.filterChanged)

        self.listWidget = QListWidget()
        self.listWidget.itemSelectionChanged.connect(self.itemSelectionChanged)
        self.listWidget.setSelectionMode(QAbstractItemView.ExtendedSelection)

        layout.addWidget(self.filterWidget)
        layout.addWidget(self.listWidget)

    def filterChanged(self, text=None):
        tx = re.escape(str(text or self.filterWidget.text()))

        for i in range(self.listWidget.count()):
            item = self.listWidget.item(i)
            b = re.search(tx, str(item.text()))
            self.listWidget.setItemHidden(item, False if b else True)

    def itemSelectionChanged(self):
        pm.select([item.text() for item in self.listWidget.selectedItems()])

    def clearItems(self):
        self.listWidget.clear()

    def addItems(self, items, bold=False, foreground=QColor(200, 200, 200)):
        font = QListWidgetItem().font()
        font.setBold(bold)

        for it in items:
            item = QListWidgetItem(it)
            item.setFont(font)
            item.setForeground(foreground)
            self.listWidget.addItem(item)
        self.filterChanged()

class PoseTreeWidget(QTreeWidget):
    somethingChanged = Signal()

    def __init__(self, **kwargs):
        super(PoseTreeWidget, self).__init__(**kwargs)

        self.setHeaderLabels(["Name"])
        self.header().setSectionResizeMode(QHeaderView.ResizeToContents)

        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.setDragEnabled(True)
        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setDropIndicatorShown(True)
        self.setAcceptDrops(True)

    def contextMenuEvent(self, event):
        if not skel or not skel.node.exists():
            return

        menu = QMenu(self)

        menu.addAction("Add", self.addPoseItem, "Insert")

        if self.selectedItems():
            menu.addAction("Duplicate", self.duplicatePoseItem, "Ctrl-D")

        menu.addAction("Remove", self.removePoseItem, "Delete")
        menu.popup(event.globalPos())

    def keyPressEvent(self, event):
        ctrl = event.modifiers() & Qt.ControlModifier

        if ctrl:
            if event.key() == Qt.Key_D:
                self.duplicatePoseItem()

        elif event.key() == Qt.Key_Insert:
            self.addPoseItem()

        elif event.key() == Qt.Key_Delete:
            self.removePoseItem()

        else:
            super(PoseTreeWidget, self).keyPressEvent(event)

    def dragEnterEvent(self, event):
        if event.mouseButtons() == Qt.MiddleButton:
            QTreeWidget.dragEnterEvent(self, event)

    def dragMoveEvent(self, event):
        QTreeWidget.dragMoveEvent(self, event)

    def dropEvent(self, event):
        QTreeWidget.dropEvent(self, event)
        self.somethingChanged.emit()

    def makePoseItem(self, label="Pose"):
        item = QTreeWidgetItem([label])
        item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEditable | Qt.ItemIsEnabled | Qt.ItemIsDragEnabled  | Qt.ItemIsDropEnabled)
        item.setData(0, Qt.UserRole, []) # patterns, data can be cloned
        return item

    def addPoseItem(self):
        selectedItems = self.selectedItems()
        selectedItem = selectedItems[0] if selectedItems else None

        item = self.makePoseItem()
        (selectedItem or self.invisibleRootItem()).addChild(item)

        self.somethingChanged.emit()

    def duplicatePoseItem(self):
        for item in self.selectedItems():
            (item.parent() or self.invisibleRootItem()).addChild(item.clone())
        self.somethingChanged.emit()

    def removePoseItem(self):
        for item in self.selectedItems():
            (item.parent() or self.invisibleRootItem()).removeChild(item)
        self.somethingChanged.emit()

    def toList(self, item=None): # hierarchy to list like [[a, [b, [c, d]]] => a|bc|d
        out = []

        if not item:
            item = self.invisibleRootItem()
        else:
            value = (item.text(0), item.data(0, Qt.UserRole))
            out.append(value)

        for i in range(item.childCount()):
            ch = item.child(i)
            lst = self.toList(ch)

            if ch.childCount() > 0:
                out.append(lst)
            else:
                out.extend(lst)

        return out

    def fromList(self, data): # [[a, [b, [c, d]]]] => a|bc|d
        def addItems(data, parent=None):
            for ch in data:
                itemLabel = ch[0][0] if isinstance(ch[0], list) else ch[0]
                itemData = ch[0][1] if isinstance(ch[0], list) else ch[1]

                item = self.makePoseItem(itemLabel)
                item.setData(0, Qt.UserRole, itemData)
                (parent or self.invisibleRootItem()).addChild(item)

                if isinstance(ch[0], list): # item with children
                    addItems(ch[1:], item)
                    item.setExpanded(True)

        self.blockSignals(True)
        self.clear()
        addItems(data)
        self.blockSignals(False)

class PatternTableWidget(QTableWidget):
    somethingChanged = Signal()

    def __init__(self, **kwargs):
        super(PatternTableWidget, self).__init__(**kwargs)

        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.setColumnCount(2)
        self.setHorizontalHeaderLabels(["Pattern", "Value"])
        self.verticalHeader().hide()

        self.itemChanged.connect(self.validateItem)

    def contextMenuEvent(self, event):
        menu = QMenu(self)

        menu.addAction("Add", self.addPatternItem, "Insert")

        if self.selectedItems():
            menu.addAction("Duplicate", self.duplicatePatternItem, "Ctrl-D")

        menu.addAction("Remove", self.removePatternItem, "Delete")
        menu.popup(event.globalPos())

    def keyPressEvent(self, event):
        ctrl = event.modifiers() & Qt.ControlModifier

        if ctrl:
            if event.key() == Qt.Key_D:
                self.duplicatePatternItem()

        elif event.key() == Qt.Key_Insert:
            self.addPatternItem()

        elif event.key() == Qt.Key_Delete:
            self.removePatternItem()

        else:
            super(PatternTableWidget, self).keyPressEvent(event)

    def validateItem(self, item):
        self.blockSignals(True)
        if item.column() == 1:
            try:
                v = float(item.text())
            except:
                v = 0
            item.setText(str(clamp(v)))
        self.blockSignals(False)

    def addPatternItem(self, name="R_", value=0):
        row = self.rowCount()
        self.insertRow(row)
        self.setItem(row,0, QTableWidgetItem(name))
        self.setItem(row,1, QTableWidgetItem(str(value)))
        self.somethingChanged.emit()

    def duplicatePatternItem(self):
        for item in self.selectedItems():
            nameItem = self.item(item.row(), 0)
            valueItem = self.item(item.row(), 1)
            if nameItem and valueItem:
                self.addPatternItem(nameItem.text(), valueItem.text())

    def removePatternItem(self):
        for item in self.selectedItems():
            row = item.row()
            self.removeRow(row)
            self.somethingChanged.emit()

    def fromJson(self, data):
        self.blockSignals(True)
        self.clearContents()
        self.setRowCount(0)
        for p in sorted(data):
            self.addPatternItem(p, data[p])
        self.blockSignals(False)

    def toJson(self):
        data = {}
        for i in range(self.rowCount()):
            nameItem = self.item(i, 0)
            if nameItem:
                valueItem = self.item(i, 1)
                data[nameItem.text()] = float(valueItem.text()) if valueItem else 0
        return data

class SplitPoseWidget(QWidget):
    def __init__(self, **kwargs):
        super(SplitPoseWidget, self).__init__(**kwargs)

        layout = QVBoxLayout()
        self.setLayout(layout)

        hsplitter = WideSplitter(Qt.Horizontal)

        self.posesWidget = PoseTreeWidget()
        self.posesWidget.itemSelectionChanged.connect(self.posesSelectionChanged)
        self.posesWidget.itemChanged.connect(lambda _=None:self.patternsItemChanged())
        self.posesWidget.somethingChanged.connect(self.patternsItemChanged)

        self.patternsWidget = PatternTableWidget()
        self.patternsWidget.itemChanged.connect(lambda _=None:self.patternsItemChanged())
        self.patternsWidget.somethingChanged.connect(self.patternsItemChanged)
        self.patternsWidget.setEnabled(False)

        self.blendShapeWidget = QLineEdit()
        getBlendshapeBtn = QPushButton("<<")
        getBlendshapeBtn.clicked.connect(self.getBlendShapeNode)

        blendLayout = QHBoxLayout()
        blendLayout.addWidget(QLabel("Split blend shapes (target names must match pose names)"))
        blendLayout.addWidget(self.blendShapeWidget)
        blendLayout.addWidget(getBlendshapeBtn)

        applyBtn = QPushButton("Apply")
        applyBtn.clicked.connect(self.apply)

        self.applySelectedWidget = QCheckBox("Apply selected")
        applyLayout = QHBoxLayout()
        applyLayout.addWidget(self.applySelectedWidget)
        applyLayout.addWidget(applyBtn)
        applyLayout.setStretch(1, 1)

        hsplitter.addWidget(self.posesWidget)
        hsplitter.addWidget(self.patternsWidget)
        layout.addWidget(hsplitter)
        layout.addLayout(blendLayout)
        layout.addLayout(applyLayout)

    def getBlendShapeNode(self):
        ls = pm.ls(sl=True)
        if ls:
            node = ls[0]
            if isinstance(node, pm.nt.BlendShape):
                self.blendShapeWidget.setText(node.name())
            else:
                blends = [n for n in pm.listHistory(node) if isinstance(n, pm.nt.BlendShape)]
                if blends:
                    self.blendShapeWidget.setText(blends[0].name())

    def posesSelectionChanged(self):
        selectedItems = self.posesWidget.selectedItems()
        self.patternsWidget.setEnabled(True if selectedItems else False)

        for item in selectedItems:
            patterns = item.data(0, Qt.UserRole)
            self.patternsWidget.fromJson(patterns)

    def patternsItemChanged(self):
        # update all patterns
        for item in self.posesWidget.selectedItems():
            data = self.patternsWidget.toJson()
            item.setData(0, Qt.UserRole, data)

        self.saveToSkeleposer()

    @undoBlock
    def apply(self):
        blendShape = self.blendShapeWidget.text()
        applySelected = self.applySelectedWidget.isChecked()

        def splitPoses(item, sourcePose=None):
            for i in range(item.childCount()):
                ch = item.child(i)
                destPose = ch.text(0)

                if sourcePose:
                    data = dict(ch.data(0, Qt.UserRole))
                    print("Split pose '{}'' into '{}' with {}".format(sourcePose, destPose, str(data)))
                    skel.addSplitPose(sourcePose, destPose, **data)

                splitPoses(ch, destPose)

        def splitBlends(item, sourcePose=None):
            children = []
            for i in range(item.childCount()):
                children.append(item.child(i).text(0))

            if sourcePose and children:
                print("Split blend '{}' into '{}'".format(sourcePose, " ".join(children)))
                skel.addSplitBlends(blendShape, sourcePose, children)

            for i in range(item.childCount()):
                ch = item.child(i)
                splitBlends(ch, ch.text(0))

        if not skel or not skel.node.exists():
            return

        if applySelected:
            for item in self.posesWidget.selectedItems():
                sourceItem = item.parent() or item

                sourcePose = sourceItem.text(0)
                splitPoses(sourceItem, sourcePose)
                if pm.objExists(blendShape):
                    splitBlends(sourceItem, sourcePose)
        else:
            rootItem = self.posesWidget.invisibleRootItem()

            splitPoses(rootItem)
            if pm.objExists(blendShape):
                splitBlends(rootItem)

        with mainWindow.treeWidget.keepState():
            mainWindow.treeWidget.updateTree()

    def fromJson(self, data): # [[a, [b, c]]] => a | b | c
        self.posesWidget.fromList(data)
        self.patternsWidget.fromJson([])

    def toJson(self):
        return self.posesWidget.toList()

    def saveToSkeleposer(self):
        if not skel.node.hasAttr("splitPosesData"):
            skel.node.addAttr("splitPosesData", dt="string")
        skel.node.splitPosesData.set(json.dumps(self.toJson()))

    def loadFromSkeleposer(self):
        if skel and skel.node.exists() and skel.node.hasAttr("splitPosesData"):
            data = json.loads(skel.node.splitPosesData.get() or "")
            self.fromJson(data)
        else:
            self.fromJson([])

class SkeleposerSelectorWidget(QLineEdit):
    nodeChanged = Signal(str)

    def __init__(self, **kwargs):
        super(SkeleposerSelectorWidget, self).__init__(**kwargs)
        self.setPlaceholderText("Right click to select skeleposer from scene")
        self.setReadOnly(True)

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        for n in cmds.ls(type="skeleposer"):
            menu.addAction(n, lambda name=n: self.nodeChanged.emit(name))
        menu.popup(event.globalPos())

    def mouseDoubleClickEvent(self, event):
        if event.button() in [Qt.LeftButton]:
            oldName = self.text()
            if pm.objExists(oldName):
                newName, ok = QInputDialog.getText(None, "Skeleposer", "New name", QLineEdit.Normal, oldName)
                if ok:
                    pm.rename(oldName, newName)
                    self.setText(skel.node.name())
        else:
            super(SkeleposerSelectorWidget, self).mouseDoubleClickEvent(event)

class SkeleposerWindow(QFrame):
    def __init__(self, **kwargs):
        super(SkeleposerWindow, self).__init__(**kwargs)

        self._callbacks = []

        self.setWindowTitle("Skeleposer Editor")
        self.setGeometry(600,300, 600, 500)
        centerWindow(self)
        self.setWindowFlags(self.windowFlags() | Qt.Dialog)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.skeleposerSelectorWidget = SkeleposerSelectorWidget()
        self.skeleposerSelectorWidget.nodeChanged.connect(self.selectSkeleposer)

        self.treeWidget = TreeWidget()
        self.toolsWidget = ToolsWidget()
        self.toolsWidget.hide()

        self.jointsListWidget = ListWithFilterWidget()
        self.treeWidget.itemSelectionChanged.connect(self.treeSelectionChanged)

        self.splitPoseWidget = SplitPoseWidget()
        self.splitPoseWidget.setEnabled(False)

        hsplitter = WideSplitter(Qt.Horizontal)
        hsplitter.addWidget(self.jointsListWidget)
        hsplitter.addWidget(self.treeWidget)
        hsplitter.setStretchFactor(1,100)
        hsplitter.setSizes([100, 400])

        tabWidget = QTabWidget()
        tabWidget.addTab(hsplitter, "Pose")
        tabWidget.addTab(self.splitPoseWidget, "Split")

        layout.setMenuBar(self.getMenu())
        layout.addWidget(self.skeleposerSelectorWidget)
        layout.addWidget(self.toolsWidget)
        layout.addWidget(tabWidget)

    def getMenu(self):
        menu = QMenuBar()

        fileMenu = QMenu("File", self)
        fileMenu.addAction(QIcon(RootDirectory+"/icons/new.png"), "New", self.newNode)
        fileMenu.addAction("Save", self.treeWidget.saveSkeleposer)
        fileMenu.addAction("Load", self.treeWidget.loadSkeleposer)
        menu.addMenu(fileMenu)

        createMenu = QMenu("Create", self)
        createMenu.addAction(QIcon(RootDirectory+"/icons/pose.png"), "Add pose", lambda: self.treeWidget.makePose("Pose", self.treeWidget.getValidParent()), "Insert")
        createMenu.addAction(QIcon(RootDirectory+"/icons/directory.png"), "Group", self.treeWidget.groupSelected, "Ctrl+G")
        createMenu.addSeparator()
        createMenu.addAction("Add corrective pose", self.treeWidget.addCorrectivePose)
        createMenu.addAction("Weight from selection", self.treeWidget.setupWeightFromSelection)
        createMenu.addAction("Inbetween from selection", self.treeWidget.createInbetweenFromSelection)
        createMenu.addAction("Add inbetween pose", self.treeWidget.addInbetweenPose)
        createMenu.addSeparator()

        menu.addMenu(createMenu)

        editMenu = QMenu("Edit", self)
        editMenu.addAction("Duplicate", self.treeWidget.duplicateItems, "Ctrl+D")
        editMenu.addAction(QIcon(RootDirectory+"/icons/reset.png"), "Remove", self.treeWidget.removeItems, "Delete")

        editMenu.addSeparator()

        editMenu.addAction(QIcon(RootDirectory+"/icons/mirror.png"), "Mirror", self.treeWidget.mirrorItems, "Ctrl+M")
        editMenu.addAction("Flip", self.treeWidget.flipItems, "Ctrl+F")
        editMenu.addAction("Flip on opposite pose", self.treeWidget.flipItemsOnOppositePose, "Ctrl+Alt+F")

        deltaMenu = QMenu("Delta", self)
        deltaMenu.addAction("Copy", self.treeWidget.copyPoseJointsDelta, "Ctrl+C")
        deltaMenu.addAction("Copy selected joints", lambda: self.treeWidget.copyPoseJointsDelta(pm.ls(sl=True, type=["joint", "transform"])))
        deltaMenu.addAction("Paste", self.treeWidget.pastePoseDelta, "Ctrl+V")
        deltaMenu.addSeparator()
        deltaMenu.addAction("Reset selected joints", self.treeWidget.resetJoints)
        editMenu.addMenu(deltaMenu)

        blendMenu = QMenu("Blend mode", self)
        blendMenu.addAction("Additive", lambda: self.treeWidget.setPoseBlendMode(0))
        blendMenu.addAction("Replace", lambda: self.treeWidget.setPoseBlendMode(1))
        editMenu.addMenu(blendMenu)

        editMenu.addSeparator()
        editMenu.addAction("Mute", self.treeWidget.muteItems, "m")
        editMenu.addAction("Reset weights", self.treeWidget.resetWeights)
        menu.addMenu(editMenu)

        bonesMenu = QMenu("Bones", self)
        bonesMenu.addAction(QIcon(RootDirectory+"/icons/bone.png"), "Add", self.treeWidget.addJoints)
        bonesMenu.addAction(QIcon(RootDirectory+"/icons/removeBone.png"), "Remove", self.treeWidget.removeJoints)
        bonesMenu.addSeparator()
        bonesMenu.addAction("Update base matrices", self.treeWidget.updateBaseMatrices)
        bonesMenu.addSeparator()
        menu.addMenu(bonesMenu)    

        toolsMenu = QMenu("Tools", self)
        toolsMenu.addAction("Replace in names", self.treeWidget.searchWindow.show, "Ctrl+R")
        toolsMenu.addAction("Collapse others", self.treeWidget.collapseOthers, "Ctrl+Space")
        toolsMenu.addAction(QIcon(RootDirectory+"/icons/layer.png"), "Add joint hierarchy as layer", self.treeWidget.addJointsAsLayer)
        
        connectionsMenu = QMenu("Output connections", self)
        connectionsMenu.addAction("Connect", self.treeWidget.reconnectOutputs)
        connectionsMenu.addAction("Disonnect", self.treeWidget.disconnectOutputs)
        toolsMenu.addMenu(connectionsMenu)

        toolsMenu.addSeparator()
        toolsMenu.addAction("Select node", lambda: pm.select(skel.node))
        menu.addMenu(toolsMenu)

        return menu

    def treeSelectionChanged(self):
        joints = []
        for sel in self.treeWidget.selectedItems():
            if sel.poseIndex is not None:
                joints += skel.getPoseJoints(sel.poseIndex)

        allJoints = set([j.name() for j in skel.getJoints()])
        poseJoints = set([j.name() for j in joints])

        self.jointsListWidget.clearItems()
        self.jointsListWidget.addItems(sorted(poseJoints), bold=True) # pose joints
        self.jointsListWidget.addItems(sorted(allJoints-poseJoints), foreground=QColor(100, 100, 100)) # all joints

    def newNode(self):
        self.selectSkeleposer(pm.createNode("skeleposer"))

    def selectSkeleposer(self, node):
        global skel
        if node:
            skel = Skeleposer(node)
            self.treeWidget.updateTree()
            self.skeleposerSelectorWidget.setText(str(node))

            self.splitPoseWidget.setEnabled(True)
            self.splitPoseWidget.loadFromSkeleposer()

            self.registerCallbacks()
            pm.select(node)
        else:
            skel = None
            self.skeleposerSelectorWidget.setText("")
            self.treeWidget.clear()
            self.splitPoseWidget.setEnabled(False)
            self.deregisterCallbacks()

        self.toolsWidget.hide()
        clearUnusedRemapValue()

    def registerCallbacks(self):
        def preRemovalCallback(node, clientData):
            self.selectSkeleposer(None)
        def nameChangedCallback(node, name, clientData):
            self.skeleposerSelectorWidget.setText(skel.node.name())

        self.deregisterCallbacks()
        nodeObject = skel.node.__apimobject__()
        self._callbacks.append( pm.api.MNodeMessage.addNodePreRemovalCallback(nodeObject, preRemovalCallback) )
        self._callbacks.append( pm.api.MNodeMessage.addNameChangedCallback(nodeObject, nameChangedCallback) )

    def deregisterCallbacks(self):
        for cb in self._callbacks:
            pm.api.MMessage.removeCallback(cb)
        self._callbacks = []

def undoRedoCallback():
    if not skel or not skel.node.exists():
        return

    tree = mainWindow.treeWidget

    def getSkeleposerState(idx=0):
        data = {"d":idx, "l":skel.node.directories[idx].directoryName.get() or "", "ch":[]}

        for chIdx in skel.node.directories[idx].directoryChildrenIndices.get() or []:
            if chIdx >= 0:
                data["ch"].append([chIdx, skel.node.poses[chIdx].poseName.get()]) # [idx, poseName]
            else:
                data["ch"].append(getSkeleposerState(-chIdx)) # directories are negative
        return data

    def getItemsState(item=tree.invisibleRootItem(), idx=0):
        data = {"d":idx, "l":item.text(0), "ch":[]}

        for i in range(item.childCount()):
            ch = item.child(i)
            if ch.poseIndex is not None:
                data["ch"].append([ch.poseIndex, ch.text(0)])
            elif ch.directoryIndex is not None:
                data["ch"].append(getItemsState(ch, ch.directoryIndex))
        return data

    if getItemsState() == getSkeleposerState():
        return

    with tree.keepState():
        print("SkeleposerEditor undo")
        tree.clear()
        tree.addItemsFromSkeleposerData(tree.invisibleRootItem(), skel.getDirectoryData())

    mainWindow.splitPoseWidget.loadFromSkeleposer()

pm.scriptJob(e=["Undo", undoRedoCallback])
pm.scriptJob(e=["Redo", undoRedoCallback])

skel = None
editPoseIndex = None

mainWindow = SkeleposerWindow(parent=mayaMainWindow)