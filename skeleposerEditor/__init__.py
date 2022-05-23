from Qt.QtGui import *
from Qt.QtCore import *
from Qt.QtWidgets import *

import maya.api.OpenMaya as om
import pymel.core as pm
import pymel.api as api
import re
import os
import json

from shiboken2 import wrapInstance
mayaMainWindow = wrapInstance(long(api.MQtUtil.mainWindow()), QMainWindow)

RootDirectory = os.path.dirname(__file__).decode("windows-1251")

def getLocalMatrix(joint):
    '''
    Get joint local matrix: t, r*jo, s
    '''
    if isinstance(joint, pm.nt.Joint):
        q = joint.getRotation().asQuaternion() * joint.getOrientation() # second one applies first for quats
    else: # transform
        q = joint.getRotation().asQuaternion()

    qm = q.asMatrix()

    sm = pm.dt.Matrix()
    sm.a00 = joint.sx.get()
    sm.a11 = joint.sy.get()
    sm.a22 = joint.sz.get()

    m = sm * qm

    m.a30 = joint.tx.get()
    m.a31 = joint.ty.get()
    m.a32 = joint.tz.get()

    return m

def matrixScale(m):
    xaxis = pm.dt.Vector(m.a00,m.a01,m.a02)
    yaxis = pm.dt.Vector(m.a10,m.a11,m.a12)
    zaxis = pm.dt.Vector(m.a20,m.a21,m.a22)
    return pm.dt.Vector(xaxis.length(), yaxis.length(), zaxis.length())

def scaledMatrix(m, scale=pm.dt.Vector(1,1,1)):
    out = pm.dt.Matrix()

    xaxis = pm.dt.Vector(m.a00,m.a01,m.a02).normal() * scale.x
    yaxis = pm.dt.Vector(m.a10,m.a11,m.a12).normal() * scale.y
    zaxis = pm.dt.Vector(m.a20,m.a21,m.a22).normal() * scale.z

    out.a00 = xaxis.x
    out.a01 = xaxis.y
    out.a02 = xaxis.z

    out.a10 = yaxis.x
    out.a11 = yaxis.y
    out.a12 = yaxis.z

    out.a20 = zaxis.x
    out.a21 = zaxis.y
    out.a22 = zaxis.z

    out.a30 = m.a30
    out.a31 = m.a31
    out.a32 = m.a32

    return out

def slerp(q1, q2, w):
    q = om.MQuaternion.slerp(om.MQuaternion(q1.x, q1.y, q1.z, q1.w),
                             om.MQuaternion(q2.x, q2.y, q2.z, q2.w), w)
    return pm.dt.Quaternion(q.x, q.y, q.z, q.w)

def blendMatrices(m1, m2, w):
    q1 = pm.dt.TransformationMatrix(scaledMatrix(m1)).getRotation().asQuaternion()
    q2 = pm.dt.TransformationMatrix(scaledMatrix(m2)).getRotation().asQuaternion()

    s = matrixScale(m1) * (1-w) + matrixScale(m2) * w
    m = scaledMatrix(slerp(q1, q2, w).asMatrix(), s)

    m.a30 = m1.a30 * (1-w) + m2.a30 * w
    m.a31 = m1.a31 * (1-w) + m2.a31 * w
    m.a32 = m1.a32 * (1-w) + m2.a32 * w
    return m

def symmat(m):
    out = pm.dt.Matrix(m)
    out.a00 *= -1
    out.a10 *= -1
    out.a20 *= -1
    out.a30 *= -1
    return out

def parentConstraintMatrix(srcBase, src, destBase):
    return destBase * srcBase.inverse() * src

def dagPose_findIndex(dagPose, j):
    for m in dagPose.members:
        inputs = m.inputs()
        if inputs and inputs[0] == j:
            return m.index()

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

def undoBlock(f):
    def inner(*args,**kwargs):
        pm.undoInfo(ock=True, cn=f.__name__)
        try:
            out = f(*args, **kwargs)
        finally:
            pm.undoInfo(cck=True)
        return out
    return inner

class Skeleposer(object):
    TrackAttrs = ["t","tx","ty","tz","r","rx","ry","rz","s","sx","sy","sz"]

    def __init__(self, node=None):
        self._editPoseData = {}

        if pm.objExists(node):
            self.node = pm.PyNode(node)
            self.removeEmptyJoints()
        else:
            self.node = pm.createNode("skeleposer")

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

    def updateBaseMatrices(self):
        for ja in self.node.joints:
            inputs = ja.inputs()
            bm = self.node.baseMatrices[ja.index()]
            if inputs and bm.isSettable():
                bm.set(getLocalMatrix(inputs[0]))
            else:
                pm.warning("updateBaseMatrices: %s is not connected"%ja.name())

        self.updateDagPose()

    def makeCorrectNode(self, drivenIndex, driverIndexList):
        c = pm.createNode("combinationShape", n=self.node.name()+"_"+str(drivenIndex)+"_combinationShape")
        for i, idx in enumerate(driverIndexList):
            self.node.poses[idx].poseWeight >> c.inputWeight[i]
        c.outputWeight >> self.node.poses[drivenIndex].poseWeight

    def makeInbetweenNode(self, drivenIndex, driverIndex):
        rv = pm.createNode("remapValue", n=self.node.name()+"_"+str(drivenIndex)+"_remapValue")
        self.node.poses[driverIndex].poseWeight >> rv.inputValue
        rv.outValue >> self.node.poses[drivenIndex].poseWeight

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
            dp = pm.dagPose(joints, s=True, sl=True, g=True, n=skel.node.name()+"_world_dagPose")
            dp.message >> self.node.dagPose
        else:
            pm.warning("updateDagPose: no joints found attached")

    def getJoints(self):
        joints = []
        for ja in self.node.joints:
            inputs = ja.inputs()
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

        joints = joints or self.getPoseJoints(fromIndex)
        indices = set([self.getJointIndex(j) for j in joints])

        destPose = self.node.poses[toIndex]
        blendMode = destPose.poseBlendMode.get()

        for mattr in self.node.poses[fromIndex].poseDeltaMatrices:
            if mattr.index() in indices:
                if blendMode == 0: # additive
                    destPose.poseDeltaMatrices[mattr.index()].set(mattr.get())

                elif blendMode == 1: # replace
                    destPose.poseDeltaMatrices[mattr.index()].set(mattr.get() * self.node.baseMatrices[mattr.index()].get())

    def mirrorPose(self, poseIndex):
        dagPose = self.dagPose()

        joints = sorted(self.getPoseJoints(poseIndex), key=lambda j: len(j.getAllParents())) # sort by parents number, process parents first
        for j in joints:
            idx = self.getJointIndex(j)

            if j.startswith("L_"):
                j_mirrored = j.replace("L_", "R_")
            elif j.startswith("R_"):
                continue
            else:
                j_mirrored = j

            j_mirrored = pm.PyNode(j_mirrored)
            mirror_idx = self.getJointIndex(j_mirrored)

            j_mbase = dagPose.worldMatrix[dagPose_findIndex(dagPose, j)].get()
            mirrored_mbase = dagPose.worldMatrix[dagPose_findIndex(dagPose, j_mirrored)].get()

            jm = self.node.poses[poseIndex].poseDeltaMatrices[idx].get() * j_mbase
            mirrored_m = parentConstraintMatrix(symmat(j_mbase), symmat(jm), mirrored_mbase)

            if j == j_mirrored:
                mirrored_m = blendMatrices(jm, mirrored_m, 0.5)

            self.node.poses[poseIndex].poseDeltaMatrices[mirror_idx].set(mirrored_m*mirrored_mbase.inverse())

    def flipPose(self, poseIndex):
        dagPose = self.dagPose()

        output = {}
        for j in self.getPoseJoints(poseIndex):
            idx = self.getJointIndex(j)

            if j.startswith("L_"):
                j_mirrored = j.replace("L_", "R_")
            elif j.startswith("R_"):
                j_mirrored = j.replace("R_", "L_")
            else:
                j_mirrored = j.name()

            j_mirrored = pm.PyNode(j_mirrored)
            mirror_idx = self.getJointIndex(j_mirrored)

            j_mbase = dagPose.worldMatrix[dagPose_findIndex(dagPose, j)].get()
            mirrored_mbase = dagPose.worldMatrix[dagPose_findIndex(dagPose, j_mirrored)].get()

            jm = self.node.poses[poseIndex].poseDeltaMatrices[idx].get() * j_mbase
            mirrored_jm = self.node.poses[poseIndex].poseDeltaMatrices[mirror_idx].get() * mirrored_mbase

            m = parentConstraintMatrix(symmat(mirrored_mbase), symmat(mirrored_jm), j_mbase)
            mirrored_m = parentConstraintMatrix(symmat(j_mbase), symmat(jm), mirrored_mbase)

            output[idx] = m*j_mbase.inverse()
            output[mirror_idx] = mirrored_m*mirrored_mbase.inverse()

        for idx in output:
            self.node.poses[poseIndex].poseDeltaMatrices[idx].set(output[idx])

        self.removeEmptyDeltas(poseIndex)

    def changePoseBlendMode(self, poseIndex, blend):
        pose = self.node.poses[poseIndex]

        for j in self.getPoseJoints(poseIndex):
            idx = self.getJointIndex(j)

            if blend == 0: # additive
                delta = pose.poseDeltaMatrices[idx].get()
                pose.poseDeltaMatrices[idx].set(delta * self.node.baseMatrices[idx].get().inverse())

            elif blend == 1: # replace
                delta = pose.poseDeltaMatrices[idx].get()
                pose.poseDeltaMatrices[idx].set(delta * self.node.baseMatrices[idx].get())

        pose.poseBlendMode.set(blend)

    @undoBlock
    def disconnectOutputs(self):
        connectionData = json.loads(self.node.connectionData.get())

        if connectionData:
            pm.warning("Already disconnected")
            return

        for ja in self.node.joints:
            j = ja.inputs()[0]

            connections = {}
            for a in Skeleposer.TrackAttrs:
                inp = j.attr(a).inputs(p=True)
                if inp:
                    connections[a] = inp[0].name()
                    pm.disconnectAttr(connections[a], j.attr(a))

            connectionData[j.name()] = connections

        self.node.connectionData.set(json.dumps(connectionData))

    @undoBlock
    def reconnectOutputs(self):
        connectionData = json.loads(self.node.connectionData.get())
        if not connectionData:
            pm.warning("Already connected")
            return

        for j in connectionData:
            for a in connectionData[j]:
                pm.connectAttr(connectionData[j][a], j+"."+a, f=True)

        self.node.connectionData.set("{}")

    def beginEditPose(self, idx):
        if self._editPoseData:
            pm.warning("Already in edit mode")
            return

        self._editPoseData = {"joints":{}, "poseIndex":idx}

        w = self.node.poses[idx].poseWeight.get()
        wInputs = self.node.poses[idx].poseWeight.inputs(p=True)
        if wInputs:
            pm.disconnectAttr(wInputs[0], self.node.poses[idx].poseWeight)
        self.node.poses[idx].poseWeight.set(0)

        for ja in self.node.joints:
            j = ja.inputs()[0]
            self._editPoseData["joints"][j.name()] = getLocalMatrix(j)

        if wInputs:
            wInputs[0] >> self.node.poses[idx].poseWeight
        else:
            self.node.poses[idx].poseWeight.set(w)

        self.disconnectOutputs()

    def endEditPose(self):
        if not self._editPoseData:
            pm.warning("Not in edit mode")
            return

        pose = self.node.poses[self._editPoseData["poseIndex"]]

        for ja in self.node.joints:
            j = ja.inputs()[0]

            if self.checkIfApplyCorrect():
                baseMatrix = self._editPoseData["joints"][j.name()]
            else:
                baseMatrix = self.node.baseMatrices[ja.index()].get()

            jmat = getLocalMatrix(j)
            if not jmat.isEquivalent(baseMatrix, 1e-4):
                poseBlendMode = pose.poseBlendMode.get()

                if poseBlendMode == 0: # additive
                    m = scaledMatrix(jmat) * scaledMatrix(baseMatrix).inverse()

                    j_scale = matrixScale(jmat)
                    baseMatrix_scale = matrixScale(baseMatrix)
                    m_scale = pm.dt.Vector(j_scale.x / baseMatrix_scale.x, j_scale.y / baseMatrix_scale.y, j_scale.z / baseMatrix_scale.z)

                    pose.poseDeltaMatrices[ja.index()].set(scaledMatrix(m, m_scale))

                elif poseBlendMode == 1: # replace
                    pose.poseDeltaMatrices[ja.index()].set(jmat)

            else:
                pm.removeMultiInstance(pose.poseDeltaMatrices[ja.index()], b=True)

        self.reconnectOutputs()
        self._editPoseData = {}

    def findActivePoseIndex(self, value=0.01):
        return [p.index() for p in self.node.poses if p.poseWeight.get() > value]

    def checkIfApplyCorrect(self):
        return len(self.findActivePoseIndex()) > 1 # true if two or more poses actived

    def getDirectoryData(self, idx=0):
        data = {"directoryIndex":idx, "children":[]}
        for chIdx in self.node.directories[idx].directoryChildrenIndices.get() or []:
            if chIdx >= 0:
                data["children"].append(chIdx)
            else:
                data["children"].append(self.getDirectoryData(-chIdx))
        return data

    def toJson(self):
        data = {"joints":{}, "baseMatrices":{}, "poses": {}, "directories": {}}

        for j in self.getJoints():
            idx = self.getJointIndex(j)
            data["joints"][idx] = j.name()

        for bm in self.node.baseMatrices:
            data["baseMatrices"][bm.index()] = bm.get().tolist()

        for d in self.node.directories:
            data["directories"][d.index()] = d.get()

        for p in self.node.poses:
            data["poses"][p.index()] = {}
            poseData = data["poses"][p.index()]

            poseData["poseName"] = p.poseName.get()
            poseData["poseWeight"] = p.poseWeight.get()
            poseData["poseDirectoryIndex"] = p.poseDirectoryIndex.get()
            poseData["poseBlendMode"] = p.poseBlendMode.get()
            poseData["poseDeltaMatrices"] = {}

            for m in p.poseDeltaMatrices:
                poseData["poseDeltaMatrices"][m.index()] = m.get().tolist()

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
            self.node.baseMatrices[idx].set(pm.dt.Matrix(m))

        for idx, d in data["directories"].items():
            a = self.node.directories[idx]
            a.directoryName.set(str(d[0]))
            a.directoryWeight.set(d[1])
            a.directoryParentIndex.set(d[2])
            a.directoryChildrenIndices.set(d[3], type="Int32Array")

        for idx, p in data["poses"].items():
            a = self.node.poses[idx]
            a.poseName.set(str(p["poseName"]))
            a.poseWeight.set(p["poseWeight"])
            a.poseDirectoryIndex.set(p["poseDirectoryIndex"])
            a.poseBlendMode.set(p["poseBlendMode"])

            for m_idx, m in p["poseDeltaMatrices"].items():
                a.poseDeltaMatrices[m_idx].set(pm.dt.Matrix(m))

####################################################################################

@undoBlock
def editButtonClicked(btn, item):
    global editPoseIndex

    w = skel.node.poses[item.poseIndex].poseWeight.get()

    if editPoseIndex is None:
        if w > 0.999:
            skel.beginEditPose(item.poseIndex)
            btn.setStyleSheet("background-color: #aaaa55")
            skeleposerWindow.toolsWidget.show()
            editPoseIndex = item.poseIndex
        else:
            pm.warning("editButtonClicked: weight must be 1 before editing")

    elif editPoseIndex == item.poseIndex:
        skel.endEditPose()
        btn.setStyleSheet("")
        skeleposerWindow.toolsWidget.hide()

        editPoseIndex = None

def setItemWidgets(item):
    tw = item.treeWidget()

    if item.directoryIndex is not None:
        attrWidget = pm.attrFieldSliderGrp(at=skel.node.directories[item.directoryIndex].directoryWeight, min=0, max=1, l="", pre=2, cw3=[0,40,100])
        w = attrWidget.asQtObject()

        for ch in w.children():
            if isinstance(ch, QSlider):
                ch.setStyleSheet("background-color: #333333; border: 1px solid #555555")

        tw.setItemWidget(item, 1, w)

        for ch in getAllChildren(item):
            setItemWidgets(ch)

    elif item.poseIndex is not None:
        attrWidget = pm.attrFieldSliderGrp(at=skel.node.poses[item.poseIndex].poseWeight,min=0, max=1, l="", pre=2, cw3=[0,40,100])
        w = attrWidget.asQtObject()

        for ch in w.children():
            if isinstance(ch, QSlider):
                ch.setStyleSheet("background-color: #333333; border: 1px solid #555555")

        tw.setItemWidget(item, 1, w)

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

def getAllChildren(item):
    children = []
    for i in range(item.childCount()):
        ch = item.child(i)
        children.append(ch)
        children += getAllChildren(ch)
    return children

def getAllParents(item):
    allParents = []

    parent = item.parent()
    if parent:
        allParents.append(parent)
        allParents += getAllParents(parent)

    return allParents[::-1]

def updateItemVisuals(item):
    if item.poseIndex is not None:
        blendMode = skel.node.poses[item.poseIndex].poseBlendMode.get()
        if blendMode == 0:
            item.setBackground(0, QTreeWidgetItem().background(0))
            item.setForeground(0, QColor(200, 200, 200))

        elif blendMode == 1:
            item.setBackground(0, QColor(140,140,200))
            item.setForeground(0, QColor(0,0,0))

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

def addItems(parentItem, data):
    for ch in data["children"]:
        if isinstance(ch, dict):
            item = makeDirectoryItem(ch["directoryIndex"])
            parentItem.addChild(item)
            addItems(item, ch)

        else:
            item = makePoseItem(ch)
            parentItem.addChild(item)

        setItemWidgets(item)

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

        changeDialog = ChangeDriverDialog(driver, parent=mayaMainWindow)
        changeDialog.accepted.connect(self.updateDriver)
        changeDialog.cleared.connect(self.clearDriver)
        changeDialog.show()

    def clearDriver(self):
        inputs = skel.node.poses[self.item.poseIndex].poseWeight.inputs(p=True)
        if inputs:
            driver = inputs[0]

            print(pm.objectType(driver.node()))

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
        if "setSectionResizeMode" in dir(self.header()):
            self.header().setSectionResizeMode(QHeaderView.ResizeToContents) # Qt5
        else:
            self.header().setResizeMode(QHeaderView.ResizeToContents) # Qt4

        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.setDragEnabled(True)
        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setDropIndicatorShown(True)
        self.setAcceptDrops(True)

        self.itemChanged.connect(lambda item, idx=None:self.treeItemChanged(item))

    def updateTree(self):
        self.clear()
        addItems(self.invisibleRootItem(), skel.getDirectoryData())

    def keyPressEvent(self, event):
        shift = event.modifiers() & Qt.ShiftModifier
        ctrl = event.modifiers() & Qt.ControlModifier
        alt = event.modifiers() & Qt.AltModifier
        key = event.key()

        if ctrl:
            if key == Qt.Key_C:
                self.copyPoseJointsDelta()

            elif key == Qt.Key_G:
                self.groupSelected()

            elif key == Qt.Key_V:
                self.pastePoseDelta()

            elif key == Qt.Key_D:
                self.duplicateItems()

            elif key == Qt.Key_R:
                self.searchWindow.show()                

            elif key == Qt.Key_M:
                self.mirrorItems()

            elif key == Qt.Key_F:
                self.flipItems()

            elif key == Qt.Key_Space:
                self.collapseOthers()

        elif key == Qt.Key_Insert:
            self.makePose("Pose", self.getValidParent())

        elif key == Qt.Key_Space:
            for item in self.selectedItems():
                item.setExpanded(not item.isExpanded())

        elif key == Qt.Key_Delete:
            self.removeItems()

        else:
            super(TreeWidget, self).keyPressEvent(event)

    def contextMenuEvent(self, event):
        if not skel:
            return

        selectedItems = self.selectedItems()

        menu = QMenu(self)

        if len(selectedItems)>1:
            addCorrectPoseAction = QAction("Add corrective pose", self)
            addCorrectPoseAction.triggered.connect(lambda _=None: self.addCorrectivePose())
            menu.addAction(addCorrectPoseAction)

            weightFromSelectionAction = QAction("Weight from selection", self)
            weightFromSelectionAction.triggered.connect(lambda _=None: self.weightFromSelection())
            menu.addAction(weightFromSelectionAction)
            menu.addSeparator()

        elif len(selectedItems)==1:
            addInbetweenAction = QAction("Add inbetween pose", self)
            addInbetweenAction.triggered.connect(lambda _=None: self.addInbetweenPose())
            menu.addAction(addInbetweenAction)
            menu.addSeparator()

        addPoseAction = QAction("Add pose\tINS", self)
        addPoseAction.triggered.connect(lambda _=None: self.makePose("Pose", self.getValidParent()))
        menu.addAction(addPoseAction)

        groupAction = QAction("Group\tCTRL-G", self)
        groupAction.triggered.connect(lambda _=None: self.groupSelected())
        menu.addAction(groupAction)

        if selectedItems:
            duplicateAction = QAction("Duplicate\tCTRL-D", self)
            duplicateAction.triggered.connect(lambda _=None: self.duplicateItems())
            menu.addAction(duplicateAction)

            removeAction = QAction("Remove\tDEL", self)
            removeAction.triggered.connect(lambda _=None: self.removeItems())
            menu.addAction(removeAction)

            menu.addSeparator()

            copyPoseDeltaAction = QAction("Copy delta\tCTRL-C", self)
            copyPoseDeltaAction.triggered.connect(lambda _=None: self.copyPoseJointsDelta())
            menu.addAction(copyPoseDeltaAction)

            copyPoseJointsDeltaAction = QAction("Copy selected joints delta", self)
            copyPoseJointsDeltaAction.triggered.connect(lambda _=None: self.copyPoseJointsDelta(pm.ls(sl=True, type=["joint", "transform"])))
            menu.addAction(copyPoseJointsDeltaAction)

            pastePoseDelta = QAction("Paste delta\tCTRL-V", self)
            pastePoseDelta.triggered.connect(lambda _=None: self.pastePoseDelta())
            pastePoseDelta.setEnabled(True if self.clipboard else False)
            menu.addAction(pastePoseDelta)

            mirrorAction = QAction("Mirror\tCTRL-M", self)
            mirrorAction.triggered.connect(lambda _=None: self.mirrorItems())
            menu.addAction(mirrorAction)

            flipAction = QAction("Flip\tCTRL-F", self)
            flipAction.triggered.connect(lambda _=None: self.flipItems())
            menu.addAction(flipAction)

            searchReplaceAction = QAction("Search/Replace\tCTRL-R", self)
            searchReplaceAction.triggered.connect(lambda _=None: self.searchWindow.show())
            menu.addAction(searchReplaceAction)            

            menu.addSeparator()

            blendMenu = QMenu("Blend mode", self)
            additiveBlendAction = QAction("Additive", self)
            additiveBlendAction.triggered.connect(lambda _=None: self.setPoseBlendMode(0))
            blendMenu.addAction(additiveBlendAction)

            replaceBlendAction = QAction("Replace", self)
            replaceBlendAction.triggered.connect(lambda _=None: self.setPoseBlendMode(1))
            blendMenu.addAction(replaceBlendAction)

            menu.addMenu(blendMenu)

            menu.addSeparator()

            selectChangedJointsAction = QAction("Select changed joints", self)
            selectChangedJointsAction.triggered.connect(lambda _=None: self.selectChangedJoints())
            menu.addAction(selectChangedJointsAction)

            resetJointsAction = QAction("Reset selected joints", self)
            resetJointsAction.triggered.connect(lambda _=None: self.resetJoints())
            menu.addAction(resetJointsAction)

        menu.addSeparator()

        collapseOthersAction = QAction("Collapse others\tCTRL-SPACE", self)
        collapseOthersAction.triggered.connect(lambda _=None: self.collapseOthers())
        menu.addAction(collapseOthersAction)

        resetWeightsAction = QAction("Reset weights", self)
        resetWeightsAction.triggered.connect(lambda _=None: self.resetWeights())
        menu.addAction(resetWeightsAction)

        connectionsMenu = QMenu("Output connections", self)
        connectAction = QAction("Connect", self)
        connectAction.triggered.connect(lambda _=None: skel.reconnectOutputs())
        connectionsMenu.addAction(connectAction)

        disconnectAction = QAction("Disonnect", self)
        disconnectAction.triggered.connect(lambda _=None: skel.disconnectOutputs())
        connectionsMenu.addAction(disconnectAction)

        menu.addMenu(connectionsMenu)

        updateBaseAction = QAction("Update base matrices", self)
        updateBaseAction.triggered.connect(lambda _=None: skel.updateBaseMatrices())
        menu.addAction(updateBaseAction)

        selectNodeAction = QAction("Select node", self)
        selectNodeAction.triggered.connect(lambda _=None: pm.select(skel.node))
        menu.addAction(selectNodeAction)

        menu.popup(event.globalPos())

    def searchAndReplace(self, searchText, replaceText):
        for sel in self.selectedItems():
            sel.setText(0, sel.text(0).replace(searchText, replaceText))  

    @undoBlock
    def addInbetweenPose(self):
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                item = self.makePose(sel.text(0)+"_inbtw", self.getValidParent())
                skel.makeInbetweenNode(item.poseIndex, sel.poseIndex)

    @undoBlock
    def setPoseBlendMode(self, blend):
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                skel.changePoseBlendMode(sel.poseIndex, blend)
                updateItemVisuals(sel)

    def collapseOthers(self):
        selectedItems = self.selectedItems()

        allParents = []
        for sel in selectedItems:
            allParents += getAllParents(sel)

        allParents = set(allParents)
        for ch in getAllChildren(self.invisibleRootItem()):
            if ch not in allParents:
                ch.setExpanded(False)

    @undoBlock
    def groupSelected(self):
        selectedItems = self.selectedItems()
        dirItem = self.makeDirectory(parent=self.getValidParent())

        for sel in selectedItems:
            (sel.parent() or self.invisibleRootItem()).removeChild(sel)
            dirItem.addChild(sel)
            self.treeItemChanged(sel)

    def copyPoseJointsDelta(self, joints=None):
        currentItem = self.currentItem()
        if currentItem and currentItem.poseIndex is not None:
            self.clipboard.append({"poseIndex": currentItem.poseIndex, "joints":joints})

    @undoBlock
    def pastePoseDelta(self):
        if self.clipboard:
            pasted = self.clipboard.pop()

            currentItem = self.currentItem()
            if currentItem and currentItem.poseIndex is not None:
                skel.copyPose(pasted["poseIndex"], currentItem.poseIndex, pasted["joints"])

    @undoBlock
    def mirrorItems(self, items=None):
        for sel in items or self.selectedItems():
            if sel.poseIndex is not None:
                skel.mirrorPose(sel.poseIndex)

            elif sel.directoryIndex is not None:
                self.mirrorItems(getAllChildren(sel))


    @undoBlock
    def flipItems(self, items=None):
        for sel in items or self.selectedItems():
            if sel.poseIndex is not None:
                skel.flipPose(sel.poseIndex)

            elif sel.directoryIndex is not None:
                self.flipItems(getAllChildren(sel))

    @undoBlock
    def resetWeights(self):
        for p in skel.node.poses:
            if p.poseWeight.isSettable():
                p.poseWeight.set(0)

    @undoBlock
    def resetJoints(self):
        joints = pm.ls(sl=True, type=["joint", "transform"])
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                skel.resetDelta(sel.poseIndex, joints)

    def selectChangedJoints(self):
        pm.select(cl=True)
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                pm.select(skel.getPoseJoints(sel.poseIndex), add=True)

    @undoBlock
    def duplicateItems(self, items=None, parent=None):
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
    def weightFromSelection(self):
        currentItem = self.currentItem()
        if currentItem and currentItem.poseIndex is not None:
            indices = [item.poseIndex for item in self.selectedItems() if item.poseIndex is not None and item is not currentItem]
            skel.makeCorrectNode(currentItem.poseIndex, indices)

    @undoBlock
    def addCorrectivePose(self):
        indices = [item.poseIndex for item in self.selectedItems() if item.poseIndex is not None]
        names = [item.text(0) for item in self.selectedItems() if item.poseIndex is not None]

        item = self.makePose("_".join(names)+"_correct", self.getValidParent())
        skel.makeCorrectNode(item.poseIndex, indices)

    @undoBlock
    def removeItems(self):
        ok = QMessageBox.question(self, "Skeleposer Editor", "Really remove?", QMessageBox.Yes and QMessageBox.No, QMessageBox.Yes) == QMessageBox.Yes
        if ok:        
            for item in self.selectedItems():
                if item.directoryIndex is not None: # remove directory
                    skel.removeDirectory(item.directoryIndex)
                    (item.parent() or self.invisibleRootItem()).removeChild(item)

                elif item.poseIndex is not None:
                    skel.removePose(item.poseIndex)
                    (item.parent() or self.invisibleRootItem()).removeChild(item)

    def getValidParent(self):
        selectedItems = self.selectedItems()
        if selectedItems:
            return selectedItems[-1].parent()

    def makePose(self, name="Pose", parent=None):
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
        parentIndex = parent.directoryIndex if parent else 0
        idx = skel.makeDirectory(name, parentIndex)

        item = makeDirectoryItem(idx)
        (parent or self.invisibleRootItem()).addChild(item)

        setItemWidgets(item)
        return item

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

    def dropEvent(self, event):
        QTreeWidget.dropEvent(self, event)

        for item in self.dragItems:
            self.treeItemChanged(item)

class SkeleposerSelectorWidget(QLineEdit):
    nodeChanged = Signal(str)

    def __init__(self, **kwargs):
        super(SkeleposerSelectorWidget, self).__init__(**kwargs)

        self.setReadOnly(True)

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        for n in pm.ls(type="skeleposer"):
            action = QAction(n.name(), self)
            action.triggered.connect(lambda _=None, name=n.name(): self.nodeChanged.emit(name))
            menu.addAction(action)

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

class BlendSliderWidget(QWidget):
    def __init__(self, **kwargs):
        super(BlendSliderWidget, self).__init__(**kwargs)

        self.matrices = {}

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
        self.valueChanged(value)

    def sliderValueChanged(self):
        value = self.sliderWidget.value()/100.0
        self.textWidget.setText(str(value))
        self.valueChanged(value)

    @undoBlock
    def valueChanged(self, v):        
        for j in pm.ls(sl=True, type="transform"):
            data = self.matrices.get(j.name())
            if data:
                j.setMatrix(blendMatrices(data["baseMatrix"], data["matrix"], v))

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

        layout.addWidget(mirrorJointsBtn)
        layout.addWidget(resetJointsBtn)
        layout.addWidget(self.blendSliderWidget)
        layout.addStretch()

    def showEvent(self, event):
        self.blendSliderWidget.matrices = {}
        for j in skel.getJoints():
            data = {"matrix": getLocalMatrix(j),
                    "baseMatrix":  skel.node.baseMatrices[skel.getJointIndex(j)].get()}

            self.blendSliderWidget.matrices[j.name()] = data

    def mirrorJoints(self):
        dagPose = skel.dagPose()

        joints = sorted(pm.ls(sl=True, type=["joint", "transform"]), key=lambda j: len(j.getAllParents())) # sort by parents number, process parents first

        for L_joint in joints:
            R_joint = L_joint.name().replace("L_", "R_")

            if pm.objExists(R_joint) and L_joint != R_joint:
                R_joint = pm.PyNode(R_joint)
            else:
                continue

            L_m = L_joint.wm.get()

            L_idx = dagPose_findIndex(dagPose, L_joint)
            R_idx = dagPose_findIndex(dagPose, R_joint)

            L_base = dagPose.worldMatrix[L_idx].get()
            R_base = dagPose.worldMatrix[R_idx].get()

            R_m = parentConstraintMatrix(symmat(L_base), symmat(L_m), R_base)
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

    def __init__(self, plug=None, **kwargs):
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

        self.limitWidget = QLineEdit("1")
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

    def updateItems(self, items):
        self.listWidget.clear()
        self.listWidget.addItems(items)
        self.filterChanged()

class SkeleposerWindow(QFrame):
    def __init__(self, **kwargs):
        super(SkeleposerWindow, self).__init__(**kwargs)

        self.setWindowTitle("Skeleposer Editor")
        self.setGeometry(500,400, 600, 400)
        self.setWindowFlags(self.windowFlags() | Qt.Dialog)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.skeleposerSelectorWidget = SkeleposerSelectorWidget()
        self.skeleposerSelectorWidget.nodeChanged.connect(self.selectSkeleposer)

        newBtn = QPushButton("New")
        newBtn.clicked.connect(self.newNode)

        addJointsBtn = QPushButton("Add joints")
        addJointsBtn.clicked.connect(self.addJoints)

        removeJointsBtn = QPushButton("Remove joints")
        removeJointsBtn.clicked.connect(self.removeJoints)

        selectJointsBtn = QPushButton("Select joints")
        selectJointsBtn.clicked.connect(self.selectJoints)

        hlayout = QHBoxLayout()
        hlayout.addWidget(QLabel("Current skeleposer"))
        hlayout.addWidget(self.skeleposerSelectorWidget)
        hlayout.addWidget(newBtn)
        hlayout.addWidget(addJointsBtn)
        hlayout.addWidget(removeJointsBtn)
        hlayout.addWidget(selectJointsBtn)

        self.treeWidget = TreeWidget()
        self.toolsWidget = ToolsWidget()
        self.toolsWidget.hide()

        self.jointsListWidget = ListWithFilterWidget()
        self.treeWidget.itemSelectionChanged.connect(self.treeSelectionChanged)

        hsplitter = WideSplitter(Qt.Horizontal)
        hsplitter.addWidget(self.jointsListWidget)
        hsplitter.addWidget(self.treeWidget)
        hsplitter.setSizes([100, 400])

        layout.addLayout(hlayout)
        layout.addWidget(self.toolsWidget)
        layout.addWidget(hsplitter)
        layout.setStretch(0,0)
        layout.setStretch(1,0)
        layout.setStretch(2,1)

    def treeSelectionChanged(self):
        joints = []
        for sel in self.treeWidget.selectedItems():
            if sel.poseIndex is not None:
                joints += skel.getPoseJoints(sel.poseIndex)

        self.jointsListWidget.updateItems([j.name() for j in joints])

    def selectJoints(self):
        if skel:
            pm.select(skel.getJoints())

    @undoBlock
    def addJoints(self):
        if skel:
            skel.addJoints(pm.ls(sl=True, type=["joint", "transform"]))

    @undoBlock
    def removeJoints(self):
        if skel:
            ok = QMessageBox.question(self, "Skeleposer Editor", "Really remove selected joints?", QMessageBox.Yes and QMessageBox.No, QMessageBox.Yes) == QMessageBox.Yes
            if ok:
                skel.removeJoints(pm.ls(sl=True, type=["joint", "transform"]))

    def newNode(self):
        self.selectSkeleposer(pm.createNode("skeleposer"))

    def selectSkeleposer(self, node):
        global skel
        skel = Skeleposer(node)
        self.treeWidget.updateTree()
        self.skeleposerSelectorWidget.setText(str(node))

'''
def undoRedoCallback():
    if editPoseIndex is None and skeleposerWindow.isVisible():
        skeleposerWindow.treeWidget.updateTree()

pm.scriptJob(e=["Undo", undoRedoCallback])
pm.scriptJob(e=["Redo", undoRedoCallback])
'''

skel = None
editPoseIndex = None

skeleposerWindow = SkeleposerWindow(parent=mayaMainWindow)
