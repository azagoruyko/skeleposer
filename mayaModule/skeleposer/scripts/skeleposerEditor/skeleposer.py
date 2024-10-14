import json
import re

import maya.api.OpenMaya as om
import pymel.core as pm
import maya.cmds as cmds

from . import utils

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
        if not self.node.hasAttr("dagPose"):
            self.node.addAttr("dagPose", at="message")

        if not self.node.hasAttr("connectionsData"):
            self.node.addAttr("connectionsData", dt="string")

        if not self.node.hasAttr("splitPosesData"):
            self.node.addAttr("splitPosesData", dt="string")            

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

    @utils.undoBlock
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

    @utils.undoBlock
    def resetToBase(self, joints):
        for jnt in joints:
            idx = self.getJointIndex(jnt)
            if idx is not None:
                jnt.setMatrix(self.node.baseMatrices[idx].get())

    @utils.undoBlock
    def resetDelta(self, poseIndex, joints):
        for j in joints:
            idx = self.getJointIndex(j)
            if idx is not None:
                pm.removeMultiInstance(self.node.poses[poseIndex].poseDeltaMatrices[idx], b=True)
                j.setMatrix(self.node.baseMatrices[idx].get())

    @utils.undoBlock
    def updateBaseMatrices(self):
        for ja in self.node.joints:
            inputs = ja.inputs()
            bm = self.node.baseMatrices[ja.index()]
            if inputs and bm.isSettable():
                bm.set(utils.getLocalMatrix(inputs[0]))
            else:
                pm.warning("updateBaseMatrices: %s is not writable. Skipped"%bm.name())

        self.updateDagPose()

    @utils.undoBlock
    def makeCorrectNode(self, drivenIndex, driverIndexList):
        c = pm.createNode("combinationShape", n=self.node.name()+"_"+str(drivenIndex)+"_combinationShape")
        c.combinationMethod.set(1) # lowest weighting
        for i, idx in enumerate(driverIndexList):
            self.node.poses[idx].poseWeight >> c.inputWeight[i]
        c.outputWeight >> self.node.poses[drivenIndex].poseWeight
        return c

    @utils.undoBlock
    def makeInbetweenNode(self, drivenIndex, driverIndex):
        rv = pm.createNode("remapValue", n=self.node.name()+"_"+str(drivenIndex)+"_remapValue")
        self.node.poses[driverIndex].poseWeight >> rv.inputValue
        rv.outValue >> self.node.poses[drivenIndex].poseWeight
        return rv

    @utils.undoBlock
    def addJoints(self, joints):
        for j in joints:
            if self.getJointIndex(j) is None:
                idx = self.findAvailableJointIndex()
                j.message >> self.node.joints[idx]

                if isinstance(j, pm.nt.Joint):
                    j.jo >> self.node.jointOrients[idx]
                else:
                    self.node.jointOrients[idx].set([0,0,0])

                self.node.baseMatrices[idx].set(utils.getLocalMatrix(j))

                self.node.outputTranslates[idx] >> j.t
                self.node.outputRotates[idx] >> j.r
                self.node.outputScales[idx] >> j.s
            else:
                pm.warning("addJoints: %s is already connected"%j)

        self.updateDagPose()

    @utils.undoBlock
    def removeJoints(self, joints):
        for jnt in joints:
            idx = self.getJointIndex(jnt)
            if idx is not None:
                self.removeJointByIndex(idx)

                for a in Skeleposer.TrackAttrs:
                    inp = jnt.attr(a).inputs(p=True)
                    if inp:
                        inp[0] // jnt.attr(a)

        self.updateDagPose()

    @utils.undoBlock
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

    @utils.undoBlock                    
    def removeEmptyJoints(self):
        for ja in self.node.joints:
            inputs = ja.inputs()
            if not inputs:
                self.removeJointByIndex(ja.index())
                pm.warning("removeEmptyJoints: removing %s as empty"%ja.name())

    @utils.undoBlock
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

    @utils.undoBlock
    def makePose(self, name):
        idx = self.findAvailablePoseIndex()
        self.node.poses[idx].poseName.set(name)

        indices = self.node.directories[0].directoryChildrenIndices.get() or []
        indices.append(idx)
        self.node.directories[0].directoryChildrenIndices.set(indices, type="Int32Array")
        return idx

    @utils.undoBlock
    def makeDirectory(self, name, parentIndex=0):
        idx = self.findAvailableDirectoryIndex()
        directory = self.node.directories[idx]
        directory.directoryName.set(name)
        directory.directoryParentIndex.set(parentIndex)

        indices = self.node.directories[parentIndex].directoryChildrenIndices.get() or []
        indices.append(-idx) # negative indices are directories
        self.node.directories[parentIndex].directoryChildrenIndices.set(indices, type="Int32Array")

        return idx

    @utils.undoBlock
    def removePose(self, poseIndex):
        directoryIndex = self.node.poses[poseIndex].poseDirectoryIndex.get()

        indices = self.node.directories[directoryIndex].directoryChildrenIndices.get() or []
        if poseIndex in indices:
            indices.remove(poseIndex)
        self.node.directories[directoryIndex].directoryChildrenIndices.set(indices, type="Int32Array")

        for m in self.node.poses[poseIndex].poseDeltaMatrices:
            pm.removeMultiInstance(m, b=True)

        pm.removeMultiInstance(self.node.poses[poseIndex], b=True)

    @utils.undoBlock
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

    @utils.undoBlock
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

    @utils.undoBlock
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

    @utils.undoBlock
    def removeEmptyDeltas(self, poseIndex):
        for m in self.node.poses[poseIndex].poseDeltaMatrices:
            if m.get().isEquivalent(pm.dt.Matrix(), 1e-4):
                pm.removeMultiInstance(m, b=True)

    @utils.undoBlock
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

    @utils.undoBlock
    def mirrorPose(self, poseIndex):
        dagPose = self.dagPose()

        blendMode = self.node.poses[poseIndex].poseBlendMode.get()

        joints = sorted(self.getPoseJoints(poseIndex), key=lambda j: len(j.getAllParents())) # sort by parents number, process parents first
        for j in joints:
            idx = self.getJointIndex(j)

            j_mirrored = utils.findSymmetricName(str(j), right=False) # find right side joint
            if not j_mirrored or not cmds.objExists(j_mirrored):
                continue

            j_mirrored = pm.PyNode(j_mirrored)

            mirror_idx = self.getJointIndex(j_mirrored)
            if mirror_idx is None: # if mirror joint is not connected, skip
                continue

            j_pmat = utils.dagPose_getParentMatrix(dagPose, j)
            j_mbase = utils.dagPose_getWorldMatrix(dagPose, j)

            mirrored_pmat = utils.dagPose_getParentMatrix(dagPose, j_mirrored)
            mirrored_pmatInv = mirrored_pmat.inverse()
            mirrored_mbase = utils.dagPose_getWorldMatrix(dagPose, j_mirrored)  

            delta = self.node.poses[poseIndex].poseDeltaMatrices[idx].get()
            jm = utils.applyDelta(delta, j_mbase * j_pmat.inverse()) if blendMode == 0 else om.MMatrix(delta)
            jm *= j_pmat

            mirrored_m = utils.mirrorMatrixByDelta(j_mbase, jm, mirrored_mbase)

            if j == j_mirrored:
                mirrored_m = utils.blendMatrices(jm, mirrored_m, 0.5)

            mirrored_delta = utils.getDelta(mirrored_m * mirrored_pmatInv, mirrored_mbase * mirrored_pmatInv) if blendMode == 0 else mirrored_m * mirrored_pmatInv
            self.node.poses[poseIndex].poseDeltaMatrices[mirror_idx].set(mirrored_delta)

    @utils.undoBlock
    def flipPose(self, poseIndex):
        dagPose = self.dagPose()

        blendMode = self.node.poses[poseIndex].poseBlendMode.get()

        output = {}
        for j in self.getPoseJoints(poseIndex):
            idx = self.getJointIndex(j)

            j_mirrored = utils.findSymmetricName(str(j))
            if not j_mirrored or not cmds.objExists(j_mirrored):
                continue

            j_mirrored = pm.PyNode(j_mirrored)

            mirror_idx = self.getJointIndex(j_mirrored)
            if mirror_idx is None: # if mirror joint is not connected, skip
                continue

            j_pmat = utils.dagPose_getParentMatrix(dagPose, j)
            j_pmatInv = j_pmat.inverse()
            j_mbase = utils.dagPose_getWorldMatrix(dagPose, j)

            mirrored_pmat = utils.dagPose_getParentMatrix(dagPose, j_mirrored)
            mirrored_pmatInv = mirrored_pmat.inverse()
            mirrored_mbase = utils.dagPose_getWorldMatrix(dagPose, j_mirrored)

            j_delta = self.node.poses[poseIndex].poseDeltaMatrices[idx].get()
            mirrored_delta = self.node.poses[poseIndex].poseDeltaMatrices[mirror_idx].get()

            jm = utils.applyDelta(j_delta, j_mbase * j_pmatInv) if blendMode == 0 else om.MMatrix(j_delta)
            jm *= j_pmat

            mirrored_jm = utils.applyDelta(mirrored_delta, mirrored_mbase * mirrored_pmatInv) if blendMode == 0 else om.MMatrix(mirrored_delta)
            mirrored_jm *= mirrored_pmat

            m = utils.mirrorMatrixByDelta(mirrored_mbase, mirrored_jm, j_mbase)
            mirrored_m = utils.mirrorMatrixByDelta(j_mbase, jm, mirrored_mbase)

            output[idx] = utils.getDelta(m * j_pmatInv, j_mbase * j_pmatInv) if blendMode == 0 else m * j_pmatInv
            output[mirror_idx] = utils.getDelta(mirrored_m * mirrored_pmatInv, mirrored_mbase * mirrored_pmatInv) if blendMode == 0 else mirrored_m * mirrored_pmatInv

        for idx in output:
            self.node.poses[poseIndex].poseDeltaMatrices[idx].set(output[idx])

        self.removeEmptyDeltas(poseIndex)

    @utils.undoBlock
    def changePoseBlendMode(self, poseIndex, blend):
        dagPose = self.dagPose()

        pose = self.node.poses[poseIndex]
        poseBlend = pose.poseBlendMode.get()

        for j in self.getPoseJoints(poseIndex):
            idx = self.getJointIndex(j)

            delta = om.MMatrix(pose.poseDeltaMatrices[idx].get())
            bmat = utils.dagPose_getWorldMatrix(dagPose, j)
            pmat = utils.dagPose_getParentMatrix(dagPose, j)
            wm = utils.applyDelta(delta, bmat) if poseBlend == 0 else delta * pmat
            pose.poseDeltaMatrices[idx].set(utils.getDelta(wm, bmat) if blend == 0 else wm * pmat.inverse())

        pose.poseBlendMode.set(blend)

    @utils.undoBlock
    def disconnectOutputs(self):
        if self.node.connectionsData.get(): # if connectionsData exists
            pm.warning("Disconnection is skipped")
            return          
              
        connectionsData = {}
        for ja in self.node.joints:
            j = ja.inputs()[0]

            connections = {}
            for a in Skeleposer.TrackAttrs:
                inp = j.attr(a).inputs(p=True)
                if inp:
                    connections[a] = inp[0].name()
                    inp[0] // j.attr(a)

            connectionsData[ja.index()] = connections

        self.node.connectionsData.set(json.dumps(connectionsData))

    @utils.undoBlock
    def reconnectOutputs(self):
        connectionsData = self.node.connectionsData.get()
        if not connectionsData: # if connectionsData doesn't exist
            pm.warning("Connection is skipped")
            return
                
        connectionsData = json.loads(connectionsData)
        for idx in connectionsData:
            for a in connectionsData[idx]:
                j = self.getJointByIndex(idx)
                pm.connectAttr(connectionsData[idx][a], j+"."+a, f=True)

        self.node.connectionsData.set("")

    @utils.undoBlock
    def setupAliases(self, install=True):
        def setupAlias(attr, name, *, prefix=""):
            name = str(name)
            isValidName = lambda n: re.match("^\\w[\\w\\d_]*$", n)

            if pm.aliasAttr(attr, q=True):
                pm.aliasAttr(attr, rm=True)

            if isValidName(name) and install:
                pm.aliasAttr(prefix+name, attr)

        for plug in self.node.joints:
            idx = plug.index()
            n = self.getJointByIndex(idx)
            setupAlias(self.node.joints[idx], n, prefix="j_")
            setupAlias(self.node.baseMatrices[idx], n, prefix="bm_")
            setupAlias(self.node.jointOrients[idx], n, prefix="jo_")
            setupAlias(self.node.outputTranslates[idx], n, prefix="t_")
            setupAlias(self.node.outputRotates[idx], n, prefix="r_")
            setupAlias(self.node.outputScales[idx], n, prefix="s_")

        for plug in self.node.poses:
            n = plug.poseName.get() or ""
            setupAlias(plug, n, prefix="p_")
            setupAlias(plug.poseWeight, n)

        for plug in self.node.directories:
            n = plug.directoryName.get() or ""
            setupAlias(plug, n, prefix="d_")

    @utils.undoBlock
    def beginEditPose(self, idx):
        if self._editPoseData:
            pm.warning("Already in edit mode")
            return

        self._editPoseData = {"joints":{}, "poseIndex":idx, "input": None}

        inputs = self.node.poses[idx].poseWeight.inputs(p=True)
        if inputs:
            inputs[0] // self.node.poses[idx].poseWeight
            self._editPoseData["input"] = inputs[0]

        self.node.poses[idx].poseWeight.set(1)

        poseEnabled = self.node.poses[idx].poseEnabled.get()
        self.node.poses[idx].poseEnabled.set(False) # disable pose

        for j in self.getJoints():
            self._editPoseData["joints"][j.name()] = utils.getLocalMatrix(j)

        self.node.poses[idx].poseEnabled.set(poseEnabled) # restore pose state

        self.disconnectOutputs()

    @utils.undoBlock
    def endEditPose(self):
        if not self._editPoseData:
            pm.warning("Not in edit mode")
            return

        pose = self.node.poses[self._editPoseData["poseIndex"]]

        for j in self.getJoints():
            jointIndex = self.getJointIndex(j)

            bmat = self._editPoseData["joints"][j.name()]

            mat = utils.getLocalMatrix(j)
            if not mat.isEquivalent(bmat, 1e-4):
                poseBlendMode = pose.poseBlendMode.get()

                if poseBlendMode == 0: # additive
                    pose.poseDeltaMatrices[jointIndex].set(utils.getDelta(mat, bmat))

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

    @utils.undoBlock
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

            for pattern in kwargs:
                if re.search(pattern, j.name()):
                    w = kwargs[pattern]
                    pdm = destPose.poseDeltaMatrices[j_idx]
                    if w > 1e-3:
                        pdm.set( utils.blendMatrices(om.MMatrix(), om.MMatrix(pdm.get()), w) )
                    else:
                        pm.removeMultiInstance(pdm, b=True)

    @utils.undoBlock
    def addSplitBlends(self, blendShape, targetName, poses):
        def findTargetIndexByName(blend, name):
            for aw in blend.w:
                if pm.aliasAttr(aw, q=True)==name:
                    return aw.index()

        def findAvailableTargetIndex(blend):
            idx = 0
            while blend.w[idx].exists():
                idx += 1
            return idx
        
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
                    inputs[0] // pose.poseWeight
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
        targetIndices, targetDeltas = utils.getBlendShapeTargetDelta(blendShape, targetIndex)
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

    @utils.undoBlock
    def addJointsAsLayer(self, rootJoint, shouldTransferSkin=True):
        rootJoint = pm.PyNode(rootJoint)
        joints = [rootJoint] + rootJoint.listRelatives(type="joint", ad=True, c=True)

        skelJoints = {j: utils.matchJoint(j) for j in joints}

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
                utils.transferSkin(old, new)

        # update skin clusters
        if shouldTransferSkin:
            skinClusters = pm.ls(type="skinCluster")
            if skinClusters:
                pm.dgdirty(skinClusters)

        return skelJoints[rootJoint]

    def toJson(self):
        data = {"joints":{}, "baseMatrices":{}, "poses": {}, "directories": {}, "splitPosesData": {}}

        for j in self.node.joints:
            inputs = j.inputs()
            if inputs:
                data["joints"][j.index()] = inputs[0].name()

        for bm in self.node.baseMatrices:
            a = "{}.baseMatrices[{}]".format(self.node, bm.index())
            data["baseMatrices"][bm.index()] = [utils.shortenValue(v) for v in cmds.getAttr(a)]

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
                poseData["poseDeltaMatrices"][m.index()] = [utils.shortenValue(v) for v in cmds.getAttr(a)]

        data["splitPosesData"] = json.loads(self.node.splitPosesData.get() or "")

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

        self.node.splitPosesData.set(json.dumps(data.get("splitPosesData", "")))

    def getWorldPoses(self, joints=None):
        dagPose = self.dagPose()

        # cache joints matrices
        jointsData = {}
        for j in self.getJoints():
            idx = self.getJointIndex(j)

            bmat = utils.dagPose_getWorldMatrix(dagPose, j)
            pmat = utils.dagPose_getParentMatrix(dagPose, j)
            jointsData[idx] = {"joint":j, "baseMatrix":bmat, "parentMatrix":pmat}

        data = {}
        for pose in self.node.poses:
            blendMode = pose.poseBlendMode.get()

            deltas = {}
            for delta in pose.poseDeltaMatrices:
                jdata = jointsData[delta.index()]

                if not joints or jdata["joint"] in joints:
                    dm = delta.get()
                    wm = utils.applyDelta(dm, jdata["baseMatrix"]) if blendMode == 0 else om.MMatrix(dm) * jdata["parentMatrix"]
                    deltas[delta.index()] = wm.tolist()

            if deltas:
                data[pose.index()] = deltas

        return data

    @utils.undoBlock
    def setWorldPoses(self, poses):
        dagPose = self.dagPose()

        # cache joints matrices
        jointsData = {}
        for j in self.getJoints():
            idx = self.getJointIndex(j)

            bmat = utils.dagPose_getWorldMatrix(dagPose, j)
            pmat = utils.dagPose_getParentMatrix(dagPose, j)
            jointsData[idx] = {"joint":j, "baseMatrix":bmat, "parentInverseMatrix":pmat.inverse()}

        for pi in poses:
            blendMode = self.node.poses[pi].poseBlendMode.get()

            for di in poses[pi]:
                jdata = jointsData[di]
                delta = utils.getDelta(poses[pi][di], jdata["baseMatrix"]) if blendMode == 0 else om.MMatrix(poses[pi][di]) * jdata["parentInverseMatrix"]
                self.node.poses[pi].poseDeltaMatrices[di].set(delta)
