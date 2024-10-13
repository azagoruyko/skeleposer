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

from . import utils
from .skeleposer import Skeleposer

from shiboken2 import wrapInstance
mayaMainWindow = wrapInstance(int(pm.api.MQtUtil.mainWindow()), QMainWindow)

RootDirectory = os.path.dirname(__file__)

def getQWidgetFromMelControl(ctrl):
    ptr = pm.api.MQtUtil.findControl(ctrl)
    return wrapInstance(int(ptr), QWidget)

@utils.undoBlock
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

        driver = utils.getRemapActualWeightInput(skel.node.poses[item.poseIndex].poseWeight)
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
        driver = utils.getRemapActualWeightInput(skel.node.poses[self.item.poseIndex].poseWeight)

        remapNode = skel.node.poses[self.item.poseIndex].poseWeight.inputs(type="remapValue")
        limit = remapNode[0].inputMax.get() if remapNode else 1

        changeDialog = ChangeDriverDialog(driver, limit, parent=mainWindow)
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
        self.labelWidget.setText(utils.getRemapActualWeightInput(skel.node.poses[self.item.poseIndex].poseWeight).name())

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

    def importSkeleposer(self):
        if not self.checkSkeleposer():
            return
        
        path, _ = QFileDialog.getOpenFileName(self, "Import skeleposer", "", "*.json")
        if path:
            with open(path, "r") as f:
                data = json.load(f)
            skel.fromJson(data)
            self.updateTree()            
            mainWindow.splitPoseWidget.loadFromSkeleposer()

    def saveSkeleposer(self):
        if not self.checkSkeleposer():
            return

        path, _ = QFileDialog.getSaveFileName(self, "Export skeleposer", "", "*.json")
        if path:
            with open(path, "w") as f:
                json.dump(skel.toJson(), f)

    @utils.undoBlock
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

    @utils.undoBlock
    def addInbetweenPose(self):
        if not self.checkSkeleposer():
            return
                
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                item = self.makePose(sel.text(0)+"_inbtw", self.getValidParent())
                skel.makeInbetweenNode(item.poseIndex, sel.poseIndex)
                setItemWidgets(item)

    @utils.undoBlock
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

    @utils.undoBlock
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

    @utils.undoBlock
    def pastePoseDelta(self):
        if not self.checkSkeleposer():
            return
                
        if self.clipboard:
            currentItem = self.currentItem()
            if currentItem and currentItem.poseIndex is not None:
                skel.copyPose(self.clipboard["poseIndex"], currentItem.poseIndex, self.clipboard["joints"])

    @utils.undoBlock
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

                destPoseName = utils.findSymmetricName(sourcePoseName)
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

    @utils.undoBlock
    def mirrorItems(self, items=None):
        if not self.checkSkeleposer():
            return
                
        for sel in items or self.selectedItems():
            if sel.poseIndex is not None:
                skel.mirrorPose(sel.poseIndex)

            elif sel.directoryIndex is not None:
                self.mirrorItems(self.getChildrenRecursively(sel))

    @utils.undoBlock
    def flipItems(self, items=None):
        if not self.checkSkeleposer():
            return
                
        for sel in items or self.selectedItems():
            if sel.poseIndex is not None:
                skel.flipPose(sel.poseIndex)

            elif sel.directoryIndex is not None:
                self.flipItems(self.getChildrenRecursively(sel))

    @utils.undoBlock
    def resetWeights(self):
        if not self.checkSkeleposer():
            return
                
        for p in skel.node.poses:
            if p.poseWeight.isSettable():
                p.poseWeight.set(0)

    @utils.undoBlock
    def resetJoints(self):
        if not self.checkSkeleposer():
            return
                
        joints = pm.ls(sl=True, type=["joint", "transform"])
        for sel in self.selectedItems():
            if sel.poseIndex is not None:
                skel.resetDelta(sel.poseIndex, joints)

    @utils.undoBlock
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
                    
            item.setSelected(False)
            newItem.setSelected(True)

    @utils.undoBlock
    def setupWeightFromSelection(self):
        if not self.checkSkeleposer():
            return
                
        currentItem = self.currentItem()
        if currentItem and currentItem.poseIndex is not None:
            indices = [item.poseIndex for item in self.selectedItems() if item.poseIndex is not None and item is not currentItem]
            skel.makeCorrectNode(currentItem.poseIndex, indices)
            setItemWidgets(currentItem)

    @utils.undoBlock
    def createInbetweenFromSelection(self):
        if not self.checkSkeleposer():
            return
                
        currentItem = self.currentItem()
        if currentItem and currentItem.poseIndex is not None:
            indices = [item.poseIndex for item in self.selectedItems() if item.poseIndex is not None and item is not currentItem]
            skel.makeInbetweenNode(currentItem.poseIndex, indices[-1])
            setItemWidgets(currentItem)

    @utils.undoBlock
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

    @utils.undoBlock
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
    
    @utils.undoBlock
    def addJoints(self):
        if not self.checkSkeleposer():
            return
                
        ls = pm.ls(sl=True, type=["joint", "transform"])
        if ls:
            skel.addJoints(ls)
        else:
            pm.warning("Select joints to add")

    @utils.undoBlock
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

    @utils.undoBlock
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

    @utils.undoBlock
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

    @utils.undoBlock
    def blendValueChanged(self, v):
        for j in pm.ls(sl=True, type="transform"):
            if j in self.matrices:
                bm, m = self.matrices[j]
                j.setMatrix(utils.blendMatrices(bm, m, v))

    def showEvent(self, event):
        self.matrices = {}
        poseIndex = skel._editPoseData["poseIndex"]
        for j in skel.getPoseJoints(poseIndex):
            self.matrices[j] = (skel.node.baseMatrices[skel.getJointIndex(j)].get(), utils.getLocalMatrix(j))

    def mirrorJoints(self):
        dagPose = skel.dagPose()

        joints = sorted(pm.ls(sl=True, type=["joint", "transform"]), key=lambda j: len(j.getAllParents())) # sort by parents number, process parents first

        for L_joint in joints:
            R_joint = utils.findSymmetricName(str(L_joint), right=False) # skip right joints
            if not R_joint or not cmds.objExists(R_joint):
                continue

            R_joint = pm.PyNode(R_joint)

            L_m = om.MMatrix(L_joint.wm.get())

            L_base = utils.dagPose_getWorldMatrix(dagPose, L_joint)
            R_base = utils.dagPose_getWorldMatrix(dagPose, R_joint)

            R_m = utils.mirrorMatrixByDelta(L_base, L_m, R_base)
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
        menu.addAction("Add", self.addPoseItem)
        if self.selectedItems():
            menu.addAction("Duplicate", self.duplicatePoseItem)
        menu.addSeparator()
        menu.addAction("Remove", self.removePoseItem)

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

        menu.addAction("Add", self.addPatternItem)
        if self.selectedItems():
            menu.addAction("Duplicate", self.duplicatePatternItem)
        menu.addSeparator()
        menu.addAction("Remove", self.removePatternItem)

        menu.popup(event.globalPos())

    def validateItem(self, item):
        self.blockSignals(True)
        if item.column() == 1:
            try:
                v = float(item.text())
            except:
                v = 0
            item.setText(str(utils.clamp(v)))
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

    @utils.undoBlock
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
        if skel and skel.node.exists():
            skel.node.splitPosesData.set(json.dumps(self.toJson()))

    def loadFromSkeleposer(self):
        if skel and skel.node.exists():
            data = skel.node.splitPosesData.get()
            self.fromJson(json.loads(data) if data else [])

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

class MainWindow(QFrame):
    def __init__(self, **kwargs):
        super(MainWindow, self).__init__(**kwargs)

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
        fileMenu.addAction("Load", self.treeWidget.importSkeleposer)
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
        toolsMenu.addAction("Setup aliases", self.setupAliases)
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

    def setupAliases(self):
        if not skel or not skel.node.exists():
            return
        skel.setupAliases()

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
        utils.clearUnusedRemapValue()

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

mainWindow = MainWindow(parent=mayaMainWindow)