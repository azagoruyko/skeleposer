# Skeleposer

What is Skeleposer? It's like Shape Editor in Maya but works with skeletons (transforms). It can be used to make complex facial rigs based on joints. It's especially good for game engines and realtime graphics.

![skeleposer](https://user-images.githubusercontent.com/9614751/159117355-97a65688-4baf-4f32-99bb-5f8c5d48d9cf.png)

## Basic workflow

You create skeleposer node, make joints, add them to skeleposer and then work with poses, fix them, refine skin weights. Then you connect controls to the poses and that's done! In practice, you work with skinCluster and poses at the same time.

![skeleposer](https://user-images.githubusercontent.com/9614751/159117978-d545a408-84db-451a-b8e4-73765b5ef404.gif)

## Features
Skeleposer supports a lot of cool features that make the working process nice:
* UI-friendly interface like Shape Editor.
* Duplicate/mirror/flip poses.<br>
  You make a facial rig on left side, then just duplicate & flip. 
  
* Corrective poses.<br>
  If two or more poses works badly, just make corrective pose for them.
  
* Inbetween poses.<br>
  It can be used to activate poses in the middle of another pose activation.

* Drivers that activate poses.<br>
  Unified interface for control connections.
  
* Directories and hierarchies.<br>
  You can duplicate/mirror/flip whole folders.
  
* Two blend modes, additive and replace for poses.<br>
  Additive is default mode. Replace mode can be used for example for auto wings closing. Such a pose just replaces previous transformations.

* Very fast for complex facial rigs!<br> 
  The whole facial rig can be evaluated in 1ms.
  
Actually the whole facial rig can be just as simple as a sequence of poses and correctives.

## How to run
### Compile C++ plugin
You need Visual Studio and CMake.

### Python script
Add skeleposerEditor folder to your scripts path and run the following:

```python
import skeleposerEditor
skeleposerEditor.skeleposerWindow.show()  
```
