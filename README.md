# Skeleposer

What is Skeleposer? <br>

Briefly, it's like Shape Editor in Maya, but works with transforms and joints. It can be used to make complex facial rigs based on joints. It's especially good for game engines and realtime graphics. 

![image](https://github.com/azagoruyko/skeleposer/assets/9614751/c5b0a40a-a09e-4fd6-8dc4-8363d2bd21f8)


Youtube: https://www.youtube.com/watch?v=yBulcW3-WS4

## Basic workflow

1. **Create skeleposer node**.<br>
  Each node can control large number of bones.
  
3. **Create joints and add them to skeleposer by pressing "Add joints" button**.<br>
  You can add/remove joints any time you like.<br>
  Just make sure that by removing joints you also remove these joints from pose deltas.
  
4. **Work with poses**. <br>
  In practice, you work with skinCluster and poses at the same time.  
  
6. **Connect controls to the poses using drivers**.<br>
  This is a convenient way of connecting controls to poses.

## Features
Skeleposer supports a lot of cool features that make the working process nice:
* **UI is very similar to Shape Editor**.
* **Duplicate, mirror and flip poses**.<br>
  You make a facial rig on the left side, then just duplicate & flip to mirror the rig on the right side.
  
* **Corrective poses**.<br>
  You see some poses are combined ugly? No problem, make a corrective pose just like you do it in Shape Editor!
  
* **Inbetween poses**.<br>
  It can be used to activate poses in the middle of another pose activation.

* **Drivers**.<br>
  There is a unified interface for control connections to poses.
  
* **Directories and hierarchies**.<br>
  You can duplicate, mirror and flip whole folders with poses.
  
* **Two blend modes for poses, additive and replace**.<br>
  Additive is the default mode. Replace mode can be used to replace previous transformations. <br>
  For example, such a situation occurs when you want to close wings from any position.

* **Very fast for complex facial rigs**.<br> 
  The whole facial rig can be evaluated less then 1ms.

* **Final rig is very simple**.<br>
  Actually there is a single node called skeleposer does the job. It keeps all the poses each one is very lightweight. So the rig file is really small.

Actually the whole facial rig can be just as simple as a sequence of poses. <br>
Skeleposer can be transferred into another character as the topology doesn't matter and initial bones positions can easily be adjusted without breaking the system.

## Correctives
All corrective poses must be placed below other poses. This is a requirement as corrective poses work like patches.

## How to run
### Compile C++ plugin
You need Visual Studio and CMake.

### Python script
Add skeleposerEditor folder to your scripts path and run the following:

```python
import skeleposerEditor
skeleposerEditor.skeleposerWindow.show()  
```
## Additional nodes
There are some other nodes provided by the plugin. 
* **stickyMatrix**.<br>
  This node is used to make "sticky rig" for two independent transforms. It's useful for eye blink, sticky lips (zip) and other stuff.<br>
  
## Unreal Engine Plugin

![skel](https://user-images.githubusercontent.com/9614751/211027037-a869ba58-f1bd-4b78-8d97-8d11f77689ca.PNG)

General approach for working with Skeleposer in Unreal Engine.
1. Export poses to a json file within Skeleposer Editor in Maya.
2. Place the exported file to a Content folder of your UE project.   
3. Create Skeleposer node in Control Rig in Unreal Engine and set a poses file.<br>
   **The file path must be relative to a Content folder of your project! (e.g. poses/man.json)**
4. Add poses and set corresponding weights.

Skeleposer can activate Morph targets!
If your skeletal mesh has morphs named like poses, then those morphs are activated along with corresponding poses. 

## Your facial rigs
It would be very interesting to see your facial rigs done with Skeleposer! Don't be shy to send me your work to azagoruyko@gmail.com.
