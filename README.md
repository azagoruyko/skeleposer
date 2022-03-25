# Skeleposer

What is Skeleposer? <br>

Briefly, it's like Shape Editor in Maya, but works with transforms and joints. It can be used to make complex facial rigs based on joints. It's especially good for game engines and realtime graphics. 

![skeleposer](https://user-images.githubusercontent.com/9614751/159117355-97a65688-4baf-4f32-99bb-5f8c5d48d9cf.png)

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

![skeleposer](https://user-images.githubusercontent.com/9614751/159117978-d545a408-84db-451a-b8e4-73765b5ef404.gif)

## Features
Skeleposer supports a lot of cool features that make the working process nice:
* **UI is very similar to Shape Editor**.
* **Duplicate/mirror/flip poses**.<br>
  You make a facial rig on the left side, then just duplicate & flip to mirror the rig on the right side.
  
* **Corrective poses**.<br>
  You see some poses are combined ugly? No problem, make a corrective pose just like you do it in Shape Editor!
  
* **Inbetween poses**.<br>
  It can be used to activate poses in the middle of another pose activation.

* **Drivers**.<br>
  It's unified interface for control connections to poses.
  
* **Directories and hierarchies**.<br>
  You can duplicate/mirror/flip whole folders with poses.
  
* **Two blend modes for poses, additive and replace**.<br>
  Additive is the default mode. Replace mode can be used to replace previous transformations. <br>
  For example, such a situation occurs when you want to close wings from any position.

* **Very fast for complex facial rigs**.<br> 
  The whole facial rig can be evaluated less then 1ms.

* **Final rig is very simple**.<br>
  Actually there is a single node called skeleposer does the job. It keeps all the poses each one is very lightweight. So the rig file is really small.

Actually the whole facial rig can be just as simple as a sequence of poses and correctives.<br>
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
* **blendMatrix**.<br>
  This node can be used to *blend* two matrices. This is done with the following approach:<br>
  a) Translation is linear.<br>
  b) Rotation is blended with slerp.<br>
  c) Scale is linear.<br>
  
* **stickyMatrix**.<br>
  This node is used to make "sticky rig" for two indepdendant transforms. It's useful for eye blink, sticky lips (zip) and other stuff.<br>
  Use `stickyMatrix` module in Rig Builder as a reference: https://github.com/azagoruyko/rigBuilder/blob/main/modules/Tools/StickyMatrix.xml
  
## Your facial rigs
It would be very interesting to see your facial rigs done with Skeleposer! Don't be shy to send me your work to azagoruyko@gmail.com.

## Thanks and gracias
If you want to support me you're welcome!

197VXFacHv8VsYR1fQcs65sugBBQm1FSa5 BTC
0x27c561D37a9163ffeAFB9FC9999f088aE4544F3B ETH
0x27c561D37a9163ffeAFB9FC9999f088aE4544F3B BSC multi
bnb1jfguvl4dwcg3ldx3vnv9qm23xve237sj5wyz26 BTCB BEP2
TWBKt2EYHS7uArz6Gbsz3bnjg293Jzdh6m TRX USDT
