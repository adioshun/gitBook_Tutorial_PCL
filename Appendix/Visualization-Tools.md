# 시각화 툴 



## Cloud Compare 

> [홈페이지](http://www.danielgm.net/cc/), [블로그](http://www.pointcloud.jp/blog_n23/)

```python
$ sudo snap install cloudcompare
$ sudo snap refresh --edge cloudcompare
$ cloudcompare.CloudCompare
```

- 플러그인 : https://github.com/CloudCompare/CloudCompare/issues/646
- 플러그인 so파일을 `/snap/cloudcompare/current/lib/cloudcompare/plugins` 
  - 프로그램 실행 - help - about plugins
- https://github.com/CloudCompare/CloudCompare/issues/536







---

## Paraview 


> [홈페이지](https://www.paraview.org/), [PCL Plugin](https://www.paraview.org/Wiki/ParaView/PCL_Plugin), [YouTube데모](https://www.youtube.com/watch?v=BZBQXcBvHW0) 

```
# apt 설치 
apt-get install paraview

# 코드 설치 
# paraview 다운로드 : https://www.paraview.org/download/
# 권장 버젼 : ParaView v4.1.0+,   PCL v1.5.1+
$ tar xvfz ParaView-3.14.1-Source.tar.gz
$ cd /home/ParaView-3.12.0 
$ mkdir build 
$ cd build 
$ ccmake .. #OR $cmake-gui Make sure that BUILD_SHARED_LIBS is set to ON Configure and generate files 
$ make 
# 실행 파일 : /home/ParaView-3.12.0/build/bin 
```




- [ParaView/PCL Plugin/Download And Build Instructions](https://www.paraview.org/Wiki/ParaView/PCL_Plugin/Download_And_Build_Instructions) : ParaView 3.14.1 + PCL Plugin v1.0


- [ROS Manual](http://wiki.ros.org/Industrial/Tutorials/PCLParaview): Paraview 3.12 + PCL Plugin 1.0 (PCL 1.5)

- A plugin to enable PCL functionality in ParaView: [Paraview 4.1](https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=binary&os=Linux&downloadFile=ParaView-4.1.0-Linux-64bit-glibc-2.3.6.tar.gz) + [PCL Plugin v1.1](https://github.com/Kitware/PCLPlugin) (PCL = 1.5.1)



- [ROS Manual](http://wiki.ros.org/Industrial/Tutorials/PCLParaview): Paraview 3.12 + PCL Plugin 1.0 (PCL 1.5)


--- 
  
# PCL-Cpp 제공 툴 

> [PCLVisualizer](http://pointclouds.org/documentation/tutorials/pcl_visualizer.php), [PCL Visualization overview](http://pointclouds.org/documentation/overview/visualization.php)

설치 
```
$ sudo apt install pcl-tools 
$ pcl_viewer [파일명.pcd]
```


```
The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.
Syntax is: pcl_viewer <file_name 1..N>.<pcd or vtk> <options>
  where options are:
                     -bc r,g,b                = background color
                     -fc r,g,b                = foreground color
                     -ps X                    = point size (1..64) 
                     -opaque X                = rendered point cloud opacity (0..1)
                     -shading X               = rendered surface shading ('flat' (default), 'gouraud', 'phong')
                     -position x,y,z          = absolute point cloud position in metres
                     -orientation r,p,y       = absolute point cloud orientation (roll, pitch, yaw) in radians
                     -ax n                    = enable on-screen display of XYZ axes and scale them to n
                     -ax_pos X,Y,Z            = if axes are enabled, set their X,Y,Z position in space (default 0,0,0)

                     -cam (*)                 = use given camera settings as initial view
 (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Field of View Y / Window Size / Window Pos] or use a <filename.cam> that contains the same information.

                     -multiview 0/1           = enable/disable auto-multi viewport rendering (default disabled)


                     -normals 0/X             = disable/enable the display of every Xth point's surface normal as lines (default disabled)
                     -normals_scale X         = resize the normal unit vector size to X (default 0.02)

                     -pc 0/X                  = disable/enable the display of every Xth point's principal curvatures as lines (default disabled)
                     -pc_scale X              = resize the principal curvatures vectors size to X (default 0.02)

                     -immediate_rendering 0/1 = use immediate mode rendering to draw the data (default: disabled)
                                                Note: the use of immediate rendering will enable the visualization of larger datasets at the expense of extra RAM.
                                                See http://en.wikipedia.org/wiki/Immediate_mode for more information.
                     -vbo_rendering 0/1       = use OpenGL 1.4+ Vertex Buffer Objects for rendering (default: disabled)
                                                Note: the use of VBOs will enable the visualization of larger datasets at the expense of extra RAM.
                                                See http://en.wikipedia.org/wiki/Vertex_Buffer_Object for more information.

                     -use_point_picking       = enable the usage of picking points on screen (default disabled)

                     -optimal_label_colors    = maps existing labels to the optimal sequential glasbey colors, label_ids will not be mapped to fixed colors (default disabled)
```
(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque} parameters; they will be automatically assigned to the right file)

Usage example:

> pcl_viewer -multiview 1 {AAA.pcd} {BBB.pcd} {CCC.pcd}

The above will load the partial_cup_model.pcd file 3 times, and will create a multi-viewport rendering (-multiview 1).




---

- [point cloud visualization with jupyter/pcl-python/and potree](https://www.youtube.com/watch?v=s2IvpYvB7Ew): YouTube데모 



- https://www.slicer.org/ : medical images


- http://www.sci.utah.edu/software/imagevis3d.html

- http://www.danielgm.net/cc/