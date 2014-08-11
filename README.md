CvStereo - DisCODe Component Library
====================================

Description
-----------

DCL containing components and tasks realted to aquisition and processing of depth maps/point clouds from stereopairs.

Dependencies
------------

OpenCV - version 2.4.8

PCL - version 1.7.1.

CvCoreTypes - DisCODe DCL

Datasets
------------
Call __make dataset__ from the build directory in order to download and unrar the files required by exemplary tasks.
   * data/bible_lr - contains images acquited from left and right camera of the Velma's active head
   * data/bible_rgbxyz - dataset containing RGB-D images (pairs of RGB and XYZ files)    

Tasks
------------
Velma acquisition:
   * __VelmaStereoLRViewer__ - Displays left and right images acquired from the stereo pair mounted on Velma head. Enables writing of LR pairs to files. 
   * __VelmaStereoToRGBXYZViewer__ - Displays a stereo (RGB and depth XYZ) images acquired from Velma (LR) cameras. Enables writing of RGB-XYZ pairs to files (in png and yml format respectivelly).
   * __VelmaStereoCameraToCloudViewer__ - Acquires stereo images from Velma head and transforms them into coloured point clouds. Enables saving of point clouds to PCD files. 
    
Stereo sequences from files:   
   * __StereoLRSequenceViewer__ - Displays a sequence of left and right images acquired from a stereo pair.
   * __StereoLRSequenceToRGBXYZViewer__ - Displays a sequence of stereo (RGB and depth XYZ) images generated from pairs of left-right (LR) camera images. Enables writing of RGB-XYZ pairs to files (in png and yml format respectivelly). 

   * __StereoLRSequenceToCloudViewer__ - Displays point clouds generated from a sequence of stereo-images (LR). Enables writing of point cloud to PCD files.
   * __StereoRGBXYZSequenceViewer__ - Displays a sequence of  stereo-images (RGB and depth XYZ).


Developers
----------

Dawid Kaczmarek (frikodwid@gmail.com)

Łukasz Żmuda (lukzmuda1@gmail.com)

Maintainer
----------

Tomasz Kornuta (tkornuta@gmail.com)

