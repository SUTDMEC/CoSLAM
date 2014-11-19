CoSLAM
======
This is a fork of the original CoSLAM in various attempts to increase robustness, error handling, as well as the additial feature of utilizing external odometry measurements to provide a metric map of the environment.


CoSLAM is a visual SLAM software that aims to use multiple freely moving cameras to simultaneously compute their egomotion and the 3D map of the surrounding scenes in a highly dynamic environment.
For more details, please refer to [CoSLAM project page](http://www.ece.nus.edu.sg/stfpage/eletp/Projects/SLAM/).


How to cite
-----------
To use the codes, please cite the paper,

Bibtex entry

        @article{
          zou2013coslam,
          title="Coslam: Collaborative visual slam in dynamic environments", 
          author="Zou, Danping and Tan, Ping", 
          journal="IEEE Trans. on Pattern Analysis and Machine Intelligence" 
          year="2013", 
          publisher="IEEE"
          }
          

          
System requirements
-----------

* A nvidia graphics card (To support Nvidia Cg language)
* Linux Unbuntu or Linux Mint (64 bit). It recommended to use linux Mint 14(nadia) 64-bit system where CoSLAM has been well tested. 


Dependent packages
-----------
Before compile the source codes, the following packages should be installed.

* 1.[LibVisualSLAM](https://github.com/danping/LibVisualSLAM)(a computer vision library for visual SLAM) 
* 2.nvidia Cg toolkit (for GPU feature tracking) 
* 3.GLEW (for shader support) 
* 4.OpenGL,GLU,glut (for Visualization) 
* 5.BLAS, LAPACK (for linear algebra)
* 6.OpenCV (for Video I/O) 
* 7.wxWidgets (for GUI) 

Here are the instructions for installing those packages.

#### 1. LibVisualSLAM
This is a computer vision library developed for CoSLAM. Please click [here](https://github.com/danping/LibVisualSLAM).

#### 2. Nvidia Cg toolkit
Please click [here](https://developer.nvidia.com/cg-toolkit-download) to download the nvidia Cg toolkit. Click the .deb file to install the package.

#### 3. GLEW
Can be installed from the repository of Ubuntu or Linux Mint by typing 

    sudo apt-get install libglew-dev
    
#### 4. OpenGL, GLU, and GLUT
OpenGL is supported by default after the nvidia graphics card driver being installed. To install GLU,GLUT, run 

    sudo apt-get install libglu1-mesa-dev
    sudo apt-get install freeglut3-dev
    
#### 5. BLAS and LAPACK
Run

    sudo apt-get install libblas-dev
    sudo apt-get install liblapack-dev
      
#### 6. OpenCV
Before installing OpenCV, you should install [ffmpeg](https://trac.ffmpeg.org/wiki/UbuntuCompilationGuide) to enable the advance Video I/O. Then download [OpenCV](http://opencv.org/downloads.html) and install it following the [installation instructions](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation). 

#### 7. wxWidgets
It important that only new versions > 2.9 are supported! The source codes can be download from [here](http://www.wxwidgets.org/downloads/). Go to the source code directory of wxWidgets and run

    ./configure --with-opengl
    make
    sudo make install
      
Download & installation
-----------
After the dependent packages are installed, go to the source code directory of CoSLAM, and run

    mkdir CoSLAM-build
    cd CoSLAM-build
    cmake ..
    make
    sudo make install
    
    
Input
-----------
The input is a txt file that lists the input video files and corresponding camera parameter files. Here is a example.

    3 #number of cameras
    0 0 #number of frames to skip and a reserve number of single camera visual SLAM
    0 0
    0 0
    /xx/xxx/video1.avi #absolute path of video files
    /xx/xxx/video2.avi
    /xx/xxx/video3.avi
    /xx/xxx/cal1.txt #absolute path of calibration files
    /xx/xxx/cal2.txt
    /xx/xxx/cal3.txt
    /xx/xxx/odo1.csv #absolute path of external odometry files
    /xx/xxx/odo2.csv 
    /xx/xxx/odo3.csv

#### Video sequences
The video sequences should be temporally synchronized. Those sequences are suggested to be in 'MPEG-4' format, though other formats may also be supported. The number of video sequences are not restricted in our system. It is however suggested that the number of input sequences be less than five, as the speed of CoSLAM system decreases significantly with the number of sequences. 

#### Camera parameters (Calibration file)
Each video sequence is associated with a camera parameter file, which is in the following form. These parameters can be obtained by using the [calibration toolkit](http://www.vision.caltech.edu/bouguetj/calib_doc/)

    #intrinsic matrix 
    fx t cx
    0 fy cy
    0 0 1 
    #five parameters for distortion
    k0 k1 k2 k3 k4

#### External Odometry Files (Required)
Use odometry measurements from external sources to be able to provide a metric map from the determined CoSLAM mapping. The process requires the matching of induvidual motions between the CoSLAM path and the odometry files, and so requires that the total scan be broken into clearly different induvidual motions. An initial calibration movement surrounded with clear pauses will be sufficient to provide proper scale. The odometry file is of the format:

        <Time from prev step (seconds)><3x3 Rotation matrix> <3x1 translation vector>
        ......

    	eg:
	dt,R1,R2,R3,R4,R5,R6,R7,R8,R9,dx,dy,dz
	dt,R1,R2,R3,R4,R5,R6,R7,R8,R9,dx,dy,dz
	dt,R1,R2,R3,R4,R5,R6,R7,R8,R9,dx,dy,dz
	...

	Where R = [R1 R2 R3 ; R4 R5 R6; R7 R8 R9] in matlab syntax representing the the camera rotation from one frame to the next.
	dx, dy, dz is the change position of pose in GLOBAL coordinates from one frame to the next.

Run
-----------
Go to the directory of the input file, then type

    CoSLAM input.txt

to run the system.

Outputs
-----------
The outputs are saved in a directory named by a time string under '~/slam_results'. The outputs include 3D map points,
camera poses, feature points and input video paths. Here is an example of the outputs of three cameras on a successful CoSLAM run:

    ~/slam_results/13-09-10=21-32/input_videos.txt      #paths of input video sequences
    ~/slam_results/13-09-10=21-32/mappts.xyz            #3D map points
    ~/slam_results/13-09-10=21-32/0_campose.xyz         #camera poses
    ~/slam_results/13-09-10=21-32/1_campose.xyz
    ~/slam_results/13-09-10=21-32/2_campose.xyz
    ~/slam_results/13-09-10=21-32/0_featpts.txt         #feature points
    ~/slam_results/13-09-10=21-32/1_featpts.txt
    ~/slam_results/13-09-10=21-32/2_featpts.txt
    ~/slam_results/13-09-10=21-32/pointsAndPath.xyz	#all of the map points and camera paths


* input_video.txt
        
        <absolute path of video #1>
        <intrinsic parameters>
        <five distortion parameters>
        <image resolution>
        ......
        <absolute path of video #2>
        ......

* mappts.xyz
The stored map points which were mapped by the system
        
        <x y z>
        ......

* x_campose.xyz
The position of the camera X for each frame.

        <x y z>
        ......

* x_featpts.txt

        <number of frames>
        <frame number> <number of feature points>
        <random id> <x y> <random id> <x y>, ....., 
        <frame number> <number of feature points>
        ......

* pointsAndPath.xyz
All of the points mapped by the system and the paths of the cameras in a single file

	<x y z>


On an unsuccessful run of CoSLAM, the system cannot create a metric map of the environment but instead saves all of the points mapped up to that point in time. The files created in this instance consist of:

     ~/slam_results/13-09-10=21-32/partX.xyz		#XYZ file of the mapped points
     ~/slam_results/13-09-10=21-32/pathAndPartX.xyz	#XYZ of the mapped points and the path



Contribute to CoSLAM
-----------
CoSLAM is a open source project that welcomes everyone to contribute to this project. 

Liciense
-----------
CoSLAM is released under [GPL v2.0](http://www.gnu.org/licenses/gpl-2.0.html).

Disclaimers
-----------
No contributer is responsible for any problems caused by using CoSLAM.

