# dotMEX_Autominy_SIM
Repository by dotMEX team from CINVESTAV Zacatenco, Mexico. 

Based on the simulator: 
https://github.com/ITAM-Robotica/EK_AutoNOMOS_Sim

## Required Dependencies
- [CUDA 11.7](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) See for example some of these pages: [1](https://techzizou.com/install-cuda-and-cudnn-on-windows-and-linux/#linux), [2](https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81#a59a), [3](https://pjreddie.com/darknet/yolo/)
- cuDNN 8.4.1.50
- OpenCV  Install it as [here](https://efcomputer.net.au/blog/4-steps-to-install-darknet-with-cuda-and-opencv-for-realtime-object-detection/).
- [darnet](https://github.com/leggedrobotics/darknet/tree/d22bbf38bd012f13d2b50c8d98149cd4a9889b7a)
- [dark_net_ros](https://github.com/leggedrobotics/darknet_ros)

## Building
Clone the repository:
	git clone https://github.com/dotmex-z/dotMEX_Autominy_SIM
	
Create the darknet repository in dotMEX_Autominy_SIM/src 
	cd dotMEX_Autominy_SIM/src
	git clone https://github.com/leggedrobotics/darknet/tree/d22bbf38bd012f13d2b50c8d98149cd4a9889b7a
	cd darknet

I recommend to use gpu and opencv configuration. Edit the Makefile to add the [gpu-architecture](https://developer.nvidia.com/cuda-gpus) and change the lines:

	GPU=1
	CUDNN=0
	CUDNN_HALF=0
	OPENCV=1

Then use the [make](https://pjreddie.com/darknet/install/) instruction.

Add to the neural network's weights to the dotMEX_Autominy_SIM/autominy_ws/src/darknet_ros/yolo_network_config/weights folder. Choose the file [dotmex_yolov3_15000.weights](https://drive.google.com/drive/folders/1a95cmAPXt_KvZuGdBtEg6sZWuQqUulx1?usp=sharing) 

Also edit the file dotMEX_Autominy_SIM/autominy_ws/src/dotmex2022/scripts/behavior_selector.py and change the "path_libs" (line 11) variable correctly.

Finally compile the workspace:
	cd dotMEX_Autominy_SIM/src
	catkin_init_workspace
	cd ..
	catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8

	






Don't forget to download the weights. The instructions are in: dotMEX_Autominy_SIM/autominy_ws/src/darknet_ros/yolo_network_config/weights, Then:

	catkin_make -DCMAKE_BUILD_TYPE=Release
	
	
-Open 
	

