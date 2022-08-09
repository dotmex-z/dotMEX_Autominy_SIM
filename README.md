# dotMEX_Autominy_SIM
Repository by dotMEX team from CINVESTAV Zacatenco, Mexico. 

Based on the simulator: 
https://github.com/ITAM-Robotica/EK_AutoNOMOS_Sim

## Required Dependencies
- [CUDA 11.7](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) See for example some of these pages: [1](https://techzizou.com/install-cuda-and-cudnn-on-windows-and-linux/#linux), [2](https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81#a59a), [3](https://pjreddie.com/darknet/yolo/)

- cuDNN 8
- [darnet](https://github.com/leggedrobotics/darknet/tree/d22bbf38bd012f13d2b50c8d98149cd4a9889b7a)
- [dark_net_ros](https://github.com/leggedrobotics/darknet_ros)

## Building
After clone the repository:

	cd dotMEX_Autominy_SIM/autominy_ws/src
	git clone https://github.com/leggedrobotics/darknet/tree/d22bbf38bd012f13d2b50c8d98149cd4a9889b7a
	cd darknet

Then edit the Makefile to add the [gpu-architecture](https://developer.nvidia.com/cuda-gpus) and change the lines:

	GPU=1
	CUDNN=0
	CUDNN_HALF=0
	OPENCV=1

Don't forget download the weights. The instructions are in dotMEX_Autominy_SIM/autominy_ws/src/darknet_ros/yolo_network_config/weights
Finally:

	catkin_make -DCMAKE_BUILD_TYPE=Release
