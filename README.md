# **COMPUTER VISION BASED SYSTEM FOR AUTONOMOUS TAXIING OF AIRCRAFT**

### [Prashant Gaikwat](#), [Abhishek Mukhopadhyay](#), [Anujith Muraleedharan](#), [Mukund Mitra](#), [Pradipta Biswas](#)
#### Indian Institute of Science, Bengaluru  
#### *AVIATION Journal Vol 27 No 4 (2023)*

## **Implementation**

For running the project, one system is used to run ROS to control the robot based on the control signal received, which is the result from the lane and object detection model on an external GPU.  


### **Initialization**
Connect to the robot via SSH and run the following commands:
```sh
rosrun tankbot_scripts differential_tank.py
rosrun lightDetection tst_server.py
```

### **On the GPU:**
Run the following script inside the `Deployment scripts` folder:
```sh
python wheel_control_generation.py
```

### **Joystick Key Inputs Detection**
After the above steps, in the ROS system, run:
```sh
rosrun lightDetection turtlebot3_pointop_key
```

### **On the GPU:**
Run:
```sh
python fusedControl_v1.py
```

### **Final Step in ROS:**
```sh
rosrun lightDetection autoUI.py
```


## **Model checkpoint**
Please copy the [resources](https://drive.google.com/drive/folders/18T3t87dYajVvqO8WMyi9NXHBhyZbazkO?usp=drive_link) folder to the Deployment Scripts folder in the repository.

## **🎥 Video Demonstration**

For a video demonstration of the project, visit the webpage [here](https://anujithm.github.io/Autonomous-Taxiing-of-Aircraft.github.io/).


## **BibTeX Citation**
If you find our work useful, please consider citing us!

```bibtex
@article{Gaikwad2023,
    title={Developing a computer vision based system for autonomous taxiing of aircraft},
    author={Gaikwad, P. and Mukhopadhyay, A. and Muraleedharan, A. and Mitra, M. and Biswas, P.},
    journal={Aviation},
    volume={27},
    number={4},
    pages={248--258},
    year={2023},
    month={Dec},
    doi={10.3846/aviation.2023.20588}
}
```


## Acknowledgments
Parts of this project page were adopted from the [Nerfies](https://nerfies.github.io/) page.

## Website License
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
