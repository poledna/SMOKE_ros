# SMOKE - ROS implementation 

This work ports the [SMOKE algorithm](https://github.com/lzccccc/SMOKE) to a ROS1 enviroment.
No changes in SMOKE architecture were made, but the weights must be loaded and then exported, in your computer.

See below the recorded SMOKE visualization on RVIz.
 
![alt text](animation/smoke_ros.gif)

it is slowed down so that it is visible. But it has achieve greater perfomance (about 20fps).

first clone this repo with 
```bash
git clone git@github.com:poledna/SMOKE_ros.git --recurse-submodules 
```
then create the virtual enviroment with:
```bash 
cd SMOKE_ros/
conda env create -f smoke_pkg.yaml
```
activate the conda enviroment (it is importante to activate this enviroment every time before the ros source)
```bash 
conda activate smoke_pkg
cd SMOKE
```
the build SMOKE
```bash 
python setup.py build develop
```
with SMOKE build 

```bash 
```


then build the workspace

```bash
cd ../smoke_ws
catkin build 
```
and source the workspace
```bash 
source devel/setup.bash
```



to cite SMOKE work:
```
@article{liu2020SMOKE,
  title={{SMOKE}: Single-Stage Monocular 3D Object Detection via Keypoint Estimation},
  author={Zechen Liu and Zizhang Wu and Roland T\'oth},
  journal={arXiv preprint arXiv:2002.10111},
  year={2020}
}
```

