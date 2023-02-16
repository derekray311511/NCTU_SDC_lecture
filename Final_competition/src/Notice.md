# Record my progress when doing this homework

## Problems
---
### Version limit
- scipy  
`$pip install scipy==1.5.2`
- scikit-learn 0.19.2  
`$pip install scikit-learn==0.19.2`
- filterpy
`$pip install filterpy==1.4.5`

### catkin_make argo_tracking
- fatal error: json/json.h: No such file or directory  
### solution :
`sudo apt-get install libjsoncpp-dev `
`sudo ln -s /usr/include/jsoncpp/json/ /usr/include/json`

## Run run_ab3dmot.py
test:
`python run_ab3dmot.py --split test --dets_dataroot /home/ee904/SDC_lecture/hw5_noetic_ws/src/argoverse_detections_2020 --raw_data_dir /home/ee904/SDC_lecture/hw5_noetic_ws/src/tracking_test_v1.1/argoverse-tracking/test --max_age 10 --min_conf 0.3`
val:
`python run_ab3dmot.py --split val --dets_dataroot /home/ee904/SDC_lecture/hw5_noetic_ws/src/argoverse_detections_2020 --raw_data_dir /data/Argoverse/argoverse-tracking/val --min_conf 0.3 --max_age 10`


### Json link library
add 3 line to cmakelist
set(lib_DIR /usr/lib/x84_64-linux-gnu)
link_directories(${lib_DIR})
target_link_libraries(pkg ${catkin_LIBRARIES} libjsoncpp.a)