# Instructions for building Kobuki

source ./venv.bash   
colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF    
colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF --no-warn-unused-cli    
colcon build --merge-install --packages-select kobuki_core --cmake-args -DBUILD_TESTING=OFF    
VERBOSE=1 colcon build --merge-install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF    
colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo    
vcs pull ./src   
  
wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules    
sudo cp 60-kobuki.rules /etc/udev/rules.d   

sudo service udev reload   
sudo service udev restart   


# EECS149_FP
Final Project for UC Berkeley EECS149.

## For Gaurav:
### Sample Badminton Shuttle Video Data: https://drive.google.com/drive/folders/1o113c1IX7R--ENCFZ3C4U23x6Y4dDpj6?usp=sharing
### Possible sources to follow:
BOunding box with Pytorch from scratch: (Chris is trying this)
https://towardsdatascience.com/bounding-box-prediction-from-scratch-using-pytorch-a8525da51ddc
Fine tuning YOLO v5: https://www.youtube.com/watch?v=XNRzZkZ-Byg
Pytorch Mobile (Like TF Lite) Tutorial for rbpi real time inference: https://pytorch.org/tutorials/intermediate/realtime_rpi.html
We can probably Transfer Learn one of these: 
https://pytorch.org/vision/stable/models.html#quantized-models
 
