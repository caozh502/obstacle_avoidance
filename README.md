# Obstacle Detection and Avoidance for UAV
offline test on Ubuntu 20.04

```bash
sudo apt install libpcl-dev
cd ~
git clone https://github.com/caozh502/obstacle_avoidance.git
cd Obstacle_Avoidance
mkdir build && cd build
cmake ..
make
./main
```