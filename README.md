# Fuzzy Controls Package

## What this does?
1). Subscribes to two topics **/odom** and **/cmd_pose** with msg types PoseStamed and Pose respectively.  
2). **/cmd_pose** has messages which describe the intended position of the bot and **/odom** has messages about the actual postion of the bot.  
3). This node finds the error and outputs the value of the force required to minimize this error to **thruster_manager/input**.  
4). Currently has two implementations of the controller, a Simple PID with constant gains and a Fuzzy PID with gains controlled by fuzzy logic.  

## Dependencies
1). [Fuzzylite](https://github.com/fuzzylite/fuzzylite), to implement the fuzzy logic. If fuzzy logic is not needed, just blacklist the FuzzyPID library in the codebase. Fuzzylite needs to be built from source and the install directory needs to be specified in CMakeLists.txt on line 22, like so.  

```
set(FUZZYLITE_DIR "/home/shivam/fuzzylite/fuzzylite")
```

2). Eigen, to handle the quaternions. To install it on ubunbu-

```
sudo apt-get install libeigen3-dev
```

On Arch-

```
sudo pacman -S eigen
```

3). Yaml-cpp, to handle the config files. To install it on ubuntu-

```
sudo apt-get install libyaml-cpp-dev
```

On Arch-

```
sudo pacman -S yaml-cpp
```
