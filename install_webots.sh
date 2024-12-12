#!/usr/bin/env bash

sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo curl -s https://cyberbotics.com/Cyberbotics.asc > Cyberbotics.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update

sudo apt install -y webots