#!/bin/bash
g++ calibrate_camera.cpp -o calibrate `pkg-config --cflags --libs opencv`
