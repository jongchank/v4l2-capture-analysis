#!/bin/sh

echo "FPS: 5"
v4l2-ctl -d /dev/video0 -p 5
for i in `seq 0 1000 500000`; do
    echo -n $i";"
    ./cap $i 
done

echo "FPS: 7.5"
v4l2-ctl -d /dev/video0 -p 7.5 
for i in `seq 0 1000 500000`; do
    echo -n $i";"
    ./cap $i 
done

echo "FPS: 10"
v4l2-ctl -d /dev/video0 -p 10 
for i in `seq 0 1000 500000`; do
    echo -n $i";"
    ./cap $i 
done

echo "FPS: 15"
v4l2-ctl -d /dev/video0 -p 15 
for i in `seq 0 1000 500000`; do
    echo -n $i";"
    ./cap $i 
done

echo "FPS: 20"
v4l2-ctl -d /dev/video0 -p 20 
for i in `seq 0 1000 500000`; do
    echo -n $i";"
    ./cap $i 
done

echo "FPS: 24"
v4l2-ctl -d /dev/video0 -p 24 
for i in `seq 0 1000 500000`; do
    echo -n $i";"
    ./cap $i 
done

echo "FPS: 30"
v4l2-ctl -d /dev/video0 -p 30 
for i in `seq 0 1000 500000`; do
    echo -n $i";"
    ./cap $i 
done
