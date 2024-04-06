# Object Mapping


## TensorRT Engien Setup

```bash
git clone -b v7.0 https://github.com/ultralytics/yolov5.git
# create conda envs and install requierments.txt for running gen_wts.py
# stupid scripts

git clone -b yolov5-v6.0 https://github.com/wang-xinyu/tensorrtx.git
cd yolov5/

wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt

cp [PATH-TO-TENSORRTX]/yolov5/gen_wts.py .
python gen_wts.py -w yolov5s.pt -o yolov5s.wts
# A file 'yolov5s.wts' will be generated.

cd [PATH-TO-TENSORRTX]/yolov5/
# Update kNumClass in src/config.h if your model is trained on custom dataset
mkdir build
cd build
cp [PATH-TO-ultralytics-yolov5]/yolov5s.wts . 
cmake ..
make

# Generate engine file (engine include 80 class of coco dataset)
./yolov5_det -s yolov5s.wts yolov5s.engine s

# Build and serialize TensorRT engine
./yolov5_seg -s yolov5s-seg.wts yolov5s-seg.engine s

```

# TODO

- [x] Add ObjectDatabase
- [x] Convert Object to Map
- [x] Publish Polygon
- [x] Test with ZED 
- [x] Add launch file
- [ ] Test with robot

