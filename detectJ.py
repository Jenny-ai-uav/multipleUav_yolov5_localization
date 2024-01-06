import argparse
import os
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized
import airsim

def detect(opt):
    source, view_img, imgsz ,Isstream= opt.source, opt.view_img, opt.img_size,opt.stream
    weights=r'.\v5lite-s.pt'
    conf_thres=0.45
    iou_thres=0.5
    classes=0
    agnostic_nms=True
    airsim_frame=opt.airsim_frame
    #save_img = not opt.nosave and not source.endswith('.txt')  # save inference images

    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    if half:
        model.half()  # to FP16

    # Set Dataloader
    dataset = LoadImages(source, img_size=imgsz, stride=stride, isStream=Isstream, AirFrame=airsim_frame)
    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    dic = []
    for path, img, im0s, vid_cap in dataset:
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        pred = model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)
            s += '%gx%g ' % img.shape[2:]  # print string
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    if True:  # Add bbox to image
                        label = f'{names[int(cls)]} {conf:.2f}'
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                        p1,p2=(int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                        dic.append([str(label),p1,p2])
            cv2.imshow('detect_', im0)
            cv2.waitKey(1)
        return dic

def parserOpt(frame):
    parser = argparse.ArgumentParser()
    parser.add_argument('--airsim_frame', default=frame)
    parser.add_argument('--stream', nargs='+', type=bool, default=True)
    parser.add_argument('--weights', nargs='+', type=str, default='v5lite-s.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='sample', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    #parser.add_argument('--conf-thres', type=float, default=0.45, help='object confidence threshold')
    #parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    #parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    #parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    #parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    #parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    #parser.add_argument('--update', action='store_true', help='update all models')
    #parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    #parser.add_argument('--name', default='exp', help='save results to project/name')
    #parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    return opt
def main(frame):
    parser = argparse.ArgumentParser()
    parser.add_argument('--airsim_frame', default=frame)
    parser.add_argument('--stream', nargs='+', type=bool, default=True)
    #parser.add_argument('--weights', nargs='+', type=str, default='v5lite-s.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='sample', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=1024, help='inference size (pixels)')
    #parser.add_argument('--conf-thres', type=float, default=0.45, help='object confidence threshold')
    #parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    #parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    #parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    #parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    #parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    opt=parser.parse_args()
    result=detect(opt)
    print(result)
    return result

UavName='Drone1'
CAMERA_NAME = '0'
# 鏡頭的位置0為front-center,3為bottom-center
IMAGE_TYPE = airsim.ImageType.Scene
if __name__=="__main__":
    #check_requirements(exclude=('pycocotools', 'thop'))
    client = airsim.MultirotorClient()
    while True:
        rawImage = client.simGetImage(CAMERA_NAME, airsim.ImageType.Scene, vehicle_name=UavName)
        frame_default = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_COLOR)
        #opt=parserOpt(frame_default)
        main(frame_default)