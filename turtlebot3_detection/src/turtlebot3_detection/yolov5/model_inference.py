import time
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn
from typing import Tuple, List, Union, TypeVar

model = TypeVar('model')

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())  # add yolov5/ to path

from utils.general import non_max_suppression, scale_coords
from utils.augmentations import letterbox
from utils.plots import Annotator, colors
from utils.torch_utils import select_device

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

def load_model(weight_path: str ="") -> model:
    return torch.load(weight_path, map_location=device)["model"].float().fuse().eval() 

@torch.no_grad()
def inference(model: model, orig_img: np.ndarray, conf_thres: float=0.4, iou_thres: float=0.6, max_det:int=4, line_thickness: float=2) ->Tuple[List[List[Union[float, str]]], np.ndarray]:
    names = model.module.names if hasattr(model, 'module') else model.names
    img = cv2.resize(orig_img, (640,640))

    img = img[:,:,::-1].transpose(2,0,1)
    img = np.ascontiguousarray(img) 
    img = torch.from_numpy(img).to(device)
    img = img.float()
    img /= 255.0
    if len(img.shape) == 3:
        img = img.unsqueeze(0)

    pred = model(img, augment=False)[0]
    pred = non_max_suppression(pred, conf_thres=conf_thres, iou_thres=iou_thres, max_det=max_det)
    
    annotator = Annotator(orig_img, line_width=line_thickness, example=str(names))
    boxes_info = []
    for det in pred:
        if len(det):
            det[:, :4] /= 640.0
            det[:, [0,2]] = (det[:, [0,2]] * orig_img.shape[1]).round().clamp(0, orig_img.shape[1])
            det[:, [1,3]] = (det[:, [1,3]] * orig_img.shape[0]).round().clamp(0, orig_img.shape[0])
            for *xyxy, conf, cls in det:
                c = int(cls)
                label = f'{names[c]} {conf:.2f}'
                boxes_info.append([xyxy[0].item() / orig_img.shape[1], xyxy[1].item() / orig_img.shape[0], xyxy[2].item() / orig_img.shape[1], xyxy[3].item() / orig_img.shape[0], conf.item(), names[c]])
                annotator.box_label(xyxy, label, color=colors(c, True))
    return boxes_info, annotator.result()

# Run this in yolov5 directory with relative path
if __name__ == "__main__":
    model = load_model("../robot_model/best.pt")
    img = cv2.imread("../test.jpg")
    start_time = time.time()
    info, result = inference(model, img)
    print(time.time() - start_time)
    print(info)
    cv2.imwrite("../output.jpg", result)