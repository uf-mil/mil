from __future__ import annotations

import os

import numpy as np
import torch
from models.experimental import attempt_load
from PIL import Image
from torchvision import transforms
from utils.general import non_max_suppression
from utils.plots import plot_one_box
from visualizer import load_visuals


class Detector:
    def __init__(self, weights):
        if weights == "robosub24":
            print("Weights loaded for Robosub24")
            self.model_name = weights
            absolute_file_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "weights/robosub24.pt"),
            )
            self.__MODEL = attempt_load(
                absolute_file_path,
                map_location=torch.device("cuda"),
            )
            self.__CLASSES, self.__COLORS = load_visuals(weights)
        else:
            print("Invalid Model")

    def test_detection(self, conf_thres, iou_thres=0.5):
        image_path = ""
        if self.model_name == "robosub24":
            image_path = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "tests/RoboSub-2023-Dataset-Cover_png.rf.ffda25ca7a57ac74ee37bc85707cf784.jpg",
                ),
            )
            self.__MODEL.eval()
        img = Image.open(image_path).convert("RGB")

        img_transform = transforms.Compose([transforms.ToTensor()])

        img_tensor = img_transform(img).to("cuda").unsqueeze(0)
        pred_results = self.__MODEL(img_tensor)[0]
        detections = non_max_suppression(
            pred_results,
            conf_thres=conf_thres,
            iou_thres=iou_thres,
        )

        arr_image = np.array(img)

        if detections:
            detections = detections[0]
            for x1, y1, x2, y2, conf, cls in detections:
                class_index = int(cls.cpu().item())
                print(f"{self.__CLASSES[class_index]} => {conf}")
                plot_one_box(
                    [x1, y1, x2, y2],
                    arr_image,
                    label=f"{self.__CLASSES[class_index]}",
                    color=self.__COLORS[class_index],
                    line_thickness=2,
                )
        else:
            print("No Detections Made")

        Image.fromarray(arr_image).show()


if __name__ == "__main__":
    Detector("robosub24").test_detection(conf_thres=0.77)
