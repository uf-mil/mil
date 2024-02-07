import numpy as np
import torch
from models.experimental import attempt_load
from PIL import Image
from torchvision import transforms
from utils.general import non_max_suppression
from utils.plots import plot_one_box

# TODO: Place this file into the yolov7 folder when implementing it in mil_commmons

MODEL = attempt_load(
    "yolopy/runs/train/exp8/weights/last.pt",
    map_location=torch.device("cuda"),
)
CLASSES = [
    "buoy_abydos_serpenscaput",
    "buoy_abydos_taurus",
    "buoy_earth_auriga",
    "buoy_earth_cetus",
    "gate_abydos",
    "gate_earth",
]
COLORS = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 155, 0),
    (255, 0, 255),
    (0, 255, 255),
]
CONFIDENCE_THRESHOLD = 0.85

MODEL.eval()

image_path = "yolopy/tests/RoboSub-2023-Dataset-Cover_png.rf.ffda25ca7a57ac74ee37bc85707cf784.jpg"
img = Image.open(image_path).convert("RGB")

img_transform = transforms.Compose([transforms.ToTensor()])

img_tensor = img_transform(img).to("cuda").unsqueeze(0)
pred_results = MODEL(img_tensor)[0]
detections = non_max_suppression(pred_results, conf_thres=0.5, iou_thres=0.5)

arr_image = np.array(img)

if detections:
    detections = detections[0]
    for x1, y1, x2, y2, conf, cls in detections:
        class_index = int(cls.cpu().item())
        print(f"{CLASSES[class_index]} => {conf}")
        if conf < CONFIDENCE_THRESHOLD:
            continue
        plot_one_box(
            [x1, y1, x2, y2],
            arr_image,
            label=f"{CLASSES[class_index]}",
            color=COLORS[class_index],
            line_thickness=2,
        )
else:
    print("No Detections Made")

Image.fromarray(arr_image).show()
