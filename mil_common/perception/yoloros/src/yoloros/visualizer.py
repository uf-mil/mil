from utils.plots import plot_one_box


def load_visuals(weights):
    CLASSES = []
    COLORS = []
    if weights == "robosub24":
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
    return CLASSES, COLORS


def draw_detections(CLASSES, COLORS, detections, frame):
    processed_detections = []
    if detections:
        detections = detections[0]

        for x1, y1, x2, y2, conf, cls in detections:
            class_index = int(cls.cpu().item())
            try:
                class_name = CLASSES[class_index]
            except:
                print("ERROR: (Internal Error) Index out of range, please check")
                return None
            print(f"{class_name} => {conf}")
            plot_one_box(
                [x1, y1, x2, y2],
                frame,
                label=f"{CLASSES[class_index]}",
                color=COLORS[class_index],
                line_thickness=2,
            )
            processed_detections.append((x1, y1, x2, y2, class_name))

        return processed_detections, frame
    else:
        print("No Detections Made")
        return None
