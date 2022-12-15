import time
import os
import sys
sys.path.append('yolov5')

import cv2
import torch

# os.chdir("yolov5")
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import non_max_suppression
# os.chdir("..")


camera = cv2.VideoCapture(0)

FPS = 0.1
weights = "pts/Nov-20-2022_10_04_26_best.pt"
conf_thres = 0.25
iou_thres = 0.45
classes = None
agnostic_nms = False
max_det = 1000

txt_path = "demo"

model = DetectMultiBackend(weights)
model.warmup()
while True:

	t_start = time.time()
	ret, image = camera.read()	# image = cv2.imread("yolov5/data/images/ShuttleData1_mp4_787.jpg")
	# print(image.shape)
	# image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	image_t = torch.from_numpy(image.transpose(2, 0, 1))[None].float() / 255.0 
	pred = model(image_t)
	pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

	for det in pred:
		for *xyxy, conf, cls in reversed(det):
			print("xyxy: ", xyxy, "conf:", conf)
			# xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()  # normalized xywh
			# line = (cls, *xywh, conf) # label format
			# with open(f'{txt_path}.txt', 'a') as f:
			#     f.write(('%g ' * len(line)).rstrip() % line + '\n')
			break
		break

	xycenter = torch.stack(xyxy).reshape(2, 2).mean(dim=0)
	y_center = xycenter[1].item()
	print(xycenter)
	width = image.shape[1]
	command = (y_center - width / 2) * 100 / width + 150
	print(command)
	# with open('pi_input/kobuki_input.txt', 'a') as f:
	# 	f.write(str(int(command)) + "\n")

	t_end = time.time()

	delta_t = t_end - t_start
	print(f"Inference took {delta_t}s")
	if delta_t < 1 / FPS:
		time.sleep(1 / FPS - delta_t)

	rounded_xyxy = torch.stack(xyxy).view(1, 4).round().int()
	print(rounded_xyxy)
	image = cv2.circle(image, rounded_xyxy[0][:2].tolist(), radius=3, color=(0, 0, 255), thickness=-1)
	image = cv2.circle(image, rounded_xyxy[0][2:].tolist(), radius=3, color=(255, 0, 0), thickness=-1)
	cv2.imwrite("image0annotated.png", image)