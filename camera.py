from io import BytesIO
from time import sleep, sleep
import os

from picamera import PiCamera
from PIL import Image
from torchvision import transforms
import torch

os.chdir("yolov5")
from common.models import DetectMultiBackend
from utils.general import non_max_suppression
os.chdir("..")


FPS = 10
weights = "pth"
conf_thres = 0.25
iou_thres = 0.45
classes = None
agnostic_nms = False
max_det = 1000

txt_path = "demo"

model = DetectMultiBackend(weights)
model.warmup()


cvtr = transforms.PILToTensor()
# Create the in-memory stream
stream = BytesIO()
camera = PiCamera()
sleep(2)
while True:
	# Create the in-memory stream

	t_start = time.time()
	stream = BytesIO()
	camera.capture(stream, format='jpeg', use_video_port=True)
	# "Rewind" the stream to the beginning so we can read its content
	stream.seek(0)
	image = Image.open(stream)
	image_t = cvtr(image)[None]

	pred = model(image_t)
	pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

	det = next(pred)
	for *xyxy, conf, cls in reversed(det):
        # print("xyxy: ", xyxy)
        if save_txt:  # Write to file
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()  # normalized xywh
            line = (cls, *xywh, conf) # label format
            with open(f'{txt_path}.txt', 'a') as f:
                f.write(('%g ' * len(line)).rstrip() % line + '\n')


	t_end = time.time()

	delta_t = t_end - t_start
	print(f"Inference took {delta_t}s")
	if delta_t < 1 / FPS:
		time.sleep(1 / FPS - delta_t)



