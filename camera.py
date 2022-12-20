import time
import os
import sys
sys.path.append('yolov5')

import serial
import cv2
import torch
import numpy as np
import shutil
import pickle

# os.chdir("yolov5")
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import non_max_suppression
# os.chdir("..")


camera = cv2.VideoCapture(-1)

# Try to connect to all the serial ports that might have the arduino

# connected = False

# try:
# 	print("Trying connection to port0")
# 	ser=serial.Serial('/dev/ttyACM0',9600)
# 	print("Connection to 0 successful!")
# 	connected = True
# except:
# 	print("Failed port 0")

# if connected == False:
# 	try:
# 		print("Trying connection to port1")
# 		ser=serial.Serial('/dev/ttyACM1',9600)
# 		print("Connection to 1 successful!")
# 		connected = True
# 	except:
# 		print("Failed port 1")

# if connected == False:
#     print("Connection Ffailed")

FPS = 1
weights = "pts/Dec-15-2022_14_36_52_best.pt"
conf_thres = 0.25
iou_thres = 0.45
classes = None
agnostic_nms = False
max_det = 1000
eps = 5

DEAD_COUNT = 2
IMG_LOGS = "image_logs"

txt_path = "demo"


model = DetectMultiBackend(weights)
model.warmup()

dead_counter = 0
bird_dead = False
prev_threshold = 0 
threshold = 30

width = 640

# function that calculates distance with two points
def dist(x1, y1, x2, y2):
	return sqrt((x1 - x2)**2 + (y1 - y2)**2)

prev_center = (0, 0)


def highest_conf_pred(pred):
	max_conf = -float('inf')
	best_xyxy = None
	for det in pred:
		for *xyxy, conf, cls in reversed(det):
			if conf > max_conf:
				conf = max_conf
				best_xyxy = torch.stack(xyxy).numpy()
	return best_xyxy


def do_inference(model, camera):
	ret, image = camera.read()
	if image is None:
		raise ValueError("Failed to capture image")
	image_t = torch.from_numpy(image.transpose(2, 0, 1))[None].float() / 255.0
	
	with torch.no_grad():
		pred = model(image_t)

	pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

	return image, pred


def get_center(box):
	return box.reshape(2, 2).mean(axis=0)

def sleep_cycle(t):
	dt = time.time() - t
	if dt < 1 / FPS:
		time.sleep(1 / FPS - dt)

def write_cmd(command):
	with open('pi_input/kobuki-input.txt', 'w') as f:
		f.write(str(command) + '\n')

def read():
	with open('pi_input/camera-input.txt', 'r') as f:
		line = f.readline()
	return line[:-1] if len(line) > 0 else ''

def main():
	prev_center = np.array([0, 0])
	dead_counter = 0
	active = False
	dead_bird = False
	# kobuki_ready = False
	steps = 1

	# Remove logs for the images if it already exists, then make it
	if os.path.exists(IMG_LOGS):
		shutil.rmtree(IMG_LOGS)
	os.mkdir(IMG_LOGS)

	# Where writing to kobuki_input.txt starts
	while True:
		t_start = time.time()
		print(steps, end=' ')
		steps += 1
		mode_cmd = read() # reads from camera-input.txt
		# 	with open('pi_input/camera-input.txt', 'r') as f:
		# line = f.readline()

		# if the bird is not found / dead?
		# if kobuki_ready == False:
		if not active:
			if mode_cmd == '2': # what is mode 2 signals Kobuki is ready
				active = True
			else:
				print("no start signal")
				sleep_cycle(t_start) # sleep for 1 cycle
				continue 
		# elif mode_cmd == '3':
		# 	active = False
		# 	print('going inactive')
		# 	sleep_cycle(t_start)
		# 	continue

		# Taking a frame from the camera and check if it is able to do so
		# If image capture is successful, do inference
		try:
			image, pred = do_inference(model, camera)
		except ValueError:
			print("camera failed")
			sleep_cycle(t_start)
			continue
		
		box = highest_conf_pred(pred)
		# so around every 5 steps, regardless if there's bird detected, write image with bounding box if image is there
		if (steps - 1) % 5 == 0:
			if box is not None:
				image = cv2.circle(image, box[:2].round().astype(int).tolist(), radius=3, color=(0, 0, 255), thickness=-1)
				image = cv2.circle(image, box[2:].round().astype(int).tolist(), radius=3, color=(255, 0, 0), thickness=-1)
			cv2.imwrite(os.path.join(IMG_LOGS, f"{steps - 1}.png"), image)
		if box is None:
			print("no bird found")
			dead_counter = 0
			sleep_cycle(t_start)
			continue
		center = get_center(box)

		# Determining if the bird is dead
		if not dead_bird:
			change = np.linalg.norm(center - prev_center)
			if dead_counter == 0:
				prev_center = center
				dead_counter += 1
				print("bird counter: ", dead_counter)
				sleep_cycle(t_start)
				continue
			elif change < eps:
				dead_counter += 1
				if dead_counter < DEAD_COUNT:
					prev_center = center
					print("bird counter: ", dead_counter)
					sleep_cycle(t_start)
					continue
			else:
				dead_counter = 0
				prev_center = np.array([0, 0])
				print("bird moved:", change)
				sleep_cycle(t_start)
				continue
			dead_bird = True
		
		# if the bird is dead for an amount of time, start feedback control
		if dead_counter == DEAD_COUNT:
			# Initialize the Kobuki to start moving
			write_cmd(0) 
			# Starting the Arduino
			# ser.write(bytearray(range(1)))
			dead_counter = 0
			prev_center = np.array([0, 0])
			print("initiate movement")
			sleep_cycle(t_start)
			continue
		# area = abs((box[2] - box[0]) * (box[3] - box[1]))
		# #use this if sensor doesn't work 
		# if area / np.prod(image.shape) > 0.5:
		# 	active = False
		# 	print("going inactive")
		# 	write_cmd(1)
		# 	sleep_cycle(t_start)
		# 	continue

		# Reading the latest message sent by the arduino:
		a=0
		print(os.getcwd())
		try:
			with open('pi_input/arduino-input.pickle', 'rb') as handle:
				a = pickle.load(handle)
		except:
			print("file cannot be opened")
		print("a = ", a)
		# If the robot is close to the shuttle, sees 1 from Arduino
		if a == 2 or a == 1:
			print("Bird is within range, move forward a little more")
			write_cmd(150)

			# reading the latest message sent by the arduino
			while(a == 2):
				try:
					with open('pi_input/arduino-input.pickle', 'rb') as handle:
						a = pickle.load(handle)
				except:
					print("file cannot be opened")

				# Wait for a bit
				time.sleep(0.1)
			
			# By this point, robot should have closed the cage. Indicate for robot to go back to starting point
			print("Signalling Kobuki to return.")
			write_cmd(1)
			active = False
			print("going inactive")
			# Notify Kobuki that bird is close
			
			sleep_cycle(t_start)
			break
			continue

		# if ser.readline()[:-1] == '3':
		# 	active = False
		# 	print("going inactive")
		# 	write(1)
		# 	sleep_cycle(t_start)
		# 	continue
		command = int(round((center[0] - width / 2) * 100 / width + 150))
		write_cmd(command)
		print("moving toward bird:", box, center, command)
		sleep_cycle(t_start)
		



# # Thresholding to determine if bird is dead is here
# while not bird_dead:

# 	# Inference
# 	ret, image = camera.read()	# image = cv2.imread("yolov5/data/images/ShuttleData1_mp4_787.jpg")
# 	image_t = torch.from_numpy(image.transpose(2, 0, 1))[None].float() / 255.0 
# 	pred = model(image_t)
# 	pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

# 	for det in pred:
# 		for *xyxy, conf, cls in reversed(det):
# 			if ((curr_diff - prev_center) <= threshold):

# 	# Increment counter
# 	if (dead_counter< 3 and thres <):
# 		dead_counter += 1
# 	else:
# 		bird_dead = True


# while True:

# 	t_start = time.time()
# 	ret, image = camera.read()	# image = cv2.imread("yolov5/data/images/ShuttleData1_mp4_787.jpg")
# 	# print(image.shape)
# 	# image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# 	image_t = torch.from_numpy(image.transpose(2, 0, 1))[None].float() / 255.0 
# 	pred = model(image_t)
# 	pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

# 	for det in pred:
# 		for *xyxy, conf, cls in reversed(det):
# 			print("xyxy: ", xyxy, "conf:", conf)
# 			# xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()  # normalized xywh
# 			# line = (cls, *xywh, conf) # label format
# 			# with open(f'{txt_path}.txt', 'a') as f:
# 			#     f.write(('%g ' * len(line)).rstrip() % line + '\n')
# 			break
# 		break

# 	xycenter = torch.stack(xyxy).reshape(2, 2).mean(dim=0)
# 	y_center = xycenter[1].item()
# 	print(xycenter)
# 	width = image.shape[1]
# 	command = (y_center - width / 2) * 100 / width + 150
# 	print(command)

# 	# Writing to the Kobuki file
# 	# with open('pi_input/kobuki_input.txt', 'a') as f:
# 	# 	f.write(str(int(command)) + "\n")


		

# 	t_end = time.time()

# 	delta_t = t_end - t_start
# 	print(f"Inference took {delta_t}s")
# 	if delta_t < 1 / FPS:
# 		time.sleep(1 / FPS - delta_t)

# 	rounded_xyxy = torch.stack(xyxy).view(1, 4).round().int()
# 	print(rounded_xyxy)
# 	image = cv2.circle(image, rounded_xyxy[0][:2].tolist(), radius=3, color=(0, 0, 255), thickness=-1)
# 	image = cv2.circle(image, rounded_xyxy[0][2:].tolist(), radius=3, color=(255, 0, 0), thickness=-1)
# 	cv2.imwrite("image0annotated.png", image)

if __name__ == '__main__':
	main()