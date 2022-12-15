import time
import cv2

def main():
    # try:
    cap = cv2.VideoCapture(-1)
    #     print(cap)
    # except:
    #     cap = cv2.VideoCapture(1)
    #     print('using 1')
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    out = cv2.VideoWriter('output.avi', fourcc, 30.0, (640,  480))

    # time.sleep(60)
    t_start = time.time()
    a_ = 0
    while time.time() - t_start < 10:
        ret, frame = cap.read()
        a = int(time.time() - t_start)
        if a % 2 == 0 and a != a_: 
            print(time.time() - t_start)
            a_ = a
        out.write(frame)

    cap.release()
    out.release()

main()