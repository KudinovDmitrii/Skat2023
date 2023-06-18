import socket
import time

import cv2
import torch

from multiprocessing import Process, Queue, Manager


def input_file(data):
    HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
    PORT = 1024  # Port to listen on (non-privileged ports are > 1023) 65432
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            conn, addr = s.accept()
            with conn:
                data[1] = conn.recv(1024)


def main():
    manager = Manager()  # create only 1 mgr
    Num_video_L = manager.dict()  # create only 1 dict
    video_get = Process(target=input_file, args=(Num_video_L,), )
    video_get.start()
    model = torch.hub.load(
        "yolov5",
        'custom',
        path="Ml/best.pt",
        source='local')

    complete = []
    last_line = []
    Frame_L = []
    while True:
        if not Num_video_L:
            continue
        Num_video_S = Num_video_L[1].decode("utf-8")
        Num_video_l = Num_video_S.split(",")
        if complete:
            for element in complete:
                Num_video_l.remove(element)
        if not Num_video_l:
            continue
        cap = cv2.VideoCapture(f"log/{Num_video_l[0]}.avi")
        Number_frame_L = list(range(0, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), 15))
        Number_frame_L.append(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - 1)
        for iter_frame in Number_frame_L:
            cap.set(cv2.CAP_PROP_POS_FRAMES, iter_frame)
            res, frame = cap.read()
            frame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
            # frame = frame.copy()[..., ::-1]
            frame = cv2.resize(frame, (640, 640))
            Frame_L.append(frame)
        results = model(Frame_L, size=640)  # , size=320
        results.print()
        for iter_results in range(0, len(results.pandas().xyxy), 1):
            if results.xyxy[iter_results].size()[0] == 0:
                continue
            if float(results.pandas().xyxy[iter_results]["confidence"].iloc[0]) < 0.3:
                continue
            Frame_L[iter_results] = cv2.cvtColor(Frame_L[iter_results].copy(), cv2.COLOR_RGB2BGR)
            for i in range(results.xyxy[iter_results].size()[0]):
                cv2.rectangle(Frame_L[iter_results], (int(results.xyxy[iter_results][i][0]),
                                                      int(results.xyxy[iter_results][i][1])),
                              (int(results.xyxy[iter_results][i][2]),
                               int(results.xyxy[iter_results][i][3])), (255, 0, 0), 2)
            cv2.imwrite(f"{iter_results}.png", Frame_L[iter_results])
        complete.append(Num_video_l[0])
        cap.release()


if __name__ == '__main__':
    main()
