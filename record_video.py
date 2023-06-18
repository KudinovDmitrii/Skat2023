import os
import cv2
import time
import socket

from multiprocessing import Process, Queue, Manager

delta_time = 10  # сколько секунд пишется видео

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
frame_size = (frame_width, frame_height)


def output_file(transfer):
    HOST = "127.0.0.1"  # The server's hostname or IP address
    PORT = 1024  # The port used by the server 65432
    transfer = str(transfer).replace(" ", "").replace("[", "").replace("]", "")
    print(transfer)
    transfer = bytes(transfer, 'utf-8')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(transfer)


def get_out_gps(multy_gps):
    i = 0
    multy_gps.append([0, 0, 0])
    multy_gps.append([0, 0, 0])
    multy_gps.append([0, 0])
    while True:
        i += 1
        if i > 255:
            i = 0
        gps1 = [0, 0, 0]  # x, y, h
        corners1 = [0, 0, 0]  # курс, тангаж, крен
        velocity1 = [i, 0]  # скорость по курсу, вертикальная скорость
        multy_gps[0], multy_gps[1], multy_gps[2] = gps1, corners1, velocity1


def main():
    itern_end = []
    manager = Manager()  # create only 1 mgr
    multy_gps = manager.list()  # create only 1 dict
    # multy_gps = [0, 0, 0]
    get_gps_proc = Process(target=get_out_gps, args=(multy_gps,))
    get_gps_proc.start()
    time.sleep(25)
    itern_video = 0
    while itern_video < 3:
        start_time_record = time.time()
        out = cv2.VideoWriter(f'log/{itern_video}.avi', cv2.VideoWriter_fourcc(*'XVID'), 30.0, frame_size)
        itern_frame = 0
        with open(f'log/{itern_video}.txt', 'w+') as log:
            while delta_time > (time.time() - start_time_record):
                _, frame = cap.read()
                out.write(frame)
                gps, corners, velocity = multy_gps[0], multy_gps[1], multy_gps[2]
                log.writelines([str(itern_frame), " ", str(gps).replace(" ", "").replace("[", "").replace("]", ""), " ",
                                str(corners).replace(" ", "").replace("[", "").replace("]", ""), " ",
                                str(velocity).replace(" ", "").replace("[", "").replace("]", ""),
                                " ", str(time.time())])
                log.write("\n")
                itern_frame += 1
        itern_end.append(itern_video)
        out.release()
        output_file(itern_end)
        itern_video += 1


if __name__ == '__main__':
    main()
    cap.release()
    print("done")
