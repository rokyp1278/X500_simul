#!/usr/bin/env python3
"""Receive the Gazebo camera UDP H264 stream via OpenCV/GStreamer and report.

Saves one frame to /tmp/cam_frame.png so we can eyeball what the drone sees.
"""
import sys
import time
import cv2

PIPE = ("udpsrc port=5600 caps=\"application/x-rtp,media=video,"
        "encoding-name=H264,payload=96\" ! rtph264depay ! avdec_h264 ! "
        "videoconvert ! appsink sync=false drop=true max-buffers=1")

cap = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("RESULT: cannot open gstreamer pipeline"); sys.exit(1)

t0 = time.time()
got = 0
frame = None
while time.time() - t0 < 20:
    ok, f = cap.read()
    if ok and f is not None:
        got += 1
        frame = f
        if got >= 15:
            break
    else:
        time.sleep(0.05)

if frame is not None:
    cv2.imwrite("/tmp/cam_frame.png", frame)
    print("RESULT: received %d frames, shape=%s, saved /tmp/cam_frame.png"
          % (got, frame.shape))
else:
    print("RESULT: opened pipeline but received 0 frames in 20s")
cap.release()
