"""
Created on Tue Feb  7 14:29:38 2023

@author: spot

TODO:
    - change port according to topic
"""

import argparse
import numpy as np
import cv2
from flask import Flask, render_template, Response
import time

app = Flask(__name__)


def parser():
    """
        Allowable arguments
    """
    parser = argparse.ArgumentParser(description="test")
    parser.add_argument('--img', action='store', type=str, nargs='+',help="the raw image as a row vector")
    parser.add_argument('--nrows', action='store', type=int)
    parser.add_argument('--p',type=int, help='the port number')
    parser.add_argument('--img_pth', type=str, help='the location of image')
    return parser.parse_known_args()



def gen_frames():
    global image_path
    while True:
        try:
            img = cv2.imread(image_path)
            ret, buffer = cv2.imencode('.jpg',img)
            print(type(buffer))
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'+frame+b'\r\n') 
            
        except:
            print('[ERROR] loading image')
        time.sleep(0.5)


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')


if __name__ == "__main__":
    args, _ = parser()
    
    global image_path
    image_path = args.img_pth
    app.run(debug=True, port=args.p)
