import torch
import os 
import numpy 
import cv2
import time

#from PIL import Image
from IPython.display import Image, display
# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5x')  # or yolov5n - yolov5x6, custom
# Images
analyze_folder = False
actual_path = os.getcwd() 
image_folder = os.path.join(actual_path, "images") # 'https://ultralytics.com/images/zidane.jpg'  # or file, Path, PIL, OpenCV, numpy, list  
image_path = os.path.join(image_folder, "video_input.jpg")
images = os.listdir(image_folder)
print(images)
cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened():
    rval, frame = vc.read() 
    cv2.imwrite(image_path, frame)
else:
    rval = False

fps = 42
while rval:
    start_time = time.time()
    results = model(image_path)
    results.print()
    cv2.putText(frame, f"fps: {int(fps)}", (21, 42), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 255), 2)
    cv2.imshow("preview", frame)
    rval, frame = vc.read()
    
    
    key = cv2.waitKey(20)
    fps = 1/(time.time()-start_time)
    if key == 27:
        break
 

if analyze_folder:
    for image in images:
        image_path = os.path.join(image_folder, image) 
        # Inference
        results = model(image_path)
        # Results
        print(results)
        results.show()  # or .show(), .save(), .crop(), .pandas(), etc. 