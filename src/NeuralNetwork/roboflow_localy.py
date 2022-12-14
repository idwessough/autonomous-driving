from roboflow import Roboflow
import cv2, time

rf = Roboflow(api_key="5jmh3Ibq6UjAVFvGxcop")
project = rf.workspace().project("projet-s9")
model = project.version(3).model
cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened():
    rval, frame = vc.read()
    

else:
    rval=False

fps=42 

while rval: 
    start_time = time.time()
    print(fps )
    key = cv2.waitKey(27)
    if key==27:
        break
    last, frame = vc.read() 
    # Viewing
    cv2.imshow("preview", frame)
    # write last
    cv2.imwrite("my_image.jpg", frame)
    # infer on a local video
    print([prediction for prediction in model.predict("my_image.jpg", confidence=40, overlap=30).json()["predictions"]])
    # Calculate frame rate in a second
    fps=1/(time.time()-start_time)
    

# visualize your prediction
# model.predict("your_image.jpg", confidence=40, overlap=30).save("prediction.jpg")

# infer on an image hosted elsewhere
# print(model.predict("URL_OF_YOUR_IMAGE", hosted=True, confidence=40, overlap=30).json())