import cv2
from ultralytics import YOLO
import timeit

# load the YOLOv8 model
model = YOLO('yolo11n.pt')

camera_index = 0 
cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print("camera no work :(")
    exit()

while True:
    ret, frame = cap.read()
    frame_resize = cv2.resize(frame, (640, 640))
    if not ret:
        print("camera no work :(")
        break
    start = timeit.timeit()
    results = model(frame_resize)
    end = timeit.timeit()
    print(end - start)
    frame_out = results[0].plot()
    # loop thru detected objects
    # for result in results:
    #     boxes = result.boxes
    #     for box in boxes:
    #         # check the box
    #         if box.xyxy.shape[0] >= 1:
    #             x1, y1, x2, y2 = box.xyxy[0]  # coordinates
    #             conf = box.conf[0]
    #             print(box.xyxy[0])
    #             print(box)
                

    #             label = f"{model.names[int(box.cls)]} {conf}"  # label the guess

    #             # draw box and label on the frame
    #             cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
    #             cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # display
    cv2.imshow('YOLOv8 Object Detection', frame_out)

    # press q to end function
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# were done
cap.release()
cv2.destroyAllWindows()
