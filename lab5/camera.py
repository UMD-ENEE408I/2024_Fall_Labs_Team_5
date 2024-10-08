import cv2

camera_index = 0
cap = cv2.VideoCapture(camera_index)

# load haar cascade
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

if not cap.isOpened():
    print("no video :(")
    exit()

while True:
    # get video
    ret, frame = cap.read()
    
    if not ret:
        print("no video :(")
        break


    # face detection
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(grayscale, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # draw rectangles around faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

    # display the resulting frame
    cv2.imshow('Live Face Detection', frame)

    # press q to stop the code
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
