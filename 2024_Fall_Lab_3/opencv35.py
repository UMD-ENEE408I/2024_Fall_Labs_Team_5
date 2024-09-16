import cv2


print("hello")
# make image grayscale
image = cv2.imread('images.jpeg')

if image is None:
    print("Error: Could not read the image. Check the file path.")


grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imshow(' regular image ', image)
cv2.imshow(' grayscale image ', grayscale)

# edge  detection 

edges = cv2.Canny(grayscale, 100, 200)

cv2.imshow('Edge Detection', edges)

# Haar 
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

faces = face_cascade.detectMultiScale(grayscale, 1.1, 5, 0, (30,30))

for (x, y, w, h) in faces:
	cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)

cv2.imshow('Face detection', image)

cv2.waitKey(0)  # Wait indefinitely for a key press
cv2.destroyAllWindows() 
