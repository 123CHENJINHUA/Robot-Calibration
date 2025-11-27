import cv2

# index = 0
# while True:
#     cap = cv2.VideoCapture(index)
#     if not cap.read()[0]:
#         print(f"Camera with index {index} is not available.")
#     else:
#         print(f"Camera with index {index} is available.")
#         cap.release()
#     index += 1
#     if index > 10:  # Limit to first 10 indices for practicality
#         break



cap = cv2.VideoCapture(8)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break