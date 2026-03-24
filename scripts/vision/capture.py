import cv2
import os

SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_images_benchy")
os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Camera opened. Press 's' to save an image, 'q' to quit.")

img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame.")
        break

    display = frame.copy()
    cv2.putText(display, f"Saved: {img_count} | S=Save  Q=Quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("Calibration Capture", display)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        filename = os.path.join(SAVE_DIR, f"calib_{img_count:03d}.jpg")
        cv2.imwrite(filename, frame)
        img_count += 1
        print(f"Saved: {filename}")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"Done. {img_count} images saved to: {SAVE_DIR}")