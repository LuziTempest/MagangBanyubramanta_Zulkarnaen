import cv2
import numpy as np
import openvino.runtime as ov

# Label untuk model
labels = ["Flare", "Celana", "Baskom", "any"]
colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0)]

# Load model
core = ov.Core()
model = core.read_model("best.onnx")
compiled_model = core.compile_model(model, "CPU")

# Buka video
cap = cv2.VideoCapture("fourth.mp4")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Resize frame menjadi ukuran input model (640x640)
    input_frame = cv2.resize(frame, (640, 640))
    input_tensor = np.expand_dims(input_frame, axis=0).astype(np.float32)

    # Inference
    results = compiled_model(input_tensor)[compiled_model.output(0)]
    
    for det in results[0]:
        confidence = det[4]
        if confidence > 0.4:
            class_id = int(np.argmax(det[5:]))
            x, y, w, h = map(int, det[:4])

            # Gambar kotak dan label
            cv2.rectangle(frame, (x, y), (x + w, y + h), colors[class_id], 2)
            cv2.putText(frame, labels[class_id], (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[class_id], 2)

    # Tampilkan frame
    cv2.imshow("Deteksi", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
