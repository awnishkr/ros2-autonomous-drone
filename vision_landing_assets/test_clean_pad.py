from pathlib import Path
import cv2
import numpy as np
import onnxruntime as ort

folder = Path(__file__).resolve().parent

model_path = folder / "landing_detector.onnx"
image_path = folder / "clean_landing_pad.png"
output_path = folder / "detected_clean_pad.png"

image = cv2.imread(str(image_path))

if image is None:
    raise RuntimeError("Cannot read platform.png")

h, w = image.shape[:2]

# Resize with letterboxing
scale = min(640 / w, 640 / h)

nw = round(w * scale)
nh = round(h * scale)

resized = cv2.resize(image, (nw, nh))

canvas = np.full((640, 640, 3), 114, dtype=np.uint8)

left = (640 - nw) // 2
top = (640 - nh) // 2

canvas[top:top+nh, left:left+nw] = resized

# Prepare image for neural network
rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

tensor = (
    rgb.transpose(2, 0, 1)[None]
    .astype(np.float32) / 255.0
)

# Load pretrained model
session = ort.InferenceSession(
    str(model_path),
    providers=["CPUExecutionProvider"]
)

print("Neural network loaded")

# Run inference
predictions = session.run(
    None,
    {session.get_inputs()[0].name: tensor}
)[0][0].T

print("Maximum confidence:", float(predictions[:, 4].max()))

boxes = []
scores = []

for x, y, bw, bh, confidence in predictions:
    if confidence < 0.25:
        continue

    x1 = int(np.clip((x - bw / 2 - left) / scale, 0, w - 1))
    y1 = int(np.clip((y - bh / 2 - top) / scale, 0, h - 1))

    x2 = int(np.clip((x + bw / 2 - left) / scale, 0, w - 1))
    y2 = int(np.clip((y + bh / 2 - top) / scale, 0, h - 1))

    if x2 <= x1 or y2 <= y1:
        continue

    boxes.append([x1, y1, x2-x1, y2-y1])
    scores.append(float(confidence))

# Remove overlapping detections
indices = cv2.dnn.NMSBoxes(
    boxes, scores, 0.25, 0.45
) if boxes else []

count = 0

for i in np.array(indices, dtype=int).flatten():
    x, y, bw, bh = boxes[i]

    cv2.rectangle(
        image, (x, y), (x+bw, y+bh),
        (0, 255, 0), 2
    )

    cv2.putText(
        image,
        f"Landing Pad: {scores[i]:.2f}",
        (x, max(20, y-10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (0, 255, 0), 2
    )

    count += 1

cv2.imwrite(str(output_path), image)

print("Landing pads detected:", count)
print("Output image:", output_path)
print("INFERENCE COMPLETED")
