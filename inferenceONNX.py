import numpy as np
import cv2
import onnxruntime as ort
from pdb import set_trace
import matplotlib.pyplot as plt

# Load and preprocess the image
def preprocess_image(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
    img = cv2.resize(img, (320, 320))
    img = img.astype(np.float32)
    img = img / 255.0  # Normalize to [0, 1]
    tensor = np.transpose(img, (2, 0, 1))  # Change data layout to (C, H, W)
    tensor = np.expand_dims(tensor, axis=0)  # Add batch dimension
    return img, tensor

def visualize_results(image, boxes, scores, labels, threshold):
    for box, score, label in zip(boxes, scores, labels):
        if score < threshold:
            continue
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(image, f'{label}: {score:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
    plt.imshow(image)
    plt.axis('off')
    plt.show()

model= './ssdlite320_mobilenet_v3_large.onnx'
#Preprocess the image
img = cv2.imread('person.jpg')
ort_session = ort.InferenceSession(model)

input_data = np.random.rand(1, 3, 320, 320).astype(np.float32)  # Example input shape

# Get the name of the input and output layers
input_name = ort_session.get_inputs()[0].name
img, input_data = preprocess_image(img)
output_name = ort_session.get_outputs()[0].name

# Run inference 
outputs = ort_session.run(None, {input_name: input_data})


boxes = outputs[0]  # Example output
scores = outputs[1]  # Example output
labels = outputs[2]  # Example output
# set_trace()

visualize_results(img, boxes, scores, labels, threshold=0.9)