import numpy as np
import cv2
import onnxruntime as ort
import torch
from torchvision import transforms
# from torchvision.models import 
from pdb import set_trace
import matplotlib.pyplot as plt
import torchvision
# from torchvision.models.detection import ssdlite320_mobilenet_v3_large 

# Load and preprocess the image
def preprocess_image(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.imshow(img)
    img = cv2.resize(img, (320, 320))
    # img = img.astype(np.float32)
    # img = img / 255.0  # Normalize to [0, 1]
    # img = np.transpose(img, (2, 0, 1))  # Change data layout to (C, H, W)
    # img = np.expand_dims(img, axis=0)  # Add batch dimension
    
    preprocess = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    input_tensor = preprocess(img)
    # Add a batch dimension
    input_tensor = input_tensor.unsqueeze(0)
    
    return img, input_tensor

def visualize_results(image, boxes, scores, labels, threshold):
    for box, score, label in zip(boxes, scores, labels):
        if score < threshold:
            continue
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(image, f'{label}: {score:.2f}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
    plt.imshow(image)
    plt.axis('off')
    plt.show()

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model = models.mobilenet_v2(pretrained=False)
path = './drone_detection_ssd_model.pth'
model.load_state_dict(torch.load(path))
model.eval() 

#Preprocess the image
img = cv2.imread('drone.jpg')

img, inp_tensor = preprocess_image(img)

with torch.no_grad():
    outputs = model(inp_tensor)

set_trace()
boxes = outputs[0]  # Example output
scores = outputs[1]  # Example output
labels = outputs[2]  # Example output

visualize_results(img, boxes, scores, labels, threshold=0.2)