import torch
import torchvision.models.detection as detection
import onnx

model = torch.load('./drone_detection_ssd_model.pth', map_location=torch.device('cpu'))
model.eval()

dummy_input = torch.randn(1, 3, 320, 320)

onnx_model_path = "./drone.onnx"
torch.onnx.export(
    model,
    dummy_input,
    onnx_model_path,
    verbose=True,
    opset_version=11,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={
        'input': {0: 'batch_size'},
        'output': {0: 'batch_size'}
    }
)

print(f"Model has been converted to ONNX and saved at {onnx_model_path}")