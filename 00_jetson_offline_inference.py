# This script is used to test inference on a Jetson device using a pre-existing image and pre-trained DINOv2 (base) model.

import os
import json
from transformers import AutoImageProcessor, AutoModel
import torch
from PIL import Image

test_image_path = "data/raster/sample_images/bitterroots.jpg"

# Ensure device is CUDA if available
device = torch.device("mps")
print(f"Using device: {device}")

# Load the DINOv2 model and processor
processor = AutoImageProcessor.from_pretrained("facebook/dinov2-base")
model = AutoModel.from_pretrained("facebook/dinov2-base").to(device)

# Load and preprocess the image
image = Image.open(test_image_path)
inputs = processor(images=image, return_tensors="pt")
inputs = {k: v.to(device) for k, v in inputs.items()}

# Perform inference
with torch.no_grad():
    model.eval()  # Set the model to evaluation mode
    outputs = model(**inputs)

# Convert outputs to CPU and to list for JSON serialization
outputs_cpu = {}
for k, v in outputs.items():
    if isinstance(v, torch.Tensor):
        outputs_cpu[k] = v.cpu().tolist()
    else:
        outputs_cpu[k] = v

# Create results directory if it doesn't exist
results_dir = "results"
os.makedirs(results_dir, exist_ok=True)

# Save outputs to a JSON file
output_file_path = os.path.join(results_dir, "offline_inference.json")
with open(output_file_path, 'w') as f:
    json.dump(outputs_cpu, f, indent=4)

print(f"Results saved to {output_file_path}")