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

# Extract the 768 features from the pooler_output
pooler_output_tensor = outputs.pooler_output

# Squeeze to remove the batch dimension (e.g., from (1, 768) to (768,)) then convert to CPU and list
features_list = pooler_output_tensor.squeeze().cpu().tolist()

# Create results directory if it doesn't exist
results_dir = "results"
os.makedirs(results_dir, exist_ok=True)

# Save the extracted features to a JSON file
output_file_path = os.path.join(results_dir, "offline_inference.json")
with open(output_file_path, 'w') as f:
    json.dump(features_list, f, indent=4)

print(f"Extracted 768 features saved to {output_file_path}")