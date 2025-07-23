# This script tests inference on a Jetson device using a live camera feed and DINOv2 (base) model.
import os
import time
import json
import torch
import cv2
from PIL import Image
from transformers import AutoImageProcessor, AutoModel
import numpy as np

#Config
subject_dir = "subject"
threshold = 0.45
device = torch.device("cuda")

#Globals
seconds_between_captures = 1
subject_mean_embedding = None
model = None
processor = None

#Load DINOv2
def load_dino():
    global model, processor
    processor = AutoImageProcessor.from_pretrained("facebook/dinov2-base")
    model = AutoModel.from_pretrained("facebook/dinov2-base").to(device).eval()

# Extracts DINO features from an image
def extract_features(image):
    inputs = processor(images=image, return_tensors="pt")
    inputs = {k: v.to(device) for k, v in inputs.items()}
    with torch.no_grad():
        outputs = model(**inputs)
    return outputs.pooler_output.squeeze().cpu().numpy()

def generate_subject_mean_embedding():
    global subject_mean_embedding
    subject_embeddings = []
    for filename in os.listdir(subject_dir): 
        if filename.lower().endswith((".jpg", ".jpeg", ".png")):
            path = os.path.join(subject_dir, filename)
            image = Image.open(path).convert("RGB")
            features = extract_features(image)
            subject_embeddings.append(features)

    if not subject_embeddings:
        raise RuntimeError("No subject images found!")

    subject_mean_embedding = np.mean(subject_embeddings, axis=0)

def cosine_similarity(a, b):
    return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))

#Captures images continuously and classifies them as either containing the subject or not
def run_detection_loop():
    global seconds_between_captures
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Camera not connected - is it video device 0?")
    print("Starting Inference")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from camera.")
            time.sleep(seconds_between_captures)
            continue
        
        #Convert from OpenCV to Python PIL image
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb)

        #Do inference and determine if match
        query_embedding = extract_features(pil_image)
        similarity = cosine_similarity(query_embedding, subject_mean_embedding)
        #Print the similarity - in case you need to adjust the threshold
        print(f"Similarity: {similarity:.4f}")
        if similarity >= threshold:
            print(f"✅ Subject Detected")
        else:
            print("Subject not seen.")

        time.sleep(seconds_between_captures)

    cap.release()




if __name__ == "__main__":
    print("Starting...")
    load_dino()
    print("DINOv2 loaded onto CUDA device")
    generate_subject_mean_embedding()
    if(subject_mean_embedding is not None):
        print("Subject successfully classified")
        run_detection_loop()
    else:
        print("Failed to classify subject")

    
