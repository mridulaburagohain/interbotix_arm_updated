from PIL import Image
import depth_pro
import numpy as np
import torch

# Load model and preprocessing transform
model, transform = depth_pro.create_model_and_transforms()
model.eval()

# Load and preprocess an image.
image_path= "/home/mridula/Pictures/RPi_images/new_12_23_2024/cali1.jpg"
image, _, f_px = depth_pro.load_rgb(image_path)
image = transform(image)

predicted_focal_length = 6162.5132  # Predicted focal length from model
actual_focal_length = 1752.92     # Actual focal length from camera specifications

# Calculate the scale factor
scale_factor = actual_focal_length / predicted_focal_length

# Correct the focal length using the scale factor
corrected_focal_length = predicted_focal_length * scale_factor

focal_length_tensor = torch.tensor(actual_focal_length).float()

# Now use this corrected focal length for depth prediction
print(f"Corrected focal length: {actual_focal_length}")

# Run inference.
prediction = model.infer(image, f_px=focal_length_tensor)
depth = prediction["depth"]  # Depth in [m].
focallength_px = prediction["focallength_px"]  # Focal length in pixels.

np.save("depth_map_12_23_4.npy", depth)  # Save depth as .npy file

print("Depth map saved as 'depth_map.npy'")
print(focallength_px)
