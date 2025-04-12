import os
import cv2

# Set input and output paths manually
input_dir = "/fs/ess/PAS2119/Self_Collected_Data/datasets/Year_4/Mcity_raw/rosbag2_2024_06_03_Construction_Reverse/image"                  # Folder containing .png images
output_path = "video.mp4"            # Output video file name
fps = 10                              # Frames per second

# Get sorted list of .png files
image_files = sorted([f for f in os.listdir(input_dir) if f.endswith(".png")])
if not image_files:
    print("❌ No PNG files found in the directory.")
    exit()

# Read first image to get frame size
first_img = cv2.imread(os.path.join(input_dir, image_files[0]))
height, width, _ = first_img.shape

# Initialize video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

# Write each image to video
for img_name in image_files:
    img_path = os.path.join(input_dir, img_name)
    img = cv2.imread(img_path)
    out.write(img)

out.release()
print(f"✅ Video saved to: {output_path}")
