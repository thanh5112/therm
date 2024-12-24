import cv2
import numpy as np
import csv
import time

# First function: Create pseudo-color mapping with labels
def create_pseudo_color_mapping(min_temp, max_temp, colormap, num_intervals=256, label_file='temp_labels.csv'):
    # Create a linear range of temperatures
    temperatures = np.linspace(min_temp, max_temp, num_intervals)

    # Normalize the temperatures for color mapping (0-1)
    normalized_temps = (temperatures - min_temp) / (max_temp - min_temp)

    # Generate a colormap from OpenCV (jet, etc.)
    cmap = cv2.applyColorMap(np.uint8(normalized_temps*255), getattr(cv2, f'COLORMAP_{colormap.upper()}'))

    # Write the temperature and corresponding color mappings to a file
    with open(label_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Temperature', 'R', 'G', 'B'])
        for i, temp in enumerate(temperatures):
            b, g, r = cmap[i][0]  # Remember OpenCV uses BGR format
            writer.writerow([temp, r, g, b])  # Writing RGB format

    print(f"Pseudo-color mapping created and saved to {label_file}")

# Main program to execute the functions sequentially
if __name__ == "__main__":
    # Step 1: Input all variables
    colormap = input("Enter colormap (jet, etc.): ").strip()
    min_temp = float(input("Enter minimum temperature: "))
    max_temp = float(input("Enter maximum temperature: "))

    # Step 2: Create the pseudo-color mapping with temperature labels
    create_pseudo_color_mapping(min_temp, max_temp, colormap)
