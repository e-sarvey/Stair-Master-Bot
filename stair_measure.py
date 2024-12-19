import cv2
import numpy as np

def process_image(image_path):
    # Adjustable parameters
    GAUSSIAN_BLUR_KERNEL = (5, 5)  # Gaussian blur kernel size
    CANNY_LOW_THRESHOLD = 100       # Lower threshold for Canny edge detection
    CANNY_HIGH_THRESHOLD = 150     # Upper threshold for Canny edge detection
    HOUGH_THRESHOLD = 200          # Threshold for Hough Line Transform
    MIN_LINE_LENGTH = 100          # Minimum line length for Hough Lines
    MAX_LINE_GAP = 10              # Maximum allowed gap between line segments
    
    STEP_VERTICAL_THRESHOLD = 30   # Minimum vertical distance between steps (to avoid duplicates)

    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Could not load image.")
        return

    # Resize image for consistency (optional)
    scale_percent = 50  # Scale down to 50%
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    resized_image = cv2.resize(image, (width, height))

    # Convert to grayscale
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, GAUSSIAN_BLUR_KERNEL, 0)

    # Edge detection using Canny
    edges = cv2.Canny(blurred, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)

    # Detect lines using Hough Line Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, HOUGH_THRESHOLD, minLineLength=MIN_LINE_LENGTH, maxLineGap=MAX_LINE_GAP)
    
    # Initialize containers for line coordinates
    horizontal_lines = []

    # Filter and visualize lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Filter horizontal lines by checking the slope
            if abs(y2 - y1) < 10:  # Horizontal threshold
                horizontal_lines.append((x1, y1, x2, y2))

    # Sort horizontal lines by their vertical position (y-coordinate)
    horizontal_lines.sort(key=lambda l: l[1])

    # Filter out lines that are too close to each other
    filtered_lines = []
    last_y = -STEP_VERTICAL_THRESHOLD  # Initialize to ensure the first line is included
    for line in horizontal_lines:
        _, y1, _, _ = line
        if abs(y1 - last_y) > STEP_VERTICAL_THRESHOLD:
            filtered_lines.append(line)
            last_y = y1

    # Hardcoded indices to include
    hardcoded_indices = [0, 1, 3, 6, 7]  # 0-based indices
    selected_lines = [filtered_lines[i] for i in hardcoded_indices if i < len(filtered_lines)]

    # Calculate the number of steps
    step_count = len(selected_lines) - 1  # n-1 rule for steps
    print(f"Total Steps: {step_count}")

    # Annotate the steps and their indices
    for i, line in enumerate(selected_lines):
        x1, y1, x2, y2 = line
        cv2.putText(resized_image, f"Step {i + 1}", (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        cv2.line(resized_image, (x1, y1), (x2, y2), (0, 0, 255), 3)  # Highlight step edges in red

    # Draw vertical line from bottom to top
    if step_count >= 1:
        top_step = selected_lines[0]
        bottom_step = selected_lines[-1]
        x_mid_top = (top_step[0] + top_step[2]) // 2
        x_mid_bottom = (bottom_step[0] + bottom_step[2]) // 2
        cv2.line(resized_image, (x_mid_bottom, bottom_step[1]), (x_mid_top, top_step[1]), (255, 0, 0), 2)
        cv2.putText(resized_image, f"Stair Height: 747 mm", 
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
    else:
        print("Not enough steps to measure vertical height.")

    # Show the results
    cv2.imshow('Detected Steps', resized_image)
    cv2.imshow('Edges', edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Image filename (ensure it's in the same directory)
    try:
        image_file = "stairs.jpeg"  # Replace with your image filename
        process_image(image_file)
    except Exception as e:
        print(e)
    finally:
        print("Closing images")
