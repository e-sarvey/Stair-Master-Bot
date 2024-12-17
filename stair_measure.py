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
    
    VERTICAL_HEIGHT_TOLERANCE = 20  # Tolerance to filter line pairs (vertical)

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
                cv2.line(resized_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw green lines

    # Sort horizontal lines by their vertical position (y-coordinate)
    horizontal_lines.sort(key=lambda l: l[1])

    # Find the top and bottom edges of the stair (if possible)
    if len(horizontal_lines) >= 2:
        top_line = horizontal_lines[0]
        bottom_line = horizontal_lines[-1]
        
        # Calculate vertical height
        vertical_height = abs(bottom_line[1] - top_line[1])
        print(f"Measured Vertical Height: {vertical_height} pixels")
        
        # Draw annotations for top and bottom edges
        cv2.line(resized_image, (top_line[0], top_line[1]), (top_line[2], top_line[3]), (0, 0, 255), 2)  # Red line for top
        cv2.line(resized_image, (bottom_line[0], bottom_line[1]), (bottom_line[2], bottom_line[3]), (255, 0, 0), 2)  # Blue line for bottom

        # Display measurement on the image
        cv2.putText(resized_image, f"Height: {vertical_height} px", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    else:
        print("Not enough horizontal lines detected to measure height.")

    # Show the results
    cv2.imshow('Detected Stairs and Height', resized_image)
    cv2.imshow('Edges', edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Image filename (ensure it's in the same directory)
    image_file = "stairs.jpeg"  # Replace with your image filename
    process_image(image_file)
