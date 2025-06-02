from PIL import Image
import numpy as np

def read_image_to_binary_array(image_path, width=24, height=22, threshold=128):
    """
    Read image pixels directly and convert to binary array
    Args:
        image_path: Path to the image file
        width: Target width (24)
        height: Target height (22) 
        threshold: Pixel value threshold (128 = middle gray)
    Returns:
        2D list where black pixels = 1, white pixels = 0
    """
    # Open the image
    img = Image.open(image_path)
    print(f"Original image size: {img.size}")
    print(f"Original image mode: {img.mode}")
    
    # Convert to grayscale for consistent pixel reading
    if img.mode != 'L':
        img = img.convert('L')
        print("Converted to grayscale")
    
    # Resize to exact dimensions
    img = img.resize((width, height))
    print(f"Resized to: {img.size}")
    
    # Read pixels directly using getpixel()
    grid = []
    for y in range(height):
        row = []
        for x in range(width):
            # Get pixel value (0-255 for grayscale)
            pixel_value = img.getpixel((x, y))
            
            # Convert to binary: black (low values) = 1, white (high values) = 0
            binary_value = 1 if pixel_value < threshold else 0
            row.append(binary_value)
        grid.append(row)
    
    return grid

# Method 1: Direct pixel reading (recommended)
img_path = 'image.jpeg'  # Replace with actual filename
grid = read_image_to_binary_array(img_path, 24, 22)

# Print in requested format
print("grid = [")
for i, row in enumerate(grid):
    if i < len(grid) - 1:
        print(f"    {row},")
    else:
        print(f"    {row}")
print("]")

# Method 2: Using numpy for faster processing (alternative)
def numpy_method(image_path, width=24, height=22):
    img = Image.open(image_path).convert('L').resize((width, height))
    img_array = np.array(img)
    binary_array = (img_array < 128).astype(int)
    return binary_array.tolist()

# Method 3: Show pixel values for debugging
def debug_pixels(image_path, width=24, height=22):
    img = Image.open(image_path).convert('L').resize((width, height))
    print("Sample pixel values (first 5x5):")
    for y in range(min(5, height)):
        row_values = []
        for x in range(min(5, width)):
            row_values.append(img.getpixel((x, y)))
        print(f"Row {y}: {row_values}")

# Usage:
# grid = read_image_to_binary_array('your_uploaded_image.png', 24, 22)
# debug_pixels('your_uploaded_image.png', 24, 22)  # For debugging