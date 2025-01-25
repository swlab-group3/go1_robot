from PIL import Image, ImageDraw, ImageFont
import os
import random

# Expanded list of fonts
font_paths = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",  # DejaVu Sans Bold
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",       # DejaVu Sans
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf",         # Ubuntu Bold
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",         # Ubuntu Regular
    "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf",  # Times New Roman
    "/usr/share/fonts/truetype/libreoffice/FreeSerif.ttf",   # FreeSerif
]

# Create an image with a number and varying font sizes and styles
def create_image_with_number(number, font_size, font_path):
    # Set the image size and background color
    width, height = 640, 640  # Updated image size
    background_color = (255, 255, 255)  # white background

    # Create a new image with white background
    img = Image.new('RGB', (width, height), background_color)

    # Set up the font and drawing object
    font = ImageFont.truetype(font_path, font_size)
    draw = ImageDraw.Draw(img)

    # Convert the number to a string to calculate the size
    number_str = str(number)

    # Get the size of the text using textbbox (bounding box of the text)
    text_width, text_height = draw.textbbox((0, 0), number_str, font=font)[2:4]

    # Calculate the position to center the text
    position = ((width - text_width) // 2, (height - text_height) // 2)

    # Add the number text to the image
    draw.text(position, number_str, fill=(0, 0, 0), font=font)

    # Create bounding box (left, top, right, bottom)
    bbox = (position[0], position[1], position[0] + text_width, position[1] + text_height)

    return img, bbox

# Create directories to store images and labels
def create_directory_structure():
    if not os.path.exists("images"):
        os.makedirs("images")
    if not os.path.exists("labels"):
        os.makedirs("labels")

# Generate images and labels
def generate_images_and_labels():
    create_directory_structure()

    numbers = [0, 1, 2, 3, 4, 5]
    class_ids = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}

    for number in numbers:
        for version in range(10):  # Create 10 versions for each number
            # Randomly select font size and font
            font_size = random.randint(50, 200)  # Font sizes from 50 to 300 for scaling
            font_path = random.choice(font_paths)

            # Generate image and bounding box
            img, bbox = create_image_with_number(number, font_size, font_path)

            # Save the image
            img_path = f"images/{number}_{version}.png"
            img.save(img_path)

            # Save the label
            label_path = f"labels/{number}_{version}.txt"
            with open(label_path, 'w') as label_file:
                # Class ID for the number
                class_id = class_ids[number]
                # Normalize bounding box values
                img_width, img_height = img.size
                x_center = (bbox[0] + bbox[2]) / 2 / img_width
                y_center = (bbox[1] + bbox[3]) / 2 / img_height
                width = (bbox[2] - bbox[0]) / img_width
                height = (bbox[3] - bbox[1]) / img_height
                # Write label in YOLO format
                label_file.write(f"{class_id} {x_center} {y_center} {width} {height}\n")

            # Optionally display the image
            # img.show()

# Run the script
generate_images_and_labels()

