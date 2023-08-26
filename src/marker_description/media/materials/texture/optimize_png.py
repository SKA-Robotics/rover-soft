
import os
import sys
import subprocess
from PIL import Image


def remove_unnecessary_chunks(img):
    # List of necessary chunks
    necessary_chunks = ["IHDR", "IDAT", "IEND"]

    # Remove other chunks
    for chunk in list(img.info.keys()):
        if chunk not in necessary_chunks:
            del img.info[chunk]
    return img


def optimize_image(input_path, output_path=None, force=False):
    # Load the image using PIL
    img = Image.open(input_path)

    # Convert to grayscale and then to 1-bit
    grayscale_img = img.convert("L")
    binary_img = grayscale_img.convert("1", dither=Image.NONE)

    # Remove unnecessary chunks
    optimized_img = remove_unnecessary_chunks(binary_img)

    # Save the image with advanced optimization and maximum compression level
    optimized_img.save(output_path if output_path else input_path,
                       "PNG", compress_level=9, optimize=True, bits=1)

    # Check if pngcrush is installed and use it for further optimization
    try:
        result = subprocess.run(
            ['pngcrush', '-s', '-ow', output_path if output_path else input_path], check=True, stdout=subprocess.PIPE)
    except subprocess.CalledProcessError as e:
        # pngcrush is not installed or there was an error in processing
        print("pngcrush was not detected. Install it to further optimize the image.")


def main():
    # Check for minimum number of arguments
    if len(sys.argv) < 2:
        print("Usage: python optimize_png.py <input_path> [output_path] [-f]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(
        sys.argv) > 2 and sys.argv[2] != "-f" else None
    force = True if "-f" in sys.argv else False

    # If input is a file
    if os.path.isfile(input_path):
        # If overwriting the original file
        if not output_path:
            if not force:
                confirm = input(
                    f"Do you really want to overwrite the original file {input_path}? (y/n): ")
                if confirm.lower() != 'y':
                    print("Operation aborted.")
                    sys.exit(0)
        optimize_image(input_path, output_path)

    # If input is a directory
    elif os.path.isdir(input_path):
        for root, dirs, files in os.walk(input_path):
            for file in files:
                if file.lower().endswith('.png'):
                    file_path = os.path.join(root, file)
                    # If overwriting the original file
                    if not output_path:
                        if not force:
                            confirm = input(
                                f"Do you really want to overwrite the original file {file_path}? (y/n): ")
                            if confirm.lower() != 'y':
                                continue
                    optimize_image(file_path)

    else:
        print(f"'{input_path}' is not a valid file or directory path.")
        sys.exit(1)

    print("Optimization completed.")


if __name__ == "__main__":
    main()
