import numpy as np
import os

def create_pgm_with_border(filename, width, height, border_size, maxval=255):
    # Get the directory of the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, filename)
    
    # Create an array of size width x height with initial value maxval
    data = np.full((height, width), maxval, dtype=np.uint8)
    
    # Add a black border (value 0)
    data[:border_size, :] = 0  # Top border
    data[-border_size:, :] = 0  # Bottom border
    data[:, :border_size] = 0  # Left border
    data[:, -border_size:] = 0  # Right border

    # Write to PGM file
    with open(filepath, 'wb') as f:
        # Write PGM file header (P5 indicates binary format)
        f.write(bytearray(f"P5\n{width} {height}\n{maxval}\n", 'raw_unicode_escape'))
        # Write image data
        data.tofile(f)

    # Print save information
    print(f"PGM file saved as {filepath}")

# Create a 200x200 PGM file with a 2-pixel black border
create_pgm_with_border('simple.pgm', 200, 200, border_size=2)

