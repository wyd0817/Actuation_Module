import numpy as np
import os

def create_pgm_with_border_and_right_up_quarter(filename, width, height, border_size, maxval=255):
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

    # Add black top-right quarter
    quarter_height = height // 2
    quarter_width = width // 2
    data[:quarter_height, quarter_width:] = 0

    # Write to PGM file
    with open(filepath, 'wb') as f:
        # Write PGM file header (P5 indicates binary format)
        f.write(bytearray(f"P5\n{width} {height}\n{maxval}\n", 'ascii'))
        # Write image data
        data.tofile(f)

    # Print save information
    print(f"PGM file saved as {filepath}")

# Create a 200x200 PGM file with a 5-pixel black border and top-right quarter black
create_pgm_with_border_and_right_up_quarter('simple_with_right_up_quarter.pgm', 200, 200, border_size=5)
