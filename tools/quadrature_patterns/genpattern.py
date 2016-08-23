#!/usr/bin/env python3

import math
from PIL import Image, ImageDraw

# Configure your quadrature pattern here.
dpi = 300
wheel_width_inches = 0.91
wheel_diameter_inches = 2.73
stripes_per_track = 20
stripe_color = (0, 0, 0)

# Calcuate the size of the image given our physical dimensions and our DPI.
width_px = math.ceil(wheel_width_inches * dpi)
height_px = math.ceil(wheel_diameter_inches * math.pi * dpi)
stripe_width_px = width_px / 2
stripe_height_px = height_px / (2 * stripes_per_track)

# Create a new image with the proper dimensions.
im = Image.new("RGB", (width_px, height_px), (255, 255, 255))
d = ImageDraw.Draw(im)

# Draw first track.
for i in range(0, stripes_per_track):
    start = i * 2 * stripe_height_px
    d.rectangle([(0, start), (stripe_width_px, start + stripe_height_px)], fill=stripe_color)

# Draw second track.
for i in range(0, stripes_per_track):
    start = i * 2 * stripe_height_px + (stripe_height_px / 2)
    d.rectangle([(stripe_width_px, start), (2 * stripe_width_px, start + stripe_height_px)], fill=stripe_color)

# Save the image with the desired DPI.
im.save("pattern.png", dpi=(dpi, dpi))
