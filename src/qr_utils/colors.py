"""
Utility procedures for manipulating colors
"""

from colormath.color_objects import LabColor, sRGBColor, HSVColor
from colormath.color_conversions import convert_color
from colormath.color_diff import delta_e_cie2000
import colornames

# colormath is using a deprecated numpy function
import numpy as np
# Add back np.asscalar if it's missing
if not hasattr(np, 'asscalar'):
    def asscalar(a):
        return a.item()
    np.asscalar = asscalar

def lab_hue_dist(lab1: LabColor, lab2: LabColor) -> float:
    return np.sqrt((lab1.lab_a - lab2.lab_a)**2 + (lab1.lab_b - lab2.lab_b)**2)
# Maybe the max value
lab_normalizer = lab_hue_dist(convert_color(sRGBColor(255, 0, 0, is_upscaled=True), LabColor),
                             convert_color(sRGBColor(0, 255, 0, is_upscaled=True), LabColor))
def rgb_dist_via_lab(rgb1 : tuple, rgb2 : tuple) -> float:
    """
    Calculate the distance between two RGB colors using the CIE76 delta E formula.
    :param rgb1: First RGB color as a tuple (R, G, B).
    :param rgb2: Second RGB color as a tuple (R, G, B).
    :return: The delta E value as a float.
    """
    rgb1 = sRGBColor(*rgb1, is_upscaled=True)
    rgb2 = sRGBColor(*rgb2, is_upscaled=True)
    
    # Convert RGB to LabColor
    lab1 = convert_color(rgb1, LabColor)
    lab2 = convert_color(rgb2, LabColor)
    
    # Calculate delta E
    #return delta_e_cie2000(lab1, lab2)

    # Actually, emphasize the hue components
    return lab_hue_dist(lab1, lab2) / lab_normalizer


#####################################################
#
#  Discrete color hacks

MAX_RGB = 2**8 - 1

RED = (255, 0, 0, 1)
GREEN = (0, 255, 0, 1)
MAGENTA = (255, 0, 255, 1)
CYAN = (0, 255, 255, 1)
BLUE = (0, 0, 255, 1)
BLACK = (0, 0, 0, 1)
WHITE = (255, 255, 255, 1)
BROWN = (140, 70, 20, 1)
GREY = (128, 128, 128, 1)
YELLOW = (255, 255, 0, 1)
TRANSPARENT = (0, 0, 0, 0)

ACHROMATIC_COLORS = {
    'white': WHITE,
    'grey': GREY,
    'black': BLACK,
}

CHROMATIC_COLORS = {
    'red': RED,
    'magenta' : MAGENTA,
    'cyan' : CYAN,
    'green': GREEN,
    'blue': BLUE,
    'yellow' : YELLOW,
    'brown' : BROWN,
}

DEFAULT_COLORS = CHROMATIC_COLORS | ACHROMATIC_COLORS

lo_val = 0.5 * 255
hi_val = 0.9 * 255
MIDDLING_COLORS = {
    'red': (hi_val, lo_val, lo_val, 1),
    'magenta' : (hi_val, lo_val, hi_val, 1),
    'cyan' : (lo_val, hi_val, hi_val, 1),
    'green': (lo_val, hi_val, lo_val, 1),
    'blue': (lo_val, lo_val, hi_val, 1),
    'yellow' : (hi_val, hi_val, lo_val, 1),
    'brown' : (128, 64, 25, 1),
    'grey' : (128, 128, 128, 1)
}

def remove_alpha(color):
    return color[:3]

COLOR_FROM_NAME = ACHROMATIC_COLORS | CHROMATIC_COLORS


def get_best_lab_color_name(rgb, color_from_name = MIDDLING_COLORS):
    # middling_colors is a lame attempt to account for some wide spectrum illumination
    return min(color_from_name, 
                  key=lambda n: get_lab_distance(color_from_name[n], rgb))

# Range 0 to 100
def get_lab_distance(c1, c2):
    return rgb_dist_via_lab(remove_alpha(c1), remove_alpha(c2))

# Range 0 to 1
def get_lab_similarity(c1, c2):
    return 1 - min(1.0, get_lab_distance(c1, c2)/100)

def get_best_color_name(rgb, **kwargs):
    r, g, b = remove_alpha(rgb)
    if 'color_from_name' in kwargs:
        return get_best_lab_color_name(rgb, color_from_name=kwargs['color_from_name'])
    else:
        name = colornames.find(int(r), int(g), int(b))
        alpha_name = ''.join(c for c in name if c.isalpha())
        # downcase the first character
        alpha_name = alpha_name[0].lower() + alpha_name[1:]
        return alpha_name
    
def get_color_distance(rgb1, rgb2):
    return get_lab_distance(rgb1, rgb2)
    
def get_color_similarity(rgb1, rgb2):
    return get_lab_similarity(rgb1, rgb2)

    






