
# coding: utf-8

# In[8]:

from wand.image import Image
from wand.display import display
def svg2png(svgscene):
    with Image(blob=svgscene) as svgimg:
        pngimg = svgimg.make_blob('png32')
        return pngimg

