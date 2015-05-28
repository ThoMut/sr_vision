#!/usr/bin/env python

from PIL import Image

from sr_object_segmentation import SrObjectSegmentation


class ColorSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon a color segmentation
    """

    def __init__(self, image):
        """
        Initialize the color segmentation object with the color chosen to segment as parameter
        @param image - image to be segmented (numpy format)
        """
        SrObjectSegmentation.__init__(self, image, {})
        self.name = 'Color segmentation algorithm'
        self.points = self.segmentation()
        self.nb_segments = len(self.points)

    def segmentation(self):
        """
        Segment the image according to the color given as parameter
        @return - dictionary of segments found with points coordinates
        """
        width = self.img.shape[0]
        height = self.img.shape[1]

        dic = {}
        main_colors = get_main_color(self.img, 6)
        for i, color in enumerate(main_colors):
            pts = []
            for x in range(width):
                for y in range(height):
                    if list(self.img[(x, y)]) == list(color):
                        pts.append((x, y))
            dic[i] = pts

        # Sort by descending size of segments
        seg_by_length = sorted(dic.values(), key=len, reverse=True)  # Remove the background (basic and noise test)
        dic = {}
        for i in range(len(seg_by_length)):
            dic[i] = seg_by_length[i]
        return dic


def get_main_color(np_img, max_nb_col):
    """
    Get the main colors present in an image
    @param np_img: numpy image
    @param max_nb_col: number of maximum colors to be returned
    @return: a list of the main colors present in the image (RGB format)
    """
    pil_img = Image.fromarray(np_img)
    colors = pil_img.getcolors(pil_img.size[0] * pil_img.size[1])
    sorted_colors = sorted(colors, key=lambda col: col[0:], reverse=True)
    if len(sorted_colors) < max_nb_col:
        nb_col = len(sorted_colors)
    else:
        nb_col = max_nb_col
    return [sorted_colors[:][i][1] for i in range(nb_col)]