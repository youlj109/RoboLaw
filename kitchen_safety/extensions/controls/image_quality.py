"""Conservative blank-image diagnostic, separate from semantic video review."""
from PIL import ImageStat


def spatial_stddev(image):
    return max(ImageStat.Stat(image.convert('RGB').resize((160, 90))).stddev)


def is_near_uniform(image, threshold=2.):
    return spatial_stddev(image) < threshold
