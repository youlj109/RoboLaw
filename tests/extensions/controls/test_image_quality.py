from PIL import Image, ImageDraw

from kitchen_safety.extensions.controls.image_quality import is_near_uniform


def test_gray_background_is_rejected():
    assert is_near_uniform(Image.new('RGB', (1280, 720), (231, 231, 231)))


def test_textured_frame_is_not_blank():
    image = Image.new('RGB', (1280, 720), (231, 231, 231))
    ImageDraw.Draw(image).rectangle((0, 0, 640, 720), fill=(20, 40, 60))
    assert not is_near_uniform(image)
