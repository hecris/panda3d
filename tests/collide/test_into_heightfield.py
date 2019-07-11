from collisions import *
from panda3d.core import PNMImage

def test_ray_into_heightfield():
    img = PNMImage(128, 128, 1)
    img.fill_val(0)
    heightfield = CollisionHeightfield(img)
    ray = CollisionRay((0, 0, 0), (1, 1, 1))
    entry, np_from, np_into = make_collision(ray, heightfield)
    # assert entry is not None
    assert entry is None

test_ray_into_heightfield()
