from collisions import *

def test_ray_into_heightfield():
    heightfield = CollisionHeightfield()
    ray = CollisionRay((0, 0, 0), (1, 1, 1))
    entry, np_from, np_into = make_collision(ray, heightfield)
    assert entry is not None
