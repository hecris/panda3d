from collisions import *

def test_line_into_heightfield():
    heightfield = CollisionHeightfield()
    line = CollisionLine((0, 0, 0), (1, 1, 1))
    entry, np_from, np_into = make_collision(line, heightfield)
    assert entry is not None
