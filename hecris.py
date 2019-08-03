from panda3d.core import *
from direct.directbase import DirectStart
load_prc_file_data("", "notify-level-collide debug")
base.cam.set_y(-20)

# Set up heightfield
img = PNMImage(17, 17, 1)
img.set_gray_val(1, 1, 100)
img.set_gray_val(2, 2, 111)
img.set_gray_val(3, 2, 90)

# Render terrain
terrain = GeoMipTerrain("mySimpleTerrain")
terrain.setHeightfield(img)
terrain.setBruteforce(True)
terrain.get_root().set_sz(10)
terrain.getRoot().reparentTo(render)

texture = loader.loadTexture('ground.jpg')
terrain.get_root().set_texture(texture)
terrain.generate()

from_solids = [
        # CollisionRay((3, 15, 1), (-.1, -.1, -.1))
        CollisionSphere((3, 14, 900/255 + 1), 1)
        # CollisionBox((3, 14, 900/255 + 1), 1, 1, 3)
        # CollisionSphere((0, 0, 0), 1)
        ]

into_solids = [
        CollisionHeightfield(img, 10, 3)
        ]

into_node = CollisionNode("into")
for solid in into_solids:
    into_node.add_solid(solid)

into_node.set_into_collide_mask(1)
into_np = render.attach_new_node(into_node)
into_np.show()

from_np_parent = render.attach_new_node("from-parent")

from_node = CollisionNode("from")
for solid in from_solids:
    from_node.add_solid(solid)

from_node.set_from_collide_mask(1)
from_np = from_np_parent.attach_new_node(from_node)
from_np.show()

handler = CollisionHandlerEvent()

base.cTrav = CollisionTraverser()
base.cTrav.show_collisions(base.render)
base.cTrav.add_collider(from_np, handler)

def update(task):
    if base.mouseWatcherNode.has_mouse():
        pos = base.mouseWatcherNode.get_mouse()
        from_np_parent.set_pos(pos[0] * 20, 0, pos[1] * 20)
    return task.cont

# base.taskMgr.add(update)

def up():
    pos = from_np.get_pos()
    pos[1] += 1
    from_np.set_pos(pos)

def down():
    pos = from_np.get_pos()
    pos[1] -= 1
    from_np.set_pos(pos)

def left():
    pos = from_np.get_pos()
    pos[0] -= 1
    from_np.set_pos(pos)

def right():
    pos = from_np.get_pos()
    pos[0] += 1
    from_np.set_pos(pos)

base.accept('arrow_up', up)
base.accept('arrow_down', down)
base.accept('arrow_left', left)
base.accept('arrow_right', right)

base.run()
=======
from panda3d.core import *
from direct.directbase import DirectStart
load_prc_file_data("", "notify-level-collide debug")
base.cam.set_y(-20)

# Set up heightfield
img = PNMImage(17, 17, 1)
img.set_gray_val(1, 1, 100)

# Render terrain
terrain = GeoMipTerrain("mySimpleTerrain")
terrain.setHeightfield(img)
terrain.setBruteforce(True)
terrain.get_root().set_sz(10)
terrain.getRoot().reparentTo(render)

texture = loader.loadTexture('ground.jpg')
terrain.get_root().set_texture(texture)
terrain.generate()

from_solids = [
        # CollisionRay((3, 15, 1), (-.1, -.1, -.1))
        CollisionSphere((3, 14, 900/255 + 1), 1)
        # CollisionBox((3, 14, 900/255 + 1), 1, 1, 3)
        # CollisionSphere((0, 0, 0), 1)
        ]

into_solids = [
        CollisionHeightfield(img, 10, 3)
        ]

into_node = CollisionNode("into")
for solid in into_solids:
    into_node.add_solid(solid)

into_node.set_into_collide_mask(1)
into_np = render.attach_new_node(into_node)
into_np.show()

from_np_parent = render.attach_new_node("from-parent")

from_node = CollisionNode("from")
for solid in from_solids:
    from_node.add_solid(solid)

from_node.set_from_collide_mask(1)
from_np = from_np_parent.attach_new_node(from_node)
from_np.show()

handler = CollisionHandlerEvent()

base.cTrav = CollisionTraverser()
base.cTrav.show_collisions(base.render)
base.cTrav.add_collider(from_np, handler)

def update(task):
    if base.mouseWatcherNode.has_mouse():
        pos = base.mouseWatcherNode.get_mouse()
        from_np_parent.set_pos(pos[0] * 20, 0, pos[1] * 20)
    return task.cont

# base.taskMgr.add(update)

def up():
    pos = from_np.get_pos()
    pos[1] += 1
    from_np.set_pos(pos)

def down():
    pos = from_np.get_pos()
    pos[1] -= 1
    from_np.set_pos(pos)

def left():
    pos = from_np.get_pos()
    pos[0] -= 1
    from_np.set_pos(pos)

def right():
    pos = from_np.get_pos()
    pos[0] += 1
    from_np.set_pos(pos)

def toggle_show():
        into_np.hide()

def toggle_show2():
        into_np.show()

def zup():
    pos = from_np.get_pos()
    pos[2] += 1
    from_np.set_pos(pos)

def zdown():
    pos = from_np.get_pos()
    pos[2] -= 1
    from_np.set_pos(pos)

base.accept('w', zup)
base.accept('s', zdown)
base.accept('arrow_up', up)
base.accept('arrow_down', down)
base.accept('arrow_down', down)
base.accept('arrow_left', left)
base.accept('arrow_right', right)
base.accept('t', toggle_show)
base.accept('r', toggle_show2)

base.run()
