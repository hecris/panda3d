/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file collisionHeightfield.I
 * @author hecris
 * @date 2019-07-01
 */

/**
 *
 */
INLINE HeightfieldQuad::
HeightfieldQuad(LVecBase2 min, LVecBase2 max):
_min(min), _max(max), _height_min(0),_height_max(0)
{}

/**
 *
 */
INLINE void HeightfieldQuad::
make_children(QuadTreeNode* &child1, QuadTreeNode* &child2,
              QuadTreeNode* &child3, QuadTreeNode* &child4) {
  LVecBase2 center = (_min + _max) / 2;
  child1 = new HeightfieldQuad(center, _min);
  child2 = new HeightfieldQuad(center, _max);
  std::swap(_min[0], _max[0]);
  child3 = new HeightfieldQuad(center, _min);
  child4 = new HeightfieldQuad(center, _max);
}

/**
 *
 */
INLINE PN_stdfloat HeightfieldQuad::
get_min_height() {
  return _height_min;
}

/**
 *
 */
INLINE void HeightfieldQuad::
set_min_height(PN_stdfloat height_min) {
  _height_min = height_min;
}

/**
 *
 */
INLINE PN_stdfloat HeightfieldQuad::
get_max_height() {
  return _height_max;
}

/**
 *
 */
INLINE void HeightfieldQuad::
set_max_height(PN_stdfloat height_max) {
  _height_max = height_max;
}

/**
 *
 */
INLINE CollisionHeightfield::
CollisionHeightfield() {
}

/**
 *
 */
INLINE CollisionHeightfield::
CollisionHeightfield(const CollisionHeightfield &copy) {
}

/**
 *
 */
INLINE void CollisionHeightfield::
flush_level() {
  _volume_pcollector.flush_level();
  _test_pcollector.flush_level();
}
