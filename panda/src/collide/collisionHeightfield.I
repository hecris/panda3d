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

#include <cmath>

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
  child1 = new HeightfieldQuad(_min, center);
  child2 = new HeightfieldQuad(center, _max);
  child3 = new HeightfieldQuad(_min, center);
  static_cast<HeightfieldQuad*>(child3)->get_min()[1] = center[1];
  static_cast<HeightfieldQuad*>(child3)->get_max()[1] = _max[1];
  child4 = new HeightfieldQuad(center, _max);
  static_cast<HeightfieldQuad*>(child4)->get_min()[0] = center[0];
  static_cast<HeightfieldQuad*>(child4)->get_max()[0] = _max[0];
}

/**
 *
 */
INLINE PN_stdfloat HeightfieldQuad::
get_min_height() const {
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
get_max_height() const {
  return _height_max;
}

/**
 *
 */
INLINE LVecBase2& HeightfieldQuad::
get_min() {
  return _min;
}

/**
 *
 */
INLINE LVecBase2& HeightfieldQuad::
get_max() {
  return _max;
}

/**
 *
 */
INLINE PN_stdfloat HeightfieldQuad::
get_area() const {
  return std::abs(_min[0] - _max[0]) *
         std::abs(_min[1] - _max[1]);
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
