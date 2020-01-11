/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file collisionHeightfield.cxx
 * @author hecris
 * @date 2019-07-01
 */

#include "collisionHeightfield.h"
#include "collisionHandler.h"
#include "collisionEntry.h"
#include "cmath.h"
#include "config_collide.h"
#include "boundingBox.h"
#include "datagram.h"
#include "datagramIterator.h"
#include "bamReader.h"
#include "bamWriter.h"
#include <algorithm>

using std::min;
using std::max;
using std::vector;
using std::sort;

PStatCollector CollisionHeightfield::_volume_pcollector(
      "Collision Volumes:CollisionHeightfield");
PStatCollector CollisionHeightfield::_test_pcollector(
      "Collision Tests:CollisionHeightfield");
TypeHandle CollisionHeightfield::_type_handle;

/**
 *
 */
CollisionHeightfield::
CollisionHeightfield(PNMImage heightfield, PN_stdfloat max_height):
_heightfield(heightfield),
_max_height(max_height) {
  unsigned int x = heightfield.get_x_size();
  unsigned int y = heightfield.get_y_size();

  HeightfieldQuad* root = new HeightfieldQuad(LVecBase2(0, 0),
                                              LVecBase2(x, y));
  _quadtree = new QuadTree(root);
  _quadtree->subdivide(3);
  fill_quadtree_heights();
}

/**
 *
 */
void CollisionHeightfield::
fill_quadtree_heights() {
  HeightfieldQuad* node = nullptr;
  QuadTreeNode* child = nullptr;

  for (iterator iter = _quadtree->end(); iter-- != _quadtree->begin();) {
    PN_stdfloat height_min = DBL_MAX;
    PN_stdfloat height_max = DBL_MIN;

    node = static_cast<HeightfieldQuad*>(iter.get_current_node());

    if (iter.is_at_leaf()) {
      for (size_t x = node->get_min()[0]; x < node->get_max()[0]; x++) {
        for (size_t y = node->get_min()[1]; y < node->get_max()[1]; y++) {
          PN_stdfloat value = _heightfield.get_gray(x, y) * _max_height;
          height_min = min(height_min, value);
          height_max = max(height_max, value);
        }
      }
    }
    else {
      for (unsigned int i = 0; i < 4; i++) {
        iter.get_child_node(i, child);
        height_min = min(height_min, static_cast<HeightfieldQuad*>(child)->get_min_height());
        height_max = max(height_max, static_cast<HeightfieldQuad*>(child)->get_max_height());
      }
    }
    node->set_min_height(height_min);
    node->set_max_height(height_max);
  }
}


/**
 *
 */
CollisionHeightfield::
~CollisionHeightfield() {
    delete _quadtree;
    _quadtree = nullptr;
}

/**
 *
 */
PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_ray(const CollisionEntry &entry) const {
  return nullptr;
}

/**
 *
 */
PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_sphere(const CollisionEntry &entry) const {
  return nullptr;
}

/**
 *
 */
PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_box(const CollisionEntry &entry) const {
  return nullptr;
}

/**
 * Generic CollisionSolid member functions
 */
void CollisionHeightfield::
fill_viz_geom() {
}

CollisionSolid *CollisionHeightfield::
make_copy() {
  return new CollisionHeightfield(*this);
}

LPoint3 CollisionHeightfield::
get_collision_origin() const {
  return LPoint3(0, 0, 0);
}

PT(BoundingVolume) CollisionHeightfield::
compute_internal_bounds() const {
  LPoint3 min = {0, 0, 0};
  LPoint3 max = {_heightfield.get_x_size(),
                 _heightfield.get_y_size(), _max_height};

  return new BoundingBox(min, max);
}

PStatCollector &CollisionHeightfield::
get_volume_pcollector() {
  return _volume_pcollector;
}

PStatCollector &CollisionHeightfield::
get_test_pcollector() {
  return _test_pcollector;
}

TypedWritable *CollisionHeightfield::
make_CollisionHeightfield(const FactoryParams &params) {
  CollisionHeightfield *me = new CollisionHeightfield;
  DatagramIterator scan;
  BamReader *manager;

  parse_params(params, scan, manager);
  me->fillin(scan, manager);
  return me;
}

void CollisionHeightfield::
fillin(DatagramIterator &scan, BamReader *manager) {
}

void CollisionHeightfield::
register_with_read_factory() {
  BamReader::get_factory()->register_factory(get_class_type(), make_CollisionHeightfield);
}
