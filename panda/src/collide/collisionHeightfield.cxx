/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file CollisionHeightfield.cxx
 * @author drose
 * @date 2000-06-22
 */

#include "collisionHeightfield.h"
#include "collisionHandler.h"
#include "collisionEntry.h"
#include "cmath.h"
#include "config_collide.h"
#include "boundingBox.h"
#include "geom.h"
#include "geomNode.h"
#include "datagram.h"
#include "datagramIterator.h"
#include "bamReader.h"
#include "bamWriter.h"
#include "geom.h"
#include "geomLinestrips.h"
#include "geomVertexWriter.h"

using std::min;
using std::max;

PStatCollector CollisionHeightfield::_volume_pcollector(
      "Collision Volumes:CollisionHeightfield");
PStatCollector CollisionHeightfield::_test_pcollector(
      "Collision Tests:CollisionHeightfield");
TypeHandle CollisionHeightfield::_type_handle;

CollisionHeightfield::
CollisionHeightfield(PNMImage &heightfield) {
  _heightfield = heightfield;
  /* setup_quadtree(); */
}

void CollisionHeightfield::
setup_quadtree(int subdivisions) {
  int nodes_count = 0;
  for (int i = 0; i < subdivisions; i++) {
    nodes_count += pow(4, i);
  }
  QuadTreeNode nodes[nodes_count];
  QuadTreeNode node = nodes[0];
  node.area.min = {0, 0};
  node.area.max = {(float)_heightfield.get_read_x_size(),
                   (float)_heightfield.get_read_y_size()};
  QuadTreeNode parent;
  for (int i = 1; i < nodes_count; i += 4) {
    parent = nodes[(i-1) / 4];
    // NW Quadrant
    node = nodes[i];
    node.area.min = {parent.area.max[0] / 2,
                     parent.area.max[1] / 2};
    node.area.max = parent.area.max;
    // NE Quadrant
    node = nodes[i + 1];
    node.area.min = {parent.area.min[0],
                     parent.area.max[1] / 2};
    node.area.max = {parent.area.max[0] / 2,
                     parent.area.max[1]};
    // SW Quadrant
    node = nodes[i + 2];
    node.area.min = {parent.area.max[0] / 2,
                     parent.area.min[1]};
    node.area.max = {parent.area.max[0],
                     parent.area.max[1] / 2};
    // SE Quadrant
    node = nodes[i + 3];
    node.area.min = parent.area.min;
    node.area.max = {parent.area.max[0] / 2,
                     parent.area.max[1] / 2};
  }

  int leaf_first_index = 0;
  for (int i = 1; i < subdivisions - 1; i++) {
    leaf_first_index += pow(4, i);
  }

  QuadTreeNode child;
  for (int i = nodes_count - 1; i >= 0; i--) {
    node = nodes[i];
    double height_min = DBL_MAX;
    double height_max = DBL_MIN;
    if (i >= leaf_first_index) {
      for (int x = node.area.min[0]; x < node.area.max[0]; x++) {
        for (int y = node.area.min[1]; y < node.area.max[1]; y++) {
          double value = _heightfield.get_gray(x, y);
          height_min = std::min(value, height_min);
          height_max = std::max(value, height_max);
        }
      }
    } else {
      for (int c = (i * 4) + 1, max = c + 4; c < max; c++) {
        child = nodes[c];
        height_min = std::min(child.height_min, height_min);
        height_max = std::max(child.height_max, height_max);
      }
    }
    node.height_min = height_min;
    node.height_max = height_max;
  }
  _nodes = nodes;
}

PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_line(const CollisionEntry &entry) const {
  PT(CollisionEntry) new_entry = new CollisionEntry(entry);
  return new_entry;
}

/*
 * Generic member functions
 */
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
  return new BoundingBox(LPoint3(0, 0, 0), LPoint3(1, 1, 1));
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
