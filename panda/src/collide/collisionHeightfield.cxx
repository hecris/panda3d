/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file CollisionHeightfield.cxx
 * @author hecris
 * @date 2019-07-01
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
#include <queue>

using std::min;
using std::max;
using std::queue;
using std::vector;

PStatCollector CollisionHeightfield::_volume_pcollector(
      "Collision Volumes:CollisionHeightfield");
PStatCollector CollisionHeightfield::_test_pcollector(
      "Collision Tests:CollisionHeightfield");
TypeHandle CollisionHeightfield::_type_handle;

CollisionHeightfield::
CollisionHeightfield(PNMImage &heightfield) {
  _heightfield = heightfield;
  setup_quadtree(1);
}

void CollisionHeightfield::
setup_quadtree(int subdivisions) {
  int nodes_count = 0;
  for (int i = 0; i <= subdivisions; i++) {
    nodes_count += pow(4, i);
  }
  QuadTreeNode nodes[nodes_count];
  nodes[0].area.min = {0, 0};
  nodes[0].area.max = {(float)_heightfield.get_read_x_size() - 1,
                       (float)_heightfield.get_read_y_size() - 1};
  QuadTreeNode parent;
  for (int i = 1; i < nodes_count; i += 4) {
    parent = nodes[(i-1) / 4];
    // NW Quadrant
    nodes[i].area.min = {parent.area.max[0] / 2,
                         parent.area.max[1] / 2};
    nodes[i].area.max = parent.area.max;
    // NE Quadrant
    nodes[i + 1].area.min = {parent.area.min[0],
                             parent.area.max[1] / 2};
    nodes[i + 1].area.max = {parent.area.max[0] / 2,
                             parent.area.max[1]};
    // SW Quadrant
    nodes[i + 2].area.min = {parent.area.max[0] / 2,
                             parent.area.min[1]};
    nodes[i + 2].area.max = {parent.area.max[0],
                             parent.area.max[1] / 2};
    // SE Quadrant
    nodes[i + 3].area.min = parent.area.min;
    nodes[i + 3].area.max = {parent.area.max[0] / 2,
                             parent.area.max[1] / 2};
  }

  int leaf_first_index = 0;
  for (int i = 1; i <= subdivisions - 1; i++) {
    leaf_first_index += pow(4, i);
  }

  QuadTreeNode node;
  QuadTreeNode child;
  for (int i = nodes_count - 1; i >= 0; i--) {
    node = nodes[i];
    float height_min = INT_MAX;
    float height_max = INT_MIN;
    if (i >= leaf_first_index) {
      for (int x = node.area.min[0]; x < node.area.max[0]; x++) {
        for (int y = node.area.min[1]; y < node.area.max[1]; y++) {
          float value = _heightfield.get_gray(x, y);
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
    nodes[i].height_min = height_min;
    nodes[i].height_max = height_max;
  }
  _nodes = nodes;
}

PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_ray(const CollisionEntry &entry) const {
  const CollisionRay *ray;
  DCAST_INTO_R(ray, entry.get_from(), nullptr);
  const LMatrix4 &wrt_mat = entry.get_wrt_mat();

  LPoint3 from_origin = ray->get_origin() * wrt_mat;
  LPoint3 from_direction = ray->get_direction() * wrt_mat;

  double t1, t2;
  queue<QuadTreeNode> q;
  /* vector<QuadTreeNode> intersected_nodes; */
  LPoint3 box_min, box_max;
  QuadTreeNode node = _nodes[0];
  q.push(node);
  while (!q.empty()) {
    node = q.front();
    q.pop();
    box_min = {node.area.min[0],
               node.area.min[1], node.height_min};
    box_max = {node.area.max[0],
               node.area.max[1], node.height_max};
    /* collide_cat.error() << box_min << '\n'; */
    /* collide_cat.error() << box_max << '\n'; */
    if (box_intersects_line(t1, t2, box_min, box_max,
                            from_origin, from_direction)) {
      if (t1 >= 0.0 || t2 >= 0.0) {
        PT(CollisionEntry) new_entry = new CollisionEntry(entry);
        return new_entry;
      }
    }
    break;
  }
  return nullptr;
}

bool CollisionHeightfield::
box_intersects_line(double &t1, double &t2,
                    const LPoint3 &box_min, const LPoint3 &box_max,
                    const LPoint3 &from, const LVector3 &delta) const {

  double tmin = -DBL_MAX;
  double tmax = DBL_MAX;

  for (int i = 0; i < 3; ++i) {
    PN_stdfloat d = delta[i];
    if (!IS_NEARLY_ZERO(d)) {
      double tmin2 = (box_min[i] - from[i]) / d;
      double tmax2 = (box_max[i] - from[i]) / d;
      if (tmin2 > tmax2) {
        std::swap(tmin2, tmax2);
      }
      tmin = std::max(tmin, tmin2);
      tmax = std::max(tmax, tmax2);

      if (tmin > tmax) {
        return false;
      }

    } else if (from[i] < box_min[i] || from[i] > box_max[i]) {
      // The line is parallel
      return false;
    }
  }

  t1 = tmin;
  t2 = tmax;
  return true;
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
  /* Todo: this seems to be incorrect */
  QuadTreeNode node = _nodes[0];
  LPoint3 box_min = {node.area.min[0], node.area.min[1],
                     node.height_min};
  LPoint3 box_max = {node.area.max[0], node.area.max[1],
                     node.height_max};
  return new BoundingBox(box_min, box_max);
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
