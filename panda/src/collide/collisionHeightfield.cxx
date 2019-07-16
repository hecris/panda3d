#define MSG(s) collide_cat.error() << s << '\n';
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
#include <algorithm>

using std::min;
using std::max;
using std::queue;
using std::vector;
using std::sort;

PStatCollector CollisionHeightfield::_volume_pcollector(
      "Collision Volumes:CollisionHeightfield");
PStatCollector CollisionHeightfield::_test_pcollector(
      "Collision Tests:CollisionHeightfield");
TypeHandle CollisionHeightfield::_type_handle;

CollisionHeightfield::
CollisionHeightfield(PNMImage &heightfield, double max_height) {
  _heightfield = heightfield;
  setup_quadtree(4, max_height);
  /* for (int i = 0; i < _nodes_count; i++) { */
  /*   if ((_nodes[i].index - 1) % 4 == 0) collide_cat.error() << '\n'; */
  /*   collide_cat.error() << _nodes[i].index << ": (" */
  /*   << _nodes[i].area.min[0] << ',' */
  /*   << _nodes[i].area.min[1] << ") (" */
  /*   << _nodes[i].area.max[0] << ',' */
  /*   << _nodes[i].area.max[1] << ")\n"; */
  /* } */
}

void CollisionHeightfield::
setup_quadtree(int subdivisions, double max_height) {
  int nodes_count = 0;
  for (int i = 0; i <= subdivisions; i++) {
    nodes_count += pow(4, i);
  }
  QuadTreeNode *nodes = new QuadTreeNode[nodes_count];
  nodes[0].area.min = {0, 0};
  nodes[0].area.max = {(float)_heightfield.get_read_x_size(),
                       (float)_heightfield.get_read_y_size()};
  nodes[0].index = 0;
  QuadTreeNode parent;
  for (int i = 1; i < nodes_count; i += 4) {
    parent = nodes[(i-1) / 4];
    LVector2 sub_area = (parent.area.max - parent.area.min) / 2;
    // SE Quadrant
    nodes[i].area.min = parent.area.min;
    nodes[i].area.max = parent.area.min + sub_area;
    nodes[i].index = i;
    // SW Quadrant
    nodes[i + 1].area.min = {parent.area.min[0] + sub_area[0],
                             parent.area.min[1]};
    nodes[i + 1].area.max = nodes[i + 1].area.min + sub_area;
    nodes[i + 1].index = i + 1;
    // NE Quadrant
    nodes[i + 2].area.min = {parent.area.min[0],
                             parent.area.min[1] + sub_area[1]};
    nodes[i + 2].area.max = nodes[i + 2].area.min + sub_area;
    nodes[i + 2].index = i + 2;
    // NW Quadrant
    nodes[i + 3].area.min = parent.area.min + sub_area;
    nodes[i + 3].area.max = parent.area.max;
    nodes[i + 3].index = i + 3;
  }

  int leaf_first_index = 1;
  for (int i = 1; i <= subdivisions - 1; i++) {
    leaf_first_index += pow(4, i);
  }

  QuadTreeNode node;
  QuadTreeNode child;
  for (int i = nodes_count - 1; i >= 0; i--) {
    node = nodes[i];
    PN_stdfloat height_min = INT_MAX;
    PN_stdfloat height_max = INT_MIN;
    if (i >= leaf_first_index) {
      for (int x = node.area.min[0]; x < node.area.max[0]; x++) {
        for (int y = node.area.min[1]; y < node.area.max[1]; y++) {
          PN_stdfloat value = _heightfield.get_gray(x, y) * max_height;
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
  _nodes_count = nodes_count;
  _leaf_first_index = leaf_first_index;
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
  vector<QuadTreeIntersection> intersections;
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
    if (!line_intersects_box(t1, t2, box_min, box_max, from_origin,
                             from_direction) || (t1 < 0.0 && t2 < 0.0)) {
      continue;
    }

    if (node.index >= _leaf_first_index) {
      // is a leaf node
      if (t1 < 0.0) t1 = t2;
      QuadTreeIntersection intersection = {node.index, t1, t2};
      intersections.push_back(intersection);
    } else {
      int child_first_index = 4 * node.index + 1;
      q.push(_nodes[child_first_index]);
      q.push(_nodes[child_first_index + 1]);
      q.push(_nodes[child_first_index + 2]);
      q.push(_nodes[child_first_index + 3]);
    }
  }

  if (intersections.size() == 0) {
    return nullptr;
  }
  std::sort(intersections.begin(), intersections.end());

  PT(CollisionEntry) new_entry = new CollisionEntry(entry);
  for (int i = 0; i < intersections.size(); i++) {
    double t = std::min(intersections[i].tmin, intersections[i].tmax);
    new_entry->set_surface_point(from_origin + from_direction * t);
  }

  return new_entry;
}

bool CollisionHeightfield::
line_intersects_box(double &t1, double &t2,
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
      tmax = std::min(tmax, tmax2);

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


bool CollisionHeightfield::
line_intersects_triangle(double &t, const LPoint3 &from,
                         const LPoint3 &delta,
                         const Triangle &triangle) const {
  const PN_stdfloat EPSILON = 1.0e-7;
  PN_stdfloat a,f,u,v;
  LVector3 edge1, edge2, h, s, q;
  edge1 = triangle.p2 - triangle.p1;
  edge2 = triangle.p3 - triangle.p1;
  h = delta.cross(edge2);
  a = dot(edge1, h);
  if (a > -EPSILON && a < EPSILON) {
    // line parallel to triangle
    return false;
  }
  f = 1.0 / a;
  s = from - triangle.p1;
  u = f * dot(s, h);
  if (u < 0.0 || u > 1.0) {
    return false;
  }
  q = s.cross(edge1);
  v = f * dot(delta, q);
  if (v < 0.0 || u + v > 1.0) {
    return false;
  }
  t = f * dot(edge2, q);
  if (t > EPSILON) {
    return true;
  } else {
    return false;
  }
}

/*
 * Generic member functions
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
