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
#include "datagram.h"
#include "datagramIterator.h"
#include "bamReader.h"
#include "bamWriter.h"
#include <queue>
#include <algorithm>
#define MSG(s) collide_cat.error() << s << '\n';

using std::min;
using std::max;
using std::queue;
using std::vector;
using std::sort;

CollisionHeightfield::
CollisionHeightfield(PNMImage &heightfield,
                     double max_height, int subdivisions) {
  _heightfield = heightfield;
  _max_height = max_height;
  setup_quadtree(subdivisions);
}

PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_ray(const CollisionEntry &entry) const {
  const CollisionRay *ray;
  DCAST_INTO_R(ray, entry.get_from(), nullptr);
  const LMatrix4 &wrt_mat = entry.get_wrt_mat();

  LPoint3 from_origin = ray->get_origin() * wrt_mat;
  LPoint3 from_direction = ray->get_direction() * wrt_mat;

  IntersectionParams params;
  params.from_origin = from_origin;
  params.from_direction = from_direction;
  vector<QuadTreeIntersection> intersections;
  intersections = find_intersections(line_intersects_box, params);

  if (intersections.size() == 0) {
    return nullptr;
  }
  std::sort(intersections.begin(), intersections.end());

  PT(CollisionEntry) new_entry = new CollisionEntry(entry);
  bool intersected = false;
  for (unsigned i = 0; i < intersections.size(); i++) {

    if (intersections[i].tmin < 0.0 && intersections[i].tmax < 0.0) continue;
    if (intersections[i].tmin < 0.0) intersections[i].tmin = intersections[i].tmax;

    LPoint3 p1 = from_origin + from_direction * intersections[i].tmin;
    LPoint3 p2 = from_origin + from_direction * intersections[i].tmax;
    // We use Bresenhaum's Line Algorithm to directly get the heightfield
    // coordinates that the line passes through.
    int m_new = 2 * (p2[1] - p1[1]);
    int slope_error_new = m_new - (p2[0] - p1[0]);

    for (int x = p1[0], y = p1[1]; x <= p2[0]; x++) {
      vector<Triangle> triangles = get_triangles(x, y);

      for (unsigned tri = 0; tri < triangles.size(); tri++) {
        double min_t = DBL_MAX;
        double t;
        if (line_intersects_triangle(t, from_origin, from_direction, triangles[tri])) {
          if (t < 0) continue;
          if (t < min_t) {
            intersected = true;
            min_t = t;
            new_entry->set_surface_point(from_origin + from_direction * t);
          }
        }
      }
      if (intersected) return new_entry;

      slope_error_new += m_new;
      if (slope_error_new >= 0) {
        y++;
        slope_error_new -= 2 * (p2[0] - p1[0]);
      }
    }
  }

  return nullptr;
}

PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_sphere(const CollisionEntry &entry) const {
  const CollisionSphere *sphere;
  DCAST_INTO_R(sphere, entry.get_from(), nullptr);

  CPT(TransformState) wrt_space = entry.get_wrt_space();
  const LMatrix4 &wrt_mat = wrt_space->get_mat();

  LPoint3 center = sphere->get_center() * wrt_mat;
  LVector3 radius_v = LVector3(sphere->get_radius(), 0.0f, 0.0f) * wrt_mat;
  PN_stdfloat radius_2 = radius_v.length_squared();
  PN_stdfloat radius = csqrt(radius_2);

  IntersectionParams params;
  params.center = center;
  params.radius = radius;

  vector<QuadTreeIntersection> intersections;
  intersections= find_intersections(sphere_intersects_box, params);

  if (intersections.size() == 0) return nullptr;

  PT(CollisionEntry) new_entry = new CollisionEntry(entry);
  LPoint3 point;
  bool intersected = false;
  for (unsigned i = 0; i < intersections.size(); i++) {
    QuadTreeNode node = _nodes[intersections[i].node_index];
    for (int x = node.area.min[0]; x < node.area.max[0]; x++) {
      for (int y = node.area.min[0]; y < node.area.max[1]; y++) {
        vector<Triangle> triangles = get_triangles(x, y);
        for (unsigned tri = 0; tri < triangles.size(); tri++) {
          if (sphere_intersects_triangle(point, center, radius, triangles[tri])) {
            intersected = true;
            new_entry->set_surface_point(point);
          }
        }
      }
    }
  }

  if (intersected) {
    return new_entry;
  } else {
    return nullptr;
  }

}

PT(CollisionEntry) CollisionHeightfield::
test_intersection_from_box(const CollisionEntry &entry) const {
  const CollisionBox *box;
  DCAST_INTO_R(box, entry.get_from(), nullptr);

  const LMatrix4 &wrt_mat = entry.get_wrt_mat();
  LPoint3 box_min = box->get_min() * wrt_mat;
  LPoint3 box_max = box->get_max() * wrt_mat;

  IntersectionParams params;
  params.box_min = box_min;
  params.box_max = box_max;

  vector<QuadTreeIntersection> intersections;
  intersections = find_intersections(box_intersects_box, params);

  if (intersections.size() == 0) {
    return nullptr;
  }

  PT(CollisionEntry) new_entry = new CollisionEntry(entry);
  LPoint3 point;
  bool intersected = false;
  for (unsigned i = 0; i < intersections.size(); i++) {
    QuadTreeNode node = _nodes[intersections[i].node_index];
    for (int x = node.area.min[0]; x < node.area.max[0]; x++) {
      for (int y = node.area.min[0]; y < node.area.max[1]; y++) {
        vector<Triangle> triangles = get_triangles(x, y);
        for (unsigned tri = 0; tri < triangles.size(); tri++) {
          if (box_intersects_triangle(box_min, box_max, triangles[tri])) {
            intersected = true;
            new_entry->set_surface_point(box_min);
          }
        }
      }
    }
  }

  if (intersected) {
    return new_entry;
  } else {
    return nullptr;
  }
}

bool CollisionHeightfield::
line_intersects_box(const LPoint3 &box_min, const LPoint3 &box_max,
                    IntersectionParams &params) {
  LPoint3 from = params.from_origin;
  LPoint3 delta = params.from_direction;

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
  params.t1 = tmin;
  params.t2 = tmax;
  return true;
}

bool CollisionHeightfield::
line_intersects_triangle(double &t, const LPoint3 &from,
                         const LPoint3 &delta,
                         const Triangle &triangle) {
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
  return true;
}

bool CollisionHeightfield::
sphere_intersects_box(const LPoint3 &box_min, const LPoint3 &box_max,
                      IntersectionParams &params) {
  LPoint3 center = params.center;
  double radius = params.radius;
  LPoint3 p = center.fmin(box_max).fmax(box_min);
  return (center - p).length_squared() <= radius * radius;
}

bool CollisionHeightfield::
sphere_intersects_triangle(LPoint3 &intersection_point,
                           const LPoint3 &center, double radius,
                           const Triangle &triangle) {
  intersection_point = closest_point_on_triangle(center, triangle);
  return (intersection_point - center).length_squared() <= radius * radius;
}

bool CollisionHeightfield::
box_intersects_triangle(const LPoint3 &box_min, const LPoint3 &box_max,
                        const Triangle &triangle) {
  // Using the Seperating Axis Theorem, we have a maximum of 13 SAT tests
  LPoint3 v0 = triangle.p1, v1 = triangle.p2, v2 = triangle.p3;

  PN_stdfloat p0, p1, p2, r;
  LVector3 c = (box_min + box_max) * 0.5f;
  PN_stdfloat e0 = (box_max[0] - box_min[0]) * 0.5f;
  PN_stdfloat e1 = (box_max[1] - box_min[1]) * 0.5f;
  PN_stdfloat e2 = (box_max[2] - box_min[2]) * 0.5f;

  v0 = v0 - c;
  v1 = v1 - c;
  v2 = v2 - c;

  LVector3 f0 = v1 - v0, f1 = v2 - v1, f2 = v0 - v2;

  r = e1 * cabs(f0[2]) + e2 * cabs(f0[1]);
  p0 =  v0[1]*(-v1[2] + v0[2]) + v0[2]*(v1[1] - v0[1]);
  p2 =  v2[1]*(-v1[2] + v0[2]) + v2[2]*(v1[1] - v0[1]);
  if (std::max(-1 * std::max(p0, p2), std::min(p0, p2)) > r) return false;

  r = e1 * cabs(f1[2]) + e2 * cabs(f1[1]);
  p0 =  v0[1]*(-v2[2] + v1[2]) + v0[2]*(v2[1] - v1[1]);
  p2 =  v2[1]*(-v2[2] + v1[2]) + v2[2]*(v2[1] - v1[1]);
  if (std::max(-1 * std::max(p0, p2), std::min(p0, p2)) > r) return false;

  r = e1 * cabs(f2[2]) + e2 * cabs(f2[1]);
  p0 =  v0[1]*(-v0[2] + v2[2]) + v0[2]*(v0[1] - v2[1]);
  p1 =  v1[1]*(-v0[2] + v2[2]) + v1[2]*(v0[1] - v2[1]);
  if (std::max(-1 * std::max(p0, p1), std::min(p0, p1)) > r) return false;

  r = e0 * cabs(f0[2]) + e2 * cabs(f0[0]);
  p0 =  v0[0]*(v1[2] - v0[2]) + v0[2]*(-v1[0] + v0[0]);
  p2 =  v2[0]*(v1[2] - v0[2]) + v2[2]*(-v1[0] + v0[0]);
  if (std::max(-1 * std::max(p0, p2), std::min(p0, p2)) > r) return false;

  r = e0 * cabs(f1[2]) + e2 * cabs(f1[0]);
  p0 =  v0[0]*(v2[2] - v1[2]) + v0[2]*(-v2[0] + v1[0]);
  p2 =  v2[0]*(v2[2] - v1[2]) + v2[2]*(-v2[0] + v1[0]);
  if (std::max(-1 * std::max(p0, p2), std::min(p0, p2)) > r) return false;

  r = e0 * cabs(f2[2]) + e2 * cabs(f2[0]);
  p0 =  v0[0]*(v0[2] - v2[2]) + v0[2]*(-v0[0] + v2[0]);
  p1 =  v1[0]*(v0[2] - v2[2]) + v1[2]*(-v0[0] + v2[0]);
  if (std::max(-1 * std::max(p0, p1), std::min(p0, p1)) > r) return false;

  r = e0 * cabs(f0[1]) + e1 * cabs(f0[0]);
  p0 =  v0[0]*(-v1[1] + v0[1]) + v0[1]*(v1[0] - v0[0]);
  p2 =  v2[0]*(-v1[1] + v0[1]) + v2[1]*(v1[0] - v0[0]);
  if (std::max(-1 * std::max(p0, p2), std::min(p0, p2)) > r) return false;

  r = e0 * cabs(f1[1]) + e1 * cabs(f1[0]);
  p0 =  v0[0]*(-v2[1] + v1[1]) + v0[1]*(v2[0] - v1[0]);
  p2 =  v2[0]*(-v2[1] + v1[1]) + v2[1]*(v2[0] - v1[0]);
  if (std::max(-1 * std::max(p0, p2), std::min(p0, p2)) > r) return false;

  r = e0 * cabs(f2[1]) + e1 * cabs(f2[0]);
  p0 =  v0[0]*(-v0[1] + v2[1]) + v0[1]*(v0[0] - v2[0]);
  p1 =  v1[0]*(-v0[1] + v2[1]) + v1[1]*(v0[0] - v2[0]);
  if (std::max(-1 * std::max(p0, p1), std::min(p0, p1)) > r) return false;

  if (std::max(std::max(v0[0], v1[0]), v2[0]) < -e0 ||
      std::min(std::min(v0[0], v1[0]), v2[0]) > e0) return false;
  if (std::max(std::max(v0[1], v1[1]), v2[1]) < -e1 ||
      std::min(std::min(v0[1], v1[1]), v2[1]) > e1) return false;
  if (std::max(std::max(v0[2], v1[2]), v2[2]) < -e2 ||
      std::min(std::min(v0[2], v1[2]), v2[2]) > e2) return false;

  LVector3 n = f0.cross(f1);
  PN_stdfloat d = dot(n, triangle.p1);

  LPoint3 e = box_max - c;

  PN_stdfloat r2 = e[0] * cabs(n[0]) + e[1] * cabs(n[1]) + e[2] * cabs(n[2]);
  PN_stdfloat s = dot(n, c) - d;
  return cabs(s) <= r2;
}

bool CollisionHeightfield::
box_intersects_box(const LPoint3 &box_min, const LPoint3 &box_max,
                   IntersectionParams &params) {

  return (box_min[0] <= params.box_max[0] && box_max[0] >= params.box_min[0]) &&
         (box_min[1] <= params.box_max[1] && box_max[1] >= params.box_min[1]) &&
         (box_min[2] <= params.box_max[2] && box_max[2] >= params.box_min[2]);
}

// Returns the closest point on a triangle to another given point.
LPoint3 CollisionHeightfield::
closest_point_on_triangle(const LPoint3 &p, const Triangle &triangle) {
  LVector3 ab = triangle.p2 - triangle.p1;
  LVector3 ac = triangle.p3 - triangle.p1;
  LVector3 ap = p - triangle.p1;
  PN_stdfloat d1 = dot(ab, ap);
  PN_stdfloat d2 = dot(ac, ap);
  if (d1 <= 0.0f && d2 <= 0.0f) return triangle.p1;

  LVector3 bp = p - triangle.p2;
  PN_stdfloat d3 = dot(ab, bp);
  PN_stdfloat d4 = dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3) return triangle.p2;

  PN_stdfloat vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    PN_stdfloat v = d1 / (d1 - d3);
    return triangle.p1 + v * ab;
  }

  LVector3 cp = p - triangle.p3;
  PN_stdfloat d5 = dot(ab, cp);
  PN_stdfloat d6 = dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6) return triangle.p3;

  PN_stdfloat vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    PN_stdfloat w = d2 / (d2 - d6);
    return triangle.p1 + w * ac;
  }

  PN_stdfloat va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    PN_stdfloat w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return triangle.p2 + w * (triangle.p3 - triangle.p2);
  }

  PN_stdfloat denom = 1.0f / (va + vb + vc);
  PN_stdfloat v = vb * denom;
  PN_stdfloat w = vc * denom;
  return triangle.p1 + ab * v + ac * w;
}

/* Given a pointer to a function that tests for intersection between a solid
 * and a box, return the quad tree nodes that are intersected by that solid. */
vector<CollisionHeightfield::QuadTreeIntersection> CollisionHeightfield::
find_intersections(BoxIntersection intersects_box, IntersectionParams params) const {
  queue<QuadTreeNode> q;
  QuadTreeNode node = _nodes[0];
  q.push(node);

  LPoint3 box_min, box_max;
  vector<QuadTreeIntersection> intersections;

  while (!q.empty()) {
    node = q.front();
    q.pop();
    box_min = {node.area.min[0], node.area.min[1], node.height_min};
    box_max = {node.area.max[0], node.area.max[1], node.height_max};
    if (!intersects_box(box_min, box_max, params)) {
      continue;
    }

    if (node.index >= _leaf_first_index) {
      QuadTreeIntersection intersection = {node.index, params.t1, params.t2};
      intersections.push_back(intersection);
    } else {
      int child_first_index = 4 * node.index + 1;
      q.push(_nodes[child_first_index]);
      q.push(_nodes[child_first_index + 1]);
      q.push(_nodes[child_first_index + 2]);
      q.push(_nodes[child_first_index + 3]);
    }
  }

  return intersections;
}

// Given a coordinate on the heightfield, return the triangles that
// are defined at this point in 3D space.
vector<CollisionHeightfield::Triangle> CollisionHeightfield::
get_triangles(int x, int y) const {
  int rows = _heightfield.get_read_x_size();
  int cols = _heightfield.get_read_y_size();
  vector<Triangle> triangles;
  if (x < 0 || y < 0 || x >= rows - 1 || y >= cols - 1)
    return triangles;

  Triangle t;
  int y2 = cols - 1 - y;
  if (x - 1 >= 0 && y2 - 1 >= 0) {
    t.p1 = LPoint3(x, y, get_height(x, y2));
    t.p2 = LPoint3(x, y - 1, get_height(x, y2 - 1));
    t.p3 = LPoint3(x -1, y, get_height(x-1, y2));
    triangles.push_back(t);
  }

  if (x + 1 < rows && y2 + 1 < cols) {
    t.p1 = LPoint3(x, y, get_height(x, y2));
    t.p2 = LPoint3(x, y+1, get_height(x, y2 + 1));
    t.p3 = LPoint3(x+1, y, get_height(x + 1, y2));
    triangles.push_back(t);
  }

  if (x - 1 >= 0 && y2 + 1 < cols) {
    t.p1 = LPoint3(x, y, get_height(x, y2));
    t.p2 = LPoint3(x-1, y+1, get_height(x-1, y2 + 1));
    t.p3 = LPoint3(x-1, y, get_height(x-1, y2));
    triangles.push_back(t);
    t.p3 = LPoint3(x, y+1, get_height(x, y2+1));
    triangles.push_back(t);
  }

  if (x + 1 < rows && y2 - 1 >= 0) {
    t.p1 = LPoint3(x, y, get_height(x, y2));
    t.p2 = LPoint3(x+1, y-1, get_height(x+1, y2-1));
    t.p3 = LPoint3(x, y-1, get_height(x, y2-1));
    triangles.push_back(t);
    t.p3 = LPoint3(x+1, y, get_height(x+1, y2));
    triangles.push_back(t);
  }

  return triangles;
}

void CollisionHeightfield::
setup_quadtree(int subdivisions) {
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

  int leaf_first_index = 0;
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
          PN_stdfloat value = _heightfield.get_gray(x,
            _heightfield.get_read_y_size() - 1 - y) * _max_height;

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

PStatCollector CollisionHeightfield::_volume_pcollector(
      "Collision Volumes:CollisionHeightfield");
PStatCollector CollisionHeightfield::_test_pcollector(
      "Collision Tests:CollisionHeightfield");
TypeHandle CollisionHeightfield::_type_handle;
