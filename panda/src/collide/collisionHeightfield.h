/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file collisionHeightfield.h
 * @author hecris
 * @date 2019-07-01
 */

#ifndef COLLISIONHEIGHTFIELD_H
#define COLLISIONHEIGHTFIELD_H

#include "pandabase.h"
#include "collisionSolid.h"
#include "pnmImage.h"
#include "quadTree.h"

/*
 * CollisionHeightfield efficiently deals with collisions on uneven
 * terrain given a heightfield image. A quad tree is implemented to
 * significantly reduce the amount of triangles tested. Each quad
 * tree node represents a sub-rectangle of the heightfield image
 * and thus a box in 3D space.
 * */

class HeightfieldQuad : public QuadTreeNode {
  public:
    INLINE HeightfieldQuad(LVecBase2 min, LVecBase2 max);

    INLINE virtual void make_children(QuadTreeNode* &child1,
                                      QuadTreeNode* &child2,
                                      QuadTreeNode* &child3,
                                      QuadTreeNode* &child4) override;

    INLINE PN_stdfloat get_min_height() const;
    INLINE void set_min_height(PN_stdfloat height_min);

    INLINE PN_stdfloat get_max_height() const;
    INLINE void set_max_height(PN_stdfloat height_max);

    INLINE LVecBase2& get_min();
    INLINE LVecBase2& get_max();
    INLINE PN_stdfloat get_area() const;

  private:
    LVecBase2 _min;
    LVecBase2 _max;
    PN_stdfloat _height_min;
    PN_stdfloat _height_max;
};

class EXPCL_PANDA_COLLIDE CollisionHeightfield : public CollisionSolid {
PUBLISHED:
  CollisionHeightfield(PNMImage heightfield, PN_stdfloat max_height);

  virtual LPoint3 get_collision_origin() const;

private:
  typedef QuadTreeIterator iterator;
  PNMImage _heightfield;
  PN_stdfloat _max_height;
  QuadTree* _quadtree;

  void fill_quadtree_heights();

protected:
  virtual PT(CollisionEntry)
  test_intersection_from_ray(const CollisionEntry &entry) const;
  virtual PT(CollisionEntry)
  test_intersection_from_sphere(const CollisionEntry &entry) const;
  virtual PT(CollisionEntry)
  test_intersection_from_box(const CollisionEntry &entry) const;

  virtual void fill_viz_geom();

public:
  INLINE CollisionHeightfield(const CollisionHeightfield &copy);
  virtual CollisionSolid *make_copy();

  virtual PStatCollector &get_volume_pcollector();
  virtual PStatCollector &get_test_pcollector();

  INLINE static void flush_level();

  ~CollisionHeightfield();

protected:
  // TODO
  virtual PT(BoundingVolume) compute_internal_bounds() const;

private:
  INLINE CollisionHeightfield();
  static PStatCollector _volume_pcollector;
  static PStatCollector _test_pcollector;

protected:
  static TypedWritable *make_CollisionHeightfield(const FactoryParams &params);
  void fillin(DatagramIterator &scan, BamReader *manager);

public:
  static void register_with_read_factory();

public:
  static TypeHandle get_class_type() {
    return _type_handle;
  }
  static void init_type() {
    CollisionSolid::init_type();
    register_type(_type_handle, "CollisionHeightfield",
                  CollisionSolid::get_class_type());
  }
  virtual TypeHandle get_type() const {
    return get_class_type();
  }
  virtual TypeHandle force_init_type() {init_type(); return get_class_type();}

private:
  static TypeHandle _type_handle;
};

#include "collisionHeightfield.I"

#endif
