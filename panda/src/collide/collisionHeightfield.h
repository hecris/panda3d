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

/*
 * CollisionHeightfield efficiently deals with collisions on uneven
 * terrain given a heightfield image. A quad tree is implemented to
 * significantly reduce the amount of triangles tested. Each quad
 * tree node represents a sub-rectangle of the heightfield image
 * and thus a box in 3D space.
 * */
class EXPCL_PANDA_COLLIDE CollisionHeightfield : public CollisionSolid {
PUBLISHED:
  virtual LPoint3 get_collision_origin() const;

private:
  PNMImage _heightfield;
  PN_stdfloat _max_height;

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
