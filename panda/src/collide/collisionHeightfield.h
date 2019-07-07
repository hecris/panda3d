#ifndef COLLISIONHEIGHTFIELD_H
#define COLLISIONHEIGHTFIELD_H

#include "pandabase.h"
#include "collisionSolid.h"

class EXPCL_PANDA_COLLIDE CollisionHeightfield : public CollisionSolid {
PUBLISHED:
  // todo: this should be inline
  INLINE CollisionHeightfield();
  virtual LPoint3 get_collision_origin() const;

public:
  // todo: this should be inline
  INLINE CollisionHeightfield(const CollisionHeightfield &copy);
  virtual CollisionSolid *make_copy();

  virtual PStatCollector &get_volume_pcollector();
  virtual PStatCollector &get_test_pcollector();

  // todo: this should be inline
  INLINE static void flush_level();

protected:
  virtual PT(BoundingVolume) compute_internal_bounds() const;

protected:
  virtual PT(CollisionEntry)
  test_intersection_from_line(const CollisionEntry &entry) const;

private:
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
