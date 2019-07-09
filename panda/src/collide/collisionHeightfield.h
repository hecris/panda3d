#ifndef COLLISIONHEIGHTFIELD_H
#define COLLISIONHEIGHTFIELD_H

#include "pandabase.h"
#include "collisionSolid.h"
#include "pnmImage.h"

class EXPCL_PANDA_COLLIDE CollisionHeightfield : public CollisionSolid {
private:
  struct Rect {
    LVector2 min;
    LVector2 max;
  };

  struct QuadTreeNode {
    Rect area;
    float height_min;
    float height_max;
  };

  void setup_quadtree(int subdivisions);


PUBLISHED:
  INLINE CollisionHeightfield();
  ~CollisionHeightfield() {
    delete[] _nodes;
  };
  CollisionHeightfield(PNMImage &heightfield);
  virtual LPoint3 get_collision_origin() const;

private:
  PNMImage _heightfield;
  // Todo: PT(QuadTreeNode) _nodes;
  QuadTreeNode *_nodes;
  int _nodes_count;

public:
  INLINE PNMImage &heightfield();

  INLINE CollisionHeightfield(const CollisionHeightfield &copy);
  virtual CollisionSolid *make_copy();

  virtual PStatCollector &get_volume_pcollector();
  virtual PStatCollector &get_test_pcollector();

  INLINE static void flush_level();

protected:
  virtual PT(BoundingVolume) compute_internal_bounds() const;

protected:
  bool box_intersects_line(double &t1, double &t2,
                           const LPoint3 &box_min, const LPoint3 &box_max,
                           const LPoint3 &from, const LVector3 &delta) const;

  virtual PT(CollisionEntry)
  test_intersection_from_ray(const CollisionEntry &entry) const;

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
