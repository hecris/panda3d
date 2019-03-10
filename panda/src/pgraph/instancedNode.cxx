/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file instancedNode.cxx
 * @author rdb
 * @date 2019-03-10
 */

#include "instancedNode.h"
#include "omniBoundingVolume.h"

TypeHandle InstancedNode::_type_handle;

/**
 *
 */
InstancedNode::
InstancedNode(const std::string &name) :
  PandaNode(name)
{
  set_cull_callback();
}

/**
 *
 */
InstancedNode::
InstancedNode(const InstancedNode &copy) :
  PandaNode(copy),
  _cycler(copy._cycler)
{
  set_cull_callback();
}

/**
 *
 */
InstancedNode::
~InstancedNode() {
}

/**
 * Returns a newly-allocated PandaNode that is a shallow copy of this one.  It
 * will be a different pointer, but its internal data may or may not be shared
 * with that of the original PandaNode.  No children will be copied.
 */
PandaNode *InstancedNode::
make_copy() const {
  return new InstancedNode(*this);
}

/**
 * Returns true if it is generally safe to flatten out this particular kind of
 * PandaNode by duplicating instances (by calling dupe_for_flatten()), false
 * otherwise (for instance, a Camera cannot be safely flattened, because the
 * Camera pointer itself is meaningful).
 */
bool InstancedNode::
safe_to_flatten() const {
  return false;
}

/**
 * Returns true if it is generally safe to combine this particular kind of
 * PandaNode with other kinds of PandaNodes of compatible type, adding
 * children or whatever.  For instance, an LODNode should not be combined with
 * any other PandaNode, because its set of children is meaningful.
 */
bool InstancedNode::
safe_to_combine() const {
  // This can happen iff the instance list is identical; see combine_with().
  return true;
}

/**
 * Transforms the contents of this node by the indicated matrix, if it means
 * anything to do so.  For most kinds of nodes, this does nothing.
 */
void InstancedNode::
xform(const LMatrix4 &mat) {
}

/**
 * Collapses this node with the other node, if possible, and returns a pointer
 * to the combined node, or NULL if the two nodes cannot safely be combined.
 *
 * The return value may be this, other, or a new node altogether.
 *
 * This function is called from GraphReducer::flatten(), and need not deal
 * with children; its job is just to decide whether to collapse the two nodes
 * and what the collapsed node should look like.
 */
PandaNode *InstancedNode::
combine_with(PandaNode *other) {
  if (is_exact_type(get_class_type()) && other->is_exact_type(get_class_type())) {
    InstancedNode *iother = DCAST(InstancedNode, other);

    // Only combine them if the instance lists for both are identical.
    Thread *current_thread = Thread::get_current_thread();
    CDReader this_cdata(_cycler, current_thread);
    CDReader other_cdata(iother->_cycler, current_thread);
    CPT(InstanceList) this_instances = this_cdata->_instances.get_read_pointer(current_thread);
    CPT(InstanceList) other_instances = other_cdata->_instances.get_read_pointer(current_thread);
    if (this_instances == other_instances) {
      return this;
    }
  }

  return nullptr;
}

/**
 * This is used to support NodePath::calc_tight_bounds().  It is not intended
 * to be called directly, and it has nothing to do with the normal Panda
 * bounding-volume computation.
 *
 * If the node contains any geometry, this updates min_point and max_point to
 * enclose its bounding box.  found_any is to be set true if the node has any
 * geometry at all, or left alone if it has none.  This method may be called
 * over several nodes, so it may enter with min_point, max_point, and
 * found_any already set.
 */
CPT(TransformState) InstancedNode::
calc_tight_bounds(LPoint3 &min_point, LPoint3 &max_point, bool &found_any,
                  const TransformState *transform, Thread *current_thread) const {

  CPT(InstanceList) instances = get_instances(current_thread);
  CPT(TransformState) next_transform = transform->compose(get_transform(current_thread));

  for (size_t ii = 0; ii < instances->size(); ++ii) {
    CPT(TransformState) instance_transform = next_transform->compose((*instances)[ii].get_transform());

    Children cr = get_children(current_thread);
    size_t num_children = cr.get_num_children();
    for (size_t ci = 0; ci < num_children; ++ci) {
      cr.get_child(ci)->calc_tight_bounds(min_point, max_point,
                                          found_any, instance_transform,
                                          current_thread);
    }
  }

  return next_transform;
}

/**
 * This function will be called during the cull traversal to perform any
 * additional operations that should be performed at cull time.  This may
 * include additional manipulation of render state or additional
 * visible/invisible decisions, or any other arbitrary operation.
 *
 * Note that this function will *not* be called unless set_cull_callback() is
 * called in the constructor of the derived class.  It is necessary to call
 * set_cull_callback() to indicated that we require cull_callback() to be
 * called.
 *
 * By the time this function is called, the node has already passed the
 * bounding-volume test for the viewing frustum, and the node's transform and
 * state have already been applied to the indicated CullTraverserData object.
 *
 * The return value is true if this node should be visible, or false if it
 * should be culled.
 */
bool InstancedNode::
cull_callback(CullTraverser *trav, CullTraverserData &data) {
  Thread *current_thread = trav->get_current_thread();

  // Disable culling from this point on, for now.
  data._view_frustum = nullptr;
  data._cull_planes = CullPlanes::make_empty();

  CPT(InstanceList) instances = get_instances(current_thread);

  for (const InstanceList::Instance &instance : *instances) {
    CullTraverserData instance_data(data);
    instance_data.apply_transform(instance.get_transform());
    trav->traverse_below(instance_data);
  }

  return false;
}

/**
 *
 */
void InstancedNode::
output(std::ostream &out) const {
  PandaNode::output(out);
  out << " (" << get_num_instances() << " instances)";
}

/**
 * Returns a newly-allocated BoundingVolume that represents the internal
 * contents of the node.  Should be overridden by PandaNode classes that
 * contain something internally.
 */
void InstancedNode::
compute_internal_bounds(CPT(BoundingVolume) &internal_bounds,
                        int &internal_vertices,
                        int pipeline_stage,
                        Thread *current_thread) const {
  //TODO: generate a better bounding volume.
  internal_bounds = new OmniBoundingVolume;
}

/**
 * Tells the BamReader how to create objects of type GeomNode.
 */
void InstancedNode::
register_with_read_factory() {
  BamReader::get_factory()->register_factory(get_class_type(), make_from_bam);
}

/**
 * Writes the contents of this object to the datagram for shipping out to a
 * Bam file.
 */
void InstancedNode::
write_datagram(BamWriter *manager, Datagram &dg) {
  PandaNode::write_datagram(manager, dg);
  manager->write_cdata(dg, _cycler);
}

/**
 * This function is called by the BamReader's factory when a new object of
 * type InstancedNode is encountered in the Bam file.  It should create the
 * InstancedNode and extract its information from the file.
 */
TypedWritable *InstancedNode::
make_from_bam(const FactoryParams &params) {
  InstancedNode *node = new InstancedNode("");
  DatagramIterator scan;
  BamReader *manager;

  parse_params(params, scan, manager);
  node->fillin(scan, manager);

  return node;
}

/**
 * This internal function is called by make_from_bam to read in all of the
 * relevant data from the BamFile for the new InstancedNode.
 */
void InstancedNode::
fillin(DatagramIterator &scan, BamReader *manager) {
  PandaNode::fillin(scan, manager);
  manager->read_cdata(scan, _cycler);
}

/**
 *
 */
InstancedNode::CData::
CData(const InstancedNode::CData &copy) :
  _instances(copy._instances)
{
}

/**
 *
 */
CycleData *InstancedNode::CData::
make_copy() const {
  return new CData(*this);
}

/**
 * Writes the contents of this object to the datagram for shipping out to a
 * Bam file.
 */
void InstancedNode::CData::
write_datagram(BamWriter *manager, Datagram &dg) const {
  CPT(InstanceList) instances = _instances.get_read_pointer();
  manager->write_pointer(dg, instances.p());
}

/**
 * Receives an array of pointers, one for each time manager->read_pointer()
 * was called in fillin(). Returns the number of pointers processed.
 */
int InstancedNode::CData::
complete_pointers(TypedWritable **p_list, BamReader *manager) {
  int pi = CycleData::complete_pointers(p_list, manager);

  _instances = DCAST(InstanceList, p_list[pi++]);
  return pi;
}

/**
 * This internal function is called by make_from_bam to read in all of the
 * relevant data from the BamFile for the new GeomNode.
 */
void InstancedNode::CData::
fillin(DatagramIterator &scan, BamReader *manager) {
  manager->read_pointer(scan);
}
