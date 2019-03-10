/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file instancedNode.I
 * @author rdb
 * @date 2019-03-10
 */

/**
 * Returns the number of instances.
 */
INLINE size_t InstancedNode::
get_num_instances() const {
  Thread *current_thread = Thread::get_current_thread();
  CDReader cdata(_cycler, current_thread);
  nassertr_always(cdata->_instances != nullptr, 0);
  return cdata->_instances.get_read_pointer(current_thread)->size();
}

/**
 * Returns the list of instances.
 */
INLINE CPT(InstanceList) InstancedNode::
get_instances(Thread *current_thread) const {
  CDReader cdata(_cycler, current_thread);
  return cdata->_instances.get_read_pointer(current_thread);
}

/**
 * Returns the list of instances.
 *
 * Don't call this in a downstream thread unless you don't mind it blowing
 * away other changes you might have recently made in an upstream thread.
 */
INLINE PT(InstanceList) InstancedNode::
modify_instances() {
  CDWriter cdata(_cycler, true);
  return cdata->_instances.get_write_pointer();
}

/**
 * Entirely replaces the list of instances with the given list.
 *
 * Don't call this in a downstream thread unless you don't mind it blowing
 * away other changes you might have recently made in an upstream thread.
 */
INLINE void InstancedNode::
set_instances(PT(InstanceList) instances) {
  CDWriter cdata(_cycler, true);
  cdata->_instances = std::move(instances);
}

/**
 *
 */
INLINE InstancedNode::CData::
CData() : _instances(new InstanceList) {
}
