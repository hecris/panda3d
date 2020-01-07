/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file collisionHeightfield.I
 * @author hecris
 * @date 2019-07-01
 */

/**
 *
 */
INLINE CollisionHeightfield::
CollisionHeightfield() {
}

/**
 *
 */
INLINE CollisionHeightfield::
CollisionHeightfield(const CollisionHeightfield &copy) {
}

/**
 *
 */
INLINE void CollisionHeightfield::
flush_level() {
  _volume_pcollector.flush_level();
  _test_pcollector.flush_level();
}
