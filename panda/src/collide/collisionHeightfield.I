INLINE CollisionHeightfield::
CollisionHeightfield() {
}

INLINE CollisionHeightfield::
CollisionHeightfield(const CollisionHeightfield &copy) {
}

INLINE void CollisionHeightfield::
flush_level() {
  _volume_pcollector.flush_level();
  _test_pcollector.flush_level();
}
