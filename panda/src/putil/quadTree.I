/**
 *
 */
INLINE QuadTreeIterator::
QuadTreeIterator(QuadTree* tree, std::size_t index):
  _tree(tree), _index(index)
{}


/**
 *
 */
INLINE QuadTreeIterator& QuadTreeIterator::
operator ++ () {
  _index++;
  return *this;
}


/**
 *
 */
INLINE QuadTreeIterator& QuadTreeIterator::
operator -- () {
  _index--;
  return *this;
}


/**
 *
 */
INLINE QuadTreeIterator QuadTreeIterator::
operator ++ (int) {
  QuadTreeIterator ret = *this;
  ++* (this);
  return ret;
}


/**
 *
 */
INLINE QuadTreeIterator QuadTreeIterator::
operator -- (int) {
  QuadTreeIterator ret = *this;
  --* (this);
  return ret;
}


/**
 *
 */
INLINE bool QuadTreeIterator::
operator == (const QuadTreeIterator& iter) const {
  return _index == iter._index;
}


/**
 *
 */
INLINE bool QuadTreeIterator::
operator != (const QuadTreeIterator& iter) const {
  return _index != iter._index;
}


/**
 *
 */
INLINE QuadTreeNode* QuadTreeIterator::
get_current_node() const {
  return _tree->_items[_index];
}


/**
 *
 */
INLINE bool QuadTreeIterator::
get_child_node(std::size_t i, QuadTreeNode* &child) const {
  nassertr(0 <= i && i <= 3, false);
  // for now, assume 0 <= i <= 3
  if (_tree->is_leaf(_index)) {
    return false;
  }
  std::size_t c = _tree->get_first_child_index(_index);
  child = _tree->_items[c + i];
  return true;
}

/**
 *
 */
INLINE bool QuadTreeIterator::
is_at_leaf() const {
  return _tree->is_leaf(_index);
}
