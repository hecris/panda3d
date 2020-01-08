#include "quadTree.h"
#include <iostream>

using std::cout;

/**
 *
 */
QuadTree::
QuadTree(QuadTreeNode* root):
_leaf_first_index(0)
{
	_items.push_back(root);
}


/**
 *
 */
bool QuadTree::
subdivide() {
	QuadTreeNode* child1 = nullptr;
	QuadTreeNode* child2 = nullptr;
	QuadTreeNode* child3 = nullptr;
	QuadTreeNode* child4 = nullptr;

	size_t size = get_size();
	for (size_t i = _leaf_first_index; i < size; i++) {
		_items[i]->make_children(child1, child2, child3, child4);
		_items.push_back(child1);
		_items.push_back(child2);
		_items.push_back(child3);
		_items.push_back(child4);
	}
	_leaf_first_index = (_leaf_first_index * 4) + 1;
	return true;
}

/**
 *
 */
bool QuadTree::
subdivide(size_t n) {
	for (int i = 0; i < n; i++) {
		if (!subdivide()) {
			return false;
		}
	}
	return true;
}


/**
 *
 */
size_t QuadTree::
get_size() const {
	return _items.size();
}


/**
 *
 */
void QuadTree::
search(SearchStrategy &strategy) {
	depth_first_search(strategy, 0);
}


/**
 *
 */
size_t QuadTree::
get_first_child_index(size_t parent_index) const {
	return parent_index * 4 + 1;
}


/**
 *
 */
size_t QuadTree::
get_parent_index(size_t child_index) const {
	return (child_index - 1) / 4;
}

/**
 *
 */
bool QuadTree::
is_leaf(size_t index) const {
	return index >= _leaf_first_index;
}

/**
 *
 */
void QuadTree::
depth_first_search(SearchStrategy &strategy, size_t index) {
	// base case: reached end of tree
	if (index >= get_size()) return;

	if (strategy.evaluate(_items[index], is_leaf(index))) {
		size_t c = get_first_child_index(index);
		depth_first_search(strategy, c);
		depth_first_search(strategy, c + 1);
		depth_first_search(strategy, c + 2);
		depth_first_search(strategy, c + 3);
	}
}


/**
 *
 */
QuadTreeIterator QuadTree::
begin() {
	return QuadTreeIterator(this);
}


/**
 *
 */
QuadTreeIterator QuadTree::
end() {
	return QuadTreeIterator(this, get_size());
}


/**
 *
 */
QuadTree::
~QuadTree()
{
	for (size_t i = 0; i < get_size(); i++) {
		delete _items[i];
		_items[i] = nullptr;
	}
}

