#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>
#include <algorithm> // std::swap
#include "pandabase.h"
#include "lvecBase2.h"

class QuadTreeNode {
  /**
   * An abstract class that allows the user to define
   * their own quad tree node, as long as they provide a
   * method to "split" a node into four children.
   */
  public:
    virtual void make_children(QuadTreeNode* &child1,
                               QuadTreeNode* &child2,
                               QuadTreeNode* &child3,
                               QuadTreeNode* &child4) = 0;
};


class SearchStrategy {
  public:
    virtual bool evaluate(QuadTreeNode* node, bool is_leaf) = 0;
};

class QuadTreeIterator;

class QuadTree {
  /**
   * A full quad tree, also known as a T pyramid, is
   * a tree where each node has 0 or 4 children. In addition,
   * all leaf nodes are on the same level.
   */
  public:
    QuadTree(QuadTreeNode* root);
    ~QuadTree();
    bool subdivide();
    std::size_t get_size() const;
    void search(SearchStrategy &strategy);

    QuadTreeIterator begin();
    QuadTreeIterator end();

  private:
    std::vector<QuadTreeNode*> _items;
    std::size_t _leaf_first_index;

    std::size_t get_first_child_index(std::size_t parent_index) const;
    std::size_t get_parent_index(std::size_t child_index) const;
    bool is_leaf(std::size_t index) const;

    void depth_first_search(SearchStrategy &strategy,
                            std::size_t index);

    friend class QuadTreeIterator;
};

class QuadTreeIterator {
  /**
   * A bidirectional iterator for the QuadTree class.
   *
   * The forward direction is to traverse through the tree
   * level by level, starting from the first level (root node)
   * and ending at the last level. At each level, the nodes are
   * visited from left to right. This is also known as "breadth
   * first traversal" or "level order traversal".
   *
   * Similarly, the backward direction is to traverse from last
   * level to first level, where at each level, the nodes are
   * visited from right to left.
   */
  public:
    INLINE QuadTreeIterator(QuadTree* tree, std::size_t index=0);

    INLINE QuadTreeIterator& operator ++ ();
    INLINE QuadTreeIterator& operator -- ();
    INLINE QuadTreeIterator operator ++ (int);
    INLINE QuadTreeIterator operator -- (int);
    INLINE bool operator == (const QuadTreeIterator& iter) const;
    INLINE bool operator != (const QuadTreeIterator& iter) const;

    INLINE QuadTreeNode* get_current_node() const;
    INLINE bool get_child_node(std::size_t i, QuadTreeNode* &child) const;
    INLINE bool is_at_leaf() const;

  private:
    std::size_t _index;
    QuadTree* _tree;
};

class Rectangle : public QuadTreeNode {
  /**
   * A derived QuadTreeNode that represents a Rectangle,
   * which is a common use case.
   */
  public:
    Rectangle(LVecBase2 min, LVecBase2 max):
    _min(min),
    _max(max)
    {}

    /**
     * Returns 4 new Rectangle objects resulting from
     * dividing this Rectangle into four sub-rectangles.
     */
    virtual void make_children(QuadTreeNode* &child1,
                               QuadTreeNode* &child2,
                               QuadTreeNode* &child3,
                               QuadTreeNode* &child4) override {
      LVecBase2 center = (_min + _max) / 2;
      child1 = new Rectangle(center, _min);
      child2 = new Rectangle(center, _max);
      std::swap(_min[0], _max[0]);
      child3 = new Rectangle(center, _min);
      child4 = new Rectangle(center, _max);
    }

  private:
    LVecBase2 _min;
    LVecBase2 _max;

};

#include "quadTree.I"

#endif
