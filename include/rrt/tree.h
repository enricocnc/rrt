#pragma once

#include <rrt/types.h>

#include <cstddef>
#include <memory>
#include <vector>

using node_id = size_t;

namespace tree {

struct Node {
	types::Pose2D configuration;
	node_id id;
	node_id parent;
	double cost;
	types::Path path_from_parent;
};

class Tree {
public:
	Tree();

	// return the current tree
	[[nodiscard]] const std::vector<Node>& getTree() const;

	// return the current tree size
	[[nodiscard]] size_t getTreeSize() const;

	// pre-allocate memory
	void reserve(size_t size);
	
	// return the node with given id
	[[nodiscard]] const Node& getNodeById(const node_id &id) const;

	// initialize the tree with the given root. Return the id assigned to this node
	node_id initializeTree(const types::Pose2D &start);

	// find the node closest to the given pose
	[[nodiscard]] node_id findNearestNode(const types::Pose2D &pose) const;

	// find the nodes inside the circle with the given radius and centered on the given pose
	[[nodiscard]] std::vector<node_id> findNodesWithinRadius(const types::Pose2D &pose, double radius) const;

	// insert a new node inside the tree, Return the id assigned to this node
	node_id insertNode(const types::Pose2D &pose, const node_id &parent, double cost, const types::Path &path_from_parent);

	// update the parent of the given node
	void updateParent(const node_id &child_id, const node_id &new_parent_id, double cost, const types::Path &new_path_from_parent);

private:
  std::vector<Node> nodes_;

	bool isValidNodeId_(const node_id &id) const;
};

} // namespace tree