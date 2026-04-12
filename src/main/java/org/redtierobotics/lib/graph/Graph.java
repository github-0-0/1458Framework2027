package org.redtierobotics.lib.graph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

public abstract class Graph<NodeType extends Node<?>, EdgeType extends Edge<?>> {
	protected Map<NodeType, Map<NodeType, EdgeType>> succession = new HashMap<>();
	protected Set<NodeType> nodes = new HashSet<>();

	/** An unweighted graph */
	public Graph(NodeType start) {
		ensureNode(start);
	}

	public Graph<NodeType, EdgeType> connectSingleSided(
			NodeType first, NodeType second, EdgeType edge) {
		ensureNode(first);
		ensureNode(second);

		succession.get(first).put(second, edge);
		return this;
	}

	@SuppressWarnings("unchecked")
	public Graph<NodeType, EdgeType> connectDoubleSided(
			NodeType first, NodeType second, EdgeType edge) {
		return connectDoubleSided(first, second, edge, (EdgeType) edge.clone());
	}

	public Graph<NodeType, EdgeType> connectDoubleSided(
			NodeType first, NodeType second, EdgeType edge, EdgeType reversedEdge) {
		ensureNode(first);
		ensureNode(second);

		succession.get(first).put(second, edge);
		succession.get(second).put(first, reversedEdge);

		return this;
	}

	private void ensureNode(NodeType node) {
		succession.computeIfAbsent(node, k -> new HashMap<>());
		nodes.add(node);
	}

	/** Whether this graph is connected with this node */
	public boolean containsNode(NodeType node) {
		return nodes.contains(node);
	}

	public abstract Queue<EdgeType> findPath(NodeType start, NodeType target);
}
