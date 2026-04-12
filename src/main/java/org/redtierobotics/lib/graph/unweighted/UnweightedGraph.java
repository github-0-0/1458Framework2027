package org.redtierobotics.lib.graph.unweighted;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Set;
import org.redtierobotics.lib.graph.Graph;

import edu.wpi.first.math.Pair;

public class UnweightedGraph<NodeType extends UnweightedNode<?>, EdgeType extends UnweightedEdge<?>>
		extends Graph<NodeType, EdgeType> {
	public UnweightedGraph(NodeType start) {
		super(start);
	}

	/**
	 * @see org.redtierobotics.lib.graph.unweighted.UnweightedGraph#bfs(NodeType, NodeType)
	 */
	@Override
	public Queue<Pair<NodeType, EdgeType>> findPath(NodeType start, NodeType target) {
		if (!containsNode(start) || !containsNode(target)) {
			return null;
		}

		if (start.equals(target)) {
			return new ArrayDeque<>();
		}

		return bfs(start, target);
	}

	/**
	 * Runs breadth-first search on a graph. Computes the fastest path in terms of number of nodes.
	 *
	 * @param start The start node
	 * @param target The target node
	 * @return A {@code Queue} containing the walk to the target in order, including target but not
	 *     including start.
	 */
	protected Queue<Pair<NodeType, EdgeType>> bfs(NodeType start, NodeType target) {
		Queue<NodeType> current = new ArrayDeque<>();
		Set<NodeType> visited = new HashSet<>();
		Map<NodeType, NodeType> parent = new HashMap<>();

		current.add(start);
		visited.add(start);
		parent.put(start, null);

		while (!current.isEmpty()) {
			NodeType node = current.poll();

			Map<NodeType, EdgeType> successors = getSuccessors(node);
			if (successors == null) {
				continue;
			}
			
			for (Entry<NodeType, EdgeType> successor : successors.entrySet()) {
				NodeType successorNode = successor.getKey();
				if (!visited.contains(successorNode)) {
					parent.put(successorNode, node);

					if (successorNode.equals(target)) {
						Deque<Pair<NodeType, EdgeType>> path = new ArrayDeque<>();
						NodeType now = target;

						while (parent.get(now) != null) {
							NodeType prev = parent.get(now);
							path.addFirst(new Pair<NodeType, EdgeType>(now, getSuccessors(node).get(now)));
							now = prev;
						}

						return path;
					}

					current.add(successorNode);
					visited.add(successorNode);
				}
			}
		}

		return null;
	}
}
