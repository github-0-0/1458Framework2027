package org.redtierobotics.lib.graph.weighted;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import org.redtierobotics.lib.graph.Graph;

public class WeightedGraph<NodeType extends WeightedNode<?>, EdgeType extends WeightedEdge<?>>
		extends Graph<NodeType, EdgeType> {
	public WeightedGraph(NodeType start) {
		super(start);
	}

	/**
	 * @see org.redtierobotics.lib.graph.weighted.WeightedGraph#ucs(NodeType, NodeType)
	 */
	@Override
	public Queue<EdgeType> findPath(NodeType start, NodeType target) {
		if (!containsNode(start) || !containsNode(target)) {
			return null;
		}

		return ucs(start, target);
	}

	/**
	 * Runs breadth-first search on a graph. Computes the fastest path in terms of number of nodes.
	 *
	 * @param start The start node
	 * @param target The target node
	 * @return A {@code Queue} containing the walk to the target in order, including target but not
	 *     including start.
	 */
	protected Queue<EdgeType> ucs(NodeType start, NodeType target) {
		PriorityQueue<NodeType> current =
				new PriorityQueue<>((a, b) -> Double.compare(a.getCost(), b.getCost()));

		Map<NodeType, Double> costSoFar = new HashMap<>();
		Map<NodeType, NodeType> parent = new HashMap<>();
		Set<NodeType> visited = new HashSet<>();

		start.setCost(0);
		current.add(start);
		costSoFar.put(start, 0.0);
		parent.put(start, null);

		while (!current.isEmpty()) {
			NodeType node = current.poll();

			if (!visited.contains(node)) {
				visited.add(node);

				if (node.equals(target)) {
					Deque<EdgeType> path = new ArrayDeque<>();
					NodeType now = target;

					while (parent.get(now) != null) {
						NodeType prev = parent.get(now);
						path.addFirst(succession.get(prev).get(now));
						now = prev;
					}

					return path;
				}

				Map<NodeType, EdgeType> successors = succession.get(node);

				for (Entry<NodeType, EdgeType> entry : successors.entrySet()) {
					NodeType next = entry.getKey();
					double newCost = costSoFar.get(node) + entry.getValue().weight;

					if (!costSoFar.containsKey(next) || newCost < costSoFar.get(next)) {
						costSoFar.put(next, newCost);
						next.setCost(newCost);
						parent.put(next, node);
						current.add(next);
					}
				}
			}
		}

		return null;
	}
}
