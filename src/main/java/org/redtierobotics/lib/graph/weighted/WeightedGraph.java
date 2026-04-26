package org.redtierobotics.lib.graph.weighted;

import edu.wpi.first.math.Pair;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
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
	public Queue<Pair<NodeType, EdgeType>> findPath(NodeType start, NodeType target) {
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
	protected Queue<Pair<NodeType, EdgeType>> ucs(NodeType start, NodeType target) {
		Map<NodeType, Double> costSoFar = new HashMap<>();
		PriorityQueue<NodeType> current =
				new PriorityQueue<>((a, b) -> Double.compare(costSoFar.get(a), costSoFar.get(b)));

		Map<NodeType, NodeType> parent = new HashMap<>();

		current.add(start);
		costSoFar.put(start, 0.0);
		parent.put(start, null);

		while (!current.isEmpty()) {
			NodeType node = current.poll();
			if (node.equals(target)) {
				Deque<Pair<NodeType, EdgeType>> path = new ArrayDeque<>();
				NodeType now = target;

				while (parent.get(now) != null) {
					NodeType prev = parent.get(now);
					path.addFirst(new Pair<NodeType, EdgeType>(now, succession.get(prev).get(now)));
					now = prev;
				}

				return path;
			}

			Map<NodeType, EdgeType> successors = succession.get(node);
			if (successors == null) {
				continue;
			}

			for (Entry<NodeType, EdgeType> entry : successors.entrySet()) {
				NodeType next = entry.getKey();
				double newCost = costSoFar.get(node) + entry.getValue().weight;

				if (!costSoFar.containsKey(next) || newCost < costSoFar.get(next)) {
					costSoFar.put(next, newCost);
					parent.put(next, node);
					current.add(next);
				}
			}
		}

		return null;
	}
}
