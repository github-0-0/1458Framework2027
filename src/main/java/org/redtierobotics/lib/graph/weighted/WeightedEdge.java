package org.redtierobotics.lib.graph.weighted;

import org.redtierobotics.lib.graph.Edge;

public class WeightedEdge<U> extends Edge<U> {
	public final double weight;

	public WeightedEdge(U value, double weight) {
		super(value);
		this.weight = weight;
	}

	@Override
	public WeightedEdge<U> clone() {
		return new WeightedEdge<>(value, weight);
	}
}
