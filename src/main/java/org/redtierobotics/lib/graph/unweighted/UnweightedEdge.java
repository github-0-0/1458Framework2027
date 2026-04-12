package org.redtierobotics.lib.graph.unweighted;

import org.redtierobotics.lib.graph.Edge;

public class UnweightedEdge<U> extends Edge<U> {
	public UnweightedEdge(U value) {
		super(value);
	}

	@Override
	public UnweightedEdge<U> clone() {
		return new UnweightedEdge<U>(value);
	}
}
