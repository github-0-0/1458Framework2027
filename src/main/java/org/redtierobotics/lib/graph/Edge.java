package org.redtierobotics.lib.graph;

public abstract class Edge<U> implements Cloneable {
	protected U value;

	public Edge(U value) {
		this.value = value;
	}

	public U get() {
		return value;
	}

	@Override
	public abstract Edge<U> clone();
}
