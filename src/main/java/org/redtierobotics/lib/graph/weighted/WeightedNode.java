package org.redtierobotics.lib.graph.weighted;

import org.redtierobotics.lib.graph.Node;

public class WeightedNode<T> extends Node<T> {
	double cost = 0.0;

	public WeightedNode(T value) {
		super(value);
	}

	void setCost(double cost) {
		this.cost = cost;
	}

	double getCost() {
		return cost;
	}
}
