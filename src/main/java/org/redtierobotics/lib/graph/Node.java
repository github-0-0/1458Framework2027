package org.redtierobotics.lib.graph;

public abstract class Node<T> {
	private T value;

	public <U> Node(T object) {
		this.value = object;
	}

	public T get() {
		return value;
	}

	public boolean equals(Node<T> other) {
		return get().equals(other.get());
	}
}
