package org.redtierobotics.lib.graph;

import java.util.Objects;

public abstract class Node<T> {
	private final T value;

	public <U> Node(T object) {
		this.value = object;
	}

	public T get() {
		return value;
	}

    @SuppressWarnings("unchecked")
	@Override
    public boolean equals(Object o) {
        if (this == o) {
			return true;
		}
        
        if (o == null || getClass() != o.getClass()) {
			return false;
		}
        
        Node<T> node = (Node<T>) o;
        
        return value.equals(node.get());
    }

    @Override
    public int hashCode() {
        return Objects.hash(value);
    }
}
