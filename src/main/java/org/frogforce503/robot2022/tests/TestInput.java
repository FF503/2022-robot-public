package org.frogforce503.robot2022.tests;

/**
 * Test input only support strings, doubles, and booleans for now.
 */
public class TestInput<E extends Object> {
    public String name;
    public E value;

    public TestInput(String name, E value) {
        this.name = name;
        this.value = value;
    }

    public E getValue() {
        return value;
    }

    public void setValue(E value) {
        this.value = value;
    }

    public String getType() {
        if (this.value instanceof String) {
            return "String";
        } else if (this.value instanceof Double) {
            return "Double";
        }

        return "Boolean";
    }
}
