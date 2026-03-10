package com.openpositioning.PositionMe.sensors;

/**
 * Minimal mutable particle state used by the PF runtime.
 */
public class Particle {

    private double x;
    private double y;
    private int floor;
    private double weight;

    public Particle(double x, double y, int floor, double weight) {
        this.x = x;
        this.y = y;
        this.floor = floor;
        this.weight = weight;
    }

    public Particle(Particle other) {
        this(other.x, other.y, other.floor, other.weight);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public int getFloor() {
        return floor;
    }

    public void setFloor(int floor) {
        this.floor = floor;
    }

    public double getWeight() {
        return weight;
    }

    public void setWeight(double weight) {
        this.weight = weight;
    }
}
