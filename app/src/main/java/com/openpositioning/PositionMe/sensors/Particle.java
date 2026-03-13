package com.openpositioning.PositionMe.sensors;

/**
 * Minimal mutable particle state used by the PF runtime.
 */
public class Particle {

    private double x;
    private double y;
    private int floor;
    private double weight;
    private double headingRad;

    public Particle(double x, double y, int floor, double weight) {
        this(x, y, floor, weight, 0.0);
    }

    public Particle(double x, double y, int floor, double weight, double headingRad) {
        this.x = x;
        this.y = y;
        this.floor = floor;
        this.weight = weight;
        this.headingRad = headingRad;
    }

    public Particle(Particle other) {
        this(other.x, other.y, other.floor, other.weight, other.headingRad);
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

    public double getHeadingRad() {
        return headingRad;
    }

    public void setHeadingRad(double headingRad) {
        this.headingRad = headingRad;
    }
}
