package frc.robot.nerds.utils;

public class Vec2 {

    private double x, y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public Vec2 add(Vec2 add) {
        return new Vec2(x + add.getX(), y + add.getY());
    }

    public Vec2 subtract(Vec2 add) {
        return new Vec2(x - add.getX(), y - add.getY());
    }

    public boolean equals(Vec2 comparing) {
        return x == comparing.x && y == comparing.y;
    }
}
