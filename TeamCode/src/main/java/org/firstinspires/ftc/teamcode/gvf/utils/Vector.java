package org.firstinspires.ftc.teamcode.gvf.utils;

import androidx.annotation.NonNull;

import org.opencv.core.Point;

public class Vector {
    private double x,y,z;

    public Vector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector(double x, double y){
        this(x,y,0);
    }

    public Vector(){
        this(0,0,0);
    }

    public Vector(Vector otherVector){
        this.x = otherVector.x;
        this.y = otherVector.y;
        this.z = otherVector.z;
    }
    public Vector(Point otherPoint){
        this.x = otherPoint.x;
        this.y = otherPoint.y;
        this.z = 0;
    }

    public static Vector fromAngleAndMagnitude(double t, double m){
        return new Vector(m*Math.cos(t), m*Math.sin(t));
    }

    //t1 is angle in xy plane, t2 is angle with xy plane
    public static Vector fromAngleAndMagnitude(double t1, double t2, double m){
        return new Vector(Math.cos(t1)*Math.cos(t2)*m, Math.sin(t1)*Math.cos(t2)*m, Math.sin(t2)*m);
    }

    public void setY(double y)
    {
        this.y = y;
    }

    public void setX(double x)
    {
        this.x = x;
    }

    public void setZ(double z)
    {
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getMagnitude(){
        return Math.sqrt(x*x + y*y + z*z);
    }

    public Vector plus(Vector other){
        return new Vector(x + other.getX(), y + other.getY(), z + other.getZ());
    }

    public void scaleToMagnitude(double targetMagnitude){
        double currentMagnitude = getMagnitude();
        scaleBy(1.0/currentMagnitude);
        scaleBy(targetMagnitude);
    }

    public void scaleBy(double a){
        x = x * a;
        y = y * a;
        z = z * a;
    }
    public Vector rotated(double theta)
    {
        return new Vector(Math.cos(theta) * x + Math.sin(theta) * y, Math.cos(theta) * y - Math.sin(theta) * x);
    }

    public Vector scaledBy(double a){
        return new Vector(x * a, y * a, z * a);
    }

    public Vector scaledToMagnitude(double targetMagnitude){
        Vector aux = new Vector(this);
        aux.scaleToMagnitude(targetMagnitude);
        return aux;
    }

    public Vector subtract(Vector v)
    {
        return new Vector(x - v.x, y - v.y, z - v.z);
    }
    public Vector subtract(Point p)
    {
        return new Vector(x - p.x, y - p.y);
    }
    public static Vector  rotateBy(Vector vector, double theta){
        return new Vector(Math.cos(theta) * vector.getX() + Math.sin(theta) * vector.getY(), Math.cos(theta) * vector.getY() - Math.sin(theta) * vector.getX());
    }


    @NonNull
    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(z);
    }
    public Pose toPose()
    {
        return new Pose(x, y, z);
    }
    public double getMagSqr()
    {
        return x * x + y * y + z * z;
    }

    /** Keep in mind result in rad **/
    public double getAngle()
    {
        return Math.atan2(y, x);
    }
}
