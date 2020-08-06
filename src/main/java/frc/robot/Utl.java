package frc.robot;


public final class Utl {

    private Utl() {}

    public static double length(double a, double b) {
        return Math.sqrt((a*a) + (b*b));
    }

    public static double max(double ... values) {
        double max = Double.NEGATIVE_INFINITY;
        for (double v : values) {
            if (v > max) {
                max = v;
            }
        }
        return max;
    }

}
