package frc.robot;


public final class Utl {

    private Utl() {}

    public static double length(double a, double b) {
        return Math.sqrt((a*a) + (b*b));
    }

    public static double max(double ... values) {
        double max = 0.0;
        for (double v : values) {
            if (v > max) {
                max = v;
            }
        }
        return max;
    }

}
