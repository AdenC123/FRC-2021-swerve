package frc.robot;



public final class Utl {

    public static final double PI = Math.PI;
    public static final double NEG_PI = -Math.PI;
    public static final double TWO_PI = Math.PI * 2.0;
    public static final double PI_OVER_2 = Math.PI * 0.5;
    public static final double NEG_PI_OVER_2 = -(Math.PI * 0.5);

    private Utl() {}

    public static double length(double ... values) {
        double lengthSquared = 0.0;
        for (double v : values) {
            lengthSquared += v * v;
        }
        return Math.sqrt(lengthSquared);
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
