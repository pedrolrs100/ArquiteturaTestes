package org.firstinspires.ftc.teamcode.common;

public class PDFL {

    private  double kP, kD, kF, kL;

    public PDFL(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public double run(double error) {

        double p = pComponent(0);
        double d = dComponent(0, 0);
        double f = fComponent();
        double l = lComponent(0);

        double response = p + d + f + l;

        return response;
    }
    private double pComponent(double error) {

        double response = kP * error;

        return 0;
    }
    private double dComponent(double delta_error, double delta_time) {
        return 0;
    }
    private double fComponent() {
        return 0;
    }
    private double lComponent(double error) {
        return 0;
    }

}
