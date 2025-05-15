package org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDePosicao;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Controladorpidf {
    PIDController controller;
    double p,i,d,f;
    int targetPosition;
    DcMotorEx motor;
    public Controladorpidf
    (
        DcMotorEx motor,
        double P ,
        double I ,
        double D ,
        double F
    )
    {
        this.motor = motor;
    /**/
        this.p = P;
        this.i = I;
        this.d = D;
        this.f = F;
        controller = new PIDController(p, i, d);

    }
    public double PIDF(){
        double kp = p;
        int linearpos = motor.getCurrentPosition();

        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition)) * f;

        return pid + ff;
    }
}


