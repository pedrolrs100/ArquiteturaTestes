package org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDePosicao;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ControladorPIDFangle extends ControladorPIDF{
    public double KdA;
    public double targetAngle;
    public double ticks_in_degree = 8192 / 360 ;
    public double angle = 0;
    public double pid = 0;
    public ControladorPIDFangle(
            DcMotorEx motor,
            double P ,
            double I ,
            double D ,
            double F,
            double KdA
    ){
        super(motor, P, I, D, F);
        this.KdA = KdA;
        controller = new PIDController(p, i, d);
    }
    @Override
    public double PIDF(){
        double kp = p;
        double kdA = f;


        //Cria o Controlador PID
        PIDController controller = new PIDController(p, 0, kdA);
        controller.setPID(p, 0, kdA);

        //Calcular correção
        this.targetAngle = targetPosition / ticks_in_degree - 235;
        this.angle = motor.getCurrentPosition() / ticks_in_degree - 235;
        double error = targetAngle - angle;

        //pid = controller.calculate(this.getPosition(),targetPosition );//motionProfile(maxAcceleration, maxVelocity, distance, elapsedTime
        pid = controller.calculate(this.angle, targetAngle);



        int linearpos = motor.getCurrentPosition();

        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition)) * f;

        return pid + ff;
    }
}
