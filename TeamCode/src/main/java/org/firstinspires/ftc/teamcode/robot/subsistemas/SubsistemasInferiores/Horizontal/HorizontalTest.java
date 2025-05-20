package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Controller.Condicoes.Condicoes;
import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDePosicao.ControladorPIDF;
import org.firstinspires.ftc.teamcode.robot.Controller.SistemaLinear;

public class HorizontalTest extends SistemaLinear {
    DcMotorEx motorH;
    int targetPosition;
    ElapsedTime time;
    private double correnteLimite = 2.2;
    private int extensaoMaxima = 100;


    public  HorizontalTest(HardwareMap hardwareMap){

        motorH = hardwareMap.get(DcMotorEx.class ,"3");
        ControladorPIDF controladorpidf = new ControladorPIDF(motorH,1.4,0,0,0);
        Condicoes condicoes = new CondicoesHorizontal(motorH,targetPosition,time,correnteLimite,extensaoMaxima);
        this.controlador = controladorpidf;
        this.motor = motorH;
        this.condicoesParada = condicoes;

        this.motorH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorH.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public Action HorizontalToGo(int target){
        motorH.setPower(controlador.PIDF());
        return this.GoTo(target);
    }
}
