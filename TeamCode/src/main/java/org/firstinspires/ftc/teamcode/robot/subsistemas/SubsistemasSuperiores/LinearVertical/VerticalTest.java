package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes.Condicoes;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes.CondicoesParadaVertical;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Controladorpidf;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.SistemaLinear;

public class VerticalTest extends SistemaLinear  {

    DcMotorEx motorL,motorR;
    int targetPosition;
    ElapsedTime time;
    private double correnteLimite = 3.14;
    private int alturaMaxima = 2000;


    int portaLinearVerticalDireita,portaLinearVerticalEsquerdo;
    public VerticalTest(HardwareMap hardwareMap){

        this.motorL =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalL);
        this.motorR =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalR);
        Controladorpidf controladorpidf = new Controladorpidf(motorR,0.1,0,0.0008,0.005);
        Condicoes condicoes = new CondicoesParadaVertical(motorR,targetPosition,time,correnteLimite,alturaMaxima);
        super.motor = motorR;
        super.controlador = controladorpidf;
        //super.parada = condicoes;


        portaLinearVerticalDireita = motorR.getPortNumber();
        portaLinearVerticalEsquerdo = motorL.getPortNumber();

        this.motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public Action ElevadorGoTo(){
        motorL.setPower(controlador.PIDF());
        motorR.setPower(controlador.PIDF());
        return super.GoTo(100);
    }




}

