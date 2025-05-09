package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes.CondicoesGeral;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes.CondicoesParadaVertical;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Controladorpidf;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.SistemaLinear;

public class VerticalTest extends SistemaLinear  {

    DcMotorEx motorL,motorR;


    int portaLinearVerticalDireita,portaLinearVerticalEsquerdo;
    public VerticalTest(HardwareMap hardwareMap){

        this.motorL =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalL);
        this.motorR =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalR);
        Controladorpidf controladorpidf = new Controladorpidf(motorR,0.1,0,0.0008,0.005);
        CondicoesGeral condicoes = new CondicoesParadaVertical(motorR);
        super.motor = motorR;
        super.controlador = controladorpidf;
        super.parada = condicoes;


        portaLinearVerticalDireita = motorR.getPortNumber();
        portaLinearVerticalEsquerdo = motorL.getPortNumber();

        this.motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public Action elevadorGoTo(){
        motorL.setPower(controlador.PIDF());
        motorR.setPower(controlador.PIDF());
        return super.GoTo(100);
    }




}

