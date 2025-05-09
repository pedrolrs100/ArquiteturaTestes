package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Controladorpidf;

public class CondicoesGeral {
    DcMotorEx motor;
    public CondicoesGeral(
            DcMotorEx motor
    ){
        this.motor = motor;
    }

    public boolean condicaoParadaGeral(double alvo, double margem,int condicoesPlus){
        //todo: chegou no alvo
        double erro = 0;
        double atual = motor.getCurrentPosition();
        erro = alvo - atual;
        // abs(erro) < margem -> true : false
        if(Math.abs(erro) < margem) return true;

        //todo: condições fora dessa função
        if(condicoesPlus>0) return true;//PODE SER UM ERRO COLOSSAL

        //todo: senão
        return false;

    }


    public boolean chegouAlvoPID(double alvo, double margem){

        double erro = 0;
        double atual = motor.getCurrentPosition();
        erro = alvo - atual;
        // abs(erro) < margem -> true : false
        return Math.abs(erro) < margem ;
    }

}
