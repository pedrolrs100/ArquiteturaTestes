package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CondicoesParadaVertical extends CondicoesGeral {

    private double correnteLimite;

    public CondicoesParadaVertical(DcMotorEx motor){
        super(motor);
    }
    private int limiteCorrete(ElapsedTime time){
        boolean condicaoParadaSurtoEnergia = (motor.getCurrent(CurrentUnit.AMPS) >= correnteLimite);

        if(condicaoParadaSurtoEnergia && time.time() > 0.1 && motor.getCurrentPosition() < 230) {
            return 1;
        }
        return 0;
    }
    private int limiteAltura(int target){
        if(motor.getCurrentPosition() > 2000) return 1;
        return 0;
    }

    private int condicaoParadaVertical(int target,ElapsedTime time){
        return limiteCorrete(time)+limiteAltura(target);
    }
    public boolean CondicaoFinal(int target,ElapsedTime time){
        return super.condicaoParadaGeral(target,50,condicaoParadaVertical(target,time));
    }








}
