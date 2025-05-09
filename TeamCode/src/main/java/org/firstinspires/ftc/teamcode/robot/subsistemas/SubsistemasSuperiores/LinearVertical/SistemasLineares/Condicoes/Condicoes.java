package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.SistemaLinear;

/**
 * Classe Base para passar todas as condições de parada de parada para o {@link SistemaLinear},
 * <p>
 * Esse negocio é muito maneiro papo reto
 */
@SuppressWarnings("unused")
public class Condicoes {
    DcMotorEx motor;
    int alvo;
    public ElapsedTime time;

    public Condicoes(
            DcMotorEx motor,
            int alvo,
            ElapsedTime time
    ){
        this.motor = motor;
        this.alvo = alvo;
        this.time = time;
    }
    public boolean chegouAlvoPID( double margem){

        double erro = 0;
        double atual = motor.getCurrentPosition();
        erro = alvo - atual;
        // abs(erro) < margem -> true : false
        return Math.abs(erro) < margem ;
    }

    public boolean condicaoParadaFinal(){
        return false;
    }

}
