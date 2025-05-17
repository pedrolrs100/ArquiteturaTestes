package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.BracoGarra;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Controller.Condicoes.Condicoes;

public class CondicoesBraco extends Condicoes {
    public CondicoesBraco(
            DcMotorEx motor,
            int alvo,
            ElapsedTime time
    ){
        super(motor, alvo, time);
    }
    @Override
    public boolean condicaoParadaFinal(){
        return false;
    }
}
