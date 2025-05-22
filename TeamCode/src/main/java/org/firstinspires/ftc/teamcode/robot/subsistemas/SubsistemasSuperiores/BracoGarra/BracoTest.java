package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.BracoGarra;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Controller.Condicoes.Condicoes;
import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDePosicao.ControladorPIDF;
import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDePosicao.ControladorPIDFangle;
import org.firstinspires.ftc.teamcode.robot.Controller.SistemaLinear;

public class BracoTest extends SistemaLinear {
    DcMotorEx motor;
    int targetposition ;
    ElapsedTime time;
    double correnteLimite ,limiteMecanico;
    public BracoTest(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class,"MotorBraco");
        ControladorPIDF controlador = new ControladorPIDFangle(motor , 0.1,0,2,3,0.2);
        Condicoes condicoes = new CondicoesBraco(motor,targetposition ,time,correnteLimite,limiteMecanico);
    }
}
