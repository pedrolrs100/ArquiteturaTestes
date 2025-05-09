package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.Garra;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.V5Modes;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.GarraAngulationStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.GarraOpeningStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.common.Garra.Garra;

import java.util.HashMap;

@Config
public class GarraSuperior extends Garra {

    public double angulacaoSuperiorPosition;
    public Servo servoRotacaoDaGarra;
    public GarraSuperiorRotetionStates garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;
    public static double positionTransfer  = 485/*0.6127777777777779*/ /* 0.485*/ ,getPositionTransferRotacion =0.6888;
    double delay = 0.3;
    double cooldownRotacaoGarra;
    public double ServoRotacaoInferiorPosition,ServoAberturaInferiorPoition,ServoAngulacaoPosition;
    public HashMap<GarraSuperiorRotetionStates, Double> mapRotation = new HashMap<>();

    public static double angReadyToHangPos = 0, angHangPos = 0;
    public GarraSuperior(HardwareMap hardwareMap) {

        super(hardwareMap,HardwareNames.aberturaGarraSuperiorServo, HardwareNames.angulacaoGarraSuperiorServo);
        servoRotacaoDaGarra = hardwareMap.get(Servo.class, HardwareNames.rotacaoGarraSuperiorServo);
        mapOpening.put(GarraOpeningStates.OPEN, 0.646);//todo okey
        mapOpening.put(GarraOpeningStates.CLOSED, 1.0);//todo okey
        mapOpening.put(GarraOpeningStates.HALF, 0.646);// transfer

        mapAngulation.put(GarraAngulationStates.TRANSFER, positionTransfer);//todo okey não
        mapAngulation.put(GarraAngulationStates.OUTTAKE_SAMPLE,0.13277777777777777);//todo okey

        mapAngulation.put(GarraAngulationStates.INTAKE_SPECIMEN,  0.14111111111111113);//todo okey
        mapAngulation.put(GarraAngulationStates.OUTTAKE_SPECIMEN,  0.6577777777777778  );//todo okey
        mapAngulation.put(GarraAngulationStates.OUTTAKE_EJECTING,0.379);
        mapAngulation.put(GarraAngulationStates.READY_TO_HANG,angReadyToHangPos);
        mapAngulation.put(GarraAngulationStates.HANG,angHangPos);
        // 0.379
        mapAngulation.put(GarraAngulationStates.READY_OUTTAKE,0.512);//todo okey
        mapAngulation.put(GarraAngulationStates.INITIAL_SPECIMEN,0.647);
        mapAngulation.put(GarraAngulationStates.ANGULAR_TRANSFER,0.678);

        mapRotation.put(GarraSuperiorRotetionStates.PARALELA, 0.016);//todo okey
        mapRotation.put(GarraSuperiorRotetionStates.PERPENDICULAR, 0.348);//todo okey -> transfer
        mapRotation.put(GarraSuperiorRotetionStates.CHAMBER, 0.675);//todo okey
        mapRotation.put(GarraSuperiorRotetionStates.TRANSFER,getPositionTransferRotacion);//todo não testado

        // 0.678
    }


    /***********************************************/
    /************ Sample functions *****************/
    /***********************************************/
    public Action goToReadyHang(){
                return new InstantAction(() -> {
                    garraAngulationState = GarraAngulationStates.READY_TO_HANG;
                    garraRotationSuperiorState = GarraSuperiorRotetionStates.PERPENDICULAR;
                    servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
                    servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
        });
    }

    public Action goToHang(){
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.HANG;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PERPENDICULAR;
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
        });
    }
    public Action gerenciadorDeRotacaoDaGarraNoTeleop(double runTime, V5Modes v5Mode) {

        if(runTime < this.cooldownRotacaoGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownRotacaoGarra= runTime + this.delay;

        if(v5Mode == V5Modes.SAMPLE){
            if (this.garraRotationSuperiorState == GarraSuperiorRotetionStates.PERPENDICULAR) {
                return this.garraChamber();
            }
            return this.garraPerpendicular();
        }
        else {
            if (this.garraRotationSuperiorState == GarraSuperiorRotetionStates.CHAMBER) {
                return this.garraParalela();
            }
            return this.garraChamber();

        }
    }

    public Action garraParalela() {

        return new InstantAction(() -> {
            this.garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;
            servoRotacaoDaGarra.setPosition(mapRotation.get(this.garraRotationSuperiorState));
        });
    }

    public Action garraPerpendicular(){
         return new InstantAction(() -> {
             this.garraRotationSuperiorState = GarraSuperiorRotetionStates.PERPENDICULAR;
             servoRotacaoDaGarra.setPosition(mapRotation.get(this.garraRotationSuperiorState));
         });
    }
    public Action garraChamber(){
        return new InstantAction(() -> {
            this.garraRotationSuperiorState = GarraSuperiorRotetionStates.CHAMBER;
            servoRotacaoDaGarra.setPosition(mapRotation.get(this.garraRotationSuperiorState));
        });
    }
    public Action angularTransfer(){
        return new InstantAction(() -> {
            this.garraAngulationState = GarraAngulationStates.ANGULAR_TRANSFER;
            servoAngulacaoGarra.setPosition(mapAngulation.get(this.garraAngulationState));
        });
    }
    public Action goToOuttakeEjecting(){//todo
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.OUTTAKE_EJECTING;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;

            //aberturaGarraSuperiorServo.getController().pwmDisable();
            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }
    public Action goToOuttakeSample(){

        return new InstantAction(()->{
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationStates.OUTTAKE_SAMPLE;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PERPENDICULAR;//CHAMBER

            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }
    public Action goToOuttakeAngulation(){
        return new InstantAction(()->{

            garraAngulationState = GarraAngulationStates.OUTTAKE_SAMPLE;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PERPENDICULAR;//CHAMBER

            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }
    public Action goToTransfer(){
        return new InstantAction(() -> {

            garraOpeningState = GarraOpeningStates.OPEN;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.TRANSFER;
            garraAngulationState = GarraAngulationStates.TRANSFER;

            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));

        });
    }



    /***********************************************/
    /************ Specimen functions ***************/
    /***********************************************/
    public Action goToInitialSpecimen(){
        return new InstantAction(() -> {
            garraRotationSuperiorState = GarraSuperiorRotetionStates.CHAMBER;
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationStates.INITIAL_SPECIMEN;
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));


        });
    }
    public Action goToOuttakeSpecimen(){//todo
        return new InstantAction(() -> {
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationStates.OUTTAKE_SPECIMEN;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.CHAMBER;

            //aberturaGarraSuperiorServo.getController().pwmDisable();
            servoAberturaDaGarra.setPosition(mapOpening.get(garraOpeningState));
            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }
    public Action goToReadOuttakeSpecimen(){//todo
        return new InstantAction(() -> {
            garraAngulationState = GarraAngulationStates.READY_OUTTAKE;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.CHAMBER;
            angulacaoSuperiorPosition = mapAngulation.get(garraAngulationState);

            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
        });
    }
    public Action goToIntakeSpecimen(){//todo
        return new InstantAction(() ->{
            garraAngulationState = GarraAngulationStates.INTAKE_SPECIMEN;
            garraRotationSuperiorState = GarraSuperiorRotetionStates.PARALELA;
            servoRotacaoDaGarra.setPosition(mapRotation.get(garraRotationSuperiorState));
            servoAngulacaoGarra.setPosition(mapAngulation.get(garraAngulationState));
        } );
    }


    public void upSetPoint(double increase) {
        double position = Range.clip(servoAngulacaoGarra.getPosition() + (increase * 0.005),0,1);
        servoAngulacaoGarra.setPosition(position);
    }
    public void downSetPoint(double decrease) {
        double position = Range.clip(servoAngulacaoGarra.getPosition() + (decrease * 0.005),0,1);
        servoAngulacaoGarra.setPosition(position);
    }

    public void upSetPointRot(double increase) {
        double position = Range.clip(servoRotacaoDaGarra.getPosition() + (increase * 0.01),0,1);
        servoRotacaoDaGarra.setPosition(position);
    }
    public void downSetPointRot(double decrease) {
        double position = Range.clip(servoRotacaoDaGarra.getPosition() + (decrease * 0.01),0,1);
        servoRotacaoDaGarra.setPosition(position);
    }



    public void monitor(Telemetry telemetry) {
        super.monitor(telemetry);
        telemetry.addData("GARRA rotação POSIÇÃO: ", servoRotacaoDaGarra.getPosition());
        telemetry.addData("GARRA rotação PWM",servoRotacaoDaGarra.getController().getPwmStatus());
        telemetry.addData("GARRA rotação Atual",garraRotationSuperiorState);

    }
}
