package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.BracoGarra;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.HashMap;

public class BracoGarraInferior {
    public Servo bracoGarraInferior;
    public double ServoBracoInferiorPosition;
    public BracoGarraInferiorStates bracoGarraInferiorState = BracoGarraInferiorStates.INITIAL;
    public HashMap<BracoGarraInferiorStates, Double> mapBraco = new HashMap<>();

    public BracoGarraInferior(HardwareMap hardwareMap){

        bracoGarraInferior = hardwareMap.get(Servo.class, HardwareNames.bracoGarraInferiorServo);
        mapBraco.put(BracoGarraInferiorStates.READY_TOINTAKE, 0.1);
        mapBraco.put(BracoGarraInferiorStates.INITIAL, 0.314);
        mapBraco.put(BracoGarraInferiorStates.TRASNFER,0.353);
        mapBraco.put(BracoGarraInferiorStates.INTAKE,0.026);
    }
    public Action goToReadytoIntake(){//todo okey


        return new InstantAction(() -> {
            bracoGarraInferiorState = BracoGarraInferiorStates.READY_TOINTAKE;

            ServoBracoInferiorPosition = mapBraco.get(bracoGarraInferiorState);
        });
    }
    public Action goToInitial(){// todo okey


        return new InstantAction(() -> {
            bracoGarraInferiorState = BracoGarraInferiorStates.INITIAL;

            ServoBracoInferiorPosition = mapBraco.get(bracoGarraInferiorState);
        });
    }
    public Action goToIntake(){// todo okey


        return new InstantAction(() -> {
            bracoGarraInferiorState = BracoGarraInferiorStates.INTAKE;

            ServoBracoInferiorPosition = mapBraco.get(bracoGarraInferiorState);
        });
    }

    public Action goToTransfer(){//todo okey


        return new InstantAction(() -> {
            bracoGarraInferiorState = BracoGarraInferiorStates.TRASNFER;

            ServoBracoInferiorPosition = mapBraco.get(bracoGarraInferiorState);
        });



    }


}
