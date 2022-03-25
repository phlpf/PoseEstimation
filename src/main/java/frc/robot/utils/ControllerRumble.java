package frc.robot.utils;

public class ControllerRumble {
    private double rumbleAmplitude = 0;
    private double rumbleSeconds = 0;

    public ControllerRumble(double rumbleAmplitude, double rumbleSeconds) {
        this.rumbleAmplitude = rumbleAmplitude;
        this.rumbleSeconds = rumbleSeconds;
    }

    public double getCurrentRumble() {
        rumbleSeconds -= 0.02;
        if(rumbleSeconds <= 0) rumbleAmplitude = 0;
        return rumbleAmplitude;
    }
}
