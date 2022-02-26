package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class PigeonWrapper extends WPI_Pigeon2 {
    public PigeonWrapper(int id) {
        super(id);
        System.out.println("--------------------------");
        if (isPigeonConnected()) {
            System.out.println("---- PIGEON CONNECTED ----");
        } else {
            System.out.println("----- PIGEON MISSING -----");
        }
        System.out.println("--------------------------");
    }

    @Override
    public double getYaw() {
        if(isPigeonConnected()) {
            return super.getYaw();
        }
        return 0;
    }

    @Override
    public void reset() {
        if(isPigeonConnected()) {
            super.reset();
        }
    }

    // TODO: Pigeon checking error is still a call to the pigeon hardware; find an alternative method to check pigeon connection
    public boolean isPigeonConnected() {
        return getLastError() == ErrorCode.OK;
    }
}
