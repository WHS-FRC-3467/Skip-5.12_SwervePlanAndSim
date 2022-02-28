package frc.swervelib.ctre;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.Gyroscope;

public class Pigeon2FactoryBuilder {
    private static BasePigeonSimCollection pigeonSim;

    public Gyroscope build(WPI_Pigeon2 pigeon) {
        return new GyroscopeImplementation(pigeon);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final WPI_Pigeon2 pigeon;

        private GyroscopeImplementation(WPI_Pigeon2 pigeon) {
            this.pigeon = pigeon;
            pigeonSim = pigeon.getSimCollection();
        }

        @Override
        public Rotation2d getGyroHeading() {
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }

        @Override
        public Boolean getGyroReady() {
            return true; //pigeon.getState().equals(PigeonState.Ready);
        }
        
        @Override
        public void zeroGyroscope() {
            pigeon.reset();
        }

        @Override
        public void setAngle(double angle) {
            pigeonSim.setRawHeading(angle);
        }
    }
}