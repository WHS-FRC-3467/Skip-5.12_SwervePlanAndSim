package frc.swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import frc.swervelib.ctre.Pigeon2FactoryBuilder;
import frc.swervelib.ctre.PigeonFactoryBuilder;

public final class GyroscopeHelper {
    private GyroscopeHelper() {
    }

    public static Gyroscope createPigeonController(TalonSRX controller) {
        WPI_PigeonIMU pigeon = new WPI_PigeonIMU(controller);
        return new PigeonFactoryBuilder().build(pigeon);
    }

    public static Gyroscope createPigeonCAN(Integer id) {
        WPI_PigeonIMU pigeon = new WPI_PigeonIMU(id);
        return new PigeonFactoryBuilder().build(pigeon);
    }

    public static Gyroscope createPigeon2CAN(Integer id) {
        WPI_Pigeon2 pigeon = new WPI_Pigeon2(id);
        return new Pigeon2FactoryBuilder().build(pigeon);
    }
}
