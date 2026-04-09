package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDMinimal extends SubsystemBase {
    @SuppressWarnings({ "deprecated", "removal" })
    private final CANdle candle = new CANdle(4, "DriveCanivore");

    private final SolidColor blueRequest = new SolidColor(0, 7).withColor(new RGBWColor(0, 0, 255));

    public LEDMinimal() {
        candle.clearAllAnimations();
        candle.setControl(blueRequest);
    }

    @Override
    public void periodic() {
        candle.setControl(blueRequest);
    }
}