// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
// import com.ctre.phoenix6.configs.LEDConfigs;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.Enable5VRailValue;
// import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
// import com.ctre.phoenix6.signals.RGBWColor;
// import com.ctre.phoenix6.signals.StripTypeValue;
// import com.ctre.phoenix6.signals.VBatOutputModeValue;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class LEDMinimal extends SubsystemBase {
//   @SuppressWarnings("deprecated")
//   private final CANdle candle = new CANdle(4, "DriveCanivore");

//   // Changes LEDs to a Solid Blue Colour
//   private final SolidColor blueRequest =
//       new SolidColor(0, 7)
//           .withLEDStartIndex(0) // The First LED INDEX NUMBER
//           .withLEDEndIndex(7) // The Last LED INDEX NUMBER
//           .withColor(new RGBWColor(0, 0, 255)); // Pure Blue

//   // These are Variables that state what status the robot is in
//   private StatusCode lastConfigStatus = StatusCode.StatusCodeNotInitialized;
//   private StatusCode lastSetControlStatus = StatusCode.StatusCodeNotInitialized;

//   // States Everything the LEDMinimal is supposed to do
//   public LEDMinimal() {
//     CANdleConfiguration config = new CANdleConfiguration(); // Configures the CANdle

//     // Below Configures What The LEDs Are Supposed To Do
//     config.withLED(
//         new LEDConfigs()
//             .withBrightnessScalar(1.0)
//             .withStripType(StripTypeValue.RGB)
//             .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning));

//     // Configures CANdle's Features
//     config.withCANdleFeatures(
//         new CANdleFeaturesConfigs()
//             .withEnable5VRail(Enable5VRailValue.Enabled)
//             .withVBatOutputMode(VBatOutputModeValue.On));

//     lastConfigStatus = candle.getConfigurator().apply(config);

//     candle.clearAllAnimations();
//     lastSetControlStatus = candle.setControl(blueRequest);

//     // Logging on SmartDashboard
//     SmartDashboard.putString("LEDMinimal/CANdleConfig", lastConfigStatus.toString());
//     SmartDashboard.putBoolean("LEDMinimal/CANdleConfigOK", lastConfigStatus.isOK());
//     SmartDashboard.putString("LEDMinimal/CANdleSetControl", lastSetControlStatus.toString());
//     SmartDashboard.putBoolean("LEDMinimal/CANdleSetControlOK", lastSetControlStatus.isOK());
//   }

//   @Override
//   public void periodic() {
//     lastSetControlStatus = candle.setControl(blueRequest);

//     // These Logging Commands Constantly Run When Robot Is On
//     SmartDashboard.putString("LEDMinimal/CANdleConfig", lastConfigStatus.toString());
//     SmartDashboard.putBoolean("LEDMinimal/CANdleConfigOK", lastConfigStatus.isOK());
//     SmartDashboard.putString("LEDMinimal/CANdleSetControl", lastSetControlStatus.toString());
//     SmartDashboard.putBoolean("LEDMinimal/CANdleSetControlOK", lastSetControlStatus.isOK());
//   }
// }
