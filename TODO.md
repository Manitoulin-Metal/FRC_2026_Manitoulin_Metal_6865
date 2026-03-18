# Kicker Motor Debug & Fix TODO

**Completed Steps:**
- [x] Step 1: Added telemetry (ShooterRPS, Kicker/Condition, Kicker/SetSpeed, Shooter/VelocityRPS) to SmartDashboard in Kicker/Shooter periodic().
- [x] Added test override: Hold driver LT (controller LT>0.5) to run kicker at -0.3 speed directly.

**Next Steps:**
- [ ] Build & deploy: `./gradlew deploy` (Windows: gradlew.bat deploy)
- [ ] Test on physical robot:
  * Check Phoenix Tuner X / REV Hardware Client: SparkMax ID62 present? Faults? Firmware?
  * Teleop: Hold driver LT → kicker should run (test wiring/motor).
  * Hold op Y → shooter ~95 RPS, condition true if >93.3 RPS, kicker auto -0.5.
  * Monitor Shuffleboard/AdvantageScope dashboard values live.
- [ ] If kicker no spin on LT: Hardware (CAN wire/power/motor).
- [ ] If spins on LT but not auto: Lower threshold in Constants.SHOOTER_KICKER_RPM_THRESHOLD or tune shooter PID.
- [ ] Revert test button after confirmed working.

