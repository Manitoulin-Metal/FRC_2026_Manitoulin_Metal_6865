# Make Autos Use Commands for Real

## Steps:
- [ ] 1. Move NamedCommands registrations BEFORE the PathPlanner auto loading loop in RobotContainer.java constructor
- [ ] 2. Add missing NamedCommands like "timedShootCommand"
- [ ] 3. Verify build with `gradlew build`
- [ ] 4. Test PathPlanner autos on robot (e.g. Centre Auto Blue verifies ClimbAutoDown runs)
- [ ] 5. Add more NamedCommands for other events as needed (intake, shoot etc.)

Current status: Starting edits.

