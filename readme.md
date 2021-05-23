### 🤖  FTC Team 11100 We Are Robo's Codebase.

 - Our robot uses three wheel odometry and drivetrain encoders to localize our field position.
 - Our software is completely abstracted and utilizes many distributed classes that cleanup the code.
 - Instead of using a FSM for our autonomous, we utilize a helper class to run actions in a threaded queue.
 - All blocking method calls are spawned on a new thread.
 - We use OpenCV to detect the starter stack size, detect rolling rings, and detect the wobble goal.
 - We have a semi-automated teleop that automatically angles our turret and calculates motor power values for the high goal and powershot.

Won the Control Award in our Lexington Qualifier and the Innovate design award at MA States.

⚡️Powered by OpenRC, Roadrunner, FTCDashboard, and RevExtensions.

Written by [@dgorbunov](https://github.com/dgorbunov).
