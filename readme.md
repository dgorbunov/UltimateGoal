### 🤖  FTC Team 11100 We Are Robo's Codebase.

 - Our robot uses three wheel odometry as well as drive encoder localization.
 - Our software is completely abstracted and utilizes many distributed classes that cleanup the code.
 - Instead of using a FSM for our autonomous, we utilize a helper class to run actions in a queue.
 - Most blocking hardware calls are multithreaded
 - We use computer vision algorithms to align with the wobble goal
 - We have a semi-automated teleop with automatic shooting for the goal and powershot.


⚡️Powered by Roadrunner, OpenRC, and FTCDashboard for ultra-fast compile times, quick changes, and precise motion profiling.

Written by [@dgorbunov](https://github.com/dgorbunov).

#### To-Do:
- Fix TeleOp auto shooting and power shots
- Localize with wall corner
- Shoot from start position at auto
- Sensor for rings intaked
- Wobble CV aligner
