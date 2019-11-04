This plugin was adapted from gareth-cross/rviz_satellite on GitHub.
Changes were made to the source code:
  1. Instead of subscribing to a NavSatFix topic, it now subscribes to Odometry (/odom for nav8) and PointStamped (/lla for nav8).
  2. Demo .rviz was adapted to the new navigator.rviz
  3. Default zoom was edited to match the satellite imagery with the navigator's enu frame.

Aeyzechiah Vasquez
azvasquez99@gmail.com
