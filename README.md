# Usage:
1. Go to include folder and locate main.h
2. Below code line 37, uncomment out the line with your robot's color and save the file
3. Double check Brain's port connections in definitions_(robot color).h
4. Tune PID values under the same header file as above
5. In main.c, locate void sortingTask() function's while loop and uncomment/comment out code based on Alliance Team's color
# In main.c, the turning degrees values (when turning exactly 90 deg) that work best for the respective robots are:
   * Orange robot: 90
   * Blue robot: 87.5
   * Pink robot: (untested)
# Controls:
   * **R1/R2** - Up/Down motion for Large rock intake
   * **L1/L2** - In/Out motion for Small rock intake
   * **A** - Toggle TankDrive
   * **B** - Toggle Sorting sensor and motor
   * **Y** - Autonomous movement
   
