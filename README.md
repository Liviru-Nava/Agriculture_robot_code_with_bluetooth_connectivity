# SEMI-AUTOMATIC AGRICULTURE ROBOT
 This repository will contain the .INO file of the code that was used to implement this robot. This is a semi automatic robot and will perform it's automatic operations when a plot is detected.

 ### DESCRIPTION
 This robot is a semi-automatic robot that is manually controlled by bluetooth via a bluetooth-connected device. By using the controls, we can guide the robot towards a plot which will be detected by the  **IR SENSOR**.
 Once detected, it will perform the automatic functions which are drilling the soil, planting a seed and pumping water to the plot of land **automatically**. Once done, the robot can once again be controlled and guided to the next plot of land where a seed needs to be planted. 

### FEATURES
- Manually controlled by a HC-05 bluetooth module connected with a mobile application.
- Performs the seed planting operation automatically.
- Performs the soil drilling operation automatically.
- Performs the water pumping operation automatically.
- Has one IR sensor mounted to detect the plot of land which has a black colour landmark as an identifier.
- Has LEDs mounted under the vehicle, red to indicate that the automatic operations are in progress and blue to indicate that the vehicle can be moved.
- Has a water level sensor mounted inside the water tank to display the current level of water inside the tank through a LCD_I2C 16x2 display.

### CONTRIBUTERS
This project was done by [Liviru-Nava](https://github.com/Liviru-Nava) , [MR.NHIP](https://github.com/8hirantha) & [Rsarith](https://github.com/SarithRanathunge)
