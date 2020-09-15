# Arduino-Inverse-Kinematics-Plot-Clock
Arduino plot clock using inverse kinematics to draw shapes on a white board. Please see the calculations image for the inverse kinematics calculations. 

A plot clock is a robot that writes (plots) the current time. In this case the plot clock is a robotic arm which writes the time on a whiteboard, waits for a minute, erases the written time and then rewrites the new time. The original project can be found at https://www.thingiverse.com/thing:248009 and updates that I also used are at https://www.instructables.com/id/Plot-Clock-for-dummies/. In this project I 3D printed the already existing plot clock and focused on the Arduino code to use the robotic arm to print the time using inverse kinematics calculations. I updated the plot clock to use inverse kinematics equations instead of the trigonometry that was originally in the code. Also, instead of writing the time, I decided to have the plot clock draw shapes. The log of updates can be found below (same log in the Arduino file).

// 1.04 Modified by Alexander Gruhl
//       - added invDrawTo and invSetXY functions to draw in inputted directions on the whiteboard
//       - added cases 14, 15, and 16 in the number function switch statement to draw on whiteboard
//          in different directions
