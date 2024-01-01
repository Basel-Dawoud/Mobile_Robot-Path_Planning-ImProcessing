# Mobile Robot Project with MATLAB

This MATLAB project focuses on implementing path planning using the Rapidly-exploring Random Tree (RRT) algorithm for a mobile robot in an environment with obstacles. The code reads an image of the environment, processes it to detect obstacles, plans a path from a starting point to a specified goal, and visualizes the path found by the RRT algorithm.

## Getting Started

### Prerequisites

Make sure your MATLAB installation includes the following toolboxes:

Image Processing Toolbox: Used for various image manipulation operations, such as image import, edge detection, and morphological operations.
Statistics and Machine Learning Toolbox: Utilized for statistical analysis and outlier correction in the provided data.
Optimization Toolbox: Used for various optimization techniques, possibly employed in trajectory planning or obstacle avoidance strategies.
If any of these toolboxes are missing, install them through MATLAB's Add-Ons menu or ensure their availability through your MATLAB.

### Usage

1. Open MATLAB.

2. Navigate to the directory where the project files are located using the MATLAB command window or Current Folder pane.

3. Open the MATLAB script named `MobileRobotProject.m` in the MATLAB editor or command window.

4. Update the path (line 141) for the Excel data file in the MATLAB script:

filename = '/path/to/your/Data.xlsx'; % Replace '/path/to/your/Data.xlsx' with the correct file path.

5. Save the changes to the script.

6. Run the script in MATLAB. This will execute the mobile robot project code.

## Functionality

- **Image Import:** The code prompts the user to select an image file (e.g., `.jpg`, `.png`) that represents the environment where the robot navigates. Select the file named "RoomImage".

- **Image Processing:** The selected image is processed to detect edges, dilate edges, and identify obstacles.

- **RRT Path Planning:** The Rapidly-exploring Random Tree (RRT) algorithm is used for path planning from the starting point to the specified goal while avoiding obstacles.

- **Visualization:** The code visualizes the detected obstacles, the planned path, and the robot's trajectory on the map.

- **Excel Data Analysis:** The code reads and analyzes data from an Excel file "Data.xlsx" related to obstacle distances detected by the robot.

## Data File

The project utilizes a data file obtained from the Arduino, which includes readings from the ultrasonic sensor and servo motor that rotates from 0 to 180 degrees. You can access the videos of the process througth the following links:

https://drive.google.com/file/d/1Wd-BzFap_OKcwnW8eeuGfH13e6g5GeTT/view?usp=sharing
https://drive.google.com/file/d/1N2X1VMb_BLQYty33er5Lbe9-iFvkOiw5/view?usp=sharing

## Acknowledgments

## RRT Algorithm

The RRT (Rapidly-exploring Random Tree) algorithm is a path planning technique employed in this project. If you're looking for an in-depth understanding of the RRT algorithm, you can refer to the following resource:

https://youtu.be/QR3U1dgc5RE?si=DEVVnwrChGER6CrS

## Authors

- Basel Ahmed Dawoud
- Adham Ashraf Salah
- Elsayed Mohamed Fahmy

## Contact Information

For any inquiries or assistance regarding this project, feel free to contact ue:

- Email: Baseldawoud@gmail.com
- WhatsApp: +20 102 181 1895
- Email: adhamashraf0114208@gmail.com
- WhatsApp: +20 114 208 7556
- Email: sedo5970@gmail.com
- WhatsApp: +20 109 369 3844

Please don't hesitate to reach out if you have questions, suggestions, or need further clarification. I'll be happy to assist you!
