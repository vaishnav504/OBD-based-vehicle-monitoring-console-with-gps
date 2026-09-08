# OBD-based-vehicle-monitoring-console-with-gps

The project involves the construction of a multifunctional, real-time vehicle monitoring console using ESP32, an ELM327 OBD-II adapter, and a GPS module. Vehicle parameters including speed, RPM, All engine data, and diagnostic information are gathered via the OBD interface, and the GPS module supplies location and navigation-related data.

The system features several display interfaces, such as a MAX7219 4-in-1 LED matrix, an LCD, and an OLED, which enable different vehicle parameters to be shown at the same time. Each of the displays can be individually turned on or off and set to show the desired information. Additionally, a servo motor is assigned so that a chosen vehicle parameter is displayed in the form of an analog gauge.

The OLED is fitted with a menu-driven configuration system which uses a potentiometer and a push button, so that the user is able to browse through the settings, choose the parameters, set the displays, and alter the operating modes without the need for a computer or a mobile application.

Because of its modular design it is possible to use a number of different monitoring modes and a personalised dashboard experience can be provided, the dashboard combining digital displays, graphical indicators, GPS information and analog-style servo instrumentation into one embedded vehicle monitoring console.
