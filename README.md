This project reads numerical depth from SonarPhone WiFi and sends SeaTalk data to a chart plotter.

This is a stand-alone ESP32 microcontroller with wifi interface to acquire depth data from a SonarPhone SP200A as a single or second master, not as a slave. The SonarPhone app can also run as a master at the same time but is not required. Settings follow the phone app master's last settings. This project does not interfere with any of the original SonarPhone app functionality. Depth is transmitted on SeaTalk to a Raymarine C70 or other compatable chart plotter. Units are properly displayed according to the chart plotter settings independantly of the SonarPhone master settings.

Any comments, additions, or corrections are welcome.

This fork is modified original code to communicate via esphome-uart-p2p, github reference: https://github.com/KG3RK3N/esphome-uart-p2p
