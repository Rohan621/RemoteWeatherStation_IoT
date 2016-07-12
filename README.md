# RemoteWeatherStation_IoT
First Project @ GitHub. Class project

Hi Guys,

This is my first GitHub Project :D. 

This project had two nodes, a Remote Weather Station & Graphics Display Node. Both these nodes communicate to each other through Ethernet protocol.

The Remote Weather Station mainly consists of two sensors, Barometric Pressure with temperature (BMP180), & Relative Humidity (DHT11). The Barometric Temperature sensor was interfaced using I2C protocol whereas DHT11 was interfaced using serial protocol. After reading values from the sensor, we compute the values of Temperature, Atmospheric Pressure, Relative Humidity, Altitude, and Dew Point. 

The Graphics Display Node receives this data from the Weather Station via Ethernet & Hub, and displays it on the LCD screen. The Graphics Display node also displays data from other nodes in the IoT network like Time Service Node, and data from Thermostat node. 
