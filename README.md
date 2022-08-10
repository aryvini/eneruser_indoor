# ENER USER

The ENERUSER project is developed in Portugal and Spain and aims to monitor the indoor air quality of houses, the outdoor air quality in their surroundings and also the energy consumption. [Read more](https://www.eneruser.com/).  


This repository is dedicated to the software running in the indoor and outdoor air quality data loggers developed at the [Polytechnic Institute of Bragança](http://portal3.ipb.pt/index.php/en/ipben/home).

The loggers are based on arduino mega microcontroller, ESP8266 and additional modules and measures the following parameters:  

| Parameter | Indoor | Outdoor|
|-----------|--------|--------|
|Air temperature|:heavy_check_mark:|:heavy_check_mark:|
|Globe temperature|:heavy_check_mark:|:x:|
|Humidity|:heavy_check_mark:|:heavy_check_mark:|
|Pressure|:heavy_check_mark:|:heavy_check_mark:|
|Particulate matter|:heavy_check_mark:|:heavy_check_mark:|
|CO|:heavy_check_mark:|:heavy_check_mark:|
|CO2|:x:|:heavy_check_mark:|
|O3|:x:|:heavy_check_mark:|

The development was made utilizing platform IO for further compatibility and to facilitate sharing.

Example of the indoor logger:  
![Indoor logger](/img/indoor.jpg)

Example of the outdoor logger:  
![Outdoor logger](/img/outdoor.jpg)