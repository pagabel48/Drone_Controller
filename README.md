# Drone Controller

![](docs/render.png)

This is the control board. It is based around an STM32f401 MCU and uses a buck converter to generate steady 3.3v from a LiPO Battery or USB. When powered over battery it can be toggled on and off by a highside power latch.

A BMP280 barometer and an ICM42670P IMU are connected over I2C for gathering position, and PWM is used to communicate with the motor ESCs and controller. Programming is done over USB, and pins are exposed for UART, I2C, and SWD protocols for extensibility.

Four layers were used to make routing easier and improve signal integrity.

## Routing

![](docs/PCB_Schematic.png)

![](docs/PCB_Backside.png)

## Schematic

![](docs/PCB_Layout.png)

# Code

![](docs/kf_code.png)

State estimation is achieved through two Kalman filters (attitude and altitude), and PID controllers are used in combination with a mixer to get final motor throttle values.

Custom drivers were written to interface with the sensors, and a matrix math library was written using c++ operator overloading to allow for readable state space control code. The IMU driver reads when triggered by a pin interupt, and the barometer driver uses polling.

A testing routine was written which can be run instead of the main drone loop - it runs unit tests for the code, then tests operation of the sensors, motors, Kalman filters, and status light.

The firmware automatically turns the drone off when the battery voltage drops below a safe level.

Controller Code is Contained in Firmare/Core/

## Code Structure

![](docs/diagram.png)

# Frame

![](docs/frame.png)

The frame was designed in fusion 360 and was 3d printed using PETG plastic. Care was taken to maximize rigidity while minimizing weight.