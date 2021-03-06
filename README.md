# MPU6050
Driver for the MPU6050 inertial sensor

## Information
This code is a simple driver intended to be used in a flight computer. The code was implemented using references:
* MPU-6000 and MPU-6050
Register Map and Descriptions
Revision 4.2
* MPU-6000 and MPU-6050
Product Specification
Revision 3.4

## Example usage:
```
>>import time
>>import MPU6050

>>sensor = mpu6050(0x68)
>>sensor.calibrate()
>>while True:
>>      print(sensor.get_accel())
>>      time.sleep(0.01)
[0.004719482421874997, 0.0016469726562500005, 1.00405419921875]
[0.0025222167968749973, 0.0033559570312500005, 1.00136865234375]
[0.0021560058593749973, -0.0016489257812499995, 1.00173486328125]
[0.004109130859374997, -0.0010385742187499995, 0.99941552734375]
[0.005207763671874997, -0.0012827148437499995, 0.99807275390625]
[0.0020339355468749973, -0.0012827148437499995, 1.00210107421875]...
```
