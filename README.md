Project structure:
- legacy firmware - any arduino or stm32 code you used to test your bits without the whole Maciej's data pipeline that you want in the cloud
- legacy - your current code that is not integrated into Maciej's pipeline
- pipelines - containing classes encapsulating the whole mouse prototype (get data from USB, process it somehow, perform some action)
- filters - classes for IMU processing. The class constructor needs to accept all the parameters for your filter as well as two queues - one for receiving packets, one for passing processed data to a function controlling the action (various cursor moving, plotting data)
- audio - same as above for mic
- prototypes - contain working instantiations of the mouse with a given settings (e.g. just read IMU data, ekf,mapping) that can be used for testing/demo


Signed Team members:
- Jia Yi Khoo
- Joey Surapakdi
-
-
- Maciej Kowalski
