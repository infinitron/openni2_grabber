This project contains necessary function which can be used to acquire data from devices such as Kinect, ASUS Xtion Pro.
This library works with OpenNI2 and PCL 1.5( or higher).
The main function gives an example of how to use the library.

Prerequisties:
PCL 1.5 (or higher) and OpenNI2 must be installed. Kinect for Windows SDK (which includes drivers must be installed if
using windows)
In Linux to use Kinect with OpenNI2 first install libfreenect drivers. Follow this URL https://github.com/piedar/OpenNI2-FreenectDriver
to download the libfreenect and OpenNI2 bridge driver and copy it in OpenNI2 drivers directory.

Usage with Visual studio 2010:
Include the property sheet (OpenNI_PCL_R.props) for the solution in Release configuration.
To compile x64 version Windows SDK 7.1 must be installed.

Usage in Linux:
Use with cmake as documented in PCL and OpenNI sites.

openni_init.h contains the openni2_grabber class which has various functions to interact with device's data.
Please refer to the Documentation folder for refrences.