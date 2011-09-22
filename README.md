Roomba Where Art Thou
===

Mac OS X Installation
---
- rename `roombacomm-client/lib/rxtx/librxtxSerial.jnilib_macosx` to `roombacomm-client/lib/rxtx/librxtxSerial.jnilib`
- download and install the VCP driver kit (http://www.silabs.com/products/mcu/Pages/USBtoUARTBridgeVCPDrivers.aspx)
- `sudo kextutil -v /System/Library/Extensions/SiLabsUSBDriver.kext/`
- `sudo touch /System/Library/Extensions`
- possible restart
