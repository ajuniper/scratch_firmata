# scratch_firmata
Daemon to connect MIT Scratch to a Firmata board.  Inspired by http://simplesi.net/scratch-arduino/sca/

In scratch use either variables or broadcasts to interact with hardware as documented on the above site.

Uses the Firmata library https://github.com/simlrh/firmatacpp and for Bluetooth support, https://github.com/edrosten/libblepp.

Tested only on Galileo 101.


Variables:
 * pinNN 0/1/on/off/high/low
 * allpins 0/1/on/off/high/low
 * pwmNN xx
 * servoNN xx
 * configNN xx (xx=out/in/pu)
 * motorNN xx (motorA = motor11 / motorB = motor12, xx is %)
 * powerNN xx (synonym for motor)

Broadcasts:
 * pinNNon
 * pinNNoff
 * configNNout
 * configNNin
 * configNNpu
 * adcNN (enable ADC reporting for pin NN)
 * adcNNoff
 * allon
 * alloff

Also adds support for TB6612FNG motor controller.  Use these broadcasts:
 * "defmotor motorname,pwmPin,in1Pin,in2Pin" to define the motor, e.g. "defmotor leftmotor,6,7,8"
 * "setmotor motorname,VAL" where VAL is:
	* NN (where NN is percentage of maximum forward speed)
	* -NN (where NN is a percentage of maximum reverse speed)
	* stop (synonymous with speed 0)
	* brake (invokes the controller brake function)

e.g. "setmotor leftmotor 50", "setmotor rightmotor -25", "setmotor leftmotor stop"

Errors in the daemon are reported to Scratch via the "error-message" sensor value which is sent whenever it changes.

Building:
 * Download and build firmatacpp with Bluetooth support from the above location.  The makefile assumes it will be unpacked and built in ~/firmatacpp-master/ - override this by setting firmatadir=/x/x/x on the Make invocation if required.  Note that at present the code there doesn't yet include Bluetooth support so you make need to download from my fork https://github.com/ajuniper/firmatacpp instead.
 * Run "make"
 * Or run "make NO_BLUETOOTH=1" in order to build without Bluetooth support

Running:
 * ./scratchdaemon -h (show usage info)
 * sudo ./scratchdaemon -i 500 -B (connects to first Bluetooth Firmata found)
 * sudo ./scratchdaemon -i 500 -b 11:22:33:44:55:66 (connects to specified Bluetooth device)
 * ./scratchdaemon -i 500 -s /dev/ttyUSB0 (connects to Firmata via specified serial port)

Alternatively copy the udev rules, the shell script from this folder and the executable to /etc/udev/rules.d and /usr/local/bin for auto start when firmata devices, or Bluetooth devices, are connected.

