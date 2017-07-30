# scratch_firmata
Daemon to connect MIT Scratch to a Firmata board.  Inspired by http://simplesi.net/scratch-arduino/sca/

In scratch use either variables or broadcasts to interact with hardware as documented on the above site.

Variables:
 * pinNN 0/1/on/off/high/low
 * allpins 0/1/on/off/high/low
 * pwmNN xx
 * servoNN xx
 * configNN xx (xx=out/in)
 * motorNN xx (motorA = motor11 / motorB = motor12, xx is %)
 * powerNN xx (synonym for motor)

Broadcasts:
 * pinNNon
 * pinNNoff
 * configNNout
 * configNNin
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

Uses the Firmata library https://github.com/simlrh/firmatacpp and for Bluetooth support, https://github.com/edrosten/libblepp.

Tested only on Galileo 101.
