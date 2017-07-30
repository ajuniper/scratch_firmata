# scratch_firmata
Daemon to connect MIT Scratch to a Firmata board.  Inspired by http://simplesi.net/scratch-arduino/sca/

In scratch use:
 *     Variables:
 *         pinNN 0/1/on/off/high/low
 *         allpins 0/1/on/off/high/low
 *         pwmNN xx
 *         servoNN xx
 *         configNN xx (xx=out/in)
 *         motorNN xx (motorA = motor11 / motorB = motor12, xx is %)
 *         powerNN xx (synonym for motor)
 *     Broadcasts:
 *         pinNNon
 *         pinNNoff
 *         configNNout
 *         configNNin
 *         adcNN (enable ADC reporting for pin NN)
 *         adcNNoff
 *         allon
 *         alloff
