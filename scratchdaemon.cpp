/*
 * Program to connect scratch to Firmata
 * In scratch use:
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
 *
 * TODO:
 *     test allon
 *     test servo
 *
 *     http://simplesi.net/scratchgpio/scratchgpio-1st-project/
 *     broadcast supports:
 *         invert (to flip sense of io)
 *         setpins (to set selected as inputs/outputs)
 *         setpinslow (to turn all outputs off)
 *         setpinsnone (to set all as input)
 *         setpinshigh
 *         sonarNN (to put pin NN into sonar mode)
 *         ultraNN (to put pin NN into ultrasonic mode)
 *         pinpatternBBBBB... (to set pins to list of states)
 *         1coil/2coil/halfstep (set stepper motor mode)
 *         map pin 40, switch (rename pin 40 to "switch")
 *         stepper (puts motor support into stepper mode, xx becomes steps)
 *
 *     variable supports:
 *         dac xx (no firmata?)
 *         stepdelay xx (time between stepper motor steps)
 *         
 *     also from firmata:
 *         stepper motor support
 *
 *     also add direct support for TB6612FNG on specified pins
 *
 */
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <string>
#include <cstdlib>
#include <netdb.h>
#include <algorithm>
#include <cctype>
#include <map>
#include <limits.h>
#include <functional>

#include "firmata.h"
#ifndef NO_BLUETOOTH
#include "firmble.h"
#endif
#include "firmserial.h"

bool s_debug = 0;
#define DBG(__x...) \
    if (s_debug) { \
        std::ostringstream __s; \
        __s << "DBG:" << syscall(SYS_gettid) << ":" << __FILE__ ":" << __LINE__ << ":" << __FUNCTION__ << ":" << __x << std::endl; \
        std::cerr <<__s.str(); \
    }
#define ERR(__x...) \
    if (1) { \
        std::ostringstream __s; \
        size_t __t; \
        __s << "ERR:" << syscall(SYS_gettid) << ":" << __FILE__ ":" << __LINE__ << ":" << __FUNCTION__ << ":"; \
        __t = __s.str().size(); \
        __s << __x; \
        report_error(__s.str().substr(__t)); \
        __s << std::endl; \
        std::cerr <<__s.str(); \
    }


bool stopping = false;
void do_stop(int sig)
{
    std::cout << "Shutting down..." << std::endl;
    stopping = true;
}



int scratch_fd = -1;
std::string scratch_host("127.0.0.1");
struct sockaddr_in scratch_addr;
int scratch_port = 42001;
// milliseconds
int samplingInterval = 100;
int numPins = -1;
bool myPins[256];
bool reportingset[256];
struct timeval tv;
typedef std::function<int (const std::string&, const std::string&)> cmdfunc;
std::map<std::string,cmdfunc> custom_commands;

firmata::Firmata<firmata::Base, firmata::I2C>* f = nullptr;
#ifndef NO_BLUETOOTH
firmata::FirmBle* bleio = nullptr;
#endif
firmata::FirmSerial* serialio = nullptr;

// report an error back to scratch, if possible
void write_scratch_message(const std::string &msgtype, const std::string &label, const std::string &value);
void report_error(const std::string & msg)
{
    if (scratch_fd != -1)
    {
        write_scratch_message("sensor-update", "error-message", msg);
    }
}

// reset timer to send next sensor updates
void reset_timeout()
{
    tv.tv_sec = (samplingInterval / 1000);
    tv.tv_usec = (samplingInterval * 1000) % 1000000;
}

// read all current pin modes
void read_pinstates()
{
    // no actual action required as firmata lib does this for us
    numPins = f->getNumPins();
    DBG("Found "<<numPins<<" pins");

    for (int pin=0; pin<numPins; ++pin)
    {
        const std::vector<uint8_t> &caps(f->getPinCaps(pin));
        std::string comma;
        std::cout << pin << ": ";
        std::vector<uint8_t>::const_iterator i = caps.begin();
        while (i != caps.end())
        {
            std::cout << comma;
            switch (*i)
            {
                case 0: std::cout << "Input"; break;
                case 1: std::cout << "Output"; break;
                case 2: std::cout << "Analog"; break;
                case 3: std::cout << "PWM"; break;
                case 4: std::cout << "Servo"; break;
                case 5: std::cout << "Shift"; break;
                case 6: std::cout << "I2C"; break;
                case 7: std::cout << "Onewire"; break;
                case 8: std::cout << "Stepper"; break;
                case 10: std::cout << "Serial"; break;
                case 11: std::cout << "Pullup"; break;
                case 127: std::cout << "Ignore"; break;
                default: std::cout << "(" << (int)(*i)<<")"; break;
            }
            comma=",";
            ++i;
        }
        std::cout << std::endl;
    }
}

void wait_for_scratch()
{
    if (scratch_fd == -1)
    {
        scratch_fd = socket(AF_INET, SOCK_STREAM, 0);
        DBG("scratch socket is "<<scratch_fd);
    }
    while ((!stopping) &&
           (connect(scratch_fd, (sockaddr *)&scratch_addr, sizeof(scratch_addr)) < 0))
    {
        // not there, keep waiting
        DBG("waiting for scratch, errno "<<strerror(errno));
        sleep(10);
    }
    ERR("Connected to scratch");
}

bool connected_to_firmata()
{
    if (f == nullptr)
    {
        DBG("no firmata");
        return false;
    }
#ifndef NO_BLUETOOTH
    if ((bleio != nullptr) && (!bleio->isOpen()))
    {
        DBG("bluetooth not connected");
        return false;
    }
#endif
    if (!f->ready())
    {
        DBG("firmata not ready");
        return false;
    }

    return true;
}

void disconnect_firmata()
{
    if (connected_to_firmata()) {
        DBG("Disconnecting firmata");
        std::vector<unsigned char> r;
        r.push_back(FIRMATA_SYSTEM_RESET);
        f->standardCommand(r);
    }
    if (f != nullptr) {
        DBG("Deleting firmata");
        // the act of deleting the firmata object will also destroy
        // the IO object too
        delete(f);
        f = nullptr;
#ifndef NO_BLUETOOTH
        bleio = nullptr;
#endif
        serialio = nullptr;
    }
}

// connect to firmata
// p1 = conn type, 1 = serial, 2/3 = Bluetooth
// p2 = port
bool connect_firmata(int type, const std::string & port)
{
    // ensure properly disconnected first
    disconnect_firmata();

    // setup bleio/serialio
    switch (type)
    {
        case 1: // serial
            DBG("connecting to serial port "<<port);
            try
            {
                if (serialio) {
                    delete serialio;
                }
                serialio = new firmata::FirmSerial(port.c_str());
            }
            catch (...)
            {
                ERR("Failed to open serial port "<<port);
            }

            break;

#ifndef NO_BLUETOOTH
        case 3: // first bluetooth (port already set up)
        case 2: // specified bluetooth
            DBG("connecting to "<<port);
            try
            {
                if (bleio) {
                    delete bleio;
                }
                bleio = new firmata::FirmBle(port.c_str());
                if (s_debug) {
                    DBG("enabling debug");
                    bleio->enableDebug();
                }
            }
            catch (...)
            {
                ERR("Failed to connect to Bluetooth device "<<port);
            }
            break;
#endif
    }

    if (f) {
        delete f;
    }

    ERR("Opening firmata");
#ifndef NO_BLUETOOTH
    if (bleio != nullptr)
    {
        f = new firmata::Firmata<firmata::Base, firmata::I2C>(bleio);
    }
#endif
    if (serialio != nullptr)
    {
        f = new firmata::Firmata<firmata::Base, firmata::I2C>(serialio);
    }
    // firmata constructor called open()
    if (f == nullptr)
    {
        ERR("failed to instantiate firmata connection");
        return false;
    }

#ifndef NO_BLUETOOTH
    if (bleio != nullptr)
    {
        if (!bleio->isOpen())
        {
            ERR("Connecting Bluetooth");
            bleio->open();
            if (bleio->isOpen())
            {
                ERR("Bluetooth now connected");
                f->init();
            } else {
                ERR("Bluetooth connect failed");
                return false;
            }
        }
    }
#endif

    if (serialio != nullptr)
    {
        // can this happen?
        if (!serialio->isOpen())
        {
            ERR("Connecting serial");
            serialio->open();
            if (serialio->isOpen())
            {
                ERR("Serial now connected");
                f->init();
            } else {
                ERR("Serial connect failed");
                return false;
            }
        }
    }

    ERR("Firmata connected and ready");
    f->setSamplingInterval(samplingInterval);
    read_pinstates();
    sleep(1);
    reset_timeout();
    return true;
}

void disconnect_scratch()
{
    if (scratch_fd != -1)
    {
        close(scratch_fd);
        scratch_fd = -1;
    }
}

int do_poll()
{
    fd_set write_set, read_set;
    int result;
    FD_ZERO(&read_set);
    FD_ZERO(&write_set);
    int fd_max = -1;

    FD_SET(scratch_fd, &read_set);
    fd_max = scratch_fd;

    result = select(fd_max+1, &read_set, &write_set, nullptr, &tv);

    if ((result < 0) && (errno != EINTR))
    {
        ERR("select failed, "<<strerror(errno));
        disconnect_scratch();
    } else if (result == 0) {
        reset_timeout();
    }

    return result;
}

//////////////////////////////////////////////////////////////////////////
//
// Handling commands from scratch

// set pin mode in firmata if required
void pinmode(uint8_t pin, uint8_t mode)
{
    bool setreporting = false;

    DBG("pin "<<(int)pin<<" mode "<<(int)mode);
    if (f->getPinMode(pin) != mode)
    {
        const std::vector<uint8_t> &caps(f->getPinCaps(pin));
        std::vector<uint8_t>::const_iterator i = caps.begin();
        while (i != caps.end())
        {
            if (*i == mode)
            {
                break;
            }
            ++i;
        }
        if (i != caps.end())
        {
            DBG("setting pin mode");
            f->pinMode(pin, mode);
            setreporting = true;
        }
        else
        {
            ERR("pin "<<(int)pin<<" does not support mode "<<(int)mode);
            return;
        }
    }

    // the first time thru we must always set reporting correctly
    // even if the pin is already in the desired mode
    if (reportingset[pin] == false)
    {
        setreporting = true;
    }

    if (setreporting)
    {
        // set the reporting state accordingly
        if ((mode == MODE_INPUT) || (mode == MODE_PULLUP))
        {
            DBG("enable digital reporting");
            f->reportDigitalPin(pin,1);
        } else if (mode == MODE_ANALOG)
        {
            DBG("enable analog reporting");
            f->reportAnalog(f->getPinAnalogChannel(pin),1);
        } else {
            DBG("disable reporting");
            f->reportDigitalPin(pin,0);
            uint8_t apin = f->getPinAnalogChannel(pin);
            if (apin < 128) {
                DBG("disable analog reporting on "<<(int)apin);
                f->reportAnalog(pin,0);
            }
        }
        reportingset[pin] = true;
    }
}

// parse the pin number from a subset of the string
// returns maxint if not a valid pin number or does not end
// at the given position
#define BADNUMBER (UINT_MAX)
#define BADCMD (UINT_MAX - 1)
unsigned int getpin(const std::string &s, size_t ofs, size_t end = std::string::npos)
{
    size_t newend = std::string::npos;
    std::string n(s.substr(ofs,end-ofs));
    unsigned int ret;
    try
    {
        ret = std::stoul(n,&newend);
    }
    catch (...)
    {
        DBG("Failed to parse pin from "<<n);
        return BADNUMBER;
    }
    if (newend != n.size()) {
        DBG("only consumed "<<newend<<" chars but wanted to use "<<n.size()<<" "<< end<<" chars from "<<n);
        return BADCMD;
    }
    DBG("from "<<s<<" got pin "<<ret);
    return ret;
}

#define ends_in(__h,__n) (__h.substr(__h.length()-strlen(#__n)) == #__n)
// set digital output pin state
// pin1on / pin9 off
// p1=cmd p2=value
// value absent = parse from number
int process_pin(const std::string &t1, const std::string &t2)
{
    unsigned int value = UINT_MAX;
    int ret = 2; // assume consume 2 tokens
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if (ends_in(t1,off)) {
        value = 0;
        end = t1.size() - 3;
        ret = 1;
    } else if (ends_in(t1,on)) {
        value = 1;
        end = t1.size() - 2;
        ret = 1;
    }
    unsigned int pin = getpin(t1,3,end);
    if (pin == BADNUMBER) {
        ERR("Failed to parse required pin state from "<<t1);
        return 0;
    } else if (pin == BADCMD) {
        ERR("Not a valid command from "<<t1);
        return 0;
    }
    if (value == UINT_MAX) {
        // need second token
        if ((t2 == "off") || (t2 == "low") || (t2 == "0")) {
            value = 0;
        } else if ((t2 == "on") || (t2 == "high") || (t2 == "1")) {
            value = 1;
        } else if (!t2.empty()) {
            ERR("Failed to parse required pin state from "<<t2);
            return 0;
        }
    }

    DBG("pin "<<pin<<" set to "<<value);
    pinmode(pin, MODE_OUTPUT);
    myPins[pin] = false;
    f->digitalWrite(pin,value);
    return ret;
}

// enable reporting for ADC pin
// adcN / adcNoff
// p1=cmd p2=value
// value absent = parse from number
// pin provided is analog pin we must map to digital
int process_adc(const std::string &t1, const std::string &t2)
{
    unsigned int value = UINT_MAX;
    int ret = 2;
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if (ends_in(t1,off)) {
        value = 0;
        end = t1.size() - 3;
        ret = 1;
    }
    unsigned int apin = getpin(t1,3,end);
    if (apin == BADNUMBER) {
        ERR("Failed to parse required pin state from "<<t1);
        return 0;
    } else if (apin == BADCMD) {
        ERR("Not a valid command from "<<t1);
        return 0;
    }
    if (value == UINT_MAX) {
        // if no clue yet then try second arg
        if (t2 == "off") {
            value = 0;
        } else if (t2 == "on") {
            value = 1;
        } else {
            // assume command without parameters
            value = 1;
            ret = 1;
        }
    }
    unsigned int pin = f->getPinFromAnalogChannel(apin);
    DBG("pin "<<pin<<" apin "<<apin<<" value "<<value);
    pinmode(pin, MODE_ANALOG);
    myPins[pin] = (value==1)?true:false;
    f->reportAnalog(apin,value);
    return ret;
}

// set ddr
// config1in / config2 out
// p1=cmd p2=value
// value absent = parse from number
int process_config(const std::string &t1, const std::string &t2)
{
    unsigned int value = UINT_MAX;
    int ret = 2;
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if (ends_in(t1,out)) {
        value = MODE_OUTPUT;
        end = t1.size() - 3;
        ret = 1;
    } else if (ends_in(t1,pu)) {
        value = MODE_PULLUP;
        end = t1.size() - 2;
        ret = 1;
    } else if (ends_in(t1,in)) {
        value = MODE_INPUT;
        end = t1.size() - 2;
        ret = 1;
    }
    unsigned int pin = getpin(t1,6,end);
    if (pin == BADNUMBER) {
        ERR("Failed to parse required pin state from "<<t1);
        return 0;
    } else if (pin == BADCMD) {
        ERR("Not a valid command from "<<t1);
        return 0;
    }
    if (value == UINT_MAX) {
        if (t2 == "out") {
            value = MODE_OUTPUT;
        } else if (t2 == "in") {
            value = MODE_INPUT;
        } else if (t2 == "pu") {
            value = MODE_PULLUP;
        } else if (!t2.empty()) {
            ERR("Failed to parse required pin state from "<<t2);
            return 0;
        } else {
            ERR("Failed to parse required pin state from "<<t1);
            return 0;
        }
    }
    DBG("pin "<<pin<<" value "<<value);
    pinmode(pin, value);
    if ((value == MODE_INPUT) || (value == MODE_PULLUP)) {
        myPins[pin] = true;
    }
    return ret;
}

// set pwm value
// pwmNN val
// p1=number p2=value%
int process_pwm(const std::string &t1, const std::string &t2)
{
    DBG("Parsing from "<<t1<<" "<<t2);
    unsigned int pin = getpin(t1,3);
    if (pin == BADNUMBER) {
        ERR("Failed to parse required pin state from "<<t1);
        return 0;
    } else if (pin == BADCMD) {
        ERR("Not a valid command from "<<t1);
        return 0;
    }
    unsigned int value = std::stoul(t2);
    DBG("pin "<<pin<<" value "<<value);
    pinmode(pin, MODE_PWM);
    f->analogWrite(pin,value);
    return 2;
}

// set servo value
// servoNN val
// p1=number p2=value%
int process_servo(const std::string &t1, const std::string &t2)
{
    DBG("Parsing from "<<t1<<" "<<t2);
    unsigned int pin = getpin(t1,5);
    if (pin == BADNUMBER) {
        ERR("Failed to parse required pin state from "<<t1);
        return 0;
    } else if (pin == BADCMD) {
        ERR("Not a valid command from "<<t1);
        return 0;
    }
    unsigned int value = std::stoul(t2);
    DBG("pin "<<pin);
    pinmode(pin, MODE_SERVO);
    f->analogWrite(pin,value);
    return 2;
}

int process_pin_percent(int pin, const std::string &t2, uint8_t mode)
{
    int value = std::stoul(t2);
    DBG("pin "<<pin<<" raw value "<<value);
    uint32_t resolution = f->getPinCapResolution(pin,mode);
    if (resolution > 0)
    {
        uint32_t max = (1<<resolution);
        uint32_t scaled = (max * abs(value)) / 100;
        if (scaled >= max)
        {
            scaled = max-1;
        }
        DBG("resolution "<<resolution<<" scaled "<<scaled);
        pinmode(pin, mode);
        f->analogWrite(pin,scaled);
        return value;
    }
    else
    {
        ERR("No pin capability for mode "<<(int)mode);
    }
    return 0;
}

void process_pin_percent(const std::string &t1, const std::string &t2, size_t baseLen, uint8_t mode)
{
    DBG("Parsing from "<<t1<<" "<<t2);
    unsigned int pin;
    if (t1[baseLen] == 'a')
    {
        pin = 11;
    }
    else if (t1[baseLen] == 'b')
    {
        pin = 12;
    }
    else {
        pin = getpin(t1,baseLen);
        if (pin == BADNUMBER) {
            ERR("Failed to parse required pin state from "<<t1);
            return;
        } else if (pin == BADCMD) {
            ERR("Not a valid command from "<<t1);
            return;
        }
    }
    process_pin_percent(pin, t2, mode);
}

// motor speed - alias for pwm
// motorA = motor11
// motorB = motor12
// motorNN val
// p1=number p2=value
int process_motor(const std::string &t1, const std::string &t2)
{
    process_pin_percent(t1, t2, 5, MODE_PWM);
    return 2;
}

// power - alias for pwm
// powerA = power11
// powerB = power12
// powerNN val
// p1=number p2=value
int process_power(const std::string &t1, const std::string &t2)
{
    process_pin_percent(t1, t2, 5, MODE_PWM);
    return 2;
}

// set all pins
// allpins value
int process_allpins(const std::string &t1, const std::string &t2)
{
    unsigned int value;
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if ((t2 == "off") || (t2 == "low") || (t2 == "0")) {
        value = 0;
    } else if ((t2 == "on") || (t2 == "high") || (t2 == "1")) {
        value = 1;
    } else if (!t2.empty()) {
        ERR("Failed to parse required pin state from "<<t2);
        return 0;
    }
    // find all digital IO pins and iterate over them
    for (uint8_t pin = 0; pin<numPins; ++pin)
    {
        // only update pins which are digital outputs
        if (f->getPinMode(pin) == MODE_OUTPUT)
        {
            DBG("pin "<<pin<<" value "<<value);
            f->digitalWrite(pin, value);
        }
    }
    return 2;
}

// set all pins on
// allpins value
int process_allon(const std::string &t1, const std::string &t2)
{
    process_allpins("allpins","on");
    return 1;
}

// set all pins off
// allpins value
int process_alloff(const std::string &t1, const std::string &t2)
{
    process_allpins("allpins","off");
    return 1;
}

typedef struct
{
    uint8_t in1;
    uint8_t in2;
    uint8_t pwm;
} tb6612fng;
std::map<std::string,tb6612fng> tb6612fng_list;

// motorname = custom name for motor
// motorname % or motorname -% or motorname stop or motorname brake
int process_setmotor(const std::string &t1, const std::string &t2)
{
    std::map<std::string,tb6612fng>::iterator i = tb6612fng_list.find(t1);
    if (i == tb6612fng_list.end())
    {
        ERR("Failed to find motor entry "<<t1);
        return 0;
    }
    DBG("Setting motor "<<t1<<" to "<<t2);

    if (t2 == "stop")
    {
        f->digitalWrite(i->second.in1,0);
        f->digitalWrite(i->second.in2,0);
        f->analogWrite(i->second.pwm,0);
    }
    else if (t2 == "brake")
    {
        f->digitalWrite(i->second.in1,1);
        f->digitalWrite(i->second.in2,1);
        f->analogWrite(i->second.pwm,0);
    }
    else
    {
        int speed = process_pin_percent(i->second.pwm, t2, MODE_PWM);
        if (speed == 0)
        {
            f->digitalWrite(i->second.in1,0);
            f->digitalWrite(i->second.in2,0);
        }
        else if (speed > 0)
        {
            f->digitalWrite(i->second.in1,1);
            f->digitalWrite(i->second.in2,0);
        }
        else
        {
            f->digitalWrite(i->second.in1,0);
            f->digitalWrite(i->second.in2,1);
        }
    }
    return 2;
}

// define a motor controlled by a TB6612FNG
// defmotor "motorname,pwmPin,in1Pin,in2Pin"
int process_defmotor(const std::string &t1, const std::string &t2)
{
    DBG("t1 "<<t1<<" t2 "<<t2);
    int j = 0;
    std::string token;
    std::string name;
    int pwm, pin1, pin2;
    size_t start = 0, end = 0;
    while (end != std::string::npos) {
        end = t2.find(',', start);
        token.assign(t2.substr( start, (end == std::string::npos) ? std::string::npos : end - start));
        start = ( end > (std::string::npos - 1)) ?  std::string::npos  :  (end + 1);
        DBG("Token: " << token);
        switch (j)
        {
            case 0: // motorname
                name.assign(token);
                DBG("Motor "<<name);
                break;
            case 1: // pwm pin
                pwm = stoul(token);
                DBG("pwm "<<pwm);
                break;
            case 2: // pin1
                pin1 = stoul(token);
                DBG("pin1 "<<pin1);
                break;
            case 3: // pin2
                pin2 = stoul(token);
                DBG("pin2 "<<pin2);
                break;
        }
        ++j;
    }
    if (j != 4)
    {
        ERR("Failed to parse motor definition from "<<t2);
        return 0;
    }
    tb6612fng_list[name].in1 = pin1;
    tb6612fng_list[name].in2 = pin2;
    tb6612fng_list[name].pwm = pwm;
    DBG("Motor "<<pin1<<" "<<pin2<<" "<<pwm);
    custom_commands[name] = process_setmotor;
    pinmode(pin1, MODE_OUTPUT);
    pinmode(pin2, MODE_OUTPUT);
    pinmode(pwm, MODE_PWM);
    return 2;
}

// check for custom commands
int process_custom(const std::string &t1, const std::string &t2 = "")
{
    std::map<std::string,cmdfunc>::const_iterator i = custom_commands.find(t1);
    if (i != custom_commands.end())
    {
        return i->second(t1,t2);
        return true;
    }
    return 0;
}

// process a single request from scratch
#define process_thing(__x,__y,__z) if (__x.find(#__y) == 0) return process_##__y(__x,__z)
int process_scratch(const std::string &t1, const std::string &t2 = "")
{
    DBG("t1 "<<t1<<" t2 "<<t2);
    int ret = process_custom(t1,t2);
    if (ret > 0)
    {
        DBG("matched custom command "<<t1);
        return ret;
    }
    process_thing(t1,pin,t2);
    process_thing(t1,adc,t2);
    process_thing(t1,config,t2);
    process_thing(t1,pwm,t2);
    process_thing(t1,servo,t2);
    process_thing(t1,allpins,t2);
    process_thing(t1,allon,t2);
    process_thing(t1,alloff,t2);
    process_thing(t1,motor,t2);
    process_thing(t1,power,t2);
    process_thing(t1,defmotor,t2);
    return 0;
}

void read_scratch_message()
{
    // msg format is
    // XXXX:msgtype "label" [value]
    // msgtype generally == broadcast
    // label can be a quoted list of pins to process (e.g. pin1on pin2off)
    // or can be a single name with value as a separate arg
    int i;
    unsigned char c[4];

    if (read(scratch_fd,&c,4) != 4)
    {
        // something bad happened
        ERR("failed to read from scratch, "<<strerror(errno));
        disconnect_scratch();
        return;
    }

    unsigned int msglen = (c[0]*16777216) + (c[1]*65536) + (c[2]*256) + c[3];
    DBG("msglen is "<<msglen);

    unsigned char * msgbuf = (unsigned char *)malloc(msglen+1);
    if (read(scratch_fd, msgbuf, msglen) != msglen)
    {
        ERR("failed to read from scratch, "<<strerror(errno));
        free(msgbuf);
        disconnect_scratch();
        return;
    }
    msgbuf[msglen]=0;

    DBG("msg is '"<<msgbuf<<"'");
    std::string token;
    i = 0;
    int j = 0;
    bool in_quotes = false;
    bool multivalue = false;
    std::vector <std::string> tokens;
    // broadcast message requires splitting on space within spaces
    bool broadcast = false ; // assume sensor-update for now
    
    while (i < msglen)
    {
        //DBG("char "<<i<<" = \""<<msgbuf[i]<<"\"");
        if (msgbuf[i] == ' ')
        {
            if (in_quotes)
            {
                if (broadcast)
                {
                    // space within quotes in a broadcast message has to be
                    // split to a new token
                    std::transform(token.begin(), token.end(), token.begin(), ::tolower);
                    tokens.push_back(token);
                    token.clear();
                    ++j;
                } else {
                    // otherwise we preserve the whitespace within the token
                    token.append(1, msgbuf[i]);
                }
            } else {
                // outside quotes, space means move to next token
                DBG("token "<<j<<" is \""<<token<<"\"");
                std::transform(token.begin(), token.end(), token.begin(), ::tolower);
                if (j == 0) {
                    // message type, broadcast or sensor-update
                    if (token == "broadcast") {
                        broadcast = true;
                        DBG("Is broadcast");
                    } else if (token != "sensor-update") {
                        ERR("Failed to find message type in "<<token);
                        break;
                    }
                }
                tokens.push_back(token);
                token.clear();
                ++j;
            }
        } else if (msgbuf[i] == '"') {
            // ditch the quotes but record the status
            if (in_quotes)
            {
                // already in quotes
                in_quotes = false;
            } else {
                in_quotes = true;
            }
        } else {
            // normal character, add to buffer
            token.append(1, msgbuf[i]);
        }
        ++i;
    }

    // preserve any final token
    if (!token.empty()) {
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);
        tokens.push_back(token);
        token.clear();
        ++j;
    }

    // dispatch the tokens
    // add a dummy extra token to the end of the tokens so that we can always find
    // an extra empty token to reference when we reach the end
    tokens.push_back("");
    i=1;
    int k = 0;
    if (bleio) { bleio->write_batch(true); }
    while (i < j) {
        DBG("Processing token "<<tokens[i]);
        k = process_scratch(tokens[i],tokens[i+1]);
        if (k == 0) {
            ERR("Failed to parse token "<<i<<" "<<tokens[i]);
            // failed to parse the command
            // for a broadcast, skip one token, for sensor-update skip 2
            if (broadcast) { k = 1; } else { k = 2; }
        }
        DBG("Consuming "<<k<<" tokens");
        i+=k;
    }
    if (bleio) { bleio->write_batch(false); }

    free(msgbuf);
}

//////////////////////////////////////////////////////////////////////
//
// Sending data to scratch

// msg format is
// XXXX:msgtype "label" [value]
void write_to_scratch(const std::ostringstream & msg)
{
    DBG("writing: "<<msg.str());
    unsigned int len = msg.str().size();
    std::string msgbuf;
    msgbuf.append(1, (char)(len>>24));
    msgbuf.append(1, (char)((len >> 16) & 0xff));
    msgbuf.append(1, (char)((len >> 8) & 0xff));
    msgbuf.append(1, (char)(len & 0xff));
    msgbuf.append(msg.str());

    if (write(scratch_fd, msgbuf.c_str(), msgbuf.size()) != msgbuf.size())
    {
        ERR("Failed to write message to scratch");
        disconnect_scratch();
    }
}

void write_scratch_message(const std::string &msgtype, const std::string &label, const std::string &value)
{
    std::ostringstream msg;
    msg << msgtype;
    msg << " \"";
    msg << label;
    msg << "\"";
    if (msgtype == "sensor-update") {
        msg << " \"";
        msg << value;
        msg << "\"";
    }
    write_to_scratch(msg);
}

void write_scratch_message(const std::string &msgtype, const std::string &label, int pin, uint32_t value)
{
    std::ostringstream msg;
    msg << msgtype;
    msg << " ";
    if (label.find(' ')) { msg << "\""; }
    msg << label;
    msg << pin;
    if (label.find(' ')) { msg << "\""; }
    if (msgtype == "sensor-update") {
        msg << " ";
        msg << value;
    }
    write_to_scratch(msg);
}

// send all interesting pin states to scratch
void write_scratch()
{
    for (int i = 0; i<numPins; ++i)
    {
        //DBG("Checking pin "<<i);
        if (myPins[i] == true)
        {
            switch (f->getPinMode(i))
            {
                case MODE_ANALOG:
                    write_scratch_message("sensor-update", "adc", f->getPinAnalogChannel(i), f->analogRead(i));
                    break;
                case MODE_INPUT:
                case MODE_PULLUP:
                    write_scratch_message("sensor-update", "input", i, f->digitalRead(i));
                    break;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////
//
// main loop and arg handling

void usage(const char * progname, const char * msg = nullptr, int ec = 1)
{
    if (msg != nullptr) std::cout << msg << std::endl;
    std::cout << "Usage: "<<progname<<" [-s serialDev] ";
#ifndef NO_BLUETOOTH
    std::cout << "[-b bdaddr] [-B] ";
#endif
    std::cout << "[-i reportingInterval] [-H scratchHost] [-P scratchPort] [-d] [-h]" << std::endl;
    std::cout << "    -s /dev/ttyS? (use given serial port)" << std::endl;
#ifndef NO_BLUETOOTH
    std::cout << "    -b bdaddr (use given bluetooth device)" << std::endl;
    std::cout << "    -B (use first available bluetooth device)" << std::endl;
#endif
    std::cout << "    -i N (use given reporting interval in ms, default 100ms)" << std::endl;
    std::cout << "    -H H (talk to scratch at given host, default localhost)" << std::endl;
    std::cout << "    -P P (talk to scratch on given port, default 42001)" << std::endl;
    std::cout << "    -d (enable debug messages)" << std::endl;
    std::cout << "    -h show this help" << std::endl;
    std::cout << std::endl;
    exit(ec);
}

int main(int argc, char * argv[])
{
    // parse arguments
    int c;
    std::string port;
    int conntype = 0;
    memset(&myPins[0], 0, sizeof(myPins));

    while ((c = getopt(argc, argv, "s:b:Bi:H:P:dh")) >= 0)
    {
        switch (c)
        {
            case 's': // firmata serial port
                port = optarg;
                conntype = 1;
                break;
#ifndef NO_BLUETOOTH
            case 'b': // firmata specified bluetooth device
                port = optarg;
                conntype = 2;
                break;
            case 'B': // firmata first bluetooth device
                conntype = 3;
                break;
#endif
            case 'i': // reporting interval
                samplingInterval = atoi(optarg);
                break;
            case 'H': // scratch host
                scratch_host = optarg;
                break;
            case 'P': // scratch port
                scratch_port = atoi(optarg);
                break;
            case 'd': // enable debug
                s_debug = 1;
                break;
            case 'h':
                usage(argv[0], nullptr, 0);
                break;
            default:
                usage(argv[0],"Unrecognised option");
                break;
        }
    }

    // setup bleio/serialio
    switch (conntype)
    {
        case 0:
            usage(argv[0],"Connection type must be specified");
            break;

        case 1: // serial
            DBG("Using serial port "<<port);
            break;

#ifndef NO_BLUETOOTH
        case 3: // first bluetooth
            try
            {
                DBG("Looking for bluetooth devices");
                std::vector<firmata::BlePortInfo> ports = firmata::FirmBle::listPorts(3);
                if (ports.size() == 0)
                {
                    usage(argv[0],"Failed to find any Bluetooth devices");
                }
                port = ports[0].port;
            }
            catch (...)
            {
                usage(argv[0],"Failed to search for Bluetooth devices, is the adapter present?");
            }

            // fall thru

        case 2: // specified bluetooth
            DBG("Using Bluetooth device "<<port);
#endif

    }
    // set up the scratch address
    struct hostent *hostinfo;

    scratch_addr.sin_family = AF_INET;
    scratch_addr.sin_port = htons (scratch_port);
    hostinfo = gethostbyname (scratch_host.c_str());
    if (hostinfo == NULL)
    {
        std::string msg("Unable to resolve scratch host "+scratch_host);
        usage(argv[0],msg.c_str());
    }
    scratch_addr.sin_addr = *(struct in_addr *) hostinfo->h_addr;

    sleep(3);

    signal(SIGINT, do_stop);
    signal(SIGTERM, do_stop);

    while (!stopping)
    {
        // presence of scratch gates everything
        wait_for_scratch();

        while ((!stopping) && (scratch_fd >= 0))
        {
            if (!connected_to_firmata())
            {
                DBG("Connecting to firmata");
                try
                {
                    connect_firmata(conntype, port);
                }
                catch (...)
                {
                    // if the connect fails then try again
                    DBG("connect failed");
                    continue;
                }
            }

            int n = do_poll();

            // firmata will throw if not connected
            try
            {
                f->parse();
                if (n > 0)
                {
                    // scratch message arrived
                    read_scratch_message();
                } else if (n == 0)
                {
                    // timeout, send updates
                    DBG("time to send samples");
                    write_scratch();
                } else {
                    DBG("poll error "<<strerror(errno));
                }
            }
            catch (...)
            {
                // caught exception, close firmata
                ERR("Firmata connection closed");
                disconnect_firmata();
                // wait a while and try again
                sleep(1);
            }
        }

        DBG("Exited inner loop");

        if (scratch_fd >= 0) {
            ERR("Firmata bridge is shut down");
        }

        // scratch went away or we are stopping
        disconnect_firmata();
        disconnect_scratch();
    }
    // all done
}

