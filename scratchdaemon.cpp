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
 *     deal with sensor-update n1 v1 n2 v2 n3 v3 ...
 *     add support for broadcast "defmotor name p p p"
 *     test allon
 *     test servo
 *     why abort when g101 reset?
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

#include "firmata.h"
#include "firmble.h"
#include "firmserial.h"

bool s_debug = 1;
#define DBG(__x...) \
    if (s_debug) { \
        std::ostringstream __s; \
        __s << "DBG:" << syscall(SYS_gettid) << ":" << __FILE__ ":" << __LINE__ << ":" << __FUNCTION__ << ":" << __x << std::endl; \
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
typedef std::function<void (const std::string&, const std::string&)> cmdfunc;
std::map<std::string,cmdfunc> custom_commands;


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
}

firmata::Firmata<firmata::Base, firmata::I2C>* f = nullptr;
firmata::FirmBle* bleio = nullptr;
firmata::FirmSerial* serialio = nullptr;

bool connected_to_firmata()
{
    if (f == nullptr)
    {
        DBG("no firmata");
        return false;
    }
    if ((bleio != nullptr) && (!bleio->isOpen()))
    {
        DBG("bluetooth not connected");
        return false;
    }
    if (!f->ready())
    {
        DBG("firmata not ready");
        return false;
    }

    return true;
}
bool connect_firmata()
{
    if (f == nullptr)
    {
        DBG("Opening firmata");
        if (bleio != nullptr)
        {
            f = new firmata::Firmata<firmata::Base, firmata::I2C>(bleio);
        }
        if (serialio != nullptr)
        {
            f = new firmata::Firmata<firmata::Base, firmata::I2C>(serialio);
        }
        // firmata constructor called open()
    }
    if (f == nullptr)
    {
        DBG("failed to instantiate firmata connection");
        return false;
    }

    if (bleio != nullptr)
    {
        if (!bleio->isOpen())
        {
            DBG("firmata not ready, reconnect bluetooth");
            bleio->open();
            if (bleio->isOpen())
            {
                DBG("bluetooth now connected");
                f->init();
            } else {
                DBG("bluetooth connect failed");
                return false;
            }
        }
    }
    if (serialio != nullptr)
    {
        // can this happen?
        if (serialio->isOpen())
        {
            DBG("firmata not ready, reconnect serial");
            serialio->open();
            if (serialio->isOpen())
            {
                DBG("serial now connected");
                f->init();
            } else {
                DBG("serial connect failed");
                return false;
            }
        }
    }

    DBG("firmata connected and ready");
    f->setSamplingInterval(samplingInterval);
    return true;
}

void disconnect_firmata()
{
    // TODO
    std::vector<unsigned char> r;
    r.push_back(FIRMATA_SYSTEM_RESET);
    f->standardCommand(r);
    delete(f);
}

void disconnect_scratch()
{
    if (scratch_fd != -1)
    {
        close(scratch_fd);
        scratch_fd = -1;
    }
}

void reset_timeout()
{
    tv.tv_sec = (samplingInterval / 1000);
    tv.tv_usec = (samplingInterval * 1000) % 1000000;
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

    if (result < 0)
    {
        DBG("select failed, "<<strerror(errno));
        disconnect_scratch();
    } else if (result == 0) {
        reset_timeout();
    }

    return result;
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
            DBG("pin "<<(int)pin<<" does not support mode "<<(int)mode);
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
        if (mode == MODE_INPUT)
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
unsigned int getpin(const std::string &s, size_t ofs, size_t end = std::string::npos)
{
    unsigned int ret = std::stoul(s.substr(ofs, end));
    DBG("from "<<s<<" got pin "<<ret);
    return ret;
}

#define ends_in(__h,__n) (__h.substr(__h.length()-strlen(#__n)) == #__n)
// set digital output pin state
// pin1on / pin9 off
// p1=cmd p2=value
// value absent = parse from number
void process_pin(const std::string &t1, const std::string &t2)
{
    unsigned int value;
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if ((t2 == "off") || (t2 == "low") || (t2 == "0")) {
        value = 0;
    } else if ((t2 == "on") || (t2 == "high") || (t2 == "1")) {
        value = 1;
    } else if (!t2.empty()) {
        DBG("Failed to parse required pin state");
        return;
    } else if (ends_in(t1,off)) {
        value = 0;
        end = t1.size() - 3;
    } else if (ends_in(t1,on)) {
        value = 1;
        end = t1.size() - 2;
    } else {
        DBG("Failed to parse required pin state");
        return;
    }
    unsigned int pin = getpin(t1,3,end);
    DBG("pin "<<pin<<" set to "<<value);
    pinmode(pin, MODE_OUTPUT);
    myPins[pin] = false;
    f->digitalWrite(pin,value);
}

// enable reporting for ADC pin
// adcN / adcNoff
// p1=cmd p2=value
// value absent = parse from number
// pin provided is analog pin we must map to digital
void process_adc(const std::string &t1, const std::string &t2)
{
    // default to enable
    unsigned int value = 1;
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if (t2 == "off") {
        value = 0;
    } else if (t2 == "on") {
        value = 1;
    } else if (!t2.empty()) {
        DBG("Failed to parse required pin state");
        return;
    } else if (ends_in(t1,off)) {
        value = 0;
        end = t1.size() - 3;
    }
    unsigned int apin = getpin(t1,3,end);
    unsigned int pin = f->getPinFromAnalogChannel(apin);
    DBG("pin "<<pin<<" apin "<<apin<<" value "<<value);
    pinmode(pin, MODE_ANALOG);
    myPins[pin] = true;
    f->reportAnalog(apin,value);
}

// set ddr
// config1in / config2 out
// p1=cmd p2=value
// value absent = parse from number
void process_config(const std::string &t1, const std::string &t2)
{
    unsigned int value;
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if (t2 == "out") {
        value = MODE_OUTPUT;
    } else if (t2 == "in") {
        value = MODE_INPUT;
    } else if (!t2.empty()) {
        DBG("Failed to parse required pin state");
        return;
    } else if (ends_in(t1,out)) {
        value = MODE_OUTPUT;
        end = t1.size() - 3;
    } else if (ends_in(t1,in)) {
        value = MODE_INPUT;
        end = t1.size() - 2;
    } else {
        DBG("Failed to parse required pin state");
        return;
    }
    unsigned int pin = getpin(t1,6,end);
    DBG("pin "<<pin<<" value "<<value);
    pinmode(pin, value);
    if (value == MODE_INPUT) {
        myPins[pin] = true;
    }
}

// set pwm value
// pwmNN val
// p1=number p2=value%
void process_pwm(const std::string &t1, const std::string &t2)
{
    DBG("Parsing from "<<t1<<" "<<t2);
    unsigned int pin = getpin(t1,3);
    unsigned int value = std::stoul(t2);
    DBG("pin "<<pin<<" value "<<value);
    pinmode(pin, MODE_PWM);
    f->analogWrite(pin,value);
}

// set servo value
// servoNN val
// p1=number p2=value%
void process_servo(const std::string &t1, const std::string &t2)
{
    DBG("Parsing from "<<t1<<" "<<t2);
    unsigned int pin = getpin(t1,5);
    unsigned int value = std::stoul(t2);
    DBG("pin "<<pin);
    pinmode(pin, MODE_SERVO);
    f->analogWrite(pin,value);
}

int process_pin_percent(int pin, const std::string &t2, uint8_t mode)
{
    int value = std::stoul(t2);
    DBG("pin "<<pin<<" raw value "<<value);
    if (value < 0) value = -value;
    uint32_t resolution = f->getPinCapResolution(pin,mode);
    if (resolution > 0)
    {
        uint32_t max = (1<<resolution);
        uint32_t scaled = (max * value) / 100;
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
        DBG("No pin capability for mode "<<mode);
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
    }
    process_pin_percent(pin, t2, mode);
}

// motor speed - alias for pwm
// motorA = motor11
// motorB = motor12
// motorNN val
// p1=number p2=value
void process_motor(const std::string &t1, const std::string &t2)
{
    process_pin_percent(t1, t2, 5, MODE_PWM);
}

// power - alias for pwm
// powerA = power11
// powerB = power12
// powerNN val
// p1=number p2=value
void process_power(const std::string &t1, const std::string &t2)
{
    process_pin_percent(t1, t2, 5, MODE_PWM);
}

// set all pins
// allpins value
void process_allpins(const std::string &t1, const std::string &t2)
{
    unsigned int value;
    size_t end = std::string::npos;
    DBG("Parsing from "<<t1<<" "<<t2);
    if ((t2 == "off") || (t2 == "low") || (t2 == "0")) {
        value = 0;
    } else if ((t2 == "on") || (t2 == "high") || (t2 == "1")) {
        value = 1;
    } else if (!t2.empty()) {
        DBG("Failed to parse required pin state");
        return;
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
}

// set all pins on
// allpins value
void process_allon(const std::string &t1, const std::string &t2)
{
    process_allpins("allpins","on");
}

// set all pins off
// allpins value
void process_alloff(const std::string &t1, const std::string &t2)
{
    process_allpins("allpins","off");
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
void process_setmotor(const std::string &t1, const std::string &t2)
{
    std::map<std::string,tb6612fng>::iterator i = tb6612fng_list.find(t1);
    if (i == tb6612fng_list.end())
    {
        DBG("Failed to find motor entry "<<t1);
        return;
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
}

// define a motor controlled by a TB6612FNG
// defmotor "motorname,pwmPin,in1Pin,in2Pin"
void process_defmotor(const std::string &t1, const std::string &t2)
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
        DBG("Failed to parse motor definition");
        return;
    }
    tb6612fng_list[name].in1 = pin1;
    tb6612fng_list[name].in2 = pin2;
    tb6612fng_list[name].pwm = pwm;
    DBG("Motor "<<pin1<<" "<<pin2<<" "<<pwm);
    custom_commands[name] = process_setmotor;
    pinmode(pin1, MODE_OUTPUT);
    pinmode(pin2, MODE_OUTPUT);
    pinmode(pwm, MODE_PWM);
}

// check for custom commands
bool process_custom(const std::string &t1, const std::string &t2 = "")
{
    std::map<std::string,cmdfunc>::const_iterator i = custom_commands.find(t1);
    if (i != custom_commands.end())
    {
        i->second(t1,t2);
        return true;
    }
    return false;
}

// process a single request from scratch
#define process_thing(__x,__y,__z) if (__x.find(#__y) == 0) process_##__y(__x,__z)
void process_scratch(const std::string &t1, const std::string &t2 = "")
{
    DBG("t1 "<<t1<<" t2 "<<t2);
    if (process_custom(t1,t2))
    {
        DBG("matched custom command "<<t1);
        return;
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
}

// dispatch the given token if required
// returns false if no further parsing required
bool dispatch_token(int toknum, bool multivalue, std::string & token, std::string &prevtoken)
{
    DBG("toknum "<<toknum<<" multi "<<multivalue<<" token "<<token<<" prev "<<prevtoken);
    std::transform(token.begin(), token.end(), token.begin(), ::tolower);
    switch (toknum)
    {
        case 0:
            // message type
            // discard the whole thing if not "broadcast"
            if ((token != "broadcast") && (token != "sensor-update"))
            {
                return false;
            }
            break;
        case 1:
            // completed token 1
            if (multivalue)
            {
                process_scratch(token);
            }
            else
            {
                // not a multivalue token for token1, so cache it
                prevtoken = token;
            }
            break;
        case 2:
            // completed token 2
            process_scratch(prevtoken, token);
    }
    token.clear();
    return true;
}

// msg format is
// XXXX:msgtype "label" [value]
void write_scratch_message(const std::string &msgtype, const std::string &label, int pin, uint32_t value)
{
    std::ostringstream msg;
    msg << msgtype;
    msg << " ";
    msg << label;
    msg << pin;
    if (msgtype == "sensor-update") {
        msg << " ";
        msg << value;
    }

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
        DBG("Failed to write message to scratch");
        disconnect_scratch();
    }
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
                    write_scratch_message("sensor-update", "input", i, f->digitalRead(i));
                    break;
            }
        }
    }
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
        DBG("failed to read from scratch, "<<strerror(errno));
        disconnect_scratch();
        return;
    }

    unsigned int msglen = (c[0]*16777216) + (c[1]*65536) + (c[2]*256) + c[3];
    DBG("msglen is "<<msglen);

    unsigned char * msgbuf = (unsigned char *)malloc(msglen+1);
    if (read(scratch_fd, msgbuf, msglen) != msglen)
    {
        DBG("failed to read from scratch, "<<strerror(errno));
        free(msgbuf);
        disconnect_scratch();
        return;
    }
    msgbuf[msglen]=0;

    DBG("msg is '"<<msgbuf<<"'");
    std::string token1, token;
    i = 0;
    int j = 0;
    bool in_quotes = false;
    bool multivalue = false;
    
    while (i < msglen)
    {
        //DBG("char "<<i<<" = \""<<msgbuf[i]<<"\"");
        if (msgbuf[i] == ' ')
        {
            if (in_quotes)
            {
                switch (j)
                {
                    case 0:
                        // message preamble, stick in token and continue
                        token.append(1, msgbuf[i]);
                        break;
                    case 1:
                        // space inside quotes means we have a multi value list
                        // for token1, so we process the token now
                        process_scratch(token);
                        token.clear();
                        multivalue = true;
                        break;
                    case 2:
                        // inside quotes as pert of second token means add to
                        // value list
                        token.append(1, msgbuf[i]);
                }
            } else {
                // outside quotes, space means move to next field
                DBG("token "<<j<<" is \""<<token<<"\"");
                if (!dispatch_token(j, multivalue, token, token1))
                {
                    DBG("end of token parsing, remaining '"<<(msgbuf+i)<<"'");
                    break;
                }
                ++j;
            }
        } else if (msgbuf[i] == '"')
        {
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

    // dispatch final token
    if (!token.empty())
    {
        dispatch_token(j, (j==1)?true:false, token, token1);
    }

    free(msgbuf);
}

void usage(const char * progname, const char * msg = nullptr, int ec = 1)
{
    if (msg != nullptr) std::cout << msg << std::endl;
    std::cout << "Usage: "<<progname<<" [-s serialDev] [ -b bdaddr] [-B] [-i reportingInterval] [-H scratchHost] [-P scratchPort] [-h]\n";
    std::cout << "    -s /dev/ttyS? (use given serial port)" << std::endl;
    std::cout << "    -b bdaddr (use given bluetooth device)" << std::endl;
    std::cout << "    -B (use first available bluetooth device)" << std::endl;
    std::cout << "    -i N (use given reporting interval in ms, default 100ms)" << std::endl;
    std::cout << "    -H H (talk to scratch at given host, default localhost)" << std::endl;
    std::cout << "    -P P (talk to scratch on given port, default 42001)" << std::endl;
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

    while ((c = getopt(argc, argv, "s:b:Bi:H:P:h")) >= 0)
    {
        switch (c)
        {
            case 's': // firmata serial port
                port = optarg;
                conntype = 1;
                break;
            case 'b': // firmata specified bluetooth device
                port = optarg;
                conntype = 2;
                break;
            case 'B': // firmata first bluetooth device
                conntype = 3;
                break;
            case 'i': // reporting interval
                samplingInterval = atoi(optarg);
                break;
            case 'H': // scratch host
                scratch_host = optarg;
                break;
            case 'P': // scratch port
                scratch_port = atoi(optarg);
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
            DBG("connecting to serial port "<<port);
            try
            {
            serialio = new firmata::FirmSerial(port.c_str());
            }
            catch (...)
            {
                std::string e("Failed to open serial port ");
                e.append(port);
                usage(argv[0],e.c_str());
            }

            f = new firmata::Firmata<firmata::Base, firmata::I2C>(serialio);
            break;

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
            DBG("connecting to "<<port);
            try
            {
                bleio = new firmata::FirmBle(port.c_str());
                f = new firmata::Firmata<firmata::Base, firmata::I2C>(bleio);
            }
            catch (...)
            {
                std::string e("Failed to connect to Bluetooth device ");
                e.append(port);
                usage(argv[0],e.c_str());
            }
            break;

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
    read_pinstates();
    reset_timeout();

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
                connect_firmata();
                DBG("Connected to firmata");
                read_pinstates();
            }

            int n = do_poll();
            f->parse();

            // firmata will throw if not connected
            try
            {
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
                DBG("looks like firmata closed on us");
                // wait a while and try again
                sleep(1);
            }
        }

        DBG("Exited inner loop");
        // scratch went away or we are stopping
        disconnect_firmata();
        disconnect_scratch();
    }
    // all done
    disconnect_scratch();
}

