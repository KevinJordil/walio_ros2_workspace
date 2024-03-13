#ifndef COMPONENT_SPECIFIC_H
#define COMPONENT_SPECIFIC_H

#include <string>
#include <vector>
#include <memory>

#include <capnp/ez-rpc.h>
//#include <opencn/uapi/hal.h>
#include "opencn_pkg/hal.h"

//#include <pins.h>
#include "opencn_pkg/pins.h"
//#include <param.h>
#include "opencn_pkg/param.h"
#include "opencn_pkg/cmctl.h"
#include "opencn_pkg/opencn-interface/opencn_interface.capnp.h"
#include "opencn_pkg/component.h"

#define MODE_CSP 1
#define MODE_CSV 2
#define MODE_CST 4
#define MODE_HM 8
#define MODE_PPM 16
#define MODE_PVM 32
#define MODE_INACTIVE 64
#define MODE_NONE 0

/************************************ MUX N to 1 *************************************************/

template <typename T>
class MuxNTo1T : public Component
{
public:
    MuxNTo1T(std::string name, capnp::EzRpcClient *client, unsigned int numInputs)
        : Component(name, client), _numInputs(numInputs),
          _output("mux." + this->_name + ".output0", client),
          _select("mux." + this->_name + ".selector", client)
    {
        for (unsigned int i = 0; i < _numInputs; i++)
        {
            std::string pin_name = "mux." + this->_name + ".vector" + std::to_string(i) + ".input0";
            _inputs.push_back(CMPinT<T>(pin_name, client));
        }
    }

    T getPin(std::string pin_name)
    {
        if (pin_name == _output.name())
            return _output.get();
        else if (pin_name == _select.name())
            return _select.get();
        else
        {
            for (unsigned int i = 0; i < _numInputs; i++)
            {
                if (pin_name == _inputs[i].name())
                    return _inputs[i].get();
            }
        }
        return T{}; // Default value for unsupported pin_name
    }

    void getPins()
    {
        _output.get();
        _select.get();
        for (unsigned int i = 0; i < _numInputs; i++)
            _inputs[i].get();
    }

    void setPin(std::string pin_name, hal_data_u value)
    {
        T val;

        // check type of T
        if (std::is_same<uint32_t, T>::value)
            val = value.u;
        else if (std::is_same<T, int32_t>::value)
            val = value.s;
        else if (std::is_same<T, double>::value)
            val = value.f;
        else if (std::is_same<T, bool>::value)
            val = value.b;
        else
            return;

        if (pin_name == _output.name())
            _output.set(val);
        else if (pin_name == _select.name())
            _select.set(value.u);
        else
        {
            for (unsigned int i = 0; i < _numInputs; i++)
            {
                if (pin_name == _inputs[i].name())
                    _inputs[i].set(val);
            }
        }
    }

    bool setInput(unsigned int input_num, T value)
    {
        if (input_num < _numInputs)
            _inputs[input_num].set(value);
        else
            return true;
        return false;
    }

    bool setSelector(unsigned int select)
    {
        if (select < _numInputs)
            _select.set(select);
        else
            return true;
        return false;
    }

    void print()
    {
        this->getPins();
        std::cout << "Mux : " << this->_name << "\n";
        std::cout << "Output : " << _output.get() << "\n";
        std::cout << "Selector : " << _select.get() << "\n";
        for (unsigned int i = 0; i < _numInputs; i++)
            std::cout << "Input " << i << " : " << _inputs[i].get() << "\n";

        std::cout << "\n";
    }

private:
protected:
    unsigned int _numInputs;
    CMPinT<T> _output;
    std::vector<CMPinT<T>> _inputs;
    CMPinU32 _select;
};

using MuxNTo1I = MuxNTo1T<int32_t>;
using MuxNTo1U = MuxNTo1T<uint32_t>;
using MuxNTo1F = MuxNTo1T<double>;
using MuxNTo1B = MuxNTo1T<bool>;

/**************************** Simple pg *********************************************/
class SimplePG : public Component
{
public:
    SimplePG(capnp::EzRpcClient *client, unsigned int numJoint) : Component("simple_pg", client),
                                                                  _numJoint(numJoint)
    {
        for (unsigned int i = 0; i < _numJoint; i++)
        {
            std::string pin_name = "simple_pg.joint" + std::to_string(i) + ".cmd-pos";
            _cmd_pos.push_back(CMPinFloat(pin_name, client));
            pin_name = "simple_pg.joint" + std::to_string(i) + ".current-position";
            _current_pos.push_back(CMPinFloat(pin_name, client));
            pin_name = "simple_pg.joint" + std::to_string(i) + ".relative-pos";
            _relative_pos.push_back(CMPinBit(pin_name, client));
            pin_name = "simple_pg.joint" + std::to_string(i) + ".stop";
            _stop.push_back(CMPinBit(pin_name, client));
            pin_name = "simple_pg.joint" + std::to_string(i) + ".is_running";
            _is_running.push_back(CMPinBit(pin_name, client));
            pin_name = "simple_pg.joint" + std::to_string(i) + "._target_position";
            _target_position.push_back(CMPinFloat(pin_name, client));
        }
    }

    void setPin(std::string pin_name, hal_data_u value)
    {
        float val;

        if (pin_name == _enable.name())
            _enable.set(value.b);
        else
        {
            for (unsigned int i = 0; i < _numJoint; i++)
            {
                if (pin_name == _cmd_pos[i].name())
                    _cmd_pos[i].set(value.f);
                else if (pin_name == _relative_pos[i].name())
                    _relative_pos[i].set(value.b);
                else if (pin_name == _stop[i].name())
                    _stop[i].set(value.b);
            }
        }
    }

    void getPins()
    {
        _enable.get();
        _is_enable.get();
        for (unsigned int i = 0; i < _numJoint; i++)
        {
            _cmd_pos[i].get();
            _relative_pos[i].get();
            _stop[i].get();
            _is_running[i].get();
            _target_position[i].get();
            _current_pos[i].get();
        }
    }

    void setEnable(bool enable)
    {
        _enable.set(enable);
    }

    void setCmdPos(unsigned int joint, float cmd_pos)
    {
        _cmd_pos[joint].set(cmd_pos);
    }

    void setRelativePos(unsigned int joint, bool relative_pos)
    {
        _relative_pos[joint].set(relative_pos);
    }

    void setStop(unsigned int joint, bool stop)
    {
        _stop[joint].set(stop);
    }

    bool getIsEnable()
    {
        return _is_enable.get();
    }

    bool getIsRunning(unsigned int joint)
    {
        return _is_running[joint].get();
    }

    float getTargetPosition(unsigned int joint)
    {
        return _target_position[joint].get();
    }

    float getCurrentPosition(unsigned int joint)
    {
        return _current_pos[joint].get();
    }

    void print()
    {
        this->getPins();
        std::cout << "Simple PG : " << this->_name << "\n";
        std::cout << "Enable : " << _enable.get() << "\n";
        std::cout << "Is Enable : " << _is_enable.get() << "\n";
        for (unsigned int i = 0; i < _numJoint; i++)
        {
            std::cout << "Joint " << i << " :\n";
            std::cout << "Cmd Pos : " << _cmd_pos[i].get() << "\n";
            std::cout << "Relative Pos : " << _relative_pos[i].get() << "\n";
            std::cout << "Stop : " << _stop[i].get() << "\n";
            std::cout << "Is Running : " << _is_running[i].get() << "\n";
            std::cout << "Target Position : " << _target_position[i].get() << "\n";
            std::cout << "Current Position : " << _current_pos[i].get() << "\n";
        }
        std::cout << "\n";
    }

protected:
    unsigned int _numJoint;
    CMPinBit _enable = CMPinBit("simple_pg.enable", _client);
    CMPinBit _is_enable = CMPinBit("simple_pg.is_enable", _client);
    std::vector<CMPinFloat> _cmd_pos;
    std::vector<CMPinFloat> _current_pos;
    std::vector<CMPinBit> _relative_pos;
    std::vector<CMPinBit> _stop;
    std::vector<CMPinBit> _is_running;
    std::vector<CMPinFloat> _target_position;
};

/********************************* lcec *********************************************/

class Lcec_epos4 : public Component
{
public:
    Lcec_epos4(std::string name, capnp::EzRpcClient *client) : Component(name, client),
                                                               /*output*/
                                                               _actual_position("lcec." + std::to_string(_numLcec) + "." + name + ".actual-position", client),
                                                               _actual_velocity("lcec." + std::to_string(_numLcec) + "." + name + ".actual-velocity", client),
                                                               _actual_torque("lcec." + std::to_string(_numLcec) + "." + name + ".actual-torque", client),
                                                               _errore_code("lcec." + std::to_string(_numLcec) + "." + name + ".error-code", client),
                                                               _homed("lcec." + std::to_string(_numLcec) + "." + name + ".homed", client),
                                                               _idle("lcec." + std::to_string(_numLcec) + "." + name + ".idle", client),
                                                               _in_fault("lcec." + std::to_string(_numLcec) + "." + name + ".in-fault", client),
                                                               _in_mode_csp("lcec." + std::to_string(_numLcec) + "." + name + ".in-mode-csp", client),
                                                               _in_mode_csv("lcec." + std::to_string(_numLcec) + "." + name + ".in-mode-csv", client),
                                                               _in_mode_cst("lcec." + std::to_string(_numLcec) + "." + name + ".in-mode-cst", client),
                                                               _in_mode_hm("lcec." + std::to_string(_numLcec) + "." + name + ".in-mode-hm", client),
                                                               _in_mode_ppm("lcec." + std::to_string(_numLcec) + "." + name + ".in-mode-ppm", client),
                                                               _in_mode_pvm("lcec." + std::to_string(_numLcec) + "." + name + ".in-mode-pvm", client),
                                                               _in_mode_inactive("lcec." + std::to_string(_numLcec) + "." + name + ".in-mode-inactive", client),
                                                               _online("lcec." + std::to_string(_numLcec) + "." + name + ".online", client),
                                                               _operational("lcec." + std::to_string(_numLcec) + "." + name + ".operational", client),
                                                               _set_target("lcec." + std::to_string(_numLcec) + "." + name + ".set-target", client),
                                                               _state_init("lcec." + std::to_string(_numLcec) + "." + name + ".state-init", client),
                                                               _state_op("lcec." + std::to_string(_numLcec) + "." + name + ".state-op", client),
                                                               _state_preop("lcec." + std::to_string(_numLcec) + "." + name + ".state-preop", client),
                                                               _state_safeop("lcec." + std::to_string(_numLcec) + "." + name + ".state-safeop", client),

                                                               /*input*/
                                                               _fault_reset("lcec." + std::to_string(_numLcec) + "." + name + ".fault-reset", client),
                                                               _hm_start("lcec." + std::to_string(_numLcec) + "." + name + ".hm.start", client),
                                                               _hm_stop("lcec." + std::to_string(_numLcec) + "." + name + ".hm.stop", client),
                                                               _ppm_start("lcec." + std::to_string(_numLcec) + "." + name + ".ppm.start", client),
                                                               _ppm_stop("lcec." + std::to_string(_numLcec) + "." + name + ".ppm.stop", client),
                                                               _quick_stop("lcec." + std::to_string(_numLcec) + "." + name + ".quick-stop", client),
                                                               _set_mode_csp("lcec." + std::to_string(_numLcec) + "." + name + ".set-mode-csp", client),
                                                               _set_mode_csv("lcec." + std::to_string(_numLcec) + "." + name + ".set-mode-csv", client),
                                                               _set_mode_cst("lcec." + std::to_string(_numLcec) + "." + name + ".set-mode-cst", client),
                                                               _set_mode_hm("lcec." + std::to_string(_numLcec) + "." + name + ".set-mode-hm", client),
                                                               _set_mode_ppm("lcec." + std::to_string(_numLcec) + "." + name + ".set-mode-ppm", client),
                                                               _set_mode_pvm("lcec." + std::to_string(_numLcec) + "." + name + ".set-mode-pvm", client),
                                                               _set_mode_inactive("lcec." + std::to_string(_numLcec) + "." + name + ".set-mode-inactive", client),

                                                               /*parameter*/
                                                               _modulo("lcec." + std::to_string(_numLcec) + "." + name + ".modulo", client),
                                                               _pos_scale("lcec." + std::to_string(_numLcec) + "." + name + ".pos-scale", client),
                                                               _velocity_scale("lcec." + std::to_string(_numLcec) + "." + name + ".velocity-scale", client),
                                                               _torque_scale("lcec." + std::to_string(_numLcec) + "." + name + ".torque-scale", client),
                                                               _endless_movement("lcec." + std::to_string(_numLcec) + "." + name + ".ppm.endless-movement", client),
                                                               _immediate_change("lcec." + std::to_string(_numLcec) + "." + name + ".ppm.immediate-change", client),
                                                               _relative_position("lcec." + std::to_string(_numLcec) + "." + name + ".ppm.relative-pos", client)

    {
        _numLcec++;
    }

    void getPins()
    {
        /*inputs*/
        _fault_reset.get();
        _hm_start.get();
        _hm_stop.get();
        _ppm_start.get();
        _ppm_stop.get();
        _set_mode_csp.get();
        _set_mode_csv.get();
        _set_mode_cst.get();
        _set_mode_hm.get();
        _set_mode_ppm.get();
        _set_mode_pvm.get();
        _set_target.get();

        /*outputs*/
        _actual_position.get();
        _actual_velocity.get();
        _actual_torque.get();
        _errore_code.get();
        _homed.get();
        _idle.get();
        _in_fault.get();
        _in_mode_csp.get();
        _in_mode_csv.get();
        _in_mode_cst.get();
        _in_mode_hm.get();
        _in_mode_ppm.get();
        _in_mode_pvm.get();
        _online.get();
        _operational.get();
        _quick_stop.get();
        _state_init.get();
        _state_op.get();
        _state_preop.get();
        _state_safeop.get();
    }

    virtual void setPin(std::string pin_name, hal_data_u value)
    {
        if (pin_name == _fault_reset.name())
            _fault_reset.set(value.b);
        else if (pin_name == _hm_start.name())
            _hm_start.set(value.b);
        else if (pin_name == _hm_stop.name())
            _hm_stop.set(value.b);
        else if (pin_name == _ppm_start.name())
            _ppm_start.set(value.b);
        else if (pin_name == _ppm_stop.name())
            _ppm_stop.set(value.b);
        else if (pin_name == _set_mode_csp.name())
            _set_mode_csp.set(value.b);
        else if (pin_name == _set_mode_csv.name())
            _set_mode_csv.set(value.b);
        else if (pin_name == _set_mode_cst.name())
            _set_mode_cst.set(value.b);
        else if (pin_name == _set_mode_hm.name())
            _set_mode_hm.set(value.b);
        else if (pin_name == _set_mode_ppm.name())
            _set_mode_ppm.set(value.b);
        else if (pin_name == _set_mode_pvm.name())
            _set_mode_pvm.set(value.b);
        else if (pin_name == _set_mode_inactive.name())
            _set_mode_inactive.set(value.b);
        else if (pin_name == _set_target.name())
            _set_target.set(value.f);
    }

    int getMode()
    {
        if (_in_mode_csp.get())
            return 0b0000001;
        else if (_in_mode_csv.get())
            return 0b0000010;
        else if (_in_mode_cst.get())
            return 0b0000100;
        else if (_in_mode_hm.get())
            return 0b0001000;
        else if (_in_mode_ppm.get())
            return 0b0010000;
        else if (_in_mode_pvm.get())
            return 0b0100000;
        else if (_in_mode_inactive.get())
            return 0b1000000;
        else
            return 0b000000;
    }

    void setMode(int mode)
    {
        _set_mode_csp.set(mode & 0b0000001);
        _set_mode_csv.set(mode & 0b0000010);
        _set_mode_cst.set(mode & 0b0000100);
        _set_mode_hm.set(mode & 0b0001000);
        _set_mode_ppm.set(mode & 0b0010000);
        _set_mode_pvm.set(mode & 0b0100000);
        _set_mode_inactive.set(mode & 0b1000000);
    }

    void printInputs()
    {
        std::cout << "Inputs: " << std::endl;
        std::cout << "Fault reset: " << _fault_reset.get() << std::endl;
        std::cout << "Hm start: " << _hm_start.get() << std::endl;
        std::cout << "Hm stop: " << _hm_stop.get() << std::endl;
        std::cout << "Ppm start: " << _ppm_start.get() << std::endl;
        std::cout << "Ppm stop: " << _ppm_stop.get() << std::endl;
        std::cout << "Set mode csp: " << _set_mode_csp.get() << std::endl;
        std::cout << "Set mode csv: " << _set_mode_csv.get() << std::endl;
        std::cout << "Set mode cst: " << _set_mode_cst.get() << std::endl;
        std::cout << "Set mode hm: " << _set_mode_hm.get() << std::endl;
        std::cout << "Set mode ppm: " << _set_mode_ppm.get() << std::endl;
        std::cout << "Set mode pvm: " << _set_mode_pvm.get() << std::endl;
        std::cout << "Set mode inactive: " << _set_mode_inactive.get() << std::endl;
        std::cout << "Set target: " << _set_target.get() << std::endl;
        std::cout << "Quick stop: " << _quick_stop.get() << std::endl;
    }

    void printOutputs()
    {
        std::cout << "Outputs: " << std::endl;
        std::cout << "Homed: " << _homed.get() << std::endl;
        std::cout << "Idle: " << _idle.get() << std::endl;
        std::cout << "In fault: " << _in_fault.get() << std::endl;
        std::cout << "Error code: " << _errore_code.get() << std::endl;
        std::cout << "Actual position: " << _actual_position.get() << std::endl;
        std::cout << "Actual velocity: " << _actual_velocity.get() << std::endl;
        std::cout << "Actual torque: " << _actual_torque.get() << std::endl;
        std::cout << "In mode csp: " << _in_mode_csp.get() << std::endl;
        std::cout << "In mode csv: " << _in_mode_csv.get() << std::endl;
        std::cout << "In mode cst: " << _in_mode_cst.get() << std::endl;
        std::cout << "In mode hm: " << _in_mode_hm.get() << std::endl;
        std::cout << "In mode ppm: " << _in_mode_ppm.get() << std::endl;
        std::cout << "In mode pvm: " << _in_mode_pvm.get() << std::endl;
        std::cout << "In mode inactive: " << _in_mode_inactive.get() << std::endl;
        std::cout << "Online: " << _online.get() << std::endl;
        std::cout << "Operational: " << _operational.get() << std::endl;
        std::cout << "State op: " << _state_op.get() << std::endl;
        std::cout << "State preop: " << _state_preop.get() << std::endl;
        std::cout << "State safeop: " << _state_safeop.get() << std::endl;
        std::cout << "State init: " << _state_init.get() << std::endl;
    }

    void printParameters()
    {
        std::cout << "Parameters: " << std::endl;
        std::cout << "Position scale: " << _pos_scale.get() << std::endl;
        std::cout << "Velocity scale: " << _velocity_scale.get() << std::endl;
        std::cout << "Torque scale: " << _torque_scale.get() << std::endl;
        std::cout << "Modulo: " << _modulo.get() << std::endl;
        std::cout << "Endless movement: " << _endless_movement.get() << std::endl;
        std::cout << "Immediate change: " << _immediate_change.get() << std::endl;
        std::cout << "Relative position: " << _relative_position.get() << std::endl;
    }

    void print()
    {
        printInputs();
        printOutputs();
        printParameters(); // TODO : uncomment when parameters are implemented
    }

    bool isHomed()
    {
        return _homed.get();
    }

    bool isIdle()
    {
        return _idle.get();
    }

    bool isInFault()
    {
        return _in_fault.get();
    }

    void resetFault()
    {
        _fault_reset.set(true);
    }

    void quickStop()
    {
        _quick_stop.set(true);
    }

    u_int32_t getErrorCode()
    {
        return _errore_code.get();
    }

protected:
    static unsigned int _numLcec;

    /*inputs*/
    CMPinBit _fault_reset;
    CMPinBit _hm_start;
    CMPinBit _hm_stop;
    CMPinBit _ppm_start;
    CMPinBit _ppm_stop;
    CMPinBit _set_mode_csp;
    CMPinBit _set_mode_csv;
    CMPinBit _set_mode_cst;
    CMPinBit _set_mode_hm;
    CMPinBit _set_mode_ppm;
    CMPinBit _set_mode_pvm;
    CMPinBit _set_mode_inactive;
    CMPinFloat _set_target;

    /*outputs*/
    CMPinFloat _actual_position;
    CMPinFloat _actual_velocity;
    CMPinFloat _actual_torque;
    CMPinU32 _errore_code;
    CMPinBit _homed;
    CMPinBit _idle;
    CMPinBit _in_fault;
    CMPinBit _in_mode_csp;
    CMPinBit _in_mode_csv;
    CMPinBit _in_mode_cst;
    CMPinBit _in_mode_hm;
    CMPinBit _in_mode_ppm;
    CMPinBit _in_mode_pvm;
    CMPinBit _in_mode_inactive;
    CMPinBit _online;
    CMPinBit _operational;
    CMPinBit _quick_stop;
    CMPinBit _state_init;
    CMPinBit _state_op;
    CMPinBit _state_preop;
    CMPinBit _state_safeop;

    /*parameters*/ // TODO: Error Parameter != Pins
    CMParamFloat _modulo;
    CMParamFloat _pos_scale;
    CMParamFloat _velocity_scale;
    CMParamFloat _torque_scale;
    CMParamBit _endless_movement;
    CMParamBit _immediate_change;
    CMParamBit _relative_position;
};

#endif // !COMPONENT_SPECIFIC_H
