/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2023 Bruno Sutterlet
 */

#ifndef PARAM_H
#define PARAM_H

#include <capnp/ez-rpc.h>
//#include <opencn/uapi/hal.h>
#include "opencn_pkg/hal.h"
#include <string>

#include "opencn_pkg/cmctl.h"
#include "opencn_pkg/opencn-interface/opencn_interface.capnp.h"

class CMParam
{

public:
    // Define the how the value is accessed:
    //   * LOCAL: The local PIN 'value' is used to set or get - not network transaction
    //   * PIN: The current PINs of the 'target' is set or get - Network transaction
    enum class AccessMode
    {
        LOCAL = 0,
        PARAM
    };

    std::string name() { return _name; } // -> Generic to interface
    hal_type_t type() { return _type; }  // -> Generic to interface

    void setError(int error) { _error = error; } // -> Generic to interface
    int error() { return _error; }               // -> Generic to interface

    void setMode(AccessMode mode) { _mode = mode; } // -> Generic to interface ?
    AccessMode &mode() { return _mode; }            // -> Generic to interface

protected:
    CMParam(std::string name, capnp::EzRpcClient *client) : _name(name), _client(client),
                                                            _cap{_client->getMain<CMCtlParams>()},
                                                            _mode(AccessMode::LOCAL),
                                                            _type(HAL_TYPE_UNSPECIFIED),
                                                            _error(CMCTL_SUCCESS) {}

    void set(hal_data_u value) // -> Generic to interface
    {
        kj::WaitScope &waitScope = _client->getWaitScope();

        auto request = _cap.setValRequest();

        request.setName(_name);
        auto val = request.getValue().getValue();

        if (_type == HAL_BIT)
            val.setB(value.b);
        else if (_type == HAL_FLOAT)
            val.setF(value.f);
        else if (_type == HAL_S32)
            val.setS(value.s);
        else if (_type == HAL_U32)
            val.setU(value.u);

        auto promise = request.send();
        auto response = promise.wait(waitScope);

        _error = response.getError();
    }

    hal_data_u get() // -> Generic to interface
    {
        kj::WaitScope &waitScope = _client->getWaitScope();

        hal_data_u value;

        auto request = _cap.getValRequest();

        request.setName(_name);

        auto promise = request.send();
        auto response = promise.wait(waitScope);

        _error = response.getError();
        if (_error == CMCTL_SUCCESS)
        {
            // Value updated only if the transaction succeeded
            auto val = response.getValue().getValue();

            if (_type == HAL_BIT)
                value.b = val.getB();
            else if (_type == HAL_FLOAT)
                value.f = val.getF();
            else if (_type == HAL_S32)
                value.s = val.getS();
            else if (_type == HAL_U32)
                value.u = val.getU();
        }

        return value;
    }

    // Capnp RPC client
    capnp::EzRpcClient *_client; // -> Generic to interface

    // CMCtlParams interface capabilities
    CMCtlParams::Client _cap; // -> Generic to interface ?

    // PIN name as reported by  halcmd show param name
    std::string _name; // -> Generic to interface

    // PIN type
    hal_type_t _type; // -> Generic to interface

    // last transaction error
    int _error; // -> Generic to interface

    // PIN value access mode
    AccessMode _mode; // -> Generic to interface
};

template <class T>
class CMParamT : public CMParam
{
public:
    CMParamT(std::string name, capnp::EzRpcClient *client) : CMParam(name, client)
    {

        if (std::is_same<int32_t, T>::value)
        {
            _type = HAL_S32;
            _value = 0;
        }
        else if (std::is_same<uint32_t, T>::value)
        {
            _type = HAL_U32;
            _value = 0;
        }
        else if (std::is_same<bool, T>::value)
        {
            _type = HAL_BIT;
            _value = false;
        }
        else if (std::is_same<double, T>::value)
        {
            _type = HAL_FLOAT;
            _value = 0.0;
        }
    }

    void set(T value)
    {
        hal_data_u val;

        if (std::is_same<int32_t, T>::value)
            val.s = value;
        else if (std::is_same<uint32_t, T>::value)
            val.u = value;
        else if (std::is_same<bool, T>::value)
            val.b = value;
        else if (std::is_same<double, T>::value)
            val.f = value;

        CMParam::set(val);

        _value = value;
    }

    T get()
    {
        hal_data_u val = CMParam::get();

        if (std::is_same<int32_t, T>::value)
            _value = val.s;
        else if (std::is_same<uint32_t, T>::value)
            _value = val.u;
        else if (std::is_same<bool, T>::value)
            _value = val.b;
        else if (std::is_same<double, T>::value)
            _value = val.f;

        return _value;
    }

    operator T()
    {
        if (_mode == AccessMode::PARAM)
            return get();
        else
            return _value;
    }

    void operator=(T value)
    {
        if (_mode == AccessMode::PARAM)
            set(value);
        else
            _value = value;
    }

protected:
    T _value;
};

using CMParamI32 = CMParamT<int32_t>;
using CMParamU32 = CMParamT<uint32_t>;
using CMParamBit = CMParamT<bool>;
using CMParamFloat = CMParamT<double>;

#endif /* PARAM */
