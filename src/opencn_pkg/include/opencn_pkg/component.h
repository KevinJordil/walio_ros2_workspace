// COMPONENT_H
#ifndef COMPONENT_H
#define COMPONENT_H

#include <string>
#include <vector>
#include <memory>

#include <capnp/ez-rpc.h>
//#include <opencn/uapi/hal.h>
#include "opencn_pkg/hal.h"

//#include <pins.h>
#include "opencn_pkg/pins.h"
#include "opencn_pkg/cmctl.h"
#include "opencn_pkg/opencn-interface/opencn_interface.capnp.h"

class Component
{
public:
    Component(std::string name, capnp::EzRpcClient *client) : _name(name), _client(client) {}

    virtual void getPins() = 0;
    virtual void setPin(std::string pin_name, hal_data_u value) = 0;

    operator std::string() const { return _name; }

private:
protected:
    std::string _name;
    capnp::EzRpcClient *_client;
};

#endif // !COMPONENT_H
