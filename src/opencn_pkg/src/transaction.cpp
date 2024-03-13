#include "opencn_pkg/transaction.h"

TransactionsElement::TransactionsElement(CMPin *pin, TransactionType transaction) : _pin(pin), _transaction(transaction)
{
    _interface = InterfaceType::PIN;
}

TransactionsElement::TransactionsElement(CMParam *param, TransactionType transaction) : _param(param), _transaction(transaction)
{
    _interface = InterfaceType::PARAM;
}

Transactions::Transactions(capnp::EzRpcClient *client) : _client(client), _cap{_client->getMain<CMCtlPins>()}
{
}

void Transactions::clear()
{
    _transactions.clear();
}

void Transactions::add(CMPin *pin, TransactionType transaction)
{
    // First, check is the new transaction is already present in the "vector"
    // It is expected that a PIN to be present only once in the vector
    for (const auto &trans : _transactions)
    {
        if (trans->_pin == pin)
        {
            std::cout << "[Transactions] add() / transaction already in the 'transactions' vector" << std::endl;
            return;
        }
    }

    auto element = new TransactionsElement(pin, transaction);

    _transactions.push_back(element);
}

void Transactions::add(CMParam *param, TransactionType transaction)
{
    // First, check is the new transaction is already present in the "vector"
    // It is expected that a PIN to be present only once in the vector
    for (const auto &trans : _transactions)
    {
        if (trans->_param == param)
        {
            std::cout << "[Transactions] add() / transaction already in the 'transactions' vector" << std::endl;
            return;
        }
    }

    auto element = new TransactionsElement(param, transaction);

    _transactions.push_back(element);
}

void Transactions::remove(CMPin *pin)
{
    int idx;
    bool found = false;

    for (size_t i = 0; i < _transactions.size(); i++)
    {
        if (_transactions[i]->_pin == pin)
        {
            idx = i;
            found = true;
        }
    }

    if (found)
    {
        auto transaction = _transactions[idx];
        _transactions.erase(_transactions.begin() + idx);
        delete transaction;
    }
}

void Transactions::remove(CMParam *param)
{
    int idx;
    bool found = false;

    for (size_t i = 0; i < _transactions.size(); i++)
    {
        if (_transactions[i]->_param == param)
        {
            idx = i;
            found = true;
        }
    }

    if (found)
    {
        auto transaction = _transactions[idx];
        _transactions.erase(_transactions.begin() + idx);
        delete transaction;
    }
}

size_t Transactions::size()
{
    return _transactions.size();
}

void Transactions::execute()
{
    // std::cout << "[Transactions] execute()"; // debug
    kj::WaitScope &waitScope = _client->getWaitScope();

    auto request = _cap.actionsRequest();

    auto actions = request.initActions(_transactions.size());

    for (size_t i = 0; i < _transactions.size(); i++)
    {
        // std::cout << "/ transaction " << i << " : "; // debug
        switch (_transactions[i]->_interface)
        {
        case InterfaceType::PIN:
            actions[i].setName(_transactions[i]->_pin->name());
            // std::cout << " ,this is a pin named " << _transactions[i]->_pin->name(); // debug

            switch (_transactions[i]->_transaction)
            {
            case TransactionType::GET:
                actions[i].setAction(CMCtlModel::Access::GET);
                // std::cout << " ,this is a get action" << std::endl; // debug
                break;
            case TransactionType::SET:
                actions[i].setAction(CMCtlModel::Access::SET);
                // std::cout << " ,this is a set action" << std::endl; // debug

                auto val = actions[i].getValue().getValue();
                // std::cout << " ,the value is of type " << _transactions[i]->_pin->type() << std::endl; // debug
                switch (_transactions[i]->_pin->type())
                {
                case HAL_BIT:
                    val.setB(*static_cast<CMPinBit *>(_transactions[i]->_pin));
                    break;
                case HAL_FLOAT:
                    val.setF(*static_cast<CMPinFloat *>(_transactions[i]->_pin));
                    break;
                case HAL_S32:
                    val.setS(*static_cast<CMPinI32 *>(_transactions[i]->_pin));
                    break;
                case HAL_U32:
                    val.setU(*static_cast<CMPinU32 *>(_transactions[i]->_pin));
                    break;
                }
                break;
            }
            break;
        case InterfaceType::PARAM:
            actions[i].setName(_transactions[i]->_param->name());
            // std::cout << ", this is a param named " << _transactions[i]->_param->name(); // debug

            switch (_transactions[i]->_transaction)
            {
            case TransactionType::GET:
                actions[i].setAction(CMCtlModel::Access::GET);
                // std::cout << " ,this is a get action" << std::endl; // debug
                break;
            case TransactionType::SET:
                actions[i].setAction(CMCtlModel::Access::SET);
                // std::cout << " ,this is a set action" << std::endl; // debug

                auto val = actions[i].getValue().getValue();
                switch (_transactions[i]->_param->type())
                {
                case HAL_BIT:
                    val.setB(*static_cast<CMParamBit *>(_transactions[i]->_param));
                    break;
                case HAL_FLOAT:
                    val.setF(*static_cast<CMParamFloat *>(_transactions[i]->_param));
                    break;
                case HAL_S32:
                    val.setS(*static_cast<CMParamI32 *>(_transactions[i]->_param));
                    break;
                case HAL_U32:
                    val.setU(*static_cast<CMParamU32 *>(_transactions[i]->_param));
                    break;
                }
                break;
            }
            break;
        }
    }

    // std::cout << "[Transactions] execute() / request.send()" << std::endl; // debug
    auto promise = request.send();
    // std::cout << "[Transactions] execute() / promise.wait()" << std::endl; // debug
    auto response = promise.wait(waitScope);
    // std::cout << "[Transactions] execute() / response.getResults()" << std::endl; // debug
    auto results = response.getResults();

    for (size_t i = 0; i < _transactions.size(); i++)
    {
        auto error = results[i].getError();

        if (_transactions[i]->_transaction == TransactionType::GET)
        {
            // std::cout << "[DEBUG] I whent in second if" << std::endl; // debug

            if (error == CMCTL_SUCCESS)
            {
                auto val = results[i].getValue().getValue();

                switch (_transactions[i]->_interface)
                {
                case InterfaceType::PIN:
                    _transactions[i]->_pin->setError(error);
                    // std::cout << "I think this is a pin" << std::endl; // debug
                    switch (_transactions[i]->_pin->type())
                    {
                    case HAL_BIT:
                        static_cast<CMPinBit *>(_transactions[i]->_pin)->set(val.getB());
                        break;
                    case HAL_FLOAT:
                        static_cast<CMPinFloat *>(_transactions[i]->_pin)->set(val.getF());
                        break;
                    case HAL_S32:
                        static_cast<CMPinI32 *>(_transactions[i]->_pin)->set(val.getS());
                        break;
                    case HAL_U32:
                        static_cast<CMPinU32 *>(_transactions[i]->_pin)->set(val.getU());
                        break;
                    }
                    break;
                case InterfaceType::PARAM:
                    _transactions[i]->_param->setError(error);
                    // std::cout << "I think it's a param" << std::endl; // debug
                    switch (_transactions[i]->_param->type())
                    {
                    case HAL_BIT:
                        static_cast<CMParamBit *>(_transactions[i]->_param)->set(val.getB());
                        break;
                    case HAL_FLOAT:
                        static_cast<CMParamFloat *>(_transactions[i]->_param)->set(val.getF());
                        break;
                    case HAL_S32:
                        static_cast<CMParamI32 *>(_transactions[i]->_param)->set(val.getS());
                        break;
                    case HAL_U32:
                        static_cast<CMParamU32 *>(_transactions[i]->_param)->set(val.getU());
                        break;
                    }
                    break;
                }
            }
            // else
            // {
            //     std::cout << "[Transactions] execute() / error != CMCTL_SUCCESS" << std::endl; // debug
            // }
        }
    }
}
