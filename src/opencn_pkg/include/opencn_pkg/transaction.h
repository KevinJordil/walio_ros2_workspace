#ifndef TRANSACTIONS_H
#define TRANSACTIONS_H

#include <iostream>
#include <vector>
#include <capnp/ez-rpc.h>

#include "opencn_pkg/pins.h"
#include "opencn_pkg/param.h"

enum class TransactionType
{
    GET,
    SET
};

enum class InterfaceType
{
    PIN = 0,
    PARAM = 1
};

class TransactionsElement
{

public:
    TransactionsElement(CMPin *pin, TransactionType transaction);
    TransactionsElement(CMParam *param, TransactionType transaction);
    CMPin *_pin = nullptr;
    CMParam *_param = nullptr;
    TransactionType _transaction;
    InterfaceType _interface;
};

class Transactions
{

public:
    Transactions(capnp::EzRpcClient *client);

    void clear();

    void add(CMPin *pin, TransactionType transaction);
    void add(CMParam *param, TransactionType transaction);

    void remove(CMPin *pin);
    void remove(CMParam *param);

    void execute();

    size_t size();

protected:
    capnp::EzRpcClient *_client;
    CMCtlPins::Client _cap;

    std::vector<TransactionsElement *> _transactions;
};

#endif /* TRANSACTIONS_H */
