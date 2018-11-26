#include <common/MoveActionServerBase.h>

MoveActionServerBase::MoveActionServerBase(const std::string &actionName):
        actionServer(nodeHandle, actionName, boost::bind(&MoveActionServerBase::executeCallback, this, _1), false) {
    actionServer.start();
}