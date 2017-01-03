#include "processHandler.h"

processHandler::processHandler(){
    pocBagPlay = new QProcess;
    pocEndPlay = new QProcess;
}
processHandler::~processHandler(){
    delete pocBagPlay;
    delete pocEndPlay;
}


