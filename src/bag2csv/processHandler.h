#ifndef PROCESSHANDLER_H
#define PROCESSHANDLER_H


#include <string>
#include <vector>
#include <QProcess>


class processHandler
{
    Q_OBJECT

public:
    explicit processHandler();
    ~processHandler();
    QProcess *pocBagPlay;
    QProcess *pocEndPlay;


private slots:
   


private:
    

};

#endif // PROCESSHANDLER_H
