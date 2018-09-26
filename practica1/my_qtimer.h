#ifndef my_qtimer_H
#define my_qtimer_H

#include <QtCore>

class my_qtimer : public QThread
{
    Q_OBJECT

public:
    bool sttop;
  
public:
    my_qtimer();
    void run();
    virtual ~my_qtimer();
    void stop_time();
signals:
    void timeout();
    

    
};
#endif // ejemplo1_H
