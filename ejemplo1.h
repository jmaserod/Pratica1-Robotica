#ifndef ejemplo1_H
#define ejemplo1_H

#include <IceUtil/Timer.h>
#include "my_qtimer.h"

//hola


#include <QtGui>
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
public:
    
    my_qtimer time;
    int contador = 0;
   
Q_OBJECT
    public:
        ejemplo1();
        virtual ~ejemplo1();
        
    public slots:
            void doButton();
            void cuenta();
};
 
        

#endif // ejemplo1_H
