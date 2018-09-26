#include "ejemplo1.h"


ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
       
 
        connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
        connect(&time, SIGNAL(timeout()), this, SLOT(cuenta()) );
        
        time.start();
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
        time.stop_time();
        
}


void ejemplo1::cuenta()
{
        contador++;
        lcdNumber->display(contador);
    
}

