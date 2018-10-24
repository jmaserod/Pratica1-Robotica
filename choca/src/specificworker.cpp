/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = new InnerModel(innermodel_path);
}
	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
	
	try{

        RoboCompGenericBase::TBaseState robotState;
        differentialrobot_proxy->getBaseState(robotState);
        // read laser data
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        //sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; });

        if(T.isActivo()){

             innerModel->updateTransformValues("base", robotState.x, 0, robotState.z, 0, robotState.alpha, 0);

             Rot2D rot (robotState.alpha);
             auto sub = QVec::vec2(T.x - robotState.x, T.z - robotState.z);
             auto relative = rot.invert() *(sub);

             float angle = atan2(relative.x(), relative.y());
             float mod =relative.norm2();

            if( 50 > mod)
            {
                differentialrobot_proxy->setSpeedBase(0,0);
                T.setInactivo();
            }
            else {
                differentialrobot_proxy->setSpeedBase(400,angle);
                }

        }
        /*	if(aleat%10 == 1){
                differentialrobot_proxy->setSpeedBase(10,0.5);
                usleep(aleatorio);

		}
		else{
			differentialrobot_proxy->setSpeedBase(10,-0.5);
			usleep(aleatorio);
		}
	}
	else
		differentialrobot_proxy->setSpeedBase(400,0);
	*/
    } catch(const Ice::Exception &ex)
	    {
	        std::cout << ex << std::endl;
	    }
  


}
void SpecificWorker::setPick(const Pick &myPick)
{
        qDebug()<<myPick.x<<myPick.z;

    	T.insertarCoordenadas(myPick.x, myPick.z);

}
