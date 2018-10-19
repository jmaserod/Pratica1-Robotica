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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <pthread.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void setPick(const Pick &myPick);

public slots:
	void compute();

private:
	InnerModel *innerModel;

	int robotX, robotZ;
	
	
	struct Target{
		QMutex mutex;
		float x, z;
		bool activo = true;
		Target(){};


		bool insertarCoordenadas(float coordenadaX, float coordenadaZ){
			
		    QMutexLocker lm(&mutex);
			x = coordenadaX;
			z = coordenadaZ;
			activo = false;
			return true;
		}

		bool isActivo(){
    
      		return activo;
	}
	
		void setActivo()
		{
			QMutexLocker lm(&mutex);
			activo = true;
        }

		std::pair<float, float> extraerCoordenadas(){
			
			std::pair<float, float> tar;
			QMutexLocker lm(&mutex);
			tar.first = x;
			tar.second = z;
			
			return tar;
		}
		
		bool haLlegado(float coordenadaX, float coordenadaZ){
			
			if (coordenadaX == x && coordenadaZ == z)
				return true;
			
			return false;
		}
		
	};
Target T;
};


#endif
