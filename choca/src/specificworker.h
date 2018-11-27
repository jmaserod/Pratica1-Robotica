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
#include "grid.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <iostream>
#include <fstream>
#include <queue>

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
	//void goToTarget(const TLaserData& lData);
    //void bug( TLaserData &lData,  TBaseState& robotState);
    //bool obstacle(TLaserData tLaser);
   // bool targetAtSight(const TLaserData lData);
    void saveToFile();
    void readFromFile();




private:
	std::shared_ptr<InnerModel> innerModel;

	int robotX, robotZ;

    enum class State {INIT,GOTO,BUG};

	struct Target{
		QMutex mutex;
		QVec posicion=QVec::zeros(3);;
		//float x, z;
		bool activo = true;
		Target(){};


		bool insertarCoordenadas(float coordenadaX, float coordenadaZ){

		    QMutexLocker lm(&mutex);
			//x = coordenadaX;
			//z = coordenadaZ;
			posicion.resize(3);
			posicion[0]=coordenadaX;
			posicion[1]=0;
			posicion[2]=coordenadaZ;
			activo = false;
			return true;
		}

		bool isActivo(){

      		return activo;
	}

		void setActivo(bool act)
		{
			QMutexLocker lm(&mutex);
			activo = act;
}

		QVec extraerCoordenadas(){
            qDebug()<< "Estoy aquiiiiiiiiiiii";

			QMutexLocker l(&mutex);
			qDebug()<<posicion[0];
			qDebug()<<posicion[2];

			return posicion;
		}

		bool haLlegado(float coordenadaX, float coordenadaZ){

			if (coordenadaX == posicion[0] && coordenadaZ == posicion[2])
				return true;

			return false;
		}

	};
       QGraphicsScene scene;
       QGraphicsView view;
       void draw();
       QGraphicsRectItem *robot;
       QGraphicsEllipseItem *noserobot;
       std::string fileName = "map.txt";
       const int tilesize = 70;
	   Target T;
	   QVec target;
	   QLine2D linea;
	   State state = State::INIT;
	   float distanciaAnterior;
	   std::atomic<bool> targetReady = false;
       std::atomic<bool> planReady = false;
       QVec currentPoint;
       std::list<QVec> path;
       std::vector<QGraphicsEllipseItem *> greenPath;

       void updateVisitedCells(int x, int z);
       void updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
       void checkTransform(const RoboCompGenericBase::TBaseState &bState);

		/// Grid
        		struct TCell
        		{
        			uint id;
        			bool free;
        			bool visited;
        			QGraphicsRectItem* rect;
        			float cost = 1;

        			// method to save the value
        			void save(std::ostream &os) const {	os << free << " " << visited; };
        			void read(std::istream &is) {	is >> free >> visited ;};
        		};

        		using TDim = Grid<TCell>::Dimensions;
        		Grid<TCell> grid;




};


#endif
