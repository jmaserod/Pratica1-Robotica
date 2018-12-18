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
	std::cout << std::boolalpha;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	qDebug() << __FILE__ ;

	// Scene
	scene.setSceneRect(-2500, -2500, 5000, 5000);
	view.setScene(&scene);
	view.scale(1, -1);
	view.setParent(scrollArea);
	//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );

	grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{0, true, false, nullptr, 0.} );

	for(auto &[key, value] : grid)
	{
		auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
		tile->setPos(key.x,key.z);
		value.rect = tile;
	}

	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
	noserobot->setBrush(Qt::magenta);

	//qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;

	connect(buttonSave, SIGNAL(clicked()), this, SLOT(saveToFile()));
	connect(buttonRead, SIGNAL(clicked()), this, SLOT(readFromFile()));


	timer.start();
	// AutoLoad Map
	readFromFile();
	qDebug() << "CELIA" ;


	return true;
}

void SpecificWorker::compute()
{
	static RoboCompGenericBase::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
    innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

     	try
     	{

            		//draw robot
            		robot->setPos(bState.x, bState.z);
            		robot->setRotation(-180.*bState.alpha/M_PI);

            		auto relative =  (innerModel->transform("base", QVec::vec3(currentPoint.x(), 0, currentPoint.z()), "world"));
                    float angle = atan2(relative.x(), relative.z());
                    float mod = relative.norm2();

            		//updateVisitedCells(bState.x, bState.z);
            		updateOccupiedCells(bState, ldata);

            		if(targetReady)
            		{
            			if(planReady)
            			{
            				if(path.empty())
            				{
            					qDebug() << "Arrived to target";
            					differentialrobot_proxy->setSpeedBase(0,0);

            					targetReady = false;
            				} //GOTO
            				else if(mod < 50)
                            {
                               qDebug() << "Picking new point";
                               currentPoint = path.front();
                               path.pop_front();
            				}
            				else if(abs(angle) > 0.2)
                            {
                                differentialrobot_proxy->setSpeedBase(0, angle);
                             } else {
                                    differentialrobot_proxy->setSpeedBase(400, 0.3 * angle);
                             }
            			}
            			else
            			{
            			//BUSCAR (DIJSKTRA)
            				qDebug() << bState.x << bState.z << target.x() << target.z() ;
            				path = grid.getOptimalPath(QVec::vec3(bState.x,0,bState.z), target);
            				for(auto &p: path)
            					greenPath.push_back(scene.addEllipse(p.x(),p.z(), 100, 100, QPen(Qt::green), QBrush(Qt::green)));
            				planReady = true;
            			}


                    	}

                    }

     	catch(const Ice::Exception &e)
    	{	std::cout  << e << std::endl; }

    	//Resize world widget if necessary, and render the world
    	if (view.size() != scrollArea->size())
    			view.setFixedSize(scrollArea->width(), scrollArea->height());
    	draw();

    }

void SpecificWorker::saveToFile()
{
	grid.saveToFile(fileName);
}

void SpecificWorker::readFromFile()
{
	std::ifstream myfile;
	myfile.open(fileName, std::ifstream::in);
    	qDebug() << "CELIA" ;

	if(!myfile.fail())
	{
		//grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
		for( auto &[k,v] : grid)
			delete v.rect;
		grid.clear();
		Grid<TCell>::Key key; TCell value;
		myfile >> key >> value;
		int k=0;
		while(!myfile.eof())
		{
			auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));;
			tile->setPos(key.x,key.z);
			value.rect = tile;
			value.id = k++;
			value.cost = 1;
			grid.insert<TCell>(key,value);
			myfile >> key >> value;
		}
		myfile.close();
		robot->setZValue(1);
		std::cout << grid.size() << " elements read to grid " << fileName << std::endl;
	}
	else
		throw std::runtime_error("Cannot open file");
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	auto *n = innerModel->getNode<InnerModelLaser>("laser");
	for(auto l: ldata)
	{
		auto r = n->laserTo("world", l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied
		auto [valid, cell] = grid.getCell(r.x(), r.z());
		if(valid)
			cell.free = false;
	}
}

void SpecificWorker::checkTransform(const RoboCompGenericBase::TBaseState &bState)
{
	auto r = innerModel->transform("base", target, "world");		// using InnerModel

	Rot2D rot(bState.alpha);																		// create a 2D clockwise rotation matrix
	QVec t = QVec::vec2(bState.x, bState.z);									  // create a 2D vector for robot translation
	QVec t2 = QVec::vec2(target.x(), target.z());								// create a 2D vector from the 3D target
	QVec q = rot.transpose() * ( t2 - t);												// multiply R_t * (y - T)
	qDebug() << target << r << q;
}

void SpecificWorker::updateVisitedCells(int x, int z)
{
	static unsigned int cont = 0;
	auto [valid, cell] = grid.getCell(x, z);
	if(valid)
	{
		auto &occupied = cell.visited;
		if(occupied)
		{
			occupied = false;
			cont++;
		}
		float percentOccupacy = 100. * cont / grid.size();
	}
}

void SpecificWorker::draw()
{
	for(auto &[key, value] : grid)
	{
// 		if(value.visited == false)
// 			value.rect->setBrush(Qt::lightGray);
		if(value.free == false)
			value.rect->setBrush(Qt::darkRed);
	}
	view.show();
}

/////////////// PATH PLANNING /////7


void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha)
{
//implementCODE

     T.insertarCoordenadas(x,y);
     T.setActivo(true);
}

void SpecificWorker::turn(const float speed)
{
//implementCODE

    differentialrobot_proxy->setSpeedBase(speed,0);

}

bool SpecificWorker::atTarget()
{
//implementCODE

    return !T.isActivo();


}

void SpecificWorker::stop()
{
//implementCODE

    differentialrobot_proxy->stopBase();

}

/////////////////////////////////////////////////////////77
/////////
//////////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick &myPick)

{
	target[0] = myPick.x;
    	target[2] = myPick.z;
    	target[1] = 0;
    	qDebug() << __FILE__ << __FUNCTION__ << myPick.x << myPick.z ;
    	targetReady = true;
    	planReady = false;
    	for(auto gp: greenPath)
    		delete gp;
    greenPath.clear();
}

void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{


}
void SpecificWorker::newAprilTag(const tagsList &tags)
{
    for(auto &marca : tags)
        qDebug()<< marca.id;
//currentTags.set(tags);


}