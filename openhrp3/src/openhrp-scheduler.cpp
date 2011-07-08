// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
// This program based on sample/example/scheduler/scedular.cpp

#include <hrpUtil/OnlineViewerUtil.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpCorba/DynamicsSimulator.hh>
#include <hrpCorba/Controller.hh>
#include <hrpUtil/Tvmet3d.h>
#include <fstream>
#include <stdlib.h>

#include <libxml/parser.h>
#include <libxml/xmlreader.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

#include <rospack/rospack.h>

using namespace std;
using namespace hrp;
using namespace OpenHRP;

template <typename X, typename X_ptr>
X_ptr checkCorbaServer(std::string n, CosNaming::NamingContext_var &cxt)
{
	CosNaming::Name ncName;
	ncName.length(1);
	ncName[0].id = CORBA::string_dup(n.c_str());
	ncName[0].kind = CORBA::string_dup("");
	X_ptr srv = NULL;
	try {
		srv = X::_narrow(cxt->resolve(ncName));
	} catch(const CosNaming::NamingContext::NotFound &exc) {
		std::cerr << n << " not found: ";
		switch(exc.why) {
		case CosNaming::NamingContext::missing_node:
			std::cerr << "Missing Node" << std::endl;
		case CosNaming::NamingContext::not_context:
			std::cerr << "Not Context" << std::endl;
			break;
		case CosNaming::NamingContext::not_object:
			std::cerr << "Not Object" << std::endl;
			break;
		}
		return (X_ptr)NULL;
	} catch(CosNaming::NamingContext::CannotProceed &exc) {
		std::cerr << "Resolve " << n << " CannotProceed" << std::endl;
	} catch(CosNaming::NamingContext::AlreadyBound &exc) {
		std::cerr << "Resolve " << n << " InvalidName" << std::endl;
	}
	return srv;
}

template <typename X, typename X_ptr>
void waitForServer (std::string n, CosNaming::NamingContext_var &cxt) {
	int waitForModelServer = 10;
	while ( --waitForModelServer > 0 ) {
		if ( checkCorbaServer <X, X_ptr> (n, cxt) != NULL ) {
			std::cerr << "Found " << n << "!!!" << std::endl;
			waitForModelServer = 0;
		} else {
			std::cerr << "Wait for " << n << "... " << waitForModelServer << std::endl;;
		}
		sleep(3);
	}
}

//
class CollisionPairItem {
public:
	string objectName1;
	string objectName2;
	string jointName1;
	string jointName2;
	double slidingFriction;
	double staticFriction;
	double cullingThresh;
};

class JointItem {
public:
	double angle;
	DynamicsSimulator::JointDriveMode mode;
	int NumOfAABB;
	Vector3 translation;
	Matrix33 rotation;
};
class ModelItem {
public:
	string url;
	BodyInfo_var body;
	map<string,JointItem> joint;
};

int main(int argc, char* argv[])
{
	string filename = string(argv[1]);

	// simulation
	double totalTime = 10.0;
	double timeStep = 0.001;
	double gravity = 9.8;  // default gravity acceleration [m/s^2]

	// model
	pair<string, ModelItem> Robot;
	map<string, ModelItem> Models;
	// collision
	vector<CollisionPairItem> Collisions;
	// controller
	double controlTimeStep = 0.002;
	string controllerName;
	//
	double logTimeStep = 0.050;

	/* Load XML document */
	xmlInitParser();
	xmlDocPtr doc = xmlParseFile(filename.c_str());
	if ( doc == NULL ) {
		std::cerr << "[" << argv[0] << "]Error : unable to parse file " << argv[1] << std::endl;
		exit (1);
	}

	/* Create xpath evaluation context */
	xmlXPathContextPtr xpathCtx = xmlXPathNewContext(doc);
	if ( xpathCtx == NULL ) {
		std::cerr << "Error : unable to create new XPath context" << std::endl;
		xmlFreeDoc(doc);
		return(-1);
	}

	/* Evaluate xpath expression */
	xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression(BAD_CAST "/grxui/mode/item", xpathCtx);
	if ( xmlXPathNodeSetIsEmpty(xpathObj->nodesetval) ) {
		std::cerr << "Error : unable to find <mode>" << std::endl;
	}

	int size;
	size = xpathObj->nodesetval->nodeNr;
	for (int i = 0; i < size; i++ ) {
		xmlNodePtr node = xpathObj->nodesetval->nodeTab[i];
		std::cerr << i << " class:" << xmlGetProp(node, (xmlChar *)"class") << std::endl;
		if ( xmlStrEqual( xmlGetProp(node, (xmlChar *)"class"), (xmlChar *)"com.generalrobotix.ui.item.GrxSimulationItem")  ) {
			xmlNodePtr cur_node = node->children;
			while ( cur_node ) {
				if ( cur_node->type == XML_ELEMENT_NODE ) {
					if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"integrate") ) {
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"viewsimulate") ) {
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"totalTime") ) {
						totalTime = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"timeStep") ) {
						timeStep = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"realTime") ) {
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"gravity") ) {
						gravity = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"method") ) {
					} else {
						std::cerr << "Unknown tag : " << cur_node->name << " "
								  << "name=" << xmlGetProp(cur_node, (xmlChar *)"name")
								  << "value=" << xmlGetProp(cur_node, (xmlChar *)"value") << std::endl;
					}
				}
				cur_node = cur_node->next;
			}
		} else if ( xmlStrEqual( xmlGetProp(node, (xmlChar *)"class"), (xmlChar *)"com.generalrobotix.ui.item.GrxModelItem")  ) {
			std::cerr << "GrxModelItem name:" << xmlGetProp(node, (xmlChar *)"name") << ", url:" << xmlGetProp(node, (xmlChar *)"url") << std::endl;
			string path = "file://";
			path += (char *)xmlGetProp(node, (xmlChar *)"url");
			if ( path.find("$(CURRENT_DIR)") != string::npos ) {
				path.replace(path.find("$(CURRENT_DIR)"),14, filename.substr(0, filename.find_last_of("/")));
			}
			if ( path.find("$(PROJECT_DIR)") != string::npos ) {
				rospack::ROSPack rp;
				try {
					rospack::Package *p = rp.get_pkg("openhrp3");
					path.replace(path.find("$(PROJECT_DIR)"),14, p->path+"/share/OpenHRP-3.1/sample/project");
				} catch (runtime_error &e) {
				}
			}
			ModelItem m;
			m.url = path;
			xmlNodePtr cur_node = node->children;
			bool isRobot = false;
			while ( cur_node ) {
				if ( cur_node->type == XML_ELEMENT_NODE ) {
					if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"isRobot") ) {
						if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"value"),(xmlChar *)"true")) {
							isRobot = true;
							controllerName = (char *)xmlGetProp(node, (xmlChar *)"name");
						}
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"controller") ) {
						controllerName = (char *)(xmlGetProp(cur_node, (xmlChar *)"value"));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"controlTime") ) {
						controlTimeStep = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					} else if ( string((char *)xmlGetProp(cur_node, (xmlChar *)"name")).rfind(".angle") != string::npos ) {
						string name = string((char *)xmlGetProp(cur_node, (xmlChar *)"name"));
						name.erase(name.rfind(".angle"));
						m.joint[name].angle = atof((char *)xmlGetProp(cur_node, (xmlChar *)"value"));
					} else if ( string((char *)xmlGetProp(cur_node, (xmlChar *)"name")).rfind(".mode") != string::npos ) {
						string name = string((char *)xmlGetProp(cur_node, (xmlChar *)"name"));
						name.erase(name.rfind(".mode"));
						m.joint[name].mode = (xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"value"), (xmlChar *)"HighGain"))?(DynamicsSimulator::HIGH_GAIN_MODE):(DynamicsSimulator::TORQUE_MODE);
					} else if ( string((char *)xmlGetProp(cur_node, (xmlChar *)"name")).rfind(".translation") != string::npos ) {
						string name = string((char *)xmlGetProp(cur_node, (xmlChar *)"name"));
						name.erase(name.rfind(".translation"));
						float x, y, z;
						sscanf(((char *)xmlGetProp(cur_node, (xmlChar *)"value")),"%f %f %f", &x, &y, &z);
						m.joint[name].translation[0] = x; m.joint[name].translation[1] = y; m.joint[name].translation[2] = z;
					} else if ( string((char *)xmlGetProp(cur_node, (xmlChar *)"name")).rfind(".rotation") != string::npos ) {
						string name = string((char *)xmlGetProp(cur_node, (xmlChar *)"name"));
						name.erase(name.rfind(".rotation"));
						float x, y, z, w;
						sscanf(((char *)xmlGetProp(cur_node, (xmlChar *)"value")),"%f %f %f %f", &x, &y, &z, &w);
						hrp::calcRodrigues(m.joint[name].rotation, Vector3(x, y, z), w);
					} else {
						std::cerr << "Unknown tag : " << cur_node->name << " "
								  << "name=" << xmlGetProp(cur_node, (xmlChar *)"name") << " "
								  << "value=" << xmlGetProp(cur_node, (xmlChar *)"value") << std::endl;
					}
				}
				cur_node = cur_node->next;
			}
			string n = string((char *)xmlGetProp(node, (xmlChar *)"name"));
			if ( isRobot ) {
				std::cerr << "Robot Model : " << n << " " << path << std::endl;
				Robot = pair<string, ModelItem>(n, m);
			} else {
				std::cerr << "Environment Model : " << n << " " << path << std::endl;
				Models.insert(pair<string, ModelItem>(n, m));
			}
		} else if ( xmlStrEqual( xmlGetProp(node, (xmlChar *)"class"), (xmlChar *)"com.generalrobotix.ui.item.GrxWorldStateItem") ) {
			CollisionPairItem c;
			xmlNodePtr cur_node = node->children;
			while ( cur_node ) {
				if ( cur_node->type == XML_ELEMENT_NODE ) {
					if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"logTimeStep") ) {
						logTimeStep = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					}
				}
				cur_node = cur_node->next;
			}
		} else if ( xmlStrEqual ( xmlGetProp(node, (xmlChar *)"class"), (xmlChar *)"com.generalrobotix.ui.item.GrxCollisionPairItem")  ) {
			CollisionPairItem c;
			xmlNodePtr cur_node = node->children;
			while ( cur_node ) {
				if ( cur_node->type == XML_ELEMENT_NODE ) {
					if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"objectName1") ) {
						c.objectName1 = (char *)(xmlGetProp(cur_node, (xmlChar *)"value"));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"objectName2") ) {
						c.objectName2 = (char *)(xmlGetProp(cur_node, (xmlChar *)"value"));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"jointName1") ) {
						c.jointName1 = (char *)(xmlGetProp(cur_node, (xmlChar *)"value"));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"jointName2") ) {
						c.jointName2 = (char *)(xmlGetProp(cur_node, (xmlChar *)"value"));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"slidingFriction") ) {
						c.slidingFriction = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"staticFriction") ) {
						c.staticFriction = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"cullingThresh") ) {
						c.cullingThresh = atof((char *)(xmlGetProp(cur_node, (xmlChar *)"value")));
					} else {
						std::cerr << "Unknown tag : " << cur_node->name << " "
								  << "name=" << xmlGetProp(cur_node, (xmlChar *)"name")
								  << "value=" << xmlGetProp(cur_node, (xmlChar *)"value") << std::endl;
					}
				}
				cur_node = cur_node->next;
			}
			Collisions.push_back(c);
		}
	}

	/* Cleanup Xpath Data */
	xmlXPathFreeObject(xpathObj);
	xmlXPathFreeContext(xpathCtx);

	/* free the document */
	xmlFreeDoc(doc);
	xmlCleanupParser();

	//================== CORBA init ===============================
	// initialize CORBA
	CORBA::ORB_var orb;
	orb = CORBA::ORB_init(argc, argv);

	// ROOT POA
	CORBA::Object_var poaObj = orb -> resolve_initial_references("RootPOA");
	PortableServer::POA_var rootPOA = PortableServer::POA::_narrow(poaObj);

	// get reference to POA manager
	PortableServer::POAManager_var manager = rootPOA -> the_POAManager();

	CosNaming::NamingContext_var cxt;
	{
		CORBA::Object_var	nS = orb->resolve_initial_references("NameService");
		cxt = CosNaming::NamingContext::_narrow(nS);
	}

	//================== Model Load ===============================
	waitForServer<ModelLoader, ModelLoader_var>("ModelLoader", cxt);
	cerr << "ModelLoader: Loading " << Robot.first << " from " << Robot.second.url << " ... ";
	Robot.second.body = loadBodyInfo(Robot.second.url.c_str(), orb);
	if(!Robot.second.body){
		cerr << "ModelLoader: " << Robot.first << " cannot be loaded" << endl;
		return 1;
	}
	cerr << Robot.second.body->name() << endl;
	for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
		it->second.body = loadBodyInfo(it->second.url.c_str(), orb);
		cerr << "ModelLoader: Loading " << it->first << " from " << it->second.url << " ... ";
		if(!it->second.body){
			cerr << "ModelLoader: " << it->first << " cannot be loaded" << endl;
			return 1;
		}
		cerr << it->second.body->name() << endl;
	}

	//==================== OnlineViewer (GrxUI) setup ===============
	cerr << "** OnlineViewer GrxUI) setup ** " << endl;
	waitForServer<OnlineViewer, OnlineViewer_var>("OnlineViewer", cxt);
	OnlineViewer_var olv = getOnlineViewer(argc, argv);
	if (CORBA::is_nil( olv )) {
		std::cerr << "OnlineViewer not found" << std::endl;
		return 1;
	}
	try {
		olv->load(Robot.second.body->name(), Robot.second.url.c_str());
		cerr << "OnlineViewer: Loading " << Robot.second.body->name()  << " from " << Robot.second.url << endl;
		for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
			olv->load(it->first.c_str(), it->second.url.c_str());
			cerr << "OnlineViewer: Loading " << it->first << " from " << it->second.url << endl;
		}
		olv->clearLog();
	} catch (CORBA::SystemException& ex) {
		cerr << "Failed to connect GrxUI." << endl;
		return 1;
	}


	//================= DynamicsSimulator setup ======================
	DynamicsSimulatorFactory_var dynamicsSimulatorFactory;
	dynamicsSimulatorFactory =
		checkCorbaServer <DynamicsSimulatorFactory, DynamicsSimulatorFactory_var> ("DynamicsSimulatorFactory", cxt);
	if (CORBA::is_nil(dynamicsSimulatorFactory)) {
		std::cerr << "DynamicsSimulatorFactory not found" << std::endl;
	}
	DynamicsSimulator_var dynamicsSimulator = dynamicsSimulatorFactory->create();

	cerr << "** Dynamics server setup ** " << endl;
	cerr << "Character  : " << Robot.second.body->name() << endl;
	dynamicsSimulator->registerCharacter(Robot.second.body->name(), Robot.second.body);
	for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
		cerr << "Character  : " << it->first << endl;
		dynamicsSimulator->registerCharacter(it->first.c_str(), it->second.body);
	}

	dynamicsSimulator->init(timeStep, DynamicsSimulator::RUNGE_KUTTA, DynamicsSimulator::ENABLE_SENSOR);
	DblSequence3 g;
	g.length(3);
	g[0] = 0.0;
	g[1] = 0.0;
	g[2] = gravity;
	dynamicsSimulator->setGVector(g);

	cerr << "Setup Character  Link Data : " << Robot.second.body->name() << endl;
	for ( map<string, JointItem>::iterator it = Robot.second.joint.begin(); it != Robot.second.joint.end(); it++ ) {
		DblSequence data;
		data.length(1);
		cerr << it->first << " : " << it->second.angle << ":" << ((it->second.mode==DynamicsSimulator::HIGH_GAIN_MODE)?"HIGH_GAIN_MODE":"TORQUE_MODE") << endl;
		cerr << "      t:"  << it->second.translation << endl;
		cerr << "      R:"  << it->second.rotation << endl;
		// mode
		data[0] = (it->second.mode==DynamicsSimulator::HIGH_GAIN_MODE)?1.0:0.0;
		dynamicsSimulator->setCharacterLinkData(Robot.second.body->name(), it->first.c_str(), DynamicsSimulator::POSITION_GIVEN, data);
		// joint angle
		data[0] = it->second.angle;
		dynamicsSimulator->setCharacterLinkData(Robot.second.body->name(), it->first.c_str(), DynamicsSimulator::JOINT_VALUE, data);
		// translation
		DblSequence trans;
		trans.length(12);
		for(int i=0; i<3; i++) trans[i] = it->second.translation[i];
		trans[2] += 0.5;
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++) trans[3+3*i+j] = it->second.rotation(i,j);
		}
		dynamicsSimulator->setCharacterLinkData(Robot.second.body->name(), it->first.c_str(), DynamicsSimulator::ABS_TRANSFORM, trans );
	}
	for ( map<string, ModelItem>::iterator mit = Models.begin(); mit != Models.end(); mit++ ) {
		cerr << "Character  : " << mit->first << ":" << mit->second.body->name() << endl;
		for ( map<string, JointItem>::iterator it = mit->second.joint.begin(); it != mit->second.joint.end(); it++ ) {
			DblSequence data;
			data.length(1);
			cerr << it->first << " : " << it->second.angle << ":" << ((it->second.mode==DynamicsSimulator::HIGH_GAIN_MODE)?"HIGH_GAIN_MODE":"TORQUE_MODE") << endl;
			cerr << "      t:"  << it->second.translation << endl;
			cerr << "      R:"  << it->second.rotation << endl;
			// mode
			data[0] = (it->second.mode==DynamicsSimulator::HIGH_GAIN_MODE)?1.0:0.0;
			dynamicsSimulator->setCharacterLinkData(mit->second.body->name(), it->first.c_str(), DynamicsSimulator::POSITION_GIVEN, data);
			// joint angle
			data[0] = it->second.angle;
			dynamicsSimulator->setCharacterLinkData(mit->second.body->name(), it->first.c_str(), DynamicsSimulator::JOINT_VALUE, data);
			// translation
			DblSequence trans;
			trans.length(12);
			for(int i=0; i<3; i++) trans[i] = it->second.translation[i];
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++) trans[3+3*i+j] = it->second.rotation(i,j);
			}
			dynamicsSimulator->setCharacterLinkData(mit->first.c_str(), it->first.c_str(), DynamicsSimulator::ABS_TRANSFORM, trans );
		}
	}
	dynamicsSimulator->calcWorldForwardKinematics();


	//================= CollisionDetector setup ======================

	cerr << "** Collision server setup ** " << endl;
	DblSequence6 K, C;    // spring-damper parameters are not used now
	K.length(0);
	C.length(0);
#if 0
	for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
		cerr << "Collision : " << it->second.body->name() << " <> " << Robot.second.body->name() << endl;
		dynamicsSimulator->registerCollisionCheckPair(it->second.body->name(), "", Robot.second.body->name(), "",
													  0.5, 0.5, K, C, 0.01);
	}
#endif
	for(vector<CollisionPairItem>::iterator it = Collisions.begin(); it != Collisions.end(); it++ ) {
		cerr << "Collision : " << ((Models.find(it->objectName1)!=Models.end())?Models[it->objectName1].body->name():Robot.second.body->name()) << "#" << it->jointName1 << " <> " << ((Models.find(it->objectName2)!=Models.end())?Models[it->objectName2].body->name():Robot.second.body->name()) << "#" << it->jointName2 << endl;
		//cerr << ((Models.find(it->objectName1)!=Models.end())?Models[it->objectName1].body->name():Robot.second.body->name()) << endl;
		//cerr << ((Models.find(it->objectName2)!=Models.end())?Models[it->objectName2].body->name():Robot.second.body->name()) << endl;
		dynamicsSimulator->registerCollisionCheckPair(((Models.find(it->objectName1)!=Models.end())?Models[it->objectName1].body->name():Robot.second.body->name()),it->jointName1.c_str(),
													  ((Models.find(it->objectName2)!=Models.end())?Models[it->objectName2].body->name():Robot.second.body->name()),it->jointName2.c_str(),
													  it->staticFriction,it->slidingFriction,K,C,it->cullingThresh);

	}
	dynamicsSimulator->initSimulation();

	// ==================  Controller setup ==========================
	waitForServer<Controller, Controller_var>(controllerName, cxt);
	cerr << "** Controller server(" << controllerName << ") setup ** " << endl;
	cerr << Robot.second.body->name() << endl;
	Controller_var controller;
	controller = checkCorbaServer <Controller, Controller_var> (controllerName, cxt);

	if (CORBA::is_nil(controller)) {
		std::cerr << "Controller not found" << std::endl;
	}

	controller->setModelName(Robot.second.body->name());
	controller->setDynamicsSimulator(dynamicsSimulator);
	controller->initialize();
	controller->setTimeStep(controlTimeStep);
	controller->start();
	cerr << "** Controller server start ** " << endl;
	cerr << "   timeStep : " << timeStep << endl;
	cerr << "   controleTimeStep : " << controlTimeStep << endl;

	// ==================  main loop   ======================
	WorldState_var state;
	int i=0;
	int j = 0;
	double time=0.0;
	double controlTime=0.0;

	while ( 1 ) {
		bool control=false;
		if(controlTime <= time){
			control=true;
			j++;
		}

		if(control)
			controller->input();

		i++;
		time = timeStep * i;
		controlTime = controlTimeStep * j;


		if(control)
			controller->control();

		// ================== simulate one step ==============
		dynamicsSimulator->stepSimulation();
		// ================== viewer update ====================

		try {
			if ((i % (int)(logTimeStep/controlTimeStep))==0) {
				dynamicsSimulator -> getWorldState( state );
				olv->update( state );
			}
		} catch (CORBA::SystemException& ex) {
			return 1;
		}

		if(control)
			controller->output();

		if( time > totalTime ) break;

	}

	dynamicsSimulator->destroy();

	return 0;
}
