// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
// This program based on sample/example/scheduler/scedular.cpp

#include <hrpUtil/OnlineViewerUtil.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpCorba/DynamicsSimulator.hh>
#include <hrpCorba/ViewSimulator.hh>
#include <hrpCorba/Controller.hh>
#include <hrpCorba/ClockGenerator.hh>
#include <hrpUtil/Tvmet3d.h>
#include <fstream>
#include <stdlib.h>
#include <getopt.h>

#include <libxml/parser.h>
#include <libxml/xmlreader.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

#include <rospack/rospack.h>


using namespace std;
using namespace hrp;
using namespace OpenHRP;

class ExecutionContext {
public:
	OpenRTM::ExtTrigExecutionContextService_ptr ec_;
	double period_;
	double nextExecutionTime_;

    ExecutionContext(OpenRTM::ExtTrigExecutionContextService_ptr ec, ::CORBA::Double period)
	{
		ec_ = ::OpenRTM::ExtTrigExecutionContextService::_duplicate(ec);
		period_ = period;
		reset();
	}

    ~ExecutionContext() {}

	bool execute(double t){
		if (t >= nextExecutionTime_){
			try{
				ec_->tick();
				nextExecutionTime_ += period_;
			}catch( ... ){
				return false;
			}
		}
		return true;
	}

	void reset(){
		nextExecutionTime_ = 0;
	}

};
class ClockGenerator_impl : public POA_OpenHRP::ClockGenerator
{
    CORBA::ORB_var orb;
    PortableServer::POA_var poa;
	vector<ExecutionContext> ecs;

public:
    ClockGenerator_impl(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa)
		:
		orb(CORBA::ORB::_duplicate(orb)),
		poa(PortableServer::POA::_duplicate(poa))
	{
	}
    virtual ~ClockGenerator_impl() {};

    void subscribe(OpenRTM::ExtTrigExecutionContextService_ptr ec, ::CORBA::Double period)
	{
        ecs.push_back(ExecutionContext(ec, period));
	}
    void unsubscribe(OpenRTM::ExtTrigExecutionContextService_ptr ec)
	{
        for (vector<ExecutionContext>::iterator it = ecs.begin(); it!=ecs.end(); it++) {
            if ( ec == it->ec_ ) {
                ecs.erase(it);
                return;
            }
        }
	}

    void updateExecutionContext(double simTime)
    {
        for (vector<ExecutionContext>::iterator it = ecs.begin(); it!=ecs.end(); it++) {
			try {
				it->execute(simTime);
			} catch  (...) {
			}
        }
    }
	void shutdown() {};
};

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
		std::cerr << "[openhrp-scheduler] " << n << " not found: ";
		switch(exc.why) {
		case CosNaming::NamingContext::missing_node:
			std::cerr << "Missing Node" << std::endl;
			break;
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
X_ptr waitAndCheckCorbaServer (std::string n, CosNaming::NamingContext_var &cxt, int maxWaitForModelServer=10) {
	int waitForModelServer = maxWaitForModelServer;
	X_ptr srv = NULL;
	do {
		if ( (srv = checkCorbaServer <X, X_ptr> (n, cxt)) != NULL ) {
			std::cerr << "[openhrp-scheduler] Found " << n << std::endl;
			waitForModelServer = 0;
		} else {
			std::cerr << "[openhrp-scheduler] Wait for " << n << "... " << waitForModelServer << std::endl;;
		}
		sleep(5);
	} while ( --waitForModelServer >= 0 );
	return srv;
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

	CollisionPairItem() {
		jointName1 = "";
		jointName2 = "";
		slidingFriction = 0.5;
		staticFriction = 0.5;
		cullingThresh = 0.01;
	}
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


class OpenHRPScheduler {
public:
	// model
	pair<string, ModelItem> Robot;
	map<string, ModelItem> Models;
	// collision
	vector<CollisionPairItem> Collisions;

	// params
	double gravity;
	double totalTime;
	double timeStep;
	double controlTimeStep;
	double logTimeStep;

	string controllerName;

	// CORBA
	PortableServer::POA_var rootPOA;
	PortableServer::POAManager_var manager;
	CORBA::ORB_var orb;
	CosNaming::NamingContext_var cxt;

	// server
	DynamicsSimulator_var dynamicsSimulator;
	Controller_var controller;
	ClockGenerator_impl* clockGeneratorImpl;
	OnlineViewer_var olv;
	ViewSimulator_var vs;

    OpenHRPScheduler()
	{
		gravity = 0.9;
		totalTime = 10.0;
		timeStep = 0.05;
		controlTimeStep = 0.005;
		logTimeStep = 0.100;
		dynamicsSimulator = NULL;
		olv = NULL;
		vs = NULL;
	}

    ~OpenHRPScheduler()
	{
		if (dynamicsSimulator) dynamicsSimulator->destroy();
	}

	double setGravity(double d) { gravity = d; return gravity; }
	double setTotalTime(double d) { totalTime = d; return totalTime; }
	double setTimeStep(double d) { timeStep = d; return timeStep; }
	double getGravity() { return gravity; }
	double getTotalTime() { return totalTime; }
	double getTimeStep() { return timeStep; }
	double getControlTimeStep() { return controlTimeStep; }
	double getLogTimeStep() { return logTimeStep; }

	void setupFromXML(string filename) {
		/* Load XML document */
		xmlInitParser();
		xmlDocPtr doc = xmlParseFile(filename.c_str());
		if ( doc == NULL ) {
			std::cerr << "[openhrp-scheduler] Error : unable to parse file " << filename << std::endl;
			exit (1);
		}

		/* Create xpath evaluation context */
		xmlXPathContextPtr xpathCtx = xmlXPathNewContext(doc);
		if ( xpathCtx == NULL ) {
			std::cerr << "[openhrp-scheduler] Error : unable to create new XPath context" << std::endl;
			xmlFreeDoc(doc);
			exit(1);
		}

		/* Evaluate xpath expression */
		xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression(BAD_CAST "/grxui/mode/item", xpathCtx);
		if ( xmlXPathNodeSetIsEmpty(xpathObj->nodesetval) ) {
			std::cerr << "[openhrp-scheduler] Error : unable to find <mode>" << std::endl;
		}

		int size;
		size = xpathObj->nodesetval->nodeNr;
		for (int i = 0; i < size; i++ ) {
			xmlNodePtr node = xpathObj->nodesetval->nodeTab[i];
			std::cerr << "[openhrp-scheduler] " << i << " class:" << xmlGetProp(node, (xmlChar *)"class") << std::endl;
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
							std::cerr << "[openhrp-scheduler] "
									  << "Unknown tag : " << cur_node->name << " "
									  << "name=" << xmlGetProp(cur_node, (xmlChar *)"name")
									  << "value=" << xmlGetProp(cur_node, (xmlChar *)"value") << std::endl;
						}
					}
					cur_node = cur_node->next;
				}
			} else if ( xmlStrEqual( xmlGetProp(node, (xmlChar *)"class"), (xmlChar *)"com.generalrobotix.ui.item.GrxModelItem")  ) {
				std::cerr << "[openhrp-scheduler] GrxModelItem name:" << xmlGetProp(node, (xmlChar *)"name") << ", url:" << xmlGetProp(node, (xmlChar *)"url") << std::endl;
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
							}
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
							std::cerr << "[openhrp-scheduler] "
									  << "Unknown tag : " << cur_node->name << " "
									  << "name=" << xmlGetProp(cur_node, (xmlChar *)"name") << " "
									  << "value=" << xmlGetProp(cur_node, (xmlChar *)"value") << std::endl;
						}
					}
					cur_node = cur_node->next;
				}
				string n = string((char *)xmlGetProp(node, (xmlChar *)"name"));
				if ( isRobot ) {
					std::cerr << "[openhrp-scheduler] Robot Model : " << n << " " << path << std::endl;
					Robot = pair<string, ModelItem>(n, m);
				} else {
					std::cerr << "[openhrp-scheduler] Environment Model : " << n << " " << path << std::endl;
					Models.insert(pair<string, ModelItem>(n, m));
				}
			} else if ( xmlStrEqual( xmlGetProp(node, (xmlChar *)"class"), (xmlChar *)"com.generalrobotix.ui.item.GrxWorldStateItem") ) {
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
							std::cerr << "[openhrp-scheduler] "
									  << "Unknown tag : " << cur_node->name << " "
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
	}

	void setupFromRobotModel(string filename) {
		BodyInfo_var body;
		{
			ModelItem m;
			m.url = string("file://")+filename;
			cerr << "[openhrp-scheduler] ModelLoader: Loading " <<filename << " ... ";
			waitAndCheckCorbaServer<ModelLoader, ModelLoader_var>("ModelLoader", cxt);
			cerr << "[openhrp-scheduler] ModelLoader: Loading " << m.url << " ... " << endl;
			body = loadBodyInfo(m.url.c_str(), orb);
			if(!body){
				cerr << "[openhrp-scheduler] ModelLoader: " << m.url << " cannot be loaded" << endl;
				exit(1);
			}
			LinkInfoSequence_var links = body->links();
			for (unsigned int i = 0; i < links->length(); i++) {
				LinkInfo link = links[i];
				cerr << "[openhrp-scheduler] " << i << " " << link.name << "(parnet " << link.parentIndex << ")" << endl;
				if ( link.parentIndex == -1 ) {
					m.joint[string(link.name)].mode = DynamicsSimulator::TORQUE_MODE;
					m.joint[string(link.name)].translation[0] = 0; m.joint[string(link.name)].translation[1] = 0; m.joint[string(link.name)].translation[2] = 1.0;
					hrp::calcRodrigues(m.joint[string(link.name)].rotation, Vector3(0, 0, 1), 0);
				} else {
					m.joint[string(link.name)].mode = DynamicsSimulator::HIGH_GAIN_MODE;
				}
				m.joint[string(link.name)].angle = 0;
			}
			string n = body->name();
			Robot = pair<string, ModelItem>(n, m);
		}
		{
			ModelItem m;
			rospack::ROSPack rp;
			rospack::Package *p = rp.get_pkg("openhrp3");
			m.url = string("file://")+p->path+"/share/OpenHRP-3.1/sample/model/floor.wrl";
			m.joint["WAIST"].mode = DynamicsSimulator::HIGH_GAIN_MODE;
			m.joint["WAIST"].translation[0] = 0; m.joint["WAIST"].translation[1] = 0; m.joint["WAIST"].translation[2] = -0.1;
			hrp::calcRodrigues(m.joint["WAIST"].rotation, Vector3(0, 0, 1), 0);
			string n = string("floor");
			Models.insert(pair<string,ModelItem>(n, m));
		}
		{
			CollisionPairItem c;
			c.objectName1 = "floor";
			c.objectName2 = Robot.first;
			Collisions.push_back(c);
		}
	}

	void initCORBA(int argc, char* argv[]) {
		// initialize CORBA
		orb = CORBA::ORB_init(argc, argv);

		// ROOT POA
		CORBA::Object_var poaObj = orb -> resolve_initial_references("RootPOA");
		rootPOA = PortableServer::POA::_narrow(poaObj);

		// get reference to POA manager
		manager = rootPOA -> the_POAManager();

		CORBA::Object_var	nS = orb->resolve_initial_references("NameService");
		cxt = CosNaming::NamingContext::_narrow(nS);
	}

	void loadModel() {
		waitAndCheckCorbaServer<ModelLoader, ModelLoader_var>("ModelLoader", cxt);
		cerr << "[openhrp-scheduler] ModelLoader: Loading " << Robot.first << " from " << Robot.second.url << " ... ";
		Robot.second.body = loadBodyInfo(Robot.second.url.c_str(), orb);
		if(!Robot.second.body){
			cerr << endl << "[openhrp-scheduler] ModelLoader: " << Robot.first << " cannot be loaded" << endl;
			exit(1);
		}
		cerr << Robot.second.body->name() << endl;
		controllerName = Robot.second.body->name();
		for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
			it->second.body = loadBodyInfo(it->second.url.c_str(), orb);
			cerr << "[openhrp-scheduler] ModelLoader: Loading " << it->first << " from " << it->second.url << " ... ";
			if(!it->second.body){
				cerr << "[openhrp-scheduler] ModelLoader: " << it->first << " cannot be loaded" << endl;
				exit(1);
			}
			cerr << "[openhrp-scheduler] " << it->second.body->name() << endl;
		}
	}

	void setupViewer(){
		cerr << "[openhrp-scheduler] ** OnlineViewer (GrxUI) setup ** " << endl;
		olv = waitAndCheckCorbaServer<OnlineViewer, OnlineViewer_var>("OnlineViewer", cxt, 5);
		//getOnlineViewer(argc, argv);
		if (CORBA::is_nil( olv )) {
			std::cerr << "[openhrp-scheduler] OnlineViewer not found" << std::endl;
			return ;
		}
		try {
			olv->load(Robot.second.body->name(), Robot.second.url.c_str());
			cerr << "[openhrp-scheduler] OnlineViewer: Loading " << Robot.second.body->name()  << " from " << Robot.second.url << endl;
			for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
				olv->load(it->first.c_str(), it->second.url.c_str());
				cerr << "OnlineViewer: Loading " << it->first << " from " << it->second.url << endl;
			}
			olv->clearLog();
		} catch (CORBA::SystemException& ex) {
			cerr << "Failed to connect GrxUI." << endl;
			exit(1);
		}

		cerr << "[openhrp-scheduler] ** ViewSimulator (GrxUI) setup ** " << endl;
		vs = waitAndCheckCorbaServer<ViewSimulator, ViewSimulator_var>("ViewSimulator", cxt, 5);
		if (CORBA::is_nil( vs )) {
			std::cerr << "ViewSimulator not found" << std::endl;
			return ;
		}
		if (olv) return;
		try {
			vs->registerCharacter(Robot.second.body->name(), Robot.second.body);
			cerr << "[openhrp-scheduler] ViewSimulator : registerCharacter " << Robot.second.body->name()  << " from " << Robot.second.url << endl;
			for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
				vs->registerCharacter(it->first.c_str(), it->second.body);
				cerr << "ViewSimulator: registerCharacter " << it->first << " from " << it->second.url << endl;
			}
		} catch (CORBA::SystemException& ex) {
			cerr << "Failed to connect ViewSimulator." << endl;
			exit(1);
		}
	}

	void setupSimulator() {
		DynamicsSimulatorFactory_var dynamicsSimulatorFactory;
		dynamicsSimulatorFactory =
			waitAndCheckCorbaServer <DynamicsSimulatorFactory, DynamicsSimulatorFactory_var> ("DynamicsSimulatorFactory", cxt);
		if (CORBA::is_nil(dynamicsSimulatorFactory)) {
			std::cerr << "DynamicsSimulatorFactory not found" << std::endl;
		}
		dynamicsSimulator = dynamicsSimulatorFactory->create();

		cerr << "[openhrp-scheduler] ** Dynamics server setup ** " << endl;
		cerr << "[openhrp-scheduler] Character  : " << Robot.second.body->name() << endl;
		dynamicsSimulator->registerCharacter(Robot.second.body->name(), Robot.second.body);
		for ( map<string, ModelItem>::iterator it = Models.begin(); it != Models.end(); it++ ) {
			cerr << "[openhrp-scheduler] Character  : " << it->first << endl;
			dynamicsSimulator->registerCharacter(it->first.c_str(), it->second.body);
		}

		dynamicsSimulator->init(timeStep, DynamicsSimulator::RUNGE_KUTTA, DynamicsSimulator::ENABLE_SENSOR);
		DblSequence3 g;
		g.length(3);
		g[0] = 0.0;
		g[1] = 0.0;
		g[2] = gravity;
		dynamicsSimulator->setGVector(g);

		cerr << "[openhrp-scheduler] Setup Character  Link Data : " << Robot.second.body->name() << endl;
		for ( map<string, JointItem>::iterator it = Robot.second.joint.begin(); it != Robot.second.joint.end(); it++ ) {
			DblSequence data;
			data.length(1);
			cerr << "[openhrp-scheduler] " << it->first << " : " << it->second.angle << ":" << ((it->second.mode==DynamicsSimulator::HIGH_GAIN_MODE)?"HIGH_GAIN_MODE":"TORQUE_MODE") << endl;
			//cerr << "      t:"  << it->second.translation << endl;
			//cerr << "      R:"  << it->second.rotation << endl;
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
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++) trans[3+3*i+j] = it->second.rotation(i,j);
			}
			dynamicsSimulator->setCharacterLinkData(Robot.second.body->name(), it->first.c_str(), DynamicsSimulator::ABS_TRANSFORM, trans );
		}
		for ( map<string, ModelItem>::iterator mit = Models.begin(); mit != Models.end(); mit++ ) {
			cerr << "[openhrp-scheduler] Character  : " << mit->first << ":" << mit->second.body->name() << endl;
			for ( map<string, JointItem>::iterator it = mit->second.joint.begin(); it != mit->second.joint.end(); it++ ) {
				DblSequence data;
				data.length(1);
				cerr << "[openhrp-scheduler] " << it->first << " : " << it->second.angle << ":" << ((it->second.mode==DynamicsSimulator::HIGH_GAIN_MODE)?"HIGH_GAIN_MODE":"TORQUE_MODE") << endl;
				//cerr << "      t:"  << it->second.translation << endl;
				//cerr << "      R:"  << it->second.rotation << endl;
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



		cerr << "[openhrp-scheduler] ** Collision server setup ** " << endl;
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
			string objectName1 = (Models.find(it->objectName1)!=Models.end())?it->objectName1:Robot.second.body->name();
			string objectName2 = (Models.find(it->objectName2)!=Models.end())?it->objectName2:Robot.second.body->name();
			cerr << "[openhrp-scheduler] Collision : " << objectName1 << "#" << it->jointName1 << " <> " << objectName2 << "#" << it->jointName2 << " Friction static : "<< it->staticFriction << " slideing : " << it->slidingFriction << " culling : " << it->cullingThresh << " " << endl;
			dynamicsSimulator->registerCollisionCheckPair(objectName1.c_str(), it->jointName1.c_str(),
														  objectName2.c_str(), it->jointName2.c_str(),
														  it->staticFriction,it->slidingFriction,K,C,it->cullingThresh);

		}
		dynamicsSimulator->initSimulation();
	}

	void setupController() {
		cerr << "[openhrp-scheduler] ** Controller server(" << controllerName << "/" << Robot.second.body->name() << ") setup ** " << endl;
		controller = waitAndCheckCorbaServer <Controller, Controller_var> (controllerName, cxt);

		if (CORBA::is_nil(controller)) {
			std::cerr << "[openhrp-scheduler] Controller not found" << std::endl;
			exit(1);
		}

		controller->setModelName(Robot.second.body->name());
		controller->setDynamicsSimulator(dynamicsSimulator);
		if ( vs ) controller->setViewSimulator(vs);
		controller->initialize();
		controller->setTimeStep(controlTimeStep);
		controller->start();
		cerr << "[openhrp-scheduler] ** Controller server start ** " << endl;
		cerr << "[openhrp-scheduler]    timeStep : " << timeStep << endl;
		cerr << "[openhrp-scheduler]    controleTimeStep : " << controlTimeStep << endl;
	}

	void setupClockGenerator() {
		// ================  ClockGenerator   ====================
		// set ClockGenerator server
		clockGeneratorImpl = new ClockGenerator_impl(orb, rootPOA);
		rootPOA->activate_object(clockGeneratorImpl);
		ClockGenerator_var clockGenerator = clockGeneratorImpl->_this();
		clockGeneratorImpl->_remove_ref();

		CosNaming::Name name;
		name.length(1);
		name[0].id = CORBA::string_dup("ClockGenerator");
		name[0].kind = CORBA::string_dup("");
		cxt->rebind(name, clockGenerator);

		manager->activate();
	}

	//
	void controllerInput() {
		controller->input();
	}
	void controllerControl(double time) {
		controller->control();
		clockGeneratorImpl->updateExecutionContext(time);
	}
	void controllerOutput() {
		controller->output();
	}
	void stepSimulation(bool use_dynamics) {
		if (use_dynamics) {
			dynamicsSimulator->stepSimulation();
		} else {
			dynamicsSimulator->calcWorldForwardKinematics();
		}
	}

	void viewerUpdate() {
		WorldState_var state;
		dynamicsSimulator -> getWorldState( state );
		if ( !vs && olv ) olv->update( state );
		if ( vs ) vs->updateScene( state );
	}
};

int main(int argc, char* argv[])
{
	// simulation
	bool use_dynamics = true;
	double gravity   = 9.8;
	double totalTime = -1;
	double timeStep  = 0.005;

	struct option lngopt[] = {
		{"nosim",     0, NULL, 0},
		{"totaltime", 1, NULL, 0},
		{"timestep",  1, NULL, 0},
		{0, 0, 0, 0}
	};
	int opt;
	int option_index = 0;
	while((opt = getopt_long(argc, argv, "", lngopt, &option_index)) != -1){
		if ( opt == 0 ) {
			cerr << "[openhrp-scheduler] opt = " << opt << " / " << option_index << endl;
			switch (option_index) {
			case 0:
				use_dynamics = false;
				break;
			case 1:
				totalTime = atof(optarg);
				break;
			case 2:
				timeStep = atof(optarg);
				break;
			default:
				cerr << "[openhrp-sceduler] unknwon option " << lngopt[option_index].name << endl;
				exit(1);
			}
		} else {
		}
	}
	string filename = string(argv[optind]);

	// controller
	OpenHRPScheduler scheduler;
	scheduler.setGravity(gravity);
	scheduler.setTotalTime(totalTime);
	scheduler.setTimeStep(timeStep);

	//================== CORBA init ===============================
	scheduler.initCORBA(argc, argv);

	/* Load XML document */
	if ( filename.substr(filename.find_last_of(".")) == ".wrl" ||
		 filename.substr(filename.find_last_of(".")) == ".dae" ) {
		scheduler.setupFromRobotModel(filename);
	} else {
		scheduler.setupFromXML(filename);
	}

	//================== Model Load ===============================
	scheduler.loadModel();

	//==================== OnlineViewer (GrxUI) setup ===============
	scheduler.setupViewer();

	//================= DynamicsSimulator setup ======================
	//================= CollisionDetector setup ======================
	scheduler.setupSimulator();

	// ==================  Controller setup ==========================
	scheduler.setupController();

	scheduler.setupClockGenerator();

	if ( ! use_dynamics ) cerr << "[openhrp-scheduler] dynamics disabled " << endl;
	cerr << "[openhrp-scheduler] ready" << endl;

	// ==================  main loop   ======================
	int i=0;
	int j = 0;
	double time=0.0;
	double controlTime=0.0;

	cerr.setf(ios::fixed, ios::floatfield);
    cerr.precision(3);
	while ( 1 ) {
		bool control=false;
		if(controlTime <= time){
			control=true;
			j++;
		}
		if(control) scheduler.controllerInput();

		i++;
		time = scheduler.getTimeStep() * i;
		controlTime = scheduler.getControlTimeStep() * j;

		if ((i % 50)==0) {
			cerr << "[openhrp-scheduler]" << setw(6) << i << ", time: " << setw(6) << time << ", controlTime: " << setw(6) << controlTime << ", control : " << ((control==true)?"true":"false") << endl;
		}
		if(control) scheduler.controllerControl(time);

		// ================== simulate one step ==============
		scheduler.stepSimulation(use_dynamics);
		// ================== viewer update ====================

		if ((i % (int)(scheduler.getLogTimeStep()/scheduler.getControlTimeStep()))==0) {
			scheduler.viewerUpdate();
		}

		if(control) scheduler.controllerOutput();

		if( totalTime > 0 && time > totalTime ) break;

	}

	return 0;
}
