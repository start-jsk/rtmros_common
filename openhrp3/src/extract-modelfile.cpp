// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#include <hrpModel/ModelLoaderUtil.h>

#include <libxml/parser.h>
#include <libxml/xmlreader.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

#include <iostream>
#include <stdexcept>

#include <rospack/rospack.h>

using namespace std;
using namespace hrp;
using namespace OpenHRP;

string readFromXMLFile(string filename) {
	/* Load XML document */
	xmlInitParser();
	xmlDocPtr doc = xmlParseFile(filename.c_str());
	if ( doc == NULL ) {
		exit (1);
	}

	/* Create xpath evaluation context */
	xmlXPathContextPtr xpathCtx = xmlXPathNewContext(doc);
	if ( xpathCtx == NULL ) {
		xmlFreeDoc(doc);
		exit(1);
	}

	/* Evaluate xpath expression */
	xmlXPathObjectPtr xpathObj = xmlXPathEvalExpression(BAD_CAST "/grxui/mode/item", xpathCtx);
	if ( xmlXPathNodeSetIsEmpty(xpathObj->nodesetval) ) {
		exit(1);
	}

	string modelpath;
	int size;
	size = xpathObj->nodesetval->nodeNr;
	for (int i = 0; i < size; i++ ) {
		xmlNodePtr node = xpathObj->nodesetval->nodeTab[i];
		if ( xmlStrEqual( xmlGetProp(node, (xmlChar *)"class"), (xmlChar *)"com.generalrobotix.ui.item.GrxModelItem")  ) {
			string path = (char *)xmlGetProp(node, (xmlChar *)"url");
			if ( path.find("$(CURRENT_DIR)") != string::npos ) {
				path.replace(path.find("$(CURRENT_DIR)"),14, filename.substr(0, filename.find_last_of("/")));
			}
			if ( path.find("$(PROJECT_DIR)") != string::npos ) {
				try {
#ifdef ROSPACK_EXPORT
					rospack::ROSPack rp;
					rospack::Package *p = rp.get_pkg("openhrp3");
					path.replace(path.find("$(PROJECT_DIR)"),14, p->path+"/share/OpenHRP-3.1/sample/project");
#else
					rospack::Rospack rp;
					std::vector<std::string> search_path;
					rp.getSearchPathFromEnv(search_path);
					rp.crawl(search_path, 1);
					std::string pkgpath;
					if (rp.find("openhrp3",pkgpath)==true) {
						path.replace(path.find("$(PROJECT_DIR)"),14, pkgpath+"/share/OpenHRP-3.1/sample/project");
					}
#endif
				} catch (std::runtime_error &e) {
				}
			}
			if ( path[0] != '/' ) {
				path = get_current_dir_name() + string("/") + path;
			}
			xmlNodePtr cur_node = node->children;
			while ( cur_node ) {
				if ( cur_node->type == XML_ELEMENT_NODE ) {
					if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"isRobot") ) {
						if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"value"),(xmlChar *)"true")) {
							modelpath = path;
						}
					} else if ( xmlStrEqual(xmlGetProp(cur_node, (xmlChar *)"name"),(xmlChar *)"controller") ) {
						return path;
					}
				}
				cur_node = cur_node->next;
			}
		}
	}

	/* Cleanup Xpath Data */
	xmlXPathFreeObject(xpathObj);
	xmlXPathFreeContext(xpathCtx);

	/* free the document */
	xmlFreeDoc(doc);
	xmlCleanupParser();

	return modelpath;
}

int main (int argc, char* argv[]) {
	string filename = string(argv[argc-1]);
	string modelpath;

	if ( filename.substr(filename.find_last_of(".")) == ".wrl" ||
		 filename.substr(filename.find_last_of(".")) == ".dae" ) {
		modelpath = filename;
	} else {
		modelpath = readFromXMLFile(filename);
	}

#ifdef MODELPATH
	{
		cout << modelpath << endl;
		return 0;
	}
#endif

	{ // wait for ClockGenerator
		CORBA::ORB_var orb;
		CORBA::Object_var obj = NULL;
		do {
			try {
				orb = CORBA::ORB_init(argc, argv);
				CosNaming::NamingContext_var cxt;
				CORBA::Object_var	nS = orb->resolve_initial_references("NameService");
				cxt = CosNaming::NamingContext::_narrow(nS);
				CosNaming::Name ncName;
				ncName.length(1);
				ncName[0].id = CORBA::string_dup("ModelLoader");
				ncName[0].kind = CORBA::string_dup("");
				obj = cxt->resolve(ncName);
			} catch(...) {
			}
			sleep(3);
		} while ( obj == NULL );

		BodyInfo_var body = loadBodyInfo(modelpath.c_str(), orb);
#ifdef ROBOTNAME
		cout << body->name() << endl;
#endif
#ifdef ROOTLINK
		LinkInfoSequence_var links = body->links();
		cout << string(links[0].name) << endl;
#endif
	}

	return 0;
}
