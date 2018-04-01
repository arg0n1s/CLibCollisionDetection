#include "..\include\wrapper_CLibJavaWrapper.h"
#include <unordered_map>
#include <string>

#include "..\include\CLibCollisionController.h"
#include "..\include\CLibErrorLogger.h"

#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>
#include <OctTree.h>

namespace wrapper {
	namespace java {

		using simobj::specs::MetaSpecification;
		using simobj::specs::AgentSpecification;
		using simobj::specs::SiteSpecification;
		using SiteSpecArray = std::vector<SiteSpecification>;
		using ShapePtr = std::shared_ptr<simobj::shapes::Shape>;
		using simobj::Agent;
		using simobj::ReferenceFrame;

		using std::string;
		using SiteSpecMap = std::unordered_map<string, SiteSpecArray>;
		using ShapeMap = std::unordered_map<string, ShapePtr>;
		using AgentSpecMap = std::unordered_map<string, AgentSpecification>;

		class JavaWrapperData {
		public:
			SiteSpecMap siteSpecs;
			ShapeMap shapes;
			AgentSpecMap agentSpecs;
			MetaSpecification metaSpecs;
			clib::CLibCollisionController collisionControl;

			JavaWrapperData() {
				siteSpecs = SiteSpecMap();
				shapes = ShapeMap();
				agentSpecs = AgentSpecMap();
				metaSpecs = MetaSpecification();
				collisionControl = clib::CLibCollisionController();
			};
			
		};

		JavaWrapperData wrapperData;

		bool javaStringToCString(JNIEnv * env, jstring* jStr, string* cStr) {
			jboolean isCopy;
			const char *convertedValue = (env)->GetStringUTFChars(*jStr, &isCopy);
			*cStr = string(convertedValue);
			return isCopy;
		}
	}
}

using namespace wrapper::java;
using namespace clib;


JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_initCollisionLibraryCLib(JNIEnv *, jobject) {
	wrapperData = JavaWrapperData();
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_initCollisionLibraryCLib__Ljava_lang_String_2(JNIEnv * env, jobject jObj, jstring jFolder) {
	wrapperData = JavaWrapperData();
	string folder = "";
	javaStringToCString(env, &jFolder, &folder);
	ErrorLogger::instance().changeLogFileFolder(folder);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_addSiteSpecificationCLib(JNIEnv * env, jobject jObj, jstring agentSpecID, jint siteSpecID, jint coordinateType, jdouble c1, jdouble c2, jdouble c3) {
	SiteSpecification siteSpec = createSiteSpecification((unsigned long)siteSpecID, (double)c1, (double)c2, (double)c3, static_cast<CoordinateType>(coordinateType));
	string agentId = "";
	javaStringToCString(env, &agentSpecID, &agentId);
	//wrapperData.siteSpecs.insert(std::make_pair(agentId, siteSpec));
	if (wrapperData.siteSpecs.find(agentId) == wrapperData.siteSpecs.end()) {
		SiteSpecArray sites = { siteSpec };
		wrapperData.siteSpecs.insert(std::make_pair(agentId, sites));
	}
	else {
		SiteSpecArray& sites = wrapperData.siteSpecs[agentId];
		sites.push_back(siteSpec);
	}

}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_addShapeSpecificationCLib__Ljava_lang_String_2ID(JNIEnv * env, jobject jObj, jstring shapeSpecID, jint shapeType, jdouble c1) {
	ShapePtr shape = CLibCollisionController::createShape(static_cast<ShapeType>(shapeType), (double)c1);
	string shapeId = "";
	javaStringToCString(env, &shapeSpecID, &shapeId);
	wrapperData.shapes.insert(std::make_pair(shapeId, shape));
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_addShapeSpecificationCLib__Ljava_lang_String_2IDD(JNIEnv * env, jobject jObj, jstring shapeSpecID, jint shapeType, jdouble c1, jdouble c2) {
	ShapePtr shape = CLibCollisionController::createShape(static_cast<ShapeType>(shapeType), (double)c1, (double)c2);
	string shapeId = "";
	javaStringToCString(env, &shapeSpecID, &shapeId);
	wrapperData.shapes.insert(std::make_pair(shapeId, shape));
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_addShapeSpecificationCLib__Ljava_lang_String_2IDDD(JNIEnv * env, jobject jObj, jstring shapeSpecID, jint shapeType, jdouble c1, jdouble c2, jdouble c3) {
	ShapePtr shape = CLibCollisionController::createShape(static_cast<ShapeType>(shapeType), (double)c1, (double)c2, (double)c3);
	string shapeId = "";
	javaStringToCString(env, &shapeSpecID, &shapeId);
	wrapperData.shapes.insert(std::make_pair(shapeId, shape));
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_addAgentSpecificationCLib(JNIEnv * env, jobject jObj, jstring agentSpecID, jstring shapeSpecID) {
	string shapeId = "";
	javaStringToCString(env, &shapeSpecID, &shapeId);
	string agentId = "";
	javaStringToCString(env, &agentSpecID, &agentId);
	AgentSpecification agentSpec = createAgentSpecification(agentId, wrapperData.shapes.at(shapeId), wrapperData.siteSpecs.at(agentId));
	wrapperData.agentSpecs.insert(std::make_pair(agentId, agentSpec));
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_initCollisionControllerCLib(JNIEnv * env, jobject jObj) {
	AgentSpecArray agents;
	for (auto agentSpec : wrapperData.agentSpecs) {
		agents.push_back(agentSpec.second);
	}
	wrapperData.metaSpecs = createMetaSpecification(agents);
	wrapperData.collisionControl = CLibCollisionController(wrapperData.metaSpecs);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_setMinimalLeafSizeCLib(JNIEnv * env, jobject jObj, jdouble minimalLeafSize) {
	wrapperData.collisionControl.setMinimalLeafDiameter(minimalLeafSize);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_setMaxInitialRootSizeCLib(JNIEnv * env, jobject jObj, jdouble initialRootSize) {
	wrapperData.collisionControl.setInitialRootDiameter(initialRootSize);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_createAgentCLib(JNIEnv * env, jobject jObj, jstring agentSpecID, jint agentID) {
	string agentSpecId = "";
	javaStringToCString(env, &agentSpecID, &agentSpecId);
	wrapperData.collisionControl.createAgent((unsigned long)agentID, agentSpecId);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_insertAgentIntoClusterCLib(JNIEnv * env, jobject jObj, jint agentID, jint clusterID) {
	wrapperData.collisionControl.addAgentToCluster((unsigned long)agentID, (unsigned long)clusterID);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_moveAgentCLib(JNIEnv * env, jobject jObj, jint agentID, jdouble x, jdouble y, jdouble z) {
	Eigen::Vector3d translation((double)x, (double)y, (double)z);
	try {
		std::shared_ptr<Agent> agnt = std::static_pointer_cast<Agent>(wrapperData.collisionControl.getAgent((unsigned long)agentID));
		agnt->move(translation);
	}
	catch (std::exception& e) {
		ErrorLogger::instance().appendErrorMsg(e.what());
	}
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_rotateAgentCLib(JNIEnv * env, jobject jObj, jint agentID, jdouble w, jdouble x, jdouble y, jdouble z) {
	Eigen::Quaternion<double> rotation((double)w, (double)x, (double)y, (double)z);
	try {
		std::shared_ptr<Agent> agnt = std::static_pointer_cast<Agent>(wrapperData.collisionControl.getAgent((unsigned long)agentID));
		agnt->rotate(rotation);
	}
	catch (std::exception& e) {
		ErrorLogger::instance().appendErrorMsg(e.what());
	}
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_getAgentPositionCLib(JNIEnv * env, jobject jObj, jint agentID, jdouble x, jdouble y, jdouble z) {
	try {
		std::shared_ptr<Agent> agnt = std::static_pointer_cast<Agent>(wrapperData.collisionControl.getAgent((unsigned long)agentID));
		x = (jdouble)agnt->getPosition(ReferenceFrame::Global).x();
		y = (jdouble)agnt->getPosition(ReferenceFrame::Global).y();
		z = (jdouble)agnt->getPosition(ReferenceFrame::Global).z();
	}
	catch (std::exception& e) {
		ErrorLogger::instance().appendErrorMsg(e.what());
		x = 0;
		y = 0;
		z = 0;
	}
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_getAgentRotationCLib(JNIEnv * env, jobject jObj, jint agentID, jdouble w, jdouble x, jdouble y, jdouble z) {
	try {
		std::shared_ptr<Agent> agnt = std::static_pointer_cast<Agent>(wrapperData.collisionControl.getAgent((unsigned long)agentID));
		w = (jdouble)agnt->getOrientation(ReferenceFrame::Global).w();
		x = (jdouble)agnt->getOrientation(ReferenceFrame::Global).x();
		y = (jdouble)agnt->getOrientation(ReferenceFrame::Global).y();
		z = (jdouble)agnt->getOrientation(ReferenceFrame::Global).z();
	}
	catch (std::exception& e) {
		ErrorLogger::instance().appendErrorMsg(e.what());
		w = 1;
		x = 0;
		y = 0;
		z = 0;
	}
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_connectAgentsCLib(JNIEnv * env, jobject jObj, jint agent1, jint agent2, jint site1, jint site2) {
	try {
		wrapperData.collisionControl.connectAgents((unsigned long)agent1, (unsigned long)agent2, (unsigned long)site1, (unsigned long)site2);
	}
	catch (std::exception& e) {
		ErrorLogger::instance().appendErrorMsg(e.what());
	}
	
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_octTreeFromClusterCLib(JNIEnv * env, jobject jObj, jint clusterID) {
	wrapperData.collisionControl.addAgentClusterToCollisionDetector((unsigned long)clusterID);
}


JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_insertAgentIntoOctTreeCLib(JNIEnv * env, jobject jObj, jint agentID, jint clusterID) {
	wrapperData.collisionControl.addAgentToCollisionDetector((unsigned long)agentID, (unsigned long)clusterID);
}

JNIEXPORT jint JNICALL Java_wrapper_CLibJavaWrapper_findNearestToAgentCLib(JNIEnv * env, jobject jObj, jint agentID, jint clusterID) {
	SimObjPtr nearest;
	if (wrapperData.collisionControl.findNearestToAgent((unsigned long)agentID, (unsigned long)clusterID, nearest)) {
		return (jint)nearest->getId();
	}
	else {
		return agentID;
	}
}

JNIEXPORT jboolean JNICALL Java_wrapper_CLibJavaWrapper_checkCollisionBetweenAgentsCLib(JNIEnv * env, jobject jObj, jint agentID1, jint agentID2) {
	return (jboolean)wrapperData.collisionControl.checkCollisionBetweenAgents((unsigned long)agentID1, (unsigned long)agentID2);
}

JNIEXPORT jdouble JNICALL Java_wrapper_CLibJavaWrapper_calculateDistanceCLib(JNIEnv * env, jobject jObj, jint agentID1, jint agentID2) {
	return (jdouble)wrapperData.collisionControl.distanceBetweenAgents((unsigned long)agentID1, (unsigned long)agentID2);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_showAgentCLib(JNIEnv * env, jobject jObj, jint agentID) {
	wrapperData.collisionControl.displayAgent((unsigned long)agentID);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_showClusterCLib(JNIEnv * env, jobject jObj, jint clusterID) {
	wrapperData.collisionControl.displayAgentCluster((unsigned long)clusterID);
}

JNIEXPORT void JNICALL Java_wrapper_CLibJavaWrapper_showOctTreeCLib(JNIEnv * env, jobject jObj, jint clusterID) {
	wrapperData.collisionControl.displayClusterCollisionTree((unsigned long)clusterID);
}

JNIEXPORT jstring JNICALL Java_wrapper_CLibJavaWrapper_agentToString(JNIEnv * env, jobject jObj, jint agentID) {
	string agentInfo = wrapperData.collisionControl.getAgent((unsigned long)agentID)->toString();
	return env->NewStringUTF(agentInfo.c_str());
}

JNIEXPORT jstring JNICALL Java_wrapper_CLibJavaWrapper_clusterToString(JNIEnv * env, jobject jObj, jint clusterID) {
	string clusterInfo = wrapperData.collisionControl.getAgentCluster((unsigned long)clusterID)->toString();
	return env->NewStringUTF(clusterInfo.c_str());
}

JNIEXPORT jstring JNICALL Java_wrapper_CLibJavaWrapper_collisionControllerToString(JNIEnv * env, jobject jObj) {
	string info = wrapperData.collisionControl.toString();
	return env->NewStringUTF(info.c_str());
}