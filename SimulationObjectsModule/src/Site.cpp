#include "..\include\Site.h"
#include "..\include\Agent.h"

using SimObjPtr = SimulationObject::SimObjPtr;

Site::Site(SimObjPtr owner)
{
	ownerAgent = std::static_pointer_cast<Agent>(owner);
	connected = false;
	otherSite = std::shared_ptr<Site>();
}


Site::~Site()
{
}

SimObjPtr Site::CreateInternal(SimObjPtr owner) { 
	return SimObjPtr( new Site(owner) );
	//return std::make_shared<Site>(Site(owner));
	//return std::shared_ptr<Site>(new Site(owner));
}

bool Site::isConnected() const {
	return connected;
}

void Site::connect(SimObjPtr otherSite) {
	this->otherSite = std::static_pointer_cast<Site>(otherSite);
	connected = true;
}

SimObjPtr Site::getOwnerAgent() {
	return ownerAgent;
}

