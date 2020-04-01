//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include <QSet>

#include "PdmsTools.h"

//System
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>

//qCC_db
//#include <ccLog.h>

using namespace PdmsTools;
using namespace PdmsCommands;
using namespace PdmsObjects;

/////////// MACROS ////////////

#define memalert(e,s) std::cerr << "Memory alert [" << __FILE__ << ", line " << __LINE__<< "] with size " << s << " : " << e.what() << std::endl;
#define memfail(e,s) memalert(e,s); abort();

/////////// FUNCTIONS ////////////
#define PDMS_SQR(a) ((a)*(a))

/////////// GLOBALS ////////////
Token DistanceValue::workingUnit = PDMS_MILLIMETRE;
static GroupElement defaultWorld(PDMS_WORLD);

///////////////////////////////
// ITEM STACK
///////////////////////////////
using ElementsStack = QSet<PdmsObjects::GenericItem *>;
static ElementsStack s_elementsStack;

void PdmsObjects::Stack::Init()
{
	assert(s_elementsStack.empty());
	s_elementsStack.clear();
}

void PdmsObjects::Stack::Clear()
{
	//DGM: warning, while deleting some entities, the stack may be modified!!!
	while (!s_elementsStack.empty())
	{
		GenericItem* item = *(s_elementsStack.begin());
		
		s_elementsStack.remove( item );
		
		if (item)
		{
			//ccLog::Print(QString("[PDMS] Should be deleting %1").arg(item->name));
			delete item;
		}
	}
	
	s_elementsStack.clear();
}

void PdmsObjects::Stack::Destroy(GenericItem* &item)
{
	if ( item && s_elementsStack.remove( item ) )
	{
		//ccLog::Print(QString("[PDMS] Destroying %1").arg((item)->name));
		delete item;
		item = nullptr;
	}
}

///////////////////////////////
// PDMS COMMANDS IMPLEMENTATION
///////////////////////////////

bool NumericalValue::handle(PointCoordinateType numvalue)
{
	value = numvalue;
	valueChanges++;
	return valueChanges == 1;
}

bool NumericalValue::isValid() const
{
	return valueChanges <= 1;
}

PointCoordinateType NumericalValue::getValue() const
{
	switch (command)
	{
	case PDMS_ANGLE:
	case PDMS_X_TOP_SHEAR:
	case PDMS_Y_TOP_SHEAR:
	case PDMS_X_BOTTOM_SHEAR:
	case PDMS_Y_BOTTOM_SHEAR:
		return static_cast<PointCoordinateType>(CC_DEG_TO_RAD) * value;
	default:
		return value;
	}
}

bool NumericalValue::execute(PdmsObjects::GenericItem* &item) const
{
	return item ? item->setValue(command, getValue()) : false;
}

bool DistanceValue::handle(Token t)
{
	if (!PdmsToken::isUnit(t))
		return false;
	if (!isValid())
		return false;
	unit = t;
	return true;
}

PointCoordinateType DistanceValue::getValueInWorkingUnit() const
{
	if (unit == PDMS_MILLIMETRE && workingUnit == PDMS_METRE)
		return value / static_cast<PointCoordinateType>(1000);
	if (unit == PDMS_METRE && workingUnit == PDMS_MILLIMETRE)
		return value * static_cast<PointCoordinateType>(1000);

	return value;
}

bool DistanceValue::execute(PdmsObjects::GenericItem* &item) const
{
	return item ? item->setValue(command, getValueInWorkingUnit()) : false;
}

Reference& Reference::operator=(const Reference &ref)
{
	command = ref.command;
	token = ref.token;
	strcpy(refname, ref.refname);

	return *this;
}

bool Reference::handle(Token t)
{
	if (isSet())
		return false;
	//Todo : handle other references than grouping elements or design element
	if (!PdmsToken::isElement(t))
		return false;

	token = t;
	return true;
}

bool Reference::handle(const char* str)
{
	if (isSet())
		return false;
	strcpy(refname, str);

	return true;
}

bool Reference::isValid() const
{
	return ((command == PDMS_LAST && isSet() <= 1) || isSet() == 1);
}

bool Reference::isNameReference() const
{
	return strlen(refname) > 0;
}

bool Reference::isTokenReference() const
{
	return token != PDMS_INVALID_TOKEN;
}

int Reference::isSet() const
{
	int nb = 0;
	if (strlen(refname) > 0)
		nb++;
	if (token != PDMS_INVALID_TOKEN)
		nb++;
	return nb;
}

bool Reference::execute(PdmsObjects::GenericItem* &item) const
{
	//Handle the PDMS_LAST command
	if (command == PDMS_LAST)
	{
		if (s_elementsStack.size() < 2)
			return false;
		ElementsStack::iterator it = s_elementsStack.end(); --it;
		if (isSet() == 1)
		{
			while (true)
			{
				if (isNameReference() && strcmp(refname, (*it)->name) == 0)
					break;
				if (isTokenReference() && (*it)->getType() == token)
					break;
				if (it == s_elementsStack.begin())
					return false;
				--it;
			}
		}
		item = *it;
		return true;
	}

	//Check that this reference is not a PDMS_OWNER termination command
	if (command == PDMS_OWNER && !isSet())
	{
		//Redirect to an ending command
		ElementEnding endCommand(PDMS_OWNER);
		return endCommand.execute(item);
	}

	GenericItem* result = nullptr;
	//Search for the referenced item depending on the reference type
	if (isNameReference())
	{
		//Use the hierarchy scanning function given the requested object name
		if (item)
			return false;
		result = item->getRoot()->scan(refname);
	}
	//Request for an element (hierarchical or design element only)
	else if (isTokenReference())
	{
		if (PdmsToken::isGroupElement(token))
		{
			//Go up in the hierarchy to find a matching, or the first group which can own the request item
			result = item;
			while (result && result->getType() > token)
				result = result->owner;
			if (!result)
				result = &defaultWorld;
		}
		else if (PdmsToken::isDesignElement(token))
		{
			//Go up in the hierarchy until we meet the requested type
			result = item;
			while (result && result->getType() != token)
				result = result->owner;
		}
		else
			return false;
	}

	//If the reference command is PDMS_OWNER, then we have to change the request item owner
	if (command == PDMS_OWNER && result)
	{
		if (!item)
			return false;
		item->owner = result;
		result = item;
	}

	if (result)
		item = result;

	return (result != nullptr);
}


bool Coordinates::handle(Token t)
{
	if (current >= 3)
		return false;

	//Check that current active command cannot handle this token (if it cannot, check that it is valid before continuing)
	if (current >= 0)
	{
		if (coords[current].handle(t))
			return true;
		if (!coords[current].isValid())
			return false;
	}

	//Handle coordinates commands
	if (!PdmsToken::isCoordinate(t))
		return false;

	if (++current >= 3)
		return false;

	coords[current].command = t;
	//coords[current].value = 1;
	coords[current].value = 0;

	return true;
}

bool Coordinates::handle(PointCoordinateType numvalue)
{
	if (current < 0 || current >= 3)
		return false;

	//Only coordinates can handle numerical values
	if (!PdmsToken::isCoordinate(coords[current].command))
		return false;

	return coords[current].handle(numvalue);
}

bool Coordinates::isValid() const
{
	//Can't be invalid
	return true;
}

bool Coordinates::getVector(CCVector3 &u) const
{
	bool ok[3] = { false, false, false };
	u = CCVector3(0, 0, 0);

	int nb = getNbComponents();
	for (int i = 0; i < nb; i++)
	{
		if (!coords[i].isValid())
			return false;
		if (ok[i])
			return false;
		switch (coords[i].command)
		{
		case PDMS_X:
		case PDMS_EST:
			u[0] = coords[i].getValueInWorkingUnit();
			ok[0] = true;
			break;

		case PDMS_WEST:
			u[0] = -coords[i].getValueInWorkingUnit();
			ok[0] = true;
			break;

		case PDMS_Y:
		case PDMS_NORTH:
			u[1] = coords[i].getValueInWorkingUnit();
			ok[1] = true;
			break;

		case PDMS_SOUTH:
			u[1] = -coords[i].getValueInWorkingUnit();
			ok[1] = true;
			break;

		case PDMS_Z:
		case PDMS_UP:
			u[2] = coords[i].getValueInWorkingUnit();
			ok[2] = true;
			break;

		case PDMS_DOWN:
			u[2] = -coords[i].getValueInWorkingUnit();
			ok[2] = true;
			break;

		default:
			return false;
		}
	}
	return true;
}

int Coordinates::getNbComponents(bool onlyset) const
{
	int nb = 0;
	for (int i = 0; i < 3; i++)
	{
		if (PdmsToken::isCoordinate(coords[i].command))
		{
			if (!onlyset || coords[nb].valueChanges >= 1)
				nb++;
		}
	}
	return nb;
}

bool Position::handle(Token t)
{
	//Check if current activ command can handle this token (if it cannot, check that it is valid before continuing)
	if (current)
	{
		if (current->handle(t))
			return true;
		if (!current->isValid())
			return false;
	}
	//If no current command is activ, then token must be either a reference command or a coordinate ID
	//Handle PDMS_WRT (and check it has not been handled yet)
	if (t == PDMS_WRT)
	{
		current = &ref;
		if (current->command)
			return false;
		current->command = t;
		return true;
	}
	//Handle coordinates ID
	if (PdmsToken::isCoordinate(t))
	{
		current = &position;
		return current->handle(t);
	}
	//Not handled
	return false;
}

bool Position::handle(PointCoordinateType numvalue)
{
	if (!current)
		return false;
	return current->handle(numvalue);
}

bool Position::handle(const char* str)
{
	if (!current)
		return false;
	return current->handle(str);
}

bool Position::isValid() const
{
	if (!position.isValid())
		return false;
	if (ref.command == PDMS_WRT)
		return ref.isValid();
	return true;
}

bool Position::execute(PdmsObjects::GenericItem* &item) const
{
	if (!item)
		return false;

	//Resolve reference if needed
	GenericItem* refpos = nullptr;
	if (ref.isValid())
	{
		refpos = item;
		if (!ref.execute(refpos))
			return false;
	}

	//Get position point
	CCVector3 p;
	position.getVector(p);
	item->setPosition(p);
	item->positionReference = refpos;

	return true;
}

bool Orientation::handle(Token t)
{
	//Check that current active command cannot handle this token (if it cannot, check that it is valid before continuing)
	if (current)
	{
		if (current->handle(t))
			return true;
		if (!current->isValid())
			return false;
	}
	//PDMS_AND command exits current coordinates system
	if (t == PDMS_AND)
	{
		if (!current || !current->isValid())
			return false;
		current = nullptr;
		return true;
	}
	//PDMS_IS activates last specified component
	if (t == PDMS_IS)
	{
		if (component < 0 || component >= 3)
			return false;
		if (current)
			return false;
		current = &orientation[component];
		return true;
	}
	//Handle PDMS_WRT (and check it has not been handled yet)
	if (t == PDMS_WRT)
	{
		if (component < 0 || component >= 3)
			return false;
		current = &refs[component];
		if (current->command)
			return false;
		current->command = t;
		return true;
	}
	//Handle coordinates ID : here, we should create a new orientation axis (since no current element is activ)
	if (PdmsToken::isCoordinate(t))
	{
		if (++component >= 3)
			return false;
		orientation[component].command = t;
		current = nullptr;
		return true;
	}
	return false;
}

bool Orientation::handle(PointCoordinateType numvalue)
{
	return current ? current->handle(numvalue) : false;
}

bool Orientation::handle(const char* str)
{
	return current ? current->handle(str) : false;
}

bool Orientation::isValid() const
{
	int nb = getNbComponents();
	if (nb <= 0)
		return false;

	for (int i = 0; i < nb; i++)
	{
		if (PdmsToken::isCoordinate(orientation[i].command))
			return false;
		if (!orientation[i].isValid())
			return false;
		if (refs[i].command == PDMS_WRT && !refs[i].isValid())
			return false;
	}

	return true;
}

bool Orientation::getAxes(CCVector3 &x, CCVector3 &y, CCVector3 &z) const
{
	x = y = z = CCVector3(0, 0, 0);

	int nb = getNbComponents();
	for (int i = 0; i < nb; i++)
	{
		if (!orientation[i].isValid())
			return false;

		switch (orientation[i].command)
		{
		case PDMS_X:
		case PDMS_EST:
			if (!axisFromCoords(orientation[i], x))
				return false;
			break;

		case PDMS_WEST:
			if (!axisFromCoords(orientation[i], x))
				return false;
			x *= -1.;
			break;

		case PDMS_Y:
		case PDMS_NORTH:
			if (!axisFromCoords(orientation[i], y))
				return false;
			break;

		case PDMS_SOUTH:
			if (!axisFromCoords(orientation[i], y))
				return false;
			y *= -1.;
			break;

		case PDMS_Z:
		case PDMS_UP:
			if (!axisFromCoords(orientation[i], z))
				return false;
			break;

		case PDMS_DOWN:
			if (!axisFromCoords(orientation[i], z))
				return false;
			z *= -1.;
			break;

		default:
			return false;
		}
	}

	return nb != 0;
}

bool Orientation::axisFromCoords(const Coordinates &coords, CCVector3 &u)
{
	if (!coords.getVector(u))
		return false;

	if (coords.getNbComponents(true) == 2)
	{
		PointCoordinateType alpha = static_cast<PointCoordinateType>(CC_DEG_TO_RAD) * u[0];
		PointCoordinateType beta = static_cast<PointCoordinateType>(CC_DEG_TO_RAD) * u[1];
		u[0] = cos(alpha)*cos(beta);
		u[1] = sin(alpha)*cos(beta);
		u[2] = sin(beta);
	}

	return true;
}

int Orientation::getNbComponents() const
{
	int nb = 0;
	while (nb < 3 && orientation[nb].command)
		nb++;
	return nb;
}

bool Orientation::execute(PdmsObjects::GenericItem* &item) const
{
	if (!item)
		return false;

	//Resolve reference if needed
	for (unsigned i = 0; i < 3; i++)
	{
		GenericItem* refori = nullptr;
		if (refs[i].isValid())
		{
			refori = item;
			if (!refs[i].execute(refori))
				return false;
		}
		item->orientationReferences[i] = refori;
	}

	//Get position point
	CCVector3 x;
	CCVector3 y;
	CCVector3 z;
	if (!getAxes(x, y, z))
		return false;

	item->setOrientation(x, y, z);
	return true;
}

bool Name::execute(PdmsObjects::GenericItem* &item) const
{
	if (!item)
		return false;

	strcpy(item->name, name);

	return true;
}

bool ElementCreation::handle(const char*str)
{
	if (!elementType)
		return false;
	if (!path.empty())
		return false;
	return splitPath(str);
}

bool ElementCreation::handle(Token t)
{
	if (PdmsToken::isElement(t))
	{
		if (elementType)
			return false;
		elementType = t;
		return true;
	}
	return false;
}

bool ElementCreation::isValid() const
{
	return PdmsToken::isElement(elementType);
}

const char* ElementCreation::GetDefaultElementName(Token token)
{
	switch (token)
	{
	case PDMS_GROUP:
		return "Group";
	case PDMS_WORLD:
		return "World";
	case PDMS_SITE:
		return "Site";
	case PDMS_ZONE:
		return "Zone";
	case PDMS_EQUIPMENT:
		return "Equipment";
	case PDMS_STRUCTURE:
		return "Structure";
	case PDMS_SUBSTRUCTURE:
		return "Sub-structure";
		//PDMS elements
	case PDMS_SCYLINDER:
		return "Cylinder";
	case PDMS_CTORUS:
		return "Torus (C)";
	case PDMS_RTORUS:
		return "Torus (R)";
	case PDMS_DISH:
		return "Dish";
	case PDMS_CONE:
		return "Cone";
	case PDMS_BOX:
		return "Box";
	case PDMS_NBOX:
		return "Box(-)";
	case PDMS_PYRAMID:
		return "Pyramid";
	case PDMS_SNOUT:
		return "Snout";
	case PDMS_EXTRU:
		return "Extrusion";
	case PDMS_NEXTRU:
		return "Extrusion(-)";
	case PDMS_LOOP:
		return "Loop";
	case PDMS_VERTEX:
		return "Vertex";

	default:
		break;
	}

	return nullptr;
}

bool ElementCreation::execute(PdmsObjects::GenericItem* &item) const
{
	GenericItem* newElement = nullptr;

	try
	{
		switch (elementType)
		{
		case PDMS_GROUP:
		case PDMS_WORLD:
		case PDMS_SITE:
		case PDMS_ZONE:
		case PDMS_EQUIPMENT:
		case PDMS_STRUCTURE:
		case PDMS_SUBSTRUCTURE:
			newElement = new GroupElement(elementType);
			break;
			//PDMS elements
		case PDMS_SCYLINDER:
			newElement = new SCylinder;
			break;
		case PDMS_CTORUS:
			newElement = new CTorus;
			break;
		case PDMS_RTORUS:
			newElement = new RTorus;
			break;
		case PDMS_DISH:
			newElement = new Dish;
			break;
		case PDMS_CONE:
			newElement = new Cone;
			break;
		case PDMS_BOX:
		case PDMS_NBOX:
			newElement = new Box;
			static_cast<Box*>(newElement)->negative = (elementType == PDMS_NBOX);
			break;
		case PDMS_PYRAMID:
			newElement = new Pyramid;
			break;
		case PDMS_SNOUT:
			newElement = new Snout;
			break;
		case PDMS_EXTRU:
		case PDMS_NEXTRU:
			newElement = new Extrusion;
			static_cast<Extrusion*>(newElement)->negative = (elementType == PDMS_NEXTRU);
			break;
		case PDMS_LOOP:
			newElement = new Loop;
			break;
		case PDMS_VERTEX:
			newElement = new Vertex;
			break;
		default:
			break;
		}
	}
	catch (std::exception &nex)
	{
		memalert(nex, 1);
		return false;
	}

	if (!newElement)
	{
		return false;
	}

	const char* name = GetDefaultElementName(elementType);
	if (name)
	{
		strcpy(newElement->name, name);
	}

	//If the path is changed during the creation, do it now
	if (path.size() > 1)
	{
		if (!item)
		{
			//delete newElement;
			PdmsObjects::Stack::Destroy(newElement);
			return false;
		}
		PdmsObjects::GenericItem* mitem = item->getRoot();
		for (unsigned i = 0; i + 1 < path.size(); i++)
		{
			mitem = mitem->scan(path[i].c_str());
			if (!mitem)
			{
				//delete newElement;
				PdmsObjects::Stack::Destroy(newElement);
				return false;
			}
		}
		item = mitem;
	}

	//Then we can (try to) push the new element in the hierarchy
	if (item && !item->push(newElement))
	{
		//delete newElement;
		PdmsObjects::Stack::Destroy(newElement);
		return false;
	}

	newElement->creator = newElement->owner;
	if (!path.empty())
	{
		strcpy(newElement->name, path.back().c_str());
	}

	try
	{
		s_elementsStack.insert(newElement);
	}
	catch (std::exception &pex)
	{
		memalert(pex, 1);
		PdmsObjects::Stack::Destroy(newElement);
		//delete newElement;
		return false;
	}
	item = newElement;
	return true;
}

bool ElementEnding::execute(PdmsObjects::GenericItem* &item) const
{
	GenericItem* result = nullptr;
	switch (command)
	{
	case PDMS_OWNER:
		//If the ending command is PDMS_OWNER, then we simply go back to the item owner
		result = item ? item->owner : nullptr;
		break;
	case PDMS_END:
		//Bug realworks : ignore END EXTRU command
		if (end.isTokenReference() && (end.token == PDMS_EXTRU || end.token == PDMS_NEXTRU))
			return true;
		//In the general case, we have to find the references item (default : this one), and go back to its creator
		result = item;
		if (end.isValid() && !end.execute(result))
		{
			return false;
		}
		if (!result)
			return false;
		result = result->creator;
		break;
	case PDMS_LAST:
		if (!end.execute(result))
			return false;
		break;
	default:
		return false;
	}

	item = result;

	return true;
}

bool ElementCreation::splitPath(const char *str)
{
	path.clear();

	//Each time a new '/' is met, we create a new entry in the path
	unsigned i = 0;
	while (str[i])
	{
		if (str[i] == '/')
		{
			if (i != 0)
				path.emplace_back(str, i);
			str = &str[i + 1];
			i = 0;
		}
		else
		{
			i++;
		}
	}

	//At the end, we have to create an entry for the last word
	if (i != 0)
		path.emplace_back(str, i);

	return (!path.empty());
}


bool HierarchyNavigation::execute(PdmsObjects::GenericItem* &item) const
{
	GenericItem* result = item;
	if (!result || !isValid())
		return true;

	//Go back to the first creator object that matches or can contain the command hierarchy level
	while (result && command < result->getType())
		result = result->creator;

	//If we went to the root, we have to create a new hierarchy level and set it as the new root
	if (!result)
	{
		try
		{
			result = new GroupElement(command);
		}
		catch (std::exception &nex)
		{
			memfail(nex, 1);
			return false;
		}
		result->push(item);
	}

	//change the current element as the new accessed element
	item = result;
	return true;
}

Command* Command::Create(Token t)
{
	try
	{
		Command *result = nullptr;
		switch (t)
		{
		case PDMS_CREATE:
			result = new ElementCreation;
			break;
		case PDMS_END:
		case PDMS_LAST:
			result = new ElementEnding(t);
			break;
		case PDMS_WRT:
		case PDMS_OWNER:
			result = new Reference(t);
			break;
		case PDMS_NAME:
			result = new Name;
			break;
			//Attributes
		case PDMS_DIAMETER:
		case PDMS_HEIGHT:
		case PDMS_RADIUS:
		case PDMS_INSIDE_RADIUS:
		case PDMS_OUTSIDE_RADIUS:
		case PDMS_TOP_DIAMETER:
		case PDMS_BOTTOM_DIAMETER:
		case PDMS_XLENGTH:
		case PDMS_YLENGTH:
		case PDMS_ZLENGTH:
		case PDMS_X_OFF:
		case PDMS_Y_OFF:
		case PDMS_X_BOTTOM:
		case PDMS_Y_BOTTOM:
		case PDMS_X_TOP:
		case PDMS_Y_TOP:
			result = new DistanceValue(t);
			break;
		case PDMS_X_TOP_SHEAR:
		case PDMS_X_BOTTOM_SHEAR:
		case PDMS_Y_TOP_SHEAR:
		case PDMS_Y_BOTTOM_SHEAR:
		case PDMS_ANGLE:
			result = new NumericalValue(t);
			break;
		case PDMS_POSITION:
			result = new Position;
			break;
		case PDMS_ORIENTATION:
			result = new Orientation;
			break;
		case PDMS_WORLD:
		case PDMS_SITE:
		case PDMS_ZONE:
		case PDMS_EQUIPMENT:
		case PDMS_STRUCTURE:
		case PDMS_SUBSTRUCTURE:
			result = new HierarchyNavigation(t);
			break;
		default: break;
		}
		return result;
	}
	catch (std::exception &nex)
	{
		memfail(nex, 1);
		return nullptr;
	}
}

///////////////////////////////
// PDMS OBJECTS IMPLEMENTATION
///////////////////////////////

GenericItem::GenericItem()
	: owner(nullptr)
	, creator(nullptr)
	, position(0, 0, 0)
	, isCoordinateSystemUpToDate(false)
	, positionReference(nullptr)
{
	orientationReferences[0] = orientationReferences[1] = orientationReferences[2] = nullptr;
	orientation[0] = CCVector3(0, 0, 0); orientation[0][0] = 1;
	orientation[1] = CCVector3(0, 0, 0); orientation[1][1] = 1;
	orientation[2] = CCVector3(0, 0, 0); orientation[2][2] = 1;
	name[0] = '\0';
}

bool GenericItem::setPosition(const CCVector3 &p)
{
	position = p;
	return true;
}

bool GenericItem::setOrientation(const CCVector3 &x, const CCVector3 &y, const CCVector3 &z)
{
	orientation[0] = x;
	orientation[1] = y;
	orientation[2] = z;
	return true;
}

bool GenericItem::isOrientationValid(unsigned i) const
{
	return (orientation[i].norm2() > ZERO_TOLERANCE);
}

bool GenericItem::completeOrientation()
{
	bool ok[3] = { isOrientationValid(0),
					isOrientationValid(1),
					isOrientationValid(2) };

	unsigned nb = static_cast<unsigned>(ok[0])
		+ static_cast<unsigned>(ok[1])
		+ static_cast<unsigned>(ok[2]);

	switch (nb)
	{
	case 0:
		return false;

	case 1:

		if (ok[0]) { orientation[0].normalize(); orientation[1] = orientation[0].orthogonal(); orientation[2] = orientation[0].cross(orientation[1]); break; }
		if (ok[1]) { orientation[1].normalize(); orientation[2] = orientation[1].orthogonal(); orientation[0] = orientation[1].cross(orientation[2]); break; }
		if (ok[2]) { orientation[2].normalize(); orientation[0] = orientation[2].orthogonal(); orientation[1] = orientation[2].cross(orientation[0]); break; }

	case 2:

		if (!ok[0]) { orientation[1].normalize(); orientation[2].normalize(); orientation[0] = orientation[1].cross(orientation[2]); }
		if (!ok[1]) { orientation[0].normalize(); orientation[2].normalize(); orientation[1] = orientation[2].cross(orientation[0]); }
		if (!ok[2]) { orientation[0].normalize(); orientation[1].normalize(); orientation[2] = orientation[0].cross(orientation[1]); }
		break;

	case 3:
		break;

	default:
		return false;
	}

	return true;
}

bool GenericItem::convertCoordinateSystem()
{
	if (isCoordinateSystemUpToDate)
		return true;
	if (!positionReference)
		positionReference = owner;
	//init orientationReferences
	{
		for (unsigned k = 0; k < 3; k++)
			if (!orientationReferences[k])
				orientationReferences[k] = owner;
	}
	//Update position coordinates
	if (positionReference)
	{
		if (!positionReference->convertCoordinateSystem())
			return false;
		//New position is reference origin plus sum(reference_axis[i]*point_coordinate[i])
		GenericItem *ref = positionReference;
		if (!ref->isCoordinateSystemUpToDate && ref->owner == this)
			return false;
		CCVector3 p = position;
		for (unsigned i = 0; i < 3; i++)
			position[i] = ref->orientation[0][i] * p[0] + ref->orientation[1][i] * p[1] + ref->orientation[2][i] * p[2];
		position += ref->position;
	}
	//The same for orientation
	for (unsigned k = 0; k < 3; k++)
	{
		if (!isOrientationValid(k))
			continue;
		if (orientationReferences[k])
		{
			if (!orientationReferences[k]->convertCoordinateSystem())
				return false;
			//New axis is sum(reference_axis[i]*axis[i])
			GenericItem *ref = orientationReferences[k];
			if (!ref->isCoordinateSystemUpToDate && ref->owner == this)
				return false;

			CCVector3 axis[3];
			{
				for (unsigned j = 0; j < 3; j++)
					axis[j] = orientation[j];
			}
			{
				for (unsigned j = 0; j < 3; j++)
					for (unsigned i = 0; i < 3; i++)
						orientation[j][i] = ref->orientation[0][i] * axis[j][0] + ref->orientation[1][i] * axis[j][1] + ref->orientation[2][i] * axis[j][2];
			}
		}
	}

	if (!completeOrientation())
		return false;
	isCoordinateSystemUpToDate = true;

	return true;
}

bool GenericItem::scan(Token t, std::vector<GenericItem *> &array)
{
	if (getType() == t)
	{
		try
		{
			array.push_back(this);
		}
		catch (std::exception &pex)
		{
			memfail(pex, array.size());
		}
		return true;
	}

	return false;
}

DesignElement::~DesignElement()
{
	for (std::list<DesignElement*>::iterator it = nelements.begin(); it != nelements.end(); ++it)
	{
		GenericItem* item = *it;
		if (item)
		{
			Stack::Destroy(item);
		}
	}
	nelements.clear();
}

bool DesignElement::push(GenericItem *i)
{
	if (i->isDesignElement())
	{
		DesignElement *element = static_cast<DesignElement*>(i);
		if (element->negative)
		{
			try
			{
				nelements.push_back(element);
			}
			catch (std::exception &pex)
			{
				memalert(pex, nelements.size());
				return false;
			}
			if (element->owner)
				element->owner->remove(element);
			element->owner = this;
			return true;
		}
	}

	//In most cases, design elements do not handle nested elements
	if (owner)
		return owner->push(i);

	return false;
}

void DesignElement::remove(GenericItem *i)
{
	for (std::list<DesignElement*>::iterator it = nelements.begin(); it != nelements.end();)
	{
		if (*it == i)
			nelements.erase(it);
		else
			++it;
	}
}

GroupElement::GroupElement(Token l)
{
	level = l;
	elements.clear();
	subhierarchy.clear();
	memset(name, 0, c_max_str_length);
}

GroupElement::~GroupElement()
{
	clear(true);
}

void GroupElement::clear(bool del)
{
	if (del)
	{
		for (std::list<DesignElement*>::iterator eit = elements.begin(); eit != elements.end(); ++eit)
		{
			GenericItem* item = *eit;
			if (*eit)
				Stack::Destroy(item);
		}
		for (std::list<GroupElement*>::iterator hit = subhierarchy.begin(); hit != subhierarchy.end(); ++hit)
		{
			GenericItem* item = *hit;
			if (*hit)
				Stack::Destroy(item);
		}
	}
	elements.clear();
	subhierarchy.clear();
}

bool GroupElement::push(GenericItem *i)
{
	//In each case, the insertion of a new element consists in finding the list in which it should be added

	//If the request item is a group, we have to find its new place in the hierarchy
	if (PdmsToken::isGroupElement(i->getType()))
	{
		//If this group can contain the request item, then insert it in the group list
		GroupElement *group = dynamic_cast<GroupElement*>(i);
		if (group->level == PDMS_GROUP || group->level > level)
		{
			if (group->owner)
				group->owner->remove(group);
			group->owner = this;

			try
			{
				subhierarchy.push_back(group);
			}
			catch (std::exception &pex)
			{
				memalert(pex, subhierarchy.size());
				return false;
			}
		}
		//else the requested item should be inserted in this group owner
		else if (owner)
			owner->push(group);
		else
			return false;
	}
	//For design elements, insert it in the group' design element list
	else if (PdmsToken::isDesignElement(i->getType()))
	{
		if (i->owner)
			i->owner->remove(i);
		i->owner = this;
		try
		{
			elements.push_back(dynamic_cast<DesignElement*>(i));
		}
		catch (std::exception &pex)
		{
			memalert(pex, elements.size());
			return false;
		}
		return true;
	}
	return true;
}

void GroupElement::remove(GenericItem *i)
{
	for (std::list<GroupElement*>::iterator hit = subhierarchy.begin(); hit != subhierarchy.end(); ++hit)
	{
		if (*hit == i)
		{
			subhierarchy.erase(hit);
			return;
		}
	}

	for (std::list<DesignElement*>::iterator eit = elements.begin(); eit != elements.end(); ++eit)
	{
		if (*eit == i)
		{
			elements.erase(eit);
			return;
		}
	}
}


bool GroupElement::convertCoordinateSystem()
{
	//Important : check that the object is not up to date, to avoid infinite loops
	if (isCoordinateSystemUpToDate)
		return true;

	if (!GenericItem::convertCoordinateSystem())
		return false;
	for (std::list<DesignElement*>::iterator eit = elements.begin(); eit != elements.end(); ++eit)
		if (!(*eit)->convertCoordinateSystem())
			return false;
	for (std::list<GroupElement*>::iterator hit = subhierarchy.begin(); hit != subhierarchy.end(); ++hit)
		if (!(*hit)->convertCoordinateSystem())
			return false;
	return true;
}

GenericItem* GroupElement::scan(const char* str)
{
	//scan all elements contained in this group, beginning with this one, while none matches the requested name
	GenericItem *item = GenericItem::scan(str);
	for (std::list<DesignElement*>::iterator eit = elements.begin(); eit != elements.end() && !item; ++eit)
		item = (*eit)->scan(str);
	for (std::list<GroupElement*>::iterator hit = subhierarchy.begin(); hit != subhierarchy.end() && !item; ++hit)
		item = (*hit)->scan(str);
	return item;
}

bool GroupElement::scan(Token t, std::vector<GenericItem*> &items)
{
	GenericItem::scan(t, items);
	size_t size = items.size();
	for (std::list<DesignElement*>::iterator eit = elements.begin(); eit != elements.end(); ++eit)
		(*eit)->scan(t, items);
	for (std::list<GroupElement*>::iterator hit = subhierarchy.begin(); hit != subhierarchy.end(); ++hit)
		(*hit)->scan(t, items);
	return (items.size() > size);
}

std::pair<int, int> GroupElement::write(std::ostream &output, int nbtabs) const
{
	{
		for (int i = 0; i < nbtabs; i++)
			output << "\t";
	}
	output << "NEW ";

	switch (level)
	{
	case PDMS_GROUP: output << "GROUP"; break;
	case PDMS_WORLD: output << "WORLD"; break;
	case PDMS_SITE: output << "SITE"; break;
	case PDMS_ZONE: output << "ZONE"; break;
	case PDMS_EQUIPMENT: output << "EQUIPMENT"; break;
	case PDMS_STRUCTURE: output << "STRUCTURE"; break;
	case PDMS_SUBSTRUCTURE: output << "SUBSTRUCTURE"; break;
	default:
		std::cout << "Error : cannot write group " << level << std::endl;
		return std::pair<int, int>(0, 0);
	}

	if (strlen(name))
		output << " /" << name;
	output << std::endl;

	std::pair<int, int> nb(1, 0);

	for (std::list<GroupElement*>::const_iterator hit = subhierarchy.begin(); hit != subhierarchy.end(); ++hit)
	{
		std::pair<int, int> n = (*hit)->write(output, nbtabs + 1);
		nb.first += n.first;
		nb.second += n.second;
	}

	for (std::list<DesignElement*>::const_iterator eit = elements.begin(); eit != elements.end(); ++eit)
	{
		std::pair<int, int> n = (*eit)->write(output, nbtabs + 1);
		nb.first += n.first;
		nb.second += n.second;
	}

	{
		for (int i = 0; i < nbtabs; i++)
			output << "\t";
	}
	output << "END" << std::endl;
	return nb;
}

bool SCylinder::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_DIAMETER: diameter = value; break;
	case PDMS_HEIGHT: height = value; break;
	case PDMS_X_TOP_SHEAR: xtshear = value; if (fabs(xtshear) > 90.) return false; break;
	case PDMS_Y_TOP_SHEAR: ytshear = value; if (fabs(ytshear) > 90.) return false; break;
	case PDMS_X_BOTTOM_SHEAR: xbshear = value; if (fabs(xbshear) > 90.) return false; break;
	case PDMS_Y_BOTTOM_SHEAR: ybshear = value; if (fabs(ybshear) > 90.) return false; break;
	default: return false;
	}
	return true;
}

PointCoordinateType SCylinder::surface() const
{
	return static_cast<PointCoordinateType>(M_PI)*diameter*height;
}

std::pair<int, int> SCylinder::write(std::ostream &output, int nbtabs) const
{
	int i;
	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "NEW SLCYLINDER";
	if (strlen(name))
		output << " /" << name;
	output << std::endl;

	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "DIAMETER " << diameter << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "HEIGHT " << height << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "XTSHEAR " << (CC_RAD_TO_DEG)*xtshear << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "XBSHEAR " << (CC_RAD_TO_DEG)*xbshear << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "YTSHEAR " << (CC_RAD_TO_DEG)*ytshear << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "YBSHEAR " << (CC_RAD_TO_DEG)*ybshear << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "AT X " << position[0] << " Y " << position[1] << " Z " << position[2] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ORI ";
	output << "X is X " << orientation[0][0] << " Y " << orientation[0][1] << " Z " << orientation[0][2];
	output << " AND Z is X " << orientation[2][0] << " Y " << orientation[2][1] << " Z " << orientation[2][2] << std::endl;

	for (i = 0; i < nbtabs; i++)
		output << "\t";

	output << "END" << std::endl;

	return std::pair<int, int>(0, 1);
}

bool CTorus::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_ANGLE: angle = value; if (fabs(angle) > (2.*M_PI)) return false; break;
	case PDMS_INSIDE_RADIUS: inside_radius = value; break;
	case PDMS_OUTSIDE_RADIUS: outside_radius = value; break;
	default: return false;
	}
	return true;
}

PointCoordinateType CTorus::surface() const
{
	PointCoordinateType r = static_cast<PointCoordinateType>(0.5)*(outside_radius - inside_radius);
	PointCoordinateType R = outside_radius - r;
	return (angle / static_cast<PointCoordinateType>(2.0*M_PI))*(static_cast<PointCoordinateType>(4.0*PDMS_SQR(M_PI))*r*R);
}

std::pair<int, int> CTorus::write(std::ostream &output, int nbtabs) const
{
	int i;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "NEW CTORUS";
	if (strlen(name))
		output << " /" << name;
	output << std::endl;

	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "RINSIDE " << inside_radius << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ROUTSIDE " << outside_radius << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ANGLE " << static_cast<PointCoordinateType>(CC_RAD_TO_DEG)*angle << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "AT X " << position[0] << " Y " << position[1] << " Z " << position[2] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ORI ";
	output << "X is X " << orientation[0][0] << " Y " << orientation[0][1] << " Z " << orientation[0][2];
	output << " AND Z is X " << orientation[2][0] << " Y " << orientation[2][1] << " Z " << orientation[2][2] << std::endl;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "END" << std::endl;

	return std::pair<int, int>(0, 1);
}

bool RTorus::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_ANGLE: angle = value; if (fabs(angle) > (2.*M_PI)) return false; break;
	case PDMS_INSIDE_RADIUS: inside_radius = value; break;
	case PDMS_OUTSIDE_RADIUS: outside_radius = value; break;
	case PDMS_HEIGHT: height = value; break;
	default: return false;
	}
	return true;
}

PointCoordinateType RTorus::surface() const
{
	PointCoordinateType inside = static_cast<PointCoordinateType>(2.0*M_PI)*inside_radius*height;
	PointCoordinateType outside = static_cast<PointCoordinateType>(2.0*M_PI)*outside_radius*height;
	PointCoordinateType updown = static_cast<PointCoordinateType>(2.0*M_PI)*(PDMS_SQR(outside_radius) - PDMS_SQR(inside_radius));
	return (angle / static_cast<PointCoordinateType>(2.0*M_PI))*(inside + outside + updown);
}

std::pair<int, int> RTorus::write(std::ostream &output, int nbtabs) const
{
	int i;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "NEW RTORUS";
	if (strlen(name))
		output << " /" << name;
	output << std::endl;

	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "RINSIDE " << inside_radius << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ROUTSIDE " << outside_radius << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "HEIGHT " << height << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ANGLE " << static_cast<PointCoordinateType>(CC_RAD_TO_DEG)*angle << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "AT X " << position[0] << " Y " << position[1] << " Z " << position[2] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ORI ";
	output << "X is X " << orientation[0][0] << " Y " << orientation[0][1] << " Z " << orientation[0][2];
	output << " AND Z is X " << orientation[2][0] << " Y " << orientation[2][1] << " Z " << orientation[2][2] << std::endl;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "END" << std::endl;

	return std::pair<int, int>(0, 1);
}

Dish::Dish()
	: diameter(0)
	, height(0)
	, radius(0)
{
}

bool Dish::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_HEIGHT: height = value; break;
	case PDMS_RADIUS: radius = value; break;
	case PDMS_DIAMETER: diameter = value; break;
	default: return false;
	}
	return true;
}

PointCoordinateType Dish::surface() const
{
	if (radius > ZERO_TOLERANCE)
	{
		PointCoordinateType r = static_cast<PointCoordinateType>(0.5f*diameter);
		if (fabs(2 * height - diameter) < ZERO_TOLERANCE)
			return static_cast<PointCoordinateType>(2.0*M_PI)*PDMS_SQR(r);
		if (2 * height > diameter)
		{
			PointCoordinateType a = acos(r / height);
			return static_cast<PointCoordinateType>(M_PI)*(PDMS_SQR(r) + (a*r*height / sin(a)));
		}
		else
		{
			PointCoordinateType a = acos(height / r);
			return static_cast<PointCoordinateType>(M_PI)*(PDMS_SQR(r) + ((PDMS_SQR(height) / sin(a))*log((1 + sin(a)) / cos(a))));
		}
	}
	return static_cast<PointCoordinateType>(M_PI)*diameter*height;
}

std::pair<int, int> Dish::write(std::ostream &output, int nbtabs) const
{
	int i;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "NEW DISH";
	if (strlen(name))
		output << " /" << name;
	output << std::endl;

	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "HEIGHT " << height << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "RADIUS " << radius << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "DIAMETER " << diameter << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "AT X " << position[0] << " Y " << position[1] << " Z " << position[2] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ORI ";
	output << "X is X " << orientation[0][0] << " Y " << orientation[0][1] << " Z " << orientation[0][2];
	output << " AND Z is X " << orientation[2][0] << " Y " << orientation[2][1] << " Z " << orientation[2][2] << std::endl;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "END" << std::endl;

	return std::pair<int, int>(0, 1);
}

bool Cone::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_TOP_DIAMETER: dtop = value; break;
	case PDMS_BOTTOM_DIAMETER: dbottom = value; break;
	case PDMS_HEIGHT: height = value; break;
	default: return false;
	}
	return true;
}

PointCoordinateType Cone::surface() const
{
	PointCoordinateType r1;
	PointCoordinateType r2;
	if (dtop < dbottom)
	{
		r1 = dtop;
		r2 = dbottom;
	}
	else
	{
		r1 = dbottom;
		r2 = dtop;
	}

	PointCoordinateType h1 = (r1*height) / (r2 - r1);
	PointCoordinateType a1 = static_cast<PointCoordinateType>(M_PI)*r1*sqrt(PDMS_SQR(r1) + PDMS_SQR(h1));
	PointCoordinateType a2 = static_cast<PointCoordinateType>(M_PI)*r2*sqrt(PDMS_SQR(r2) + PDMS_SQR(h1 + height));

	return a2 - a1;
}

std::pair<int, int> Cone::write(std::ostream &output, int nbtabs) const
{
	int i;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "NEW CONE";
	if (strlen(name))
		output << " /" << name;
	output << std::endl;

	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "HEIGHT " << height << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "DBOTTOM " << dbottom << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "DTOP " << dtop << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "AT X " << position[0] << " Y " << position[1] << " Z " << position[2] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ORI ";
	output << "X is X " << orientation[0][0] << " Y " << orientation[0][1] << " Z " << orientation[0][2];
	output << " AND Z is X " << orientation[2][0] << " Y " << orientation[2][1] << " Z " << orientation[2][2] << std::endl;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "END" << std::endl;

	return std::pair<int, int>(0, 1);
}

Box::Box()
	: lengths(0, 0, 0)
{
}

bool Box::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_XLENGTH:
		lengths[0] = value;
		break;
	case PDMS_YLENGTH:
		lengths[1] = value;
		break;
	case PDMS_ZLENGTH:
		lengths[2] = value;
		break;
	default: return false;
	}
	return true;
}

PointCoordinateType Box::surface() const
{
	return 2 * ((lengths[0] * lengths[1]) + (lengths[1] * lengths[2]) + (lengths[2] * lengths[0]));
}

std::pair<int, int> Box::write(std::ostream &output, int nbtabs) const
{
	int i;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	if (negative)
		output << "NEW NBOX";
	else
		output << "NEW BOX";
	if (strlen(name))
		output << " /" << name;
	output << std::endl;

	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "XLENGTH " << lengths[0] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "YLENGTH " << lengths[1] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ZLENGTH " << lengths[2] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "AT X " << position[0] << " Y " << position[1] << " Z " << position[2] << std::endl;
	for (i = 0; i <= nbtabs; i++)
		output << "\t";
	output << "ORI ";
	output << "X is X " << orientation[0][0] << " Y " << orientation[0][1] << " Z " << orientation[0][2];
	output << " AND Z is X " << orientation[2][0] << " Y " << orientation[2][1] << " Z " << orientation[2][2] << std::endl;

	for (i = 0; i < nbtabs; i++)
		output << "\t";
	output << "END" << std::endl;

	return std::pair<int, int>(0, 1);
}

std::pair<int, int> Vertex::write(std::ostream &output, int nbtabs) const
{
	return std::pair<int, int>(0, 0);
}

bool Loop::push(GenericItem *i)
{
	if (i->getType() == PDMS_VERTEX)
	{
		loop.push_back(dynamic_cast<Vertex*>(i));
		if (i->owner)
			i->owner->remove(i);
		i->owner = this;
		return true;
	}
	return false;
}

void Loop::remove(GenericItem *i)
{
	for (std::list<Vertex*>::iterator it = loop.begin(); it != loop.end(); )
	{
		if ((*it) == i)
			loop.erase(it);
		else
			++it;
	}
}

std::pair<int, int> Loop::write(std::ostream &output, int nbtabs) const
{
	return std::pair<int, int>(0, 0);
}

bool Extrusion::push(GenericItem *l)
{
	if (l->getType() == PDMS_LOOP)
	{
		if (loop)
			return false;
		loop = dynamic_cast<Loop*>(l);
		if (l->owner)
			l->owner->remove(l);
		l->owner = this;

		return true;
	}

	return DesignElement::push(l);
}

PointCoordinateType Extrusion::surface() const
{
	PointCoordinateType p = 0;
	if (loop)
	{
		std::list<Vertex*>::const_iterator it1 = loop->loop.begin();
		std::list<Vertex*>::const_iterator it2 = it1; ++it2;
		while (it1 != loop->loop.end())
		{
			if (it2 == loop->loop.end())
				it2 = loop->loop.begin();
			p += ((*it1)->v - (*it2)->v).norm();
			++it1;
			++it2;
		}
	}

	return p * static_cast<PointCoordinateType>(height);
}

std::pair<int, int> Extrusion::write(std::ostream &output, int nbtabs) const
{
	return std::pair<int, int>(0, 0);
}

bool Pyramid::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_X_BOTTOM:xbot = value; break;
	case PDMS_Y_BOTTOM:ybot = value; break;
	case PDMS_X_TOP:xtop = value; break;
	case PDMS_Y_TOP:ytop = value; break;
	case PDMS_X_OFF:xoff = value; break;
	case PDMS_Y_OFF:yoff = value; break;
	case PDMS_HEIGHT:height = value; break;
	default:
		return false;
	}
	return true;
}

PointCoordinateType Pyramid::surface() const
{
	//TODO
	return 0;
}

std::pair<int, int> Pyramid::write(std::ostream &output, int nbtabs) const
{
	return std::pair<int, int>(0, 0);
}

bool Snout::setValue(Token t, PointCoordinateType value)
{
	switch (t)
	{
	case PDMS_BOTTOM_DIAMETER:dbottom = value; break;
	case PDMS_TOP_DIAMETER:dtop = value; break;
	case PDMS_X_OFF:xoff = value; break;
	case PDMS_Y_OFF:yoff = value; break;
	case PDMS_HEIGHT:height = value; break;
	default:
		return false;
	}
	return true;
}

PointCoordinateType Snout::surface() const
{
	//TODO
	return 0;
}

std::pair<int, int> Snout::write(std::ostream &output, int nbtabs) const
{
	return std::pair<int, int>(0, 0);
}
