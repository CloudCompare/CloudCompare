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

#ifndef PDMS_TOOLS_HEADER
#define PDMS_TOOLS_HEADER

//CCLib
#include <CCConst.h>
#include <CCGeom.h>

//system
#include <cstring>
#include <list>
#include <ostream>
#include <vector>

namespace PdmsTools
{
	const int c_max_str_length = 2048;

	//! Tokens (entities ID)
	enum Token
	{
		// DO NOT MODIFIY TOKENS ORDER !!!!!!!
		//Some token for parser use
		PDMS_INVALID_TOKEN = 0,
		PDMS_UNKNOWN,
		PDMS_EOS,
		PDMS_UNUSED,
		PDMS_COMMENT_LINE,
		PDMS_COMMENT_BLOCK,
		PDMS_NAME_STR,
		//Some keywords
		PDMS_IS,
		PDMS_AND,
		PDMS_NUM_VALUE,
		//PDMS commands
		PDMS_NAME,
		PDMS_OWNER,
		PDMS_WRT,
		PDMS_CREATE,
		PDMS_END,
		PDMS_RETURN,
		PDMS_LAST,
		PDMS_ENTER_METAGROUP,
		PDMS_LEAVE_METAGROUP,
		//Coordinates systems
		PDMS_EST,
		PDMS_NORTH,
		PDMS_UP,
		PDMS_WEST,
		PDMS_SOUTH,
		PDMS_DOWN,
		PDMS_X,
		PDMS_Y,
		PDMS_Z,
		//PDMS hierarchy
		PDMS_GROUP,
		PDMS_LOWER_HIER_LEVEL, //purely virtual (non PDMS token)
		PDMS_WORLD,
		PDMS_SITE,
		PDMS_ZONE,
		PDMS_EQUIPMENT,
		PDMS_STRUCTURE,
		PDMS_SUBSTRUCTURE,
		//PDMS elements
		PDMS_SCYLINDER,
		PDMS_CTORUS,
		PDMS_RTORUS,
		PDMS_DISH,
		PDMS_CONE,
		PDMS_PYRAMID,
		PDMS_SNOUT,
		PDMS_BOX,
		PDMS_NBOX,
		PDMS_EXTRU,
		PDMS_NEXTRU,
		PDMS_LOOP,
		PDMS_VERTEX,
		//Attributes
		PDMS_DIAMETER,
		PDMS_HEIGHT,
		PDMS_X_TOP_SHEAR,
		PDMS_X_BOTTOM_SHEAR,
		PDMS_Y_TOP_SHEAR,
		PDMS_Y_BOTTOM_SHEAR,
		PDMS_X_BOTTOM,
		PDMS_Y_BOTTOM,
		PDMS_X_TOP,
		PDMS_Y_TOP,
		PDMS_X_OFF,
		PDMS_Y_OFF,
		PDMS_XLENGTH,
		PDMS_YLENGTH,
		PDMS_ZLENGTH,
		PDMS_ANGLE,
		PDMS_RADIUS,
		PDMS_INSIDE_RADIUS,
		PDMS_OUTSIDE_RADIUS,
		PDMS_TOP_DIAMETER,
		PDMS_BOTTOM_DIAMETER,
		//Coordinates elements
		PDMS_POSITION,
		PDMS_ORIENTATION,
		//Units system
		PDMS_METRE,
		PDMS_MILLIMETRE
	};

	namespace PdmsToken
	{
		static bool isCommand(Token t) { return (t >= PDMS_NAME && t <= PDMS_RETURN); }
		static bool isDesignElement(Token t) { return (t >= PDMS_SCYLINDER && t <= PDMS_VERTEX); }
		static bool isGroupElement(Token t) { return (t >= PDMS_GROUP && t <= PDMS_SUBSTRUCTURE); }
		static bool isElement(Token t) { return (isDesignElement(t) || isGroupElement(t)); }
		static bool isCoordinate(Token t) { return (t >= PDMS_EST && t <= PDMS_Z); }
		static bool isParameter(Token t) { return (t >= PDMS_DIAMETER && t <= PDMS_BOTTOM_DIAMETER); }
		static bool isSpaceSystem(Token t) { return (t >= PDMS_POSITION && t <= PDMS_ORIENTATION); }
		static bool isUnit(Token t) { return (t == PDMS_MILLIMETRE || t == PDMS_METRE); }
	};

	namespace PdmsObjects
	{
		//! Generic item
		class GenericItem
		{
		public:
			//! Parent imte
			GenericItem *owner;
			//! Creator item
			GenericItem *creator;

			//! Object position
			CCVector3 position;
			//! Object orientation (X,Y and Z)
			CCVector3 orientation[3];
			//! Coordinate system update flag
			bool isCoordinateSystemUpToDate;

			//! Reference object (position)
			GenericItem *positionReference;
			//! Reference(s) object(s) (orientation)
			GenericItem *orientationReferences[3];

			//! Name
			char name[c_max_str_length];

			//! Default constructor
			GenericItem();
			//! Destructor
			virtual ~GenericItem() = default;

			//coordinate system
			virtual bool setPosition(const CCVector3 &p);
			virtual bool setOrientation(const CCVector3 &x, const CCVector3 &y, const CCVector3 &z);
			virtual bool convertCoordinateSystem();

			//tree structure
			virtual GenericItem* getRoot() { GenericItem *item = this; while (item->owner) item = item->owner; return item; }
			virtual bool push(GenericItem *i) = 0;
			virtual void remove(GenericItem *i) {}

			//type management
			virtual bool isGroupElement() { return false; }
			virtual bool isDesignElement() { return false; }
			virtual Token getType() const { return PDMS_INVALID_TOKEN; }

			//attributes
			virtual bool setValue(Token t, PointCoordinateType value) { return false; }

			//other
			virtual GenericItem* scan(const char* str) { return strcmp(name, str) == 0 ? this : nullptr; }
			virtual bool scan(Token t, std::vector<GenericItem*> &array);
			virtual std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const = 0;

		protected:
			bool isOrientationValid(unsigned i) const;
			bool completeOrientation();
		};

		//! Item stack
		class Stack
		{
		public:
			static void Init();
			static void Clear();
			static void Destroy(GenericItem* &item);
		};

		//! Design element
		class DesignElement : public GenericItem
		{
		public:
			bool negative;
			std::list<DesignElement*> nelements;

			DesignElement() : negative(false) {}
			~DesignElement() override;

			//reimplemented from GenericItem
			bool isDesignElement() override { return true; }
			bool push(GenericItem *i) override;
			void remove(GenericItem *i) override;

			//virtual Shape* toShape() {return nullptr;}
			virtual PointCoordinateType surface() const { return 0; }
		};

		//! Group of elements
		class GroupElement : public GenericItem
		{
		public:
			Token level;
			std::list<DesignElement*> elements;
			std::list<GroupElement*> subhierarchy;

			explicit GroupElement(Token l);
			~GroupElement() override;

			//GroupElement(const Model* model);
			//virtual bool push(const Shape* shape);

			virtual void clear(bool del = false);

			//reimplemented from GenericItem
			bool push(GenericItem *i) override;
			void remove(GenericItem *i) override;
			bool isGroupElement() override { return true; }
			bool convertCoordinateSystem() override;
			GenericItem* scan(const char* str) override;
			bool scan(Token t, std::vector<GenericItem*> &array) override;
			Token getType() const override { return level; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
		};

		//! Cylinder
		class SCylinder : public DesignElement
		{
		public:
			PointCoordinateType diameter;
			PointCoordinateType height;
			PointCoordinateType xtshear;
			PointCoordinateType ytshear;
			PointCoordinateType xbshear;
			PointCoordinateType ybshear;

			SCylinder()
				: diameter(0)
				, height(0)
				, xtshear(0)
				, ytshear(0)
				, xbshear(0)
				, ybshear(0)
			{}

			//virtual Shape* toShape();

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return PDMS_SCYLINDER; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		//! Torus (circular section)
		class CTorus : public DesignElement
		{
		public:
			PointCoordinateType inside_radius;
			PointCoordinateType outside_radius;
			PointCoordinateType angle;

			CTorus() : inside_radius(0), outside_radius(0), angle(0) {}

			//virtual Shape* toShape();

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return PDMS_CTORUS; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		// Torus (rectangular section)
		class RTorus : public CTorus
		{
		public:
			PointCoordinateType height;

			RTorus() : CTorus(), height(0) {}

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return PDMS_RTORUS; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		//! Dish
		class Dish : public DesignElement
		{
		public:
			PointCoordinateType diameter;
			PointCoordinateType height;
			PointCoordinateType radius;

			Dish();

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return PDMS_DISH; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		//! Cone
		class Cone : public DesignElement
		{
		public:
			PointCoordinateType dtop;
			PointCoordinateType dbottom;
			PointCoordinateType height;

			Cone() : dtop(0), dbottom(0), height(0) {}

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return PDMS_CONE; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		//! Pyramid
		class Pyramid : public DesignElement
		{
		public:
			PointCoordinateType xbot, xtop, xoff, ybot, ytop, yoff, height;

			Pyramid() = default;

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return PDMS_PYRAMID; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		//! Snout
		class Snout : public Cone
		{
		public:
			PointCoordinateType xoff, yoff;

			Snout() : Cone(), xoff(0), yoff(0) {}

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return PDMS_SNOUT; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		//! Box
		class Box : public DesignElement
		{
		public:
			CCVector3 lengths;

			Box();

			//reimplemented from GenericItem
			bool setValue(Token t, PointCoordinateType value) override;
			Token getType() const override { return negative ? PDMS_NBOX : PDMS_BOX; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

		//! 2D vertex
		class Vertex : public DesignElement
		{
		public:
			CCVector2 v;

			Vertex() : v(0.) {}

			//reimplemented from GenericItem
			bool setPosition(const CCVector3 &p) override { v.x = p.x; v.y = p.y; return true; }
			Token getType() const override { return PDMS_VERTEX; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
		};

		//! Loop
		class Loop : public DesignElement
		{
		public:
			std::list<Vertex*> loop;

			Loop() = default;
			~Loop() override
			{
				while (!loop.empty())
				{
					GenericItem* v = loop.back();
					Stack::Destroy(v);
					loop.pop_back();
				}
			}

			//reimplemented from GenericItem
			bool push(GenericItem *i) override;
			void remove(GenericItem *i) override;
			Token getType() const override { return PDMS_LOOP; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
		};

		//! Extrusion
		class Extrusion : public DesignElement
		{
		public:
			Loop *loop;
			PointCoordinateType height;

			Extrusion() : loop(nullptr), height(0.0f) {}
			~Extrusion() override { if (loop) { GenericItem* i = loop; Stack::Destroy(i); } }

			//reimplemented from GenericItem
			bool push(GenericItem *l) override;
			void remove(GenericItem *i) override { if (loop == i) loop = nullptr; }
			bool setValue(Token t, PointCoordinateType value) override { if (t == PDMS_HEIGHT) { height = value; return true; } return false; }
			Token getType() const override { return negative ? PDMS_NEXTRU : PDMS_EXTRU; }
			std::pair<int, int> write(std::ostream &output, int nbtabs = 0) const override;
			PointCoordinateType surface() const override;
		};

	};


	namespace PdmsCommands
	{
		class Command
		{
		public:
			Token command;

			explicit Command(Token t) { command = t; }
			Command(const Command &com) { command = com.command; }
			virtual ~Command() = default;

			//! Factory
			static Command* Create(Token t);

			virtual bool handle(PointCoordinateType numvalue) { return false; }
			virtual bool handle(const char* str) { return false; }
			virtual bool handle(Token t) { return false; }
			virtual bool isValid() const { return false; }
			virtual bool execute(PdmsObjects::GenericItem* &item) const { return false; }
		};

		class NumericalValue : public Command
		{
		public:
			PointCoordinateType value;
			int valueChanges;

			explicit NumericalValue(Token t) : Command(t), valueChanges(0) {}
			bool handle(PointCoordinateType numvalue) override;
			bool isValid() const override;
			virtual PointCoordinateType getValue() const;
			bool execute(PdmsObjects::GenericItem* &item) const override;
		};

		class DistanceValue : public NumericalValue
		{
		public:
			Token unit;
			static Token workingUnit;

			explicit DistanceValue(Token t = PDMS_INVALID_TOKEN) : NumericalValue(t), unit(PDMS_INVALID_TOKEN) {}
			static void setWorkingUnit(Token wu) { workingUnit = wu; }
			bool handle(Token t) override;
			bool handle(PointCoordinateType numvalue) override { return NumericalValue::handle(numvalue); }
			PointCoordinateType getValueInWorkingUnit() const;
			bool execute(PdmsObjects::GenericItem* &item) const override;
		};

		class Reference : public Command
		{
		public:
			Token token;
			char refname[c_max_str_length];

			explicit Reference(Token t = PDMS_INVALID_TOKEN) : Command(t), token(PDMS_INVALID_TOKEN) { memset(refname, 0, c_max_str_length); }
			Reference(const Reference &ref) : Command(ref), token(ref.token) { strcpy(refname, ref.refname); }
			Reference& operator=(const Reference &ref);
			bool handle(Token t) override;
			bool handle(const char* str) override;
			bool isValid() const override;
			virtual bool isNameReference() const;
			virtual bool isTokenReference() const;
			bool execute(PdmsObjects::GenericItem* &item) const override;

		protected:
			int isSet() const;
		};

		class Coordinates : public Command
		{
		public:
			DistanceValue coords[3];
			int current;

			explicit Coordinates(Token t = PDMS_INVALID_TOKEN) : Command(t) { current = -1; }
			bool handle(Token t) override;
			bool handle(PointCoordinateType numvalue) override;
			bool isValid() const override;
			bool getVector(CCVector3 &u) const;
			int getNbComponents(bool onlyset = false) const;
		};

		class Position : public Command
		{
		public:
			Coordinates position;
			Reference ref;
			Command *current;

			Position() : Command(PDMS_POSITION) { current = nullptr; }
			bool handle(Token t) override;
			bool handle(PointCoordinateType numvalue) override;
			bool handle(const char* str) override;
			bool isValid() const override;
			bool execute(PdmsObjects::GenericItem* &item) const override;
		};

		class Orientation : public Command
		{
		public:
			Coordinates orientation[3];
			Reference refs[3];
			Command *current;
			int component;

			Orientation() : Command(PDMS_ORIENTATION) { current = nullptr; component = -1; }
			bool handle(Token t) override;
			bool handle(PointCoordinateType numvalue) override;
			bool handle(const char* str) override;
			bool isValid() const override;
			bool getAxes(CCVector3 &x, CCVector3 &y, CCVector3 &z) const;
			bool execute(PdmsObjects::GenericItem* &item) const override;

		protected:
			int getNbComponents() const;
			static bool axisFromCoords(const Coordinates &coords, CCVector3 &u);
		};

		class Name : public Command
		{
		public:
			char name[c_max_str_length];

			Name() : Command(PDMS_NAME) { memset(name, 0, c_max_str_length); }
			
			bool handle(const char* str) override
			{
				if (strlen(name) > 0)
					return false;
				strcpy(name, str);
				return true;
			}
			
			bool isValid() const override
			{
				return strlen(name) > 0;
			}
			
			bool execute(PdmsObjects::GenericItem* &item) const override;
		};

		class ElementCreation : public Command
		{
		public:
			Token elementType;
			std::vector<std::string> path;

			ElementCreation() : Command(PDMS_CREATE) { elementType = PDMS_INVALID_TOKEN; }
			bool handle(const char*str) override;
			bool handle(Token t) override;
			bool isValid() const override;
			bool execute(PdmsObjects::GenericItem* &item) const override;

			static const char* GetDefaultElementName(Token token);

		protected:
			bool splitPath(const char *str);
		};

		class ElementEnding : public Command
		{
		public:
			Reference end;

			explicit ElementEnding(Token t = PDMS_END) : Command(t) {}
			bool handle(Token t) override { end.command = command; return end.handle(t); }
			bool handle(const char* str) override { end.command = command; return end.handle(str); }
			bool isValid() const override { if (!end.command) return true; return end.isValid(); }
			bool execute(PdmsObjects::GenericItem* &item) const override;
		};

		class HierarchyNavigation : public Command
		{
		public:
			explicit HierarchyNavigation(Token t) : Command(t) {}
			bool isValid() const override { return (PdmsToken::isGroupElement(command)); }
			bool execute(PdmsObjects::GenericItem* &item) const override;
		};
	};
};

#endif //PDMS_TOOLS_HEADER
