#include <xiot/X3DDataTypeFactory.h>

#include <xiot/X3DParseException.h>

#include <sstream>
#include <algorithm>
#include <cctype>

using namespace std;

#define isWhiteSpace(c) ((c) == ' ' || (c) == '\t' || (c) == '\v' ||  (c) == '\n' || (c) == '\r' || (c) == '\f') 
#define isWhiteSpaceOrComma(c) (isWhiteSpace((c)) || ((c) == ','))

namespace XIOT {

	bool X3DDataTypeFactory::getSFBoolFromString(const std::string &s){
		std::string lower(s);
		std::transform(lower.begin(), lower.end(), lower.begin(), (int(*)(int)) std::tolower);

		if (lower == "true")
			return true;
		if (lower == "false")
			return false;

		std::stringstream reason("Unknown value for SFBool: ");
		reason << s;
		throw X3DParseException(reason.str());
	}

	float X3DDataTypeFactory:: getSFFloatFromString(const std::string &s){
		std::stringstream ss;
		float f;
		
		ss << s;
		ss >> f;

		return f;
	}

	int X3DDataTypeFactory::getSFInt32FromString(const std::string &s){
		std::stringstream ss;
		int i;
		
		ss << s;
		ss >> i;

		return i;
	}

	void X3DDataTypeFactory::getSFVec3fFromString(const std::string &s, SFVec3f& vec){
		std::stringstream ss;
		ss << s;
		ss >> vec.x >> vec.y >> vec.z;
	}

	void X3DDataTypeFactory::getSFVec2fFromString(const std::string &s, SFVec2f &vec){
		std::stringstream ss;
		ss << s;
		ss >> vec.x >> vec.y;
	}

	void X3DDataTypeFactory::getSFRotationFromString(const std::string &s, SFRotation &rot){
		std::stringstream ss;
		ss << s;
		ss >> rot.x >> rot.y >> rot.z >> rot.angle;
	}

	void X3DDataTypeFactory::getSFStringFromString(const SFString &s, SFString &value){
		value.assign(s);
	}

	void X3DDataTypeFactory::getSFColorFromString(const std::string &s, SFColor &col){
		std::stringstream ss;
		ss << s;
		ss >> col.r >> col.g >> col.b;
	}

	void X3DDataTypeFactory::getSFColorRGBAFromString(const std::string &s, SFColorRGBA &col){
		std::stringstream ss;
    ss << s;
		ss >> col.r >> col.g >> col.b >> col.a;
	}

	void X3DDataTypeFactory::getSFImageFromString(const std::string &s, SFImage &value){
		std::stringstream ss;
		SFImage img;
		unsigned int	tmp, index = 0;

    
		ss << s;

		// read width, height and components
		while(!(ss.eof() || ss.fail()) && index < 3)
      {
			ss >> tmp;	
      img.push_back(tmp);
      index++;
      }

     while(!(ss.eof() || ss.fail()))
      {
      ss >> std::hex >> tmp;	
      if (!ss.fail())
        img.push_back(tmp);
      }


		std::swap(img, value);
	} 

	// Multi Field
	void X3DDataTypeFactory::getMFFloatFromString(const std::string &s, MFFloat &value){
		std::vector<float> vec;
		std::stringstream ss;
		
		float fTemp;
		 
		ss << s;

		// in case of a parsing error, ss.fail() will return true
		while(!(ss.eof() || ss.fail()))
		{
			ss >> fTemp;
			
			// look for WS or ',' and skip it
			int c = ss.peek();
			while(isWhiteSpaceOrComma(c))
			{
				ss.ignore(1);
				c = ss.peek();
			}
			vec.push_back(fTemp);
		}		
		std::swap(vec, value);
	}

	void X3DDataTypeFactory::getMFInt32FromString(const std::string &s, MFInt32 &value){
		MFInt32 vec;
    std::istringstream ss(s, istringstream::in);
		int iTemp;


		// in case of a parsing error, ss.fail() will return true
		while(!(ss.eof() || ss.fail()))
		{
			ss >> iTemp;

			int c = ss.peek();
			while(isWhiteSpaceOrComma(c))
			{
				ss.ignore(1);
				c = ss.peek();
			}
			
			vec.push_back(iTemp);
		}		
		std::swap(vec, value);
	}

	void X3DDataTypeFactory::getMFVec3fFromString(const std::string &s, MFVec3f &value){
		MFVec3f vec;
		std::stringstream ss;
		
		SFVec3f tempVec;
		 
		ss << s;

		// in case of a parsing error, ss.fail() will return true
		while(!(ss.eof() || ss.fail()))
		{

			ss >> tempVec.x >> tempVec.y >> tempVec.z;
			
			char c = static_cast<char>(ss.peek());
			while(isWhiteSpaceOrComma(c))
			{
				ss.ignore();
				c = static_cast<char>(ss.peek());
			}
						
			vec.push_back(tempVec);
		}		
		std::swap(vec, value);
	}

	void X3DDataTypeFactory::getMFVec2fFromString(const std::string &s, MFVec2f &value){
		MFVec2f vec;
		std::stringstream ss(s);
		
		char c;
		// in case of a parsing error, ss.fail() will return true
		while(!(ss.eof() || ss.fail()))
		{
			vec.resize(vec.size()+1);
			ss >> vec.back().x >> vec.back().y;
			
			c = static_cast<char>(ss.peek());	// look for ',' and skip it
			if(c == ',')
				ss.ignore();
		}		
		std::swap(vec, value);
	}

	void X3DDataTypeFactory::getMFRotationFromString(const std::string &s, MFRotation &value){
		MFRotation vec;
		std::stringstream ss;
		SFRotation tempRot;
		char c;
	 
		ss << s;

		// in case of a parsing error, ss.fail() will return true
		while(!(ss.eof() || ss.fail()))
		{
			ss >> tempRot.x >> tempRot.y >> tempRot.z >> tempRot.angle;
			
			c = static_cast<char>(ss.peek());	// look for ',' and skip it
			if(c == ',')
				ss.ignore();
			
			vec.push_back(tempRot);
		}		
		std::swap(vec, value);
	}

	void X3DDataTypeFactory::getMFStringFromString(const std::string &s, MFString &value){
		
    MFString result;

    const char* rawStr = s.c_str();
    while (*rawStr)
      {
      bool inStr = false;
      std::string oneString;
      while (*rawStr && !inStr)
        {
        inStr = *rawStr=='"';
        rawStr++;
        }
      while(*rawStr && inStr)
        {
        inStr = *rawStr!='"';
        if(inStr)
          oneString.append(rawStr,1);
        rawStr++;
        }
      if (!oneString.empty())
        result.push_back(oneString);
      }
    
		std::swap(result, value);
	}

	void X3DDataTypeFactory::getMFColorFromString(const std::string &s, MFColor &value){
		MFColor vec;
		std::stringstream ss;
		
		SFColor tempColor;
		 
		ss << s;

		// in case of a parsing error, ss.fail() will return true
		while(!(ss.eof() || ss.fail()))
		{
			ss >> tempColor.r >> tempColor.g >> tempColor.b;
			
			// look for ',' and skip it
			int c = ss.peek();
			while(isWhiteSpaceOrComma(c))
			{
				ss.ignore(1);
				c = ss.peek();
			}
			vec.push_back(tempColor);
		}		
		std::swap(vec, value);
	}

	void X3DDataTypeFactory::getMFColorRGBAFromString(const std::string &s, MFColorRGBA &value){
		MFColorRGBA vec;
		std::stringstream ss;
		
		SFColorRGBA tempColor;
		 
		ss << s;

		// in case of a parsing error, ss.fail() will return true
		while(!(ss.eof() || ss.fail()))
		{
			ss >> tempColor.r >> tempColor.g >> tempColor.b >> tempColor.a;
			
			// look for ',' and skip it
			int c = ss.peek();
			while(isWhiteSpaceOrComma(c))
			{
				ss.ignore(1);
				c = ss.peek();
			}
			vec.push_back(tempColor);
		}		
		std::swap(vec, value);
	}

}

