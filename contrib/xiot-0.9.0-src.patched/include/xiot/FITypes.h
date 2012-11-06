/*=========================================================================
     This file is part of the XIOT library.

     Copyright (C) 2008-2009 EDF R&D
     Author: Kristian Sons (xiot@actor3d.com)

     This library is free software; you can redistribute it and/or modify
     it under the terms of the GNU Lesser Public License as published by
     the Free Software Foundation; either version 2.1 of the License, or
     (at your option) any later version.

     The XIOT library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU Lesser Public License for more details.

     You should have received a copy of the GNU Lesser Public License
     along with XIOT; if not, write to the Free Software
     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
     MA 02110-1301  USA
=========================================================================*/
#ifndef FI_FITYPES_H
#define FI_FITYPES_H

#include <xiot/FIConfig.h>

#include <vector>
#include <sstream>
#include <stdexcept>
#include <exception>

namespace FI {

	/// Constant that identifies that there is no index set for
	/// a table and that the ocetets need to be processed
	static const unsigned int INDEX_NOT_SET = 0;

	/// Possible encoding formats of the EncodedCharacterString
	enum EncodingFormat {
		ENCODINGFORMAT_UTF8,
		ENCODINGFORMAT_UTF16,
		ENCODINGFORMAT_RESTRICTED_ALPHABET,
		ENCODINGFORMAT_ENCODING_ALGORITHM
	} ;
	
	/** @defgroup ASN1Types ASN.1 types
	  * These are the implementations of the types 
	  * as defined in the chapter 7 of the 
	  * Fast InfoSet standard.
	  */



	/**
     * The NonEmptyOctetString type is:
	 * <code>NonEmptyOctetString ::= OCTET STRING (SIZE(1-four-gig))</code>
	 * @ingroup ASN1Types
     */
	typedef std::basic_string<unsigned char> NonEmptyOctetString;
	
	/**
     * 7.17 The EncodedCharacterString type
	 * @ingroup ASN1Types
     */
	struct EncodedCharacterString {
		EncodingFormat _encodingFormat;
		union {
			int _restrictedAlphabet;
			int _encodingAlgorithm;
		};
		NonEmptyOctetString _octets;
	};

	/**
	 * 7.13 The IdentifyingStringOrIndex type
	 * @ingroup ASN1Types
	 */
	struct IdentifyingStringOrIndex {
		IdentifyingStringOrIndex() : _stringIndex(INDEX_NOT_SET) {};
		NonEmptyOctetString _literalCharacterString;
		unsigned int _stringIndex;
	};

	/**
	 * 7.16 The QualifiedNameOrIndex type
	 * @ingroup ASN1Types
	 */
	struct QualifiedNameOrIndex {
		QualifiedNameOrIndex() : _nameSurrogateIndex(INDEX_NOT_SET) {};
		IdentifyingStringOrIndex _prefix;
		IdentifyingStringOrIndex _namespaceName;
		IdentifyingStringOrIndex _localName;
		unsigned int _nameSurrogateIndex;
	};

	/**
	 * 7.15 The NameSurrogate type
	 * @ingroup ASN1Types
	 */
	struct NameSurrogate {
		NameSurrogate() : _prefixStringIndex(INDEX_NOT_SET), _namespaceNameStringIndex(INDEX_NOT_SET), _localNameStringIndex(INDEX_NOT_SET) {};
		unsigned int _prefixStringIndex;
		unsigned int _namespaceNameStringIndex;
		unsigned int _localNameStringIndex;
	};
	
	/**
	 * 7.14 The NonIdentifyingStringOrIndex type
	 * @ingroup ASN1Types
	 */
	struct NonIdentifyingStringOrIndex {
		NonIdentifyingStringOrIndex() : _stringIndex(INDEX_NOT_SET), _addToTable(false) {};
		bool _addToTable;
		EncodedCharacterString _characterString;
		unsigned int _stringIndex;
	};



	/**
	 * 7.7 The CharacterChunk type
	 * @ingroup ASN1Types
	 */
	struct CharacterChunk {
		NonIdentifyingStringOrIndex characterCodes;
	};


	/**
	 * 7.4 The Attribute type
	 * @ingroup ASN1Types
	 */
	struct Attribute
	{
		QualifiedNameOrIndex _qualifiedName;
		NonIdentifyingStringOrIndex  _normalizedValue;
	};

	typedef std::vector<Attribute> Attributes;

	/**
	 * 7.3 The Element type
	 * @warning incomplete
	 * @ingroup ASN1Types
	 */
	struct Element
	{
		// std::vector<NamespaceAttribute> _namespaceAttribute;
		QualifiedNameOrIndex _qualifiedName;
		std::vector<Attribute> _attributes;
		// _children
	};

	/**
	 * 7.2 The Document type
	 * @ingroup ASN1Types
	 * @warning incomplete
	 */
	struct Document
	{

	};

	/**
	 * A resolved Qualified Name
	 */
	struct QualifiedName
	{
		QualifiedName(std::string prefix, std::string namespaceName, std::string localName) : _prefix(prefix), _namespaceName(namespaceName), _localName(localName) {};
		QualifiedName(std::string localName) : _prefix(""), _namespaceName(""), _localName(localName) {};
		std::string _prefix;
		std::string _namespaceName;
		std::string _localName;

		static inline std::string getQName(const std::string &p, const std::string &l) {
			if (p.empty()) return l;
			std::string result(p);
			return result.append(":").append(l);
		}
	};

	inline std::ostream& operator<<(std::ostream& o, const QualifiedName &qn)
	{
		return o << QualifiedName::getQName(qn._prefix, qn._localName).c_str();
	}



} // end namespace FI

#endif
