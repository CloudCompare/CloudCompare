#ifndef CHAISCRIPTING_BOOTSTRAP_GU_CLASSES_HPP
#define CHAISCRIPTING_BOOTSTRAP_GU_CLASSES_HPP

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScripting                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                     COPYRIGHT: Chris S Brown                           #
//#                                                                        #
//##########################################################################
#include <QtGui>
#include <QString>


#include <chaiscript/chaiscript.hpp>
#include <chaiscript/utility/utility.hpp>

#include "chaiscript_math.hpp"
#include "chaiscript_string_methods.hpp"


namespace chaiscript
{
	namespace cloudCompare
	{
		namespace generalUtility
		{

			ModulePtr bs_vector_conversions(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::vector_conversion<std::vector<int>>());
				m->add(chaiscript::vector_conversion<std::vector<unsigned>>());
				m->add(chaiscript::vector_conversion<std::vector<std::string>>());
				return m;
			}

			ModulePtr bs_map_conversions(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::map_conversion<std::map<std::string, int>>());
				m->add(chaiscript::map_conversion<std::map<std::string, Boxed_Value>>());
				//m->add(chaiscript::map_conversion< std::map<unsigned long long, unsigned>>());
				return m;
			}

			
			ModulePtr bs_QChar(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::user_type<QChar>(), "QChar");
				m->add(chaiscript::constructor<QChar()>(), "QChar");
				m->add(chaiscript::constructor<QChar(ushort)>(), "QChar");
				m->add(chaiscript::constructor<QChar(uchar, uchar)>(), "QChar");
				m->add(chaiscript::constructor<QChar(short)>(), "QChar");
				m->add(chaiscript::constructor<QChar(uint)>(), "QChar");
				m->add(chaiscript::constructor<QChar(int)>(), "QChar");
				m->add(chaiscript::constructor<QChar(QChar::SpecialCharacter)>(), "QChar");
				m->add(chaiscript::constructor<QChar(QLatin1Char)>(), "QChar");
				m->add(chaiscript::constructor<QChar(char)>(), "QChar");
				m->add(chaiscript::constructor<QChar(uchar)>(), "QChar");
				m->add(fun(static_cast<bool(*)(QChar,QChar)>(&operator==)), "==");
				m->add(fun(static_cast<bool(*)(QChar, QChar)>(&operator<)), "<");
				m->add(fun(static_cast<bool(*)(QChar, QChar)>(&operator!=)), "!=");
				m->add(fun(static_cast<bool(*)(QChar, QChar)>(&operator>=)), ">=");
				m->add(fun(static_cast<bool(*)(QChar, QChar)>(&operator<=)), "<=");
				m->add(fun(static_cast<QDataStream&(*)(QDataStream&, QChar)>(&operator<<)), "<<");
				m->add(fun(static_cast<QDataStream&(*)(QDataStream&, QChar&)>(&operator>>)), ">>");

				chaiscript::utility::add_class<QChar::SpecialCharacter>(*m,
					"SpecialCharacter",
					{
						{ QChar::SpecialCharacter::Null, "Null" },
						{ QChar::SpecialCharacter::Tabulation, "Tabulation" },
						{ QChar::SpecialCharacter::LineFeed, "LineFeed" },
						{ QChar::SpecialCharacter::CarriageReturn, "CarriageReturn" },
						{ QChar::SpecialCharacter::Space, "Space" },
						{ QChar::SpecialCharacter::Nbsp, "Nbsp" },
						{ QChar::SpecialCharacter::SoftHyphen, "SoftHyphen" },
						{ QChar::SpecialCharacter::ReplacementCharacter, "ReplacementCharacter" },
						{ QChar::SpecialCharacter::ObjectReplacementCharacter, "ObjectReplacementCharacter" },
						{ QChar::SpecialCharacter::ByteOrderMark, "ByteOrderMark" },
						{ QChar::SpecialCharacter::ByteOrderSwapped, "ByteOrderSwapped" },
						{ QChar::SpecialCharacter::ParagraphSeparator, "ParagraphSeparator" },
						{ QChar::SpecialCharacter::LineSeparator, "LineSeparator" },
						{ QChar::SpecialCharacter::LastValidCodePoint, "LastValidCodePoint" },
					}
				);
				return m;
			}


			ModulePtr bs_QString(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::user_type<QString>(), "QString");
				m->add(chaiscript::constructor<QString()>(), "QString");
				m->add(chaiscript::constructor<QString(const QChar*)>(), "QString");
				m->add(chaiscript::constructor<QString(QChar)>(), "QString");
				m->add(chaiscript::constructor<QString(int, QChar)>(), "QString");
				m->add(chaiscript::constructor<QString(const QString&)>(), "QString");
				m->add(chaiscript::constructor<QString(QLatin1String)>(), "QString");
				m->add(fun(static_cast<QString&(QString::*)(QChar::SpecialCharacter)>(&QString::operator+=)), "+=");
				m->add(fun(static_cast<QString&(QString::*)(const QString&)>(&QString::operator+=)), "+=");
				m->add(fun(static_cast<QString&(QString::*)(QChar)>(&QString::operator=)), "=");
				m->add(fun(static_cast<QString&(QString::*)(const QString&)>(&QString::operator=)), "=");
				m->add(fun(static_cast<QString&(QString::*)(QLatin1String)>(&QString::operator=)), "=");
				m->add(fun(static_cast<QDataStream & (*)(QDataStream&, const QString&)>(&operator<<)), "<<");
				m->add(fun(static_cast<QDataStream & (*)(QDataStream&, QString&)>(&operator>>)), ">>");
				m->add(fun(static_cast<const QChar(QString::*)(int)const>(&QString::operator[])), "[]");
				m->add(fun(static_cast<const QChar(QString::*)(uint)const>(&QString::operator[])), "[]");
				m->add(fun(static_cast<QString(QString::*)(qlonglong, int, int,QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, qlonglong arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(qulonglong, int, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, qulonglong arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(long, int, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, long arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(ulong, int, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, ulong arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(int, int, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, int arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(uint, int, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, uint arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(short, int, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, short arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(ushort, int, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, ushort arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(double, int, char, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, double arg) {return qstr->arg(arg); }), "arg");
				m->add(fun([](QString* qstr, double arg, char fmt) {return qstr->arg(arg,0,fmt); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(char, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, char arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(QChar, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, QChar arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, int, QChar)const>(&QString::arg)), "arg");
				m->add(fun([](QString* qstr, const QString &arg) {return qstr->arg(arg); }), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, const QString&)const>(&QString::arg)), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, const QString&, const QString&)const>(&QString::arg)), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, const QString&, const QString&, const QString&)const>(&QString::arg)), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, const QString&, const QString&, const QString&, const QString&)const>(&QString::arg)), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, const QString&, const QString&, const QString&, const QString&, const QString&)const>(&QString::arg)), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, const QString&, const QString&, const QString&, const QString&, const QString&, const QString&)const>(&QString::arg)), "arg");
				m->add(fun(static_cast<QString(QString::*)(const QString&, const QString&, const QString&, const QString&, const QString&, const QString&, const QString&, const QString&)const>(&QString::arg)), "arg");
				m->add(chaiscript::type_conversion<std::string, QString>([](const std::string& str) {return QString::fromStdString(str); }));
				m->add(chaiscript::type_conversion<QString, std::string>([](const QString& str) {std::string ret = str.toLocal8Bit().constData(); return ret; }));
				m->add(chaiscript::type_conversion<QStringList, std::vector<std::string>>([](const QStringList& str)
					{
						std::vector<QString> tmp = str.toVector().toStdVector();
						std::vector<std::string> ret;
						for (QString itm : tmp)
						{
							ret.push_back(itm.toLocal8Bit().constData());
						}
						return ret;
					}));
				m->add(chaiscript::type_conversion<std::vector<std::string>, QStringList>([](const std::vector<std::string>& str)
					{
						QStringList ret;
						for (std::string itm : str)
						{
							ret.push_back(QString::fromStdString(itm));
						}
						return ret;
					}));
				m->add(fun([](const std::string& str) {return QString::fromUtf8(str.c_str()); }), "to_QString");
				m->add(fun([](const QString& str) {std::string ret = str.toLocal8Bit().constData(); return ret; }), "to_string");

				
				
				return m;
			}

			template<typename T>
			ModulePtr bs_QList(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using Base = QList<T>;
				m->add(chaiscript::user_type<Base>(), shortCutName);
				m->add(chaiscript::constructor<Base()>(), shortCutName);
				m->add(chaiscript::constructor<Base(const Base&)>(), shortCutName);
				m->add(chaiscript::constructor<Base(std::initializer_list<T>)>(), shortCutName);
				m->add(fun(&Base::operator==), "==");
				m->add(fun(&Base::operator!=), "!=");
				m->add(fun(&Base::size), "size");
				m->add(fun(&Base::isEmpty), "isEmpty");
				m->add(fun(&Base::clear), "clear");
				m->add(fun(&Base::at), "at");
				m->add(fun(static_cast<const T & (Base::*)(int)const>(&Base::operator[])), "[]");
				m->add(fun(static_cast<T & (Base::*)(int)>(&Base::operator[])), "[]");
				m->add(fun(static_cast<Base &(Base::*)(const Base&)>(&Base::operator=)), "=");

				return m;
			}

			ModulePtr bs_QStringList(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::user_type<QStringList>(), "QStringList");
				m->add(chaiscript::constructor<QStringList()>(), "QStringList");
				m->add(chaiscript::constructor<QStringList(const QString&)>(), "QStringList");
				m->add(chaiscript::constructor<QStringList(const QList<QString>&)>(), "QStringList");
				m->add(chaiscript::constructor<QStringList(std::initializer_list<QString>)>(), "QStringList");
				m->add(fun(static_cast<bool(QStringList::*)(const QString&, Qt::CaseSensitivity)const>(&QStringList::contains)), "contains");
				m->add(fun(static_cast<QStringList &(QStringList::*)(const QString&)>(&QStringList::operator<<)), "<<");
				m->add(fun(static_cast<QStringList & (QStringList::*)(const QStringList&)>(&QStringList::operator<<)), "<<");
				m->add(fun(static_cast<QStringList & (QStringList::*)(const QList<QString>&)>(&QStringList::operator<<)), "<<");
				m->add(fun(static_cast<QStringList & (QStringList::*)(const QList<QString>&)>(&QStringList::operator=)), "=");
				bs_QList<QString>("QList_String",m);

				m->add(chaiscript::base_class<QList<QString>, QStringList>());
				return m;
			}

			

			

			ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::extras::math::bootstrap(m);
				chaiscript::extras::string_methods::bootstrap(m);
				bs_vector_conversions(m);
				bs_map_conversions(m);
				bs_QChar(m);
				bs_QString(m);
				bs_QStringList(m);
				
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_GU_CLASSES_HPP