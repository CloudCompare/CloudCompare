#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_IO_CLASSES_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_IO_CLASSES_HPP

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


#include <chaiscript/chaiscript.hpp>
#include <chaiscript/utility/utility.hpp>

#include <FileIOFilter.h>
#include <FileIO.h>


namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_io
			{
				ModulePtr bs_FileIOFilter(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<FileIOFilter>(), "FileIOFilter");
					m->add(user_type<FileIOFilter::LoadParameters>(), "LoadParameters");
					m->add(constructor<FileIOFilter::LoadParameters()>(), "LoadParameters");
					m->add(fun(&FileIOFilter::LoadParameters::shiftHandlingMode), "shiftHandlingMode");
					m->add(fun(&FileIOFilter::LoadParameters::alwaysDisplayLoadDialog), "alwaysDisplayLoadDialog");
					m->add(fun(&FileIOFilter::LoadParameters::coordinatesShiftEnabled), "coordinatesShiftEnabled");
					m->add(fun(&FileIOFilter::LoadParameters::coordinatesShift), "coordinatesShift");
					m->add(fun(&FileIOFilter::LoadParameters::preserveShiftOnSave), "preserveShiftOnSave");
					m->add(fun(&FileIOFilter::LoadParameters::autoComputeNormals), "autoComputeNormals");
					m->add(fun(&FileIOFilter::LoadParameters::parentWidget), "parentWidget");
					m->add(fun(&FileIOFilter::LoadParameters::sessionStart), "sessionStart");
					m->add(user_type<FileIOFilter::SaveParameters>(), "SaveParameters");
					m->add(constructor<FileIOFilter::SaveParameters()>(), "SaveParameters");
					m->add(fun(&FileIOFilter::SaveParameters::alwaysDisplaySaveDialog), "alwaysDisplaySaveDialog");
					m->add(fun(&FileIOFilter::SaveParameters::parentWidget), "parentWidget");
					
					m->add(user_type<FileIOFilter::Shared>(), "Shared");
					m->add(fun(&FileIOFilter::importSupported), "importSupported");
					m->add(fun(&FileIOFilter::exportSupported), "exportSupported");
					m->add(fun(&FileIOFilter::getFileFilters), "getFileFilters");
					m->add(fun(&FileIOFilter::getDefaultExtension), "getDefaultExtension");
					m->add(fun(&FileIOFilter::loadFile), "loadFile");
					m->add(fun(&FileIOFilter::saveToFile), "saveToFile");
					m->add(fun(&FileIOFilter::canSave), "canSave");
					m->add(fun(&FileIOFilter::ImportFilterList), "ImportFilterList");
					m->add(fun(static_cast<ccHObject*(*)(const QString&, FileIOFilter::LoadParameters&, FileIOFilter::Shared, CC_FILE_ERROR&)>(&FileIOFilter::LoadFromFile)), "LoadFromFile");
					m->add(fun(static_cast<ccHObject*(*)(const QString&, FileIOFilter::LoadParameters&, CC_FILE_ERROR&, const QString&)>(&FileIOFilter::LoadFromFile)), "LoadFromFile");
					m->add(fun(static_cast<CC_FILE_ERROR(*)(ccHObject*, const QString&, const FileIOFilter::SaveParameters&, FileIOFilter::Shared)>(&FileIOFilter::SaveToFile)), "SaveToFile");
					m->add(fun(static_cast<CC_FILE_ERROR(*)(ccHObject*, const QString&, const FileIOFilter::SaveParameters&, const QString&)>(&FileIOFilter::SaveToFile)), "SaveToFile");
					m->add(fun(&FileIOFilter::HandleGlobalShift), "HandleGlobalShift");
					m->add(fun(&FileIOFilter::DisplayErrorMessage), "DisplayErrorMessage");
					m->add(fun(&FileIOFilter::CheckForSpecialChars), "CheckForSpecialChars");
					m->add(fun(&FileIOFilter::ResetSesionCounter), "ResetSesionCounter");
					m->add(fun(&FileIOFilter::IncreaseSesionCounter), "IncreaseSesionCounter");
					m->add(fun(&FileIOFilter::InitInternalFilters), "InitInternalFilters");
					m->add(fun(&FileIOFilter::Register), "Register");
					m->add(fun(&FileIOFilter::GetFilter), "GetFilter");
					m->add(fun(&FileIOFilter::FindBestFilterForExtension), "FindBestFilterForExtension");
					m->add(user_type<FileIOFilter::FilterContainer>(), "FilterContainer");
//					m->add(chaiscript::bootstrap::standard_library::vector_type<FileIOFilter::FilterContainer>("FilterContainer"));
					m->add(fun(&FileIOFilter::GetFilters), "GetFilters");
					m->add(fun(&FileIOFilter::UnregisterAll), "UnregisterAll");
					m->add(fun(&FileIOFilter::unregister), "unregister");
					chaiscript::utility::add_class<FileIOFilter::FilterFeature>(*m,
						"FilterFeature",
						{
							{ FileIOFilter::FilterFeature::NoFeatures, "NoFeatures" },
							{ FileIOFilter::FilterFeature::Import, "Import" },
							{ FileIOFilter::FilterFeature::Export, "Export" },
							{ FileIOFilter::FilterFeature::BuiltIn, "BuiltIn" },
							{ FileIOFilter::FilterFeature::DynamicInfo, "DynamicInfo" }
						}
					);

					return m;
				}

				ModulePtr bs_ccGlobalShiftManager(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<ccGlobalShiftManager>(), "ccGlobalShiftManager");
				
					m->add(fun(&ccGlobalShiftManager::Handle), "Handle");
					m->add(fun(static_cast<bool(*)(const CCVector3d&)>(&ccGlobalShiftManager::NeedShift)), "NeedShift");
					m->add(fun(static_cast<bool(*)(double)>(&ccGlobalShiftManager::NeedShift)), "NeedShift");
					m->add(fun(&ccGlobalShiftManager::NeedRescale), "NeedRescale");
					m->add(fun(&ccGlobalShiftManager::BestShift), "BestShift");
					m->add(fun(&ccGlobalShiftManager::BestScale), "BestScale");
					m->add(fun(&ccGlobalShiftManager::MaxCoordinateAbsValue), "MaxCoordinateAbsValue");
					m->add(fun(&ccGlobalShiftManager::SetMaxCoordinateAbsValue), "SetMaxCoordinateAbsValue");
					m->add(fun(&ccGlobalShiftManager::MaxBoundgBoxDiagonal), "MaxBoundgBoxDiagonal");
					m->add(fun(&ccGlobalShiftManager::SetMaxBoundgBoxDiagonal), "SetMaxBoundgBoxDiagonal");
					m->add(fun(&ccGlobalShiftManager::StoreShift), "StoreShift");
					m->add(user_type<ccGlobalShiftManager::ShiftInfo>(), "ShiftInfo");
					m->add(constructor<ccGlobalShiftManager::ShiftInfo(QString)>(), "ShiftInfo");
					m->add(constructor<ccGlobalShiftManager::ShiftInfo(QString, const CCVector3d&, double)>(), "ShiftInfo");
					m->add(fun(&ccGlobalShiftManager::ShiftInfo::shift), "shift");
					m->add(fun(&ccGlobalShiftManager::ShiftInfo::scale), "scale");
					m->add(fun(&ccGlobalShiftManager::ShiftInfo::name), "name");
					m->add(fun(&ccGlobalShiftManager::ShiftInfo::preserve), "preserve");

					m->add(fun(static_cast<bool(*)(ccGlobalShiftManager::ShiftInfo&)>(&ccGlobalShiftManager::GetLast)), "GetLast");
					m->add(fun(static_cast<bool(*)(std::vector<ccGlobalShiftManager::ShiftInfo>&)>(&ccGlobalShiftManager::GetLast)), "GetLast");
					

					chaiscript::utility::add_class<ccGlobalShiftManager::Mode>(*m,
						"ccGlobalShiftManager_Mode",
						{
							{ ccGlobalShiftManager::Mode::NO_DIALOG, "NO_DIALOG" },
							{ ccGlobalShiftManager::Mode::NO_DIALOG_AUTO_SHIFT, "NO_DIALOG_AUTO_SHIFT" },
							{ ccGlobalShiftManager::Mode::DIALOG_IF_NECESSARY, "DIALOG_IF_NECESSARY" },
							{ ccGlobalShiftManager::Mode::ALWAYS_DISPLAY_DIALOG, "ALWAYS_DISPLAY_DIALOG" }
						}
					);

					return m;
				}

				/*ModulePtr bs_ccShiftAndScaleCloudDlg(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<ccShiftAndScaleCloudDlg>(), "ccShiftAndScaleCloudDlg");
					m->add(constructor<ccShiftAndScaleCloudDlg(const CCVector3d&, double, QWidget*)>(), "ccShiftAndScaleCloudDlg");
					m->add(constructor<ccShiftAndScaleCloudDlg(const CCVector3d&, double, const CCVector3d&, double, QWidget*)>(), "ccShiftAndScaleCloudDlg");
					m->add(fun(&ccShiftAndScaleCloudDlg::setShiftFieldsPrecision), "setShiftFieldsPrecision");
					m->add(fun(&ccShiftAndScaleCloudDlg::getShift), "getShift");
					m->add(fun(&ccShiftAndScaleCloudDlg::getScale), "getScale");
					m->add(fun(&ccShiftAndScaleCloudDlg::applyAll), "applyAll");
					m->add(fun(&ccShiftAndScaleCloudDlg::cancelled), "cancelled");
					m->add(fun(&ccShiftAndScaleCloudDlg::showScaleItems), "showScaleItems");
					m->add(fun(&ccShiftAndScaleCloudDlg::showApplyAllButton), "showApplyAllButton");
					m->add(fun(&ccShiftAndScaleCloudDlg::showApplyButton), "showApplyButton");
					m->add(fun(&ccShiftAndScaleCloudDlg::showNoButton), "showNoButton");
					m->add(fun(&ccShiftAndScaleCloudDlg::showCancelButton), "showCancelButton");
					m->add(fun(&ccShiftAndScaleCloudDlg::showWarning), "showWarning");
					m->add(fun(&ccShiftAndScaleCloudDlg::showTitle), "showTitle");
					m->add(fun(&ccShiftAndScaleCloudDlg::showKeepGlobalPosCheckbox), "showKeepGlobalPosCheckbox");
					m->add(fun(&ccShiftAndScaleCloudDlg::keepGlobalPos), "keepGlobalPos");
					m->add(fun(&ccShiftAndScaleCloudDlg::setKeepGlobalPos), "setKeepGlobalPos");
					m->add(fun(&ccShiftAndScaleCloudDlg::showPreserveShiftOnSave), "showPreserveShiftOnSave");
					m->add(fun(&ccShiftAndScaleCloudDlg::preserveShiftOnSave), "preserveShiftOnSave");
					m->add(fun(&ccShiftAndScaleCloudDlg::setPreserveShiftOnSave), "setPreserveShiftOnSave");
					m->add(fun(static_cast<int(ccShiftAndScaleCloudDlg::*)(const ccGlobalShiftManager::ShiftInfo&)>(&ccShiftAndScaleCloudDlg::addShiftInfo)), "addShiftInfo");
					m->add(fun(static_cast<int(ccShiftAndScaleCloudDlg::*)(const std::vector<ccGlobalShiftManager::ShiftInfo>&)>(&ccShiftAndScaleCloudDlg::addShiftInfo)), "addShiftInfo");
					m->add(fun(&ccShiftAndScaleCloudDlg::getInfo), "getInfo");
					m->add(fun(&ccShiftAndScaleCloudDlg::infoCount), "infoCount");
					m->add(fun(&ccShiftAndScaleCloudDlg::setCurrentProfile), "setCurrentProfile");
					m->add(fun(&ccShiftAndScaleCloudDlg::addFileInfo), "addFileInfo");
					m->add(fun(&ccShiftAndScaleCloudDlg::SetLastInfo), "SetLastInfo");

					m->add(chaiscript::base_class<QDialog, ccShiftAndScaleCloudDlg>());
					return m;
				}*/


				ModulePtr bs_FileIO(ModulePtr m = std::make_shared<Module>())
				{
					m->add(fun(&FileIO::setWriterInfo), "setWriterInfo");
					m->add(fun(&FileIO::writerInfo), "writerInfo");
					m->add(fun(&FileIO::applicationName), "applicationName");
					m->add(fun(&FileIO::version), "version");
					m->add(fun(&FileIO::createdBy), "createdBy");
					m->add(fun(&FileIO::createdDateTime), "createdDateTime");
					return m;
				}

				ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
				{
					bs_FileIOFilter(m);
					bs_ccGlobalShiftManager(m);
					//bs_ccShiftAndScaleCloudDlg(m);
					bs_FileIO(m);
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_IO_CLASSES_HPP