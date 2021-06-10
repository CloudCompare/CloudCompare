#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_ENUMS_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_ENUMS_HPP

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

#include <cmath>
#include <memory>

#include <chaiscript/chaiscript.hpp>
#include <chaiscript/utility/utility.hpp>
//#include "../../../../../qCC/ccSubsamplingDlg.h"
//#include "../../../../../qCC/ccScalarFieldArithmeticsDlg.h"
//#include "../../../../../qCC/ccLibAlgorithms.h"
//#include "../../../../../qCC/ccHistogramWindow.h"
//#include "../../../../../qCC/ccFilterByValueDlg.h"
#include "../../../../../qCC/ccEntityAction.h"
//#include "../../../../../qCC/ccContourExtractor.h"
//#include "../../../../../qCC/ccComputeOctreeDlg.h"
//#include "../../../../../qCC/ccComparisonDlg.h"
//#include "../../../../../qCC/ccColorGradientDlg.h"
//#include "../../../../../qCC/ccAlignDlg.h"


namespace chaiscript
{
	/*namespace utility
	{
		template<typename EnumClass, typename ModuleType>
		typename std::enable_if<std::is_enum<EnumClass>::value, void>::type
			add_class(ModuleType& t_module,
				const std::string& t_class_name,
				const std::vector<std::pair<EnumClass, std::string>>& t_constants
			)
		{
			t_module.add(chaiscript::user_type<EnumClass>(), t_class_name);

			t_module.add(chaiscript::constructor<EnumClass()>(), t_class_name);
			t_module.add(chaiscript::constructor<EnumClass(const EnumClass&)>(), t_class_name);

			using namespace chaiscript::bootstrap::operators;
			equal<EnumClass>(t_module);
			not_equal<EnumClass>(t_module);
			assign<EnumClass>(t_module);

			for (const auto& constant : t_constants)
			{
				t_module.add_global_const(chaiscript::const_var(EnumClass(constant.first)), constant.second);
			}
		}
	}*/

	namespace cloudCompare
	{
		namespace qCC
		{



			template<typename Ret, typename Param>
			ModulePtr cos(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::fun([](Param p) { return std::cos(p); }), "cos");
				return m;
			}

			ModulePtr ConsoleMessageLevel(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccMainAppInterface::ConsoleMessageLevel>(*m,
					"ConsoleMessageLevel",
					{
						{ ccMainAppInterface::ConsoleMessageLevel::STD_CONSOLE_MESSAGE, "STD_CONSOLE_MESSAGE" },
						{ ccMainAppInterface::ConsoleMessageLevel::WRN_CONSOLE_MESSAGE, "WRN_CONSOLE_MESSAGE" },
						{ ccMainAppInterface::ConsoleMessageLevel::ERR_CONSOLE_MESSAGE, "ERR_CONSOLE_MESSAGE" }
					}
				);
				return m;
			}

			/*ModulePtr CC_SUBSAMPLING_METHOD(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccSubsamplingDlg::CC_SUBSAMPLING_METHOD>(*m,
					"ConsoleMessageLevel",
					{
						{ ccSubsamplingDlg::CC_SUBSAMPLING_METHOD::RANDOM, "RANDOM" },
						{ ccSubsamplingDlg::CC_SUBSAMPLING_METHOD::SPACE, "SPACE" },
						{ ccSubsamplingDlg::CC_SUBSAMPLING_METHOD::OCTREE, "OCTREE" }
					}
				);
				return m;
			}*/



			/*ModulePtr ccScalarFieldArithmeticsDlg(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccScalarFieldArithmeticsDlg::Operation>(*m,
					"Operation",
					{
						{ ccScalarFieldArithmeticsDlg::Operation::PLUS, "PLUS" },
						{ ccScalarFieldArithmeticsDlg::Operation::MINUS, "MINUS" },
						{ ccScalarFieldArithmeticsDlg::Operation::MULTIPLY, "MULTIPLY" },
						{ ccScalarFieldArithmeticsDlg::Operation::DIVIDE, "DIVIDE" },
						{ ccScalarFieldArithmeticsDlg::Operation::SQRT, "SQRT" },
						{ ccScalarFieldArithmeticsDlg::Operation::POW2, "POW2" },
						{ ccScalarFieldArithmeticsDlg::Operation::POW3, "POW3" },
						{ ccScalarFieldArithmeticsDlg::Operation::EXP, "EXP" },
						{ ccScalarFieldArithmeticsDlg::Operation::LOG10, "LOG" },
						{ ccScalarFieldArithmeticsDlg::Operation::COS, "COS" },
						{ ccScalarFieldArithmeticsDlg::Operation::SIN, "SIN" },
						{ ccScalarFieldArithmeticsDlg::Operation::TAN, "TAN" },
						{ ccScalarFieldArithmeticsDlg::Operation::ACOS, "ACOS" },
						{ ccScalarFieldArithmeticsDlg::Operation::ASIN, "ASIN" },
						{ ccScalarFieldArithmeticsDlg::Operation::ATAN, "ATAN" },
						{ ccScalarFieldArithmeticsDlg::Operation::INT, "INT" },
						{ ccScalarFieldArithmeticsDlg::Operation::INVERSE, "INVERSE" },
						{ ccScalarFieldArithmeticsDlg::Operation::INVALID, "INVALID" }
					}
				);
				return m;
			}*/


			/*ModulePtr CC_LIB_ALGORITHM(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccLibAlgorithms::CC_LIB_ALGORITHM>(*m,
					"CC_LIB_ALGORITHM",
					{
						{ ccLibAlgorithms::CC_LIB_ALGORITHM::CCLIB_ALGO_SF_GRADIENT, "CCLIB_ALGO_SF_GRADIENT" }
					}
				);

				chaiscript::utility::add_class<ccLibAlgorithms::ScaleMatchingAlgorithm>(*m,
					"ScaleMatchingAlgorithm",
					{
						{ ccLibAlgorithms::ScaleMatchingAlgorithm::BB_MAX_DIM, "BB_MAX_DIM" },
						{ ccLibAlgorithms::ScaleMatchingAlgorithm::BB_VOLUME, "BB_VOLUME" },
						{ ccLibAlgorithms::ScaleMatchingAlgorithm::PCA_MAX_DIM, "PCA_MAX_DIM" },
						{ ccLibAlgorithms::ScaleMatchingAlgorithm::ICP_SCALE, "ICP_SCALE" }
					}
				);
				return m;
			}*/


			/*ModulePtr ccHistogramWindow(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccHistogramWindow::HISTOGRAM_COLOR_SCHEME>(*m,
					"HISTOGRAM_COLOR_SCHEME",
					{
						{ ccHistogramWindow::HISTOGRAM_COLOR_SCHEME::USE_SOLID_COLOR, "USE_SOLID_COLOR" },
						{ ccHistogramWindow::HISTOGRAM_COLOR_SCHEME::USE_CUSTOM_COLOR_SCALE, "USE_CUSTOM_COLOR_SCALE" },
						{ ccHistogramWindow::HISTOGRAM_COLOR_SCHEME::USE_SF_SCALE, "USE_SF_SCALE" }
					}
				);
				return m;
			}*/


			/*ModulePtr ccFilterByValueDlg(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccFilterByValueDlg::Mode>(*m,
					"Mode",
					{
						{ ccFilterByValueDlg::Mode::EXPORT, "EXPORT" },
						{ ccFilterByValueDlg::Mode::SPLIT, "SPLIT" },
						{ ccFilterByValueDlg::Mode::CANCEL, "CANCEL" }
					}
				);
				return m;
			}*/

			/*ModulePtr ccEntityAction(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccEntityAction::NORMAL_CONVERSION_DEST>(*m,
					"NORMAL_CONVERSION_DEST",
					{
						{ ccEntityAction::NORMAL_CONVERSION_DEST::HSV_COLORS, "HSV_COLORS" },
						{ ccEntityAction::NORMAL_CONVERSION_DEST::DIP_DIR_SFS, "DIP_DIR_SFS" }
					}
				);

				chaiscript::utility::add_class<ccEntityAction::CLEAR_PROPERTY>(*m,
					"CLEAR_PROPERTY",
					{
						{ ccEntityAction::CLEAR_PROPERTY::COLORS, "COLORS" },
						{ ccEntityAction::CLEAR_PROPERTY::NORMALS, "NORMALS" },
						{ ccEntityAction::CLEAR_PROPERTY::CURRENT_SCALAR_FIELD, "CURRENT_SCALAR_FIELD" },
						{ ccEntityAction::CLEAR_PROPERTY::ALL_SCALAR_FIELDS, "ALL_SCALAR_FIELDS" }
					}
				);

				chaiscript::utility::add_class<ccEntityAction::TOGGLE_PROPERTY>(*m,
					"TOGGLE_PROPERTY",
					{
						{ ccEntityAction::TOGGLE_PROPERTY::ACTIVE, "ACTIVE" },
						{ ccEntityAction::TOGGLE_PROPERTY::VISIBLE, "VISIBLE" },
						{ ccEntityAction::TOGGLE_PROPERTY::COLOR, "COLOR" },
						{ ccEntityAction::TOGGLE_PROPERTY::NORMALS, "NORMALS" },
						{ ccEntityAction::TOGGLE_PROPERTY::SCALAR_FIELD, "SCALAR_FIELD" },
						{ ccEntityAction::TOGGLE_PROPERTY::MATERIAL, "MATERIAL" },
						{ ccEntityAction::TOGGLE_PROPERTY::NAME, "NAME" }
					}
				);
				return m;
			}*/

			/*ModulePtr ccContourExtractor(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccContourExtractor::ContourType>(*m,
					"ContourType",
					{
						{ ccContourExtractor::ContourType::LOWER, "LOWER" },
						{ ccContourExtractor::ContourType::UPPER, "UPPER" },
						{ ccContourExtractor::ContourType::FULL, "FULL" }
					}
				);
				return m;
			}*/

			/*ModulePtr ccComputeOctreeDlg(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccComputeOctreeDlg::ComputationMode>(*m,
					"ComputationMode",
					{
						{ ccComputeOctreeDlg::ComputationMode::DEFAULT, "DEFAULT" },
						{ ccComputeOctreeDlg::ComputationMode::MIN_CELL_SIZE, "MIN_CELL_SIZE" },
						{ ccComputeOctreeDlg::ComputationMode::CUSTOM_BBOX, "CUSTOM_BBOX" }
					}
				);
				return m;
			}*/

			/*ModulePtr ccComparisonDlg(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccComparisonDlg::CC_COMPARISON_TYPE>(*m,
					"CC_COMPARISON_TYPE",
					{
						{ ccComparisonDlg::CC_COMPARISON_TYPE::CLOUDCLOUD_DIST, "CLOUDCLOUD_DIST" },
						{ ccComparisonDlg::CC_COMPARISON_TYPE::CLOUDMESH_DIST, "CLOUDMESH_DIST" }
					}
				);
				return m;
			}*/


			/*ModulePtr ccColorGradientDlg(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccColorGradientDlg::GradientType>(*m,
					"GradientType",
					{
						{ ccColorGradientDlg::GradientType::Default, "Default" },
						{ ccColorGradientDlg::GradientType::TwoColors, "TwoColors" },
						{ ccColorGradientDlg::GradientType::Banding, "Banding" }
					}
				);
				return m;
			}*/

			/*ModulePtr ccAlignDlg(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<ccAlignDlg::CC_SAMPLING_METHOD>(*m,
					"CC_SAMPLING_METHOD",
					{
						{ ccAlignDlg::CC_SAMPLING_METHOD::NONE, "NONE" },
						{ ccAlignDlg::CC_SAMPLING_METHOD::RANDOM, "RANDOM" },
						{ ccAlignDlg::CC_SAMPLING_METHOD::SPACE, "SPACE" },
						{ ccAlignDlg::CC_SAMPLING_METHOD::OCTREE, "OCTREE" }
					}
				);
				return m;
			}*/


			ModulePtr bootstrap_enum(ModulePtr m = std::make_shared<Module>())
			{
				cos<double, double>(m);
				ConsoleMessageLevel(m);
				//CC_SUBSAMPLING_METHOD(m);
				//ccScalarFieldArithmeticsDlg(m);
				//CC_LIB_ALGORITHM(m);
				//ccHistogramWindow(m);
				//ccFilterByValueDlg(m);
				//ccEntityAction(m);
				//ccContourExtractor(m);
				//ccComputeOctreeDlg(m);
				//ccComparisonDlg(m);
				//ccColorGradientDlg(m);
				//ccAlignDlg(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_ENUMS_HPP