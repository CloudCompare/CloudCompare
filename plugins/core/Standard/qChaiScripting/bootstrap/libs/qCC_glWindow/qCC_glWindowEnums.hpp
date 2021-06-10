#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_ENUMS_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_ENUMS_HPP

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

#include <ccGLWindow.h>

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_glWindow
			{

				ModulePtr bs_ccGLWindow_enums(ModulePtr m = std::make_shared<Module>())
				{
					chaiscript::utility::add_class<ccGLWindow::PICKING_MODE>(*m,
						"PICKING_MODE",
						{
							{ ccGLWindow::PICKING_MODE::NO_PICKING, "NO_PICKING" },
							{ ccGLWindow::PICKING_MODE::ENTITY_PICKING, "ENTITY_PICKING" },
							{ ccGLWindow::PICKING_MODE::ENTITY_RECT_PICKING, "ENTITY_RECT_PICKING" },
							{ ccGLWindow::PICKING_MODE::FAST_PICKING, "FAST_PICKING" },
							{ ccGLWindow::PICKING_MODE::POINT_PICKING, "POINT_PICKING" },
							{ ccGLWindow::PICKING_MODE::TRIANGLE_PICKING, "TRIANGLE_PICKING" },
							{ ccGLWindow::PICKING_MODE::POINT_OR_TRIANGLE_PICKING, "POINT_OR_TRIANGLE_PICKING" },
							{ ccGLWindow::PICKING_MODE::LABEL_PICKING, "LABEL_PICKING" },
							{ ccGLWindow::PICKING_MODE::DEFAULT_PICKING, "DEFAULT_PICKING" }
						}
					);

					chaiscript::utility::add_class<ccGLWindow::INTERACTION_FLAG>(*m,
						"INTERACTION_FLAG",
						{
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_NONE, "INTERACT_NONE" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_ROTATE, "INTERACT_ROTATE" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_PAN, "INTERACT_PAN" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_CTRL_PAN, "INTERACT_CTRL_PAN" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_ZOOM_CAMERA, "INTERACT_ZOOM_CAMERA" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_2D_ITEMS, "INTERACT_2D_ITEMS" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_CLICKABLE_ITEMS, "INTERACT_CLICKABLE_ITEMS" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_TRANSFORM_ENTITIES, "INTERACT_TRANSFORM_ENTITIES" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_SIG_RB_CLICKED, "INTERACT_SIG_RB_CLICKED" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_SIG_LB_CLICKED, "INTERACT_SIG_LB_CLICKED" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_SIG_MOUSE_MOVED, "INTERACT_SIG_MOUSE_MOVED" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_SIG_BUTTON_RELEASED, "INTERACT_SIG_BUTTON_RELEASED" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_SIG_MB_CLICKED, "INTERACT_SIG_MB_CLICKED" },
							{ ccGLWindow::INTERACTION_FLAG::INTERACT_SEND_ALL_SIGNALS, "INTERACT_SEND_ALL_SIGNALS" }
						}
					);

					chaiscript::utility::add_class<ccGLWindow::MessagePosition>(*m,
						"MessagePosition",
						{
							{ ccGLWindow::MessagePosition::LOWER_LEFT_MESSAGE, "LOWER_LEFT_MESSAGE" },
							{ ccGLWindow::MessagePosition::UPPER_CENTER_MESSAGE, "UPPER_CENTER_MESSAGE" },
							{ ccGLWindow::MessagePosition::SCREEN_CENTER_MESSAGE, "SCREEN_CENTER_MESSAGE" }
						}
					);


					chaiscript::utility::add_class<ccGLWindow::MessageType>(*m,
						"MessageType",
						{
							{ ccGLWindow::MessageType::CUSTOM_MESSAGE, "CUSTOM_MESSAGE" },
							{ ccGLWindow::MessageType::SCREEN_SIZE_MESSAGE, "SCREEN_SIZE_MESSAGE" },
							{ ccGLWindow::MessageType::PERSPECTIVE_STATE_MESSAGE, "PERSPECTIVE_STATE_MESSAGE" },
							{ ccGLWindow::MessageType::SUN_LIGHT_STATE_MESSAGE, "SUN_LIGHT_STATE_MESSAGE" },
							{ ccGLWindow::MessageType::CUSTOM_LIGHT_STATE_MESSAGE, "CUSTOM_LIGHT_STATE_MESSAGE" },
							{ ccGLWindow::MessageType::MANUAL_TRANSFORMATION_MESSAGE, "MANUAL_TRANSFORMATION_MESSAGE" },
							{ ccGLWindow::MessageType::MANUAL_SEGMENTATION_MESSAGE, "MANUAL_SEGMENTATION_MESSAGE" },
							{ ccGLWindow::MessageType::ROTAION_LOCK_MESSAGE, "ROTAION_LOCK_MESSAGE" },
							{ ccGLWindow::MessageType::FULL_SCREEN_MESSAGE, "FULL_SCREEN_MESSAGE" }
						}
					);


					chaiscript::utility::add_class<ccGLWindow::PivotVisibility>(*m,
						"PivotVisibility",
						{
							{ ccGLWindow::PivotVisibility::PIVOT_HIDE, "PIVOT_HIDE" },
							{ ccGLWindow::PivotVisibility::PIVOT_SHOW_ON_MOVE, "PIVOT_SHOW_ON_MOVE" },
							{ ccGLWindow::PivotVisibility::PIVOT_ALWAYS_SHOW, "PIVOT_ALWAYS_SHOW" }
						}
					);

					return m;
				}

				ModulePtr bs_ccGLUtils_enums(ModulePtr m = std::make_shared<Module>())
				{
					chaiscript::utility::add_class<CC_VIEW_ORIENTATION>(*m,
						"CC_VIEW_ORIENTATION",
						{
							{ CC_VIEW_ORIENTATION::CC_TOP_VIEW, "CC_TOP_VIEW" },
							{ CC_VIEW_ORIENTATION::CC_BOTTOM_VIEW, "CC_BOTTOM_VIEW" },
							{ CC_VIEW_ORIENTATION::CC_FRONT_VIEW, "CC_FRONT_VIEW" },
							{ CC_VIEW_ORIENTATION::CC_BACK_VIEW, "CC_BACK_VIEW" },
							{ CC_VIEW_ORIENTATION::CC_LEFT_VIEW, "CC_LEFT_VIEW" },
							{ CC_VIEW_ORIENTATION::CC_RIGHT_VIEW, "CC_RIGHT_VIEW" },
							{ CC_VIEW_ORIENTATION::CC_ISO_VIEW_1, "CC_ISO_VIEW_1" },
							{ CC_VIEW_ORIENTATION::CC_ISO_VIEW_2, "CC_ISO_VIEW_2" }
						}
					);

					return m;
				}


				ModulePtr bootstrap_enum(ModulePtr m = std::make_shared<Module>())
				{
					bs_ccGLWindow_enums(m);
					bs_ccGLUtils_enums(m);
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_ENUMS_HPP