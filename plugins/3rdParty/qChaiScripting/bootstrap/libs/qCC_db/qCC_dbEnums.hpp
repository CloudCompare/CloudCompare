#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_DB_ENUMS_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_DB_ENUMS_HPP

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

#include <ccObject.h>

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_db
			{

				struct internalCC_CLASS_ENUM
				{
					const int64_t mOBJECT, mHIERARCHY_OBJECT, mPOINT_CLOUD, mMESH, mSUB_MESH, mMESH_GROUP;
					const int64_t mFACET, mPOINT_OCTREE, mPOINT_KDTREE, mPOLY_LINE, mIMAGE, mCALIBRATED_IMAGE;
					const int64_t mSENSOR, mGBL_SENSOR, mCAMERA_SENSOR, mPRIMITIVE, mPLANE, mSPHERE;
					const int64_t mTORUS, mCONE, mOLD_CYLINDER_ID, mCYLINDER, mBOX, mDISH;
					const int64_t mEXTRU, mQUADRIC, mMATERIAL_SET, mARRAY, mNORMALS_ARRAY, mNORMAL_INDEXES_ARRAY;
					const int64_t mRGB_COLOR_ARRAY, mTEX_COORDS_ARRAY, mLABEL_2D, mVIEWPORT_2D_OBJECT;
					const int64_t mVIEWPORT_2D_LABEL, mCLIPPING_BOX, mTRANS_BUFFER; 
					const int64_t mCUSTOM_H_OBJECT, mCUSTOM_LEAF_OBJECT;
					internalCC_CLASS_ENUM()
						: mOBJECT(CC_TYPES::OBJECT), mHIERARCHY_OBJECT(CC_TYPES::HIERARCHY_OBJECT),
						mPOINT_CLOUD(CC_TYPES::POINT_CLOUD), mMESH(CC_TYPES::MESH), mSUB_MESH(CC_TYPES::SUB_MESH),
						mMESH_GROUP(CC_TYPES::MESH_GROUP), mFACET(CC_TYPES::FACET), mPOINT_OCTREE(CC_TYPES::POINT_OCTREE),
						mPOINT_KDTREE(CC_TYPES::POINT_KDTREE), mPOLY_LINE(CC_TYPES::POLY_LINE), mIMAGE(CC_TYPES::IMAGE),
						mCALIBRATED_IMAGE(CC_TYPES::CALIBRATED_IMAGE), mSENSOR(CC_TYPES::SENSOR), mGBL_SENSOR(CC_TYPES::GBL_SENSOR),
						mCAMERA_SENSOR(CC_TYPES::CAMERA_SENSOR), mPRIMITIVE(CC_TYPES::PRIMITIVE), mPLANE(CC_TYPES::PLANE),
						mSPHERE(CC_TYPES::SPHERE), mTORUS(CC_TYPES::TORUS), mCONE(CC_TYPES::CONE),
						mOLD_CYLINDER_ID(CC_TYPES::OLD_CYLINDER_ID), mCYLINDER(CC_TYPES::CYLINDER), mBOX(CC_TYPES::BOX),
						mDISH(CC_TYPES::DISH), mEXTRU(CC_TYPES::EXTRU), mQUADRIC(CC_TYPES::QUADRIC),
						mMATERIAL_SET(CC_TYPES::MATERIAL_SET), mARRAY(CC_TYPES::ARRAY), mNORMALS_ARRAY(CC_TYPES::NORMALS_ARRAY),
						mNORMAL_INDEXES_ARRAY(CC_TYPES::NORMAL_INDEXES_ARRAY), mRGB_COLOR_ARRAY(CC_TYPES::RGB_COLOR_ARRAY),
						mTEX_COORDS_ARRAY(CC_TYPES::TEX_COORDS_ARRAY), mLABEL_2D(CC_TYPES::LABEL_2D),
						mVIEWPORT_2D_OBJECT(CC_TYPES::VIEWPORT_2D_OBJECT), mVIEWPORT_2D_LABEL(CC_TYPES::VIEWPORT_2D_LABEL),
						mCLIPPING_BOX(CC_TYPES::CLIPPING_BOX), mTRANS_BUFFER(CC_TYPES::TRANS_BUFFER),
						mCUSTOM_H_OBJECT(CC_TYPES::CUSTOM_H_OBJECT), mCUSTOM_LEAF_OBJECT(CC_TYPES::CUSTOM_LEAF_OBJECT)
					{ }
				};

				static const internalCC_CLASS_ENUM g_internalCC_CLASS_ENUM;

				ModulePtr bs_CC_CLASS_ENUM(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type< internalCC_CLASS_ENUM >(), "internalCC_CLASS_ENUM");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mOBJECT), "OBJECT");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mHIERARCHY_OBJECT), "HIERARCHY_OBJECT");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mPOINT_CLOUD), "POINT_CLOUD");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mMESH), "MESH");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mSUB_MESH), "SUB_MESH");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mMESH_GROUP), "MESH_GROUP");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mFACET), "FACET");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mPOINT_OCTREE), "POINT_OCTREE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mPOINT_KDTREE), "POINT_KDTREE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mPOLY_LINE), "POLY_LINE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mIMAGE), "IMAGE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mCALIBRATED_IMAGE), "CALIBRATED_IMAGE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mSENSOR), "SENSOR");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mGBL_SENSOR), "GBL_SENSOR");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mCAMERA_SENSOR), "CAMERA_SENSOR");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mPRIMITIVE), "PRIMITIVE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mPLANE), "PLANE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mSPHERE), "SPHERE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mTORUS), "TORUS");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mCONE), "CONE");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mOLD_CYLINDER_ID), "OLD_CYLINDER_ID");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mCYLINDER), "CYLINDER");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mBOX), "BOX");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mDISH), "DISH");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mEXTRU), "EXTRU");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mQUADRIC), "QUADRIC");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mMATERIAL_SET), "MATERIAL_SET");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mARRAY), "ARRAY");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mNORMALS_ARRAY), "NORMALS_ARRAY");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mNORMAL_INDEXES_ARRAY), "NORMAL_INDEXES_ARRAY");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mRGB_COLOR_ARRAY), "RGB_COLOR_ARRAY");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mTEX_COORDS_ARRAY), "TEX_COORDS_ARRAY");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mLABEL_2D), "LABEL_2D");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mVIEWPORT_2D_OBJECT), "VIEWPORT_2D_OBJECT");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mVIEWPORT_2D_LABEL), "VIEWPORT_2D_LABEL");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mCLIPPING_BOX), "CLIPPING_BOX");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mTRANS_BUFFER), "TRANS_BUFFER");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mCUSTOM_H_OBJECT), "CUSTOM_H_OBJECT");
					m->add(chaiscript::fun(&internalCC_CLASS_ENUM::mCUSTOM_LEAF_OBJECT), "CUSTOM_LEAF_OBJECT");
					m->add_global_const(chaiscript::Boxed_Value(&g_internalCC_CLASS_ENUM), "CC_CLASS_ENUM");
					/*chaiscript::utility::add_class<CC_CLASS_ENUM>(*m,
						"CC_CLASS_ENUM",
						{
							{ CC_CLASS_ENUM::OBJECT, "OBJECT" },
							{ CC_CLASS_ENUM::HIERARCHY_OBJECT, "HIERARCHY_OBJECT" },
							{ CC_CLASS_ENUM::POINT_CLOUD, "POINT_CLOUD" },
							{ CC_CLASS_ENUM::MESH, "MESH" },
							{ CC_CLASS_ENUM::SUB_MESH, "SUB_MESH" },
							{ CC_CLASS_ENUM::MESH_GROUP, "MESH_GROUP" },
							{ CC_CLASS_ENUM::FACET, "FACET" },
							{ CC_CLASS_ENUM::POINT_OCTREE, "POINT_OCTREE" },
							{ CC_CLASS_ENUM::POINT_KDTREE, "POINT_KDTREE" },
							{ CC_CLASS_ENUM::POLY_LINE, "POLY_LINE" },
							{ CC_CLASS_ENUM::IMAGE, "IMAGE" },
							{ CC_CLASS_ENUM::CALIBRATED_IMAGE, "CALIBRATED_IMAGE" },
							{ CC_CLASS_ENUM::SENSOR, "SENSOR" },
							{ CC_CLASS_ENUM::GBL_SENSOR, "GBL_SENSOR" },
							{ CC_CLASS_ENUM::CAMERA_SENSOR, "CAMERA_SENSOR" },
							{ CC_CLASS_ENUM::PRIMITIVE, "PRIMITIVE" },
							{ CC_CLASS_ENUM::PLANE, "PLANE" },
							{ CC_CLASS_ENUM::SPHERE, "SPHERE" },
							{ CC_CLASS_ENUM::TORUS, "TORUS" },
							{ CC_CLASS_ENUM::CONE, "CONE" },
							{ CC_CLASS_ENUM::OLD_CYLINDER_ID, "OLD_CYLINDER_ID" },
							{ CC_CLASS_ENUM::CYLINDER, "CYLINDER" },
							{ CC_CLASS_ENUM::BOX, "BOX" },
							{ CC_CLASS_ENUM::DISH, "DISH" },
							{ CC_CLASS_ENUM::EXTRU, "EXTRU" },
							{ CC_CLASS_ENUM::QUADRIC, "QUADRIC" },
							{ CC_CLASS_ENUM::MATERIAL_SET, "MATERIAL_SET" },
							{ CC_CLASS_ENUM::ARRAY, "ARRAY" },
							{ CC_CLASS_ENUM::NORMALS_ARRAY, "NORMALS_ARRAY" },
							{ CC_CLASS_ENUM::NORMAL_INDEXES_ARRAY, "NORMAL_INDEXES_ARRAY" },
							{ CC_CLASS_ENUM::RGB_COLOR_ARRAY, "RGB_COLOR_ARRAY" },
							{ CC_CLASS_ENUM::TEX_COORDS_ARRAY, "TEX_COORDS_ARRAY" },
							{ CC_CLASS_ENUM::LABEL_2D, "LABEL_2D" },
							{ CC_CLASS_ENUM::VIEWPORT_2D_OBJECT, "VIEWPORT_2D_OBJECT" },
							{ CC_CLASS_ENUM::VIEWPORT_2D_LABEL, "VIEWPORT_2D_LABEL" },
							{ CC_CLASS_ENUM::CLIPPING_BOX, "CLIPPING_BOX" },
							{ CC_CLASS_ENUM::TRANS_BUFFER, "TRANS_BUFFER" },
							{ CC_CLASS_ENUM::CUSTOM_H_OBJECT, "CUSTOM_H_OBJECT" },
							{ CC_CLASS_ENUM::CUSTOM_LEAF_OBJECT, "CUSTOM_LEAF_OBJECT" },
						}
					);*/
					return m;
				}


				ModulePtr bootstrap_enum(ModulePtr m = std::make_shared<Module>())
				{
					bs_CC_CLASS_ENUM(m);
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_DB_ENUMS_HPP