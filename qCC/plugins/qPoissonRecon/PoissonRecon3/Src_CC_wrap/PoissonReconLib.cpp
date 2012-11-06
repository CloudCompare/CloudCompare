//##########################################################################
//#                                                                        #
//#               CLOUDCOMPARE WRAPPER: PoissonReconLib                    #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "PoissonReconLib.h"

#define DumpOutput(...)     ((void)0)
#define DumpOutput2(...)	((void)0)

#include "../Src/FunctionData.h"
#include "../Src/SparseMatrix.h"
#include "../Src/Octree.h"
#include "../Src/MultiGridOctreeData.h"

//default reconsutrction degree (DGM: we keep Kazhdan's guess!)
#define DEFAULT_RECON_DEGREE 2

template<int Degree>
class OctreeWrapper : public Octree<Degree>
{
public:

	//overloaded method
	void setTree(int cnt,
				const float* P,
				const float* N,
				const int& maxDepth,
				const int& kernelDepth,
				const Real& samplesPerNode,
				const Real& scaleFactor,
				Point3D<Real>& center,
				Real& scale,
				const int& resetSamples,
				const int& useConfidence)
	{
		Point3D<Real> min,max,boxDim,position,normal;
		int i,j,k;
		const float *_P,*_N;

		TreeNodeData::UseIndex=1;
		this->neighborKey.set(maxDepth);
		int splatDepth=kernelDepth;
		if(splatDepth<0)
			splatDepth=0;

		DumpOutput("Setting bounding box\n");
		// Read through once to get the center and scale
		min.coords[0]=max.coords[0]=P[0];
		min.coords[1]=max.coords[1]=P[1];
		min.coords[2]=max.coords[2]=P[2];
		for (j=1,_P=P+3;j<cnt;++j)
		{
			for (i=0;i<DIMENSION;++i,++_P)
			{
				if(*_P<min.coords[i])
					min.coords[i]=*_P;
				else if (*_P>max.coords[i])
					max.coords[i]=*_P;
			}
		}

		for(i=0;i<DIMENSION;i++)
		{
			boxDim.coords[i]=max.coords[i]-min.coords[i];
			if(!i || scale<boxDim.coords[i])
				scale = boxDim.coords[i];
			center.coords[i]=Real(max.coords[i]+min.coords[i])/2;
		}

		DumpOutput("Samples: %d\n",cnt);
		scale*=scaleFactor;
		for(i=0;i<DIMENSION;i++)
			center.coords[i]-=scale/2;

		if(splatDepth>0)
		{
			DumpOutput("Setting sample weights\n");
			for (k=0,_P=P,_N=N; k<cnt; ++k,_P+=3,_N+=3)
			{
				for(i=0;i<DIMENSION;i++)
				{
					position.coords[i]=(_P[i]-center.coords[i])/scale;
					normal.coords[i]=_N[i];
				}
				Point3D<Real> myCenter;
				myCenter.coords[0]=myCenter.coords[1]=myCenter.coords[2]=Real(0.5);
				Real myWidth=Real(1.0);
				for(i=0;i<DIMENSION;i++)
					if(position.coords[i]<myCenter.coords[i]-myWidth/2 || position.coords[i]>myCenter.coords[i]+myWidth/2)
						break;
				if(i!=DIMENSION)
					continue;
				TreeOctNode* temp=&this->tree;
				int d=0;
				Real weight=Real(1.0);
				if(useConfidence)
					weight=Real(Length(normal));
				while(d<splatDepth)
				{
					NonLinearUpdateWeightContribution(temp,position,weight);
					if(!temp->children)
						temp->initChildren();
					int cIndex=TreeOctNode::CornerIndex(myCenter,position);
					temp=&temp->children[cIndex];
					myWidth/=2;
					if(cIndex&1){myCenter.coords[0]+=myWidth/2;}
					else		{myCenter.coords[0]-=myWidth/2;}
					if(cIndex&2){myCenter.coords[1]+=myWidth/2;}
					else		{myCenter.coords[1]-=myWidth/2;}
					if(cIndex&4){myCenter.coords[2]+=myWidth/2;}
					else		{myCenter.coords[2]-=myWidth/2;}
					d++;
				}
				NonLinearUpdateWeightContribution(temp,position,weight);
			}
		}

		DumpOutput("Adding Points and Normals\n");
		this->normals=new std::vector<Point3D<Real> >();
		for (k=0,_P=P,_N=N; k<cnt; ++k,_P+=3,_N+=3)
		{
			for(i=0;i<DIMENSION;i++)
			{
				position.coords[i]=(_P[i]-center.coords[i])/scale;
				normal.coords[i]=_N[i];
			}
			Point3D<Real> myCenter;
			myCenter.coords[0]=myCenter.coords[1]=myCenter.coords[2]=Real(0.5);
			Real myWidth=Real(1.0);
			for(i=0;i<DIMENSION;i++)
				if(position.coords[i]<myCenter.coords[i]-myWidth/2 || position.coords[i]>myCenter.coords[i]+myWidth/2)
					break;
			if(i!=DIMENSION)
				continue;
			Real l=Real(Length(normal));
			if(l!=l || l<EPSILON)
				continue;
			if(!useConfidence)
			{
				normal.coords[0]/=l;
				normal.coords[1]/=l;
				normal.coords[2]/=l;
			}
			l=Real(2<<maxDepth);
			normal.coords[0]*=l;
			normal.coords[1]*=l;
			normal.coords[2]*=l;

			if(resetSamples && samplesPerNode>0 && splatDepth)
			{
				NonLinearSplatOrientedPoint(position,normal,splatDepth,samplesPerNode,1,maxDepth);
			}
			else
			{
				Real alpha=1;
				TreeOctNode* temp=&this->tree;
				int d=0;
				if(splatDepth)
				{
					while(d<splatDepth)
					{
						int cIndex=TreeOctNode::CornerIndex(myCenter,position);
						temp=&temp->children[cIndex];
						myWidth/=2;
						if(cIndex&1){myCenter.coords[0]+=myWidth/2;}
						else		{myCenter.coords[0]-=myWidth/2;}
						if(cIndex&2){myCenter.coords[1]+=myWidth/2;}
						else		{myCenter.coords[1]-=myWidth/2;}
						if(cIndex&4){myCenter.coords[2]+=myWidth/2;}
						else		{myCenter.coords[2]-=myWidth/2;}
						d++;
					}
					alpha=NonLinearGetSampleWeight(temp,position);
				}
				for(i=0;i<DIMENSION;i++)
					normal.coords[i]*=alpha;
				while(d<maxDepth)
				{
					if(!temp->children)
						temp->initChildren();
					int cIndex=TreeOctNode::CornerIndex(myCenter,position);
					temp=&temp->children[cIndex];
					myWidth/=2;
					if(cIndex&1){myCenter.coords[0]+=myWidth/2;}
					else		{myCenter.coords[0]-=myWidth/2;}
					if(cIndex&2){myCenter.coords[1]+=myWidth/2;}
					else		{myCenter.coords[1]-=myWidth/2;}
					if(cIndex&4){myCenter.coords[2]+=myWidth/2;}
					else		{myCenter.coords[2]-=myWidth/2;}
					d++;
				}
				NonLinearSplatOrientedPoint(temp,position,normal);
			}
		}
		DumpOutput("Memory Usage: %.3f MB\n",float(MemoryUsage()));
	}
};

bool PoissonReconLib::reconstruct(unsigned count,
								  const float* P,
								  const float* N,
								  CoredVectorMeshData& outMesh,
								  int Depth/*=8*/,
								  PoissonReconResultInfo* pInfo/*=0*/)
{
	//hidden parameters
	int /*Depth(8),*/SolverDivide(8),IsoDivide(8),Refine(3);
	float SamplesPerNode(1.0f),Scale(1.1f);

	assert(Depth>1);
	int kernelDepth=Depth-2;

	bool noResetSamples = false;
	bool confidence = false;
	bool noClipTree = false;

	//translation applied to input cloud
	Point3D<float> center;
	center.coords[0]=center.coords[1]=center.coords[2]=0;
	//scaling applied to input cloud
	float scale=1.0;

	//we reste static objects in case of multiple calls!
	TreeNodeData::UseIndex=1;
	TreeOctNode::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);

	OctreeWrapper<DEFAULT_RECON_DEGREE> tree;
	tree.setFunctionData(PPolynomial<DEFAULT_RECON_DEGREE>::GaussianApproximation(),Depth,0,Real(1.0)/Real(1<<Depth));

	assert(kernelDepth<=Depth); //'KernelDepth can't be greater than Depth'

	//tree.setTree(In.value,  Depth.value,   Binary.set, kernelDepth, Real(SamplesPerNode.value), Scale.value, center, scale, !NoResetSamples.set, Confidence.set);
	tree.setTree( count,P,N,  Depth,                     kernelDepth, Real(SamplesPerNode),       Scale,       center, scale, !noResetSamples,     confidence);

	if(!noClipTree)
		tree.ClipTree();

	tree.finalize1(Refine);

	tree.SetLaplacianWeights();

	tree.finalize2(Refine);

	tree.LaplacianMatrixIteration(SolverDivide);

	Real isoValue=tree.GetIsoValue();

	if(IsoDivide>0)
		tree.GetMCIsoTriangles(isoValue,IsoDivide,&outMesh);
	else
		tree.GetMCIsoTriangles(isoValue,&outMesh);

	if (pInfo)
	{
		pInfo->scale = scale;
		pInfo->center[0] = center.coords[0];
		pInfo->center[1] = center.coords[1];
		pInfo->center[2] = center.coords[2];
	}

	return true;
}
